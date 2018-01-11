#include <RFM69registers.h>
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <MK.h> //my lib
#include <avr/boot.h>

/**********************************************************************************************/
#define NODEID      2    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID   100  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID   1
#define FREQUENCY   RF69_868MHZ
//#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
/**********************************************************************************************/
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
/******************************************************************************/
/* Sensor setup */
#define MYSENSOR SENSOR_DS18B20
//#define USE_RADIO_TEMP
#include <MkSensor.h>
/******************************************************************************/
#define SERIAL_BAUD   9600 //38400 //115200

#define RX_WAIT_MSEC 50
#define TX_RETRY 2
#define TX_POWER_LEVEL_MAX 20

/* dynamic config parameters */
int8_t txPowerLevel = 0;

#define LOOP_PERIOD_SEC (5*60)
#define NO_CHANGE_CNT_MAX 6

unsigned int txPeriodSec = LOOP_PERIOD_SEC;
unsigned int noChangeCntMax = NO_CHANGE_CNT_MAX; 
bool startupTx = true;
bool serialDebug = false;

/**********************************************************************************************/
/* PIN definitions */
/**********************************************************************************************/
#define LED_PIN          9

// Atmega32u4
#if defined(__AVR_ATmega32U4__)
  #define RFM69_RESET_PIN 18
  #define INT_PIN 7
  #define INT_NUM 4

#elif defined(__AVR_ATmega328P__)
  // Atmega328P
  #define RFM69_RESET_PIN 14 
  #define INT_PIN 3
  #define INT_NUM 1
#else
  #error "need pin definitions"
#endif
/**********************************************************************************************/

/******************************************************************/
/* module classes */
/******************************************************************/
RFM69 radio(10, INT_PIN /*DIO0/INT*/, true /*isRFM69HW*/, INT_NUM /*IRQ Num*/);
MkSensor mkSensor(MYSENSOR);
/******************************************************************/
uint8_t txCnt = 0;

tPayload tinytx;
tPayload lastTx;
bool lastAck = false;
unsigned int noChangeCnt = 0;

/*******************************************************/
void setup() 
{
  initClock();

  uint8_t extFuse = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
  interrupts();
  delay(10);

  pinMode(LED_PIN, OUTPUT);
  
  pinMode(RFM69_RESET_PIN, OUTPUT);
  digitalWrite(RFM69_RESET_PIN,HIGH);
  delay(20);
  digitalWrite(RFM69_RESET_PIN,LOW);
  delay(20);

  if(startupTx)
  {
    txPeriodSec = 5;
    noChangeCntMax = 1;
  }
    Serial.begin(SERIAL_BAUD);

    Serial.print("init sensor\n");
    mkSensor.init();

    Serial.print("extFuse: ");
    Serial.println(extFuse, HEX);

    Serial.println("setup radio");  
    radio.initialize(FREQUENCY,NODEID,NETWORKID);

    radio.setPowerLevel(txPowerLevel);

    delay(4000); //to allow programming

    Serial.println("done");
  
#ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
#ifdef ENCRYPTKEY
    radio.encrypt(ENCRYPTKEY);
#endif
    //radio.setFrequency(919000000); //set frequency to some custom frequency

#ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
#endif

    char buff[50];
    sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(buff);

#ifdef ENABLE_ATC
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif
}

bool transmitWithRetry(uint8_t nTxRetry)
{
  bool gotAck = false;
  bool requestACK;
  uint8_t retry;
  
  if(nTxRetry > 0)
    requestACK = true;
  else
    requestACK = false;
      
  for(retry = 0; retry <= nTxRetry; retry++)
  {
      if(serialDebug)
      {
          Serial.print("transmit (retry: ");
          Serial.print(retry);
          Serial.print(")\n");
      }
        
      radio.send(GATEWAYID, &tinytx, sizeof(tinytx), requestACK);

      if(nTxRetry == 0)
      {
        lastTx = tinytx;
        return true;
      }
  
      if(serialDebug)
      {
        Serial.println("wait ACK");
      }
      long ackWait = millis();
      while(millis() - ackWait < RX_WAIT_MSEC)
      {
        if(serialDebug)
        {
          Serial.print("rssi: ");
          Serial.print(radio.readRSSI());
          Serial.print("\n");
        }
  
        if(radio.ACKReceived(GATEWAYID))
        {
          lastTx = tinytx;
          gotAck = true;

          tAckData *pAck = (tAckData*) &radio.DATA[0];
          if(pAck->type == TYPE_TXPOWERCHANGE)
          {
              txPowerLevel += pAck->txPowerChange;
          }

          if(txPowerLevel > TX_POWER_LEVEL_MAX)
            txPowerLevel = TX_POWER_LEVEL_MAX;
          if(txPowerLevel < 0)
              txPowerLevel = 0;
    
          radio.setPowerLevel(txPowerLevel);

          if(serialDebug)
          {
            Serial.print("ACK received");
            
            Serial.print(", datalen: ");
            Serial.print(radio.DATALEN);
  
            Serial.print(", rssi: ");
            Serial.print(radio.RSSI);
  
            Serial.print(", ack data: ");
            tAckData *pAck = (tAckData*) &radio.DATA[0];
            if(pAck->type == TYPE_TXPOWERCHANGE)
            {
                Serial.print(", txPowerChange: ");
                Serial.print(pAck->txPowerChange);
            }
            Serial.print("\n");
          }

          Serial.print(", txPowerLevel: ");
          Serial.print(txPowerLevel);

          break;
       }
        
        delay(5);
      }
      if(gotAck)
        break;
      else
      {
        txPowerLevel += 6;
      }
      if(txPowerLevel > TX_POWER_LEVEL_MAX)
          txPowerLevel = TX_POWER_LEVEL_MAX;
          
      if(txPowerLevel < 0)
          txPowerLevel = 0;

      radio.setPowerLevel(txPowerLevel);
  }
  return gotAck;
}

void shutdown()
{
  mkSensor.shutdown();
  radio.sleep();
  while(1)
  {
    sleepSec(60);
  }
}

void setLed()
{
  digitalWrite(LED_PIN, HIGH);
}

void clearLed()
{
   digitalWrite(LED_PIN, LOW);
}

/*******************************************************/
void loop() 
{
    bool forceTx = false;  
  
    if(startupTx && (txCnt > 2))
    {
        txPeriodSec = LOOP_PERIOD_SEC;
        noChangeCntMax = NO_CHANGE_CNT_MAX; 
        startupTx = false;
        forceTx = true;
    }
  
    setLed();

    tinytx.type = TYPE_CNT_VCC_TEMP_HUMI_PRES;
    
#ifdef USE_RADIO_TEMP
    float t = radio.readTemperature();
    tinytx.temp= t*100;
    radio.sleep();
#endif

    mkSensor.read(tinytx);
    tinytx.vcc = readVcc(); // Get supply voltage
    tinytx.txLev = txPowerLevel;
    
    clearLed();
    
    if(tinytx.vcc < 2200) //low bat
    {
      shutdown();
    }
  
    if(noChangeCnt >= noChangeCntMax)
    {
      forceTx = true;
    }

    bool tempChange = abs(tinytx.temp - lastTx.temp) >  20;
    bool humiChange = abs(tinytx.humi - lastTx.humi) > 200;
    bool vccChange = abs(tinytx.vcc - lastTx.vcc) > 50;
    bool txNow = false;
    
    if(forceTx || lastAck == false || 
       tempChange || humiChange || vccChange)
    {
        tinytx.cnt = txCnt++;
        txNow = true;
    }

    if(serialDebug)
    {
        mkSensor.print(tinytx);
        
        Serial.print(", noChangeCnt: ");
        Serial.print(noChangeCnt);
#if 0
        if(tempChange)
          Serial.print(", tempChange ");
        
        if(humiChange)
          Serial.print(", humiChange ");

        if(vccChange)
          Serial.print(", vccChange ");
#endif  
        if(forceTx)
          Serial.print(", forceTx ");
          
        if(lastAck == false)
          Serial.print(", noAck ");
                
        Serial.print("\n");

    }

    if(txNow)
    {
        setLed();
        
        noChangeCnt = 0;
        uint8_t retry = 0;
        if(startupTx)
          retry = 1;
        bool gotAck = transmitWithRetry(retry);
        lastAck = gotAck;

        if(serialDebug)
        {
            Serial.print("txPowerLevel: ");
            Serial.print(txPowerLevel);
            Serial.print("\n");

            Serial.println("sleep");
            delay(20);
        }
        clearLed();
        
        delay(10);
        
        radio.sleep();
    }
    else
    {
      if(serialDebug)
      {
        Serial.print("no change: ");
        Serial.print(noChangeCnt);
        Serial.print("\n");
        delay(20);
      }

      noChangeCnt++;
    }

    if(serialDebug)
      delay(50);
      
    sleepSec(txPeriodSec);
}
