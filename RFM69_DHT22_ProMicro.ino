
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

#include <RFM69registers.h>

#include "DHT.h"

//#include <Narcoleptic.h>
#include "LowPower.h"

//typedef char int8_t;
//typedef short int16_t;

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID      2    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID   100  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID   1
#define FREQUENCY   RF69_868MHZ
//#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************

/* PIN definitions */
#define LED_PIN          9

#define DHT22_DATA_PIN 8
#define DHT22_VCC_PIN  6
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Atmega32u4
//if defined(__AVR_ATmega32U4__)
#if 0
  #define RFM69_RESET_PIN 18
  #define INT_PIN 7
  #define INT_NUM 4

#else
  // Atmega328P
  #define RFM69_RESET_PIN 14 //18
  #define INT_PIN 7
  #define INT_NUM 23
#endif

#define SERIAL_BAUD   9600

#define RX_WAIT_MSEC 100
#define TX_RETRY 2
#define TX_POWER_LEVEL_MAX 20

/* dynamic config parameters */
bool powerDownDHT = true;
bool rfm69Sleep = true;
int8_t txPowerLevel = 2;
unsigned long txPeriodMsec = 6 * 60 * 1000;
unsigned int noChangeCntMax = 10; //-> 50*txPeriod
bool requestACK = true;
bool enableNarcoleptic = true;

/*******************************************/
/* module classes */
/*******************************************/
#ifdef ENABLE_ATC
  RFM69_ATC radio(10, 4 /*DIO0/INT*/);
#else
  RFM69 radio(10, INT_PIN /*DIO0/INT*/, true /*isRFM69HW*/, INT_NUM /*IRQ Num*/);
#endif

DHT dht(DHT22_DATA_PIN, DHTTYPE);
/*******************************************/

uint8_t txCnt = 0;

#define TYPE_CNT_VCC_TEMP_HUMI 1

typedef struct {
  byte type;
  byte cnt;
  int vcc;  // Supply voltage
  int temp;  // Temperature reading
  int humi;
} tPayload;

#define TYPE_TXPOWERCHANGE 2
typedef struct {
  byte type;
  int8_t txPowerChange;
} tAckData;

tPayload tinytx;
tPayload lastTx;
bool lastAck = false;
unsigned int noChangeCnt = 0;


/* DHT helper functions */
void enableDHT(bool startup)
{
  if(startup || powerDownDHT)
  {
    pinMode(DHT22_VCC_PIN, OUTPUT);
    digitalWrite(DHT22_VCC_PIN, HIGH);
    digitalWrite(DHT22_DATA_PIN, OUTPUT);
    delay(1000);
  }
}

void disableDHT(bool startup)
{
  if(startup || powerDownDHT)
  {
    //digitalWrite(DHT22_VCC_PIN, LOW);
    pinMode(DHT22_VCC_PIN, INPUT);
    digitalWrite(DHT22_DATA_PIN, INPUT);
  }
}

void printDHT()
{
   // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
}


/*******************************************************/
void setup() 
{
  if(false)
  {
    noInterrupts();
    CLKPR = 0x80;
    CLKPR = 0x03; //div by 8
    interrupts();
  }
  delay(10);
  
  enableDHT(true);
  
  pinMode(RFM69_RESET_PIN, OUTPUT);
  digitalWrite(RFM69_RESET_PIN,HIGH);
  delay(20);
  digitalWrite(RFM69_RESET_PIN,LOW);
  delay(20);

#if 0
  int i;
  for(i = 0; i < 10; i++)
  {
    if (Serial)
    {
      enableNarcoleptic = false;
      txPeriodMsec = 5000;
      noChangeCntMax = 5;
      break;
    }

    delay(500);
   }
#endif

    txPeriodMsec = 10000;
    noChangeCntMax = 2;

  dht.begin();
  
  Serial.println("setup radio");  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);

  radio.setPowerLevel(txPowerLevel);

  delay(5000); //to allow programming
  
  Serial.println("done");
  
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
#ifdef ENCRYPTKEY
  radio.encrypt(ENCRYPTKEY);
#endif
  //radio.setFrequency(919000000); //set frequency to some custom frequency

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
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

void Blink(byte PIN, int DELAY_MS)
{
  return;
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void myDelay(int milliseconds) {
  while (milliseconds >= 8000) { LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); milliseconds -= 8000; }
  if (milliseconds >= 4000)    { LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF); milliseconds -= 4000; }
  if (milliseconds >= 2000)    { LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); milliseconds -= 2000; }
  if (milliseconds >= 1000)    { LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF); milliseconds -= 1000; }
  if (milliseconds >= 500)     { LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF); milliseconds -= 500; }
  if (milliseconds >= 250)     { LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF); milliseconds -= 250; }
  if (milliseconds >= 125)     { LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF); milliseconds -= 120; }
  if (milliseconds >= 64)      { LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF); milliseconds -= 60; }
  if (milliseconds >= 32)      { LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF); milliseconds -= 30; }
  if (milliseconds >= 16)      { LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF); milliseconds -= 15; }
}


bool transmitWithRetry()
{
  bool gotAck = false;
  uint8_t retry;
  for(retry = 0; retry < TX_RETRY; retry++)
  {
      if(Serial)
      {
          Serial.print("transmit (retry: ");
          Serial.print(retry);
          Serial.print(")\n");
      }
        
      radio.send(GATEWAYID, &tinytx, sizeof(tinytx), requestACK);
      Blink(LED_PIN,3);
  
      if(Serial)
      {
        Serial.println("wait ACK");
      }
      long ackWait = millis();
      while(millis() - ackWait < RX_WAIT_MSEC)
      {
        if(Serial)
        {
          Serial.print("rssi: ");
          Serial.print(radio.readRSSI());
          Serial.print("\n");
        }
  
        if(radio.ACKReceived(GATEWAYID))
        {
          lastTx = tinytx;
          gotAck = true;
          if(Serial)
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
                txPowerLevel += pAck->txPowerChange;
            }
            Serial.print("\n");

            if(txPowerLevel > TX_POWER_LEVEL_MAX)
              txPowerLevel = TX_POWER_LEVEL_MAX;
            if(txPowerLevel < 0)
                txPowerLevel = 0;
      
            radio.setPowerLevel(txPowerLevel);

          }
  
          break;
       }
        
        delay(100);
      }
      if(gotAck)
        break;
      else
      {
        txPowerLevel += 10;
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
  disableDHT(false);
  radio.sleep();
  while(1)
  {
    myDelay(txPeriodMsec);
  }
}

/*******************************************************/
void loop() 
{
    enableDHT(false);

  
    bool txNow = false;
    float h = dht.readHumidity();
    
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    disableDHT(false);
    
    tinytx.type = TYPE_CNT_VCC_TEMP_HUMI;
    tinytx.temp= t*100;
    tinytx.humi= h*100;
    tinytx.vcc = readVcc(); // Get supply voltage

    if(tinytx.vcc < 3000) //low bat
    {
      shutdown();
    }

    bool forceTx = false;
    if(noChangeCnt >= noChangeCntMax)
    {
      forceTx = true;
    }

    if(forceTx || lastAck == false || 
       abs(tinytx.temp - lastTx.temp) >  20 ||
       abs(tinytx.humi - lastTx.humi) > 200 || 
       abs(tinytx.vcc - lastTx.vcc) > 50 )
    {
      tinytx.cnt = txCnt++;
      txNow = true;
    }

    if(Serial)
    {
        Serial.print("type: ");
        Serial.print(tinytx.type);
        
        Serial.print(", cnt: ");
        Serial.print(tinytx.cnt);
        
        Serial.print(", vcc: ");
        Serial.print(tinytx.vcc);

        Serial.print(", temp: ");
        Serial.print(tinytx.temp);
        
        Serial.print(", humi: ");
        Serial.print(tinytx.humi);
                
        Serial.print("\n");
    }

    if(txNow)
    {
        noChangeCnt = 0;
        bool gotAck = transmitWithRetry();
        
        lastAck = gotAck;

        Serial.print("txPowerLevel: ");
        Serial.print(txPowerLevel);
        Serial.print("\n");

        if(Serial)
        {
            Serial.println("sleep");
        }

        if(rfm69Sleep)
            radio.sleep();
        else
            radio.standby();
    }
    else
    {
      if(Serial)
      {
        Serial.print("no change: ");
        Serial.print(noChangeCnt);
        Serial.print("\n");
      }

      noChangeCnt++;
    }

    if(enableNarcoleptic)
      myDelay(txPeriodMsec);
    else
      delay(txPeriodMsec);
    
    radio.receiveDone();
  
}
