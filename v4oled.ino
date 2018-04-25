/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
             (c) 2017 Tom Vijlbrief

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with static payload,
   using frequency and encryption settings matching those of
   the (early prototype version of) The Things Network.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
    0.1% in g2).

   ToDo:
   - set NWKSKEY (value from staging.thethingsnetwork.com)
   - set APPKSKEY (value from staging.thethingsnetwork.com)
   - set DEVADDR (value from staging.thethingsnetwork.com)
   - optionally comment #define DEBUG
   - optionally comment #define SLEEP

 *******************************************************************************/
#define MESSAGE_DELAY 600000 //gap between messages in milli seconds
int prevMillis=0;
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "itoa.h"
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
char temp[25];int lines=0;


#include "BMP085.h"
Adafruit_BMP085 bmp;
// show debug statements; comment next line to disable debug statements
#define DEBUG

// Enable OTA?
//#define OTA

// use low power sleep: 0.5mA
//#define SLEEP
//void mdelay(int n, bool mode = false);
//#ifdef SLEEP
// or DeepSleep: 0.05mA, but RAM is lost and reboots on wakeup.
// We safe some data in the RTC backup ram which survives DeepSleep
#define DEEP_SLEEP  false



#define led       LED_BUILTIN
#define voltage   PA0

#define USE_SPI   1

#ifndef OTA
// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { };

// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { };

// LoRaWAN end-device address (DevAddr), ie 0x91B375AC  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x ; // <-- Change this address for every node!
#else

static const u1_t APPEUI[8] = { }; // reversed 8 bytes of AppEUI registered with ttnctl

static const unsigned char APPKEY[16] ={ }; // non-reversed 16 bytes of the APPKEY used when registering a device with ttnctl register DevEUI AppKey
#endif

// STM32 Unique Chip IDs
#define STM32_ID  ((u1_t *) 0x1FFFF7E8)

//SPIClass mySPI(USE_SPI);

//extern SPIClass *SPIp;

// Blink a led
#define BLINK

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

#define RATE        DR_SF7

struct {
  unsigned short temp;
  unsigned short pres;
  byte alt;
  byte volt;
} mydata;
byte payload[10];

#ifdef BLINK
void blinkN(int n, int d = 250, int t = 800)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, 0);
//    mdelay(5);
delay(5);
    digitalWrite(LED_BUILTIN, 1);
//    mdelay(d);
    delay(d);
  }
  pinMode(LED_BUILTIN, INPUT);
 // mdelay(t);
  delay(t);
}
#endif
#ifndef OTA
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#else
void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
#if 0
  // Use human friendly format:
  u1_t* p = STM32_ID;
  buf[0] = (p[0] & 0x7) + 1;
  buf[1] = (p[1] & 0x7) + 1;
  buf[2] = (p[2] & 0x7) + 1;
  buf[3] = (p[3] & 0x7) + 1;
  buf[4] = (p[4] & 0x7) + 1;
  buf[5] = (p[5] & 0x7) + 1;
  buf[6] = (p[6] & 0x7) + 1;
  buf[7] = (p[7] & 0x7) + 1;
#endif
}

static const u1_t DEVEUI[8]={ 0x43, 0x35, 0x42, 0x32, 0x24, 0x45, 0x23, 0x25 }; // reversed 8 bytes of DevEUI registered with ttnctl
void os_getDevEui (u1_t* buf) {
// use chip ID:
//  memcpy(buf, &STM32_ID[1], 8);
memcpy(buf,DEVEUI,8);
  // Make locally registered:
//  buf[0] = buf[0] & ~0x3 | 0x1;
}
#endif

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
#if USE_SPI == 1
  //.nss = PA4,
  .nss = PA4,
  .rxtx = LMIC_UNUSED_PIN,
  //.rst = PB0,
  .rst = PB12,
  .dio = {PB15, PB14, PB13}
#else // USE_SPI == 2
  .nss = PB12,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PA8,
  .dio = {PB1, PB10, PB11}
#endif
};


bool TX_done = false;

bool joined = false;

void onEvent (ev_t ev) {
Serial.print(os_getTime());
Serial.print(": ");

#ifdef DEBUG
msgOLED(F("Enter onEvent"));
#endif

  switch (ev) {
    case EV_SCAN_TIMEOUT:
    msgOLED(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
    msgOLED(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
    msgOLED(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
    msgOLED(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
    msgOLED(F("EV_JOINING"));
      break;
    case EV_JOINED:
    msgOLED(F("EV_JOINED"));
      joined = true;            LMIC_setLinkCheckMode(0);

      break;
    case EV_RFU1:
    msgOLED(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
    msgOLED(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
    msgOLED(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      TX_done = true;
    msgOLED(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
      msgOLED(F("Data Received: "));
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      msgOLED("");
        mydata.volt = (LMIC.frame + LMIC.dataBeg)[0];
        if (LMIC.dataLen > 1) {
          switch ((LMIC.frame + LMIC.dataBeg)[1]) {
            case 7: LMIC_setDrTxpow(DR_SF7, 14); break;
            case 8: LMIC_setDrTxpow(DR_SF8, 14); break;
            case 9: LMIC_setDrTxpow(DR_SF9, 14); break;
            case 10: LMIC_setDrTxpow(DR_SF10, 14); break;
            case 11: LMIC_setDrTxpow(DR_SF11, 14); break;
            case 12: LMIC_setDrTxpow(DR_SF7, 14); break;
          }
        }
      }
      // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

      break;
    case EV_LOST_TSYNC:
    msgOLED(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
    msgOLED(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
    msgOLED(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
    msgOLED(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
    msgOLED(F("EV_LINK_ALIVE"));
      break;
    default:
    msgOLED(F("Unknown event"));
      break;
  }
#ifdef DEBUG
msgOLED(F("Leave onEvent"));
#endif
#ifdef SLEEP
  next = true; // Always send after any event, to recover from a dead link
#endif
}

void do_send(osjob_t* j) {

#ifdef DEBUG
msgOLED(F("Enter do_send"));
#endif

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
  msgOLED(F("OP_TXRXPEND, not sending"));
  }else{ 
 readData();
//  LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
int temperature = (payload[0]<<8)|payload[1];
int pressure = (payload[2]<<8)|payload[3];
int altitude = (payload[4]<<8)|payload[5];
int volt= (payload[6]<<8)|payload[7];
int chipTemp=(payload[8]<<8)|payload[9];

  Serial.println("Setting payload data");
  String s="Temperature=";s+=temperature;s+=" Pressure=";s+=pressure;s+="Altitude=";s+=altitude;s+=" Voltage:";s+=volt;s+=" Chip Temperature:";s+=chipTemp;
  Serial.println(s);
  LMIC_setTxData2(1, payload, sizeof(payload), 0);
  Serial.println(sizeof(payload)-1);
  msgOLED(F("Packet queued"));
  // Next TX is scheduled after TX_COMPLETE event.
  }
#ifdef DEBUG
msgOLED(F("Leave do_send"));
#endif
  TX_done = false;

}

void blinkTemp(int n, int d = 500, int t = 800)
{
  const int tempBlinkPin = LED_BUILTIN;

  pinMode(tempBlinkPin, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(tempBlinkPin, 0);
//    mdelay(5);
    delay(5);
    digitalWrite(tempBlinkPin, 1);
//    mdelay(d);
    delay(d);

  }
  pinMode(tempBlinkPin, INPUT);
//  mdelay(t);
  delay(5);
}

#define tempPin       PA0
#define powerNTCPin   PA2

void readData()
{
  msgOLED(F("Inside readData()"));
 float Vdd = readVdd(),tempChip=readTempSensor(Vdd);
  Serial.print("Vdd=  ");
  Serial.print(Vdd);
  Serial.print(" V Temp= ");
  Serial.print(readTempSensor(Vdd));
  Serial.println(" °C");
 
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();

  double steinhart = bmp.readTemperature();
  steinhart = 4095 / steinhart - 1;
  steinhart = 10000 * steinhart;
  steinhart = steinhart / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 4050;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  double Temp = steinhart;
/*
  Vdd += 5;
  if (Vdd < 2000 || Vdd >= 3000)*/
  blinkN(Vdd / 1000);
  blinkN((int)(Vdd*1000) % 1000 / 100);
  blinkN((int)(Vdd*1000) % 100 / 10);

  mydata.temp = bmp.readTemperature()*100;
  blinkTemp(int(Temp) / 10);
  blinkTemp(int(Temp) % 10);



  //mydata.power = (vref / 10) - 200;
  mydata.alt=bmp.readAltitude();
  mydata.pres = bmp.readPressure()/100;
  String dataString="T:";
  dataString+=bmp.readTemperature();
  dataString+="C P:";
  dataString+=bmp.readPressure();
  dataString+="Bar Alt:";
  dataString+=bmp.readAltitude();
  dataString+=" Vdd:";
  dataString+=Vdd;
  dataString+="V Tc:";
  dataString+=tempChip;
  msgOLED(dataString);  
  payload[0]=highByte(mydata.temp);
  payload[1]=lowByte(mydata.temp);
  payload[2]=highByte(mydata.pres);
  payload[3]=lowByte(mydata.pres);
  payload[4]=highByte(mydata.alt);  
  payload[5]=lowByte(mydata.alt); 
  payload[6]=highByte((int)(Vdd*1000.0));  
  payload[7]=lowByte((int)(Vdd*1000.0)); 
  int chipTemperature=tempChip*100;
  payload[8]=highByte(chipTemperature);  
  payload[9]=lowByte(chipTemperature); 

}
/*
void allInput()
{
  //adc_disable(ADC1);
  //adc_disable(ADC2);

  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  pinMode(PA2, INPUT);
  pinMode(PA3, INPUT);
  pinMode(PA4, INPUT);
  pinMode(PA5, INPUT);
  pinMode(PA6, INPUT);
  pinMode(PA7, INPUT);
  pinMode(PA8, INPUT);
  pinMode(PA9, INPUT);
  pinMode(PA10, INPUT);
  
  pinMode(PA11, INPUT);
  pinMode(PA12, INPUT);
  pinMode(PA13, INPUT);
  pinMode(PA14, INPUT);
  pinMode(PA15, INPUT);

  pinMode(PB0, INPUT);
  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(PB3, INPUT);
  pinMode(PB4, INPUT);
  pinMode(PB5, INPUT);
  pinMode(PB6, INPUT);
  pinMode(PB7, INPUT);
  pinMode(PB8, INPUT);
  pinMode(PB9, INPUT);
  pinMode(PB10, INPUT);
  pinMode(PB11, INPUT);
  pinMode(PB12, INPUT);
  pinMode(PB13, INPUT);
  pinMode(PB14, INPUT);
  pinMode(PB15, INPUT);
}
*/
void setup() {
  analogRead(A0); // Workaround to init g_current_pin used in HAL_ADC_MspInit

 if (!bmp.begin()) 
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
//  allInput();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.setRotation(270);  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  
  display.display();
//  delay(2000);
  display.clearDisplay();
 
    msgOLED("   STM32 Lora Node");
  // Show the display buffer on the hardware.
  // NOTE: You _must_ call display after making any drawing commands
  // to make them visible on the display hardware!
  display.display();



//  SPIp = &mySPI;
  SPI.begin();

  pinMode(led, OUTPUT);
#ifdef DEBUG
  digitalWrite(led, LOW);
  delay(20);
  digitalWrite(led, HIGH);
#endif

Serial.begin(115200);

#if 0
  // Show ID in human friendly format (digits 1..8)
  u1_t* p = STM32_ID;
  blinkN((p[0] & 0x7) + 1);
  blinkN((p[1] & 0x7) + 1);
  blinkN((p[2] & 0x7) + 1);
  blinkN((p[3] & 0x7) + 1);
  blinkN((p[4] & 0x7) + 1);
  blinkN((p[5] & 0x7) + 1);
  blinkN((p[6] & 0x7) + 1);
  blinkN((p[7] & 0x7) + 1);
#endif
  msgOLED("Starting os subsystem..");
  // LMIC init
  os_init();
    msgOLED("Device reset..");
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#ifndef OTA
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    msgOLED("Setting up device..");
#endif

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 433175000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 433175000, DR_RANGE_MAP(DR_SF7,  DR_SF7),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.


#ifndef OTA
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF7;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(RATE, 14);
#endif


  // Start job
  msgOLED("Starting job..");
  do_send(&sendjob);

#ifdef DEBUG
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.setRotation(270);  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
//  display.display();
//  delay(2000);


msgOLED(F("Leave setup"));
#endif
}


void loop() {
  enum { LED_ON=0, LED_OFF=1 };    int millivolts;
    uint32_t t0 = millis();
 // for Blue Pill .. active low led
    digitalWrite(led,LED_ON);
    delay(50);
    digitalWrite(led,LED_OFF);


#ifdef BLINK
  static int count;
  digitalWrite(led,
               ! ((++count < 1000) || !TX_done)
              );
#endif
 os_runloop_once();

#ifdef BLINK
    digitalWrite(led, HIGH);
    delay(5);
    digitalWrite(led, LOW);
#endif





    if(millis()-prevMillis>MESSAGE_DELAY)
    {
//      LMIC_shutdown();
//      os_init();
//      LMIC_reset();
      prevMillis=millis();
      msgOLED("Sending TTN data");
      readData();
      do_send(&sendjob);
    }
    displayTPA();
}
void msgOLED(String s)
{
  Serial.println(s);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.setRotation(270);  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if(lines++>7){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    lines=0;
  }
  display.println(s);
  display.display();
}
void msgOLED(int i)
{
  char s[25];itoa(i,s,10);
  Serial.println(s);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.setRotation(270);  
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if(lines++>7){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    lines=0;
  }
  display.println(s);
  display.display();
}
void displayTPA(){
  display.setCursor(0,0);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  mydata.pres = bmp.readPressure();
  String dataString="T:";
  dataString+=bmp.readTemperature();
  dataString+="C \nP:";
  dataString+=bmp.readPressure()/100;
  dataString+="Bar \nAlt:";
  dataString+=bmp.readAltitude();
  display.println(dataString);
  display.print("Next:");
  long timeleft=(MESSAGE_DELAY-(millis()-prevMillis))/1000;
  display.print(timeleft);
  display.print(" sec");  
  display.display();
  delay(1000);
  display.setTextSize(1);
  display.display();

}

uint16_t adc_read(uint32_t channel)
{
  ADC_HandleTypeDef AdcHandle = {};
  ADC_ChannelConfTypeDef  AdcChannelConf = {};
  __IO uint16_t uhADCxConvertedValue = 0;

  AdcHandle.Instance = ADC1;
  AdcHandle.State = HAL_ADC_STATE_RESET;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Specifies the number of ranks that will be converted within the regular group sequencer. */
  AdcHandle.Init.NbrOfDiscConversion   = 0;                             /* Parameter discarded because sequencer is disabled */

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
    return 0;
  }

  AdcChannelConf.Channel      = channel;             /* Specifies the channel to configure into ADC */
  AdcChannelConf.Rank         = ADC_REGULAR_RANK_1;               /* Specifies the rank in the regular group sequencer */
  AdcChannelConf.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;                     /* Sampling time value to be set for the selected channel */

  /*##-2- Configure ADC regular channel ######################################*/
  if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK)
  {
    /* Channel Configuration Error */
    return 0;
  }
  /*##-2.1- Calibrate ADC then Start the conversion process ####################*/
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) !=  HAL_OK) {
    /* ADC Calibration Error */
    return 0;
  }
  /*##-3- Start the conversion process ####################*/
  if (HAL_ADC_Start(&AdcHandle) != HAL_OK) {
    /* Start Conversation Error */
    return 0;
  }
  /*##-4- Wait for the end of conversion #####################################*/
  /*  For simplicity reasons, this example is just waiting till the end of the
      conversion, but application may perform other tasks while conversion
      operation is ongoing. */
  if (HAL_ADC_PollForConversion(&AdcHandle, 10) != HAL_OK) {
    /* End Of Conversion flag not set on time */
    return 0;
  }
  /* Check if the continous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
    /*##-5- Get the converted value of regular channel  ########################*/
    uhADCxConvertedValue = HAL_ADC_GetValue(&AdcHandle);
  }

  if (HAL_ADC_Stop(&AdcHandle) != HAL_OK) {
    /* Stop Conversation Error */
    return 0;
  }
  if(HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
    return 0;
  }
  return uhADCxConvertedValue;
}
static float readVdd()
{
  return (1.20 * 4096.0 / adc_read(ADC_CHANNEL_VREFINT)); // ADC sample to V
  //return (1200 * 4096 / adc_read(ADC_CHANNEL_VREFINT)); // ADC sample to mV
}

static float readTempSensor(float Vdd)
{
  return ((1.43 - (Vdd / 4096.0 * adc_read(ADC_CHANNEL_TEMPSENSOR))) / 0.0043 + 25.0);
}
