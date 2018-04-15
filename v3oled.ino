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

//#if DEEP_SLEEP
//#undef OTA
//#endif
//#endif

#define led       LED_BUILTIN
#define voltage   PA0

#define USE_SPI   1

#ifndef OTA
// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static unsigned char NWKSKEY[16] = { 0x7C, 0xCF, 0x4F, 0x28, 0x91, 0xFA, 0x19, 0x4B, 0x06, 0xBF, 0x8B, 0x4A, 0xDF, 0x0B, 0x5F, 0x4C };

// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static unsigned char APPSKEY[16] = { 0x49, 0x8F, 0x17, 0x75, 0xE4, 0x31, 0x97, 0x18, 0x23, 0x39, 0xA5, 0x92, 0x17, 0x2E, 0xB2, 0xF2 };

// LoRaWAN end-device address (DevAddr), ie 0x91B375AC  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x26011836 ; // <-- Change this address for every node!
#else

static const u1_t APPEUI[8] = { 0xDC, 0xB6, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // reversed 8 bytes of AppEUI registered with ttnctl

static const unsigned char APPKEY[16] ={ 0xD9, 0xF7, 0x1F, 0x20, 0x74, 0x99, 0x56, 0x77, 0x78, 0x0B, 0xEB, 0x3F, 0x78, 0x46, 0x0A, 0x9E }; // non-reversed 16 bytes of the APPKEY used when registering a device with ttnctl register DevEUI AppKey
#endif

// STM32 Unique Chip IDs
#define STM32_ID  ((u1_t *) 0x1FFFF7E8)

//SPIClass mySPI(USE_SPI);

//extern SPIClass *SPIp;

// Blink a led
#define BLINK

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
#ifdef SLEEP
int txInterval = (DEEP_SLEEP ? 300 : 60); // Note that the LED flashing takes some time
#else
int txInterval = 60;
#endif

#define RATE        DR_SF7

struct {
  unsigned short temp;
  unsigned short pres;
  byte power;
  byte rate2;
} mydata;

#ifdef SLEEP

// Defined for power and sleep functions pwr.h and scb.h
//#include <libmaple/pwr.h>
//#include <libmaple/scb.h>

//#include <RTClock.h>

//RTClock rt(RTCSEL_LSI, 399); // 10 milli second alarm

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   ((uint32_t *)(0x40006C00 +0x04))

void storeBR(int i, uint32_t v) {
  BKP_REG_BASE[2 * i] = (v << 16);
  BKP_REG_BASE[2 * i + 1] = (v & 0xFFFF);
}

uint32_t readBR(int i) {
  return ((BKP_REG_BASE[2 * i] & 0xFFFF) >> 16) | (BKP_REG_BASE[2 * i + 1] & 0xFFFF);
}

bool next = false;

void sleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;

  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  // Enable wakeup pin bit.
  PWR_BASE->CR |=  PWR_CSR_EWUP;

  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag) {
    // Set Power down deepsleep bit.
    PWR_BASE->CR |= PWR_CR_PDDS;
    // Unset Low-power deepsleep.
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
//    adc_disable(ADC1);
//    adc_disable(ADC2);
#if STM32_HAVE_DAC
    dac_disable_channel(DAC, 1);
    dac_disable_channel(DAC, 2);
#endif
    //  Unset Power down deepsleep bit.
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    // set Low-power deepsleep.
    PWR_BASE->CR |= PWR_CR_LPDS;
  }

  // Now go into stop mode, wake up on interrupt
  asm("    wfi");

  // Clear SLEEPDEEP bit so we can use SLEEP mode
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

uint32 sleepTime;

void AlarmFunction () {
  // We always wake up with the 8Mhz HSI clock!
  // So adjust the clock if needed...

#if F_CPU == 8000000UL
  // nothing to do, using about 8 mA
#elif F_CPU == 16000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);
#elif F_CPU == 48000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
#elif F_CPU == 72000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
#else
//#error "Unknown F_CPU!?"
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);

#endif

  extern volatile uint32 systick_uptime_millis;
  systick_uptime_millis += sleepTime;
}

void mdelay(int n, bool mode = false)
{
  sleepTime = n;
  time_t nextAlarm = (rt.getTime() + n / 10); // Calculate from time now.
  rt.createAlarm(&AlarmFunction, nextAlarm);
  sleepMode(mode);
}

void msleep(uint32_t ms)
{
  uint32_t start = rt.getTime();

  while (rt.getTime() - start < ms) {
    asm("    wfi");
  }
}
#endif

void blinkN(int n, int d = 400, int t = 800)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, 0);
//    mdelay(5);
delay(5);
    digitalWrite(LED_BUILTIN, 1);
//    mdelay(d);
    delay(5);
  }
  pinMode(LED_BUILTIN, INPUT);
 // mdelay(t);
  delay(5);
}


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
  .dio = {PA15, PA14, PA13}
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
      joined = true;
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
        mydata.rate2 = (LMIC.frame + LMIC.dataBeg)[0];
        txInterval = (1 << mydata.rate2);
        if (LMIC.dataLen > 1) {
          switch ((LMIC.frame + LMIC.dataBeg)[1]) {
            case 7: LMIC_setDrTxpow(DR_SF7, 14); break;
            case 8: LMIC_setDrTxpow(DR_SF8, 14); break;
            case 9: LMIC_setDrTxpow(DR_SF9, 14); break;
            case 10: LMIC_setDrTxpow(DR_SF10, 14); break;
            case 11: LMIC_setDrTxpow(DR_SF11, 14); break;
            case 12: LMIC_setDrTxpow(DR_SF12, 14); break;
          }
        }
      }
      // Schedule next transmission
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(txInterval), do_send);
#endif

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
  } else {
    readData();
#ifdef SLEEP
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
#endif
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
  msgOLED(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
msgOLED(F("Leave do_send"));
#endif
  TX_done = false;

}

void blinkTemp(int n, int d = 500, int t = 800)
{
  const int tempBlinkPin = PB7;

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
//  adc_enable(ADC1);

//  adc_reg_map *regs = ADC1->regs;
//  regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
//  regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
    int vref=3.3;
//  int vref = 1200 * 4096 / adc_read(ADC1, 17); // ADC sample to millivolts
//  regs->CR2 &= ~ADC_CR2_TSVREFE; // disable VREFINT and temp sensor

  pinMode(powerNTCPin, OUTPUT);
  digitalWrite(powerNTCPin, 1);

  int v = analogRead(tempPin);
  pinMode(powerNTCPin, INPUT);
  //adc_disable(ADC1);

  double steinhart = v;
  steinhart = 4095 / steinhart - 1;
  steinhart = 10000 * steinhart;
  steinhart = steinhart / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 4050;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  double Temp = steinhart;

  vref += 5;
  if (vref < 2000 || vref >= 3000)
    blinkN(vref / 1000);
  blinkN(vref % 1000 / 100);
  blinkN(vref % 100 / 10);

  mydata.temp = Temp * 10;
   mydata.temp = 50;
  Temp += 0.5; // round
  blinkTemp(int(Temp) / 10);
  blinkTemp(int(Temp) % 10);

#ifdef DEBUG
msgOLED(v);
#endif

  //mydata.power = (vref / 10) - 200;
  mydata.power=55;
  mydata.pres = 1111;
}

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

void setup() {
//  allInput();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  display.setRotation(270);  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
//  delay(2000);

 
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
  LMIC_setupChannel(0, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 433175000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 433175000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

#if F_CPU == 8000000UL
  // HSI is less accurate
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
#endif

#ifndef OTA
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(RATE, 14);
#endif

#ifdef SLEEP
  if (DEEP_SLEEP)
    LMIC.seqnoUp = readBR(0);

#if defined(OTA) && DEEP_SLEEP
#error "DEEP_SLEEP and OTA cannot be combined!"
#endif
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

  // Clear the buffer.
//  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
//  display.setCursor(0,0);
  
msgOLED(F("Leave setup"));
#endif
}


void loop() {

#ifndef SLEEP

#ifdef BLINK
  static int count;
  digitalWrite(led,
               ! ((++count < 1000) || !TX_done)
              );
#endif
  os_runloop();

#else

#ifdef OTA
  if (!joined) {
    os_runloop_once();
    return;
  }
#endif

  if (next == false) {
    digitalWrite(led, LOW);
    //if (DEEP_SLEEP)
    LMIC.skipRX = 1; // Do NOT wait for downstream data!
    os_runloop();

  } else {

#ifdef BLINK
    digitalWrite(led, HIGH);
#endif

#ifdef DEBUG
  msgOLED(LMIC.seqnoUp);
#endif

    if (DEEP_SLEEP)
      storeBR(0, LMIC.seqnoUp);

//    SPIp->end();


    digitalWrite(PA5, LOW); // SCK
    pinMode(PA5, OUTPUT);

    digitalWrite(PA7, LOW); // MOSI
    pinMode(PA7, OUTPUT);

    pinMode(PA6, INPUT); // MISO

    digitalWrite(lmic_pins.nss, LOW); // NSS
    pinMode(lmic_pins.nss, OUTPUT);

    // DIO Inputs
    pinMode(PA11, INPUT);
    pinMode(PA12, INPUT);
    pinMode(PA15, INPUT);

    pinMode(lmic_pins.rst, INPUT);

    // Serial
    pinMode(PA9, INPUT);
    pinMode(PA10, INPUT);

    mdelay(txInterval * 1000, DEEP_SLEEP);
    delay(txInterval * 1000, DEEP_SLEEP);

    begin(115200);

    extern void hal_io_init();
    digitalWrite(lmic_pins.rst, 1); // prevent reset
    hal_io_init();

    SPI.begin();

#ifdef DEBUG
  msgOLED(F("Sleep complete"));
#endif
    next = false;
    // Start job
    do_send(&sendjob);
  }
#endif
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

