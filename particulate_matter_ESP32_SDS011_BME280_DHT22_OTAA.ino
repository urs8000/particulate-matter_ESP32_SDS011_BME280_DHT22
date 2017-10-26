/******************************************************************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 * History: 
 * 2017-02-14 rxf
 *   anstelle von Dallas-One-Wire nun eine DHT22 auslesen
 *
 * 2017-01-29 rxf
 *   adopted, to use SDS011 Particulate Matter Sensor
 *   Sends data every minute to LoRaWan
 * Jan 2016, Modified by Maarten to run on ESP8266. Running on Wemos D1-mini
 * 
 * 2017-08-xx : changed for ESP32 (DOIT module)
 * 2017-10-16 : added BME280 measuring also pressure  (using Adafruit libraries)
 *------------------------------------------------------------------------------
 * code (esp-rfm-LoRa.ino) modified by ursm
 * addapted to ESP32 with hardware seriel
 * Sensors used DHT22 , SDS011
 * LoRa used    RFM95
 * Libraries
 * DHT:     https://github.com/adafruit/DHT-sensor-library
 * BME280:  Adafruit_Sensor & Adafruit_BME280   https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout?view=all 
 * LoRa:    LMIC 1.5.0 from Matthijs Kooijman   https://github.com/matthijskooijman/arduino-lmic  (commit of 20170627)
 * 
 * remarks
 * RFM9x:  NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK (not used in this code)
 * DHT22:  multiple read recommended, if failed set both values to zero
 * SDS011: 30seconds warm-up (ventilation) time (changed from 10)
 * keys for ABP have been separated
 * // Key - Structure  for ABP
 * // application "feinstaub_your_city" Application EUI    700123456789ABCD
 * //                                   Device EUI         0011223344556677
 * // DeviceID    "feinstaub_1234"    
 * static const u4_t DEVADDR = 0x01234567; 
 * static const u1_t PROGMEM NWKSKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
 * static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
 * 
 * ESP:
 * ESP includes added as described https://github.com/espressif/arduino-esp32 --> select platform in the description
 * compiling this code with tool -> board: ESP32 Dev.Module, 80MHz, upload 512000 (use a short cable), debug: none
 * 
 * compiled with debugging ON
 * Sketch uses 127286 bytes (9%) of program storage space. Maximum is 1310720 bytes.
 * Global variables use 10508 bytes (3%) of dynamic memory, leaving 284404 bytes for local variables. Maximum is 294912 bytes.
 * 
 * compiled with debugging OFF 
 * Sketch uses 124942 bytes (9%) of program storage space. Maximum is 1310720 bytes.
 * Global variables use 10508 bytes (3%) of dynamic memory, leaving 284404 bytes for local variables. Maximum is 294912 bytes.
 *
 * ESP32 (DOIT)  ---->  HOPR-RFM95 modul
 *  D5    ->    nSS
 *  D18   ->    SCK
 *  D19   ->    MISO
 *  D23   ->    MOSI
 *  D13   ->    RST
     
 *        ---->  BMP280   (+3V3, GND)
 *  D21   ->    SDA
 *  D22   ->    SCL
 *  
 *            ---->  DHT22    (+3V3, GND)
 *  D27   ->    Data    
 *      
 *            ---->  SDS011   (+3V3, GND)
 *      Rx2   ->    Tx
 *      Tx2   ->    Rx    
 * 
 * 
 ******************************************************************************************************************************/
 // 
 // actual message structure, using DHT22 and BME280   (14 bytes , 3 bytes spare)    51msec airtime SF7
 // Sensor-ID:  4 bytes
 // PM10:       2 bytes    multiplied by 10
 // PM2.5:      2 bytes    multiplied by 10
 // Temp DHT:   2 bytes    multiplied by 100       !! will be overwritten by values from BME280 if available
 // Humid DHT:  2 bytes    multiplied by 100       !! message lengst and structure could be extended if desired
 // Temp BME:   2 bytes    multiplied by 100
 // Humid BME:  2 bytes    multiplied by 100
 // Press BME:  2 bytes    no nachkommastellen
 // 
 // on non-recoverable error DHT values or BME values will be 0
 // errors on SDS seems to be a problem of weak connections at the sensor  (jumper cable, no original connectors)
 //
 // known issues:
 // the fan starts to blow while on TX_interval
 // with debugging on, the USB disconnects sometimes
 //
 // to do:
 // when using only one t/h-sensor, shorten message length
 // Cayenne-type Sensor Definitions     (cayenne misses pollution sensors
 // error handling for SDS011
 // sending software-version number on initial message (AA , [sensor-id] , [softwere version] , 00 00 00 ...
 //
 // power consumption, 5V from power source:   (measured with an USB tester)
 // on intervall delay: 50mA
 // while fan blows   : 110 - 130mA
 //
 // ----------------------------------------------------------------------------------------------------------------------------

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#define my_DEBUG                         // uncomment for debug messages on Serial Monitor  (sporadically disconnect from USB)

// ------------------ usage for ABP --------------------------------------------------
// #include <Keys_prod_feinstaub_feinstaub_0001_RFM95.h>
/* ****** Keys read from separate file  keyfile.h  ,  structure explained above ******
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
   static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
   static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
// LoRaWAN end-device address (DevAddr)
   static const u4_t DEVADDR = 0x01234567 ; // <-- Change this address for every node!
*/
// ------------------ usage for OTAA -------------------------------------------------
#include <Keys_prod_feinstaub_1001_RFM95_otaa.h>
/*
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
*/

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
// void os_getArtEui (u1_t* buf) { }
// void os_getDevEui (u1_t* buf) { }
// void os_getDevKey (u1_t* buf) { }


uint8_t mydata[18];                        // should be addapted to the needed length
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 600;          // 600 is the minimal delay used between measurements

// SCK, MISO, MOSI connected to their corresponding pins (9, 10, 11)  (see espressif doculentation)
/* for ESP32
static const uint8_t SS    = 5;
static const uint8_t MOSI  = 23;
static const uint8_t MISO  = 19;
static const uint8_t SCK   = 18;
*/
// Pin mapping , could be modified (do NOT use pin 3
const lmic_pinmap lmic_pins = {
    .nss = 5,                              // 
    .rxtx = LMIC_UNUSED_PIN,               // 
    .rst = 13,                             // 
    .dio = {12, 14, LMIC_UNUSED_PIN},      // 12, 14, 27
};

//---------------------------------------------------------
// Sensor declarations
//---------------------------------------------------------
#define SDS011    1                        // use SDS011
#define DHT22     1                        // use DHT22
#define BME280    1                        // use BME280


#if BME280
// added BME280 from example bme280test.  remark: code for SPI connection has been removed
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;                       // I2C

long  cnt_BMEfail  = 0;
long  cnt_BMEok    = 0;
bool  BME_failed   = false;
int   BME_try      = 0;
float bme_temp     = 0.0;
float bme_hum      = 0.0;
float bme_press    = 0.0;
int   bme_temp_i   = 0;
int   bme_hum_i    = 0;
int   bme_press_i  = 0;
#endif

#if DHT22
// REMOVE DHT_U.h & .cpp from the library, otherwise you will get an error with NodeMCU
#include "DHT.h"                         
#define DHTPIN 27                          // what pin we're connected to     was 21
                                           // ESP8266 (LoLin 8 geht nicht, GPIO14=D5 OK)
// Uncomment whatever type you're using!
//  #define DHTTYPE DHT11                  // DHT 11 
#define DHTTYPE DHT22                      // DHT 22  (AM2302)       only DHT22 tested!
//  #define DHTTYPE DHT21                  // DHT 21  (AM2301)
long  cnt_DHTfail   = 0;
long  cnt_DHTok     = 0;
bool  DHT_failed    = false;
int   DHT_try       = 0;
float DHT_temp      = 0.0;
float DHT_hum       = 0.0;
int   DHT_t_i       = 0;
int   DHT_h_i       = 0;
DHT dht(DHTPIN, DHTTYPE);
#endif

#if SDS011
//---------------------------------------------------------
// div. timings for SDS011
//---------------------------------------------------------
#define SDS_SAMPLE_TIME 1000
#define SDS_WARMUP_TIME   30            // changed from 10" to be longer ventilated
#define SDS_READ_TIME      5

// SDS-Variables
unsigned long act_milli, prev_milli;    // Timer-Ticks to calculate 1 sec
bool    SDS_done    = false;            // true if SDS has ben read
bool is_SDS_running = true;             // true, if SDS011 is running
uint8_t timer_SDS;                      // Timer with 1sec ticks for SDS011 timimg

// Variables to calculate avereage for SDS011-Data
int sds_pm10_sum       = 0;         
int sds_pm25_sum       = 0;
int sds_val_count      = 0;
int sp1_av_i           = 0;
int sp2_av_i           = 0;
unsigned long SDS_ID   = 0;


// Kommands to start and stop SDS011
const byte stop_SDS_cmd[]  = {0xFF, 0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
const byte start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};


// Serial for SDS011
HardwareSerial Serial1(2);
#define serialSDS Serial1
#endif

int LED = 2;                            // GPIO02     blue

String result_SDS = "";
String result_DHT = "";
String result_BME = "";
String tosend_s   = "";


//---------------------------------------------------------
// Debugging included 
//---------------------------------------------------------
#ifdef my_DEBUG
#define DEBUG_WRT(...) { Serial.write(__VA_ARGS__); }
#define DEBUG_PRT(...) { Serial.print(__VA_ARGS__); }
#define DEBUG_PLN(...) { Serial.println(__VA_ARGS__); }
#define DEBUG_BEGIN()  { Serial.begin(57600); }
#else
#define DEBUG_WRT(...) {}
#define DEBUG_PRT(...) {}
#define DEBUG_PLN(...) {}
#define DEBUG_BEGIN() {}
#endif


//-------------------------------------------------------------------------------------------------
/*****************************************************************
*  convert float to string with a                                *
*  precision of 1 decimal place                                  *
******************************************************************/
String Float2String(const float value) {
  // Convert a float to String with two decimals.
  char temp[15];
  String s;

  dtostrf(value,13, 1, temp);
  s = String(temp);
  s.trim();
  return s;
}

void blink_error(int blinks) {
    if ( blinks != 0 ) {
      for ( int z = 0; z == blinks-1; z++ ) {
       digitalWrite(LED, HIGH);
       delay(100);                            // wait some time
       digitalWrite(LED, LOW);    
       delay(100);                            // wait some time 
      }
    } else {
      for ( int z = 0; z != 10000; z++ ) {
         digitalWrite(LED, HIGH);
         delay(100);                            // wait some time
         digitalWrite(LED, LOW);    
         delay(100);                            // wait some time 
      }
    }
}


//-------------------------------------------------------------------------------------------------
#if BME280
void sensorBME()  {
  // Reading Tempersture, Humidity and Pressure
    bme_temp    = bme.readTemperature();                    // Degree Celsius
    bme_hum     = bme.readHumidity();                       // % rel.Hum
    bme_press   = bme.readPressure() / 100.0F;              // hPa

  if ( !bme.begin() )  {
    blink_error(1);
    cnt_BMEfail++;
    BME_try++;
    BME_failed = true;
    DEBUG_PRT("Failed to read from BME, fail counter: ");
    DEBUG_PRT(cnt_BMEfail);
    DEBUG_PLN();
    delay(500);
  } else {
    if ( isnan(bme_temp) || isnan(bme_hum) || isnan(bme_press) ) {
       bme_temp   = 0.0;  bme_hum    = 0.0;  bme_press  = 0.0;
    }
    result_BME   = String(bme_temp) + ";" + String(bme_hum) + ";" + String(bme_press) + ";";
    cnt_BMEok++;
    BME_failed = false;
    DEBUG_PRT("BME Temp:  ");   DEBUG_PLN(bme_temp);
    DEBUG_PRT("BME Hum:   ");   DEBUG_PLN(bme_hum);
    DEBUG_PRT("BME Press: ");   DEBUG_PLN(bme_press);
	  DEBUG_PRT("BME ERROR cnt ok:"); DEBUG_PRT(cnt_BMEok);  DEBUG_PRT("  NOK: "); DEBUG_PRT(cnt_BMEfail);
  }  
  DEBUG_PLN();
    if ( (BME_try == 5) || ( cnt_BMEfail == 50) ) {
      bool status;
      status = bme.begin();
      DEBUG_PLN("BME reinit");
    }  
}
#endif


//-------------------------------------------------------------------------------------------------
#if DHT22
void sensorDHT()  {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  DHT_hum  = dht.readHumidity();         delay(100);
  DHT_temp = dht.readTemperature();      

  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if ( isnan(DHT_temp) || isnan(DHT_hum) ) {
    blink_error(1);
    cnt_DHTfail++;
    DHT_try++;
    DHT_failed = true;
    DEBUG_PRT("Failed to read from DHT, fail counter: ");
    DEBUG_PRT(cnt_DHTfail);

    delay(500);
  } else {
    result_DHT   = String(DHT_temp) + ";" + String(DHT_hum) + ";";
    cnt_DHTok++;
    DHT_failed = false;
    DEBUG_PRT("DHT: Temp: "); 
    DEBUG_PRT(DHT_temp);
    DEBUG_PLN("*C");
    DEBUG_PRT("Humidity:  "); 
    DEBUG_PRT(DHT_hum);
    DEBUG_PLN("%");
    DEBUG_PRT("DHT ERROR cnt ok:"); DEBUG_PRT(cnt_DHTok);  DEBUG_PRT("  NOK: "); DEBUG_PRT(cnt_DHTfail);
  }
    DEBUG_PLN();
    if ( (DHT_try == 5) || ( cnt_DHTfail == 50) ) {
      dht.begin();
      DEBUG_PLN("DHT reinit");
    }
}
#endif

// ------------------------------------------------------------------------------------------------------------
/*****************************************************************
*  read SDS011 sensor values                                     *
******************************************************************/
#if SDS011
void sensorSDS() {
  //  DEBUG_PLN("... 1 ...");
  char buffer;
  int value;
  int len = 0;
  int pm10_serial = 0;
  int pm25_serial = 0;
      SDS_ID      = 0;
  int checksum_is;
  int checksum_ok = 0;
  String sp1_av   = "0.0";
  String sp2_av   = "0.0";
  String SDS_ID_s = "";

  if (! is_SDS_running) {
    // DEBUG_PLN("... 2 ...");
    return;
  }
  
  // SDS runs: read serial buffer
  while (serialSDS.available() > 0) {
    buffer = serialSDS.read();
//      DEBUG_PLN(String(len)+" - "+String(buffer,DEC)+" - "+String(buffer,HEX)+" - "+int(buffer)+" .");
//      "aa" = 170, "ab" = 171, "c0" = 192
    value = int(buffer);
    switch (len) {
      case (0): if (value != 170) { len = -1; }; break;
      case (1): if (value != 192) { len = -1; }; break;
      case (2): pm25_serial = value;           checksum_is = value; break;
      case (3): pm25_serial += (value << 8);   checksum_is += value; break;
      case (4): pm10_serial = value;           checksum_is += value; break;
      case (5): pm10_serial += (value << 8);   checksum_is += value; break;
      case (6): SDS_ID = value;                checksum_is += value; break;
      case (7): SDS_ID += (value << 8);        checksum_is += value; break;
      case (8): 
             // DEBUG_PLN("Checksum is: "+String(checksum_is % 256)+" - should: "+String(value));
                if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
      case (9): if (value != 171) { len = -1; }; break;
    }
    len++;
    if ((len == 10 && checksum_ok == 1) && (timer_SDS > SDS_WARMUP_TIME)) {
      if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
        sds_pm10_sum += pm10_serial;
        sds_pm25_sum += pm25_serial;
        sds_val_count++;
      }
      len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
    }
    // yield();           used only if connected by WiFi
  }

  // Data for SDS_READTIME time is read: now calculate the average and return value
  if (timer_SDS > (SDS_WARMUP_TIME + SDS_READ_TIME)) {
    // Calculate average
    DEBUG_PLN("Sum: " + String(sds_pm10_sum) + "  Cnt: " + String(sds_val_count));
    sp1_av   = Float2String(float(sds_pm10_sum)/(sds_val_count*10.0));   sp1_av_i = int( (sds_pm10_sum)/(sds_val_count*10.0) * 10 );
    sp2_av   = Float2String(float(sds_pm25_sum)/(sds_val_count*10.0));   sp2_av_i = int( (sds_pm25_sum)/(sds_val_count*10.0) * 10 );
    SDS_ID_s = Float2String(SDS_ID);    
    int dezPoint = SDS_ID_s.indexOf('.');
    SDS_ID_s = SDS_ID_s.substring(0, dezPoint);                          SDS_ID = int(SDS_ID);
    DEBUG_PLN("Send-ID: " + SDS_ID_s);
    DEBUG_PLN("PM10:    " + sp1_av + "  " + String(sp1_av_i));
    DEBUG_PLN("PM2.5:   " + sp2_av + "  " + String(sp2_av_i));
    DEBUG_PLN("------------------");
    // result_SDS = Value2JsonMQTT("P1",sp1_av.c_str(),"P2",sp2_av.c_str(),NULL);
    result_SDS = SDS_ID_s + ";" + sp1_av + ";" + sp2_av + ";";                                      // CONVERT TO BYTE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // clear sums and count
    sds_pm10_sum = 0; sds_pm25_sum = 0; sds_val_count = 0;
    // and STOP SDS
    serialSDS.write(stop_SDS_cmd,sizeof(stop_SDS_cmd)); 
    is_SDS_running = false;
    DEBUG_PLN("SDS stopped");
    SDS_done = true;
  }
}
#endif


// ------------------------------------------------------------------------------------------------------------
// Read Sensors
void ReadSensors() {
    tosend_s = "";
    DEBUG_PLN("\n----------------------------------------------------------");
    
    // SDS
    // Now start SDS senor
	  #if SDS011
     serialSDS.write(start_SDS_cmd,sizeof(start_SDS_cmd)); 
     is_SDS_running = true;
     SDS_done       = false;
     timer_SDS      = 0;              // start timer
     DEBUG_PLN("SDS started");

     while(1) 
     {
       act_milli = millis();       // read system-tick
       if((act_milli - prev_milli) >= SDS_SAMPLE_TIME) {     // after SAMPLE_TIME (==0 1sec)
         prev_milli = act_milli;
         timer_SDS += 1;           // Count SDS-Timer
         sensorSDS();              // check (and read)  SDS011   
         if ( SDS_done ) break;
       }
     }
     tosend_s += result_SDS;
    #endif

  // BME280
    #if BME280
     BME_try    = 0;
     BME_failed = false;
     while ( BME_try <= 5 )
     {
       sensorBME();
       if ( BME_failed == false ) {
         break;
       } else {
         result_BME ="0.0;0.0;0;";
         bme_temp   = 0.0;  bme_hum    = 0.0;  bme_press  = 0.0;
       }
     }
      tosend_s   += result_BME;
      bme_temp_i  = int(bme_temp * 100);
      bme_hum_i   = int(bme_hum  * 100);
      bme_press_i = int(bme_press);
      // DEBUG_PLN( "BME:  Temp: " + String(bme_temp_i) + "  Humidity: " + String(bme_hum_i) + "  Pressure: " + String(bme_press_i) );
    #endif
  
    // DHT
    // it seems, that DHT sometimes failes, so give it 5 tries to read good data
    #if DHT22
     DHT_try    = 0;
     DHT_failed = false;
     while ( DHT_try <= 5 )                         //|| (DHT_failed == true)
     {
       sensorDHT();
       if ( DHT_failed == false ) {
          break; 
       } else {
         result_DHT ="0.0;0.0;";
         DHT_temp = 0.0;  DHT_hum = 0.0;
       }
     }
       tosend_s += result_DHT;
       DHT_t_i = int(DHT_temp * 100);
       DHT_h_i = int(DHT_hum  * 100);

    #endif

     DEBUG_PLN(" SDS             BME            DHT");
     DEBUG_PLN(tosend_s);

  // for debugging:                       423F BACB 5BA0 F280 223D       3F000000 BACB 5BA0 F280 223D 0000003FBACB5BA0F280223D
  /* SDS_ID   = 999999;                   // 999999 = 
     sp1_av_i =  47819;                   //  47819 =          (4781.9)
     sp2_av_i =  23456;                   //  23456 =          (2345.6)
     t_i      =  -3456;                   //  -3456 =          (-34.56)
     h_i      =   8765;                   //   8765 =          ( 87.65)
   */
   #if SDS011                                    // bytes 00..07 are for SDS011
     mydata[0]  = byte(SDS_ID);
     mydata[1]  = byte(SDS_ID >>  8);
     mydata[2]  = byte(SDS_ID >> 16);
     mydata[3]  = byte(SDS_ID >> 24);
     mydata[4]  = highByte(sp1_av_i);
     mydata[5]  =  lowByte(sp1_av_i);
     mydata[6]  = highByte(sp2_av_i);
     mydata[7]  =  lowByte(sp2_av_i);
    #endif
    #if ( DHT22 && !BME280 )                     // just to save some cycles
     mydata[8]  = highByte(DHT_t_i);
     mydata[9]  =  lowByte(DHT_t_i);
     mydata[10] = highByte(DHT_h_i);
     mydata[11] =  lowByte(DHT_h_i);
    #endif
    #if BME280                                   // BME280
      if ( !BME_failed ) {                       // do not fill 
         mydata[8]  = highByte(bme_temp_i);
         mydata[9]  =  lowByte(bme_temp_i);
         mydata[10] = highByte(bme_hum_i);
         mydata[11] =  lowByte(bme_hum_i);
         mydata[12] = highByte(bme_press_i);
         mydata[13] =  lowByte(bme_press_i);
      }
    #endif	
    #if ( BME280 && DHT22 )
      if ( BME_failed ) {
         mydata[8]  = highByte(DHT_t_i);
         mydata[9]  =  lowByte(DHT_t_i);
         mydata[10] = highByte(DHT_h_i);
         mydata[11] =  lowByte(DHT_h_i);
      }
    #endif    
    #if BME280
      if ( BME_failed ) {
         mydata[12] = 0;
         mydata[13] = 0;
      }
    #endif
	
/*   mydata[x] = highByte(Vcc_i);
     mydata[x] =  lowByte(Vcc_i);    */

     for ( int i=0; i!=(sizeof(mydata)-1); i++ )  {
       DEBUG_PRT( mydata[i], HEX );  delay(10);
       }
       DEBUG_PLN();

     // use this to send string, but only for debugging, please.   mydata buffer must be resized !!!
     // sprintf((char *)mydata,"%s",tosend_s.c_str());
    
}


// ------------------------------------------------------------------------------------------------------------
void onEvent (ev_t ev) {
    DEBUG_PRT(os_getTime());
    DEBUG_PRT(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            DEBUG_PLN(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            DEBUG_PLN(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            DEBUG_PLN(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            DEBUG_PLN(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            DEBUG_PLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            DEBUG_PLN(F("EV_JOINED"));
            break;
        case EV_RFU1:
            DEBUG_PLN(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            DEBUG_PLN(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            DEBUG_PLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            DEBUG_PLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              DEBUG_PLN(F("Received ack"));
            if (LMIC.dataLen) {
              DEBUG_PRT(F("Received "));
              DEBUG_PRT(LMIC.dataLen);
              DEBUG_PRT(F(" bytes of payload , Data: "));
              DEBUG_WRT(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              DEBUG_PLN();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            DEBUG_PLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            DEBUG_PLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            DEBUG_PLN(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            DEBUG_PLN(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            DEBUG_PLN(F("EV_LINK_ALIVE"));
            break;
         default:
            DEBUG_PLN(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        DEBUG_PLN(F("OP_TXRXPEND, not sending"));
    } else {
      ReadSensors();
        // Prepare upstream data transmission at the next possible time.
           LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);                                    // sending bytes
        // LMIC_setTxData2(1, mydata, strlen((char *)mydata), 0);                              // sending string  (avoided!)
        
      DEBUG_PLN(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


// ------------------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
      delay(100);
      pinMode(LED, OUTPUT);                              // initialize onboard LED as output

    DEBUG_PLN("Starting on USB Serial");
      delay(100);
    serialSDS.begin(9600);                               // 
      delay(100);
      // serialSDS.println("Starting on Serial-1 ...");  // debugging only, use a FTDIadapter at Tx (only Tx & GND connected!)
      // delay(100);
    #if DHT22
      dht.begin();
      delay(500);
    #endif

    #if BME280
       bool status;
       status = bme.begin();
       if (!status) {
            DEBUG_PLN("Could not find a valid BME280 sensor, check wiring!");
            blink_error(0);
            // watchdog_reset_esp32();
            while (1);
       }
    #endif
    
    // now read each sensor once

    #if DHT22
      sensorDHT();
    #endif  
    #if BME280
      sensorBME();
    #endif
    #if SDS011
      sensorSDS();
    #endif
    delay(100);
    
    // LMIC init
    os_init();
    DEBUG_PLN("os_init done");
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    DEBUG_PLN("lmic reset done");

    // start joining
    // LMIC_startJoining();

     
/*   when using this initial message an error in radio.c at line 660 will occur
     mydata[00] = 0xAA;        // at startup you could send an ID for example
     mydata[01] = 0x31;        // "1"
     mydata[02] = 0x32;        // "2"
     mydata[03] = 0x33;        // "3"
     delay(100);
     LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
*/

    // Start job
    do_send(&sendjob);
}

void loop() {

   os_runloop_once();
   
}

// ------------------------------------------------------------------------------------------------------------
/*
PAYLOAD FUNCTION

function Decoder(bytes, port) {
  var SDS_ID      = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
  var SDS_PM10    = (bytes[4] << 8)  | bytes[5];
  var SDS_PM25    = (bytes[6] << 8)  | bytes[7];  
  var temperature = (bytes[8] <<8)   | bytes[9];
  var humidity    = (bytes[10] <<8)  | bytes[11];
//  var pressure    = (bytes[4] <<8) | bytes[5];
//  var batteryV    = (bytes[6] <<8) | bytes[7];
  
  return {
  SDS_ID:      SDS_ID,
  PM10:        SDS_PM10 / 10,
  PM25:        SDS_PM25 / 10,
    temperature: temperature / 100,
    humidity:    humidity / 100,
//    pressure:  pressure,
//    batteryV:  batteryV / 1000
  }
  
}

test with:  D5B3000000F5006E08DE168A

result
{
  "PM10": 24.5,
  "PM25": 11,
  "SDS_ID": 46037,
  "humidity": 57.7,
  "temperature": 22.7
}

from console:
payload:D5B3000000B0005E08CA1734  PM10:17.6  PM25:9.4  SDS_ID:46037  humidity:59.4  temperature:22.5


20171016
Humidity: 50.20 %  Temperature: 27.10 *C    ok: 6   fails: 2
t: 2710 h: 5020
SDS started
Sum: 1309  Cnt: 6
Send-ID: 46037
PM10:    21.8  218
PM2.5:   8.0  80
------------------
SDS stopped
27.10;50.20;46037;21.8;8.0;
D5B3000DA050A96139C
Packet queued
12128619: EV_TXCOMPLETE (includes waiting for RX windows)



D5B30000019C004F0A90118003C9000000





------------------------------------------------------------------------------

load & run
2 full cycles
remove DHT22  5xfailed, reinit, failed --> values = 0.0;0.0  (46037;5.9;2.2;28.54;41.61;973.03;0.0;0.0;)
insert DHT22      --> 46037;6.1;2.2;28.73;40.82;972.99;26.90;53.80;
remove BME280  ===> HANGS , reset
--------------
1 full cycle      --> 46037;8.2;2.1;29.40;39.73;972.94;26.80;47.60;
remove BME280     --> 46037;7.7;2.4;0.0;0.0;0;26.70;46.20;     temp & hum from DHT22, press 0
insert BME280     ==> HANGS , reset
--------------
ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
ets Jun  8 2016 00:22:57

rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0x00
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0008,len:8
load:0x3fff0010,len:160
load:0x40078000,len:10632
load:0x40080000,len:252
entry 0x40080034
Starting on USB Serial
DHT: Temp: 27.60*C
Humidity:  57.20%
DHT ERROR cnt ok:1  NOK: 0
BME Temp:  28.80
BME Hum:   41.26
BME Press: 973.02
BME ERROR cnt ok:1  NOK: 0
os_init done
lmic reset done

----------------------------------------------------------
SDS started
Sum: 272  Cnt: 6
Send-ID: 46037
PM10:    4.5  45
PM2.5:   2.0  20
------------------
SDS stopped
BME Temp:  28.56
BME Hum:   41.01
BME Press: 973.05
BME ERROR cnt ok:2  NOK: 0
DHT: Temp: 28.30*C
Humidity:  44.30%
DHT ERROR cnt ok:2  NOK: 0
 SDS             BME            DHT
46037;4.5;2.0;28.56;41.01;973.05;28.30;44.30;
D5B30002D014B281043CD000
Packet queued
1243881: EV_JOINING
2012454: EV_JOINED
2143698: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 576  Cnt: 6
Send-ID: 46037
PM10:    9.6  96
PM2.5:   2.5  25
------------------
SDS stopped
BME Temp:  28.15
BME Hum:   42.20
BME Press: 973.02
BME ERROR cnt ok:3  NOK: 0
Failed to read from DHT, fail counter: 1
Failed to read from DHT, fail counter: 2
Failed to read from DHT, fail counter: 3
Failed to read from DHT, fail counter: 4
Failed to read from DHT, fail counter: 5
DHT reinit
DHT: Temp: 27.00*C
Humidity:  46.90%
DHT ERROR cnt ok:3  NOK: 5
DHT reinit
 SDS             BME            DHT
46037;9.6;2.5;28.15;42.20;973.02;27.00;46.90;
D5B300060019AFF107C3CD000
Packet queued
7432073: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 357  Cnt: 6
Send-ID: 46037
PM10:    5.9  59
PM2.5:   2.2  22
------------------
SDS stopped
BME Temp:  28.54
BME Hum:   41.61
BME Press: 973.03
BME ERROR cnt ok:4  NOK: 0
Failed to read from DHT, fail counter: 6
Failed to read from DHT, fail counter: 7
Failed to read from DHT, fail counter: 8
Failed to read from DHT, fail counter: 9
Failed to read from DHT, fail counter: 10
DHT reinit
Failed to read from DHT, fail counter: 11
 SDS             BME            DHT
46037;5.9;2.2;28.54;41.61;973.03;0.0;0.0;
D5B30003B016B2610413CD000
Packet queued
12763283: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 365  Cnt: 6
Send-ID: 46037
PM10:    6.1  60
PM2.5:   2.2  21
------------------
SDS stopped
BME Temp:  28.73
BME Hum:   40.82
BME Press: 972.99
BME ERROR cnt ok:5  NOK: 0
DHT: Temp: 26.90*C
Humidity:  53.80%
DHT ERROR cnt ok:4  NOK: 11
 SDS             BME            DHT
46037;6.1;2.2;28.73;40.82;972.99;26.90;53.80;
D5B30003C015B39FF23CC000
Packet queued
17767361: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 566  Cnt: 6
Send-ID: 46037
PM10:    9.4  94
PM2.5:   2.7  27
------------------
SDS stopped
Failed to read from BME, fail counter: 1

ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
ets Jun  8 2016 00:22:57

rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0x00
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0008,len:8
load:0x3fff0010,len:160
load:0x40078000,len:10632
load:0x40080000,len:252
entry 0x40080034
Starting on USB Serial
Failed to read from DHT, fail counter: 1
BME Temp:  30.26
BME Hum:   38.47
BME Press: 972.95
BME ERROR cnt ok:1  NOK: 0
os_init done
lmic reset done

----------------------------------------------------------
SDS started
Sum: 494  Cnt: 6
Send-ID: 46037
PM10:    8.2  82
PM2.5:   2.1  21
------------------
SDS stopped
BME Temp:  29.40
BME Hum:   39.73
BME Press: 972.94
BME ERROR cnt ok:2  NOK: 0
DHT: Temp: 26.80*C
Humidity:  47.60%
DHT ERROR cnt ok:1  NOK: 1
 SDS             BME            DHT
46037;8.2;2.1;29.40;39.73;972.94;26.80;47.60;
D5B300052015B7CF843CC000
Packet queued
1287507: EV_JOINING
1646388: EV_JOINED
1777632: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 460  Cnt: 6
Send-ID: 46037
PM10:    7.7  76
PM2.5:   2.4  23
------------------
SDS stopped
Failed to read from BME, fail counter: 1

Failed to read from BME, fail counter: 2

Failed to read from BME, fail counter: 3

Failed to read from BME, fail counter: 4

Failed to read from BME, fail counter: 5

BME reinit
Failed to read from BME, fail counter: 6

Failed to read from DHT, fail counter: 2
Failed to read from DHT, fail counter: 3
Failed to read from DHT, fail counter: 4
DHT: Temp: 26.70*C
Humidity:  46.20%
DHT ERROR cnt ok:2  NOK: 4
 SDS             BME            DHT
46037;7.7;2.4;0.0;0.0;0;26.70;46.20;
D5B30004C017A6E12C00000
Packet queued
6073592: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 303  Cnt: 6
Send-ID: 46037
PM10:    5.1  50
PM2.5:   2.5  25
------------------
SDS stopped
ets Jun  8 2016 00:22:57

rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
ets Jun  8 2016 00:22:57

rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0x00
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0008,len:8
load:0x3fff0010,len:160
load:0x40078000,len:10632
load:0x40080000,len:252
entry 0x40080034
Starting on USB Serial
DHT: Temp: 26.70*C
Humidity:  46.10%
DHT ERROR cnt ok:1  NOK: 0
BME Temp:  27.66
BME Hum:   41.02
BME Press: 972.99
BME ERROR cnt ok:1  NOK: 0
os_init done
lmic reset done

----------------------------------------------------------
SDS started
Sum: 364  Cnt: 6
Send-ID: 46037
PM10:    6.1  60
PM2.5:   3.1  31
------------------
SDS stopped
BME Temp:  27.72
BME Hum:   38.25
BME Press: 972.93
BME ERROR cnt ok:2  NOK: 0
Failed to read from DHT, fail counter: 1
Failed to read from DHT, fail counter: 2
Failed to read from DHT, fail counter: 3
Failed to read from DHT, fail counter: 4
Failed to read from DHT, fail counter: 5
DHT reinit
Failed to read from DHT, fail counter: 6
 SDS             BME            DHT
46037;6.1;3.1;27.72;38.25;972.93;0.0;0.0;
D5B30003C01FAD4EF13CC000
Packet queued
1571694: EV_JOINING
2308069: EV_JOINED
2439313: EV_TXCOMPLETE (includes waiting for RX windows)

----------------------------------------------------------
SDS started
Sum: 837  Cnt: 6
Send-ID: 46037
PM10:    13.9  139
PM2.5:   3.3  32
------------------
SDS stopped
BME Temp:  28.40
BME Hum:   37.48
BME Press: 972.97
BME ERROR cnt ok:3  NOK: 0
Failed to read from DHT, fail counter: 7
Failed to read from DHT, fail counter: 8
Failed to read from DHT, fail counter: 9
DHT: Temp: 26.30*C
Humidity:  42.20%
DHT ERROR cnt ok:2  NOK: 9
 SDS             BME            DHT
46037;13.9;3.3;28.40;37.48;972.97;26.30;42.20;
D5B30008B020B18EA43CC000
Packet queued
7610511: EV_TXCOMPLETE (includes waiting for RX windows)



*/
