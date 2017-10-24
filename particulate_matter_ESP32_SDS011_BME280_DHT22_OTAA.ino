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
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
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
const unsigned TX_INTERVAL = 60;           // 600 is the minimal delay used between measurements

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
#define DHT22     0                        // use DHT22
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



//-------------------------------------------------------------------------------------------------
#if BME280
void sensorBME()  {
  // Reading Tempersture, Humidity and Pressure
  bme_temp    = bme.readTemperature();               // Degree Celsius
  bme_hum     = bme.readHumidity();                  // % rel.Hum
  bme_press   = bme.readPressure() / 100.0F;         // hPa

  if ( isnan(bme_temp) || isnan(bme_hum) || isnan(bme_press) )  {
    cnt_BMEfail++;
    BME_try++;
    BME_failed = true;
    DEBUG_PRT("Failed to read from BME, fail counter: ");
    DEBUG_PRT(cnt_BMEfail);
    DEBUG_PLN();
    delay(500);
  } else {
    result_BME   = String(bme_temp) + ";" + String(bme_hum) + ";" + String(bme_press) + ";";
    cnt_BMEok++;
    BME_failed = false;
    DEBUG_PRT("BME Temp:  ");   DEBUG_PLN(bme_temp);
    DEBUG_PRT("BME Hum:   ");   DEBUG_PLN(bme_hum);
    DEBUG_PRT("BME Press: ");   DEBUG_PLN(bme_press);
	  DEBUG_PRT("ERROR cnt ok:"); DEBUG_PRT(cnt_BMEok);  DEBUG_PRT("  NOK: "); DEBUG_PLN(cnt_BMEfail);
  }  
  DEBUG_PLN();
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
    cnt_DHTfail++;
    DHT_try++;
    DHT_failed = true;
    DEBUG_PRT("Failed to read from DHT, fail counter: ");
    DEBUG_PRT(cnt_DHTfail);
    DEBUG_PLN();
    delay(500);
  } else {
    result_DHT   = String(DHT_temp) + ";" + String(DHT_hum) + ";";
    cnt_DHTok++;
    DHT_failed = false;
    DEBUG_PRT("DHT: Temp: "); 
    DEBUG_PRT(DHT_temp);
    DEBUG_PRT("*C");
    DEBUG_PRT(" Humidity: "); 
    DEBUG_PRT(DHT_hum);
    DEBUG_PLN("%");
    
    DEBUG_PRT("ERROR cnt ok:"); DEBUG_PRT(cnt_DHTok);  DEBUG_PRT("  NOK: "); DEBUG_PLN(cnt_DHTfail);
  }
    DEBUG_PLN();
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
    
    // BME280
	#if BME280
     BME_try = 0;
     while ( (BME_try <= 5) || (BME_failed == true) )
     {
       sensorBME();
       if ( BME_failed == false ) {
          tosend_s   += result_BME;
          bme_temp_i  = int(bme_temp * 100);
          bme_hum_i   = int(bme_hum  * 100);
		      bme_press_i = int(bme_press);
          DEBUG_PLN( "BME:  Temp: " + String(bme_temp_i) + "  Humidity: " + String(bme_hum_i) + "  Pressure: " + String(bme_press_i) );
          break; 
       } else {
         result_BME ="0.0;0.0;0;";
       }
     }
    #endif
	
    // DHT
    // it seems, that DHT sometimes failes, so give it 5 tryes to read good data
    #if DHT22
     DHT_try = 0;
     while ( (DHT_try <= 5) || (DHT_failed == true) )
     {
       sensorDHT();
       if ( DHT_failed == false ) {
          tosend_s += result_DHT;
          DHT_t_i = int(DHT_temp * 100);
          DHT_h_i = int(DHT_hum  * 100);
          break; 
       } else {
         result_DHT ="0.0;0.0;";
       }
     }
    #endif
	
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

     DEBUG_PLN(" BME            DHT    SDS");
     DEBUG_PLN(tosend_s);

  // for debugging:                       423F BACB 5BA0 F280 223D       3F000000 BACB 5BA0 F280 223D 0000003FBACB5BA0F280223D
  /* SDS_ID   = 999999;                   // 999999 = 
     sp1_av_i =  47819;                   //  47819 =          (4781.9)
     sp2_av_i =  23456;                   //  23456 =          (2345.6)
     t_i      =  -3456;                   //  -3456 =          (-34.56)
     h_i      =   8765;                   //   8765 =          ( 87.65)
   */
   #if SDS011
     mydata[0]  = byte(SDS_ID);
     mydata[1]  = byte(SDS_ID >>  8);
     mydata[2]  = byte(SDS_ID >> 16);
     mydata[3]  = byte(SDS_ID >> 24);
     mydata[4]  = highByte(sp1_av_i);
     mydata[5]  =  lowByte(sp1_av_i);
     mydata[6]  = highByte(sp2_av_i);
     mydata[7]  =  lowByte(sp2_av_i);
    #endif
    #if DHT22                               // will be overwritten by values of the BME280 sensor
     mydata[8]  = highByte(DHT_t_i);
     mydata[9]  =  lowByte(DHT_t_i);
     mydata[10] = highByte(DHT_h_i);
     mydata[11] =  lowByte(DHT_h_i);
    #endif
    #if BME280
     mydata[8]  = highByte(bme_temp_i);
     mydata[9]  =  lowByte(bme_temp_i);
     mydata[10] = highByte(bme_hum_i);
     mydata[11] =  lowByte(bme_hum_i);
     mydata[12] = highByte(bme_press_i);
     mydata[13] =  lowByte(bme_press_i);
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
      // default settings
      status = bme.begin();
      if (!status) {
          Serial.println("Could not find a valid BME280 sensor, check wiring!");
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
Failed to read from BME, fail counter: 1

RXMODE_RSSI
os_init done
lmic reset done

----------------------------------------------------------
BME Temp:  26.01
BME Hum:   41.03
BME Press: 973.93
ERROR cnt ok:1  NOK: 1

BME:  Temp: 2601  Humidity: 4103  Pressure: 973
SDS started
Sum: 486  Cnt: 6
Send-ID: 46037
PM10:    8.1  81
PM2.5:   1.2  12
------------------
SDS stopped
 BME            DHT    SDS
26.01;41.03;973.93;46037;8.1;1.2;
D5B3000510CA291073CD000
1158440: engineUpdate, opmode=0x8
1158452: Scheduled job 0x3ffc2890, cb 0x400d337c ASAP
Packet queued
1158459: Running job 0x3ffc2890, cb 0x400d337c, deadline 0
1158649: EV_JOINING
1158762: engineUpdate, opmode=0xc
1158953: Uplink join pending
1159115: Airtime available at 1286025 (previously determined)
1159457: Uplink delayed until 1286025
1159669: Scheduled job 0x3ffc2890, cb 0x400d3388 at 1285900
1285900: Running job 0x3ffc2890, cb 0x400d3388, deadline 1285900
1285904: engineUpdate, opmode=0xc
1285907: Uplink join pending
1285917: Airtime available at 1286025 (previously determined)
1286258: Ready for uplink
1286418: Updating info for TX at 1285907, airtime will be 3856. Setting available time for band 0 to 5141907
1287026: TXMODE, freq=868300000, len=23, SF=7, BW=125, CR=4/5, IH=0
1290893: irq: dio: 0x0 flags: 0x8
1290901: Scheduled job 0x3ffc2890, cb 0x400d1ff8 ASAP
1290905: Running job 0x3ffc2890, cb 0x400d1ff8, deadline 0
1291015: Scheduled job 0x3ffc2890, cb 0x400d1cf0 at 1603362
1603362: Running job 0x3ffc2890, cb 0x400d1cf0, deadline 1603362
1603488: RXMODE_SINGLE, freq=868300000, SF=7, BW=125, CR=4/5, IH=0
1607933: irq: dio: 0x0 flags: 0x40
1607950: Scheduled job 0x3ffc2890, cb 0x400d3330 ASAP
1607954: Running job 0x3ffc2890, cb 0x400d3330, deadline 0
1608079: Setup channel, idx=3, freq=867100000
1608315: Setup channel, idx=4, freq=867300000
1608569: Setup channel, idx=5, freq=867500000
1608824: Setup channel, idx=6, freq=867700000
1609079: Setup channel, idx=7, freq=867900000
1609341: EV_JOINED
1609442: engineUpdate, opmode=0x808
1609643: Uplink data pending
1609806: Considering band 0, which is available at 1608078
1610132: Considering band 3, which is available at 0
1610425: No channel found in band 3
1610625: Considering band 0, which is available at 1608078
1610951: Airtime available at 1608078 (channel duty limit)
1611276: Ready for uplink
1611445: Updating info for TX at 1609643, airtime will be 4496. Setting available time for band 0 to 6105643
1612043: TXMODE, freq=867100000, len=30, SF=7, BW=125, CR=4/5, IH=0
1616550: irq: dio: 0x0 flags: 0x8
1616558: Scheduled job 0x3ffc2890, cb 0x400d1fdc ASAP
1616562: Running job 0x3ffc2890, cb 0x400d1fdc, deadline 0
1616671: Scheduled job 0x3ffc2890, cb 0x400d1ce4 at 1679019
1679019: Running job 0x3ffc2890, cb 0x400d1ce4, deadline 1679019
1679146: RXMODE_SINGLE, freq=867100000, SF=7, BW=125, CR=4/5, IH=0
1679485: irq: dio: 0x1 flags: 0x80
1679493: Scheduled job 0x3ffc2890, cb 0x400d3504 ASAP
1679546: Running job 0x3ffc2890, cb 0x400d3504, deadline 0
1679871: Scheduled job 0x3ffc2890, cb 0x400d1d34 at 1741807
1741807: Running job 0x3ffc2890, cb 0x400d1d34, deadline 1741807
1741934: RXMODE_SINGLE, freq=869525000, SF=9, BW=125, CR=4/5, IH=0
1743233: irq: dio: 0x1 flags: 0x80
1743241: Scheduled job 0x3ffc2890, cb 0x400d3534 ASAP
1743245: Running job 0x3ffc2890, cb 0x400d3534, deadline 0
1743360: EV_TXCOMPLETE (includes waiting for RX windows)
1743674: Scheduled job 0x3ffc1dac, cb 0x400d11c0 at 5493674
1744005: engineUpdate, opmode=0x900


*/
