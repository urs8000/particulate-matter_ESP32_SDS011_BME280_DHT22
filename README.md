### Feinstaub-Sensor connected with LoRaWan by RFM95
 based on the ESP8266 version of the community Stuttgart (Open Data Stuttgart)
http://codefor.de/stuttgart/  build: http://luftdaten.info/feinstaubsensor-bauen/

### used hardware
* microcontroller:	ESP32 (DOIT)
* Feinstaubsensor:	SDS011		http://inovafitness.com/en/Laser-PM2-5-Sensor-SDS011-35.html
* Temp/Hum:			DHT22 (no resistor between Vcc & Data when powered by 3.3V!)
* Temp/Hum/Press:	BME280
* LoRa Module:		RFM95 on breadboard
* Battery			Power Pack (10000mAh)

### power supply
* powered via USB: 5V, Vin on board connected to SDS011 

### power measurement
running:  110 - 130mA
waiting:   70mA  (no sleep mode implemented)

### authentication
* both authentication methods are available
  you have to configure the device via the console in the desired method
* - Over-the-Air Activation (OTAA)
* - Activation by Personalization (ABP)

### remarks
* please read carefully the changes made in lmic/config.h
* and the compiling selections made for the board
* you need to install ESP32 in the Arduino IDE (link in code)
* Keys for ABP were separeted, structure explained
* the values from DHT22 will be overwritten if both sensors are selected (see remarks in the code)

function Decoder(bytes, port) {
  var SDS_ID      = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
  var SDS_PM10    = (bytes[4] << 8)  | bytes[5];
  var SDS_PM25    = (bytes[6] << 8)  | bytes[7];  
  var temp_x      = (bytes[8] <<8)   | bytes[9];
  var hum_x       = (bytes[10] <<8)  | bytes[11];
  var press_bme   = (bytes[12] <<8)  | bytes[13];
//  var batteryV    = (bytes[x] <<8) | bytes[x];
  
  return {
    SDS_ID:       SDS_ID,
	PM10:         SDS_PM10 / 10,
	PM25:         SDS_PM25 / 10,
    Temperature:  temp_x / 100,
    Humidity:     hum_x / 100,
    Pressure:     press_bme
//  batteryV:  batteryV / 1000         not i,plemented jet
  }
  
}
