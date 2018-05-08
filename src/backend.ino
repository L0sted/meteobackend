//backend
#include <BH1750.h>
#include <BMP280.h>
#include <Wire.h>
#include <NTPClient.h> 
#include <WiFiUdp.h> 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include "DHTesp.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include "wifi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

float inTemp,humid,extTemp;
double bmpTemp,pressure,altitude,lux;

const short ds18pin = 4, bmpsda = 5, bmpscl = 16, lightSCL = 13, lightSDA = 12, dhtpin = 15;
/*
  light connection:

    VCC -> 3V3 or 5V
    GND -> GND
    SCL -> SCL (A5 on Arduino Uno, Leonardo, etc or 21 on Mega and Due, on esp8266 free selectable)
    SDA -> SDA (A4 on Arduino Uno, Leonardo, etc or 20 on Mega and Due, on esp8266 free selectable)
    ADD -> (not connected) or GND

  ADD pin is used to set sensor I2C address. If it has voltage greater or equal to
  0.7VCC voltage (e.g. you've connected it to VCC) the sensor address will be
  0x5C. In other case (if ADD voltage less than 0.7 * VCC) the sensor address will
  be 0x23 (by default).

*/

BMP280 bmp;

DHTesp dht;
OneWire oneWire(ds18pin);
DallasTemperature DS18B20(&oneWire);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "192.168.100.102", 1883);
Adafruit_MQTT_Publish extTempMQTT = Adafruit_MQTT_Publish(&mqtt, "externalTemp");
Adafruit_MQTT_Publish inTempMQTT = Adafruit_MQTT_Publish(&mqtt, "inTemp");
Adafruit_MQTT_Publish humidMQTT = Adafruit_MQTT_Publish(&mqtt, "humid");
Adafruit_MQTT_Publish bmpTempMQTT = Adafruit_MQTT_Publish(&mqtt, "bmpTemp");
Adafruit_MQTT_Publish pressureMQTT = Adafruit_MQTT_Publish(&mqtt, "pressure");

BH1750 lightMeter;

void setup(){
  Serial.begin(115200);
  //==WIFI CONNECT==
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("Connected to " + String(ssid) + "; IP address: " + WiFi.localIP());
  MDNS.begin("esp8266-backend");
  dht.setup(dhtpin);
  //==BMP INIT==
  if(!bmp.begin(bmpsda,bmpscl)){
      Serial.println("BMP init failed!\n Reset in 10 seconds");
      delay(10000);
      ESP.reset();
  }
  else {
    Serial.println("BMP init success!");  
    bmp.setOversampling(4);
  }
  
  Wire.begin(lightSCL, lightSDA);
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    Serial.println("lightMeter error!");
}

void loop(){
  // Serial.println("DHT");
  getAccurateDHT();
  // Serial.println("DS18B20");
  getDS18();
  // Serial.println("BMP280");
  getBMP();
  // Serial.println("MQTT");
  MQTT_loop();
}


//====================IN PROGRESS===================

//===================WELL DONE=======================
void getBMP(){
  char result = bmp.startMeasurment();
  if(result!=0){
    delay(result);
    bmp.getTemperatureAndPressure(bmpTemp,pressure);
  }
}

void getDS18(){
  do {
    DS18B20.requestTemperatures(); 
    extTemp = DS18B20.getTempCByIndex(0);
  } while (extTemp == 85.0 || extTemp == (-127.0));
}

void getLight(){
  lux = lightMeter.readLightLevel();
}

void serialPrint() {
  Serial.println("BMP280 Temperature: " + String(bmpTemp) + "degC");
  Serial.println("Pressure: " + String(pressure) + "mBar");
  Serial.println("Altitude: " + String(altitude) + "m");

  Serial.println("DS18B20 Temperature: " + String(extTemp) + "degC");

  Serial.println("DHT11 Temperature: " + String(inTemp) + "degC");
  Serial.println("Humidity" + String(humid) + "%");
}

void getAccurateDHT(){
  humid = 0.0;
  inTemp = 0.0;
  for (int i = 0;i < 3; ++i) { //i dunno why it is incorrect sometimes
    delay(dht.getMinimumSamplingPeriod());
    humid += (dht.getHumidity())/3.0;
    inTemp += (dht.getTemperature())/3.0;
  }
}
void MQTT_loop() {
  MQTT_connect();

  if (! extTempMQTT.publish(extTemp)) {
    Serial.println(F("Failed"));
  }
  if (! inTempMQTT.publish(inTemp)) {
    Serial.println(F("Failed"));
  }
  if (! humidMQTT.publish(humid)) {
    Serial.println(F("Failed"));
  }
  if (! bmpTempMQTT.publish(bmpTemp)) {
    Serial.println(F("Failed"));
  }
  if (! pressureMQTT.publish(pressure)) {
    Serial.println(F("Failed"));
  }

  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  // Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
