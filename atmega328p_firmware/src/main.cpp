#include <Arduino.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include "TinyGPS++.h"

#define ONEWIRE_PIN 2
#define DHT_TYPE DHT22

/*****************************************************************
 *  GPS PINS
 ****************************************************************/
#define GPS_RX_PIN 6
#define GPS_TX_PIN 7

/*****************************************************************
 * DHT declaration                                               *
 *****************************************************************/
DHT dht(ONEWIRE_PIN, DHT_TYPE);

/*****************************************************************
 *  GPS DECLARATION
 * ****************************************************************/
SoftwareSerial serialGPS(GPS_TX_PIN,GPS_RX_PIN);
TinyGPSPlus gps;

/******************************************************************
 *  DHT VARIABLES
 * ***************************************************************/
float dht_humidity = 0.0;
float dht_temperature = 0.0;

/******************************************************************
 *  GPS VARIABLES
 * ****************************************************************/
double last_value_GPS_lat = -200.0;
double last_value_GPS_lon = -200.0;
double last_value_GPS_alt = -1000.0;
String last_value_GPS_date;
String last_value_GPS_time;

/*****************************************************
 * READ DHT SENSOR VALUES
 ********************************************************/
void fetchSensorDHT(float &t, float &h){

  	t = dht.readTemperature();
	  h = dht.readHumidity();

		if (isnan(t) || isnan(h)) {
			delay(100);
			t = dht.readTemperature(false);
			h = dht.readHumidity();
		}
		if (isnan(t) || isnan(h)) {
			Serial.println(F("DHT11/DHT22 read failed"));
		}
}


void setup() {

  Serial.begin(9600);
  serialGPS.begin(9600); //set GPS baudrate
  dht.begin();

  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());

}

void loop() {

  fetchSensorDHT(dht_temperature,dht_humidity);

  Serial.print("TEMPERATURE : ");
  Serial.print(dht_temperature);
  Serial.print(" HUMIDITY : ");
  Serial.println(dht_humidity);

}