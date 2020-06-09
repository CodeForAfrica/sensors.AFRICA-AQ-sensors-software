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

/*****************************************************************
 * read GPS sensor values                                        *
 *****************************************************************/

void fetchSensorGPS()
{
  if (gps.location.isUpdated()) {
		if (gps.location.isValid()) {
			last_value_GPS_lat = gps.location.lat();
			last_value_GPS_lon = gps.location.lng();
		} else {
			last_value_GPS_lat = -200;
			last_value_GPS_lon = -200;
			Serial.println(F("Lat/Lng INVALID"));
		}
		if (gps.altitude.isValid()) {
			last_value_GPS_alt = gps.altitude.meters();
			String gps_alt(last_value_GPS_lat);
		} else {
			last_value_GPS_alt = -1000;
			Serial.println(F("Altitude INVALID"));
		}
		if (gps.date.isValid()) {
			char gps_date[16];
			snprintf_P(gps_date, sizeof(gps_date), PSTR("%02d/%02d/%04d"),
					gps.date.month(), gps.date.day(), gps.date.year());
			last_value_GPS_date = gps_date;
		} else {
			Serial.println(F("Date INVALID"));
		}
		if (gps.time.isValid()) {
			char gps_time[20];
			snprintf_P(gps_time, sizeof(gps_time), PSTR("%02d:%02d:%02d.%02d"),
				gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
			last_value_GPS_time = gps_time;
		} else {
			Serial.println(F("Time: INVALID"));
		}
	}

	Serial.print("LOCATION ");
	Serial.print(last_value_GPS_lat);
	Serial.print(" , ");
	Serial.print(last_value_GPS_lon);
	Serial.print(" , ");
	Serial.println(last_value_GPS_alt);
	Serial.print("Date ");
	Serial.print(last_value_GPS_date);
	Serial.print(" TIME ");
	Serial.println(last_value_GPS_time);

}


void setup() {

  Serial.begin(9600);
  serialGPS.begin(9600); //set GPS baudrate
  dht.begin();

  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());

}

void loop() {
	while (serialGPS.available() > 0){
		if (gps.encode(serialGPS.read())){
			fetchSensorGPS();
		}
	}

 	 if (millis() > 5000 && gps.charsProcessed() < 10) {
		  Serial.println(F("No GPS detected: check wiring."));
		  while(true);
		  }

  fetchSensorDHT(dht_temperature,dht_humidity);

  Serial.print("TEMPERATURE : ");
  Serial.print(dht_temperature);
  Serial.print(" HUMIDITY : ");
  Serial.println(dht_humidity);

}