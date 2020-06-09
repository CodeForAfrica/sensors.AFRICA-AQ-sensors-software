#include <Arduino.h>
#include "DHT.h"

#define ONEWIRE_PIN 2
#define DHT_TYPE DHT22

/*****************************************************************
 * DHT declaration                                               *
 *****************************************************************/
DHT dht(ONEWIRE_PIN, DHT_TYPE);

/******************************************************************
 *  DHT VARIABLES
 * ***************************************************************/
float dht_humidity = 0.0;
float dht_temperature = 0.0;

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
  dht.begin();
  
}

void loop() {

  fetchSensorDHT(dht_temperature,dht_humidity);

  Serial.print("TEMPERATURE : ");
  Serial.print(dht_temperature);
  Serial.print(" HUMIDITY : ");
  Serial.println(dht_humidity);

}