#include <Arduino.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include "TinyGPS++.h"

#define GPS_SAMPLE_DURATION 1000

#define ONEWIRE_PIN 2
#define DHT_TYPE DHT22

/*****************************************************************
 *  GPS PINS
 ****************************************************************/
#define GPS_RX_PIN 6
#define GPS_TX_PIN 7

/****************************************************************
 * NODEMCU PINS
 * **************************************************************/
#define NodeMCU_RX 11
#define NodeMCU_TX  10

/*****************************************************************
 * DHT declaration                                               *
 *****************************************************************/
DHT dht(ONEWIRE_PIN, DHT_TYPE);

/******************************************************************
 *  NodeMCU SoftwareSerial DECLARATION
 * ****************************************************************/
SoftwareSerial NodeMCU(NodeMCU_TX,NodeMCU_RX);

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
bool read_dht = false;

/******************************************************************
 *  GPS VARIABLES
 * ****************************************************************/
double last_value_GPS_lat = -200.0;
double last_value_GPS_lon = -200.0;
double last_value_GPS_alt = -1000.0;
String last_value_GPS_date;
String last_value_GPS_time;

bool read_gps = false;
uint64_t gps_time = 0;

/*****************************************************************
 * Payload variables
 * **************************************************************/
String received_command;
uint32_t payload_time = 0;



/****************************************************************
 * SEND SERIAL DATA
 * **************************************************************/
void send_serial_data(String data){
	NodeMCU.println(data);
}


/***************************************************************
 * Package DHT Sensor Values for transmission
 * *************************************************************/
String package_DHT_payload(){
  String dht_data;
  dht_data = "DHT#"+String(dht_temperature)+","+String(dht_humidity)+"*";
  return dht_data;
}


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

	String dht_values = package_DHT_payload();
	send_serial_data(dht_values);
	read_dht = false;

}

/***************************************************************
 * Package  GPS Sensor Values for transmission
 * *************************************************************/
String package_GPS_payload(){
  String gps_data;
  gps_data ="GPS#"+String(last_value_GPS_lat,6)+","+String(last_value_GPS_lon,6)+","+String(last_value_GPS_alt)+","+last_value_GPS_date+","+last_value_GPS_time+"*&";
  return gps_data;
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


		String gps_values = package_GPS_payload();
		send_serial_data(gps_values);
		read_gps = false;
	                                            
}





void setup() {
	
	pinMode(NodeMCU_RX, OUTPUT);
	pinMode(GPS_RX_PIN, OUTPUT);
	
	pinMode(NodeMCU_TX, INPUT);
	pinMode(GPS_TX_PIN, INPUT);
	
	
	Serial.begin(9600);
	NodeMCU.begin(9600);
	serialGPS.begin(9600); //set GPS baudrate
	dht.begin();
	
	Serial.print(F("Testing TinyGPS++ library v. "));
  	Serial.println(TinyGPSPlus::libraryVersion());

}

void loop() {

	NodeMCU.listen();
	delay(50);

	while(NodeMCU.available() > 0){
		received_command = NodeMCU.readString();
		Serial.println(received_command);
			if(received_command.indexOf("fetchSensorGPS") >= 0){
					read_gps = true;
			}
			if(received_command.indexOf("fetchSensorDHT") >= 0){
					read_dht = true;
			}
		}



	if(read_gps){

		serialGPS.listen();
		delay(50);
		gps_time = millis();
		
		while((millis() - gps_time) < GPS_SAMPLE_DURATION){
			while (serialGPS.available() > 0){
				gps.encode(serialGPS.read());
			}
		}
		
		fetchSensorGPS();
	}

	if(read_dht){
		
		fetchSensorDHT(dht_temperature,dht_humidity);
		
	}


 

 	Serial.print("TEMPERATURE : ");
  	Serial.print(dht_temperature);
  	Serial.print(" HUMIDITY : ");
  	Serial.println(dht_humidity);

  

}