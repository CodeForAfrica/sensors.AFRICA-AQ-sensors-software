/************************************************************************
 *                                                                      *
 *  This source code needs to be compiled for the board                 *
 *  NodeMCU 1.0 (ESP-12E Module)                                        *
 *                                                                      *
 ************************************************************************
 *                                                                      *
 *    airRohr firmware                                                  *
 *    Copyright (C) 2016-2020  Code for Stuttgart a.o.                  *
 *    Copyright (C) 2019-2020  Dirk Mueller                             *
 *                                                                      *
 * This program is free software: you can redistribute it and/or modify *
 * it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or    *
 * (at your option) any later version.                                  *
 *                                                                      *
 * This program is distributed in the hope that it will be useful,      *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 * GNU General Public License for more details.                         *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 *                                                                      *
 ************************************************************************
 * OK LAB Particulate Matter Sensor                                     *
 *      - nodemcu-LoLin board                                           *
 *      - Nova SDS0111                                                  *
 *  http://inovafitness.com/en/Laser-PM2-5-Sensor-SDS011-35.html        *
 *                                                                      *
 * Wiring Instruction see included Readme.md                            *
 *                                                                      *
 ************************************************************************
 *                                                                      *
 * Alternative                                                          *
 *      - nodemcu-LoLin board                                           *
 *                                                                      *
 * Wiring Instruction:                                                  *
 *      Pin 2 of dust sensor PM2.5 -> Digital 6 (PWM)                   *
 *      Pin 3 of dust sensor       -> +5V                               *
 *      Pin 4 of dust sensor PM1   -> Digital 3 (PMW)                   *
 *                                                                      *
 *                                                                      *
 ************************************************************************
 *                                                                      *
 * Please check Readme.md for other sensors and hardware                *
 *                                                                      *
 ************************************************************************
 *
 * latest mit lib 2.6.1
 * DATA:    [====      ]  40.7% (used 33316 bytes from 81920 bytes)
 * PROGRAM: [=====     ]  49.3% (used 514788 bytes from 1044464 bytes)

 * latest mit lib 2.5.2
 * DATA:    [====      ]  39.4% (used 32304 bytes from 81920 bytes)
 * PROGRAM: [=====     ]  48.3% (used 504812 bytes from 1044464 bytes)
 *
 ************************************************************************/
#include <WString.h>
#include <pgmspace.h>

// increment on change
#define SOFTWARE_VERSION_STR "NRZ-2020-129"
String SOFTWARE_VERSION(SOFTWARE_VERSION_STR);

/*****************************************************************
 * Includes                                                      *
 *****************************************************************/

#if defined(ESP8266)
#include <FS.h> // must be first
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#include <Hash.h>
#include <ctime>
#include <coredecls.h>
#include <sntp.h>
#endif

#if defined(ESP32)
#define FORMAT_SPIFFS_IF_FAILED true
#include <FS.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <HardwareSerial.h>
#include <hwcrypto/sha.h>
#include <WebServer.h>
#endif

// includes common to ESP8266 and ESP32 (especially external libraries)
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <SPI.h>
#include <StreamString.h>
#include <Adafruit_FONA.h>

// display funtion declarations
static void init_display();
static void init_lcd();
static void display_debug(const String &text1, const String &text2);
static String check_display_value(double value, double undef, uint8_t len, uint8_t str_len);
static void display_values();
static String displayGenerateFooter(unsigned int screen_count);

// other function declaritions
static void writeConfig();
static String delayToString(unsigned time_ms);
static void sensor_restart();
static String SDS_version_date();
static void add_Value2Json(String &res, const __FlashStringHelper *type, const String &value);
static void add_Value2Json(String &res, const __FlashStringHelper *type, const __FlashStringHelper *debug_type, const float &value);
void switch_status_LEDs_on(uint8_t LED, bool on_state);
void switch_status_LEDs_off(uint8_t LED, bool off_state);
static void yield_for_serial_buffer(size_t length);
static String sha1Hex(const String &s);
static String hmac1(const String &secret, const String &s);

/******************************************************************
 * Constants                                                      *
 ******************************************************************/
constexpr unsigned SMALL_STR = 64 - 1;
constexpr unsigned MED_STR = 256 - 1;
constexpr unsigned LARGE_STR = 512 - 1;
constexpr unsigned XLARGE_STR = 1024 - 1;

#define RESERVE_STRING(name, size)      \
	String name((const char *)nullptr); \
	name.reserve(size)

const unsigned long SAMPLETIME_MS = 30000;	   // time between two measurements of the PPD42NS
const unsigned long SAMPLETIME_SDS_MS = 1000;  // time between two measurements of the SDS011, PMSx003, Honeywell PM sensor
const unsigned long WARMUPTIME_SDS_MS = 15000; // time needed to "warm up" the sensor before we can take the first measurement
const unsigned long READINGTIME_SDS_MS = 5000; // how long we read data from the PM sensors
const unsigned long SAMPLETIME_GPS_MS = 50;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 5000; // time between switching display to next "screen"
const unsigned long ONE_DAY_IN_MS = 24 * 60 * 60 * 1000;
const unsigned long PAUSE_BETWEEN_UPDATE_ATTEMPTS_MS = ONE_DAY_IN_MS;		// check for firmware updates once a day
const unsigned long DURATION_BEFORE_FORCED_RESTART_MS = ONE_DAY_IN_MS * 28; // force a reboot every ~4 weeks

#include "namespace_cfg.h" // include namespace cfg

#define JSON_BUFFER_SIZE 2300

LoggerConfig loggerConfigs[LoggerCount];

long int sample_count = 0;
bool htu21d_init_failed = false;
bool bmp_init_failed = false;
bool bmx280_init_failed = false;
bool sht3x_init_failed = false;
bool dnms_init_failed = false;
bool gps_init_failed = false;
bool airrohr_selftest_failed = false;

#include "defines.h"
#include "./airrohr-cfg.h"

/*****************************************************************
 * SDS011 declarations                                           *
 *****************************************************************/
#if defined(ESP8266)
SoftwareSerial serialSDS;
SoftwareSerial *serialGPS;
#endif
#if defined(ESP32)
#define serialSDS (Serial1)
#define serialGPS (&(Serial2))
#endif

/*****************************************************************
 * Variables for Noise Measurement DNMS                          *
 *****************************************************************/
float last_value_dnms_laeq = -1.0;
float last_value_dnms_la_min = -1.0;
float last_value_dnms_la_max = -1.0;

/*****************************************************************
 * Variable Definitions for PPD24NS                              *
 * P1 for PM10 & P2 for PM25                                     *
 *****************************************************************/

/*****************************************************************
/* GSM declaration                                               *
/*****************************************************************/
#if defined(ESP8266)
#include "GSM_handler.h"
#endif

boolean trigP1 = false;
boolean trigP2 = false;
unsigned long trigOnP1;
unsigned long trigOnP2;

unsigned long lowpulseoccupancyP1 = 0;
unsigned long lowpulseoccupancyP2 = 0;

bool send_now = false;
unsigned long starttime;
unsigned long time_point_device_start_ms;
unsigned long starttime_SDS;
unsigned long starttime_GPS;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;

bool is_SDS_running = true;
bool is_PMS_running = true;
bool is_HPM_running = true;

unsigned long sending_time = 0;
unsigned long last_update_attempt;
int last_update_returncode;
int last_sendData_returncode;

float last_value_BMP_T = -128.0;
float last_value_BMP_P = -1.0;
float last_value_BMX280_T = -128.0;
float last_value_BMX280_P = -1.0;
float last_value_BME280_H = -1.0;
float last_value_DHT_T = -128.0;
float last_value_DHT_H = -1.0;
float last_value_DS18B20_T = -1.0;
float last_value_HTU21D_T = -128.0;
float last_value_HTU21D_H = -1.0;
float last_value_SHT3X_T = -128.0;
float last_value_SHT3X_H = -1.0;

uint32_t sds_pm10_sum = 0;
uint32_t sds_pm25_sum = 0;
uint32_t sds_val_count = 0;
uint32_t sds_pm10_max = 0;
uint32_t sds_pm10_min = 20000;
uint32_t sds_pm25_max = 0;
uint32_t sds_pm25_min = 20000;

int pms_pm1_sum = 0;
int pms_pm10_sum = 0;
int pms_pm25_sum = 0;
int pms_val_count = 0;
int pms_pm1_max = 0;
int pms_pm1_min = 20000;
int pms_pm10_max = 0;
int pms_pm10_min = 20000;
int pms_pm25_max = 0;
int pms_pm25_min = 20000;

int hpm_pm10_sum = 0;
int hpm_pm25_sum = 0;
int hpm_val_count = 0;
int hpm_pm10_max = 0;
int hpm_pm10_min = 20000;
int hpm_pm25_max = 0;
int hpm_pm25_min = 20000;

float last_value_SPS30_P0 = -1.0;
float last_value_SPS30_P1 = -1.0;
float last_value_SPS30_P2 = -1.0;
float last_value_SPS30_P4 = -1.0;
float last_value_SPS30_N05 = -1.0;
float last_value_SPS30_N1 = -1.0;
float last_value_SPS30_N25 = -1.0;
float last_value_SPS30_N4 = -1.0;
float last_value_SPS30_N10 = -1.0;
float last_value_SPS30_TS = -1.0;
float value_SPS30_P0 = 0.0;
float value_SPS30_P1 = 0.0;
float value_SPS30_P2 = 0.0;
float value_SPS30_P4 = 0.0;
float value_SPS30_N05 = 0.0;
float value_SPS30_N1 = 0.0;
float value_SPS30_N25 = 0.0;
float value_SPS30_N4 = 0.0;
float value_SPS30_N10 = 0.0;
float value_SPS30_TS = 0.0;

uint16_t SPS30_measurement_count = 0;
unsigned long SPS30_read_counter = 0;
unsigned long SPS30_read_error_counter = 0;
unsigned long SPS30_read_timer = 0;
bool sps30_init_failed = false;

float last_value_PPD_P1 = -1.0;
float last_value_PPD_P2 = -1.0;
float last_value_SDS_P1 = -1.0;
float last_value_SDS_P2 = -1.0;
float last_value_PMS_P0 = -1.0;
float last_value_PMS_P1 = -1.0;
float last_value_PMS_P2 = -1.0;
float last_value_HPM_P1 = -1.0;
float last_value_HPM_P2 = -1.0;
double last_value_GPS_lat = -200.0;
double last_value_GPS_lon = -200.0;
double last_value_GPS_alt = -1000.0;
String last_value_GPS_date;
String last_value_GPS_time;
String last_value_GPS_timestamp;
String last_data_string;
int last_signal_strength;

String esp_chipid;
String last_value_SDS_version;

unsigned long SDS_error_count;
unsigned long sendData_error_count;
unsigned long WiFi_error_count;

unsigned long last_page_load = millis();

bool wificonfig_loop = false;

uint8_t sntp_time_set;

unsigned long count_sends = 0;
unsigned long last_display_millis = 0;
uint8_t next_display_count = 0;

struct struct_wifiInfo
{
	char ssid[LEN_WLANSSID];
	uint8_t encryptionType;
	int32_t RSSI;
	int32_t channel;
#if defined(ESP8266)
	bool isHidden;
	uint8_t unused[3];
#endif
};

struct struct_wifiInfo *wifiInfo;
uint8_t count_wifiInfo;

template <typename T, std::size_t N>
constexpr std::size_t array_num_elements(const T (&)[N])
{
	return N;
}

#define msSince(timestamp_before) (act_milli - (timestamp_before))

const char data_first_part[] PROGMEM = "{\"software_version\": \"" SOFTWARE_VERSION_STR "\", \"sensordatavalues\":[";
const char JSON_SENSOR_DATA_VALUES[] PROGMEM = "sensordatavalues";

/****************************************
 * Refactored / to be refactored files
 ****************************************/
#include "ext_def.h"
#include "utils/_debug_helper.h"
#include "webserver/webserver.h"
#include "ca-root.h"
#include "displays/displays.h"
#include "utils/wifi_config.h"
#include "utils/_network_time.h"
#include "OTA.h"
// Sensors
#include "./sensors/Noise/DNMS.h"
#include "./sensors/Temperature_Humidity/DHT/DHT_func.h"
#include "./sensors/Temperature_Humidity/HTU21DF/HTU21DF.h"
#include "./sensors/Temperature_Humidity/SHT3X/SHT31.h"
#include "./sensors/Temperature_Humidity/Dallas/ds18b20.h"
#include "./sensors/Pressure/BMP085.h"
#include "./sensors/Pressure/BMX280.h"
#include "./sensors/GPS/gps.h"
#include "./sensors/PM/SPS30/sps30.h"
#include "./sensors/PM/pm_utils.h"
#include "./utils/send_data.h"
#include "./utils/_spiffs_helper.h"
#include "./utils/_loggers.h"

/*****************************************************************
 * add value to json string                                  *
 *****************************************************************/
static void add_Value2Json(String &res, const __FlashStringHelper *type, const String &value)
{
	RESERVE_STRING(s, SMALL_STR);

	s = F("{\"value_type\":\"{t}\",\"value\":\"{v}\"},");
	s.replace("{t}", String(type));
	s.replace("{v}", value);
	res += s;
}

static void add_Value2Json(String &res, const __FlashStringHelper *type, const __FlashStringHelper *debug_type, const float &value)
{
	debug_outln_info(FPSTR(debug_type), value);
	add_Value2Json(res, type, String(value));
}

// workaround for https://github.com/plerup/espsoftwareserial/issues/127
static void yield_for_serial_buffer(size_t length)
{
	unsigned long startMillis = millis();
	unsigned long yield_timeout = length * 9 * 1000 / 9600;

	while (serialSDS.available() < (int)length &&
		   millis() - startMillis < yield_timeout)
	{
		yield();
		serialSDS.perform_work();
	}
}

/*****************************************************************
 * Prepare information for data Loggers                          *
 *****************************************************************/
static void createLoggerConfigs()
{
#if defined(ESP8266)
	auto new_session = []()
	{ return new BearSSL::Session; };
#else
	auto new_session = []()
	{ return nullptr; };
#endif
	if (cfg::send2cfa)
	{
		loggerConfigs[LoggerCFA].destport = 80;
		if (cfg::ssl_cfa)
		{
			loggerConfigs[LoggerCFA].destport = 80;
			loggerConfigs[LoggerCFA].session = new_session();
		}
	}
	if (cfg::send2dusti)
	{
		loggerConfigs[LoggerSensorCommunity].destport = 80;
		if (cfg::ssl_dusti)
		{
			loggerConfigs[LoggerSensorCommunity].destport = 443;
			loggerConfigs[LoggerSensorCommunity].session = new_session();
		}
	}
	loggerConfigs[LoggerMadavi].destport = PORT_MADAVI;
	if (cfg::send2madavi && cfg::ssl_madavi)
	{
		loggerConfigs[LoggerMadavi].destport = 443;
		loggerConfigs[LoggerMadavi].session = new_session();
	}
	loggerConfigs[LoggerSensemap].destport = PORT_SENSEMAP;
	loggerConfigs[LoggerSensemap].session = new_session();
	loggerConfigs[LoggerFSapp].destport = PORT_FSAPP;
	loggerConfigs[LoggerFSapp].session = new_session();
	loggerConfigs[Loggeraircms].destport = PORT_AIRCMS;
	loggerConfigs[LoggerInflux].destport = cfg::port_influx;
	if (cfg::send2influx && cfg::ssl_influx)
	{
		loggerConfigs[LoggerInflux].session = new_session();
	}
	loggerConfigs[LoggerCustom].destport = cfg::port_custom;
	if (cfg::send2custom && (cfg::ssl_custom || (cfg::port_custom == 443)))
	{
		loggerConfigs[LoggerCustom].session = new_session();
	}
}

static void sensor_restart()
{
#if defined(ESP8266)
	WiFi.disconnect();
	WiFi.mode(WIFI_OFF);
	delay(100);
#endif
	SPIFFS.end();
	serialSDS.end();
	debug_outln_info(F("Restart."));
	delay(500);
	ESP.restart();
	// should not be reached
	while (true)
	{
		yield();
	}
}

static String delayToString(unsigned time_ms)
{

	char buf[64];
	String s;

	if (time_ms > 2 * 1000 * 60 * 60 * 24)
	{
		sprintf_P(buf, PSTR("%d days, "), time_ms / (1000 * 60 * 60 * 24));
		s += buf;
		time_ms %= 1000 * 60 * 60 * 24;
	}

	if (time_ms > 2 * 1000 * 60 * 60)
	{
		sprintf_P(buf, PSTR("%d hours, "), time_ms / (1000 * 60 * 60));
		s += buf;
		time_ms %= 1000 * 60 * 60;
	}

	if (time_ms > 2 * 1000 * 60)
	{
		sprintf_P(buf, PSTR("%d min, "), time_ms / (1000 * 60));
		s += buf;
		time_ms %= 1000 * 60;
	}

	if (time_ms > 2 * 1000)
	{
		sprintf_P(buf, PSTR("%ds, "), time_ms / 1000);
		s += buf;
	}

	if (s.length() > 2)
	{
		s = s.substring(0, s.length() - 2);
	}

	return s;
}

/*****************************************************************
 * Switch status LEDs on/ off                                    *
 *****************************************************************/
void switch_status_LEDs_on(uint8_t LED, bool on_state)
{
	digitalWrite(LED, HIGH);
}

void switch_status_LEDs_off(uint8_t LED, bool off_state)
{
	digitalWrite(LED, LOW);
}

static void powerOnTestSensors()
{
	if (cfg::ppd_read)
	{
		pinMode(PPD_PIN_PM1, INPUT_PULLUP); // Listen at the designated PIN
		pinMode(PPD_PIN_PM2, INPUT_PULLUP); // Listen at the designated PIN
		debug_outln_info(F("Read PPD..."));
	}

	if (cfg::sds_read)
	{
		debug_outln_info(F("Read SDS...: "), SDS_version_date());
		SDS_cmd(PmSensorCmd::ContinuousMode);
		delay(100);
		debug_outln_info(F("Stopping SDS011..."));
		is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
	}

	if (cfg::pms_read)
	{
		debug_outln_info(F("Read PMS(1,3,5,6,7)003..."));
		PMS_cmd(PmSensorCmd::Start);
		delay(100);
		PMS_cmd(PmSensorCmd::ContinuousMode);
		delay(100);
		debug_outln_info(F("Stopping PMS..."));
		is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
	}

	if (cfg::hpm_read)
	{
		debug_outln_info(F("Read HPM..."));
		HPM_cmd(PmSensorCmd::Start);
		delay(100);
		HPM_cmd(PmSensorCmd::ContinuousMode);
		delay(100);
		debug_outln_info(F("Stopping HPM..."));
		is_HPM_running = HPM_cmd(PmSensorCmd::Stop);
	}

	if (cfg::sps30_read)
	{
		debug_outln_info(F("Read SPS30..."));
		initSPS30();
	}

	if (cfg::dht_read)
	{
		dht.begin(); // Start DHT
		debug_outln_info(F("Read DHT..."));
	}

	if (cfg::htu21d_read)
	{
		debug_outln_info(F("Read HTU21D..."));
		// begin() might return false when using Si7021
		// so validate reading via Humidity (will return 0.0 when failed)
		if (!htu21d.begin() && htu21d.readHumidity() < 1.0f)
		{
			debug_outln_error(F("Check HTU21D wiring"));
			htu21d_init_failed = true;
		}
	}

	if (cfg::bmp_read)
	{
		debug_outln_info(F("Read BMP..."));
		if (!bmp.begin())
		{
			debug_outln_error(F("No valid BMP085 sensor, check wiring!"));
			bmp_init_failed = true;
		}
	}

	if (cfg::bmx280_read)
	{
		debug_outln_info(F("Read BMP280/BME280..."));
		if (!initBMX280(0x76) && !initBMX280(0x77))
		{
			debug_outln_error(F("Check BMP280/BME280 wiring"));
			bmx280_init_failed = true;
		}
	}

	if (cfg::sht3x_read)
	{
		debug_outln_info(F("Read SHT3x..."));
		if (!sht3x.begin())
		{
			debug_outln_error(F("Check SHT3x wiring"));
			sht3x_init_failed = true;
		}
	}

	if (cfg::ds18b20_read)
	{
		ds18b20.begin(); // Start DS18B20
		debug_outln_info(F("Read DS18B20..."));
	}

	if (cfg::dnms_read)
	{
		debug_outln_info(F("Read DNMS..."));
		initDNMS();
	}
}

/*****************************************************************
 * The Setup                                                     *
 *****************************************************************/

void setup(void)
{

	Serial.begin(9600); // Output to Serial at 9600 baud

#if defined(ESP8266)
	serialSDS.begin(9600, SWSERIAL_8N1, PM_SERIAL_RX, PM_SERIAL_TX);
#endif
#if defined(ESP32)
	serialSDS.begin(9600, SERIAL_8N1, PM_SERIAL_RX, PM_SERIAL_TX);
#endif
	serialSDS.enableIntTx(true);
	serialSDS.setTimeout((12 * 9 * 1000) / 9600);

#if defined(WIFI_LoRa_32_V2)
	// reset the OLED display, e.g. of the heltec_wifi_lora_32 board
	pinMode(RST_OLED, OUTPUT);
	digitalWrite(RST_OLED, LOW);
	delay(50);
	digitalWrite(RST_OLED, HIGH);
#endif
	Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);
	pinMode(PMS_LED, OUTPUT);
	pinMode(DHT_LED, OUTPUT);

#if defined(ESP8266)
	esp_chipid = std::move(String(ESP.getChipId()));
#endif
#if defined(ESP32)
	uint64_t chipid_num;
	chipid_num = ESP.getEfuseMac();
	esp_chipid = String((uint16_t)(chipid_num >> 32), HEX);
	esp_chipid += String((uint32_t)chipid_num, HEX);
#endif
	cfg::initNonTrivials(esp_chipid.c_str());
	WiFi.persistent(false);

	debug_outln_info(F("airRohr: " SOFTWARE_VERSION_STR "/"), String(CURRENT_LANG));
	if ((airrohr_selftest_failed = !ESP.checkFlashConfig(true) /* after 2.7.0 update: || !ESP.checkFlashCRC() */))
	{
		debug_outln_error(F("ERROR: SELF TEST FAILED!"));
		SOFTWARE_VERSION += F("-STF");
	}
	logEnabledAPIs();
	logEnabledDisplays();
	init_config();
	init_display();
	init_lcd();
	setup_webserver();
	createLoggerConfigs();
	debug_outln_info(F("\nChipId: "), esp_chipid);

	if (cfg::gsm_capable)
	{
		is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		Serial.println("Attempting to setup GSM connection");
		if (!GSM_init(fonaSerial))
		{
			Serial.println("GSM not fully configured");
			Serial.print("Failure point: ");
			Serial.println(GSM_INIT_ERROR);
			Serial.println();
		}
	}
	if (!GPRS_CONNECTED)
	{
		connectWifi();
	}
	if (cfg::gps_read)
	{
#if defined(ESP8266)
		serialGPS = new SoftwareSerial;
		serialGPS->begin(9600, SWSERIAL_8N1, GPS_SERIAL_RX, GPS_SERIAL_TX, false, 128);
#endif
#if defined(ESP32)
		serialGPS->begin(9600, SERIAL_8N1, GPS_SERIAL_RX, GPS_SERIAL_TX);
#endif
		debug_outln_info(F("Read GPS..."));
		disable_unneeded_nmea();
	}

	setupNetworkTime();

	powerOnTestSensors();
	// logEnabledAPIs();
	// logEnabledDisplays();

	delay(50);

	// sometimes parallel sending data and web page will stop nodemcu, watchdogtimer set to 120 seconds
#if defined(ESP8266)
	wdt_disable();
#if defined(NDEBUG)
	wdt_enable(120000);
#endif
#endif

	starttime = millis(); // store the start time
	last_update_attempt = time_point_device_start_ms = starttime;
	last_display_millis = starttime_SDS = starttime;
}

/*****************************************************************
 * And action                                                    *
 *****************************************************************/
void loop(void)
{
	String result_PPD, result_SDS, result_PMS, result_HPM;
	String result_GPS, result_DNMS;

	unsigned sum_send_time = 0;

	act_micro = micros();
	act_milli = millis();
	send_now = msSince(starttime) > cfg::sending_intervall_ms;
	// Wait at least 30s for each NTP server to sync

	if (!sntp_time_set && send_now &&
		msSince(time_point_device_start_ms) < 1000 * 2 * 30 + 5000)
	{
		debug_outln_info(F("NTP sync not finished yet, skipping send"));
		send_now = false;
		starttime = act_milli;
	}

	sample_count++;

#if defined(ESP8266)
	wdt_reset(); // nodemcu is alive
#endif

	if (last_micro != 0)
	{
		unsigned long diff_micro = act_micro - last_micro;
		if (max_micro < diff_micro)
		{
			max_micro = diff_micro;
		}
		if (min_micro > diff_micro)
		{
			min_micro = diff_micro;
		}
	}
	last_micro = act_micro;

	if (msSince(time_point_device_start_ms) > DURATION_BEFORE_FORCED_RESTART_MS)
	{
		sensor_restart();
	}

	if (msSince(last_update_attempt) > PAUSE_BETWEEN_UPDATE_ATTEMPTS_MS)
	{
		twoStageOTAUpdate();
		last_update_attempt = act_milli;
	}

	if (cfg::sps30_read && (!sps30_init_failed))
	{
		if ((msSince(starttime) - SPS30_read_timer) > SPS30_WAITING_AFTER_LAST_READ)
		{
			struct sps30_measurement sps30_values;
			int16_t ret_SPS30;

			SPS30_read_timer = msSince(starttime);

			ret_SPS30 = sps30_read_measurement(&sps30_values);
			++SPS30_read_counter;
			if (ret_SPS30 < 0)
			{
				debug_outln_info(F("SPS30 error reading measurement"));
				SPS30_read_error_counter++;
			}
			else
			{
				if (SPS_IS_ERR_STATE(ret_SPS30))
				{
					debug_outln_info(F("SPS30 measurements may not be accurate"));
					SPS30_read_error_counter++;
				}
				value_SPS30_P0 += sps30_values.mc_1p0;
				value_SPS30_P2 += sps30_values.mc_2p5;
				value_SPS30_P4 += sps30_values.mc_4p0;
				value_SPS30_P1 += sps30_values.mc_10p0;
				value_SPS30_N05 += sps30_values.nc_0p5;
				value_SPS30_N1 += sps30_values.nc_1p0;
				value_SPS30_N25 += sps30_values.nc_2p5;
				value_SPS30_N4 += sps30_values.nc_4p0;
				value_SPS30_N10 += sps30_values.nc_10p0;
				value_SPS30_TS += sps30_values.tps;
				++SPS30_measurement_count;
			}
		}
	}

	if (cfg::ppd_read)
	{
		fetchSensorPPD(result_PPD);
	}

	if ((msSince(starttime_SDS) > SAMPLETIME_SDS_MS) || send_now)
	{
		starttime_SDS = act_milli;
		if (cfg::sds_read)
		{
			fetchSensorSDS(result_SDS);
		}

		if (cfg::pms_read)
		{
			fetchSensorPMS(result_PMS);
		}

		if (cfg::hpm_read)
		{
			fetchSensorHPM(result_HPM);
		}
	}

	if (cfg::gps_read && !gps_init_failed)
	{
		// process serial GPS data..
		while (serialGPS->available() > 0)
		{
			gps.encode(serialGPS->read());
		}

		if ((msSince(starttime_GPS) > SAMPLETIME_GPS_MS) || send_now)
		{
			// getting GPS coordinates
			fetchSensorGPS(result_GPS);
			starttime_GPS = act_milli;
		}
	}

	if ((msSince(last_display_millis) > DISPLAY_UPDATE_INTERVAL_MS) &&
		(cfg::has_display || cfg::has_sh1106 || lcd_1602 || lcd_2004))
	{
		display_values();
		last_display_millis = act_milli;
	}

	server.handleClient();
	yield();
	if (firmware_bin_saved)
	{
		Serial.println("Beginning firmware update from webserver upload...");
		firmware_update();
	}

	if (send_now)
	{
		last_signal_strength = WiFi.RSSI();
		RESERVE_STRING(data, LARGE_STR);
		data = FPSTR(data_first_part);
		RESERVE_STRING(result, MED_STR);

		if (cfg::ppd_read)
		{
			data += result_PPD;
			sum_send_time += sendCFA(result_PPD, PPD_API_PIN, FPSTR(SENSORS_PPD42NS), "PPD_");
			sum_send_time += sendSensorCommunity(result_PPD, PPD_API_PIN, FPSTR(SENSORS_PPD42NS), "PPD_");
		}
		if (cfg::sds_read)
		{
			data += result_SDS;
			sum_send_time += sendCFA(result_SDS, SDS_API_PIN, FPSTR(SENSORS_SDS011), "SDS_");
			sum_send_time += sendSensorCommunity(result_SDS, SDS_API_PIN, FPSTR(SENSORS_SDS011), "SDS_");
		}
		if (cfg::pms_read)
		{
			data += result_PMS;
			sum_send_time += sendCFA(result_PMS, PMS_API_PIN, FPSTR(SENSORS_PMSx003), "PMS_");
			sum_send_time += sendSensorCommunity(result_PMS, PMS_API_PIN, FPSTR(SENSORS_PMSx003), "PMS_");
		}
		if (cfg::hpm_read)
		{
			data += result_HPM;
			sum_send_time += sendCFA(result_HPM, HPM_API_PIN, FPSTR(SENSORS_HPM), "HPM_");
			sum_send_time += sendSensorCommunity(result_HPM, HPM_API_PIN, FPSTR(SENSORS_HPM), "HPM_");
		}
		if (cfg::sps30_read && (!sps30_init_failed))
		{
			fetchSensorSPS30(result);
			data += result;
			sum_send_time += sendCFA(result, SPS30_API_PIN, FPSTR(SENSORS_SPS30), "SPS30_");
			sum_send_time += sendSensorCommunity(result, SPS30_API_PIN, FPSTR(SENSORS_SPS30), "SPS30_");
			result = emptyString;
		}
		if (cfg::dht_read)
		{
			// getting temperature and humidity (optional)
			fetchSensorDHT(result);
			data += result;
			sum_send_time += sendCFA(result, DHT_API_PIN, FPSTR(SENSORS_DHT22), "DHT_");
			sum_send_time += sendSensorCommunity(result, DHT_API_PIN, FPSTR(SENSORS_DHT22), "DHT_");
			result = emptyString;
		}
		if (cfg::htu21d_read && (!htu21d_init_failed))
		{
			// getting temperature and humidity (optional)
			fetchSensorHTU21D(result);
			data += result;
			sum_send_time += sendCFA(result, HTU21D_API_PIN, FPSTR(SENSORS_HTU21D), "HTU21D_");
			sum_send_time += sendSensorCommunity(result, HTU21D_API_PIN, FPSTR(SENSORS_HTU21D), "HTU21D_");
			result = emptyString;
		}
		if (cfg::bmp_read && (!bmp_init_failed))
		{
			// getting temperature and pressure (optional)
			fetchSensorBMP(result);
			data += result;
			sum_send_time += sendCFA(result, BMP_API_PIN, FPSTR(SENSORS_BMP180), "BMP_");
			sum_send_time += sendSensorCommunity(result, BMP_API_PIN, FPSTR(SENSORS_BMP180), "BMP_");
			result = emptyString;
		}
		if (cfg::bmx280_read && (!bmx280_init_failed))
		{
			// getting temperature, humidity and pressure (optional)
			fetchSensorBMX280(result);
			data += result;
			if (bmx280.sensorID() == BME280_SENSOR_ID)
			{
				sum_send_time += sendCFA(result, BME280_API_PIN, FPSTR(SENSORS_BMX280), "BME280_");
				sum_send_time += sendSensorCommunity(result, BME280_API_PIN, FPSTR(SENSORS_BMX280), "BME280_");
			}
			else
			{
				sum_send_time += sendCFA(result, BMP280_API_PIN, FPSTR(SENSORS_BMX280), "BMP280_");
				sum_send_time += sendSensorCommunity(result, BMP280_API_PIN, FPSTR(SENSORS_BMX280), "BMP280_");
			}
			result = emptyString;
		}
		if (cfg::sht3x_read && (!sht3x_init_failed))
		{
			// getting temperature and humidity (optional)
			fetchSensorSHT3x(result);
			data += result;
			sum_send_time += sendCFA(result, SHT3X_API_PIN, FPSTR(SENSORS_SHT3X), "SHT3X_");
			sum_send_time += sendSensorCommunity(result, SHT3X_API_PIN, FPSTR(SENSORS_SHT3X), "SHT3X_");
			result = emptyString;
		}
		if (cfg::ds18b20_read)
		{
			// getting temperature (optional)
			fetchSensorDS18B20(result);
			data += result;
			sum_send_time += sendCFA(result, DS18B20_API_PIN, FPSTR(SENSORS_DS18B20), "DS18B20_");
			sum_send_time += sendSensorCommunity(result, DS18B20_API_PIN, FPSTR(SENSORS_DS18B20), "DS18B20_");
			result = emptyString;
		}
		if (cfg::dnms_read && (!dnms_init_failed))
		{
			// getting noise measurement values from dnms (optional)
			fetchSensorDNMS(result);
			data += result;
			sum_send_time += sendCFA(result, DNMS_API_PIN, FPSTR(SENSORS_DNMS), "DNMS_");
			sum_send_time += sendSensorCommunity(result, DNMS_API_PIN, FPSTR(SENSORS_DNMS), "DNMS_");
			result = emptyString;
		}
		if (cfg::gps_read)
		{
			data += result_GPS;
			sum_send_time += sendCFA(result_GPS, GPS_API_PIN, F("GPS"), "GPS_");
			sum_send_time += sendSensorCommunity(result_GPS, GPS_API_PIN, F("GPS"), "GPS_");
			result = emptyString;
		}
		add_Value2Json(data, F("samples"), String(sample_count));
		add_Value2Json(data, F("min_micro"), String(min_micro));
		add_Value2Json(data, F("max_micro"), String(max_micro));
		add_Value2Json(data, F("signal"), String(last_signal_strength));

		if ((unsigned)(data.lastIndexOf(',') + 1) == data.length())
		{
			data.remove(data.length() - 1);
		}
		data += "]}";

		yield();

		sum_send_time += sendDataToOptionalApis(data);

		// https://en.wikipedia.org/wiki/Moving_average#Cumulative_moving_average
		sending_time = (3 * sending_time + sum_send_time) / 4;
		if (sum_send_time > 0)
		{
			debug_outln_info(F("Time for Sending (ms): "), String(sending_time));
		}

		// reconnect to WiFi if disconnected
		/*if (WiFi.status() != WL_CONNECTED) {
			debug_outln_info(F("Connection lost, reconnecting "));
			WiFi_error_count++;
			WiFi.reconnect();
			waitForWifiToConnect(20);
		}*/

		// Resetting for next sampling
		last_data_string = std::move(data);
		lowpulseoccupancyP1 = 0;
		lowpulseoccupancyP2 = 0;
		sample_count = 0;
		last_micro = 0;
		min_micro = 1000000000;
		max_micro = 0;
		sum_send_time = 0;
		starttime = millis(); // store the start time
		count_sends++;
	}
	yield();
#if defined(ESP8266)
	MDNS.update();
	serialSDS.perform_work();
#endif

	if (sample_count % 500 == 0)
	{
		//		Serial.println(ESP.getFreeHeap(),DEC);
	}
}
