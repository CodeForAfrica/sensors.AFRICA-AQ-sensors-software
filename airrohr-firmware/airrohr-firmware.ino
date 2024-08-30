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
#include "./DHT.h"
#include <SPI.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_SHT31.h>
#include <StreamString.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include "./bmx280_i2c.h"
#include "./sps30_i2c.h"
#include <Adafruit_FONA.h>

// display funtion declarations
static void display_debug(const String &text1, const String &text2);
static String check_display_value(double value, double undef, uint8_t len, uint8_t str_len);

// other function declaritions
static void writeConfig();
static String delayToString(unsigned time_ms);
static void sensor_restart();
static String SDS_version_date();
static void add_Value2Json(String &res, const __FlashStringHelper *type, const String &value);
static void add_Value2Json(String &res, const __FlashStringHelper *type, const __FlashStringHelper *debug_type, const float &value);
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

enum class PmSensorCmd
{
	Start,
	Stop,
	ContinuousMode
};

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
 * DHT declaration                                               *
 *****************************************************************/
DHT dht(ONEWIRE_PIN, DHT_TYPE);

/*****************************************************************
 * HTU21D declaration                                            *
 *****************************************************************/
Adafruit_HTU21DF htu21d;

/*****************************************************************
 * BMP declaration                                               *
 *****************************************************************/
Adafruit_BMP085 bmp;

/*****************************************************************
 * BMP/BME280 declaration                                        *
 *****************************************************************/
BMX280 bmx280;

/*****************************************************************
 * SHT3x declaration                                             *
 *****************************************************************/
Adafruit_SHT31 sht3x;

/*****************************************************************
 * DS18B20 declaration                                            *
 *****************************************************************/
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds18b20(&oneWire);

/*****************************************************************
 * GPS declaration                                               *
 *****************************************************************/
TinyGPSPlus gps;

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

/*****************************************************************
 * display values                                                *
 *****************************************************************/
static void display_debug(const String &text1, const String &text2)
{
	debug_outln_info(F("output debug text to displays..."));
	if (cfg::has_display)
	{
		display.clear();
		display.displayOn();
		display.setTextAlignment(TEXT_ALIGN_LEFT);
		display.drawString(0, 12, text1);
		display.drawString(0, 24, text2);
		display.display();
	}
	if (cfg::has_sh1106)
	{
		display_sh1106.clear();
		display_sh1106.displayOn();
		display_sh1106.setTextAlignment(TEXT_ALIGN_LEFT);
		display_sh1106.drawString(0, 12, text1);
		display_sh1106.drawString(0, 24, text2);
		display_sh1106.display();
	}
	if (lcd_1602)
	{
		lcd_1602->clear();
		lcd_1602->setCursor(0, 0);
		lcd_1602->print(text1);
		lcd_1602->setCursor(0, 1);
		lcd_1602->print(text2);
	}
	if (lcd_2004)
	{
		lcd_2004->clear();
		lcd_2004->setCursor(0, 0);
		lcd_2004->print(text1);
		lcd_2004->setCursor(0, 1);
		lcd_2004->print(text2);
	}
}

/*****************************************************************
 * check display values, return '-' if undefined                 *
 *****************************************************************/
static String check_display_value(double value, double undef, uint8_t len, uint8_t str_len)
{
	RESERVE_STRING(s, 15);
	s = (value != undef ? String(value, len) : String("-"));
	while (s.length() < str_len)
	{
		s = " " + s;
	}
	return s;
}

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

/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version    *
 *****************************************************************/

static bool SDS_checksum_valid(const uint8_t (&data)[8])
{
	uint8_t checksum_is = 0;
	for (unsigned i = 0; i < 6; ++i)
	{
		checksum_is += data[i];
	}
	return (data[7] == 0xAB && checksum_is == data[6]);
}

static void SDS_rawcmd(const uint8_t cmd_head1, const uint8_t cmd_head2, const uint8_t cmd_head3)
{
	constexpr uint8_t cmd_len = 19;

	uint8_t buf[cmd_len];
	buf[0] = 0xAA;
	buf[1] = 0xB4;
	buf[2] = cmd_head1;
	buf[3] = cmd_head2;
	buf[4] = cmd_head3;
	for (unsigned i = 5; i < 15; ++i)
	{
		buf[i] = 0x00;
	}
	buf[15] = 0xFF;
	buf[16] = 0xFF;
	buf[17] = cmd_head1 + cmd_head2 + cmd_head3 - 2;
	buf[18] = 0xAB;
	serialSDS.write(buf, cmd_len);
}

static bool SDS_cmd(PmSensorCmd cmd)
{
	switch (cmd)
	{
	case PmSensorCmd::Start:
		SDS_rawcmd(0x06, 0x01, 0x01);
		break;
	case PmSensorCmd::Stop:
		SDS_rawcmd(0x06, 0x01, 0x00);
		break;
	case PmSensorCmd::ContinuousMode:
		// TODO: Check mode first before (re-)setting it
		SDS_rawcmd(0x08, 0x01, 0x00);
		SDS_rawcmd(0x02, 0x01, 0x00);
		break;
	}

	return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * send Plantower PMS sensor command start, stop, cont. mode     *
 *****************************************************************/
static bool PMS_cmd(PmSensorCmd cmd)
{
	static constexpr uint8_t start_cmd[] PROGMEM = {
		0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
	static constexpr uint8_t stop_cmd[] PROGMEM = {
		0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
	static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
		0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
	constexpr uint8_t cmd_len = array_num_elements(start_cmd);

	uint8_t buf[cmd_len];
	switch (cmd)
	{
	case PmSensorCmd::Start:
		memcpy_P(buf, start_cmd, cmd_len);
		break;
	case PmSensorCmd::Stop:
		memcpy_P(buf, stop_cmd, cmd_len);
		break;
	case PmSensorCmd::ContinuousMode:
		memcpy_P(buf, continuous_mode_cmd, cmd_len);
		break;
	}
	serialSDS.write(buf, cmd_len);
	return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * send Honeywell PMS sensor command start, stop, cont. mode     *
 *****************************************************************/
static bool HPM_cmd(PmSensorCmd cmd)
{
	static constexpr uint8_t start_cmd[] PROGMEM = {
		0x68, 0x01, 0x01, 0x96};
	static constexpr uint8_t stop_cmd[] PROGMEM = {
		0x68, 0x01, 0x02, 0x95};
	static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
		0x68, 0x01, 0x40, 0x57};
	constexpr uint8_t cmd_len = array_num_elements(start_cmd);

	uint8_t buf[cmd_len];
	switch (cmd)
	{
	case PmSensorCmd::Start:
		memcpy_P(buf, start_cmd, cmd_len);
		break;
	case PmSensorCmd::Stop:
		memcpy_P(buf, stop_cmd, cmd_len);
		break;
	case PmSensorCmd::ContinuousMode:
		memcpy_P(buf, continuous_mode_cmd, cmd_len);
		break;
	}
	serialSDS.write(buf, cmd_len);
	return cmd != PmSensorCmd::Stop;
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
 * read SDS011 sensor serial and firmware date                   *
 *****************************************************************/
static String SDS_version_date()
{

	if (cfg::sds_read && !last_value_SDS_version.length())
	{
		debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(DBG_TXT_SDS011_VERSION_DATE));
		is_SDS_running = SDS_cmd(PmSensorCmd::Start);
		delay(250);
		serialSDS.perform_work();
		serialSDS.flush();
		// Query Version/Date
		SDS_rawcmd(0x07, 0x00, 0x00);
		delay(400);
		const constexpr uint8_t header_cmd_response[2] = {0xAA, 0xC5};
		while (serialSDS.find(header_cmd_response, sizeof(header_cmd_response)))
		{
			uint8_t data[8];
			yield_for_serial_buffer(sizeof(data));
			unsigned r = serialSDS.readBytes(data, sizeof(data));
			if (r == sizeof(data) && data[0] == 0x07 && SDS_checksum_valid(data))
			{
				char tmp[20];
				snprintf_P(tmp, sizeof(tmp), PSTR("%02d-%02d-%02d(%02x%02x)"),
						   data[1], data[2], data[3], data[4], data[5]);
				last_value_SDS_version = tmp;
				break;
			}
		}
		debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(DBG_TXT_SDS011_VERSION_DATE));
	}

	return last_value_SDS_version;
}

/*****************************************************************
 * disable unneeded NMEA sentences, TinyGPS++ needs GGA, RMC     *
 *****************************************************************/
static void disable_unneeded_nmea()
{
	serialGPS->println(F("$PUBX,40,GLL,0,0,0,0*5C")); // Geographic position, latitude / longitude
													  //	serialGPS->println(F("$PUBX,40,GGA,0,0,0,0*5A"));       // Global Positioning System Fix Data
	serialGPS->println(F("$PUBX,40,GSA,0,0,0,0*4E")); // GPS DOP and active satellites
													  //	serialGPS->println(F("$PUBX,40,RMC,0,0,0,0*47"));       // Recommended minimum specific GPS/Transit data
	serialGPS->println(F("$PUBX,40,GSV,0,0,0,0*59")); // GNSS satellites in view
	serialGPS->println(F("$PUBX,40,VTG,0,0,0,0*5E")); // Track made good and ground speed
}

/*****************************************************************
 * read config from spiffs                                       *
 *****************************************************************/

/* backward compatibility for the times when we stored booleans as strings */
static bool boolFromJSON(const DynamicJsonDocument &json, const __FlashStringHelper *key)
{
	if (json[key].is<char *>())
	{
		return !strcmp_P(json[key].as<char *>(), PSTR("true"));
	}
	return json[key].as<bool>();
}

static void readConfig(bool oldconfig = false)
{
	bool rewriteConfig = false;

	String cfgName(F("/config.json"));
	if (oldconfig)
	{
		cfgName += F(".old");
	}

	File configFile = SPIFFS.open(cfgName, "r");
	if (!configFile)
	{
		if (!oldconfig)
		{
			return readConfig(true /* oldconfig */);
		}

		debug_outln_error(F("failed to open config file."));
		return;
	}

	debug_outln_info(F("opened config file..."));
	DynamicJsonDocument json(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json, configFile);
	configFile.close();

	if (!err)
	{
		debug_outln_info(F("parsed json..."));
		for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
		{
			ConfigShapeEntry c;
			memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
			if (json[c.cfg_key].isNull())
			{
				continue;
			}
			switch (c.cfg_type)
			{
			case Config_Type_Bool:
				*(c.cfg_val.as_bool) = boolFromJSON(json, c.cfg_key);
				break;
			case Config_Type_UInt:
			case Config_Type_Time:
				*(c.cfg_val.as_uint) = json[c.cfg_key].as<unsigned int>();
				break;
			case Config_Type_String:
			case Config_Type_Password:
				strncpy(c.cfg_val.as_str, json[c.cfg_key].as<char *>(), c.cfg_len);
				c.cfg_val.as_str[c.cfg_len] = '\0';
				break;
			};
		}
		String writtenVersion(json["SOFTWARE_VERSION"].as<char *>());
		if (writtenVersion.length() && writtenVersion[0] == 'N' && SOFTWARE_VERSION != writtenVersion)
		{
			debug_outln_info(F("Rewriting old config from: "), writtenVersion);
			// would like to do that, but this would wipe firmware.old which the two stage loader
			// might still need
			// SPIFFS.format();
			rewriteConfig = true;
		}
		if (strcmp_P(cfg::senseboxid, PSTR("00112233445566778899aabb")) == 0)
		{
			cfg::senseboxid[0] = '\0';
			cfg::send2sensemap = false;
			rewriteConfig = true;
		}
		if (strlen(cfg::measurement_name_influx) == 0)
		{
			strcpy_P(cfg::measurement_name_influx, MEASUREMENT_NAME_INFLUX);
			rewriteConfig = true;
		}
		if (strcmp_P(cfg::host_influx, PSTR("api.luftdaten.info")) == 0)
		{
			cfg::host_influx[0] = '\0';
			cfg::send2influx = false;
			rewriteConfig = true;
		}
		if (boolFromJSON(json, F("pm24_read")) || boolFromJSON(json, F("pms32_read")))
		{
			cfg::pms_read = true;
			rewriteConfig = true;
		}
		if (boolFromJSON(json, F("bmp280_read")) || boolFromJSON(json, F("bme280_read")))
		{
			cfg::bmx280_read = true;
			rewriteConfig = true;
		}
	}
	else
	{
		debug_outln_error(F("failed to load json config"));

		if (!oldconfig)
		{
			return readConfig(true /* oldconfig */);
		}
	}

	if (rewriteConfig)
	{
		writeConfig();
	}
}

static void init_config()
{

	debug_outln_info(F("mounting FS..."));
#if defined(ESP32)
	bool spiffs_begin_ok = SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED);
#else
	bool spiffs_begin_ok = SPIFFS.begin();
#endif

	if (!spiffs_begin_ok)
	{
		debug_outln_error(F("failed to mount FS"));
		return;
	}
	readConfig();
}

/*****************************************************************
 * write config to spiffs                                        *
 *****************************************************************/
static void writeConfig()
{
	DynamicJsonDocument json(JSON_BUFFER_SIZE);
	debug_outln_info(F("Saving config..."));
	json["SOFTWARE_VERSION"] = SOFTWARE_VERSION;

	for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
	{
		ConfigShapeEntry c;
		memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
		switch (c.cfg_type)
		{
		case Config_Type_Bool:
			json[c.cfg_key].set(*c.cfg_val.as_bool);
			break;
		case Config_Type_UInt:
		case Config_Type_Time:
			json[c.cfg_key].set(*c.cfg_val.as_uint);
			break;
		case Config_Type_Password:
		case Config_Type_String:
			json[c.cfg_key].set(c.cfg_val.as_str);
			break;
		};
	}

	SPIFFS.remove(F("/config.json.old"));
	SPIFFS.rename(F("/config.json"), F("/config.json.old"));

	File configFile = SPIFFS.open(F("/config.json"), "w");
	if (configFile)
	{
		serializeJson(json, configFile);
		configFile.close();
		debug_outln_info(F("Config written successfully."));
	}
	else
	{
		debug_outln_error(F("failed to open config file for writing"));
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

/*****************************************************************
 * aircms.online helper functions                                *
 *****************************************************************/
static String sha1Hex(const String &s)
{
	char sha1sum_output[20];

#if defined(ESP8266)
	br_sha1_context sc;

	br_sha1_init(&sc);
	br_sha1_update(&sc, s.c_str(), s.length());
	br_sha1_out(&sc, sha1sum_output);
#endif
#if defined(ESP32)
	esp_sha(SHA1, (const unsigned char *)s.c_str(), s.length(), (unsigned char *)sha1sum_output);
#endif
	String r;
	for (uint16_t i = 0; i < 20; i++)
	{
		char hex[3];
		snprintf(hex, sizeof(hex), "%02x", sha1sum_output[i]);
		r += hex;
	}
	return r;
}

static String hmac1(const String &secret, const String &s)
{
	String str = sha1Hex(s);
	str = secret + str;
	return sha1Hex(str);
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
 * send data to rest api                                         *
 *****************************************************************/
static unsigned long sendData(const LoggerEntry logger, const String &data, const int pin, const char *host, const char *url)
{
#if defined(ESP8266)
	unsigned long start_send = millis();
	const __FlashStringHelper *contentType;
	int result = 0;
	int port;

	String s_Host(FPSTR(host));
	String s_url(FPSTR(url));

	switch (logger)
	{
	case Loggeraircms:
		contentType = FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN);
		break;
	case LoggerInflux:
		contentType = FPSTR(TXT_CONTENT_TYPE_INFLUXDB);
		break;
	default:
		contentType = FPSTR(TXT_CONTENT_TYPE_JSON);
		break;
	}

	std::unique_ptr<WiFiClient> client(getNewLoggerWiFiClient(logger));

	String request_head = F("POST ");
	request_head += String(s_url);
	request_head += F(" HTTP/1.1\r\n");
	request_head += F("Host: ");
	request_head += String(s_Host) + "\r\n";
	request_head += F("Content-Type: ");
	request_head += contentType;
	request_head += F("\r\n");
	request_head += F("X-PIN: ");
	request_head += String(pin) + "\r\n";
	request_head += F("X-Sensor: esp8266-");
	request_head += esp_chipid + "\r\n";
	request_head += F("Content-Length: ");
	request_head += String(data.length(), DEC) + "\r\n";
	request_head += F("Connection: close\r\n\r\n");

	if (!GPRS_CONNECTED)
	{
		if (!GPRS_init())
		{

			GPRS_INIT_FAIL_COUNT += 1;
			Serial.print("GPRS INIT FAIL COUNT: ");
			Serial.println(GPRS_INIT_FAIL_COUNT);
			if (GPRS_INIT_FAIL_COUNT == 5)
			{ //! RESET COUNTER
				GPRS_INIT_FAIL_COUNT = 0;
				GSM_soft_reset();
				GSM_init(fonaSerial);
			}
		}
	}
	if (GPRS_CONNECTED)
	{
		int retry_count = 0;
		uint16_t statuscode;
		int16_t length;

		String gprs_request_head = F("X-PIN: ");
		gprs_request_head += String(pin) + "\\r\\n";
		gprs_request_head += F("X-Sensor: esp8266-");
		gprs_request_head += esp_chipid;

		debug_out(F("Start connecting via GPRS"), DEBUG_MIN_INFO);
		debug_out(F("HOST "), DEBUG_MIN_INFO);
		debug_out(s_Host, DEBUG_MIN_INFO);
		debug_out(F("URL "), DEBUG_MIN_INFO);
		debug_out(s_url, DEBUG_MIN_INFO);
		debug_out(gprs_request_head, DEBUG_MIN_INFO);

		const char *data_copy = data.c_str();
		char gprs_data[strlen(data_copy)];
		strcpy(gprs_data, data_copy);

		String post_url = String(s_Host);
		post_url += String(s_url);
		const char *url_copy = post_url.c_str();
		char gprs_url[strlen(url_copy)];
		strcpy(gprs_url, url_copy);

		Serial.println("POST URL  " + String(gprs_url));

		debug_out(F("Sending data via gsm"), DEBUG_MIN_INFO);
		debug_out(F("http://"), DEBUG_MIN_INFO);
		debug_out(gprs_url, DEBUG_MIN_INFO);
		debug_out(gprs_data, DEBUG_MIN_INFO);

		flushSerial();
		debug_out(F("## Sending via gsm\n\n"), DEBUG_MIN_INFO);

		if (!fona.HTTP_POST_start((char *)gprs_url, F("application/json"), gprs_request_head, (uint8_t *)gprs_data, strlen(gprs_data), &statuscode, (uint16_t *)&length))
		{
			debug_outln_error(F("Failed with status code "));
			debug_out(String(statuscode), DEBUG_ERROR); // !ERROR not handled correctly: POST in most cases is successul but the status code !=200
			// disableGPRS();
			return 0;
		}
		while (length > 0)
		{
			while (fona.available())
			{
				char c = fona.read();
// Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
				loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
				UDR0 = c;
#else
				Serial.write(c);
// debug_out(String(c), DEBUG_MAX_INFO, 0);
#endif
				length--;
				if (!length)
					break;
			}
		}
		debug_out(F("\n\n## End sending via gsm \n\n"), DEBUG_MIN_INFO);
		fona.HTTP_POST_end();
		// disableGPRS();
	}
	else if (WiFi.status() == WL_CONNECTED)
	{
		HTTPClient http;
		http.setTimeout(20 * 1000);
		http.setUserAgent(SOFTWARE_VERSION + '/' + esp_chipid);
		http.setReuse(false);
		bool send_success = false;
		if (logger == LoggerCustom && (*cfg::user_custom || *cfg::pwd_custom))
		{
			http.setAuthorization(cfg::user_custom, cfg::pwd_custom);
		}
		if (logger == LoggerInflux && (*cfg::user_influx || *cfg::pwd_influx))
		{
			http.setAuthorization(cfg::user_influx, cfg::pwd_influx);
		}
		if (http.begin(*client, s_Host, loggerConfigs[logger].destport, s_url, !!loggerConfigs[logger].session))
		{
			http.addHeader(F("Content-Type"), contentType);
			http.addHeader(F("X-Sensor"), String(F(SENSOR_BASENAME)) + esp_chipid);
			if (pin)
			{
				http.addHeader(F("X-PIN"), String(pin));
			}

			result = http.POST(data);

			if (result >= HTTP_CODE_OK && result <= HTTP_CODE_ALREADY_REPORTED)
			{
				debug_outln_info(F("Succeeded - "), s_Host);
				send_success = true;
			}
			else if (result >= HTTP_CODE_BAD_REQUEST)
			{
				debug_outln_info(F("Request failed with error: "), String(result));
				debug_outln_info(F("Details:"), http.getString());
			}
			http.end();
		}
	}
	else
	{
		debug_outln_info(F("Failed connecting to "), s_Host);
	}

	wdt_reset();
	yield();
	return millis() - start_send;
#endif
}
/*****************************************************************
 * send single sensor data to sensors.AFRICA api                  *
 *****************************************************************/
static unsigned long sendCFA(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str)
{
	unsigned long sum_send_time = 0;

	if (cfg::send2cfa && data.length())
	{
		RESERVE_STRING(data_CFA, LARGE_STR);
		data_CFA = FPSTR(data_first_part);

		debug_outln_info(F("## Sending to sensors.AFRICA - "), sensorname);
		data_CFA += data;
		data_CFA.remove(data_CFA.length() - 1);
		data_CFA.replace(replace_str, emptyString);
		data_CFA += "]}";
		Serial.println(data_CFA);

		sum_send_time = sendData(LoggerCFA, data_CFA, pin, HOST_CFA, URL_CFA);
	}

	return sum_send_time;
}

/*****************************************************************
 * send single sensor data to sensor.community api                *
 *****************************************************************/
static unsigned long sendSensorCommunity(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str)
{
	unsigned long sum_send_time = 0;

	if (cfg::send2dusti && data.length())
	{
		RESERVE_STRING(data_sensorcommunity, LARGE_STR);
		data_sensorcommunity = FPSTR(data_first_part);

		debug_outln_info(F("## Sending to sensor.community - "), sensorname);
		data_sensorcommunity += data;
		data_sensorcommunity.remove(data_sensorcommunity.length() - 1);
		data_sensorcommunity.replace(replace_str, emptyString);
		data_sensorcommunity += "]}";
		sum_send_time = sendData(LoggerSensorCommunity, data_sensorcommunity, pin, HOST_SENSORCOMMUNITY, URL_SENSORCOMMUNITY);
	}

	return sum_send_time;
}

/*****************************************************************
 * send data to mqtt api                                         *
 *****************************************************************/
// rejected (see issue #33)

/*****************************************************************
 * send data to influxdb                                         *
 *****************************************************************/
static void create_influxdb_string_from_data(String &data_4_influxdb, const String &data)
{
	debug_outln_verbose(F("Parse JSON for influx DB: "), data);
	DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json2data, data);
	if (!err)
	{
		data_4_influxdb += cfg::measurement_name_influx;
		data_4_influxdb += F(",node=" SENSOR_BASENAME);
		data_4_influxdb += esp_chipid + " ";
		for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
		{
			data_4_influxdb += measurement["value_type"].as<char *>();
			data_4_influxdb += '=';
			data_4_influxdb += measurement["value"].as<char *>();
			data_4_influxdb += ',';
		}
		if ((unsigned)(data_4_influxdb.lastIndexOf(',') + 1) == data_4_influxdb.length())
		{
			data_4_influxdb.remove(data_4_influxdb.length() - 1);
		}

		data_4_influxdb += '\n';
	}
	else
	{
		debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
	}
}

/*****************************************************************
 * send data as csv to serial out                                *
 *****************************************************************/
static void send_csv(const String &data)
{
	DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json2data, data);
	debug_outln_info(F("CSV Output: "), data);
	if (!err)
	{
		String headline = F("Timestamp_ms;");
		String valueline(act_milli);
		valueline += ';';
		for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
		{
			headline += measurement["value_type"].as<char *>();
			headline += ';';
			valueline += measurement["value"].as<char *>();
			valueline += ';';
		}
		static bool first_csv_line = true;
		if (first_csv_line)
		{
			if (headline.length() > 0)
			{
				headline.remove(headline.length() - 1);
			}
			Serial.println(headline);
			first_csv_line = false;
		}
		if (valueline.length() > 0)
		{
			valueline.remove(valueline.length() - 1);
		}
		Serial.println(valueline);
	}
	else
	{
		debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
	}
}

/*****************************************************************
 * read DHT22 sensor values                                      *
 *****************************************************************/
static void fetchSensorDHT(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_DHT22));

	// Check if valid number if non NaN (not a number) will be send.
	last_value_DHT_T = -128;
	last_value_DHT_H = -1;

	int count = 0;
	const int MAX_ATTEMPTS = 5;
	while ((count++ < MAX_ATTEMPTS))
	{
		auto t = dht.readTemperature();
		auto h = dht.readHumidity();
		if (isnan(t) || isnan(h))
		{
			delay(100);
			t = dht.readTemperature(false);
			h = dht.readHumidity();
		}
		if (isnan(t) || isnan(h))
		{
			debug_outln_error(F("DHT11/DHT22 read failed"));
		}
		else
		{
			last_value_DHT_T = t;
			last_value_DHT_H = h;
			add_Value2Json(s, F("temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_DHT_T);
			add_Value2Json(s, F("humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_DHT_H);
			switch_status_LEDs_on(DHT_LED, HIGH);
			delay(5000);
			switch_status_LEDs_off(DHT_LED, LOW);
			break;
		}
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_DHT22));
}

/*****************************************************************
 * read HTU21D sensor values                                     *
 *****************************************************************/
static void fetchSensorHTU21D(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_HTU21D));

	const auto t = htu21d.readTemperature();
	const auto h = htu21d.readHumidity();
	if (isnan(t) || isnan(h))
	{
		last_value_HTU21D_T = -128.0;
		last_value_HTU21D_H = -1.0;
		debug_outln_error(F("HTU21D read failed"));
	}
	else
	{
		last_value_HTU21D_T = t;
		last_value_HTU21D_H = h;
		add_Value2Json(s, F("HTU21D_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_HTU21D_T);
		add_Value2Json(s, F("HTU21D_humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_HTU21D_H);
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_HTU21D));
}

/*****************************************************************
 * read BMP180 sensor values                                     *
 *****************************************************************/
static void fetchSensorBMP(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_BMP180));

	const auto p = bmp.readPressure();
	const auto t = bmp.readTemperature();
	if (isnan(p) || isnan(t))
	{
		last_value_BMP_T = -128.0;
		last_value_BMP_P = -1.0;
		debug_outln_error(F("BMP180 read failed"));
	}
	else
	{
		last_value_BMP_T = t;
		last_value_BMP_P = p;
		add_Value2Json(s, F("BMP_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMP_P);
		add_Value2Json(s, F("BMP_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMP_T);
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_BMP180));
}

/*****************************************************************
 * read SHT3x sensor values                                      *
 *****************************************************************/
static void fetchSensorSHT3x(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_SHT3X));

	const auto t = sht3x.readTemperature();
	const auto h = sht3x.readHumidity();
	if (isnan(h) || isnan(t))
	{
		last_value_SHT3X_T = -128.0;
		last_value_SHT3X_H = -1.0;
		debug_outln_error(F("SHT3X read failed"));
	}
	else
	{
		last_value_SHT3X_T = t;
		last_value_SHT3X_H = h;
		add_Value2Json(s, F("SHT3X_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_SHT3X_T);
		add_Value2Json(s, F("SHT3X_humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_SHT3X_H);
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_SHT3X));
}

/*****************************************************************
 * read BMP280/BME280 sensor values                              *
 *****************************************************************/
static void fetchSensorBMX280(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_BMX280));

	bmx280.takeForcedMeasurement();
	const auto t = bmx280.readTemperature();
	const auto p = bmx280.readPressure();
	const auto h = bmx280.readHumidity();
	if (isnan(t) || isnan(p))
	{
		last_value_BMX280_T = -128.0;
		last_value_BMX280_P = -1.0;
		last_value_BME280_H = -1.0;
		debug_outln_error(F("BMP/BME280 read failed"));
	}
	else
	{
		last_value_BMX280_T = t;
		last_value_BMX280_P = p;
		if (bmx280.sensorID() == BME280_SENSOR_ID)
		{
			add_Value2Json(s, F("BME280_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMX280_T);
			add_Value2Json(s, F("BME280_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMX280_P);
			last_value_BME280_H = h;
			add_Value2Json(s, F("BME280_humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_BME280_H);
		}
		else
		{
			add_Value2Json(s, F("BMP280_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMX280_P);
			add_Value2Json(s, F("BMP280_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMX280_T);
		}
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_BMX280));
}

/*****************************************************************
 * read DS18B20 sensor values                                    *
 *****************************************************************/
static void fetchSensorDS18B20(String &s)
{
	float t;
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_DS18B20));

	// it's very unlikely (-127: impossible) to get these temperatures in reality. Most times this means that the sensor is currently faulty
	// try 5 times to read the sensor, otherwise fail
	const int MAX_ATTEMPTS = 5;
	int count = 0;
	do
	{
		ds18b20.requestTemperatures();
		// for now, we want to read only the first sensor
		t = ds18b20.getTempCByIndex(0);
		++count;
		debug_outln_info(F("DS18B20 trying...."));
	} while (count < MAX_ATTEMPTS && (isnan(t) || t >= 85.0f || t <= (-127.0f)));

	if (count == MAX_ATTEMPTS)
	{
		last_value_DS18B20_T = -128.0;
		debug_outln_error(F("DS18B20 read failed"));
	}
	else
	{
		last_value_DS18B20_T = t;
		add_Value2Json(s, F("DS18B20_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_DS18B20_T);
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_DS18B20));
}

/*****************************************************************
 * read SDS011 sensor values                                     *
 *****************************************************************/
static void fetchSensorSDS(String &s)
{

	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_SDS011));

	if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS) &&
		msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
	{
		if (is_SDS_running)
		{
			is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
		}
	}
	else
	{
		if (!is_SDS_running)
		{
			is_SDS_running = SDS_cmd(PmSensorCmd::Start);
		}

		const uint8_t constexpr header_measurement[2] = {0xAA, 0xC0};

		while (serialSDS.available() >= 10 &&
			   serialSDS.find(header_measurement, sizeof(header_measurement)))
		{
			uint8_t data[8];
			yield_for_serial_buffer(sizeof(data));
			unsigned r = serialSDS.readBytes(data, sizeof(data));
			if (r == sizeof(data) && SDS_checksum_valid(data))
			{
				uint32_t pm25_serial = data[0] | (data[1] << 8);
				uint32_t pm10_serial = data[2] | (data[3] << 8);

				if (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS))
				{
					sds_pm10_sum += pm10_serial;
					sds_pm25_sum += pm25_serial;
					if (sds_pm10_min > pm10_serial)
					{
						sds_pm10_min = pm10_serial;
					}
					if (sds_pm10_max < pm10_serial)
					{
						sds_pm10_max = pm10_serial;
					}
					if (sds_pm25_min > pm25_serial)
					{
						sds_pm25_min = pm25_serial;
					}
					if (sds_pm25_max < pm25_serial)
					{
						sds_pm25_max = pm25_serial;
					}
					debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial / 10.0f));
					debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial / 10.0f));
					sds_val_count++;
				}
			}
		}
	}
	if (send_now)
	{
		last_value_SDS_P1 = -1;
		last_value_SDS_P2 = -1;
		if (sds_val_count > 2)
		{
			sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
			sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
			sds_val_count = sds_val_count - 2;
		}
		if (sds_val_count > 0)
		{
			last_value_SDS_P1 = float(sds_pm10_sum) / (sds_val_count * 10.0f);
			last_value_SDS_P2 = float(sds_pm25_sum) / (sds_val_count * 10.0f);
			add_Value2Json(s, F("SDS_P1"), F("PM10:  "), last_value_SDS_P1);
			add_Value2Json(s, F("SDS_P2"), F("PM2.5: "), last_value_SDS_P2);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
			if (sds_val_count < 3)
			{
				SDS_error_count++;
			}
		}
		else
		{
			SDS_error_count++;
		}
		sds_pm10_sum = 0;
		sds_pm25_sum = 0;
		sds_val_count = 0;
		sds_pm10_max = 0;
		sds_pm10_min = 20000;
		sds_pm25_max = 0;
		sds_pm25_min = 20000;
		if ((cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
		{

			if (is_SDS_running)
			{
				is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
			}
		}
	}

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_SDS011));
}

/*****************************************************************
 * read Plantronic PM sensor sensor values                       *
 *****************************************************************/
static void fetchSensorPMS(String &s)
{
	char buffer;
	int value;
	int len = 0;
	int pm1_serial = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_should = 0;
	bool checksum_ok = false;
	int frame_len = 24; // min. frame length

	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_PMSx003));
	if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
	{
		if (is_PMS_running)
		{
			is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
		}
	}
	else
	{
		if (!is_PMS_running)
		{
			is_PMS_running = PMS_cmd(PmSensorCmd::Start);
		}

		while (serialSDS.available() > 0)
		{
			buffer = serialSDS.read();
			debug_outln(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO);
			//			"aa" = 170, "ab" = 171, "c0" = 192
			value = int(buffer);
			switch (len)
			{
			case 0:
				if (value != 66)
				{
					len = -1;
				};
				break;
			case 1:
				if (value != 77)
				{
					len = -1;
				};
				break;
			case 2:
				checksum_is = value;
				break;
			case 3:
				frame_len = value + 4;
				break;
			case 10:
				pm1_serial += (value << 8);
				break;
			case 11:
				pm1_serial += value;
				break;
			case 12:
				pm25_serial = (value << 8);
				break;
			case 13:
				pm25_serial += value;
				break;
			case 14:
				pm10_serial = (value << 8);
				break;
			case 15:
				pm10_serial += value;
				break;
			case 22:
				if (frame_len == 24)
				{
					checksum_should = (value << 8);
				};
				break;
			case 23:
				if (frame_len == 24)
				{
					checksum_should += value;
				};
				break;
			case 30:
				checksum_should = (value << 8);
				break;
			case 31:
				checksum_should += value;
				break;
			}
			if ((len > 2) && (len < (frame_len - 2)))
			{
				checksum_is += value;
			}
			len++;
			if (len == frame_len)
			{
				debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_IS), String(checksum_is + 143));
				debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_SHOULD), String(checksum_should));
				if (checksum_should == (checksum_is + 143))
				{
					checksum_ok = true;
				}
				else
				{
					len = 0;
				};
				if (checksum_ok && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS)))
				{
					if ((!isnan(pm1_serial)) && (!isnan(pm10_serial)) && (!isnan(pm25_serial)))
					{
						pms_pm1_sum += pm1_serial;
						pms_pm10_sum += pm10_serial;
						pms_pm25_sum += pm25_serial;
						if (pms_pm1_min > pm1_serial)
						{
							pms_pm1_min = pm1_serial;
						}
						if (pms_pm1_max < pm1_serial)
						{
							pms_pm1_max = pm1_serial;
						}
						if (pms_pm25_min > pm25_serial)
						{
							pms_pm25_min = pm25_serial;
						}
						if (pms_pm25_max < pm25_serial)
						{
							pms_pm25_max = pm25_serial;
						}
						if (pms_pm10_min > pm10_serial)
						{
							pms_pm10_min = pm10_serial;
						}
						if (pms_pm10_max < pm10_serial)
						{
							pms_pm10_max = pm10_serial;
						}
						debug_outln_verbose(F("PM1 (sec.): "), String(pm1_serial));
						debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial));
						debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial));
						pms_val_count++;
					}
					len = 0;
					checksum_ok = false;
					pm1_serial = 0;
					pm10_serial = 0;
					pm25_serial = 0;
					checksum_is = 0;
				}
			}
			yield();
		}
	}
	if (send_now)
	{
		last_value_PMS_P0 = -1;
		last_value_PMS_P1 = -1;
		last_value_PMS_P2 = -1;
		if (pms_val_count > 2)
		{
			pms_pm1_sum = pms_pm1_sum - pms_pm1_min - pms_pm1_max;
			pms_pm10_sum = pms_pm10_sum - pms_pm10_min - pms_pm10_max;
			pms_pm25_sum = pms_pm25_sum - pms_pm25_min - pms_pm25_max;
			pms_val_count = pms_val_count - 2;
		}
		if (pms_val_count > 0)
		{
			last_value_PMS_P0 = float(pms_pm1_sum) / float(pms_val_count);
			last_value_PMS_P1 = float(pms_pm10_sum) / float(pms_val_count);
			last_value_PMS_P2 = float(pms_pm25_sum) / float(pms_val_count);
			add_Value2Json(s, F("PMS_P0"), F("PM1:   "), last_value_PMS_P0);
			add_Value2Json(s, F("PMS_P1"), F("PM10:  "), last_value_PMS_P1);
			add_Value2Json(s, F("PMS_P2"), F("PM2.5: "), last_value_PMS_P2);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		pms_pm1_sum = 0;
		pms_pm10_sum = 0;
		pms_pm25_sum = 0;
		pms_val_count = 0;
		pms_pm1_max = 0;
		pms_pm1_min = 20000;
		pms_pm10_max = 0;
		pms_pm10_min = 20000;
		pms_pm25_max = 0;
		pms_pm25_min = 20000;
		if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))
		{
			is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
		}
		switch_status_LEDs_on(PMS_LED, HIGH);
		delay(5000);
		switch_status_LEDs_off(PMS_LED, LOW);
	}

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_PMSx003));
}

/*****************************************************************
 * read Honeywell PM sensor sensor values                        *
 *****************************************************************/
static void fetchSensorHPM(String &s)
{
	char buffer;
	int value;
	int len = 0;
	int pm10_serial = 0;
	int pm25_serial = 0;
	int checksum_is = 0;
	int checksum_should = 0;
	bool checksum_ok = false;

	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_HPM));
	if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
	{
		if (is_HPM_running)
		{
			is_HPM_running = HPM_cmd(PmSensorCmd::Stop);
		}
	}
	else
	{
		if (!is_HPM_running)
		{
			is_HPM_running = HPM_cmd(PmSensorCmd::Start);
		}

		while (serialSDS.available() > 0)
		{
			buffer = serialSDS.read();
			debug_outln(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO);
			//			"aa" = 170, "ab" = 171, "c0" = 192
			value = int(buffer);
			switch (len)
			{
			case 0:
				if (value != 66)
				{
					len = -1;
				};
				break;
			case 1:
				if (value != 77)
				{
					len = -1;
				};
				break;
			case 2:
				checksum_is = value;
				break;
			case 6:
				pm25_serial += (value << 8);
				break;
			case 7:
				pm25_serial += value;
				break;
			case 8:
				pm10_serial = (value << 8);
				break;
			case 9:
				pm10_serial += value;
				break;
			case 30:
				checksum_should = (value << 8);
				break;
			case 31:
				checksum_should += value;
				break;
			}
			if (len > 2 && len < 30)
			{
				checksum_is += value;
			}
			len++;
			if (len == 32)
			{
				debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_IS), String(checksum_is + 143));
				debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_SHOULD), String(checksum_should));
				if (checksum_should == (checksum_is + 143))
				{
					checksum_ok = true;
				}
				else
				{
					len = 0;
				};
				if (checksum_ok && (long(msSince(starttime)) > (long(cfg::sending_intervall_ms) - long(READINGTIME_SDS_MS))))
				{
					if ((!isnan(pm10_serial)) && (!isnan(pm25_serial)))
					{
						hpm_pm10_sum += pm10_serial;
						hpm_pm25_sum += pm25_serial;
						if (hpm_pm10_min > pm10_serial)
						{
							hpm_pm10_min = pm10_serial;
						}
						if (hpm_pm10_max < pm10_serial)
						{
							hpm_pm10_max = pm10_serial;
						}
						if (hpm_pm25_min > pm25_serial)
						{
							hpm_pm25_min = pm25_serial;
						}
						if (hpm_pm25_max < pm25_serial)
						{
							hpm_pm25_max = pm25_serial;
						}
						debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial));
						debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial));
						hpm_val_count++;
					}
					len = 0;
					checksum_ok = false;
					pm10_serial = 0;
					pm25_serial = 0;
					checksum_is = 0;
				}
			}
			yield();
		}
	}
	if (send_now)
	{
		last_value_HPM_P1 = -1.0f;
		last_value_HPM_P2 = -1.0f;
		if (hpm_val_count > 2)
		{
			hpm_pm10_sum = hpm_pm10_sum - hpm_pm10_min - hpm_pm10_max;
			hpm_pm25_sum = hpm_pm25_sum - hpm_pm25_min - hpm_pm25_max;
			hpm_val_count = hpm_val_count - 2;
		}
		if (hpm_val_count > 0)
		{
			last_value_HPM_P1 = float(hpm_pm10_sum) / float(hpm_val_count);
			last_value_HPM_P2 = float(hpm_pm25_sum) / float(hpm_val_count);
			add_Value2Json(s, F("HPM_P1"), F("PM2.5: "), last_value_HPM_P1);
			add_Value2Json(s, F("HPM_P2"), F("PM10:  "), last_value_HPM_P2);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		hpm_pm10_sum = 0;
		hpm_pm25_sum = 0;
		hpm_val_count = 0;
		hpm_pm10_max = 0;
		hpm_pm10_min = 20000;
		hpm_pm25_max = 0;
		hpm_pm25_min = 20000;
		if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))
		{
			is_HPM_running = HPM_cmd(PmSensorCmd::Stop);
		}
	}

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_HPM));
}

/*****************************************************************
 * read PPD42NS sensor values                                    *
 *****************************************************************/
static void fetchSensorPPD(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_PPD42NS));

	if (msSince(starttime) <= SAMPLETIME_MS)
	{

		// Read pins connected to ppd42ns
		boolean valP1 = digitalRead(PPD_PIN_PM1);
		boolean valP2 = digitalRead(PPD_PIN_PM2);

		if (valP1 == LOW && trigP1 == false)
		{
			trigP1 = true;
			trigOnP1 = act_micro;
		}

		if (valP1 == HIGH && trigP1 == true)
		{
			lowpulseoccupancyP1 += act_micro - trigOnP1;
			trigP1 = false;
		}

		if (valP2 == LOW && trigP2 == false)
		{
			trigP2 = true;
			trigOnP2 = act_micro;
		}

		if (valP2 == HIGH && trigP2 == true)
		{
			unsigned long durationP2 = act_micro - trigOnP2;
			lowpulseoccupancyP2 += durationP2;
			trigP2 = false;
		}
	}
	// Checking if it is time to sample
	if (send_now)
	{
		auto calcConcentration = [](const float ratio)
		{
			/* spec sheet curve*/
			return (1.1f * ratio * ratio * ratio - 3.8f * ratio * ratio + 520.0f * ratio + 0.62f);
		};

		last_value_PPD_P1 = -1;
		last_value_PPD_P2 = -1;
		float ratio = lowpulseoccupancyP1 / (SAMPLETIME_MS * 10.0f);
		float concentration = calcConcentration(ratio);

		// json for push to api / P1
		last_value_PPD_P1 = concentration;
		add_Value2Json(s, F("durP1"), F("LPO P10    : "), lowpulseoccupancyP1);
		add_Value2Json(s, F("ratioP1"), F("Ratio PM10%: "), ratio);
		add_Value2Json(s, F("P1"), F("PM10 Count : "), last_value_PPD_P1);

		ratio = lowpulseoccupancyP2 / (SAMPLETIME_MS * 10.0f);
		concentration = calcConcentration(ratio);

		// json for push to api / P2
		last_value_PPD_P2 = concentration;
		add_Value2Json(s, F("durP2"), F("LPO PM25   : "), lowpulseoccupancyP2);
		add_Value2Json(s, F("ratioP2"), F("Ratio PM25%: "), ratio);
		add_Value2Json(s, F("P2"), F("PM25 Count : "), last_value_PPD_P2);

		debug_outln_info(FPSTR(DBG_TXT_SEP));
	}

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_PPD42NS));
}

/*****************************************************************
   read SPS30 PM sensor values
 *****************************************************************/
static void fetchSensorSPS30(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_SPS30));

	last_value_SPS30_P0 = value_SPS30_P0 / SPS30_measurement_count;
	last_value_SPS30_P2 = value_SPS30_P2 / SPS30_measurement_count;
	last_value_SPS30_P4 = value_SPS30_P4 / SPS30_measurement_count;
	last_value_SPS30_P1 = value_SPS30_P1 / SPS30_measurement_count;
	last_value_SPS30_N05 = value_SPS30_N05 / SPS30_measurement_count;
	last_value_SPS30_N1 = value_SPS30_N1 / SPS30_measurement_count;
	last_value_SPS30_N25 = value_SPS30_N25 / SPS30_measurement_count;
	last_value_SPS30_N4 = value_SPS30_N4 / SPS30_measurement_count;
	last_value_SPS30_N10 = value_SPS30_N10 / SPS30_measurement_count;
	last_value_SPS30_TS = value_SPS30_TS / SPS30_measurement_count;

	add_Value2Json(s, F("SPS30_P0"), F("PM1.0: "), last_value_SPS30_P0);
	add_Value2Json(s, F("SPS30_P2"), F("PM2.5: "), last_value_SPS30_P2);
	add_Value2Json(s, F("SPS30_P4"), F("PM4.0: "), last_value_SPS30_P4);
	add_Value2Json(s, F("SPS30_P1"), F("PM 10: "), last_value_SPS30_P1);
	add_Value2Json(s, F("SPS30_N05"), F("NC0.5: "), last_value_SPS30_N05);
	add_Value2Json(s, F("SPS30_N1"), F("NC1.0: "), last_value_SPS30_N1);
	add_Value2Json(s, F("SPS30_N25"), F("NC2.5: "), last_value_SPS30_N25);
	add_Value2Json(s, F("SPS30_N4"), F("NC4.0: "), last_value_SPS30_N4);
	add_Value2Json(s, F("SPS30_N10"), F("NC10:  "), last_value_SPS30_N10);
	add_Value2Json(s, F("SPS30_TS"), F("TPS:   "), last_value_SPS30_TS);

	debug_outln_info(F("SPS30 read counter: "), String(SPS30_read_counter));
	debug_outln_info(F("SPS30 read error counter: "), String(SPS30_read_error_counter));

	SPS30_measurement_count = 0;
	SPS30_read_counter = 0;
	SPS30_read_error_counter = 0;
	value_SPS30_P0 = 0.0;
	value_SPS30_P2 = 0.0;
	value_SPS30_P4 = 0.0;
	value_SPS30_P1 = 0.0;
	value_SPS30_N05 = 0.0;
	value_SPS30_N1 = 0.0;
	value_SPS30_N25 = 0.0;
	value_SPS30_N4 = 0.0;
	value_SPS30_N10 = 0.0;
	value_SPS30_TS = 0.0;

	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_SPS30));
}

/*****************************************************************
 * read GPS sensor values                                        *
 *****************************************************************/
static void fetchSensorGPS(String &s)
{
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), "GPS");

	if (gps.location.isUpdated())
	{
		if (gps.location.isValid())
		{
			last_value_GPS_lat = gps.location.lat();
			last_value_GPS_lon = gps.location.lng();
		}
		else
		{
			last_value_GPS_lat = -200;
			last_value_GPS_lon = -200;
			debug_outln_verbose(F("Lat/Lng INVALID"));
		}
		if (gps.altitude.isValid())
		{
			last_value_GPS_alt = gps.altitude.meters();
			String gps_alt(last_value_GPS_lat);
		}
		else
		{
			last_value_GPS_alt = -1000;
			debug_outln_verbose(F("Altitude INVALID"));
		}
		if (gps.date.isValid())
		{
			char gps_date[16];
			snprintf_P(gps_date, sizeof(gps_date), PSTR("%02d/%02d/%04d"),
					   gps.date.month(), gps.date.day(), gps.date.year());
			last_value_GPS_date = gps_date;
			last_value_GPS_timestamp = gps_date;
		}
		else
		{
			debug_outln_verbose(F("Date INVALID"));
		}
		if (gps.time.isValid())
		{
			char gps_time[20];
			snprintf_P(gps_time, sizeof(gps_time), PSTR("%02d:%02d:%02d.%02d"),
					   gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
			last_value_GPS_time = gps_time;
			last_value_GPS_timestamp += "T";
			last_value_GPS_timestamp += gps_time;
		}
		else
		{
			debug_outln_verbose(F("Time: INVALID"));
		}
	}

	if (send_now)
	{
		debug_outln_info(F("Lat: "), String(last_value_GPS_lat, 6));
		debug_outln_info(F("Lng: "), String(last_value_GPS_lon, 6));
		debug_outln_info(F("Date: "), last_value_GPS_date);
		debug_outln_info(F("Time "), last_value_GPS_time);

		add_Value2Json(s, F("GPS_lat"), String(last_value_GPS_lat, 6));
		add_Value2Json(s, F("GPS_lon"), String(last_value_GPS_lon, 6));
		add_Value2Json(s, F("GPS_height"), F("Altitude: "), last_value_GPS_alt);
		add_Value2Json(s, F("GPS_timestamp"), last_value_GPS_timestamp);
		debug_outln_info(FPSTR(DBG_TXT_SEP));
	}

	if (count_sends > 0 && gps.charsProcessed() < 10)
	{
		debug_outln_error(F("No GPS data received: check wiring"));
		gps_init_failed = true;
	}

	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), "GPS");
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

static String displayGenerateFooter(unsigned int screen_count)
{
	String display_footer;
	for (unsigned int i = 0; i < screen_count; ++i)
	{
		display_footer += (i != (next_display_count % screen_count)) ? " . " : " o ";
	}
	return display_footer;
}

/*****************************************************************
 * display values                                                *
 *****************************************************************/
static void display_values()
{
	float t_value = -128.0;
	float h_value = -1.0;
	float p_value = -1.0;
	String t_sensor, h_sensor, p_sensor;
	float pm01_value = -1.0;
	float pm04_value = -1.0;
	float pm10_value = -1.0;
	float pm25_value = -1.0;
	String pm10_sensor;
	String pm25_sensor;
	float nc005_value = -1.0;
	float nc010_value = -1.0;
	float nc025_value = -1.0;
	float nc040_value = -1.0;
	float nc100_value = -1.0;
	float la_eq_value = -1.0;
	float la_max_value = -1.0;
	float la_min_value = -1.0;
	String la_sensor;
	float tps_value = -1.0;
	double lat_value = -200.0;
	double lon_value = -200.0;
	double alt_value = -1000.0;
	String display_header;
	String display_lines[3] = {"", "", ""};
	uint8_t screen_count = 0;
	uint8_t screens[8];
	int line_count = 0;
	debug_outln_info(F("output values to display..."));
	if (cfg::ppd_read)
	{
		pm10_value = last_value_PPD_P1;
		pm10_sensor = FPSTR(SENSORS_PPD42NS);
		pm25_value = last_value_PPD_P2;
		pm25_sensor = FPSTR(SENSORS_PPD42NS);
	}
	if (cfg::pms_read)
	{
		pm10_value = last_value_PMS_P1;
		pm10_sensor = FPSTR(SENSORS_PMSx003);
		pm25_value = last_value_PMS_P2;
		pm25_sensor = FPSTR(SENSORS_PMSx003);
	}
	if (cfg::hpm_read)
	{
		pm10_value = last_value_HPM_P1;
		pm10_sensor = FPSTR(SENSORS_HPM);
		pm25_value = last_value_HPM_P2;
		pm25_sensor = FPSTR(SENSORS_HPM);
	}
	if (cfg::sps30_read)
	{
		pm10_sensor = FPSTR(SENSORS_SPS30);
		pm25_sensor = FPSTR(SENSORS_SPS30);
		pm01_value = last_value_SPS30_P0;
		pm25_value = last_value_SPS30_P2;
		pm04_value = last_value_SPS30_P4;
		pm10_value = last_value_SPS30_P1;
		nc005_value = last_value_SPS30_N05;
		nc010_value = last_value_SPS30_N1;
		nc025_value = last_value_SPS30_N25;
		nc040_value = last_value_SPS30_N4;
		nc100_value = last_value_SPS30_N10;
		tps_value = last_value_SPS30_TS;
	}
	if (cfg::sds_read)
	{
		pm10_sensor = pm25_sensor = FPSTR(SENSORS_SDS011);
		pm10_value = last_value_SDS_P1;
		pm25_value = last_value_SDS_P2;
	}
	if (cfg::dht_read)
	{
		t_sensor = h_sensor = FPSTR(SENSORS_DHT22);
		t_value = last_value_DHT_T;
		h_value = last_value_DHT_H;
	}
	if (cfg::ds18b20_read)
	{
		t_sensor = FPSTR(SENSORS_DS18B20);
		t_value = last_value_DS18B20_T;
	}
	if (cfg::htu21d_read)
	{
		h_sensor = t_sensor = FPSTR(SENSORS_HTU21D);
		t_value = last_value_HTU21D_T;
		h_value = last_value_HTU21D_H;
	}
	if (cfg::bmp_read)
	{
		t_sensor = h_sensor = FPSTR(SENSORS_BMP180);
		t_value = last_value_BMP_T;
		p_value = last_value_BMP_P;
	}
	if (cfg::bmx280_read)
	{
		t_sensor = p_sensor = FPSTR(SENSORS_BMX280);
		t_value = last_value_BMX280_T;
		p_value = last_value_BMX280_P;
		if (bmx280.sensorID() == BME280_SENSOR_ID)
		{
			h_sensor = FPSTR(SENSORS_BMX280);
			h_value = last_value_BME280_H;
		}
	}
	if (cfg::sht3x_read)
	{
		h_sensor = t_sensor = FPSTR(SENSORS_SHT3X);
		t_value = last_value_SHT3X_T;
		h_value = last_value_SHT3X_H;
	}
	if (cfg::dnms_read)
	{
		la_sensor = FPSTR(SENSORS_DNMS);
		la_eq_value = last_value_dnms_laeq;
		la_max_value = last_value_dnms_la_max;
		la_min_value = last_value_dnms_la_min;
	}
	if (cfg::gps_read)
	{
		lat_value = last_value_GPS_lat;
		lon_value = last_value_GPS_lon;
		alt_value = last_value_GPS_alt;
	}
	if (cfg::ppd_read || cfg::pms_read || cfg::hpm_read || cfg::sds_read)
	{
		screens[screen_count++] = 1;
	}
	if (cfg::sps30_read)
	{
		screens[screen_count++] = 2;
	}
	if (cfg::dht_read || cfg::ds18b20_read || cfg::htu21d_read || cfg::bmp_read || cfg::bmx280_read || cfg::sht3x_read)
	{
		screens[screen_count++] = 3;
	}
	if (cfg::gps_read)
	{
		screens[screen_count++] = 4;
	}
	if (cfg::dnms_read)
	{
		screens[screen_count++] = 5;
	}
	if (cfg::display_wifi_info)
	{
		screens[screen_count++] = 6; // Wifi info
	}
	if (cfg::display_device_info)
	{
		screens[screen_count++] = 7; // chipID, firmware and count of measurements
	}
	// update size of "screens" when adding more screens!

	if (cfg::has_display || cfg::has_sh1106 || lcd_2004)
	{
		switch (screens[next_display_count % screen_count])
		{
		case 1:
			display_header = pm25_sensor;
			if (pm25_sensor != pm10_sensor)
			{
				display_header += " / " + pm10_sensor;
			}
			display_lines[0] = std::move(tmpl(F("PM2.5: {v} µg/m³"), check_display_value(pm25_value, -1, 1, 6)));
			display_lines[1] = std::move(tmpl(F("PM10: {v} µg/m³"), check_display_value(pm10_value, -1, 1, 6)));
			display_lines[2] = emptyString;
			break;
		case 2:
			display_header = FPSTR(SENSORS_SPS30);
			display_lines[0] = "PM: " + check_display_value(pm01_value, -1, 1, 4) + " " + check_display_value(pm25_value, -1, 1, 4) + " " + check_display_value(pm04_value, -1, 1, 4) + " " + check_display_value(pm10_value, -1, 1, 4);
			display_lines[1] = "NC: " + check_display_value(nc005_value, -1, 0, 3) + " " + check_display_value(nc010_value, -1, 0, 3) + " " + check_display_value(nc025_value, -1, 0, 3) + " " + check_display_value(nc040_value, -1, 0, 3) + " " + check_display_value(nc100_value, -1, 0, 3);
			display_lines[2] = std::move(tmpl(F("TPS: {v} µm"), check_display_value(tps_value, -1, 2, 5)));
			break;
		case 3:
			display_header = t_sensor;
			if (h_sensor && t_sensor != h_sensor)
			{
				display_header += " / " + h_sensor;
			}
			if ((h_sensor && p_sensor && (h_sensor != p_sensor)) || (h_sensor == "" && p_sensor && (t_sensor != p_sensor)))
			{
				display_header += " / " + p_sensor;
			}
			if (t_sensor != "")
			{
				display_lines[line_count] = "Temp.: ";
				display_lines[line_count] += check_display_value(t_value, -128, 1, 6);
				display_lines[line_count++] += " °C";
			}
			if (h_sensor != "")
			{
				display_lines[line_count] = "Hum.:  ";
				display_lines[line_count] += check_display_value(h_value, -1, 1, 6);
				display_lines[line_count++] += " %";
			}
			if (p_sensor != "")
			{
				display_lines[line_count] = "Pres.: ";
				display_lines[line_count] += check_display_value(p_value / 100, (-1 / 100.0), 1, 6);
				display_lines[line_count++] += " hPa";
			}
			while (line_count < 3)
			{
				display_lines[line_count++] = emptyString;
			}
			break;
		case 4:
			display_header = "NEO6M";
			display_lines[0] = "Lat: ";
			display_lines[0] += check_display_value(lat_value, -200.0, 6, 10);
			display_lines[1] = "Lon: ";
			display_lines[1] += check_display_value(lon_value, -200.0, 6, 10);
			display_lines[2] = "Alt: ";
			display_lines[2] += check_display_value(alt_value, -1000.0, 2, 10);
			break;
		case 5:
			display_header = FPSTR(SENSORS_DNMS);
			display_lines[0] = std::move(tmpl(F("LAeq: {v} db(A)"), check_display_value(la_eq_value, -1, 1, 6)));
			display_lines[1] = std::move(tmpl(F("LA_max: {v} db(A)"), check_display_value(la_max_value, -1, 1, 6)));
			display_lines[2] = std::move(tmpl(F("LA_min: {v} db(A)"), check_display_value(la_min_value, -1, 1, 6)));
			break;
		case 6:
			display_header = F("Wifi info");
			display_lines[0] = "IP: ";
			display_lines[0] += WiFi.localIP().toString();
			display_lines[1] = "SSID: ";
			display_lines[1] += WiFi.SSID();
			display_lines[2] = std::move(tmpl(F("Signal: {v} %"), String(calcWiFiSignalQuality(last_signal_strength))));
			break;
		case 7:
			display_header = F("Device Info");
			display_lines[0] = "ID: ";
			display_lines[0] += esp_chipid;
			display_lines[1] = "FW: ";
			display_lines[1] += SOFTWARE_VERSION;
			display_lines[2] = F("Measurements: ");
			display_lines[2] += String(count_sends);
			break;
		}

		if (cfg::has_display)
		{
			display.clear();
			display.displayOn();
			display.setTextAlignment(TEXT_ALIGN_CENTER);
			display.drawString(64, 1, display_header);
			display.setTextAlignment(TEXT_ALIGN_LEFT);
			display.drawString(0, 16, display_lines[0]);
			display.drawString(0, 28, display_lines[1]);
			display.drawString(0, 40, display_lines[2]);
			display.setTextAlignment(TEXT_ALIGN_CENTER);
			display.drawString(64, 52, displayGenerateFooter(screen_count));
			display.display();
		}
		if (cfg::has_sh1106)
		{
			display_sh1106.clear();
			display_sh1106.displayOn();
			display_sh1106.setTextAlignment(TEXT_ALIGN_CENTER);
			display_sh1106.drawString(64, 1, display_header);
			display_sh1106.setTextAlignment(TEXT_ALIGN_LEFT);
			display_sh1106.drawString(0, 16, display_lines[0]);
			display_sh1106.drawString(0, 28, display_lines[1]);
			display_sh1106.drawString(0, 40, display_lines[2]);
			display_sh1106.setTextAlignment(TEXT_ALIGN_CENTER);
			display_sh1106.drawString(64, 52, displayGenerateFooter(screen_count));
			display_sh1106.display();
		}
		if (lcd_2004)
		{
			display_header = std::move(String((next_display_count % screen_count) + 1) + '/' + String(screen_count) + ' ' + display_header);
			display_lines[0].replace(" µg/m³", emptyString);
			display_lines[0].replace("°", String(char(223)));
			display_lines[1].replace(" µg/m³", emptyString);
			lcd_2004->clear();
			lcd_2004->setCursor(0, 0);
			lcd_2004->print(display_header);
			lcd_2004->setCursor(0, 1);
			lcd_2004->print(display_lines[0]);
			lcd_2004->setCursor(0, 2);
			lcd_2004->print(display_lines[1]);
			lcd_2004->setCursor(0, 3);
			lcd_2004->print(display_lines[2]);
		}
	}

	// ----5----0----5----0
	// PM10/2.5: 1999/999
	// T/H: -10.0°C/100.0%
	// T/P: -10.0°C/1000hPa

	if (lcd_1602)
	{
		switch (screens[next_display_count % screen_count])
		{
		case 1:
			display_lines[0] = "PM2.5: ";
			display_lines[0] += check_display_value(pm25_value, -1, 1, 6);
			display_lines[1] = "PM10:  ";
			display_lines[1] += check_display_value(pm10_value, -1, 1, 6);
			break;
		case 2:
			display_lines[0] = "PM1.0: ";
			display_lines[0] += check_display_value(pm01_value, -1, 1, 4);
			display_lines[1] = "PM4: ";
			display_lines[1] += check_display_value(pm04_value, -1, 1, 4);
			break;
		case 3:
			display_lines[0] = std::move(tmpl(F("T: {v} °C"), check_display_value(t_value, -128, 1, 6)));
			display_lines[1] = std::move(tmpl(F("H: {v} %"), check_display_value(h_value, -1, 1, 6)));
			break;
		case 4:
			display_lines[0] = "Lat: ";
			display_lines[0] += check_display_value(lat_value, -200.0, 6, 11);
			display_lines[1] = "Lon: ";
			display_lines[1] += check_display_value(lon_value, -200.0, 6, 11);
			break;
		case 5:
			display_lines[0] = std::move(tmpl(F("LAeq: {v} db(A)"), check_display_value(la_eq_value, -1, 1, 6)));
			display_lines[1] = std::move(tmpl(F("LA_max: {v} db(A)"), check_display_value(la_max_value, -1, 1, 6)));
			break;
		case 6:
			display_lines[0] = WiFi.localIP().toString();
			display_lines[1] = WiFi.SSID();
			break;
		case 7:
			display_lines[0] = "ID: ";
			display_lines[0] += esp_chipid;
			display_lines[1] = "FW: ";
			display_lines[1] += SOFTWARE_VERSION;
			break;
		}

		display_lines[0].replace("°", String(char(223)));

		lcd_1602->clear();
		lcd_1602->setCursor(0, 0);
		lcd_1602->print(display_lines[0]);
		lcd_1602->setCursor(0, 1);
		lcd_1602->print(display_lines[1]);
	}
	yield();
	next_display_count++;
}

/*****************************************************************
 * Init BMP280/BME280                                            *
 *****************************************************************/
static bool initBMX280(char addr)
{
	debug_out(String(F("Trying BMP280/BME280 sensor on ")) + String(addr, HEX), DEBUG_MIN_INFO);

	if (bmx280.begin(addr))
	{
		debug_outln_info(FPSTR(DBG_TXT_FOUND));
		bmx280.setSampling(
			BMX280::MODE_FORCED,
			BMX280::SAMPLING_X1,
			BMX280::SAMPLING_X1,
			BMX280::SAMPLING_X1);
		return true;
	}
	else
	{
		debug_outln_info(FPSTR(DBG_TXT_NOT_FOUND));
		return false;
	}
}

/*****************************************************************
   Init SPS30 PM Sensor
 *****************************************************************/
static void initSPS30()
{
	char serial[SPS_MAX_SERIAL_LEN];
	debug_out(F("Trying SPS30 sensor on 0x69H "), DEBUG_MIN_INFO);
	sps30_reset();
	delay(200);
	if (sps30_get_serial(serial) != 0)
	{
		debug_outln_info(FPSTR(DBG_TXT_NOT_FOUND));

		debug_outln_info(F("Check SPS30 wiring"));
		sps30_init_failed = true;
		return;
	}
	debug_outln_info(F(" ... found, Serial-No.: "), String(serial));
	if (sps30_set_fan_auto_cleaning_interval(SPS30_AUTO_CLEANING_INTERVAL) != 0)
	{
		debug_outln_error(F("setting of Auto Cleaning Intervall SPS30 failed!"));
		sps30_init_failed = true;
		return;
	}
	delay(100);
	if (sps30_start_measurement() != 0)
	{
		debug_outln_error(F("SPS30 error starting measurement"));
		sps30_init_failed = true;
		return;
	}
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
static void logEnabledAPIs()
{
	debug_outln_info(F("Send to :"));
	if (cfg::send2dusti)
	{
		debug_outln_info(F("sensor.community"));
	}
	if (cfg::send2cfa)
	{
		debug_outln_info(F("CFA"));
	}

	if (cfg::send2fsapp)
	{
		debug_outln_info(F("Feinstaub-App"));
	}

	if (cfg::send2madavi)
	{
		debug_outln_info(F("Madavi.de"));
	}

	if (cfg::send2csv)
	{
		debug_outln_info(F("Serial as CSV"));
	}

	if (cfg::send2custom)
	{
		debug_outln_info(F("custom API"));
	}

	if (cfg::send2aircms)
	{
		debug_outln_info(F("aircms API"));
	}

	if (cfg::send2influx)
	{
		debug_outln_info(F("custom influx DB"));
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	if (cfg::auto_update)
	{
		debug_outln_info(F("Auto-Update active..."));
	}
}

static void logEnabledDisplays()
{
	if (cfg::has_display || cfg::has_sh1106)
	{
		debug_outln_info(F("Show on OLED..."));
	}
	if (lcd_1602)
	{
		debug_outln_info(F("Show on LCD 1602 ..."));
	}
	if (lcd_2004)
	{
		debug_outln_info(F("Show on LCD 2004 ..."));
	}
}

static unsigned long sendDataToOptionalApis(const String &data)
{
	unsigned long sum_send_time = 0;

	if (cfg::send2madavi)
	{
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("madavi.de: "));
		sum_send_time += sendData(LoggerMadavi, data, 0, HOST_MADAVI, URL_MADAVI);
	}

	if (cfg::send2sensemap && (cfg::senseboxid[0] != '\0'))
	{
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("opensensemap: "));
		String sensemap_path(tmpl(FPSTR(URL_SENSEMAP), cfg::senseboxid));
		sum_send_time += sendData(LoggerSensemap, data, 0, HOST_SENSEMAP, sensemap_path.c_str());
	}

	if (cfg::send2fsapp)
	{
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("Server FS App: "));
		sum_send_time += sendData(LoggerFSapp, data, 0, HOST_FSAPP, URL_FSAPP);
	}

	if (cfg::send2aircms)
	{
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("aircms.online: "));
		unsigned long ts = millis() / 1000;
		String token = WiFi.macAddress();
		String aircms_data("L=");
		aircms_data += esp_chipid;
		aircms_data += "&t=";
		aircms_data += String(ts, DEC);
		aircms_data += F("&airrohr=");
		aircms_data += data;
		String aircms_url(FPSTR(URL_AIRCMS));
		aircms_url += hmac1(sha1Hex(token), aircms_data + token);

		sum_send_time += sendData(Loggeraircms, aircms_data, 0, HOST_AIRCMS, aircms_url.c_str());
	}

	if (cfg::send2influx)
	{
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("custom influx db: "));
		RESERVE_STRING(data_4_influxdb, LARGE_STR);
		create_influxdb_string_from_data(data_4_influxdb, data);
		sum_send_time += sendData(LoggerInflux, data_4_influxdb, 0, cfg::host_influx, cfg::url_influx);
	}

	if (cfg::send2custom)
	{
		String data_to_send = data;
		data_to_send.remove(0, 1);
		String data_4_custom(F("{\"esp8266id\": \""));
		data_4_custom += esp_chipid;
		data_4_custom += "\", ";
		data_4_custom += data_to_send;
		debug_outln_info(FPSTR(DBG_TXT_SENDING_TO), F("custom api: "));
		sum_send_time += sendData(LoggerCustom, data_4_custom, 0, cfg::host_custom, cfg::url_custom);
	}

	if (cfg::send2csv)
	{
		debug_outln_info(F("## Sending as csv: "));
		send_csv(data);
	}

	return sum_send_time;
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
