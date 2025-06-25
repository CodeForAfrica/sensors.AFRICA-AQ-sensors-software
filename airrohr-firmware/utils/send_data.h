#ifndef _SEND_DATA_H
#define _SEND_DATA_H

// Function declarations
static unsigned long sendData(const LoggerEntry logger, const String &data, const int pin, const char *host, const char *url);
static unsigned long sendCFA(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str);
static unsigned long sendSensorCommunity(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str);
static void create_influxdb_string_from_data(String &data_4_influxdb, const String &data);
static unsigned long sendDataToOptionalApis(const String &data);
static void send_csv(const String &data);

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
            return 0;
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

        // debug_out(F("Start connecting via GPRS"), DEBUG_MIN_INFO);
        // debug_out(F("HOST "), DEBUG_MIN_INFO);
        // debug_out(s_Host, DEBUG_MIN_INFO);
        // debug_out(F("URL "), DEBUG_MIN_INFO);
        // debug_out(s_url, DEBUG_MIN_INFO);
        // debug_out(gprs_request_head, DEBUG_MIN_INFO);

#ifdef QUECTEL
        String Quectel_headers[3];
        Quectel_headers[0] = "X-PIN: " + String(pin);
        Quectel_headers[1] = "X-Sensor: esp8266-" + esp_chipid;
        // Quectel_headers[1] = "X-Sensor: esp8266-quectel-test";       // testing node, comment and insert desired testing node ID
        Quectel_headers[2] = "Content-Type: " + String(contentType); // 30

        int header_size = sizeof(Quectel_headers) / sizeof(Quectel_headers[0]);

#endif

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
        Serial.println("GPRS REQUEST HEAD:");
        Serial.println(gprs_request_head);
        Serial.println();
        flushSerial();
        debug_out(F("## Sending via gsm\n\n"), DEBUG_MIN_INFO);

#ifdef QUECTEL
        QUECTEL_POST((char *)gprs_url, Quectel_headers, header_size, data, data.length(), statuscode);
        // ToDo: close HTTP session/ PDP context
#else
        if (!fona.HTTP_POST_start((char *)gprs_url, F("application/json"), gprs_request_head, (uint8_t *)gprs_data, strlen(gprs_data), &statuscode, (uint16_t *)&length))
        {
            debug_outln_error(F("Failed with status code "));
            debug_out(String(statuscode), DEBUG_ERROR); // !ERROR not handled correctly: POST in most cases is successul but the status code !=200
            disableGPRS();
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
        disableGPRS();
#endif

        if (!(statuscode == 200 || statuscode == 201))
        {
            StaticJsonDocument<255> doc;
            deserializeJson(doc, data);
            doc["API_PIN"] = pin;

            String serializedData;
            serializeJson(doc, serializedData);

            File fileDataLogger = SPIFFS.open(SENSORS_FAILED_DATA_SEND_STORE_FILE, "a");
            if (!fileDataLogger)
            {
                Serial.println("Error opening spiffs to append to data logger");
            }

            if (fileDataLogger.println(serializedData))
            {
                Serial.println("Failed-to-send-data appended");
            }
            else
            {
                Serial.println("File to append to data logger");
            }
        }
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
#endif