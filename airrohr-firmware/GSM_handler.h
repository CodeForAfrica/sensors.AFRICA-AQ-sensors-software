#define MCU_RXD D5
#define MCU_TXD D6
#define QUECTEL_PWR_KEY D0
#define QUECTEL_DTR D9
#define GSM_RST_PIN D8
#include <SoftwareSerial.h>
SoftwareSerial GSMSerial(MCU_RXD, MCU_TXD);

/// @brief Reset sequence enumeration for GSM module reset control
enum RST_SEQ
{
    HIGH_LOW_HIGH, ///< Reset sequence: HIGH -> LOW -> HIGH
    LOW_HIGH_LOW   ///< Reset sequence: LOW -> HIGH -> LOW
};

char SIM_PIN[5] = GSM_PIN;
bool GSM_CONNECTED = false;
bool SIM_AVAILABLE = false;
bool GPRS_CONNECTED = false;
bool SIM_PIN_SET = false;
bool SIM_USABLE = false;
uint16_t CGATT_status;
char SIM_CCID[21] = "";
String GSM_INIT_ERROR = "";
String NETWORK_NAME = "";

// FAIL FLAGS
#ifdef QUECTEL
int HTTPCFG_CONNECT_FAIL = 0;
#endif
int GPRS_INIT_FAIL_COUNT = 0;
int HTTP_POST_FAIL = 0;
int REGISTER_TO_NETWORK_FAIL = 0;

uint16_t HTTPOST_RESPONSE_STATUS;

/// @brief Network mode enumeration for Quectel modem
enum NetMode
{
    AUTO = 0, ///< Automatic network mode selection
    _2G = 1,  ///< 2G network mode
    _4G = 3   ///< 4G network mode
};
NetMode current_network = NetMode::AUTO;

/// @brief Network registration status enumeration (values 0-5)
enum NetRegStatus
{
    NOT_REGISTERED,             ///< Not registered to any network
    REGISTERED_TO_HOME_NETWORK, ///< Registered to home network
    SEARCHING,                  ///< Searching for network
    REGISTRATION_DENIED,        ///< Network registration denied
    UNKNOWN,                    ///< Unknown registration status
    REGISTERED_ROAMING          ///< Registered to roaming network
};

/// @brief Human-readable network registration status descriptions
static const char *const NET_STATUS_VERBOSE[] = {
    "Not registered to network",
    "Registered to home network",
    "Searching for network",
    "Network registration denied",
    "Network registration status unknown",
    "Registered to roaming network"};

struct GSMRuntimeInfo
{
    String operator_name;       // Network operator name
    int8_t signal_strength;     // RSSI //!not in dBm: need mapping
    String network_band[16];    // e.g., "LTE Band 7"
    char network_technology[8]; // "2G", "3G", "4G"
    char sim_ccid[21];
    String imei;
    String firmware_version;
    String model_id; // EC200XXXXXXX
};

enum GSMMQTTConnStatus
{
    MQTT_DISCONNECTED = 0,
    MQTT_BROKER_OPEN = 1,
    MQTT_CONNECTED = 2
};

GSMMQTTConnStatus mqtt_status = MQTT_DISCONNECTED;
int MQTT_INIT_FAIL_COUNT = 0;
int MQTT_PUB_FAIL = 0;
String MQTT_INIT_ERROR = "";

bool GSM_init();
bool register_to_network();
void SIM_PIN_Setup();
bool is_SIMCID_valid();
bool GPRS_init();
void GSM_soft_reset();
void restart_GSM();
bool activateGPRS();
bool deactivateGPRS();
int8_t GPRS_status();
void flushSerial();
void SerialFlush();
void QUECTEL_POST(const char *url, char headers[][256], int header_size, const char *data, size_t data_length, int &response_status);
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until);
void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool wait_timeout = false, unsigned long timeout = 3000);
int16_t getNumber(const char *AT_cmd, const char *expected_reply, uint8_t index_from, uint8_t length);
void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS);
bool sendAndCheck(const char *AT_cmd, const char *expected_reply = "OK", unsigned long timeout = 10000);
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, String &response, unsigned long timeout = 10000);
bool waitForReply(const char *expectedReply, unsigned long timeout);
bool waitForReply(const char *expectedReply, String &buffer, unsigned long timeout);
bool waitForURC(const char *urcPrefix, char *response, size_t responseLen, unsigned long timeout);
bool GSM_Serial_begin();
bool getNetworkTime(char *time);
void GSMreset(RST_SEQ seq, uint8_t timing_delay = 120);
bool reset_http_config();
bool http_preconfig();
bool https_preconfig();
void GSM_sleep();
void troubleshoot_GSM();
String getNetworkName();
int8_t getSignalStrength();
String getNetworkBand();
bool setNetworkMode(NetMode mode);
void cycleNetworkMode();
String getFirwmareVersion();
String getModelID();
String getProductInfo();
bool pingIP(const char *host, uint8_t contextID = 1);
bool setCACert(const char *certPath, uint8_t sslCtxID = 1);
bool GsmFileCheck(const char *filename);
String getBatteryStatus();

// MQTT Functions
bool MQTT_configure(uint8_t client_id = 0, uint8_t recv_mode = 0, uint8_t msg_len = 1);
bool MQTT_open(uint8_t client_id, const char *broker, uint16_t port);
bool MQTT_connect(uint8_t client_id, const char *clientid, const char *username = nullptr, const char *password = nullptr);
bool MQTT_subscribe(uint8_t client_id, uint16_t msg_id, const char *topic, uint8_t qos = 0);
bool MQTT_publish(uint8_t client_id, uint16_t msg_id, const char *topic, const char *payload, uint8_t qos = 0, uint8_t retain = 0);
bool MQTT_unsubscribe(uint8_t client_id, uint16_t msg_id, const char *topic);
bool MQTT_disconnect(uint8_t client_id);
bool MQTT_isBrokerConnected(uint8_t client_id);
GSMMQTTConnStatus MQTT_getStatus(uint8_t client_id);
bool MQTT_isClientConnected(uint8_t client_id);
bool MQTT_hasBufferedMessage(uint8_t client_id);
bool MQTT_readBufferedMessage(uint8_t client_id, char *topic_out, size_t topic_size, char *payload_out, size_t payload_size);
bool MQTT_readBufferedMessage(uint8_t client_id, uint8_t recv_id, char *topic_out, size_t topic_size, char *payload_out, size_t payload_size);

/// @brief Initialize GSM module and SIM card
/// @return true if GSM initialization successful
bool GSM_init()
{
    String error_msg = "";

    Serial.println("Restarting GSM...");

#ifdef GSM_RST_PIN

    GSMreset(RST_SEQ::LOW_HIGH_LOW);
#else
    GSM_soft_reset();

#endif

    // Check if SIM is inserted
    if (!is_SIMCID_valid())
    {
        error_msg = "Could not get SIM CID";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        return false;
    }

    // Serial.println("Setting up SIM..");
    // SIM_PIN_Setup();
    // if (!SIM_PIN_SET)
    // {
    //     error_msg = "Unable to set SIM PIN";
    //     GSM_INIT_ERROR = error_msg;
    //     Serial.println(error_msg);
    //     return false;
    // }

    // Set if SIM is usable flag
    SIM_USABLE = true;

    return true;
}

/// @brief Query modem firmware version
/// @return Firmware version string
String getFirwmareVersion()
{
    String res;
    if (!sendAndCheck("AT+GMR", "OK", res))
    {
        return "";
    }
    return res.substring(res.indexOf("\n") + 1, res.indexOf("OK") - 3);
}

/// @brief Query modem model identifier
/// @return Model ID string
String getModelID()
{
    String res;
    if (!sendAndCheck("AT+GMM", "OK", res))
    {
        return "";
    }
    return res.substring(res.indexOf("\n") + 1, res.indexOf("OK") - 3);
}

/// @brief Query complete product information
/// @return Product info string
String getProductInfo() // Combination of AT+GMI, AT+GMM and AT+GMR,
{
    String res;
    if (!sendAndCheck("ATI", "OK", res))
    {
        return "";
    }
    return res.substring(res.indexOf("\n") + 1, res.indexOf("OK") - 3);
}

/// @brief Query IMEI : AT+GSN=1
/// @return IMEI string
String getIMEI()
{
    String res;
    if (!sendAndCheck("AT+GSN=1", "OK", res))
    {
        return "";
    }
    return res.substring(res.indexOf("\"") + 1, res.indexOf("OK") - 5);
}

/// @brief Set modem network mode preference
/// @param mode Network mode (AUTO, 2G, or 4G)
/// @return true if mode set successfully
bool setNetworkMode(NetMode mode)

{
    char setnetmode[24] = "AT+QCFG=\"nwscanmode\",";
    char _mode[1];
    itoa(mode, _mode, 10);

    strcat(setnetmode, _mode);

    char mode_str[8];
    switch (mode)
    {
    case (NetMode::_2G):
        strcpy(mode_str, "2G");
        break;
    case (NetMode::_4G):
        strcpy(mode_str, "4G");
        break;
    case (NetMode::AUTO):
        strcpy(mode_str, "AUTO");
        break;
    }

    Serial.print("Setting network mode to: ");
    Serial.println(mode_str);

    if (!sendAndCheck(setnetmode, "OK", 2000))
    {
        Serial.print("Failed to set network mode: ");
        Serial.println(mode_str);
        return false;
    }
    delay(1000);
    current_network = mode;
    return true;
}

/// @brief Cycle through available network modes
void cycleNetworkMode()
{
    bool set_mode = setNetworkMode(current_network);
    // Cycle to the next network mode (AUTO -> 2G -> 4G -> AUTO)
    switch (current_network)
    {
    case (NetMode::AUTO):
        current_network = NetMode::_2G;
        break;
    case (NetMode::_2G):
        current_network = NetMode::_4G;
        break;
    case (NetMode::_4G):
        current_network = NetMode::AUTO;
        break;
    default:
        current_network = NetMode::AUTO;
        break;
    }

    if (!set_mode) // Attempt setting the net mode
        cycleNetworkMode();
}

/// @brief Register device to mobile network
/// @return true if successfully registered
bool register_to_network()
{

    String error_msg = "";
    bool registered_to_network = false;
    int retry_count = 0;
    int8_t status;
    // cycleNetworkMode(); //! Only useful when register_to_network is called [1] in while loop either infinetely or timeout longer than time take for the function to excute; or [2] called multiple times in the code; otherwise, can be removed to save time during initialization. To be reviewed and refactored in future iterations.

    if (!sendAndCheck("AT+CREG=1\0", "OK"))
    {
        Serial.println("Manual network registration failed.");
    }

    do
    {
        status = getNumber("AT+CREG?\0", "+CREG: ", 2, 1);
        if (status == NetRegStatus::REGISTERED_TO_HOME_NETWORK || status == NetRegStatus::REGISTERED_ROAMING)
        {
            registered_to_network = true;
            Serial.printf("%s: %s\n", NET_STATUS_VERBOSE[status], getNetworkName().c_str());
            return registered_to_network;
        }
        else
        {
            Serial.println(NET_STATUS_VERBOSE[status]);
        }
        retry_count++;
        delay(3000);

    } while (!registered_to_network && retry_count < 20);

    return registered_to_network;
}

// static void unlock_pin(char *PIN)
// {

//     // Attempt to SET PIN if not empty
//     Serial.print("GSM CONFIG SET PIN: ");
//     Serial.println(PIN);
//     // Serial.print("Length of PIN");
//     Serial.println(strlen(PIN));
//     if (strlen(PIN) > 1)
//     {
//         SIM_PIN_SET = true;
//     }
// }

/// @brief Configure SIM PIN settings
void SIM_PIN_Setup()
{

    if (sendAndCheck("AT+CPIN?", "+CPIN: READY", 3000))
    {
        Serial.println("SIM PIN READY");
        SIM_PIN_SET = true;
        return;
    }

    else
    {
        Serial.println("SIM PIN NOT SET");
        return;
        // ToDO:Set PIN
    }
}

/// @brief Validate SIM card CCID (Integrated Circuit Card ID)
/// @return true if SIM is valid and present
bool is_SIMCID_valid()
{
    // TODO: Verify function returns false correctly when there is "ERROR" in response
    char qccid[21];
    String AT_response = "";

    char expected_reply[] = "+QCCID: ";
    if (!sendAndCheck("AT+QCCID\0", "OK", AT_response))
        return false;

    if (extractText((char *)AT_response.c_str(), expected_reply, qccid, 21, '\r') && strlen(qccid) == 20)
    {
        strcpy(SIM_CCID, qccid);
        SIM_AVAILABLE = true;
        return SIM_AVAILABLE;
    }
    else
    {
        return false;
    }
}

/// @brief Initialize GPRS data connection
/// @return true if GPRS successfully initialized
bool GPRS_init()
{
    String err = "";

#ifdef QUECTEL
    Serial.println("Quectel GPRS init...");

    int timeout = 5000;
    Serial.println("Configuring PDP context ");
    bool PDP_config = false;
    while (timeout > 0)
    {

        PDP_config = sendAndCheck("AT+QICSGP=1,1\0", "OK");

        if (PDP_config)
        {
            Serial.println("PDP context set");
            break;
        }
        Serial.print(".");
        timeout -= 1000;
        delay(2000);
    }

    if (!PDP_config)
    {
        err = "Failed to config GPRS PDP context";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        return false;
    }

    // Check CGATT status
    Serial.println("\nChecking CGATT Status..");
    CGATT_status = GPRS_status();
    // Serial.println("CGATT_status: " + (String)CGATT_status);

    if (CGATT_status == 1)
    {
        GPRS_CONNECTED = true;
        GPRS_INIT_FAIL_COUNT = 0;
    }

    // Attach CGATT
    else
    {

        if (activateGPRS())
        {
            delay(2000);
            CGATT_status = getNumber("AT+CGATT?\0", "+CGATT: ", 0, 1);
            if (CGATT_status == 1)
                GPRS_CONNECTED = true;
        }
        else
        {
            // TODO: CGATT status sometimes not reached when using if statement; delay needed
            Serial.println("CGATT status set to: " + (String)CGATT_status);
        }
    }

    // TODO: Implement QIACT configuration

#else
    // Alternative GPRS commands for other modules:
    // "AT+SAPBR=1,1"
    // "AT+QCFG=\"gprsattach\",1"
#endif
    if (!GPRS_CONNECTED)
    {
        Serial.println("Failed to init GPRS");
        GPRS_INIT_FAIL_COUNT += 1;
    }
    else
    {
        Serial.println("GPRS initialized!");
    }
    return GPRS_CONNECTED;
}

/// @brief Perform soft reset of GSM module with AT commands
void GSM_soft_reset()
{
    deactivateGPRS();

    if (!sendAndCheck("AT+CFUN=1,1", "OK"))
    {
        Serial.println("Soft resetting GSM with full functionality failed!");
        return;
    }
    Serial.println("Soft resetting the GSM module...");
    delay(30000); // wait for GSM to warm up
}

/// @brief Restart GSM module (full initialization)
/// @note Called in multiple locations; implementation should be reviewed
/// @todo Refactor to shut down GSM and then call GSM_init() for cleaner reset
void restart_GSM()
{
    Serial.println("Restarting GSM");
    // Note: The AQ PCB board has the GSM reset physically connected to the ESP chip

    if (!GSM_init())
    {
        Serial.println("GSM not fully configured");
        Serial.print("Failure point: ");
        Serial.println(GSM_INIT_ERROR);
        Serial.println();
    }
}

/// @brief Flush GSM serial buffer
void flushSerial()
{
    while (GSMSerial.available())
    {
        GSMSerial.read();
    }
}

/// @brief Check if a file exists on the GSM module's filesystem
/// @param filename Name of the file to check
bool GsmFileCheck(const char *filename)
{

    char cmd[64];
    // AT+QFLST=<name_pattern>
    snprintf(cmd, sizeof(cmd), "AT+QFLST=\"UFS:%s\"", filename);
    return sendAndCheck(cmd, filename);
}

/// @brief Write content to a file on the GSM module's filesystem
/// @param filename Name of the file to write to
/// @param content Content to write to the file
/// @return true if file was successfully written
bool GsmWriteFile(const char *filename, const char *content)
{
    bool file_written = false;

    String file_handle_number = "";
    char cmd[64];
    snprintf(cmd, 64, "AT+QFOPEN=\"%s\"", filename);
    if (!sendAndCheck(cmd, "OK", file_handle_number))
    {
        Serial.println("Could not open file for writing");
        Serial.printf("QFOPEN response: %s\n", file_handle_number.c_str());
        return false;
    }
    else
    {

        file_handle_number = file_handle_number.substring(file_handle_number.indexOf("+QFOPEN:"));
        file_handle_number = file_handle_number.substring(file_handle_number.indexOf(":") + 1, file_handle_number.indexOf("\r"));
        Serial.printf("QFOPEN substring:%s", file_handle_number.c_str());
        file_handle_number = file_handle_number.toInt();
        Serial.printf("Extracted file handle num: %s", file_handle_number.c_str());

        size_t file_size = strlen(content);
        Serial.print("File size: ");
        Serial.println(file_size);

        snprintf(cmd, 64, "AT+QFWRITE=%s,%d", file_handle_number.c_str(), file_size);
        if (!sendAndCheck(cmd, "CONNECT"))
        {
            Serial.println("Failed to initiate file write");
            return false;
        }
        else
        {

            size_t sentbytes = GSMSerial.write(content, file_size);

            if (sentbytes != file_size)
            {
                Serial.printf("Failed to write to file bytes %d. Only %d bytes written", file_size, sentbytes);
            }
            else
            {
                file_written = true;
            }

            snprintf(cmd, 64, "AT+QFSEEK=%s,0,0", file_handle_number.c_str());
            if (!sendAndCheck(cmd))
            {
                Serial.println("Failed to set seek position");
            }
            // Read the file
            snprintf(cmd, 64, "AT+QFREAD=%s,%d", file_handle_number.c_str(), file_size);
            if (sendAndCheck(cmd, "CONNECT"))
            {
                String res = GSMSerial.readString();
                Serial.printf("QFREAD response: %s\n", res.c_str());
            }

            // close the file
            snprintf(cmd, 64, "AT+QFCLOSE=%s", file_handle_number.c_str());
            if (!sendAndCheck(cmd))
            {
                Serial.printf("Failed to close file: %s\n", filename);
            }
        }
    }

    return file_written;
}

bool setCACert(const char *certPath, uint8_t sslCtxID)
{
    char cmd[64]; // e.g. AT+QSSLCFG="cacert",1,"UFS:cacert.pem"
    snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"cacert\",%d,\"UFS:%s\"", sslCtxID, certPath);
    return sendAndCheck(cmd);
}

/// @brief Preconfigure HTTP settings
bool reset_http_config()
{
    String resp;
    if (!sendAndCheck("AT+QHTTPCFG=\"reset\"", "OK", resp, 5000))
    {
        Serial.println("Failed to reset HTTP(S) config");
        Serial.println(resp);
        return false;
    }
    Serial.println(resp);
    return true;
}

/// @brief Preconfigure shared HTTP(S) settings
bool http_preconfig()
{
    String resp;
    bool configured = true;

    configured &= sendAndCheck("AT+QHTTPCFG=\"contextid\",1", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QHTTPCFG=\"requestheader\",0", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QHTTPCFG=\"responseheader\",1", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QHTTPCFG=\"rspout/auto\",0", "OK", resp);
    Serial.println(resp);

    if (!configured)
    {
        Serial.println("Failed to configure shared HTTP(S) settings");
    }

    return configured;
}

/// @brief Preconfigure HTTPS settings for secure connections
bool https_preconfig()
{
    String resp;
    bool configured = true;

    configured &= sendAndCheck("AT+QHTTPCFG=\"sslctxid\",1", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QSSLCFG=\"sslversion\",1,3", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QSSLCFG=\"ciphersuite\",1,0XFFFF", "OK", resp); // Enable all ciphersuites; can be configured to specific ciphersuites if needed
    Serial.println(resp);
    configured &= sendAndCheck("AT+QSSLCFG=\"seclevel\",1,0", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QSSLCFG=\"ignorelocaltime\",1,1", "OK", resp);
    Serial.println(resp);
    configured &= sendAndCheck("AT+QSSLCFG=\"sni\",1,1", "OK", resp);
    Serial.println(resp);
    // check current Certificate configuration
    // char cmd[64];
    // snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"cacert\",1"); // Query current CA cert config for SSL context ID 2

    // String res;
    // if (sendAndCheck(cmd, "OK", res))
    // {
    //     Serial.printf("Current CA cert config: %s\n", res.c_str());
    // }

    if (!configured)
        Serial.println("Failed to configure HTTPS SSL settings");

    return configured;
}

/// @brief Send HTTP POST request via Quectel module
/// @param url Target URL including http:// or https://
/// @param headers Array of HTTP headers
/// @param header_size Number of headers
/// @param data Request body data
/// @param data_length Length of request body
/// @param response_status HTTP response status code
void QUECTEL_POST(const char *url, char headers[][256], int header_size, const char *data, size_t data_length, int &response_status)
{
    response_status = 0;
    String resp;
    bool is_https = (strncmp(url, "https://", 8) == 0);

    if (!reset_http_config())
    {
        HTTPCFG_CONNECT_FAIL += 1;
        return;
    }

    if (!http_preconfig())
    {
        HTTPCFG_CONNECT_FAIL += 1;
        return;
    }

    if (is_https)
    {

        Serial.println("HTTPS URL detected; SSL context enabled");
        if (!https_preconfig())
        {
            HTTPCFG_CONNECT_FAIL += 1;
            return;
        }
    }

    char HTTP_CFG[384] = {};
    int cfg_len = snprintf(HTTP_CFG, sizeof(HTTP_CFG), "AT+QHTTPCFG=\"url\",\"%s\"", url);
    if (cfg_len < 0 || cfg_len >= (int)sizeof(HTTP_CFG))
    {
        Serial.println("HTTP URL config command too long");
        HTTPCFG_CONNECT_FAIL += 1;
        return;
    }

    if (!sendAndCheck(HTTP_CFG, "OK", resp, 2000))
    {
        Serial.println("Failed to set HTTP(S) URL");
        Serial.println(resp);
        HTTPCFG_CONNECT_FAIL += 1;
        return;
    }
    Serial.println(resp);

    // Setting request headers
    // Headers sent in format 0: headers are sent before post body
    // (Format 1 would send headers as part of the body)

    for (int i = 0; i < header_size; i++)
    {
        cfg_len = snprintf(HTTP_CFG, sizeof(HTTP_CFG), "AT+QHTTPCFG=\"header\",\"%s\"", headers[i]);
        if (cfg_len < 0 || cfg_len >= (int)sizeof(HTTP_CFG))
        {
            Serial.println("HTTP header config command too long");
            HTTPCFG_CONNECT_FAIL += 1;
            return;
        }

        Serial.println("Setting header: " + String(headers[i]));
        if (sendAndCheck(HTTP_CFG, "OK"))
        {
            Serial.println("Header set successfully");
        }
        else
        {
            Serial.println("Failed to set header");
            return;
        }
    }

    char HTTP_POST_RESPONSE_STATUS[4] = "000";

    // Prepare POST request
    char http_post_prepare[40] = {};
    snprintf(http_post_prepare, sizeof(http_post_prepare), "AT+QHTTPPOST=%lu,30,60", (unsigned long)data_length);

    Serial.println(http_post_prepare);
    // Allow sufficient time to connect to HTTP(S) server
    if (sendAndCheck(http_post_prepare, "CONNECT", resp, 10000))
    {
        Serial.println("Posting gprs data..");
        get_http_response_status(data, HTTP_POST_RESPONSE_STATUS);
        response_status = atoi(HTTP_POST_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("HTTP POST CONNECT FAIL");
        HTTPCFG_CONNECT_FAIL += 1;
        Serial.println(resp);
        return;
    }

    if (response_status >= 200 && response_status < 300)
    {
        Serial.println("Requested processed successfully with status: " + (String)HTTP_POST_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("Requested processing failed with status: " + (String)HTTP_POST_RESPONSE_STATUS);
        HTTP_POST_FAIL += 1;
    }
}

/// @brief Flush ESP serial buffer
void SerialFlush()
{
    while (Serial.available())
    {
        Serial.read();
    }
}

/// @brief Send AT command and receive raw response
/// @param cmd AT command to send
/// @param res_buff Buffer to store response
/// @param buff_size Size of response buffer
/// @param wait_timeout If true, wait for timeout; if false, return when data received
/// @param timeout Maximum time to wait in milliseconds
void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool wait_timeout, unsigned long timeout)
{
    flushSerial();
    memset(res_buff, '\0', buff_size);
    size_t buff_pos = 0;
    Serial.print("Received Command: ");
    Serial.println(cmd);
    GSMSerial.println(cmd);
    unsigned long sendStartMillis = millis();
    do
    {
        if (buff_pos >= buff_size) // Check if buff is full
            break;

        while (GSMSerial.available())
        {
            if (buff_pos >= buff_size)
                break;
            res_buff[buff_pos] = GSMSerial.read();
            buff_pos++;
        }

        delay(2);
    } while ((wait_timeout || (buff_pos == 0)) && (millis() - sendStartMillis < timeout));
    Serial.println("\n-------\r\nGSM RAW RESPONSE:");
    Serial.println(res_buff);
    Serial.println("-------");
}

/// @brief Extract text between target string and delimiter in char array
/// @param input Char array containing text to parse
/// @param target Target string to search for
/// @param output Char array to store extracted string
/// @param output_size Size of output buffer
/// @param _until Delimiter character marking extraction end
/// @return true if extraction successful, false otherwise
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until)
{

    const char *found_target = strstr(input, target);

    if (found_target != nullptr)
    {
        // Find the start of the extraction point
        const char *start = found_target + strlen(target);

        // Find the end of the extraction point (the next comma by default)
        const char *end = strchr(start, _until);

        if (end != nullptr)
        {
            // Calculate the length of the text to be extracted
            size_t length = end - start;

            if (length < output_size)
            {
                strncpy(output, start, length);
                output[length] = '\0';
                return true;
            }
            else
            {
                Serial.println("Extracted piece of text longer than ouput size");
                return false;
            }
        }
    }
    // Serial.println("Could not extact substring '" + (String)target + "' from the source");
    return false;
}

/// @brief Extract numeric value from AT command response
/// @param AT_cmd AT command to send
/// @param expected_reply Expected response prefix
/// @param index_from Index offset from expected_reply to start extraction
/// @param length Length of number to extract
/// @return Extracted number or -1 on error
int16_t getNumber(const char *AT_cmd, const char *expected_reply, uint8_t index_from, uint8_t length)
{
    int16_t num;
    char number[8];

    if (length > sizeof(number))
    {
        Serial.println("Max length allowed is 8");
        return -1;
    }

    String AT_response = "";

    if (!sendAndCheck(AT_cmd, "OK", AT_response))
        return -1;

    const char *found_target = strstr(AT_response.c_str(), expected_reply);

    if (found_target == nullptr)
        return -1;

    // Find extraction start point
    const char *start = found_target + strlen(expected_reply);
    start += index_from; // Offset into response (e.g., to extract 5 from +CREG: 0,5,7)

    if (length < sizeof(number))
    {
        strncpy(number, start, length);
        number[length] = '\0';
    }

    num = atoi(number);
    return num;
}

/// @brief Send AT command and verify expected response
/// @param AT_cmd AT command to send
/// @param expected_reply Expected response string
/// @param timeout Maximum time to wait in milliseconds
/// @return true if expected reply received
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, unsigned long timeout)
{
    flushSerial();
    GSMSerial.println(AT_cmd);
    return waitForReply(expected_reply, timeout);
}

/// @brief Send AT command and capture full response
/// @param AT_cmd AT command to send
/// @param expected_reply Expected response string
/// @param response String buffer to store full response
/// @param timeout Maximum time to wait in milliseconds
/// @return true if expected reply received
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, String &response,
                  unsigned long timeout)
{
    flushSerial();
    GSMSerial.println(AT_cmd);

    return waitForReply(expected_reply, response, timeout);
}

/// @brief Wait for specific reply string
/// @param expectedReply String to wait for
/// @param timeout Maximum time to wait in milliseconds
/// @return true if reply received
bool waitForReply(const char *expectedReply, unsigned long timeout)
{
    unsigned long start = millis();
    String buffer = "";

    while (millis() - start < timeout)
    {
        while (GSMSerial.available())
        {
            char c = GSMSerial.read();
            buffer += c;

            if (buffer.indexOf(expectedReply) >= 0)
                return true;

            // Maintain manageable buffer size
            if (buffer.length() > 256)
                buffer = buffer.substring(buffer.length() - 128);
        }
    }
    return false;
}

/// @brief Wait for reply and capture response
/// @param expectedReply String to wait for
/// @param buffer String buffer to store response
/// @param timeout Maximum time to wait in milliseconds
/// @return true if reply received
bool waitForReply(const char *expectedReply, String &buffer, unsigned long timeout)
{
    unsigned long start = millis();
    buffer = "";

    while (millis() - start < timeout)
    {
        while (GSMSerial.available())
        {
            char c = GSMSerial.read();
            buffer += c;

            if (buffer.indexOf(expectedReply) >= 0)
                return true;

            if (buffer.length() > 256)
                buffer = buffer.substring(buffer.length() - 128);
        }
    }
    return false;
}

/// @brief Wait for unsolicited result code (URC)
/// @param urcPrefix URC prefix to search for
/// @param response Buffer for URC response
/// @param responseLen Size of response buffer
/// @param timeout Maximum time to wait in milliseconds
/// @return true if URC received
bool waitForURC(const char *urcPrefix, char *response,
                size_t responseLen, unsigned long timeout)
{
    unsigned long start = millis();
    String buffer = "";

    while (millis() - start < timeout)
    {
        while (GSMSerial.available())
        {
            char c = GSMSerial.read();
            buffer += c;

            if (c == '\n')
            {
                if (buffer.indexOf(urcPrefix) >= 0)
                {
                    strncpy(response, buffer.c_str(), responseLen - 1);
                    response[responseLen - 1] = '\0';
                    return true;
                }
                buffer = "";
            }

            if (buffer.length() > 256)
            {
                buffer = "";
            }
        }
    }
    return false;
}

/// @brief Parse HTTP response status from data
/// @param data Response data to parse
/// @param HTTP_RESPONSE_STATUS Buffer for status code
void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS)
{
    strcpy(HTTP_RESPONSE_STATUS, "000");
    const char *data_copy = data.c_str();
    char gprs_data[strlen(data_copy) + 1];
    strcpy(gprs_data, data_copy);

    // Check HTTP RESPONSE status (0 = Operation successful)
    const char *expected_reply = "+QHTTPPOST: 0,";
    sendAndCheck(gprs_data);

    char qurc[32];
    // Note: Space after colon is significant for URC matching
    if (!waitForURC("+QHTTPPOST: ", qurc, sizeof(qurc), 10000))
    {
        Serial.println("HTTP POST QURC not received!");
        return;
    }
    // Try extracting with comma first, then newline if comma fails
    // URC response may be +QHTTPPOST: 0,201\r\n (no length) or +QHTTPPOST: 0,201,83 (with length)
    if (extractText(qurc, expected_reply, HTTP_RESPONSE_STATUS, 4, ',') ||
        extractText(qurc, expected_reply, HTTP_RESPONSE_STATUS, 4, '\r'))
    {
        Serial.print("HTTP(S) responsestatus code: ");
        Serial.println(HTTP_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("Could not extract HTTP response status code");
    }
}

/// @brief Troubleshoot GSM connectivity issues
void troubleshoot_GSM()
{
    // Note: Use GSM_soft_reset() if GSM reset pin is not connected
    GSM_init();

    register_to_network();

    GPRS_init();

    // RESET FLAGS
    HTTPCFG_CONNECT_FAIL = 0;
    HTTP_POST_FAIL = 0;
    GPRS_INIT_FAIL_COUNT = 0;
}

/// @brief Query current GPRS attachment status
/// @return 1 if attached, 0 if detached, -1 on error
int8_t GPRS_status()
{
    return getNumber("AT+CGATT?\0", "+CGATT: ", 0, 1);
}

/// @brief Activate GPRS connection
/// @return true if activation successful
bool activateGPRS()
{
    if (GPRS_status() == 1)
    {
        Serial.println("GPRS already active");
        return true;
    }
    if (sendAndCheck("AT+CGATT=1", "OK"))
    {

        return true;
    }
    else
    {
        Serial.println("Failed to enable GPRS");
        return false;
    }
}

/// @brief Deactivate GPRS connection
/// @return true if deactivation successful
bool deactivateGPRS()
{
    if (GPRS_status() == 0)
    {
        Serial.println("GPRS already inactive");
        return true;
    }

    if (sendAndCheck("AT+CGATT=0", "OK"))
    {
        GPRS_status();
        return true;
    }

    Serial.println("Failed to disable GPRS");
    return false;
}

/// @brief Query module real-time clock (RTC)
/// @param time Output buffer for time string in format "yy/MM/dd,hh:mm:ss±zz"
///        - Year (2 digits), Month, Day, Hour, Minute, Second
///        - Timezone as quarter-hour offset from GMT (range: -48 to +56)
///        - Example: May 6, 1994 22:10:00 GMT+2 = "94/05/06,22:10:00+08"
/// @return true if time is successfully extracted, false otherwise
bool getNetworkTime(char *time)
{
    String AT_response = "";
    char time_buff[23] = {};
    uint8_t retries = 0;

    sendAndCheck("AT+CCLK?", "OK", AT_response);

    while (!extractText((char *)AT_response.c_str(), "+CCLK: \"", time_buff, 32, '\"') && retries < 10)
    {
        sendAndCheck("AT+CCLK?", "OK", AT_response);
        retries++;
        delay(1000);
    }

    String time_str = String(time_buff);

    // Validate time format
    if (time_str.charAt(2) == '/' && time_str.charAt(5) == '/' &&
        time_str.charAt(8) == ',' && time_str.charAt(11) == ':' && time_str.charAt(14) == ':')
    {
        strcpy(time, time_buff);
        return true;
    }

    return false;
}

/// @brief Query current network operator name
/// @return Network name string
String getNetworkName()
{
    String AT_response = "";
    const char AT_cmd[] = "AT+QSPN";
    char NetworkName[64];

    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return "";

    if (extractText((char *)AT_response.c_str(), "+QSPN: \"", NetworkName, 64, '"'))
    {
        NETWORK_NAME = String(NetworkName);
        return NETWORK_NAME;
    };

    NETWORK_NAME = "";
    return NETWORK_NAME;
}
/// @brief Query signal strength from network
/// @return RSSI value or 99 on error //! Integer indicator. Not actual RSSI signal strength in dBm
int8_t getSignalStrength()
{
    String AT_response = "";
    const char AT_cmd[] = "AT+CSQ";
    char rssi[4];

    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return 99;

    if (extractText((char *)AT_response.c_str(), "+CSQ: ", rssi, sizeof(rssi), ','))
        return atoi(rssi);

    return 99;
}

/// @brief Query active network band information
/// @return Band name string
String getNetworkBand()
{
    const char AT_cmd[] = "AT+QNWINFO";
    char band[64];
    String AT_response = "";

    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return "";

    // Extract network band from response
    // Expected format: +QNWINFO: "FDD LTE","63902","LTE BAND 3",1650
    // Extract text between second and third comma
    if (extractText((char *)AT_response.c_str(), "+QNWINFO: \"", band, sizeof(band), '\n'))
    {
        // ToDo: Extract Access Technology: Particulary interested in "NO SERVICE" as part of response
        // Find the second occurrence of comma and extract from there
        const char *start = strchr((char *)AT_response.c_str(), ',');
        if (start != nullptr)
        {
            start = strchr(start + 1, ',');
            if (start != nullptr)
            {
                start++; // Move past the comma
                // Skip leading quote if present
                if (*start == '"')
                    start++;

                const char *end = strchr(start, '"');
                if (end != nullptr)
                {
                    size_t length = end - start;
                    if (length < sizeof(band))
                    {
                        strncpy(band, start, length);
                        band[length] = '\0';
                        return String(band);
                    }
                }
            }
        }
    }

    return "";
}

/// @brief Ping an IP address or hostname to check connectivity
/// @param host IP address or hostname to ping
/// @param contextID PDP context ID to use for ping (usually 1)
/// @return true if ping successful, false otherwise
bool pingIP(const char *host, uint8_t contextID)
{
    char cmd[256];

    // AT+QPING=<contextID>,<host>[,<timeout>[,<pingnum>]]

    snprintf(cmd, sizeof(cmd), "AT+QPING=%d,\"%s\",32,4",
             contextID, host);

    flushSerial();

    // Wait for OK
    if (!sendAndCheck(cmd, "OK", 2000))
    {
        return false;
    }

    char urc[64];
    if (!waitForURC("+QPING:", urc, sizeof(urc), 5000))
    {
        return false;
    }

    char expected[32];
    snprintf(expected, sizeof(expected), "+QPING:");
    Serial.println(urc);

    return strstr(urc, expected) != NULL;
}

/// @brief Initialize serial communication with GSM module
/// @return true if communication established
bool GSM_Serial_begin()
{
    pinMode(QUECTEL_PWR_KEY, OUTPUT);
    pinMode(GSM_RST_PIN, OUTPUT);
    digitalWrite(GSM_RST_PIN, LOW);
    delay(1000);
    digitalWrite(QUECTEL_PWR_KEY, LOW);
    delay(500);
    digitalWrite(QUECTEL_PWR_KEY, HIGH);
    delay(2500);
    digitalWrite(QUECTEL_PWR_KEY, HIGH);
    delay(4000);

    GSMSerial.begin(115200);

    bool comm_init = false;

    unsigned long timeout = 60000;

    Serial.println("Attempting to initate comms with GSM module");

    while (millis() < timeout)
    {
        while (GSMSerial.available())
            GSMSerial.read();
        if (sendAndCheck("AT", "OK"))
        {
            comm_init = true;
            Serial.println("GSM module found!");
            break;
        }
    }
    if (!comm_init)
    {
        Serial.println("Failed to establish communication with GSM module");
        return false;
    }

// debug
#ifdef GSM_DEBUG
    sendAndCheck("ATE1", "OK");
    sendAndCheck("AT+CMEE=2", "OK");
#else
    sendAndCheck("ATE0", "OK");
    sendAndCheck("AT+CMEE=1", "OK");
#endif
    sendAndCheck("ATI", "OK");

    // Set automatic timezone and update Locate time to RTC
    sendAndCheck("AT+CTZU=3", "OK");
    sendAndCheck("AT&W", "OK");

    return comm_init;
}

/// @brief Reset GSM module via GPIO reset pin
/// @param seq: Reset sequence pattern (HIGH_LOW_HIGH or LOW_HIGH_LOW)
/// @param timing_delay :  Delay in milliseconds for each pin state change for the reset to happen
void GSMreset(RST_SEQ seq, uint8_t timing_delay)
{

    pinMode(GSM_RST_PIN, OUTPUT);

    if (seq == LOW_HIGH_LOW)
    {
        digitalWrite(GSM_RST_PIN, LOW);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, HIGH);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, LOW);
    }
    else if (seq == HIGH_LOW_HIGH)
    {
        digitalWrite(GSM_RST_PIN, HIGH);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, LOW);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, HIGH);
    }

    delay(30000); // Allow enough time for GSM to warm up
}

/// @brief Put GSM module into low-power sleep mode
void GSM_sleep()
{

    if (sendAndCheck("AT+QSCLK=2", "OK"))
    {
        Serial.println("GSM module is now in sleep mode. Will only wake up if data is sent on the serial port");
    }
    else
    {
        Serial.println("Failed to put GSM module in sleep mode");
    }
}

String getBatteryStatus()
{
    String AT_response = "";
    const char AT_cmd[] = "AT+CBC";
    char buffer[64];

    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return "";

    // Extract the full "+CBC: <bcs>,<bcl>,<voltage>"" response line
    if (extractText((char *)AT_response.c_str(), "+CBC: ", buffer, sizeof(buffer), '\n'))
    {
        // Parse the three values separated by commas
        int bcs, bcl, voltage;
        if (sscanf(buffer, "%d,%d,%d", &bcs, &bcl, &voltage) == 3)
        {
            // Map battery charge status to human-readable text
            const char *status_text;
            switch (bcs)
            {
            case 0:
                status_text = "Not charging";
                break;
            case 1:
                status_text = "Charging";
                break;
            case 2:
                status_text = "Charging complete";
                break;
            default:
                status_text = "Unknown";
                break;
            }

            char formatted[64];
            snprintf(formatted, sizeof(formatted), "%s %d%% %.1fv", status_text, bcl, voltage / 1000.0); // e.g. "charging 75% 4.2v"
            return String(formatted);
        }
    }

    return "";
}

// ================================================================================
//          MQTT FUNCTIONS
// ================================================================================

/// @brief Configure MQTT settings
/// @param client_id MQTT client ID (0-5)
/// @param recv_mode Reception mode (0=URC with data, 1=URC without data)
/// @param msg_len Message length display (0=no, 1=yes)
/// @return true if configuration successful
bool MQTT_configure(uint8_t client_id, uint8_t recv_mode, uint8_t msg_len)
{
    char config_cmd[64] = "";

    snprintf(config_cmd, sizeof(config_cmd), "AT+QMTCFG=\"recv/mode\",%d,%d,%d",
             client_id, recv_mode, msg_len);

    Serial.print("MQTT Config: ");
    Serial.println(config_cmd);

    if (!sendAndCheck(config_cmd, "OK", 5000))
    {
        MQTT_INIT_ERROR = "Failed to configure MQTT receiving mode";
        Serial.println(MQTT_INIT_ERROR);
        return false;
    }

    Serial.println("MQTT configured successfully");
    return true;
}

/// @brief Open MQTT broker connection
/// @param client_id MQTT client ID (0-5)
/// @param broker Broker hostname or IP address
/// @param port Broker port (typically 1883 for non-SSL, 8883 for SSL)
/// @return true if broker connection opened successfully
bool MQTT_open(uint8_t client_id, const char *broker, uint16_t port)
{

    char open_cmd[128] = "";

    snprintf(open_cmd, sizeof(open_cmd), "AT+QMTOPEN=%d,\"%s\",%d",
             client_id, broker, port);

    Serial.print("MQTT Open: ");
    Serial.println(open_cmd);

    if (!sendAndCheck(open_cmd, "OK", 10000))
    {
        MQTT_INIT_ERROR = "Failed to open MQTT broker connection";
        Serial.println(MQTT_INIT_ERROR);
        MQTT_INIT_FAIL_COUNT++;
        return false;
    }

    // Wait for broker open URC: +QMTOPEN: <client_id>,0
    // Docs state max response time is 120 s (network-dependent); 30 s covers most cases.
    char broker_open[16];
    snprintf(broker_open, sizeof(broker_open), "+QMTOPEN: %d,0", client_id);

    char urc[32];
    if (!waitForURC(broker_open, urc, sizeof(urc), 30000))
    {
        MQTT_INIT_ERROR = "MQTT broker open URC not received";
        Serial.println(MQTT_INIT_ERROR);
        MQTT_INIT_FAIL_COUNT++;
        return false;
    }

    return true;
}

/// @brief Connect MQTT client to broker
/// @param client_id MQTT client ID (0-5)
/// @param clientid MQTT client identifier string
/// @param username Username for MQTT authentication (optional)
/// @param password Password for MQTT authentication (optional)
/// @return true if client connected successfully
bool MQTT_connect(uint8_t client_id, const char *clientid, const char *username, const char *password)
{
    char conn_cmd[256] = "";

    // Build AT command based on whether credentials are provided
    if (username && password && *username && *password)
    {
        snprintf(conn_cmd, sizeof(conn_cmd), "AT+QMTCONN=%d,\"%s\",\"%s\",\"%s\"",
                 client_id, clientid, username, password);
    }
    else
    {
        snprintf(conn_cmd, sizeof(conn_cmd), "AT+QMTCONN=%d,\"%s\"",
                 client_id, clientid);
    }

    Serial.print("MQTT Connect: ");
    Serial.println(conn_cmd);

    if (!sendAndCheck(conn_cmd, "OK", 10000))
    {
        MQTT_INIT_ERROR = "Failed to send MQTT connect command";
        Serial.println(MQTT_INIT_ERROR);
        MQTT_INIT_FAIL_COUNT++;
        return false;
    }

    // Wait for connect URC: +QMTCONN: <client_id>,0,0
    char urc[32];
    if (!waitForURC("+QMTCONN:", urc, sizeof(urc), 10000))
    {
        MQTT_INIT_ERROR = "MQTT connect URC not received";
        Serial.println(MQTT_INIT_ERROR);
        MQTT_INIT_FAIL_COUNT++;
        return false;
    }

    // Parse URC response - successful connection returns +QMTCONN: <client_id>,0,0
    if (strstr(urc, "+QMTCONN:") && strstr(urc, ",0,0"))
    {
        mqtt_status = MQTT_CONNECTED;
        MQTT_INIT_FAIL_COUNT = 0;
        Serial.println("MQTT client connected to broker successfully");
        return true;
    }

    MQTT_INIT_ERROR = "Invalid MQTT connect response";
    Serial.println(MQTT_INIT_ERROR);
    MQTT_INIT_FAIL_COUNT++;
    return false;
}

/// @brief Subscribe to MQTT topic
/// @param client_id MQTT client ID (0-5)
/// @param msg_id Message ID for subscription (used to track subscription)
/// @param topic Topic name to subscribe to
/// @param qos Quality of Service level (0-2)
/// @return true if subscription successful
bool MQTT_subscribe(uint8_t client_id, uint16_t msg_id, const char *topic, uint8_t qos)
{
    // msg_id must be 1–65535 for AT+QMTSUB — 0 is rejected by the modem
    if (msg_id == 0)
    {
        Serial.println("MQTT subscribe: msg_id must be >= 1");
        return false;
    }

    char sub_cmd[128] = "";

    if (qos > 2)
        qos = 2;

    snprintf(sub_cmd, sizeof(sub_cmd), "AT+QMTSUB=%d,%d,\"%s\",%d",
             client_id, msg_id, topic, qos);

    Serial.print("MQTT Subscribe: ");
    Serial.println(sub_cmd);

    flushSerial();

    if (!sendAndCheck(sub_cmd, "OK", 10000))
    {
        Serial.println("Failed to send MQTT subscribe command");
        return false;
    }

    // Wait for subscription URC: +QMTSUB: <client_id>,<msg_id>,0,<qos>
    char urc[32];
    if (!waitForURC("+QMTSUB:", urc, sizeof(urc), 5000))
    {
        Serial.println("MQTT subscribe URC not received");
        return false;
    }

    // Validate result field — format: +QMTSUB: <client_id>,<msg_id>,<result>
    // Find the third comma-separated field (result)
    const char *p = strchr(urc, ','); // after client_id
    if (p)
        p = strchr(p + 1, ','); // after msg_id
    if (!p)
    {
        Serial.println("MQTT subscribe: malformed URC");
        return false;
    }

    int result = atoi(p + 1);
    if (result != 0)
    {
        Serial.print("MQTT subscribe failed with result: ");
        Serial.println(result);
        return false;
    }

    Serial.print("Subscribed to topic: ");
    Serial.println(topic);
    return true;
}

/// @brief Publish MQTT message to topic
/// @param client_id MQTT client ID (0-5)
/// @param msg_id Message ID for tracking
/// @param topic Topic name to publish to
/// @param payload Message payload to publish
/// @param qos Quality of Service level (0-2)
/// @param retain Retain flag (0=no, 1=yes)
/// @return true if publish successful
bool MQTT_publish(uint8_t client_id, uint16_t msg_id, const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    char pub_cmd[256] = "";
    size_t payload_len = strlen(payload);
    String resp = "";
    if (qos > 2)
        qos = 2;
    if (retain > 1)
        retain = 1;

    // Validate payload length (max 1500 bytes for EC200U)
    if (payload_len > 1500)
    {
        Serial.println("MQTT payload exceeds maximum length of 1500 bytes");
        return false;
    }

    // Build publish command: AT+QMTPUBEX=<client_id>,<msg_id>,<qos>,<retain>,"topic",<length>
    snprintf(pub_cmd, sizeof(pub_cmd), "AT+QMTPUBEX=%d,%d,%d,%d,\"%s\",%d",
             client_id, msg_id, qos, retain, topic, (int)payload_len);

    Serial.print("MQTT Publish: ");
    Serial.println(pub_cmd);

    flushSerial();
    sendAndCheck("AT+QISTATE=0,1", "OK", resp, 5000);
    GSMSerial.println(pub_cmd);

    // Wait for ">" prompt to send payload.
    // Per Quectel docs the modem returns "OK" then ">" — waitForReply scans the full
    // stream so it catches ">" whenever it arrives. 15 s covers a slow GSM TCP handshake.
    if (!waitForReply(">", 15000))
    {
        Serial.println("MQTT publish: no '>' data-input prompt received from modem");
        MQTT_PUB_FAIL++;
        return false;
    }

    // Send payload
    GSMSerial.write((const uint8_t *)payload, payload_len);

    // Wait for OK
    if (!waitForReply("OK", 10000))
    {
        Serial.println("MQTT publish failed - no OK response");
        MQTT_PUB_FAIL++;
        sendAndCheck("AT+QISTATE=0,1", "OK", resp, 5000);
        return false;
    }
    sendAndCheck("AT+QISTATE=0,1", "OK", resp, 5000);
    // Wait for publish URC: +QMTPUBEX: <client_id>,<msg_id>,0
    char urc[32];
    if (!waitForURC("+QMTPUBEX:", urc, sizeof(urc), 5000))
    {
        Serial.println("MQTT publish URC not received");
        MQTT_PUB_FAIL++;
        return false;
    }

    if (strstr(urc, ",0")) // Check for success code 0 //! Success code is actually <client_idx>,<msg_id>,0 . Message id may be 0 for QoS 0
    {
        Serial.print("Published to topic: ");
        Serial.println(topic);
        MQTT_PUB_FAIL = 0;
        sendAndCheck("AT+QISTATE=0,1", "OK", resp, 5000);
        return true;
    }
    sendAndCheck("AT+QISTATE=0,1", "OK", resp, 5000);
    Serial.println("MQTT publish failed");
    MQTT_PUB_FAIL++;
    return false;
}

/// @brief Unsubscribe from MQTT topic
/// @param client_id MQTT client ID (0-5)
/// @param msg_id Message ID for unsubscription
/// @param topic Topic name to unsubscribe from
/// @return true if unsubscription successful
bool MQTT_unsubscribe(uint8_t client_id, uint16_t msg_id, const char *topic)
{
    char unsub_cmd[128] = "";

    snprintf(unsub_cmd, sizeof(unsub_cmd), "AT+QMTUNS=%d,%d,\"%s\"",
             client_id, msg_id, topic);

    Serial.print("MQTT Unsubscribe: ");
    Serial.println(unsub_cmd);

    flushSerial();

    if (!sendAndCheck(unsub_cmd, "OK", 10000))
    {
        Serial.println("Failed to send MQTT unsubscribe command");
        return false;
    }

    // Wait for unsubscribe URC: +QMTUNS: <client_id>,<msg_id>,0
    char urc[32];
    if (!waitForURC("+QMTUNS:", urc, sizeof(urc), 5000))
    {
        Serial.println("MQTT unsubscribe URC not received");
        return false;
    }

    Serial.print("Unsubscribed from topic: ");
    Serial.println(topic);
    return true;
}

/// @brief Disconnect MQTT client from broker
/// @param client_id MQTT client ID (0-5)
/// @return true if disconnection successful
bool MQTT_disconnect(uint8_t client_id)
{
    char disc_cmd[32] = "";

    snprintf(disc_cmd, sizeof(disc_cmd), "AT+QMTDISC=%d", client_id);

    Serial.print("MQTT Disconnect: ");
    Serial.println(disc_cmd);

    if (!sendAndCheck(disc_cmd, "OK", 10000))
    {
        Serial.println("Failed to send MQTT disconnect command");
        return false;
    }

    char success[16];
    snprintf(success, sizeof(success), "+QMTDISC: %d,0", client_id);
    // Wait for disconnect URC: +QMTDISC: <client_id>,0
    char urc[32];
    if (!waitForURC(success, urc, sizeof(urc), 5000))
    {
        Serial.println("MQTT disconnect URC not received");
        return false;
    }
    mqtt_status = MQTT_DISCONNECTED;
    Serial.println("MQTT client disconnected successfully");
    return true;
}

/// @brief Get current MQTT connection status
/// @param client_id MQTT client ID (0-5)
/// @return Current MQTT connection status
GSMMQTTConnStatus MQTT_getStatus(uint8_t client_id)
{
    char query_cmd[32] = "";

    snprintf(query_cmd, sizeof(query_cmd), "AT+QMTOPEN?");

    String response = "";
    if (!sendAndCheck(query_cmd, "OK", response, 5000))
    {
        return MQTT_DISCONNECTED;
    }

    // Parse response to check for +QMTOPEN: <client_id> pattern
    // Response format: +QMTOPEN: <client_idx>,<host_name>,<port>
    char search_pattern[16] = "";
    snprintf(search_pattern, sizeof(search_pattern), "+QMTOPEN: %d,", client_id);

    if (strstr(response.c_str(), search_pattern))
    {
        // Extract and print hostname and port
        const char *start = strstr(response.c_str(), search_pattern);
        if (start != nullptr)
        {
            // Skip past the pattern to get to hostname
            start += strlen(search_pattern);
            const char *end = strchr(start, '\r');
            if (end == nullptr)
                end = strchr(start, '\n');

            if (end != nullptr)
            {
                size_t length = end - start;
                char host_port_info[128];
                strncpy(host_port_info, start, length);
                host_port_info[length] = '\0';
                // Serial.print("MQTT connection found - Host/Port: ");
                // Serial.println(host_port_info);
            }
        }
        return MQTT_CONNECTED;
    }

    return MQTT_DISCONNECTED;
}

/// @brief Check if MQTT is connected and ready
/// @param client_id MQTT client id index (0-5)
/// @return true if MQTT client is connected to broker
bool MQTT_isClientConnected(uint8_t client_id)
{
    GSMMQTTConnStatus status = MQTT_getStatus(client_id);
    if (status == GSMMQTTConnStatus::MQTT_CONNECTED)
        return true;
    else
        return false;
}

/// @brief Check id broker is connected for an MQTT client
/// @param client_id MQTT client id (0-5)
/// @return true if connected (3), false if otherwise
bool MQTT_isBrokerConnected(uint8_t client_id)
{

    String response = ""; // [+QMTCONN: <client_idx>,<state>]
    if (!sendAndCheck("AT+QMTCONN?", "OK", response, 2000))
        return false;

    char prefix[16];
    snprintf(prefix, sizeof(prefix), "+QMTCONN: %d,", client_id);

    char status_str[4] = {0};
    if (!extractText((char *)response.c_str(), prefix, status_str, sizeof(status_str), '\r'))
        return false;

    int state = atoi(status_str);
    if (state == 3)
    {
        // Serial.println("MQTT Brocker connected");
        return true;
    }
    return false;
}

/// @brief Check if there are any messages waiting in the buffer for a specific MQTT client. A maximum of 5 messages can be stored in the buffer.
/// @param client_id   MQTT client id index (0-5)
/// @return true if at least one message is waiting
bool MQTT_hasBufferedMessage(uint8_t client_id)
{

    char expected_qurc[16]; //+QMTRECV: <client_idx>,<recv_id>
    snprintf(expected_qurc, sizeof(expected_qurc), "+QMTRECV: %d", client_id);
    char res[16];
    if (waitForURC(expected_qurc, res, sizeof(res), 5000))
    {
        return true;
    }

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QMTRECV?");

    String response = "";
    if (!sendAndCheck(cmd, "OK", response, 1000))
        return false;

    // Serial.println(response);
    // Build exact prefix for this client, e.g. "+QMTRECV: 0,"
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "+QMTRECV: %d,", client_id);

    char status_str[32] = {0};

    if (!extractText((char *)response.c_str(), prefix, status_str, sizeof(status_str), '\r'))
        return false;
    // status_str now contains something like "0,1,0,0,0" (the five store_status values)
    char *p = status_str;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (*p == '1') // 1 = message waiting in this slot
        {
            return true;
        }
        p = strchr(p, ','); // move to next status
        if (!p)
            break;
        p++;
    }

    return false; // all slots empty for this client
}

/// @brief Read buffered MQTT message from broker
/// @param client_id MQTT client ID (0-5)
/// @param recv_id Message ID of the buffered message to read
/// @param topic_out Output buffer for topic name
/// @param topic_size Size of topic output buffer
/// @param payload_out Output buffer for message payload
/// @param payload_size Size of payload output buffer
/// @return true if message read successfully, false otherwise
//! When configuring recv/mode using AT+QMTCFG="recv/mode",<client_idx>[,<msg_recv_mode>[,<msg_len_enable>]]
//! set recv_mode=1 (buffered), msg_len=1 (include length) otherwise this function will fail
//! be careful when using MQTT_configure() for this to work properly
bool MQTT_readBufferedMessage(uint8_t client_id, uint8_t recv_id,
                              char *topic_out, size_t topic_size,
                              char *payload_out, size_t payload_size) // ToDo: Remove if recv_id is not neccesary to get payload
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QMTRECV=%d,%d", client_id, recv_id);

    String full_response = "";
    if (!sendAndCheck(cmd, "OK", full_response, 5000))
    {
        Serial.println("Failed to read buffered MQTT message");
        return false;
    }

    // Find the start of the +QMTRECV line
    int start_idx = full_response.indexOf("+QMTRECV:");
    if (start_idx < 0)
    {
        Serial.println("No +QMTRECV line found in response");
        return false;
    }

    String resp = full_response.substring(start_idx);

    // Format we expect:
    // +QMTRECV: 0,1234,"sensors/africa/cmd",42,{"action":"restart"}

    // 1. Skip "+QMTRECV: " + client_idx + ","
    int pos = resp.indexOf(':');
    if (pos < 0)
        return false;
    pos += 1; // after :
    while (resp[pos] == ' ')
        pos++; // skip spaces

    // Skip client_idx and comma
    while (isdigit(resp[pos]) || resp[pos] == ',')
        pos++;

    // Skip msgid and comma
    while (isdigit(resp[pos]) || resp[pos] == ',')
        pos++;

    // 2. Extract topic (quoted string)
    if (resp[pos] != '"')
        return false;
    pos++; // skip opening quote

    int topic_end = resp.indexOf('"', pos);
    if (topic_end < 0)
        return false;

    String topic = resp.substring(pos, topic_end);
    if (topic.length() >= topic_size)
    {
        Serial.println("Topic too long for buffer");
        return false;
    }
    strcpy(topic_out, topic.c_str());

    pos = topic_end + 1; // after closing quote

    // 3. Expect comma + payload_len
    if (resp[pos] != ',')
        return false;
    pos++;

    // Read payload length
    int payload_len = 0;
    while (isdigit(resp[pos]))
    {
        payload_len = payload_len * 10 + (resp[pos] - '0');
        pos++;
    }

    // 4. Expect comma after length
    if (resp[pos] != ',')
        return false;
    pos++;

    // 5. Now the payload starts — take exactly payload_len bytes
    if (payload_len >= (int)payload_size)
    {
        Serial.println("Payload too large for buffer");
        return false;
    }

    // Copy exactly payload_len characters
    strncpy(payload_out, resp.c_str() + pos, payload_len);
    payload_out[payload_len] = '\0'; // null-terminate

    // Optional safety: if there are extra characters before OK, we ignore them
    // (but in practice there should be none or just \r\n)

    Serial.print("MQTT message received (buffered) → Topic: ");
    Serial.print(topic_out);
    Serial.print(" | Length: ");
    Serial.print(payload_len);
    Serial.print(" | Payload: ");
    Serial.println(payload_out);

    return true;
}

/// @brief Read buffered MQTT message from broker
/// @param client_id MQTT client ID (0-5)
/// @param topic_out Output buffer for topic name
/// @param topic_size Size of topic output buffer
/// @param payload_out Output buffer for message payload
/// @param payload_size Size of payload output buffer
/// @return true if message read successfully, false otherwise
//! When configuring recv/mode using AT+QMTCFG="recv/mode",<client_idx>[,<msg_recv_mode>[,<msg_len_enable>]]
//! set recv_mode=1 (buffered), msg_len=1 (include length) otherwise this function will fail
//! be careful when using MQTT_configure() for this to work properly
bool MQTT_readBufferedMessage(uint8_t client_id, char *topic_out, size_t topic_size,
                              char *payload_out, size_t payload_size) // ToDo: Remove if recv_id is neccesary to get payload
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QMTRECV=%d", client_id);

    String full_response = "";
    if (!sendAndCheck(cmd, "OK", full_response, 5000))
    {
        // Serial.println("Failed to read buffered MQTT message");
        return false;
    }

    // Find the start of the +QMTRECV line
    int start_idx = full_response.indexOf("+QMTRECV:");
    if (start_idx < 0)
    {
        return false;
    }

    String resp = full_response.substring(start_idx);
    Serial.print(resp);
    // Format we expect:
    // +QMTRECV: 0,1234,"devices/nodes/cmd",20,{"action":"restart"}

    // 1. Skip "+QMTRECV: " + client_idx + ","
    int pos = resp.indexOf(':');
    if (pos < 0)
        return false;
    pos += 1; // after :
    while (resp[pos] == ' ')
        pos++; // skip spaces

    // Skip client_idx and comma
    while (isdigit(resp[pos]) || resp[pos] == ',')
        pos++;

    // Skip msgid and comma
    while (isdigit(resp[pos]) || resp[pos] == ',')
        pos++;

    // 2. Extract topic (quoted string)
    if (resp[pos] != '"')
        return false;
    pos++; // skip opening quote

    int topic_end = resp.indexOf('"', pos);
    if (topic_end < 0)
        return false;

    String topic = resp.substring(pos, topic_end);
    if (topic.length() >= topic_size)
    {
        Serial.println("Topic too long for buffer");
        return false;
    }
    strcpy(topic_out, topic.c_str());

    pos = topic_end + 1; // after closing quote

    // 3. Expect comma + payload_len
    if (resp[pos] != ',')
        return false;
    pos++;

    // Read payload length
    int payload_len = 0;
    while (isdigit(resp[pos]))
    {
        payload_len = payload_len * 10 + (resp[pos] - '0');
        pos++;
    }

    // 4. Expect comma after length
    if (resp[pos] != ',')
        return false;
    pos++;

    if (resp[pos] == '"')
        pos++;

    // 5. Now the payload starts — take exactly payload_len bytes
    if (payload_len >= (int)payload_size)
    {
        Serial.println("Payload too large for buffer");
        return false;
    }

    // Copy exactly payload_len characters
    strncpy(payload_out, resp.c_str() + pos, payload_len);
    payload_out[payload_len] = '\0'; // null-terminate

    // Optional safety: if there are extra characters before OK, we ignore them
    // (but in practice there should be none or just \r\n)

    Serial.print("\nMQTT message received (buffered) → Topic: ");
    Serial.print(topic_out);
    Serial.print(" | Length: ");
    Serial.print(payload_len);
    Serial.print(" | Payload: ");
    Serial.println(payload_out);

    return true;
}

// Testing POST data
// http://staging.api.sensors.africa/v1/push-sensor-data/

// POST /v1/push-sensor-data/\r\nHost: http://staging.api.sensors.africa\r\nAccept: */*\r\nUser-Agent: QUECTEL EC200\r\nContent-Type: application/json\r\nX-Sensor: esp8266-15355455\r\nX-PIN: 1\r\nContent-Length: 385\r\n\r\n{"software_version": "NRZ-2020-129", "sensordatavalues":[{"value_type":"P0","value":"7.80"},{"value_type":"P1","value":"10.50"},{"value_type":"P2","value":"13.40"}]}\r\n
// data length 252

// Accept: */*\r\nUser-Agent: QUECTEL EC200\r\nContent-Type: application/json\r\nX-Sensor: esp8266-15355455\r\nX-PIN: 1\r\nContent-Length: 165\r\n\r\n{"software_version": "NRZ-2020-129", "sensordatavalues":[{"value_type":"P0","value":"7.80"},{"value_type":"P1","value":"10.50"},{"value_type":"P2","value":"13.40"}]}\r\n
/// 1234

// AT commands sequence

// AT+CGATT=1
// AT+QICSGP=1,1,"safaricom","saf","data"
// AT+QIACT=1
// AT+QIACT?
// AT+QHTTPCFG="contextid",1
// AT+QHTTPCFG="requestheader",1
// AT+QHTTPCFG="responseheader",1
// AT+QHTTPURL=54,30,60
// http://staging.api.sensors.africa/v1/push-sensor-data/
// AT+QHTTPPOST=385,30,60
// AT+QHTTPREAD
