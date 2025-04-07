#include <SoftwareSerial.h>
#include "ext_def.h"

#define MCU_RXD D5
#define MCU_TXD D6
#define QUECTEL_PWR_KEY D0
#define QUECTEL_DTR D9
SoftwareSerial GSMSerial(MCU_RXD, MCU_TXD);
enum RST_SEQ
{
    HIGH_LOW_HIGH,
    LOW_HIGH_LOW
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

enum NetMode // Quectel
{
    AUTO = 0,
    _2G = 1,
    _4G = 3,
};
NetMode current_network = NetMode::AUTO;

/**** Function Declacrations **/
bool GSM_init();
bool register_to_network();
// static void unlock_pin(char *PIN);
void SIM_PIN_Setup();
bool is_SIMCID_valid();
bool GPRS_init();
void GSM_soft_reset();
void restart_GSM();
void flushSerial();
void SerialFlush();
void QUECTEL_POST(char *url, String headers[], int header_size, const String &data, int data_length);
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until); // ? should go to utils
void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool fill_buffer = false, unsigned long timeout = 1000);
int16_t getNumber(char *AT_cmd, char *expected_reply, uint8_t index_from, uint8_t length);
void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS);
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, unsigned long timeout = 1000, bool wait_timeout = false);
bool configurePDP();
void getIPAddress(char *IP);
void setNetworkMode(NetMode mode);
void troubleshoot_GSM();
int8_t GPRS_status();
bool activateGPRS();
bool deactivateGPRS();
bool GSM_Serial_begin();
void GSMreset(RST_SEQ seq, uint8_t timing_delay = 120);

// Set a decent delay before this to warm up the GSM module
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

bool register_to_network()
{

    String error_msg = "";
    bool registered_to_network = false;
    int retry_count = 0;
    setNetworkMode(current_network);
    while (!registered_to_network && retry_count < 20)
    {
        int8_t status = getNumber("AT+CREG?", "+CREG: ", 2, 1);

        if (status == 1 || status == 5)
        {
            registered_to_network = true;
            break;
        }

        else
        {
            Serial.println("Not registered to network ");
        }

        retry_count++;
        delay(3000);
    }

    if (!registered_to_network)
    {
        error_msg = "Network not registered";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        REGISTER_TO_NETWORK_FAIL += 1;

        // Attempt to enable network registration

        if (!sendAndCheck("AT+CREG=1", "OK"))
        {
            Serial.println("Manual network registration failed.");
        }

        if (REGISTER_TO_NETWORK_FAIL > 5)
        {
            GSM_soft_reset();
            //? Check if the SIM card is still there?
            REGISTER_TO_NETWORK_FAIL = 0;
        }
        return false;
    }

    sendAndCheck("AT+COPS?", "OK");
    return true;
}

// static void unlock_pin(char *PIN)
// {

//     // Attempt to SET PIN if not empty
//     Serial.print("GSM CONFIG SET PIN: ");
//     Serial.println(PIN);
//     Serial.print("Length of PIN");
//     Serial.println(strlen(PIN));
//     if (strlen(PIN) <4)
//     {

//     // ToDo: WIP
//         SIM_PIN_SET = true;
//     }
// }

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

bool is_SIMCID_valid() // ! Seems to be returning true even when there is "ERROR" in response
{
    char qccid[21];

    char AT_response[255] = {};

    char expected_reply[] = "+QCCID: ";

    get_raw_response("AT+QCCID\0", AT_response, 255, true, 5000);

    if (extractText(AT_response, expected_reply, qccid, 21, '\r') && strlen(qccid) == 20)
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

// Similar to FONA enableGPRS() but quicker because APN setting are not configured as it is configured during GSM_init()
bool GPRS_init()
{

    String err = "";
    Serial.println("Quectel GPRS init...");

    if (!activateGPRS())
    {
        err = "Failed to attach to GPRS network";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        GPRS_INIT_FAIL_COUNT += 1;
        return false;
    }

    if (!configurePDP())
    {
        err = "Failed to config GPRS PDP context";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        GPRS_INIT_FAIL_COUNT += 1;
        return false;
    }

    //? QIACT
    GPRS_CONNECTED = true;
    GPRS_INIT_FAIL_COUNT = 0;

    return GPRS_CONNECTED;
}

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

/***
 * ? Called 3 times. Review the impelementation of this
 * Todo: Change implementation to shut down GSM and then call GSM_init();
 *
 *
 ***/
void restart_GSM()
{
    Serial.println("Restarting GSM");
    //! The AQ PCB board has the GSM reset physically connected to the ESP chip

    if (!GSM_init())
    {
        Serial.println("GSM not fully configured");
        Serial.print("Failure point: ");
        Serial.println(GSM_INIT_ERROR);
        Serial.println();
    }
}

/*****************************************************************
flushSerial
*****************************************************************/
void flushSerial()
{
    while (GSMSerial.available())
        GSMSerial.read();
}

/// @brief Easy implementation of Quectel HTTP functionality
/// @param url url for http request sans protocol
/// @param headers array of request headers
/// @param header_size size of the headers array
/// @param data post body data
/// @param data_length length of the data
void QUECTEL_POST(char *url, String headers[], int header_size, const String &data, int data_length)
{
    /* SETTING request headers
    ! Headers are sent in two formats
    1. Format 0: headers are sent before post body
    2. Format 1: headers are sent as part of the body
    */

    // Using format 0

    // Config URL
    // String HTTP_SETUP = "AT+QHTTPURL=" + String(strlen(url), DEC) + ",10,60";

    String HTTP_CFG = "AT+QHTTPCFG=\"url\",\"http://" + String(url) + "\""; // protocol must be set before URL
    Serial.print("Quectel URL config: ");
    Serial.println(HTTP_CFG);
    sendAndCheck(HTTP_CFG.c_str(), "OK");

    sendAndCheck("AT+QHTTPCFG=\"contextid\",1", "OK");      // set context id
    sendAndCheck("AT+QHTTPCFG=\"requestheader\",0", "OK");  // disable request headers
    sendAndCheck("AT+QHTTPCFG=\"responseheader\",1", "OK"); // enable response headers
    sendAndCheck("AT+QHTTPCFG=\"rspout/auto\",1", "OK");    // enable auto response and "disable" HTTTPREAD

    for (int i = 0; i < header_size; i++)
    {
        HTTP_CFG = "AT+QHTTPCFG=\"header\",\"" + headers[i] + "\"";
        if (sendAndCheck(HTTP_CFG.c_str(), "OK"))
        {
            Serial.println("Header set successfully");
        }
        else
        {
            Serial.println("Failed to set header");
            return;
        }
    }

    char HTTP_POST_RESPONSE_STATUS[4];

    // POST data
    // HTTP_CFG = "AT+QHTTPPOST=" + String(data_length) + ",30,60";
    char http_post_prepare[32] = "AT+QHTTPPOST=";
    char data_len[4];
    itoa(data_length, data_len, 10);
    strcat(http_post_prepare, data_len);
    strcat(http_post_prepare, ",30,60");

    Serial.println(http_post_prepare);
    if (sendAndCheck(http_post_prepare, "CONNECT", 10000)) // Allow enough time to connect to HTTP(S) server. Should be less than the timeout the GSM expects the data to be sent.
    {
        Serial.println("Posting gprs data..");
        get_http_response_status(data, HTTP_POST_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("HTTP POST CONNECT FAIL");
        HTTPCFG_CONNECT_FAIL += 1;
        return;
    }

    if (strstr(HTTP_POST_RESPONSE_STATUS, "20"))
    {
        Serial.println("Requested processed successfully with status: " + (String)HTTP_POST_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("Requested processing failed with status: " + (String)HTTP_POST_RESPONSE_STATUS);
        HTTP_POST_FAIL += 1;
    }
}

void SerialFlush()
{
    // Serial.flush();
    while (Serial.available())
    {
        Serial.read();
    }
}

void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool fill_buffer, unsigned long timeout)
{

    flushSerial();
    memset(res_buff, '\0', buff_size);
    // Serial.println("Size of response buffer: " + (String)buff_size);
    size_t buff_pos = 0;
    // Serial.print("Received Command in get raw: ");
    // Serial.println(cmd);
    GSMSerial.println(cmd);
    unsigned long sendStartMillis = millis();
    do
    {
        if (buff_pos >= buff_size) // Check if buff is full
            break;

        while (GSMSerial.available())
        {

            res_buff[buff_pos] = GSMSerial.read();
            buff_pos++;

            if (buff_pos == buff_size)
                break;
        }

        // eat unsolicited result code "RDY"
        if (strstr(res_buff, "RDY"))
        {
            // reset buff
            memset(res_buff, '\0', buff_size);
            buff_pos = 0;
            Serial.println("Eating URC 'RDY'");
        }

        delay(2);
    } while ((fill_buffer ? fill_buffer : strlen(res_buff) == 0) && (millis() - sendStartMillis < timeout));
    Serial.println("\n-------\r\nGSM RAW RESPONSE:");
    Serial.println(res_buff);
    Serial.println("-------");
}

/***
 @brief : Extract a piece of text matching the target from a char array
 @param input : The char array that contains the string to be parsed from
 @param target : Ocuurence of a particular string
 @param output : A char array to store extracted string
 @param _until : The first character matching to read from after finding occurence of the target
 @return
 ****/
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until)
{

    const char *found_target = strstr(input, target);

    if (found_target != nullptr)
    {

        Serial.print("Substring found at position: ");
        Serial.println(found_target - input);

        // Find the start of the extraction point
        const char *start = found_target + strlen(target);

        // Find the end of the extraction point (the next comma by default)
        const char *end = strchr(start, _until);

        if (end != nullptr)
        {
            // Calculate the length of the text to be extracted
            size_t length = end - start;

            if (length < output_size)
            { // check for buffer overflow.
                strncpy(output, start, length);
                output[length] = '\0'; // Null-terminate the string
                return true;
            }
            else
            {
                Serial.println("Extracted piece of text longer than ouput size");
                return false;
            }
        }
    }
    Serial.println("Could not extact substring '" + (String)target + "' from the source");
    return false;
}

// extract an integer
int16_t getNumber(char *AT_cmd, char *expected_reply, uint8_t index_from, uint8_t length)
{

    int16_t num;

    char AT_response[255];
    size_t AT_res_size = sizeof(AT_response);

    char number[8];

    if (length > sizeof(number))
    {
        Serial.println("max length allowed is 8");
        return -1;
    }

    get_raw_response(AT_cmd, AT_response, AT_res_size);

    const char *found_target = strstr(AT_response, expected_reply);

    if (found_target == nullptr)
        return -1;

    // Find the start of desired extraction point
    const char *start = found_target + strlen(expected_reply);
    start += index_from; // E.g to extract 5 from +CREG: 0,5,7 will start from '+CREG: ' + 2 indices

    if (length < sizeof(number))
    {

        strncpy(number, start, length);
        number[length] = '\0';
    }

    Serial.print("Extracted number: ");
    Serial.println(number);

    num = atoi(number);
    return num;
}

/// @brief simple function to send AT command and check for expected reply
/// @param AT_cmd : AT command to send
/// @param expected_reply : expect reply from the AT command to contain this string
/// @return true if expected reply is found
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, unsigned long timeout, bool wait_timeout)
{
    char AT_response[255];
    size_t AT_res_size = sizeof(AT_response);

    get_raw_response(AT_cmd, AT_response, AT_res_size, wait_timeout, timeout);

    if (strstr(AT_response, expected_reply))
    {
        return true;
    }

    return false;
}

void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS)
{
    char HTTP_RESPONSE[255];
    size_t BUFFER_SIZE = sizeof(HTTP_RESPONSE);
    const char *data_copy = data.c_str();
    char gprs_data[strlen(data_copy)];
    strcpy(gprs_data, data_copy);
    get_raw_response(gprs_data, HTTP_RESPONSE, BUFFER_SIZE, true, 10000);

    // Check HTTP RESPONSE status
    const char *expected_reply = "+QHTTPPOST: 0,"; // Operartion successful

    if (extractText(HTTP_RESPONSE, expected_reply, HTTP_RESPONSE_STATUS, 4, ','))
    {

        Serial.print("Gotten http status code: ");
        Serial.println(HTTP_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("Could not extract HTTP response status code");
        //? Maybe troubleshoot
    }
}

// Simple function to troubleshoot GSM //? More to be done
void troubleshoot_GSM()
{

    GSM_init(); // ! Use GSM soft reset if GSM reset pin is not connected

    register_to_network();

    GPRS_init();

    // RESET FLAGS
    HTTPCFG_CONNECT_FAIL = 0;
    HTTP_POST_FAIL = 0;
    GPRS_INIT_FAIL_COUNT = 0;
}

void setNetworkMode(NetMode mode)

{
    if (mode != NetMode::AUTO || mode != NetMode::_2G || mode != NetMode::_4G)
    {
        Serial.println("Invalid network mode");
        return;
    }

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

    if (!sendAndCheck(setnetmode, "OK"))
    {
        Serial.print("Failed to set network mode: ");
        Serial.println(mode_str);
        return;
    }
    delay(1000);
    current_network = mode;
}

/// @brief Configure PDP context
bool configurePDP()
{

    char PDP_config[32] = "AT+CGDCONT=1,\"IP\",\"hologram\""; //! APN name should be a global variable after testing

    if (!sendAndCheck(PDP_config, "OK", 30000, true))
    {
        Serial.println("Failed to set PDP context");
        return false;
    }

    char ipaddr[16] = {};
    getIPAddress(ipaddr);

    if (strlen(ipaddr) < 7 || strcmp(ipaddr, "0.0.0.0") == 0)
    {
        // recursive call to get IP address on different network modes
        Serial.println("IP address not set.");
        return false;
    }

    return true;
}

void getIPAddress(char *IP)
{
    char ipaddr[16] = {}; // 15 characters for IPV4 address

    char AT_response[64];
    get_raw_response("AT+CGPADDR=1", AT_response, 64, false); // ! context id assumed to be 1

    if (extractText(AT_response, "+CGPADDR: 1,\"", ipaddr, 16, '"'))
    {
        Serial.print("IP Address: ");
        Serial.println(ipaddr);
        strcpy(IP, ipaddr);
    }
    else
    {
        Serial.println("Failed to get IP address");
    }
}

int8_t GPRS_status()
{

    int8_t status = getNumber("AT+CGATT?\0", "+CGATT: ", 0, 1);
    Serial.print("CGATT status: ");
    Serial.println(status);
    return status;
}

/// @brief attach GPRS Network
bool activateGPRS()
{
    if (GPRS_status() == 1)
    {
        Serial.println("GPRS already active");
        return true;
    }

    bool activated = sendAndCheck("AT+CGATT=1", "OK");
    if (activated)
    {
        Serial.println("GPRS attached");
        return true;
    }

    if (!activated)
    {
        Serial.print("GPRS failed activate on network mode:  ");
        Serial.print(current_network);
        Serial.println("\t(AUTO:0, 2G:1, 4G:3)");

        switch (current_network)
        {
        case NetMode::AUTO:
            current_network = NetMode::_2G;
            register_to_network();
            activateGPRS();
            break;
        case NetMode::_2G:
            current_network = NetMode::_4G;
            register_to_network();
            activateGPRS();
            break;
        case NetMode::_4G:
            current_network = NetMode::AUTO;
            register_to_network();
            activateGPRS();
            break;
        }
    }

    // Serial.println("Failed to enable GPRS"); //? will be printed bt previous recursive call
    return false;
}

bool deactivateGPRS()
{

    if (GPRS_status() == 0)
    {
        Serial.println("GPRS already inactive");
        return true;
    }
    else
    {
        if (sendAndCheck("AT+CGATT=0", "OK"))
        {
            // query GPRS status
            GPRS_status();
            return true;
        }
        else
        {
            Serial.println("Failed to disable GPRS");
            return false;
        }
        return true;
    }
}

bool GSM_Serial_begin()
{
    pinMode(QUECTEL_PWR_KEY, OUTPUT);
    digitalWrite(QUECTEL_PWR_KEY, HIGH);

    GSMSerial.begin(115200);

    bool comm_init = false;

    int16_t timeout = 30000;

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

    return comm_init;
}

/// @brief Reset GSM module
/// @param seq: Sequence to to toggle reset pin to trigger a restart
/// @param timing_delay : Timing function for the reset to happen
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