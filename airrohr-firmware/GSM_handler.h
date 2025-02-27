#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include "ext_def.h"

// SoftwareSerial fonaSS(FONA_TX, FONA_RX);
#define MCU_RXD D5
#define MCU_TXD D6
#define QUECTEL_PWR_KEY D8
#define QUECTEL_DTR D9
SoftwareSerial fonaSS(MCU_RXD, MCU_TXD); // Testing Quectel Board
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

char SIM_PIN[5] = GSM_PIN;
bool GSM_CONNECTED = false;
bool SIM_AVAILABLE = false;
bool GPRS_CONNECTED = false;
bool SIM_PIN_SET = false;
bool SIM_USABLE = false;
char SIM_CID[21] = "";
String GSM_INIT_ERROR = "";
String NETWORK_NAME = "";

/**** Function Declacrations **/
bool GSM_init(SoftwareSerial *gsm_serial);
static void unlock_pin(char *PIN);
String handle_AT_CMD(String cmd, int _delay = 1000);
void SIM_PIN_Setup();
bool is_SIMCID_valid();
bool GPRS_init();
void GSM_soft_reset();
void restart_GSM();
void enableGPRS();
void flushSerial();
void QUECTEL_POST(char *url, String headers[], int header_size, const String &data, int data_length);
int GPRS_INIT_FAIL_COUNT = 0;
int HTTP_POST_FAIL = 0;
// Set a decent delay before this to warm up the GSM module
bool GSM_init(SoftwareSerial *gsm_serial)
{ // Pass a ptr to SoftwareSerial GSM instance
    gsm_serial->begin(115200);
    String error_msg = "";
    // Check if there is serial communication with a GSM module
    /* if (!fona.begin(*gsm_serial)) */
    fona.begin(*gsm_serial);
    // digitalWrite(QUECTEL_PWR_KEY, LOW);
    // delay(3000);
    // digitalWrite(QUECTEL_PWR_KEY, HIGH);
    // delay(5000);
    if (!fona.sendCheckReply(F("AT"), F("OK")))
    {
        error_msg = "Could not find GSM module";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        GSM_CONNECTED = false;
        return false;
    }

    Serial.print("GSM module found!");

    // Check if SIM is inserted
    if (!is_SIMCID_valid())
    {
        error_msg = "Could not get SIM CID";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        return false;
    }

    // SIM setup
    Serial.println("SIM card available");
    Serial.println("Setting up SIM..");

    SIM_PIN_Setup();

    if (!SIM_PIN_SET)
    {
        error_msg = "Unable to set SIM PIN";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);

        return false;
    }
    // Set if SIM is usable flag
    SIM_USABLE = true;

    // Register to network
    bool registered_to_network = false;
    int retry_count = 0;
    while (!registered_to_network && retry_count < 10)
    {
        if (fona.getNetworkStatus() == 1 || 5)
            registered_to_network = true;

        retry_count++;
        delay(3000);
    }

    if (!registered_to_network)
    {
        error_msg = "Could not register to network";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        return false;
    }

    fona.sendCheckReply(F("AT+COPS?"), F("OK"));

    // Set GPRS APN details
    // fona.setGPRSNetworkSettings(F(GPRS_APN), F(GPRS_USERNAME), F(GPRS_PASSWORD));

    // Attempt to enable GPRS
    Serial.println("Attempting to enable GPRS");
    // delay(2000);

    if (!GPRS_init())
        return false;

    Serial.println("GPRS enabled!");

    GPRS_CONNECTED = true;
    // ToDo: Attempt to do a ping test to determine whether we can communicate with the internet

    return true;
}

static void unlock_pin(char *PIN)
{
    // flushSerial();

    // Attempt to SET PIN if not empty
    Serial.print("GSM CONFIG SET PIN: ");
    Serial.println(PIN);
    Serial.print("Length of PIN");
    Serial.println(strlen(PIN));
    if (strlen(PIN) > 1)
    {
        // debug_outln(F("\nAttempting to Unlock SIM please wait: "), DEBUG_MIN_INFO);
        Serial.print("Attempting to unlock SIM using PIN: ");
        Serial.println(PIN);
        if (!fona.unlockSIM(PIN))
        {
            // debug_outln(F("Failed to Unlock SIM card with pin: "), DEBUG_MIN_INFO);
            Serial.print("Failed to Unlock SIM card with PIN: ");
            // debug_outln(gsm_pin, DEBUG_MIN_INFO);
            Serial.println(PIN);
            SIM_PIN_SET = false;
            return;
        }

        SIM_PIN_SET = true;
    }
}

String handle_AT_CMD(String cmd, int _delay)
{
    while (Serial.available() > 0)
    {
        Serial.read();
    }
    String RESPONSE = "";
    fona.println(cmd);
    int sendStartMillis = millis();
    // delay(_delay); // Avoid putting any code that might delay the receiving all contents from the serial buffer as it is quickly filled up
    do
    {
        if (fona.available())
        {
            RESPONSE += fona.readString();
        }

        delay(2);
    } while (RESPONSE == "" || (millis() - sendStartMillis < _delay));

    Serial.println();
    Serial.println("GSM RESPONSE:");
    Serial.println("-------");
    Serial.print(RESPONSE);
    Serial.println("-----");

    return RESPONSE;
}

void SIM_PIN_Setup()
{

    // String res = handle_AT_CMD("AT+CPIN?");
    // int start_index = res.indexOf(":");
    // res = res.substring(start_index + 1);
    // res.trim();
    // Serial.print("PIN STATUS: ");
    // Serial.println(res);
    // if (res.startsWith("READY"))
    // {
    //     SIM_PIN_SET = true;
    //     return;
    // }

    // else if (res.startsWith("SIM PIN"))
    // {
    //     unlock_pin(SIM_PIN);
    //     return;
    // }
    // else if (res.startsWith("SIM PUK"))
    // { // ToDo: Attempt to set PUK;
    //     return;
    // }

    if (fona.sendCheckReply(F("AT+CPIN?"), F("+CPIN: READY"), 3000))
    {
        Serial.println("SIM PIN READY");
        SIM_PIN_SET = true;
        return;
    }

    else
    {
        Serial.println("SIM PIN NOT SET");
        return;
    }
}

bool is_SIMCID_valid() // ! Seems to be returning true even when there is "ERROR" in response
{
    // char res[30];
    // fona.getSIMCCID(res);
    // Serial.println(res);
    // String ccid = String(res);
    String ccid = handle_AT_CMD("AT+CCID");
    if (ccid.indexOf("ERROR") > -1) // Means string has the word error
    {
        SIM_AVAILABLE = false;
        return false;
    }

    else
    {
        // strcpy(SIM_CID, res);
        SIM_AVAILABLE = true;
        Serial.print("SIM CCID: ");
        Serial.println(ccid);
        return true;
    }
}

// Similar to FONA enableGPRS() but quicker because APN setting are not configured as it is configured during GSM_init()
bool GPRS_init()
{

    String err = "";

    // if (!fona.sendCheckReply(F("AT+CGATT=1"), F("OK"), 10000))
    // {
    //     err = "Failed to attach GPRS service";
    //     GSM_INIT_ERROR = err;
    //     Serial.println(err);
    //     GPRS_CONNECTED = false;
    //     return GPRS_CONNECTED;
    // }
    if (fona.sendCheckReply(F("AT+CGATT?"), F("0"))) // equivalent to fona.GPRSstate()
    {
        if (!fona.sendCheckReply(F("AT+CGATT=1"), F("OK"), 3000))
        {
            err = "Failed to attach GPRS service";
            GSM_INIT_ERROR = err;
            Serial.println(err);
            GPRS_CONNECTED = false;
            return GPRS_CONNECTED;
        }
    }

#ifdef QUECTEL
    Serial.println("Quectel GPRS init...");

    if (!fona.sendCheckReply(F("AT+QICSGP=1,1"), F("OK"), 3000))
    {
        err = "Failed to config GPRS PDP context";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        GPRS_CONNECTED = false;
        return GPRS_CONNECTED;
    }

    if (!fona.sendCheckReply(F("AT+QIACT=1"), F("OK"), 3000))
    {
        err = "Failed to activate GPRS PDP context";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        GPRS_CONNECTED = false;
        return GPRS_CONNECTED;
    }

#else
    String res = handle_AT_CMD("AT+SAPBR=1,1"); // Enable GPRS
    String res = handle_AT_CMD("AT+QCFG=\"gprsattach\",1");
    if (res.indexOf("OK") == -1)
    {
        err = "Failed to enable GPRS";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        GPRS_CONNECTED = false;
        return GPRS_CONNECTED;
    }
#endif

    GPRS_CONNECTED = true;
    return GPRS_CONNECTED;
}

void GSM_soft_reset()
{
    // #ifdef QUECTEL
    //     // ! Observation per v1 of Quectel PCB is that it POWERS BACK ON immediately after sending POWER DOWN command
    //     if (fona.sendCheckReply(F("AT+QPOWD"), F("POWERED DOWN")))
    //     {
    //         Serial.println("Restarting QUECTEL GSM");
    //         delay(10000); // Give module enough time to register to network
    //     }
    //     else
    //     {
    //         Serial.println("Failed to power down Quectel module");
    //     }

    // #else
    fona.enableGPRS(false); // basically shut down GPRS service

    if (!fona.sendCheckReply(F("AT+CFUN=1,1"), F("OK")))
    {
        Serial.println("Soft resetting GSM with full functionality failed!");
        return;
    }
    Serial.println("Soft resetting the GSM module...");
    delay(30000); // wait for GSM to warm up
    // #endif

    // if (!GSM_init(fonaSerial))
    // {
    //     Serial.println("GSM not fully configured");
    //     Serial.print("Failure point: ");
    //     Serial.println(GSM_INIT_ERROR);
    //     Serial.println();
    // }
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
    // GSM_soft_reset();
    // if (!fona.begin(*fonaSerial))
    // {
    //     Serial.println("Couldn't find GSM");
    //     return;
    // }

    if (!GSM_init(fonaSerial))
    {
        Serial.println("GSM not fully configured");
        Serial.print("Failure point: ");
        Serial.println(GSM_INIT_ERROR);
        Serial.println();
    }
}

void enableGPRS()
{
    // fona.setGPRSNetworkSettings(FONAFlashStringPtr(gprs_apn), FONAFlashStringPtr(gprs_username), FONAFlashStringPtr(gprs_password));

    int retry_count = 0;
    while ((fona.GPRSstate() != 0) && (retry_count < 40))
    {
        delay(3000);
        fona.enableGPRS(true);
        retry_count++;
    }

    fona.enableGPRS(true);
}

void disableGPRS()
{
    fona.enableGPRS(false);
    GPRS_CONNECTED = false;
}

/*****************************************************************
/* flushSerial                                                   *
/*****************************************************************/
void flushSerial()
{
    while (fonaSS.available())
        fonaSS.read();
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
    handle_AT_CMD(HTTP_CFG);

    fona.sendCheckReply(F("AT+QHTTPCFG=\"contextid\",1"), F("OK"));      // set context id
    fona.sendCheckReply(F("AT+QHTTPCFG=\"requestheader\",0"), F("OK"));  // disable request headers
    fona.sendCheckReply(F("AT+QHTTPCFG=\"responseheader\",1"), F("OK")); // enable response headers
    fona.sendCheckReply(F("AT+QHTTPCFG=\"rspout/auto\",1"), F("OK"));    // enable auto response and "disable" HTTTPREAD

    for (int i = 0; i < header_size; i++)
    {
        HTTP_CFG = "AT+QHTTPCFG=\"header\",\"" + headers[i] + "\"";
        Serial.println(HTTP_CFG);
        // fonaSerial->println(HTTP_CFG);
        handle_AT_CMD(HTTP_CFG);
    }

    // POST data
    HTTP_CFG = "AT+QHTTPPOST=" + String(data_length) + ",30,60";
    Serial.println(HTTP_CFG);
    // fonaSerial->println(HTTP_CFG);
    String res = handle_AT_CMD(HTTP_CFG);
    // if (res.indexOf("OK") == -1)
    // {
    //     HTTP_POST_FAIL += 1;
    //     if (HTTP_POST_FAIL > 5)
    //     {
    //         HTTP_POST_FAIL = 0;
    //         GSM_soft_reset();
    //     }
    // }
    Serial.print("Quectel post body: ");
    Serial.println(data);
    res = handle_AT_CMD(data, 10000);
    // if (res.indexOf("OK") == -1)
    // {
    //     HTTP_POST_FAIL += 1;
    //     if (HTTP_POST_FAIL > 5)
    //     {
    //         HTTP_POST_FAIL = 0;
    //         GSM_soft_reset();
    //     }
    // }
}

// Testing data
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
