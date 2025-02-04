/******************************************************************
 * The variables inside the cfg namespace are persistent          *
 * configuration values. They have defaults which can be          *
 * configured at compile-time via the ext_def.h file              *
 * They can be changed by the user via the web interface, the     *
 * changes are persisted to the flash and read back after reboot. *
 * Note that the names of these variables can't be easily changed *
 * as they are part of the json format used to persist the data.  *
 ******************************************************************/
#ifndef NAMESPACE_CFG_H
#define NAMESPACE_CFG_H

#include "defines.h"
#include "ext_def.h"
namespace cfg
{
    unsigned debug = DEBUG;

    unsigned time_for_wifi_config = 60000;
    unsigned sending_intervall_ms = 30000;

    char current_lang[3];

    // credentials for basic auth of internal web server
    bool www_basicauth_enabled = WWW_BASICAUTH_ENABLED;
    char www_username[LEN_WWW_USERNAME];
    char www_password[LEN_CFG_PASSWORD];

    // wifi credentials
    char wlanssid[LEN_WLANSSID];
    char wlanpwd[LEN_CFG_PASSWORD];

    // credentials of the sensor in access point mode
    char fs_ssid[LEN_FS_SSID] = FS_SSID;
    char fs_pwd[LEN_CFG_PASSWORD] = FS_PWD;

    // (in)active sensors
    bool dht_read = DHT_READ;
    bool htu21d_read = HTU21D_READ;
    bool ppd_read = PPD_READ;
    bool sds_read = SDS_READ;
    bool pms_read = PMS_READ;
    bool hpm_read = HPM_READ;
    bool sps30_read = SPS30_READ;
    bool bmp_read = BMP_READ;
    bool bmx280_read = BMX280_READ;
    bool sht3x_read = SHT3X_READ;
    bool ds18b20_read = DS18B20_READ;
    bool dnms_read = DNMS_READ;
    char dnms_correction[LEN_DNMS_CORRECTION] = DNMS_CORRECTION;
    bool gps_read = GPS_READ;
    bool gsm_capable = GSM_CAPABLE;

    // send to "APIs"
    bool send2cfa = SEND2CFA;
    bool send2dusti = SEND2SENSORCOMMUNITY;
    bool send2madavi = SEND2MADAVI;
    bool send2sensemap = SEND2SENSEMAP;
    bool send2fsapp = SEND2FSAPP;
    bool send2aircms = SEND2AIRCMS;
    bool send2custom = SEND2CUSTOM;
    bool send2influx = SEND2INFLUX;
    bool send2csv = SEND2CSV;

    bool auto_update = AUTO_UPDATE;
    bool use_beta = USE_BETA;

    // (in)active displays
    bool has_display = HAS_DISPLAY; // OLED with SSD1306 and I2C
    bool has_sh1106 = HAS_SH1106;
    bool has_flipped_display = HAS_FLIPPED_DISPLAY;
    bool has_lcd1602 = HAS_LCD1602;
    bool has_lcd1602_27 = HAS_LCD1602_27;
    bool has_lcd2004 = HAS_LCD2004;
    bool has_lcd2004_27 = HAS_LCD2004_27;

    bool display_wifi_info = DISPLAY_WIFI_INFO;
    bool display_device_info = DISPLAY_DEVICE_INFO;

    // API settings
    bool ssl_cfa = SSL_CFA;
    bool ssl_madavi = SSL_MADAVI;
    bool ssl_dusti = SSL_SENSORCOMMUNITY;
    char senseboxid[LEN_SENSEBOXID] = SENSEBOXID;
    unsigned port_cfa = PORT_CFA;

    char host_influx[LEN_HOST_INFLUX];
    char url_influx[LEN_URL_INFLUX];
    unsigned port_influx = PORT_INFLUX;
    char user_influx[LEN_USER_INFLUX] = USER_INFLUX;
    char pwd_influx[LEN_CFG_PASSWORD] = PWD_INFLUX;
    char measurement_name_influx[LEN_MEASUREMENT_NAME_INFLUX];
    bool ssl_influx = SSL_INFLUX;

    char host_custom[LEN_HOST_CUSTOM];
    char url_custom[LEN_URL_CUSTOM];
    bool ssl_custom = SSL_CUSTOM;
    unsigned port_custom = PORT_CUSTOM;
    char user_custom[LEN_USER_CUSTOM] = USER_CUSTOM;
    char pwd_custom[LEN_CFG_PASSWORD] = PWD_CUSTOM;

    char gsm_pin[LEN_GSM_PIN] = GSM_PIN;
    char gprs_apn[LEN_GPRS_APN] = GPRS_APN;
    char gprs_username[LEN_GPRS_USERNAME] = GPRS_USERNAME;
    char gprs_password[LEN_GPRS_PASSWORD] = GPRS_PASSWORD;

    void initNonTrivials(const char *id)
    {
        strcpy(cfg::current_lang, CURRENT_LANG);
        strcpy_P(www_username, WWW_USERNAME);
        strcpy_P(www_password, WWW_PASSWORD);
        strcpy_P(wlanssid, WLANSSID);
        strcpy_P(wlanpwd, WLANPWD);
        strcpy_P(host_custom, HOST_CUSTOM);
        strcpy_P(url_custom, URL_CUSTOM);
        strcpy_P(host_influx, HOST_INFLUX);
        strcpy_P(url_influx, URL_INFLUX);
        strcpy_P(measurement_name_influx, MEASUREMENT_NAME_INFLUX);
        strcpy_P(gsm_pin, GSM_PIN);
        strcpy_P(gprs_apn, GPRS_APN);
        strcpy_P(gprs_username, GPRS_USERNAME);
        strcpy_P(gprs_password, GPRS_USERNAME);

        if (!*fs_ssid)
        {
            strcpy(fs_ssid, SSID_BASENAME);
            strcat(fs_ssid, id);
        }
        else
        {
            strcat(fs_ssid, id);
        }
    }
}

#endif