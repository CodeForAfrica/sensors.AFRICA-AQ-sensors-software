/*****************************************************************
 * Webserver functions                                        *
 *****************************************************************/

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "languages/lang_def.h"
#include "html_helper.h"
#include "sensors-africa-logo.h"
#include "../namespace_cfg.h"

// Function declarations
static void setup_webserver();
static bool webserver_request_auth();
static void sendHttpRedirect();
static void webserver_root();
static void webserver_config_send_body_get(String &page_content);
static void webserver_config_send_body_post(String &page_content);
static void webserver_config();
static void webserver_wifi();
static void webserver_values();
static void webserver_status();
static void webserver_debug_level();
static void webserver_removeConfig();
static void webserver_reset();
static void webserver_data_json();
static void webserver_prometheus_endpoint();
static void webserver_images();
static void webserver_not_found();

// Function definitions

/*****************************************************************
 * Webserver setup                                               *
 *****************************************************************/
static void setup_webserver()
{
    server.on("/", webserver_root);
    server.on(F("/config"), webserver_config);
    server.on(F("/wifi"), webserver_wifi);
    server.on(F("/values"), webserver_values);
    server.on(F("/status"), webserver_status);
    server.on(F("/generate_204"), webserver_config);
    server.on(F("/fwlink"), webserver_config);
    server.on(F("/debug"), webserver_debug_level);
    server.on(F("/removeConfig"), webserver_removeConfig);
    server.on(F("/reset"), webserver_reset);
    server.on(F("/data.json"), webserver_data_json);
    server.on(F("/metrics"), webserver_prometheus_endpoint);
    server.on(F("/images"), webserver_images);
    server.onNotFound(webserver_not_found);

    debug_outln_info(F("Starting Webserver... "), WiFi.localIP().toString());
    server.begin();
}

/*****************************************************************
 * Webserver request auth: prompt for BasicAuth
 *
 * -Provide BasicAuth for all page contexts except /values and images
 *****************************************************************/
static bool webserver_request_auth()
{
    if (cfg::www_basicauth_enabled && !wificonfig_loop)
    {
        debug_outln_info(F("validate request auth..."));
        if (!server.authenticate(cfg::www_username, cfg::www_password))
        {
            server.requestAuthentication(BASIC_AUTH, "Sensor Login", F("Authentication failed"));
            return false;
        }
    }
    return true;
}

static void sendHttpRedirect()
{
    server.sendHeader(F("Location"), F("http://192.168.4.1/config"));
    server.send(302, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), emptyString);
}

/*****************************************************************
 * Webserver root: show all options                              *
 *****************************************************************/
static void webserver_root()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        sendHttpRedirect();
    }
    else
    {
        if (!webserver_request_auth())
        {
            return;
        }

        RESERVE_STRING(page_content, XLARGE_STR);
        start_html_page(page_content, emptyString);
        debug_outln_info(F("ws: root ..."));

        // Enable Pagination
        page_content += FPSTR(WEB_ROOT_PAGE_CONTENT);
        page_content.replace(F("{t}"), FPSTR(INTL_CURRENT_DATA));
        page_content.replace(F("{s}"), FPSTR(INTL_DEVICE_STATUS));
        page_content.replace(F("{conf}"), FPSTR(INTL_CONFIGURATION));
        page_content.replace(F("{restart}"), FPSTR(INTL_RESTART_SENSOR));
        page_content.replace(F("{debug_setting}"), FPSTR(INTL_DEBUG_SETTING_TO));
        end_html_page(page_content);
    }
}

/*****************************************************************
 * Webserver config: show config page                            *
 *****************************************************************/

static void webserver_config_send_body_get(String &page_content)
{
    using namespace cfg;

    auto add_form_checkbox = [&page_content](const ConfigShapeId cfgid, const String &info)
    {
        page_content += form_checkbox(cfgid, info, true);
    };

    auto add_form_checkbox_sensor = [&add_form_checkbox](const ConfigShapeId cfgid, __const __FlashStringHelper *info)
    {
        add_form_checkbox(cfgid, add_sensor_type(info));
    };

    debug_outln_info(F("begin webserver_config_body_get ..."));
    page_content += F("<form method='POST' action='/config' style='width:100%;'>\n<b>" INTL_WIFI_SETTINGS "</b><br/>");
    debug_outln_info(F("ws: config page 1"));
    if (wificonfig_loop)
    { // scan for wlan ssids
        page_content += F("<div id='wifilist'>" INTL_WIFI_NETWORKS "</div><br/>");
    }
    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_wlanssid, FPSTR(INTL_FS_WIFI_NAME), LEN_WLANSSID - 1);
    add_form_input(page_content, Config_wlanpwd, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);
    page_content += F("<hr/>\n<br/><b>");

    page_content += FPSTR(INTL_AB_HIER_NUR_ANDERN);
    page_content += FPSTR(WEB_B_BR);
    page_content += FPSTR(BR_TAG);

    // Paginate page after ~ 1500 Bytes
    server.sendContent(page_content);
    page_content = emptyString;

    add_form_checkbox(Config_www_basicauth_enabled, FPSTR(INTL_BASICAUTH));
    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_www_username, FPSTR(INTL_USER), LEN_WWW_USERNAME - 1);
    add_form_input(page_content, Config_www_password, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);
    page_content += FPSTR(BR_TAG);

    // Paginate page after ~ 1500 Bytes
    server.sendContent(page_content);

    if (!wificonfig_loop)
    {
        page_content = FPSTR(INTL_FS_WIFI_DESCRIPTION);
        page_content += FPSTR(BR_TAG);

        page_content += FPSTR(TABLE_TAG_OPEN);
        add_form_input(page_content, Config_fs_ssid, FPSTR(INTL_FS_WIFI_NAME), LEN_FS_SSID - 1);
        add_form_input(page_content, Config_fs_pwd, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
        page_content += FPSTR(TABLE_TAG_CLOSE_BR);

        // Paginate page after ~ 1500 Bytes
        server.sendContent(page_content);
    }
    page_content = FPSTR(WEB_BR_LF_B);
    page_content += F(INTL_FIRMWARE "</b>&nbsp;");
    add_form_checkbox(Config_auto_update, FPSTR(INTL_AUTO_UPDATE));
    add_form_checkbox(Config_use_beta, FPSTR(INTL_USE_BETA));

    page_content += FPSTR(TABLE_TAG_OPEN);
    page_content += form_select_lang();
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);

    page_content += F("<script>"
                      "var $ = function(e) { return document.getElementById(e); };"
                      "function updateOTAOptions() { "
                      "$('current_lang').disabled = $('use_beta').disabled = !$('auto_update').checked; "
                      "}; updateOTAOptions(); $('auto_update').onchange = updateOTAOptions;"
                      "</script>");

    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_debug, FPSTR(INTL_DEBUG_LEVEL), 1);
    add_form_input(page_content, Config_sending_intervall_ms, FPSTR(INTL_MEASUREMENT_INTERVAL), 5);
    add_form_input(page_content, Config_time_for_wifi_config, FPSTR(INTL_DURATION_ROUTER_MODE), 5);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);

    server.sendContent(page_content);
    page_content = FPSTR(WEB_BR_LF_B);
    page_content += FPSTR(INTL_MORE_SETTINGS);
    page_content += FPSTR(WEB_B_BR);

    add_form_checkbox(Config_has_display, FPSTR(INTL_DISPLAY));
    add_form_checkbox(Config_has_sh1106, FPSTR(INTL_SH1106));
    add_form_checkbox(Config_has_flipped_display, FPSTR(INTL_FLIP_DISPLAY));
    add_form_checkbox(Config_has_lcd1602_27, FPSTR(INTL_LCD1602_27));
    add_form_checkbox(Config_has_lcd1602, FPSTR(INTL_LCD1602_3F));

    // Paginate page after ~ 1500 Bytes
    server.sendContent(page_content);
    page_content = emptyString;

    add_form_checkbox(Config_has_lcd2004_27, FPSTR(INTL_LCD2004_27));
    add_form_checkbox(Config_has_lcd2004, FPSTR(INTL_LCD2004_3F));
    add_form_checkbox(Config_display_wifi_info, FPSTR(INTL_DISPLAY_WIFI_INFO));
    add_form_checkbox(Config_display_device_info, FPSTR(INTL_DISPLAY_DEVICE_INFO));

    server.sendContent(page_content);
    page_content = FPSTR(WEB_BR_LF_B);
    page_content += FPSTR(INTL_SENSORS);
    page_content += FPSTR(WEB_B_BR);
    add_form_checkbox_sensor(Config_sds_read, FPSTR(INTL_SDS011));
    add_form_checkbox_sensor(Config_hpm_read, FPSTR(INTL_HPM));
    add_form_checkbox_sensor(Config_sps30_read, FPSTR(INTL_SPS30));

    // Paginate page after ~ 1500 Bytes
    server.sendContent(page_content);
    page_content = emptyString;

    add_form_checkbox_sensor(Config_dht_read, FPSTR(INTL_DHT22));
    add_form_checkbox_sensor(Config_htu21d_read, FPSTR(INTL_HTU21D));
    add_form_checkbox_sensor(Config_bmx280_read, FPSTR(INTL_BMX280));
    add_form_checkbox_sensor(Config_sht3x_read, FPSTR(INTL_SHT3X));
    add_form_checkbox_sensor(Config_ds18b20_read, FPSTR(INTL_DS18B20));

    // Paginate page after ~ 1500 Bytes
    server.sendContent(page_content);
    page_content = emptyString;

    add_form_checkbox_sensor(Config_dnms_read, FPSTR(INTL_DNMS));
    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_dnms_correction, FPSTR(INTL_DNMS_CORRECTION), LEN_DNMS_CORRECTION - 1);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);

    page_content += FPSTR(WEB_BR_LF_B);
    page_content += FPSTR(INTL_MORE_SENSORS);
    page_content += FPSTR(WEB_B_BR);

    add_form_checkbox_sensor(Config_pms_read, FPSTR(INTL_PMS));
    add_form_checkbox_sensor(Config_bmp_read, FPSTR(INTL_BMP180));
    add_form_checkbox(Config_gps_read, FPSTR(INTL_NEO6M));

    page_content += FPSTR(WEB_BR_LF_B);
    page_content += F("GSM");
    page_content += FPSTR(WEB_B_BR);

    page_content += form_checkbox(Config_gsm_capable, F("GSM_capable"), false);
    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_gsm_pin, FPSTR(INTL_GSM_PIN), LEN_GSM_PIN - 1);
    add_form_input(page_content, Config_gprs_apn, FPSTR(INTL_GPRS_APN), LEN_GPRS_APN - 1);
    add_form_input(page_content, Config_gprs_username, FPSTR(INTL_GPRS_USERNAME), LEN_GPRS_USERNAME - 1);
    add_form_input(page_content, Config_gprs_password, FPSTR(INTL_GPRS_PASSWORD), LEN_GPRS_PASSWORD - 1);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);
    page_content += FPSTR(BR_TAG);

    page_content += FPSTR(WEB_BR_LF_B);
    page_content += F("APIs");
    page_content += FPSTR(WEB_B_BR);
    // Paginate page after ~ 1500 Bytes
    server.sendContent(page_content);

    page_content = form_checkbox(Config_send2dusti, F("API Sensor.Community"), false);
    page_content += FPSTR(WEB_NBSP_NBSP_BRACE);
    page_content += form_checkbox(Config_ssl_dusti, FPSTR(WEB_HTTPS), false);
    page_content += FPSTR(WEB_BRACE_BR);
    page_content += form_checkbox(Config_send2madavi, F("API Madavi.de"), false);
    page_content += FPSTR(WEB_NBSP_NBSP_BRACE);
    page_content += form_checkbox(Config_ssl_madavi, FPSTR(WEB_HTTPS), false);
    page_content += FPSTR(WEB_BRACE_BR);
    add_form_checkbox(Config_send2csv, tmpl(FPSTR(INTL_SEND_TO), FPSTR(WEB_CSV)));
    add_form_checkbox(Config_send2fsapp, tmpl(FPSTR(INTL_SEND_TO), FPSTR(WEB_FEINSTAUB_APP)));
    add_form_checkbox(Config_send2aircms, tmpl(FPSTR(INTL_SEND_TO), F("aircms.online")));
    add_form_checkbox(Config_send2sensemap, tmpl(FPSTR(INTL_SEND_TO), F("OpenSenseMap")));
    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_senseboxid, F("senseBox&nbsp;ID"), LEN_SENSEBOXID - 1);

    server.sendContent(page_content);
    page_content += FPSTR(BR_TAG);
    page_content += form_checkbox(Config_send2custom, FPSTR(INTL_SEND_TO_OWN_API), false);
    page_content += FPSTR(WEB_NBSP_NBSP_BRACE);
    page_content += form_checkbox(Config_ssl_custom, FPSTR(WEB_HTTPS), false);
    page_content += FPSTR(WEB_BRACE_BR);

    server.sendContent(page_content);
    page_content = FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_host_custom, FPSTR(INTL_SERVER), LEN_HOST_CUSTOM - 1);
    add_form_input(page_content, Config_url_custom, FPSTR(INTL_PATH), LEN_URL_CUSTOM - 1);
    add_form_input(page_content, Config_port_custom, FPSTR(INTL_PORT), MAX_PORT_DIGITS);
    add_form_input(page_content, Config_user_custom, FPSTR(INTL_USER), LEN_USER_CUSTOM - 1);
    add_form_input(page_content, Config_pwd_custom, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);

    page_content += FPSTR(BR_TAG);

    server.sendContent(page_content);
    page_content = form_checkbox(Config_send2influx, tmpl(FPSTR(INTL_SEND_TO), F("InfluxDB")), false);

    page_content += FPSTR(WEB_NBSP_NBSP_BRACE);
    page_content += form_checkbox(Config_ssl_influx, FPSTR(WEB_HTTPS), false);
    page_content += FPSTR(WEB_BRACE_BR);
    page_content += FPSTR(TABLE_TAG_OPEN);
    add_form_input(page_content, Config_host_influx, FPSTR(INTL_SERVER), LEN_HOST_INFLUX - 1);
    add_form_input(page_content, Config_url_influx, FPSTR(INTL_PATH), LEN_URL_INFLUX - 1);
    add_form_input(page_content, Config_port_influx, FPSTR(INTL_PORT), MAX_PORT_DIGITS);
    add_form_input(page_content, Config_user_influx, FPSTR(INTL_USER), LEN_USER_INFLUX - 1);
    add_form_input(page_content, Config_pwd_influx, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
    add_form_input(page_content, Config_measurement_name_influx, F("Measurement"), LEN_MEASUREMENT_NAME_INFLUX - 1);
    page_content += FPSTR(TABLE_TAG_CLOSE_BR);
    page_content += form_submit(FPSTR(INTL_SAVE_AND_RESTART));
    page_content += FPSTR(BR_TAG);
    page_content += FPSTR(WEB_BR_FORM);
    if (wificonfig_loop)
    { // scan for wlan ssids
        page_content += F("<script>window.setTimeout(load_wifi_list,1000);</script>");
    }

    server.sendContent(page_content);
    page_content = emptyString;
}

static void webserver_config_send_body_post(String &page_content)
{
    String masked_pwd;

    using namespace cfg;

    for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
    {
        ConfigShapeEntry c;
        memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
        const String s_param(c.cfg_key);
        if (!server.hasArg(s_param))
        {
            continue;
        }

        switch (c.cfg_type)
        {
        case Config_Type_UInt:
            *(c.cfg_val.as_uint) = server.arg(s_param).toInt();
            break;
        case Config_Type_Time:
            *(c.cfg_val.as_uint) = server.arg(s_param).toInt() * 1000;
            break;
        case Config_Type_Bool:
            *(c.cfg_val.as_bool) = (server.arg(s_param) == "1");
            break;
        case Config_Type_String:
            strncpy(c.cfg_val.as_str, server.arg(s_param).c_str(), c.cfg_len);
            c.cfg_val.as_str[c.cfg_len] = '\0';
            break;
        case Config_Type_Password:
            const String server_arg(server.arg(s_param));
            masked_pwd = emptyString;
            for (uint8_t i = 0; i < server_arg.length(); i++)
                masked_pwd += '*';
            if (masked_pwd != server_arg || !server_arg.length())
            {
                server_arg.toCharArray(c.cfg_val.as_str, LEN_CFG_PASSWORD);
            }
            break;
        }
    }

    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), F("Sensor.Community"), send2dusti);
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), F("Madavi"), send2madavi);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_DHT22), dht_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_HTU21D), htu21d_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_SDS011), sds_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_PMSx003), pms_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_HPM), hpm_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_SPS30), sps30_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_PPD42NS), ppd_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_BMX280), bmx280_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_SHT3X), sht3x_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_DS18B20), ds18b20_read);
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), FPSTR(SENSORS_DNMS), dnms_read);
    add_line_value(page_content, FPSTR(INTL_DNMS_CORRECTION), String(dnms_correction));
    add_line_value_bool(page_content, FPSTR(INTL_READ_FROM), F("GPS"), gps_read);

    // Paginate after ~ 1500 bytes
    server.sendContent(page_content);
    page_content = emptyString;

    add_line_value_bool(page_content, FPSTR(INTL_DISPLAY), has_display);
    add_line_value_bool(page_content, FPSTR(INTL_SH1106), has_sh1106);
    add_line_value_bool(page_content, FPSTR(INTL_FLIP_DISPLAY), has_flipped_display);
    add_line_value_bool(page_content, FPSTR(INTL_LCD1602_27), has_lcd1602_27);
    add_line_value_bool(page_content, FPSTR(INTL_LCD1602_3F), has_lcd1602);
    add_line_value_bool(page_content, FPSTR(INTL_LCD2004_27), has_lcd2004_27);
    add_line_value_bool(page_content, FPSTR(INTL_LCD2004_3F), has_lcd2004);
    add_line_value_bool(page_content, FPSTR(INTL_DISPLAY_WIFI_INFO), display_wifi_info);
    add_line_value_bool(page_content, FPSTR(INTL_DISPLAY_DEVICE_INFO), display_device_info);
    add_line_value_bool(page_content, FPSTR(INTL_AUTO_UPDATE), auto_update);
    add_line_value_bool(page_content, FPSTR(INTL_USE_BETA), use_beta);
    add_line_value(page_content, FPSTR(INTL_DEBUG_LEVEL), String(debug));
    add_line_value(page_content, FPSTR(INTL_MEASUREMENT_INTERVAL), String(sending_intervall_ms));
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), F("Feinstaub-App"), send2fsapp);
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), F("aircms.online"), send2aircms);
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), FPSTR(WEB_CSV), send2csv);
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), FPSTR(WEB_FEINSTAUB_APP), send2fsapp);
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO), F("opensensemap"), send2sensemap);

    // Paginate after ~ 1500 bytes
    server.sendContent(page_content);
    page_content = emptyString;

    page_content += F("<br/>senseBox-ID ");
    page_content += senseboxid;
    page_content += FPSTR(WEB_BR_BR);
    add_line_value_bool(page_content, FPSTR(INTL_SEND_TO_OWN_API), send2custom);
    add_line_value(page_content, FPSTR(INTL_SERVER), host_custom);
    add_line_value(page_content, FPSTR(INTL_PATH), url_custom);
    add_line_value(page_content, FPSTR(INTL_PORT), String(port_custom));
    add_line_value(page_content, FPSTR(INTL_USER), user_custom);
    add_line_value(page_content, FPSTR(INTL_PASSWORD), pwd_custom);
    page_content += F("<br/><br/>InfluxDB: ");
    page_content += String(send2influx);
    add_line_value(page_content, FPSTR(INTL_SERVER), host_influx);
    add_line_value(page_content, FPSTR(INTL_PATH), url_influx);
    add_line_value(page_content, FPSTR(INTL_PORT), String(port_influx));
    add_line_value(page_content, FPSTR(INTL_USER), user_influx);
    add_line_value(page_content, FPSTR(INTL_PASSWORD), pwd_influx);
    add_line_value(page_content, F("Measurement"), measurement_name_influx);
    add_line_value(page_content, F("SSL"), String(ssl_influx));
    page_content += FPSTR(WEB_BR_BR);
    page_content += FPSTR(INTL_SENSOR_IS_REBOOTING);

    server.sendContent(page_content);
    page_content = emptyString;
}

static void webserver_config()
{
    if (!webserver_request_auth())
    {
        return;
    }

    debug_outln_info(F("ws: config page ..."));

    server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
    server.sendHeader(F("Pragma"), F("no-cache"));
    server.sendHeader(F("Expires"), F("0"));
    // Enable Pagination (Chunked Transfer)
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);

    RESERVE_STRING(page_content, XLARGE_STR);

    start_html_page(page_content, FPSTR(INTL_CONFIGURATION));
    if (wificonfig_loop)
    { // scan for wlan ssids
        page_content += FPSTR(WEB_CONFIG_SCRIPT);
    }

    if (server.method() == HTTP_GET)
    {
        webserver_config_send_body_get(page_content);
    }
    else
    {
        webserver_config_send_body_post(page_content);
    }
    end_html_page(page_content);

    if (server.method() == HTTP_POST)
    {
        display_debug(F("Writing config"), F("and restarting"));
        writeConfig();
        sensor_restart();
    }
}

/*****************************************************************
 * Webserver wifi: show available wifi networks                  *
 *****************************************************************/
static void webserver_wifi()
{
    String page_content;

    debug_outln_info(F("wifi networks found: "), String(count_wifiInfo));
    if (count_wifiInfo == 0)
    {
        page_content += FPSTR(BR_TAG);
        page_content += FPSTR(INTL_NO_NETWORKS);
        page_content += FPSTR(BR_TAG);
    }
    else
    {
        std::unique_ptr<int[]> indices(new int[count_wifiInfo]);
        debug_outln_info(F("ws: wifi ..."));
        for (unsigned i = 0; i < count_wifiInfo; ++i)
        {
            indices[i] = i;
        }
        for (unsigned i = 0; i < count_wifiInfo; i++)
        {
            for (unsigned j = i + 1; j < count_wifiInfo; j++)
            {
                if (wifiInfo[indices[j]].RSSI > wifiInfo[indices[i]].RSSI)
                {
                    std::swap(indices[i], indices[j]);
                }
            }
        }
        int duplicateSsids = 0;
        for (int i = 0; i < count_wifiInfo; i++)
        {
            if (indices[i] == -1)
            {
                continue;
            }
            for (int j = i + 1; j < count_wifiInfo; j++)
            {
                if (strncmp(wifiInfo[indices[i]].ssid, wifiInfo[indices[j]].ssid, sizeof(wifiInfo[0].ssid)) == 0)
                {
                    indices[j] = -1; // set dup aps to index -1
                    ++duplicateSsids;
                }
            }
        }

        page_content += FPSTR(INTL_NETWORKS_FOUND);
        page_content += String(count_wifiInfo - duplicateSsids);
        page_content += FPSTR(BR_TAG);
        page_content += FPSTR(BR_TAG);
        page_content += FPSTR(TABLE_TAG_OPEN);
        // if (n > 30) n=30;
        for (int i = 0; i < count_wifiInfo; ++i)
        {
            if (indices[i] == -1
#if defined(ESP8266)
                || wifiInfo[indices[i]].isHidden
#endif
            )
            {
                continue;
            }
            // Print SSID and RSSI for each network found
#if defined(ESP8266)
            page_content += wlan_ssid_to_table_row(wifiInfo[indices[i]].ssid, ((wifiInfo[indices[i]].encryptionType == ENC_TYPE_NONE) ? " " : u8"🔒"), wifiInfo[indices[i]].RSSI);
#endif
#if defined(ESP32)
            page_content += wlan_ssid_to_table_row(wifiInfo[indices[i]].ssid, ((wifiInfo[indices[i]].encryptionType == WIFI_AUTH_OPEN) ? " " : u8"🔒"), wifiInfo[indices[i]].RSSI);
#endif
        }
        page_content += FPSTR(TABLE_TAG_CLOSE_BR);
        page_content += FPSTR(BR_TAG);
    }
    server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), page_content);
}

/*****************************************************************
 * Webserver root: show latest values                            *
 *****************************************************************/
static void webserver_values()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        sendHttpRedirect();
    }
    else
    {
        RESERVE_STRING(page_content, XLARGE_STR);
        start_html_page(page_content, FPSTR(INTL_CURRENT_DATA));
        const String unit_PM("µg/m³");
        const String unit_Deg("°");
        const String unit_T("°C");
        const String unit_H("%");
        const String unit_P("hPa");
        const String unit_NC(F("#/cm³"));
        const String unit_TS("µm");
        const String unit_LA(F("dB(A)"));

        const int signal_quality = calcWiFiSignalQuality(last_signal_strength);
        debug_outln_info(F("ws: values ..."));
        if (!count_sends)
        {
            page_content += F("<b style='color:red'>");
            add_warning_first_cycle(page_content);
            page_content += FPSTR(WEB_B_BR_BR);
        }
        else
        {
            add_age_last_values(page_content);
        }

        server.sendContent(page_content);
        page_content = F("<table cellspacing='0' border='1' cellpadding='5'>\n"
                         "<tr><th>" INTL_SENSOR "</th><th> " INTL_PARAMETER "</th><th>" INTL_VALUE "</th></tr>");
        if (cfg::ppd_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_PPD42NS), FPSTR(WEB_PM1), check_display_value(last_value_PPD_P1, -1, 1, 0), FPSTR(INTL_PARTICLES_PER_LITER));
            add_table_row_from_value(page_content, FPSTR(SENSORS_PPD42NS), FPSTR(WEB_PM25), check_display_value(last_value_PPD_P2, -1, 1, 0), FPSTR(INTL_PARTICLES_PER_LITER));
        }
        if (cfg::sds_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SDS011), FPSTR(WEB_PM25), check_display_value(last_value_SDS_P2, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SDS011), FPSTR(WEB_PM10), check_display_value(last_value_SDS_P1, -1, 1, 0), unit_PM);
        }
        if (cfg::pms_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_PMSx003), FPSTR(WEB_PM1), check_display_value(last_value_PMS_P0, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_PMSx003), FPSTR(WEB_PM25), check_display_value(last_value_PMS_P2, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_PMSx003), FPSTR(WEB_PM10), check_display_value(last_value_PMS_P1, -1, 1, 0), unit_PM);
        }
        if (cfg::hpm_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_HPM), FPSTR(WEB_PM25), check_display_value(last_value_HPM_P2, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_HPM), FPSTR(WEB_PM10), check_display_value(last_value_HPM_P1, -1, 1, 0), unit_PM);
        }
        if (cfg::sps30_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_PM1), check_display_value(last_value_SPS30_P0, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_PM25), check_display_value(last_value_SPS30_P2, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_PM4), check_display_value(last_value_SPS30_P4, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_PM10), check_display_value(last_value_SPS30_P1, -1, 1, 0), unit_PM);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_NC0k5), check_display_value(last_value_SPS30_N05, -1, 0, 0), unit_NC);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_NC1k0), check_display_value(last_value_SPS30_N1, -1, 0, 0), unit_NC);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_NC2k5), check_display_value(last_value_SPS30_N25, -1, 0, 0), unit_NC);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_NC4k0), check_display_value(last_value_SPS30_N4, -1, 0, 0), unit_NC);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_NC10), check_display_value(last_value_SPS30_N10, -1, 0, 0), unit_NC);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), FPSTR(WEB_TPS), check_display_value(last_value_SPS30_TS, -1, 1, 0), unit_TS);
        }

        if (cfg::dht_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_DHT22), FPSTR(INTL_TEMPERATURE), check_display_value(last_value_DHT_T, -128, 1, 0), unit_T);
            add_table_row_from_value(page_content, FPSTR(SENSORS_DHT22), FPSTR(INTL_HUMIDITY), check_display_value(last_value_DHT_H, -1, 1, 0), unit_H);
        }
        if (cfg::htu21d_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_HTU21D), FPSTR(INTL_TEMPERATURE), check_display_value(last_value_HTU21D_T, -128, 1, 0), unit_T);
            add_table_row_from_value(page_content, FPSTR(SENSORS_HTU21D), FPSTR(INTL_HUMIDITY), check_display_value(last_value_HTU21D_H, -1, 1, 0), unit_H);
        }
        if (cfg::bmp_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_BMP180), FPSTR(INTL_TEMPERATURE), check_display_value(last_value_BMP_T, -128, 1, 0), unit_T);
            add_table_row_from_value(page_content, FPSTR(SENSORS_BMP180), FPSTR(INTL_PRESSURE), check_display_value(last_value_BMP_P / 100.0f, (-1 / 100.0f), 2, 0), unit_P);
        }
        if (cfg::bmx280_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_BMX280), FPSTR(INTL_TEMPERATURE), check_display_value(last_value_BMX280_T, -128, 1, 0), unit_T);
            add_table_row_from_value(page_content, FPSTR(SENSORS_BMX280), FPSTR(INTL_PRESSURE), check_display_value(last_value_BMX280_P / 100.0f, (-1 / 100.0f), 2, 0), unit_P);
            if (bmx280.sensorID() == BME280_SENSOR_ID)
            {
                add_table_row_from_value(page_content, FPSTR(SENSORS_BMX280), FPSTR(INTL_HUMIDITY), check_display_value(last_value_BME280_H, -1, 1, 0), unit_H);
            }
        }
        if (cfg::sht3x_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SHT3X), FPSTR(INTL_TEMPERATURE), check_display_value(last_value_SHT3X_T, -128, 1, 0), unit_T);
            add_table_row_from_value(page_content, FPSTR(SENSORS_SHT3X), FPSTR(INTL_HUMIDITY), check_display_value(last_value_SHT3X_H, -1, 1, 0), unit_H);
        }
        if (cfg::ds18b20_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_DS18B20), FPSTR(INTL_TEMPERATURE), check_display_value(last_value_DS18B20_T, -128, 1, 0), unit_T);
        }
        if (cfg::dnms_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(SENSORS_DNMS), FPSTR(INTL_LEQ_A), check_display_value(last_value_dnms_laeq, -1, 1, 0), unit_LA);
            add_table_row_from_value(page_content, FPSTR(SENSORS_DNMS), FPSTR(INTL_LA_MIN), check_display_value(last_value_dnms_la_min, -1, 1, 0), unit_LA);
            add_table_row_from_value(page_content, FPSTR(SENSORS_DNMS), FPSTR(INTL_LA_MAX), check_display_value(last_value_dnms_la_max, -1, 1, 0), unit_LA);
        }
        if (cfg::gps_read)
        {
            page_content += FPSTR(EMPTY_ROW);
            add_table_row_from_value(page_content, FPSTR(WEB_GPS), FPSTR(INTL_LATITUDE), check_display_value(last_value_GPS_lat, -200.0, 6, 0), unit_Deg);
            add_table_row_from_value(page_content, FPSTR(WEB_GPS), FPSTR(INTL_LONGITUDE), check_display_value(last_value_GPS_lon, -200.0, 6, 0), unit_Deg);
            add_table_row_from_value(page_content, FPSTR(WEB_GPS), FPSTR(INTL_ALTITUDE), check_display_value(last_value_GPS_alt, -1000.0, 2, 0), "m");
            add_table_row_from_value(page_content, FPSTR(WEB_GPS), FPSTR(INTL_DATE), last_value_GPS_date, emptyString);
            add_table_row_from_value(page_content, FPSTR(WEB_GPS), FPSTR(INTL_TIME), last_value_GPS_time, emptyString);
        }

        server.sendContent(page_content);
        page_content = FPSTR(EMPTY_ROW);

        add_table_row_from_value(page_content, F("WiFi"), FPSTR(INTL_SIGNAL_STRENGTH), String(last_signal_strength), "dBm");
        add_table_row_from_value(page_content, F("WiFi"), FPSTR(INTL_SIGNAL_QUALITY), String(signal_quality), "%");

        page_content += FPSTR(EMPTY_ROW);
        page_content += FPSTR(TABLE_TAG_CLOSE_BR);
        end_html_page(page_content);
    }
}

/*****************************************************************
 * Webserver root: show device status
 *****************************************************************/
static void webserver_status()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        sendHttpRedirect();
        return;
    }
    RESERVE_STRING(page_content, XLARGE_STR);
    start_html_page(page_content, FPSTR(INTL_DEVICE_STATUS));

    debug_outln_info(F("ws: status ..."));
    server.sendContent(page_content);
    page_content = F("<table cellspacing='0' border='1' cellpadding='5'>\n"
                     "<tr><th> " INTL_PARAMETER "</th><th>" INTL_VALUE "</th></tr>");
    String versionHtml(SOFTWARE_VERSION);
    versionHtml += F("/ST:");
    versionHtml += String(!airrohr_selftest_failed);
    versionHtml += '/';
    versionHtml += ESP.getFullVersion();
    versionHtml.replace("/", FPSTR(BR_TAG));
    add_table_row_from_value(page_content, FPSTR(INTL_FIRMWARE), versionHtml);
    add_table_row_from_value(page_content, F("Free Memory"), String(ESP.getFreeHeap()));
    add_table_row_from_value(page_content, F("Heap Fragmentation"), String(ESP.getHeapFragmentation()), "%");
    if (cfg::auto_update)
    {
        add_table_row_from_value(page_content, F("Last OTA"), delayToString(millis() - last_update_attempt));
    }

#if defined(ESP8266)
    add_table_row_from_value(page_content, F("NTP Sync"), String(sntp_time_set));
    StreamString ntpinfo;

    for (unsigned i = 0; i < SNTP_MAX_SERVERS; i++)
    {
        const ip_addr_t *sntp = sntp_getserver(i);
        if (sntp && !ip_addr_isany(sntp))
        {
            const char *name = sntp_getservername(i);
            if (!ntpinfo.isEmpty())
            {
                ntpinfo.print(FPSTR(BR_TAG));
            }
            ntpinfo.printf_P(PSTR("%s (%s)"), IPAddress(*sntp).toString().c_str(), name ? name : "?");
            ntpinfo.printf_P(PSTR(" reachable: %s"), sntp_getreachability(i) ? "Yes" : "No");
        }
    }
    add_table_row_from_value(page_content, F("NTP Info"), ntpinfo);
#endif

    time_t now = time(nullptr);
    add_table_row_from_value(page_content, FPSTR(INTL_TIME), ctime(&now));
    add_table_row_from_value(page_content, F("Uptime"), delayToString(millis() - time_point_device_start_ms));
    add_table_row_from_value(page_content, F("Reset Reason"), ESP.getResetReason());
    if (cfg::sds_read)
    {
        page_content += FPSTR(EMPTY_ROW);
        add_table_row_from_value(page_content, FPSTR(SENSORS_SDS011), last_value_SDS_version);
    }

    page_content += FPSTR(EMPTY_ROW);
    page_content += F("<tr><td colspan='2'><b>" INTL_ERROR "</b></td></tr>");
    add_table_row_from_value(page_content, F("WiFi"), String(WiFi_error_count));
    if (last_update_returncode != 0)
    {
        add_table_row_from_value(page_content, F("OTA Return"),
                                 last_update_returncode > 0 ? String(last_update_returncode) : HTTPClient::errorToString(last_update_returncode));
    }
    add_table_row_from_value(page_content, F("Data Send"), String(sendData_error_count));
    if (last_sendData_returncode != 0)
    {
        add_table_row_from_value(page_content, F("Data Send Return"),
                                 last_sendData_returncode > 0 ? String(last_sendData_returncode) : HTTPClient::errorToString(last_sendData_returncode));
    }
    if (cfg::sds_read)
    {
        add_table_row_from_value(page_content, FPSTR(SENSORS_SDS011), String(SDS_error_count));
    }
    if (cfg::sps30_read)
    {
        add_table_row_from_value(page_content, FPSTR(SENSORS_SPS30), String(SPS30_read_error_counter));
    }
    page_content += FPSTR(EMPTY_ROW);
    page_content += F("<tr><td colspan='2'><b>"
                      "LOG COUNT"
                      "</b></td></tr>");

    server.sendContent(page_content);
    page_content = emptyString;

    if (count_sends > 0)
    {
        page_content += FPSTR(EMPTY_ROW);
        add_table_row_from_value(page_content, F(INTL_NUMBER_OF_MEASUREMENTS), String(count_sends));
        if (sending_time > 0)
        {
            add_table_row_from_value(page_content, F(INTL_TIME_SENDING_MS), String(sending_time), "ms");
        }
    }

    page_content += FPSTR(TABLE_TAG_CLOSE_BR);
    end_html_page(page_content);
}

/*****************************************************************
 * Webserver set debug level                                     *
 *****************************************************************/
static void webserver_debug_level()
{
    if (!webserver_request_auth())
    {
        return;
    }

    RESERVE_STRING(page_content, LARGE_STR);
    start_html_page(page_content, FPSTR(INTL_DEBUG_LEVEL));
    debug_outln_info(F("ws: debug level ..."));

    if (server.hasArg("lvl"))
    {
        const int lvl = server.arg("lvl").toInt();
        if (lvl >= 0 && lvl <= 5)
        {
            cfg::debug = lvl;
            page_content += F("<h3>");
            page_content += FPSTR(INTL_DEBUG_SETTING_TO);
            page_content += ' ';

            const __FlashStringHelper *lvlText;
            switch (lvl)
            {
            case DEBUG_ERROR:
                lvlText = F(INTL_ERROR);
                break;
            case DEBUG_WARNING:
                lvlText = F(INTL_WARNING);
                break;
            case DEBUG_MIN_INFO:
                lvlText = F(INTL_MIN_INFO);
                break;
            case DEBUG_MED_INFO:
                lvlText = F(INTL_MED_INFO);
                break;
            case DEBUG_MAX_INFO:
                lvlText = F(INTL_MAX_INFO);
                break;
            default:
                lvlText = F(INTL_NONE);
            }

            page_content += lvlText;
            page_content += F(".</h3>");
        }
    }
    end_html_page(page_content);
}

/*****************************************************************
 * Webserver remove config                                       *
 *****************************************************************/
static void webserver_removeConfig()
{
    if (!webserver_request_auth())
    {
        return;
    }

    RESERVE_STRING(page_content, LARGE_STR);
    start_html_page(page_content, FPSTR(INTL_DELETE_CONFIG));
    debug_outln_info(F("ws: removeConfig ..."));

    if (server.method() == HTTP_GET)
    {
        page_content += FPSTR(WEB_REMOVE_CONFIG_CONTENT);
    }
    else
    {
        // Silently remove the desaster backup
        SPIFFS.remove(F("/config.json.old"));
        if (SPIFFS.exists(F("/config.json")))
        { // file exists
            debug_outln_info(F("removing config.json..."));
            if (SPIFFS.remove(F("/config.json")))
            {
                page_content += F("<h3>" INTL_CONFIG_DELETED ".</h3>");
            }
            else
            {
                page_content += F("<h3>" INTL_CONFIG_CAN_NOT_BE_DELETED ".</h3>");
            }
        }
        else
        {
            page_content += F("<h3>" INTL_CONFIG_NOT_FOUND ".</h3>");
        }
    }
    end_html_page(page_content);
}

/*****************************************************************
 * Webserver reset NodeMCU                                       *
 *****************************************************************/
static void webserver_reset()
{
    if (!webserver_request_auth())
    {
        return;
    }

    String page_content;
    page_content.reserve(512);

    start_html_page(page_content, FPSTR(INTL_RESTART_SENSOR));
    debug_outln_info(F("ws: reset ..."));

    if (server.method() == HTTP_GET)
    {
        page_content += FPSTR(WEB_RESET_CONTENT);
    }
    else
    {
        sensor_restart();
    }
    end_html_page(page_content);
}

/*****************************************************************
 * Webserver data.json                                           *
 *****************************************************************/
static void webserver_data_json()
{
    String s1;
    unsigned long age = 0;

    debug_outln_info(F("ws: data json..."));
    if (!count_sends)
    {
        s1 = FPSTR(data_first_part);
        s1 += "]}";
        age = cfg::sending_intervall_ms - msSince(starttime);
        if (age > cfg::sending_intervall_ms)
        {
            age = 0;
        }
        age = 0 - age;
    }
    else
    {
        s1 = last_data_string;
        debug_outln(last_data_string, DEBUG_MED_INFO);
        age = msSince(starttime);
        if (age > cfg::sending_intervall_ms)
        {
            age = 0;
        }
    }
    String s2 = F(", \"age\":\"");
    s2 += String((long)((age + 500) / 1000));
    s2 += F("\", \"sensordatavalues\"");
    s1.replace(F(", \"sensordatavalues\""), s2);
    server.send(200, FPSTR(TXT_CONTENT_TYPE_JSON), s1);
}

/*****************************************************************
 * Webserver prometheus metrics endpoint                         *
 *****************************************************************/
static void webserver_prometheus_endpoint()
{
    debug_outln_info(F("ws: prometheus endpoint..."));
    String data_4_prometheus = F("software_version{version=\"" SOFTWARE_VERSION_STR "\",{id}} 1\nuptime_ms{{id}} {up}\nsending_intervall_ms{{id}} {si}\nnumber_of_measurements{{id}} {cs}\n");
    debug_outln_info(F("Parse JSON for Prometheus"));
    String id(F("node=\"" SENSOR_BASENAME));
    id += esp_chipid;
    id += '\"';
    data_4_prometheus.replace("{id}", id);
    data_4_prometheus.replace("{up}", String(msSince(time_point_device_start_ms)));
    data_4_prometheus.replace("{si}", String(cfg::sending_intervall_ms));
    data_4_prometheus.replace("{cs}", String(count_sends));
    DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
    DeserializationError err = deserializeJson(json2data, last_data_string);
    if (!err)
    {
        for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
        {
            data_4_prometheus += measurement["value_type"].as<char *>();
            data_4_prometheus += '{';
            data_4_prometheus += id;
            data_4_prometheus += "} ";
            data_4_prometheus += measurement["value"].as<char *>();
            data_4_prometheus += '\n';
        }
        data_4_prometheus += F("last_sample_age_ms{");
        data_4_prometheus += id;
        data_4_prometheus += "} ";
        data_4_prometheus += String(msSince(starttime));
        data_4_prometheus += '\n';
    }
    else
    {
        debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
    }
    debug_outln(data_4_prometheus, DEBUG_MED_INFO);
    server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), data_4_prometheus);
}

/*****************************************************************
 * Webserver Images                                              *
 *****************************************************************/
static void webserver_images()
{
    server.sendHeader(F("Cache-Control"), F("max-age=2592000, public"));

    if (server.arg("name") == F("sensorsafrica_logo"))
    {
        debug_outln_info(F("ws: sensorsafrica_logo..."));
        server.send_P(200, TXT_CONTENT_TYPE_IMAGE_PNG,
                      SENSORSAFRICA_LOGO, SENSORSAFRICA_LOGO_PNG_SIZE);
    }
    else
    {
        webserver_not_found();
    }
}

/*****************************************************************
 * Webserver page not found                                      *
 *****************************************************************/
static void webserver_not_found()
{
    last_page_load = millis();
    debug_outln_info(F("ws: not found ..."));
    if (WiFi.status() != WL_CONNECTED)
    {
        if ((server.uri().indexOf(F("success.html")) != -1) || (server.uri().indexOf(F("detect.html")) != -1))
        {
            server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), FPSTR(WEB_IOS_REDIRECT));
        }
        else
        {
            sendHttpRedirect();
        }
    }
    else
    {
        server.send(404, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), F("Not found."));
    }
}

#endif