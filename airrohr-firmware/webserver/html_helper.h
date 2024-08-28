/*****************************************************************
 * html helper functions                                         *
 *****************************************************************/

#ifndef HTML_HELPER_H
#define HTML_HELPER_H

#include "html-content.h"

// Function declarations --------------------------------

static void start_html_page(String &page_content, const String &title);
static void end_html_page(String &page_content);
static void add_form_input(String &page_content, const ConfigShapeId cfgid, const __FlashStringHelper *info, const int length);
static String form_checkbox(const ConfigShapeId cfgid, const String &info, const bool linebreak);
static String form_submit(const String &value);
static String form_select_lang();
static String tmpl(const __FlashStringHelper *patt, const String &value);
static void add_line_value(String &s, const __FlashStringHelper *name, const String &value);
static void add_line_value_bool(String &s, const __FlashStringHelper *name, const bool value);
static void add_line_value_bool(String &s, const __FlashStringHelper *patt, const __FlashStringHelper *name, const bool value);
static void add_table_row_from_value(String &page_content, const __FlashStringHelper *sensor, const __FlashStringHelper *param, const String &value, const String &unit);
static void add_table_row_from_value(String &page_content, const __FlashStringHelper *param, const String &value, const char *unit = nullptr);
static int32_t calcWiFiSignalQuality(int32_t rssi);
static String wlan_ssid_to_table_row(const String &ssid, const String &encryption, int32_t rssi);
static void add_warning_first_cycle(String &page_content);
static void add_age_last_values(String &s);
static String add_sensor_type(const String &sensor_text);

// Function definations

static void start_html_page(String &page_content, const String &title)
{
    last_page_load = millis();

    RESERVE_STRING(s, LARGE_STR);
    s = FPSTR(WEB_PAGE_HEADER);
    s.replace("{t}", title);
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), s);

    server.sendContent_P(WEB_PAGE_HEADER_HEAD);

    s = FPSTR(WEB_PAGE_HEADER_BODY);
    s.replace("{t}", title);
    if (title != " ")
    {
        s.replace("{n}", F("&raquo;"));
    }
    else
    {
        s.replace("{n}", emptyString);
    }
    s.replace("{id}", esp_chipid);
    s.replace("{mac}", WiFi.macAddress());
    page_content += s;
}

static void end_html_page(String &page_content)
{
    if (page_content.length())
    {
        server.sendContent(page_content);
    }
    server.sendContent_P(WEB_PAGE_FOOTER);
}

static void add_form_input(String &page_content, const ConfigShapeId cfgid, const __FlashStringHelper *info, const int length)
{
    RESERVE_STRING(s, MED_STR);
    s = F("<tr>"
          "<td title='[&lt;= {l}]'>{i}:&nbsp;</td>"
          "<td style='width:{l}em'>"
          "<input type='{t}' name='{n}' id='{n}' placeholder='{i}' value='{v}' maxlength='{l}'/>"
          "</td></tr>");
    String t_value;
    ConfigShapeEntry c;
    memcpy_P(&c, &configShape[cfgid], sizeof(ConfigShapeEntry));
    switch (c.cfg_type)
    {
    case Config_Type_UInt:
        t_value = String(*c.cfg_val.as_uint);
        s.replace("{t}", F("number"));
        break;
    case Config_Type_Time:
        t_value = String((*c.cfg_val.as_uint) / 1000);
        s.replace("{t}", F("number"));
        break;
    default:
        t_value = c.cfg_val.as_str;
        t_value.replace("'", "&#39;");
        if (c.cfg_type == Config_Type_Password)
        {
            s.replace("{t}", F("password"));
        }
        else
        {
            s.replace("{t}", F("text"));
        }
    }
    s.replace("{i}", info);
    s.replace("{n}", String(c.cfg_key));
    s.replace("{v}", t_value);
    s.replace("{l}", String(length));
    page_content += s;
}

static String form_checkbox(const ConfigShapeId cfgid, const String &info, const bool linebreak)
{
    RESERVE_STRING(s, MED_STR);
    s = F("<label for='{n}'>"
          "<input type='checkbox' name='{n}' value='1' id='{n}' {c}/>"
          "<input type='hidden' name='{n}' value='0'/>"
          "{i}</label><br/>");
    if (*configShape[cfgid].cfg_val.as_bool)
    {
        s.replace("{c}", F(" checked='checked'"));
    }
    else
    {
        s.replace("{c}", emptyString);
    };
    s.replace("{i}", info);
    s.replace("{n}", String(configShape[cfgid].cfg_key));
    if (!linebreak)
    {
        s.replace("<br/>", emptyString);
    }
    return s;
}

static String form_submit(const String &value)
{
    String s = F("<tr>"
                 "<td>&nbsp;</td>"
                 "<td>"
                 "<input type='submit' name='submit' value='{v}' />"
                 "</td>"
                 "</tr>");
    s.replace("{v}", value);
    return s;
}

static String form_select_lang()
{
    String s_select = F(" selected='selected'");
    String s = F("<tr>"
                 "<td>" INTL_LANGUAGE ":&nbsp;</td>"
                 "<td>"
                 "<select id='current_lang' name='current_lang'>"
                 "<option value='BG'>Bulgarian (BG)</option>"
                 "<option value='CZ'>Český (CZ)</option>"
                 "<option value='DE'>Deutsch (DE)</option>"
                 "<option value='DK'>Dansk (DK)</option>"
                 "<option value='EN'>English (EN)</option>"
                 "<option value='ES'>Español (ES)</option>"
                 "<option value='FR'>Français (FR)</option>"
                 "<option value='IT'>Italiano (IT)</option>"
                 "<option value='LU'>Lëtzebuergesch (LU)</option>"
                 "<option value='NL'>Nederlands (NL)</option>"
                 "<option value='PL'>Polski (PL)</option>"
                 "<option value='PT'>Português (PT)</option>"
                 "<option value='RS'>Srpski (RS)</option>"
                 "<option value='RU'>Русский (RU)</option>"
                 "<option value='SE'>Svenska (SE)</option>"
                 "<option value='TR'>Türkçe (TR)</option>"
                 "<option value='UA'>український (UA)</option>"
                 "</select>"
                 "</td>"
                 "</tr>");

    s.replace("'" + String(cfg::current_lang) + "'>", "'" + String(cfg::current_lang) + "'" + s_select + ">");
    return s;
}

static String tmpl(const __FlashStringHelper *patt, const String &value)
{
    String s = patt;
    s.replace("{v}", value);
    return s;
}

static void add_line_value(String &s, const __FlashStringHelper *name, const String &value)
{
    s += F("<br/>");
    s += name;
    s += ": ";
    s += value;
}

static void add_line_value_bool(String &s, const __FlashStringHelper *name, const bool value)
{
    add_line_value(s, name, String(value));
}

static void add_line_value_bool(String &s, const __FlashStringHelper *patt, const __FlashStringHelper *name, const bool value)
{
    s += F("<br/>");
    s += tmpl(patt, name);
    s += ": ";
    s += String(value);
}

static void add_table_row_from_value(String &page_content, const __FlashStringHelper *sensor, const __FlashStringHelper *param, const String &value, const String &unit)
{
    RESERVE_STRING(s, MED_STR);
    s = F("<tr><td>{s}</td><td>{p}</td><td class='r'>{v}&nbsp;{u}</td></tr>");
    s.replace("{s}", sensor);
    s.replace("{p}", param);
    s.replace("{v}", value);
    s.replace("{u}", unit);
    page_content += s;
}

static void add_table_row_from_value(String &page_content, const __FlashStringHelper *param, const String &value, const char *unit = nullptr)
{
    RESERVE_STRING(s, MED_STR);
    s = F("<tr><td>{p}</td><td class='r'>{v}&nbsp;{u}</td></tr>");
    s.replace("{p}", param);
    s.replace("{v}", value);
    s.replace("{u}", String(unit));
    page_content += s;
}

static int32_t calcWiFiSignalQuality(int32_t rssi)
{
    // Treat 0 or positive values as 0%
    if (rssi >= 0 || rssi < -100)
    {
        rssi = -100;
    }
    if (rssi > -50)
    {
        rssi = -50;
    }
    return (rssi + 100) * 2;
}

static String wlan_ssid_to_table_row(const String &ssid, const String &encryption, int32_t rssi)
{
    String s = F("<tr>"
                 "<td>"
                 "<a href='#wlanpwd' onclick='setSSID(this)' class='wifi'>{n}</a>&nbsp;{e}"
                 "</td>"
                 "<td style='width:80%;vertical-align:middle;'>"
                 "{v}%"
                 "</td>"
                 "</tr>");
    s.replace("{n}", ssid);
    s.replace("{e}", encryption);
    s.replace("{v}", String(calcWiFiSignalQuality(rssi)));
    return s;
}

static void add_warning_first_cycle(String &page_content)
{
    String s = FPSTR(INTL_TIME_TO_FIRST_MEASUREMENT);
    unsigned int time_to_first = cfg::sending_intervall_ms - msSince(starttime);
    if (time_to_first > cfg::sending_intervall_ms)
    {
        time_to_first = 0;
    }
    s.replace("{v}", String(((time_to_first + 500) / 1000)));
    page_content += s;
}

static void add_age_last_values(String &s)
{
    s += "<b>";
    unsigned int time_since_last = msSince(starttime);
    if (time_since_last > cfg::sending_intervall_ms)
    {
        time_since_last = 0;
    }
    s += String((time_since_last + 500) / 1000);
    s += FPSTR(INTL_TIME_SINCE_LAST_MEASUREMENT);
    s += FPSTR(WEB_B_BR_BR);
}

static String add_sensor_type(const String &sensor_text)
{
    RESERVE_STRING(s, SMALL_STR);
    s = sensor_text;
    s.replace("{pm}", FPSTR(INTL_PARTICULATE_MATTER));
    s.replace("{t}", FPSTR(INTL_TEMPERATURE));
    s.replace("{h}", FPSTR(INTL_HUMIDITY));
    s.replace("{p}", FPSTR(INTL_PRESSURE));
    s.replace("{l_a}", FPSTR(INTL_LEQ_A));
    return s;
}

#endif