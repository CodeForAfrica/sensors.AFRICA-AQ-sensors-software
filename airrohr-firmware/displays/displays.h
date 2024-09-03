#ifndef DISPLAYS_H
#define DISPLAYS_H

#include "./oledfont.h" // avoids including the default Arial font, needs to be included before SSD1306.h
#include <SSD1306.h>
#include <SH1106.h>
#include <LiquidCrystal_I2C.h>

/*****************************************************************
 * Display definitions                                           *
 *****************************************************************/
SSD1306 display(0x3c, I2C_PIN_SDA, I2C_PIN_SCL);
SH1106 display_sh1106(0x3c, I2C_PIN_SDA, I2C_PIN_SCL);
LiquidCrystal_I2C *lcd_1602 = nullptr;
LiquidCrystal_I2C *lcd_2004 = nullptr;

/*****************************************************************
 * Init OLED display                                             *
 *****************************************************************/
static void init_display()
{
    display.init();
    display_sh1106.init();
    if (cfg::has_flipped_display)
    {
        display.flipScreenVertically();
        display_sh1106.flipScreenVertically();
    }
}

/*****************************************************************
 * Init LCD display                                              *
 *****************************************************************/
static void init_lcd()
{
    if (cfg::has_lcd1602)
    {
        lcd_1602 = new LiquidCrystal_I2C(0x3f, 16, 2);
    }
    else if (cfg::has_lcd1602_27)
    {
        lcd_1602 = new LiquidCrystal_I2C(0x27, 16, 2);
    }
    if (lcd_1602)
    {
        lcd_1602->init();
        lcd_1602->backlight();
    }

    if (cfg::has_lcd2004)
    {
        lcd_2004 = new LiquidCrystal_I2C(0x3f, 20, 4);
    }
    else if (cfg::has_lcd2004_27)
    {
        lcd_2004 = new LiquidCrystal_I2C(0x27, 20, 4);
    }
    if (lcd_2004)
    {
        lcd_2004->init();
        lcd_2004->backlight();
    }
}

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

static String displayGenerateFooter(unsigned int screen_count)
{
    String display_footer;
    for (unsigned int i = 0; i < screen_count; ++i)
    {
        display_footer += (i != (next_display_count % screen_count)) ? " . " : " o ";
    }
    return display_footer;
}

#endif
