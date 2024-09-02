#ifndef BMP085_H
#define BMP085_H

#include <Adafruit_BMP085.h>

/*****************************************************************
 * BMP declaration                                               *
 *****************************************************************/
Adafruit_BMP085 bmp;

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

#endif