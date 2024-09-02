#ifndef SHT3X_H
#define SHT3X_H

#include <Adafruit_SHT31.h>

/*****************************************************************
 * SHT3x declaration                                             *
 *****************************************************************/
Adafruit_SHT31 sht3x;

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

#endif