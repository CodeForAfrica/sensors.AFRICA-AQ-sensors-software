#ifndef BMX280_H
#define BMX280_H

#include "./bmx280_i2c.h"

/*****************************************************************
 * BMP/BME280 declaration                                        *
 *****************************************************************/
BMX280 bmx280;

/*****************************************************************
 * read BMP280/BME280 sensor values                              *
 *****************************************************************/
static void fetchSensorBMX280(String &s)
{
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_BMX280));

    bmx280.takeForcedMeasurement();
    const auto t = bmx280.readTemperature();
    const auto p = bmx280.readPressure();
    const auto h = bmx280.readHumidity();
    if (isnan(t) || isnan(p))
    {
        last_value_BMX280_T = -128.0;
        last_value_BMX280_P = -1.0;
        last_value_BME280_H = -1.0;
        debug_outln_error(F("BMP/BME280 read failed"));
    }
    else
    {
        last_value_BMX280_T = t;
        last_value_BMX280_P = p;
        if (bmx280.sensorID() == BME280_SENSOR_ID)
        {
            add_Value2Json(s, F("BME280_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMX280_T);
            add_Value2Json(s, F("BME280_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMX280_P);
            last_value_BME280_H = h;
            add_Value2Json(s, F("BME280_humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_BME280_H);
        }
        else
        {
            add_Value2Json(s, F("BMP280_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMX280_P);
            add_Value2Json(s, F("BMP280_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMX280_T);
        }
    }
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_BMX280));
}

#endif