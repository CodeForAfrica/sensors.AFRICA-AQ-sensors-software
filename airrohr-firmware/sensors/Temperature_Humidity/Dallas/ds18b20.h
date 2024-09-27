#ifndef DS180B20
#define DS180B20

#include <DallasTemperature.h>

/*****************************************************************
 * DS18B20 declaration                                            *
 *****************************************************************/
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds18b20(&oneWire);

/*****************************************************************
 * read DS18B20 sensor values                                    *
 *****************************************************************/
static void fetchSensorDS18B20(String &s)
{
    float t;
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_DS18B20));

    // it's very unlikely (-127: impossible) to get these temperatures in reality. Most times this means that the sensor is currently faulty
    // try 5 times to read the sensor, otherwise fail
    const int MAX_ATTEMPTS = 5;
    int count = 0;
    do
    {
        ds18b20.requestTemperatures();
        // for now, we want to read only the first sensor
        t = ds18b20.getTempCByIndex(0);
        ++count;
        debug_outln_info(F("DS18B20 trying...."));
    } while (count < MAX_ATTEMPTS && (isnan(t) || t >= 85.0f || t <= (-127.0f)));

    if (count == MAX_ATTEMPTS)
    {
        last_value_DS18B20_T = -128.0;
        debug_outln_error(F("DS18B20 read failed"));
    }
    else
    {
        last_value_DS18B20_T = t;
        add_Value2Json(s, F("DS18B20_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_DS18B20_T);
    }
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_DS18B20));
}

#endif