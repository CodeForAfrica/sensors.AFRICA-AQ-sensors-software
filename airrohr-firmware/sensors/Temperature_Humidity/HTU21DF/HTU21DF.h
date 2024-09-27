
#include <Adafruit_HTU21DF.h>
/*****************************************************************
 * HTU21D declaration                                            *
 *****************************************************************/
Adafruit_HTU21DF htu21d;

/*****************************************************************
 * read HTU21D sensor values                                     *
 *****************************************************************/
static void fetchSensorHTU21D(String &s)
{
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_HTU21D));

    const auto t = htu21d.readTemperature();
    const auto h = htu21d.readHumidity();
    if (isnan(t) || isnan(h))
    {
        last_value_HTU21D_T = -128.0;
        last_value_HTU21D_H = -1.0;
        debug_outln_error(F("HTU21D read failed"));
    }
    else
    {
        last_value_HTU21D_T = t;
        last_value_HTU21D_H = h;
        add_Value2Json(s, F("HTU21D_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_HTU21D_T);
        add_Value2Json(s, F("HTU21D_humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_HTU21D_H);
    }
    debug_outln_info(FPSTR(DBG_TXT_SEP));

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_HTU21D));
}