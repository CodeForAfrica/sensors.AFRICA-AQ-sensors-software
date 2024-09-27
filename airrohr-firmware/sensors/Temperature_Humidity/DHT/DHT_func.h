#include "./DHT.h"

/*****************************************************************
 * DHT declaration                                               *
 *****************************************************************/
DHT dht(ONEWIRE_PIN, DHT_TYPE);

/*****************************************************************
 * read DHT22 sensor values                                      *
 *****************************************************************/
static void fetchSensorDHT(String &s)
{
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_DHT22));

    // Check if valid number if non NaN (not a number) will be send.
    last_value_DHT_T = -128;
    last_value_DHT_H = -1;

    int count = 0;
    const int MAX_ATTEMPTS = 5;
    while ((count++ < MAX_ATTEMPTS))
    {
        auto t = dht.readTemperature();
        auto h = dht.readHumidity();
        if (isnan(t) || isnan(h))
        {
            delay(100);
            t = dht.readTemperature(false);
            h = dht.readHumidity();
        }
        if (isnan(t) || isnan(h))
        {
            debug_outln_error(F("DHT11/DHT22 read failed"));
        }
        else
        {
            last_value_DHT_T = t;
            last_value_DHT_H = h;
            add_Value2Json(s, F("temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_DHT_T);
            add_Value2Json(s, F("humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_DHT_H);
            switch_status_LEDs_on(DHT_LED, HIGH);
            delay(5000);
            switch_status_LEDs_off(DHT_LED, LOW);
            break;
        }
    }
    debug_outln_info(FPSTR(DBG_TXT_SEP));

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_DHT22));
}
