#ifndef SPS30_H
#define SPS30_H

#include "./sps30_i2c.h"

/*****************************************************************
   Init SPS30 PM Sensor
 *****************************************************************/
static void initSPS30()
{
    char serial[SPS_MAX_SERIAL_LEN];
    debug_out(F("Trying SPS30 sensor on 0x69H "), DEBUG_MIN_INFO);
    sps30_reset();
    delay(200);
    if (sps30_get_serial(serial) != 0)
    {
        debug_outln_info(FPSTR(DBG_TXT_NOT_FOUND));

        debug_outln_info(F("Check SPS30 wiring"));
        sps30_init_failed = true;
        return;
    }
    debug_outln_info(F(" ... found, Serial-No.: "), String(serial));
    if (sps30_set_fan_auto_cleaning_interval(SPS30_AUTO_CLEANING_INTERVAL) != 0)
    {
        debug_outln_error(F("setting of Auto Cleaning Intervall SPS30 failed!"));
        sps30_init_failed = true;
        return;
    }
    delay(100);
    if (sps30_start_measurement() != 0)
    {
        debug_outln_error(F("SPS30 error starting measurement"));
        sps30_init_failed = true;
        return;
    }
}
/*****************************************************************
   read SPS30 PM sensor values
 *****************************************************************/
static void fetchSensorSPS30(String &s)
{
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_SPS30));

    last_value_SPS30_P0 = value_SPS30_P0 / SPS30_measurement_count;
    last_value_SPS30_P2 = value_SPS30_P2 / SPS30_measurement_count;
    last_value_SPS30_P4 = value_SPS30_P4 / SPS30_measurement_count;
    last_value_SPS30_P1 = value_SPS30_P1 / SPS30_measurement_count;
    last_value_SPS30_N05 = value_SPS30_N05 / SPS30_measurement_count;
    last_value_SPS30_N1 = value_SPS30_N1 / SPS30_measurement_count;
    last_value_SPS30_N25 = value_SPS30_N25 / SPS30_measurement_count;
    last_value_SPS30_N4 = value_SPS30_N4 / SPS30_measurement_count;
    last_value_SPS30_N10 = value_SPS30_N10 / SPS30_measurement_count;
    last_value_SPS30_TS = value_SPS30_TS / SPS30_measurement_count;

    add_Value2Json(s, F("SPS30_P0"), F("PM1.0: "), last_value_SPS30_P0);
    add_Value2Json(s, F("SPS30_P2"), F("PM2.5: "), last_value_SPS30_P2);
    add_Value2Json(s, F("SPS30_P4"), F("PM4.0: "), last_value_SPS30_P4);
    add_Value2Json(s, F("SPS30_P1"), F("PM 10: "), last_value_SPS30_P1);
    add_Value2Json(s, F("SPS30_N05"), F("NC0.5: "), last_value_SPS30_N05);
    add_Value2Json(s, F("SPS30_N1"), F("NC1.0: "), last_value_SPS30_N1);
    add_Value2Json(s, F("SPS30_N25"), F("NC2.5: "), last_value_SPS30_N25);
    add_Value2Json(s, F("SPS30_N4"), F("NC4.0: "), last_value_SPS30_N4);
    add_Value2Json(s, F("SPS30_N10"), F("NC10:  "), last_value_SPS30_N10);
    add_Value2Json(s, F("SPS30_TS"), F("TPS:   "), last_value_SPS30_TS);

    debug_outln_info(F("SPS30 read counter: "), String(SPS30_read_counter));
    debug_outln_info(F("SPS30 read error counter: "), String(SPS30_read_error_counter));

    SPS30_measurement_count = 0;
    SPS30_read_counter = 0;
    SPS30_read_error_counter = 0;
    value_SPS30_P0 = 0.0;
    value_SPS30_P2 = 0.0;
    value_SPS30_P4 = 0.0;
    value_SPS30_P1 = 0.0;
    value_SPS30_N05 = 0.0;
    value_SPS30_N1 = 0.0;
    value_SPS30_N25 = 0.0;
    value_SPS30_N4 = 0.0;
    value_SPS30_N10 = 0.0;
    value_SPS30_TS = 0.0;

    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_SPS30));
}

#endif