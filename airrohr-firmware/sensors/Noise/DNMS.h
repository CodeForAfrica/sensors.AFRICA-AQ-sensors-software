/***************************************
 * Digital Noise Measurement Sensor
 **************************************/

#ifndef DNMS_H
#define DNMS_H

#include "./dnms_i2c.h"
// Function declarations
static void initDNMS();
static float readDNMScorrection();
static void fetchSensorDNMS(String &s);

// Function definition

/*****************************************************************
   Init DNMS - Digital Noise Measurement Sensor
 *****************************************************************/
static void initDNMS()
{
    char dnms_version[DNMS_MAX_VERSION_LEN + 1];

    debug_out(F("Trying DNMS sensor on 0x55H "), DEBUG_MIN_INFO);
    dnms_reset();
    delay(1000);
    if (dnms_read_version(dnms_version) != 0)
    {
        debug_outln_info(FPSTR(DBG_TXT_NOT_FOUND));
        debug_outln_error(F("Check DNMS wiring"));
        dnms_init_failed = true;
    }
    else
    {
        dnms_version[DNMS_MAX_VERSION_LEN] = 0;
        debug_outln_info(FPSTR(DBG_TXT_FOUND), String(": ") + String(dnms_version));
    }
}

/*****************************************************************
  read DNMS values
*****************************************************************/

static float readDNMScorrection()
{
    char *pEnd = nullptr;
    // Avoiding atof() here as this adds a lot (~ 9kb) of code size
    float r = float(strtol(cfg::dnms_correction, &pEnd, 10));
    if (pEnd && pEnd[0] == '.' && pEnd[1] >= '0' && pEnd[1] <= '9')
    {
        r += (r >= 0 ? 1.0 : -1.0) * ((pEnd[1] - '0') / 10.0);
    }
    return r;
}

static void fetchSensorDNMS(String &s)
{
    static bool dnms_error = false;
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_DNMS));
    last_value_dnms_laeq = -1.0;
    last_value_dnms_la_min = -1.0;
    last_value_dnms_la_max = -1.0;

    if (dnms_calculate_leq() != 0)
    {
        // error
        dnms_error = true;
    }
    uint16_t data_ready = 0;
    dnms_error = true;
    for (unsigned i = 0; i < 20; i++)
    {
        delay(2);
        int16_t ret_dnms = dnms_read_data_ready(&data_ready);
        if ((ret_dnms == 0) && (data_ready != 0))
        {
            dnms_error = false;
            break;
        }
    }
    if (!dnms_error)
    {
        struct dnms_measurements dnms_values;
        if (dnms_read_leq(&dnms_values) == 0)
        {
            float dnms_corr_value = readDNMScorrection();
            last_value_dnms_laeq = dnms_values.leq_a + dnms_corr_value;
            last_value_dnms_la_min = dnms_values.leq_a_min + dnms_corr_value;
            last_value_dnms_la_max = dnms_values.leq_a_max + dnms_corr_value;
        }
        else
        {
            // error
            dnms_error = true;
        }
    }
    if (dnms_error)
    {
        // es gab einen Fehler
        dnms_reset(); // try to reset dnms
        debug_outln_error(F("DNMS read failed"));
    }
    else
    {
        add_Value2Json(s, F("DNMS_noise_LAeq"), F("LAeq: "), last_value_dnms_laeq);
        add_Value2Json(s, F("DNMS_noise_LA_min"), F("LA_MIN: "), last_value_dnms_la_min);
        add_Value2Json(s, F("DNMS_noise_LA_max"), F("LA_MAX: "), last_value_dnms_la_max);
    }
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_DNMS));
}

#endif