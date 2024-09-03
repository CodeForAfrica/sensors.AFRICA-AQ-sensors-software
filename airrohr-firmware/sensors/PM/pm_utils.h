enum class PmSensorCmd
{
    Start,
    Stop,
    ContinuousMode
};

/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version    *
 *****************************************************************/

static bool SDS_checksum_valid(const uint8_t (&data)[8])
{
    uint8_t checksum_is = 0;
    for (unsigned i = 0; i < 6; ++i)
    {
        checksum_is += data[i];
    }
    return (data[7] == 0xAB && checksum_is == data[6]);
}

static void SDS_rawcmd(const uint8_t cmd_head1, const uint8_t cmd_head2, const uint8_t cmd_head3)
{
    constexpr uint8_t cmd_len = 19;

    uint8_t buf[cmd_len];
    buf[0] = 0xAA;
    buf[1] = 0xB4;
    buf[2] = cmd_head1;
    buf[3] = cmd_head2;
    buf[4] = cmd_head3;
    for (unsigned i = 5; i < 15; ++i)
    {
        buf[i] = 0x00;
    }
    buf[15] = 0xFF;
    buf[16] = 0xFF;
    buf[17] = cmd_head1 + cmd_head2 + cmd_head3 - 2;
    buf[18] = 0xAB;
    serialSDS.write(buf, cmd_len);
}

static bool SDS_cmd(PmSensorCmd cmd)
{
    switch (cmd)
    {
    case PmSensorCmd::Start:
        SDS_rawcmd(0x06, 0x01, 0x01);
        break;
    case PmSensorCmd::Stop:
        SDS_rawcmd(0x06, 0x01, 0x00);
        break;
    case PmSensorCmd::ContinuousMode:
        // TODO: Check mode first before (re-)setting it
        SDS_rawcmd(0x08, 0x01, 0x00);
        SDS_rawcmd(0x02, 0x01, 0x00);
        break;
    }

    return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * read SDS011 sensor serial and firmware date                   *
 *****************************************************************/
static String SDS_version_date()
{

    if (cfg::sds_read && !last_value_SDS_version.length())
    {
        debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(DBG_TXT_SDS011_VERSION_DATE));
        is_SDS_running = SDS_cmd(PmSensorCmd::Start);
        delay(250);
        serialSDS.perform_work();
        serialSDS.flush();
        // Query Version/Date
        SDS_rawcmd(0x07, 0x00, 0x00);
        delay(400);
        const constexpr uint8_t header_cmd_response[2] = {0xAA, 0xC5};
        while (serialSDS.find(header_cmd_response, sizeof(header_cmd_response)))
        {
            uint8_t data[8];
            yield_for_serial_buffer(sizeof(data));
            unsigned r = serialSDS.readBytes(data, sizeof(data));
            if (r == sizeof(data) && data[0] == 0x07 && SDS_checksum_valid(data))
            {
                char tmp[20];
                snprintf_P(tmp, sizeof(tmp), PSTR("%02d-%02d-%02d(%02x%02x)"),
                           data[1], data[2], data[3], data[4], data[5]);
                last_value_SDS_version = tmp;
                break;
            }
        }
        debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(DBG_TXT_SDS011_VERSION_DATE));
    }

    return last_value_SDS_version;
}

/*****************************************************************
 * read SDS011 sensor values                                     *
 *****************************************************************/
static void fetchSensorSDS(String &s)
{

    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_SDS011));

    if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS) &&
        msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
    {
        if (is_SDS_running)
        {
            is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
        }
    }
    else
    {
        if (!is_SDS_running)
        {
            is_SDS_running = SDS_cmd(PmSensorCmd::Start);
        }

        const uint8_t constexpr header_measurement[2] = {0xAA, 0xC0};

        while (serialSDS.available() >= 10 &&
               serialSDS.find(header_measurement, sizeof(header_measurement)))
        {
            uint8_t data[8];
            yield_for_serial_buffer(sizeof(data));
            unsigned r = serialSDS.readBytes(data, sizeof(data));
            if (r == sizeof(data) && SDS_checksum_valid(data))
            {
                uint32_t pm25_serial = data[0] | (data[1] << 8);
                uint32_t pm10_serial = data[2] | (data[3] << 8);

                if (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS))
                {
                    sds_pm10_sum += pm10_serial;
                    sds_pm25_sum += pm25_serial;
                    if (sds_pm10_min > pm10_serial)
                    {
                        sds_pm10_min = pm10_serial;
                    }
                    if (sds_pm10_max < pm10_serial)
                    {
                        sds_pm10_max = pm10_serial;
                    }
                    if (sds_pm25_min > pm25_serial)
                    {
                        sds_pm25_min = pm25_serial;
                    }
                    if (sds_pm25_max < pm25_serial)
                    {
                        sds_pm25_max = pm25_serial;
                    }
                    debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial / 10.0f));
                    debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial / 10.0f));
                    sds_val_count++;
                }
            }
        }
    }
    if (send_now)
    {
        last_value_SDS_P1 = -1;
        last_value_SDS_P2 = -1;
        if (sds_val_count > 2)
        {
            sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
            sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
            sds_val_count = sds_val_count - 2;
        }
        if (sds_val_count > 0)
        {
            last_value_SDS_P1 = float(sds_pm10_sum) / (sds_val_count * 10.0f);
            last_value_SDS_P2 = float(sds_pm25_sum) / (sds_val_count * 10.0f);
            add_Value2Json(s, F("SDS_P1"), F("PM10:  "), last_value_SDS_P1);
            add_Value2Json(s, F("SDS_P2"), F("PM2.5: "), last_value_SDS_P2);
            debug_outln_info(FPSTR(DBG_TXT_SEP));
            if (sds_val_count < 3)
            {
                SDS_error_count++;
            }
        }
        else
        {
            SDS_error_count++;
        }
        sds_pm10_sum = 0;
        sds_pm25_sum = 0;
        sds_val_count = 0;
        sds_pm10_max = 0;
        sds_pm10_min = 20000;
        sds_pm25_max = 0;
        sds_pm25_min = 20000;
        if ((cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
        {

            if (is_SDS_running)
            {
                is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
            }
        }
    }

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_SDS011));
}

/*****************************************************************
 * send Plantower PMS sensor command start, stop, cont. mode     *
 *****************************************************************/
static bool PMS_cmd(PmSensorCmd cmd)
{
    static constexpr uint8_t start_cmd[] PROGMEM = {
        0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
    static constexpr uint8_t stop_cmd[] PROGMEM = {
        0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
    static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
        0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
    constexpr uint8_t cmd_len = array_num_elements(start_cmd);

    uint8_t buf[cmd_len];
    switch (cmd)
    {
    case PmSensorCmd::Start:
        memcpy_P(buf, start_cmd, cmd_len);
        break;
    case PmSensorCmd::Stop:
        memcpy_P(buf, stop_cmd, cmd_len);
        break;
    case PmSensorCmd::ContinuousMode:
        memcpy_P(buf, continuous_mode_cmd, cmd_len);
        break;
    }
    serialSDS.write(buf, cmd_len);
    return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * read Plantronic PM sensor sensor values                       *
 *****************************************************************/
static void fetchSensorPMS(String &s)
{
    char buffer;
    int value;
    int len = 0;
    int pm1_serial = 0;
    int pm10_serial = 0;
    int pm25_serial = 0;
    int checksum_is = 0;
    int checksum_should = 0;
    bool checksum_ok = false;
    int frame_len = 24; // min. frame length

    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_PMSx003));
    if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
    {
        if (is_PMS_running)
        {
            is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
        }
    }
    else
    {
        if (!is_PMS_running)
        {
            is_PMS_running = PMS_cmd(PmSensorCmd::Start);
        }

        while (serialSDS.available() > 0)
        {
            buffer = serialSDS.read();
            debug_outln(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO);
            //			"aa" = 170, "ab" = 171, "c0" = 192
            value = int(buffer);
            switch (len)
            {
            case 0:
                if (value != 66)
                {
                    len = -1;
                };
                break;
            case 1:
                if (value != 77)
                {
                    len = -1;
                };
                break;
            case 2:
                checksum_is = value;
                break;
            case 3:
                frame_len = value + 4;
                break;
            case 10:
                pm1_serial += (value << 8);
                break;
            case 11:
                pm1_serial += value;
                break;
            case 12:
                pm25_serial = (value << 8);
                break;
            case 13:
                pm25_serial += value;
                break;
            case 14:
                pm10_serial = (value << 8);
                break;
            case 15:
                pm10_serial += value;
                break;
            case 22:
                if (frame_len == 24)
                {
                    checksum_should = (value << 8);
                };
                break;
            case 23:
                if (frame_len == 24)
                {
                    checksum_should += value;
                };
                break;
            case 30:
                checksum_should = (value << 8);
                break;
            case 31:
                checksum_should += value;
                break;
            }
            if ((len > 2) && (len < (frame_len - 2)))
            {
                checksum_is += value;
            }
            len++;
            if (len == frame_len)
            {
                debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_IS), String(checksum_is + 143));
                debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_SHOULD), String(checksum_should));
                if (checksum_should == (checksum_is + 143))
                {
                    checksum_ok = true;
                }
                else
                {
                    len = 0;
                };
                if (checksum_ok && (msSince(starttime) > (cfg::sending_intervall_ms - READINGTIME_SDS_MS)))
                {
                    if ((!isnan(pm1_serial)) && (!isnan(pm10_serial)) && (!isnan(pm25_serial)))
                    {
                        pms_pm1_sum += pm1_serial;
                        pms_pm10_sum += pm10_serial;
                        pms_pm25_sum += pm25_serial;
                        if (pms_pm1_min > pm1_serial)
                        {
                            pms_pm1_min = pm1_serial;
                        }
                        if (pms_pm1_max < pm1_serial)
                        {
                            pms_pm1_max = pm1_serial;
                        }
                        if (pms_pm25_min > pm25_serial)
                        {
                            pms_pm25_min = pm25_serial;
                        }
                        if (pms_pm25_max < pm25_serial)
                        {
                            pms_pm25_max = pm25_serial;
                        }
                        if (pms_pm10_min > pm10_serial)
                        {
                            pms_pm10_min = pm10_serial;
                        }
                        if (pms_pm10_max < pm10_serial)
                        {
                            pms_pm10_max = pm10_serial;
                        }
                        debug_outln_verbose(F("PM1 (sec.): "), String(pm1_serial));
                        debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial));
                        debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial));
                        pms_val_count++;
                    }
                    len = 0;
                    checksum_ok = false;
                    pm1_serial = 0;
                    pm10_serial = 0;
                    pm25_serial = 0;
                    checksum_is = 0;
                }
            }
            yield();
        }
    }
    if (send_now)
    {
        last_value_PMS_P0 = -1;
        last_value_PMS_P1 = -1;
        last_value_PMS_P2 = -1;
        if (pms_val_count > 2)
        {
            pms_pm1_sum = pms_pm1_sum - pms_pm1_min - pms_pm1_max;
            pms_pm10_sum = pms_pm10_sum - pms_pm10_min - pms_pm10_max;
            pms_pm25_sum = pms_pm25_sum - pms_pm25_min - pms_pm25_max;
            pms_val_count = pms_val_count - 2;
        }
        if (pms_val_count > 0)
        {
            last_value_PMS_P0 = float(pms_pm1_sum) / float(pms_val_count);
            last_value_PMS_P1 = float(pms_pm10_sum) / float(pms_val_count);
            last_value_PMS_P2 = float(pms_pm25_sum) / float(pms_val_count);
            add_Value2Json(s, F("PMS_P0"), F("PM1:   "), last_value_PMS_P0);
            add_Value2Json(s, F("PMS_P1"), F("PM10:  "), last_value_PMS_P1);
            add_Value2Json(s, F("PMS_P2"), F("PM2.5: "), last_value_PMS_P2);
            debug_outln_info(FPSTR(DBG_TXT_SEP));
        }
        pms_pm1_sum = 0;
        pms_pm10_sum = 0;
        pms_pm25_sum = 0;
        pms_val_count = 0;
        pms_pm1_max = 0;
        pms_pm1_min = 20000;
        pms_pm10_max = 0;
        pms_pm10_min = 20000;
        pms_pm25_max = 0;
        pms_pm25_min = 20000;
        if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))
        {
            is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
        }
        switch_status_LEDs_on(PMS_LED, HIGH);
        delay(5000);
        switch_status_LEDs_off(PMS_LED, LOW);
    }

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_PMSx003));
}

/*****************************************************************
 * send Honeywell PMS sensor command start, stop, cont. mode     *
 *****************************************************************/
static bool HPM_cmd(PmSensorCmd cmd)
{
    static constexpr uint8_t start_cmd[] PROGMEM = {
        0x68, 0x01, 0x01, 0x96};
    static constexpr uint8_t stop_cmd[] PROGMEM = {
        0x68, 0x01, 0x02, 0x95};
    static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
        0x68, 0x01, 0x40, 0x57};
    constexpr uint8_t cmd_len = array_num_elements(start_cmd);

    uint8_t buf[cmd_len];
    switch (cmd)
    {
    case PmSensorCmd::Start:
        memcpy_P(buf, start_cmd, cmd_len);
        break;
    case PmSensorCmd::Stop:
        memcpy_P(buf, stop_cmd, cmd_len);
        break;
    case PmSensorCmd::ContinuousMode:
        memcpy_P(buf, continuous_mode_cmd, cmd_len);
        break;
    }
    serialSDS.write(buf, cmd_len);
    return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * read Honeywell PM sensor sensor values                        *
 *****************************************************************/
static void fetchSensorHPM(String &s)
{
    char buffer;
    int value;
    int len = 0;
    int pm10_serial = 0;
    int pm25_serial = 0;
    int checksum_is = 0;
    int checksum_should = 0;
    bool checksum_ok = false;

    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_HPM));
    if (msSince(starttime) < (cfg::sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
    {
        if (is_HPM_running)
        {
            is_HPM_running = HPM_cmd(PmSensorCmd::Stop);
        }
    }
    else
    {
        if (!is_HPM_running)
        {
            is_HPM_running = HPM_cmd(PmSensorCmd::Start);
        }

        while (serialSDS.available() > 0)
        {
            buffer = serialSDS.read();
            debug_outln(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .", DEBUG_MAX_INFO);
            //			"aa" = 170, "ab" = 171, "c0" = 192
            value = int(buffer);
            switch (len)
            {
            case 0:
                if (value != 66)
                {
                    len = -1;
                };
                break;
            case 1:
                if (value != 77)
                {
                    len = -1;
                };
                break;
            case 2:
                checksum_is = value;
                break;
            case 6:
                pm25_serial += (value << 8);
                break;
            case 7:
                pm25_serial += value;
                break;
            case 8:
                pm10_serial = (value << 8);
                break;
            case 9:
                pm10_serial += value;
                break;
            case 30:
                checksum_should = (value << 8);
                break;
            case 31:
                checksum_should += value;
                break;
            }
            if (len > 2 && len < 30)
            {
                checksum_is += value;
            }
            len++;
            if (len == 32)
            {
                debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_IS), String(checksum_is + 143));
                debug_outln_verbose(FPSTR(DBG_TXT_CHECKSUM_SHOULD), String(checksum_should));
                if (checksum_should == (checksum_is + 143))
                {
                    checksum_ok = true;
                }
                else
                {
                    len = 0;
                };
                if (checksum_ok && (long(msSince(starttime)) > (long(cfg::sending_intervall_ms) - long(READINGTIME_SDS_MS))))
                {
                    if ((!isnan(pm10_serial)) && (!isnan(pm25_serial)))
                    {
                        hpm_pm10_sum += pm10_serial;
                        hpm_pm25_sum += pm25_serial;
                        if (hpm_pm10_min > pm10_serial)
                        {
                            hpm_pm10_min = pm10_serial;
                        }
                        if (hpm_pm10_max < pm10_serial)
                        {
                            hpm_pm10_max = pm10_serial;
                        }
                        if (hpm_pm25_min > pm25_serial)
                        {
                            hpm_pm25_min = pm25_serial;
                        }
                        if (hpm_pm25_max < pm25_serial)
                        {
                            hpm_pm25_max = pm25_serial;
                        }
                        debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial));
                        debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial));
                        hpm_val_count++;
                    }
                    len = 0;
                    checksum_ok = false;
                    pm10_serial = 0;
                    pm25_serial = 0;
                    checksum_is = 0;
                }
            }
            yield();
        }
    }
    if (send_now)
    {
        last_value_HPM_P1 = -1.0f;
        last_value_HPM_P2 = -1.0f;
        if (hpm_val_count > 2)
        {
            hpm_pm10_sum = hpm_pm10_sum - hpm_pm10_min - hpm_pm10_max;
            hpm_pm25_sum = hpm_pm25_sum - hpm_pm25_min - hpm_pm25_max;
            hpm_val_count = hpm_val_count - 2;
        }
        if (hpm_val_count > 0)
        {
            last_value_HPM_P1 = float(hpm_pm10_sum) / float(hpm_val_count);
            last_value_HPM_P2 = float(hpm_pm25_sum) / float(hpm_val_count);
            add_Value2Json(s, F("HPM_P1"), F("PM2.5: "), last_value_HPM_P1);
            add_Value2Json(s, F("HPM_P2"), F("PM10:  "), last_value_HPM_P2);
            debug_outln_info(FPSTR(DBG_TXT_SEP));
        }
        hpm_pm10_sum = 0;
        hpm_pm25_sum = 0;
        hpm_val_count = 0;
        hpm_pm10_max = 0;
        hpm_pm10_min = 20000;
        hpm_pm25_max = 0;
        hpm_pm25_min = 20000;
        if (cfg::sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))
        {
            is_HPM_running = HPM_cmd(PmSensorCmd::Stop);
        }
    }

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_HPM));
}

/*****************************************************************
 * read PPD42NS sensor values                                    *
 *****************************************************************/
static void fetchSensorPPD(String &s)
{
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_PPD42NS));

    if (msSince(starttime) <= SAMPLETIME_MS)
    {

        // Read pins connected to ppd42ns
        boolean valP1 = digitalRead(PPD_PIN_PM1);
        boolean valP2 = digitalRead(PPD_PIN_PM2);

        if (valP1 == LOW && trigP1 == false)
        {
            trigP1 = true;
            trigOnP1 = act_micro;
        }

        if (valP1 == HIGH && trigP1 == true)
        {
            lowpulseoccupancyP1 += act_micro - trigOnP1;
            trigP1 = false;
        }

        if (valP2 == LOW && trigP2 == false)
        {
            trigP2 = true;
            trigOnP2 = act_micro;
        }

        if (valP2 == HIGH && trigP2 == true)
        {
            unsigned long durationP2 = act_micro - trigOnP2;
            lowpulseoccupancyP2 += durationP2;
            trigP2 = false;
        }
    }
    // Checking if it is time to sample
    if (send_now)
    {
        auto calcConcentration = [](const float ratio)
        {
            /* spec sheet curve*/
            return (1.1f * ratio * ratio * ratio - 3.8f * ratio * ratio + 520.0f * ratio + 0.62f);
        };

        last_value_PPD_P1 = -1;
        last_value_PPD_P2 = -1;
        float ratio = lowpulseoccupancyP1 / (SAMPLETIME_MS * 10.0f);
        float concentration = calcConcentration(ratio);

        // json for push to api / P1
        last_value_PPD_P1 = concentration;
        add_Value2Json(s, F("durP1"), F("LPO P10    : "), lowpulseoccupancyP1);
        add_Value2Json(s, F("ratioP1"), F("Ratio PM10%: "), ratio);
        add_Value2Json(s, F("P1"), F("PM10 Count : "), last_value_PPD_P1);

        ratio = lowpulseoccupancyP2 / (SAMPLETIME_MS * 10.0f);
        concentration = calcConcentration(ratio);

        // json for push to api / P2
        last_value_PPD_P2 = concentration;
        add_Value2Json(s, F("durP2"), F("LPO PM25   : "), lowpulseoccupancyP2);
        add_Value2Json(s, F("ratioP2"), F("Ratio PM25%: "), ratio);
        add_Value2Json(s, F("P2"), F("PM25 Count : "), last_value_PPD_P2);

        debug_outln_info(FPSTR(DBG_TXT_SEP));
    }

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_PPD42NS));
}
