static void logEnabledAPIs()
{
    debug_outln_info(F("Send to :"));
    if (cfg::send2dusti)
    {
        debug_outln_info(F("sensor.community"));
    }
    if (cfg::send2cfa)
    {
        debug_outln_info(F("CFA"));
    }

    if (cfg::send2fsapp)
    {
        debug_outln_info(F("Feinstaub-App"));
    }

    if (cfg::send2madavi)
    {
        debug_outln_info(F("Madavi.de"));
    }

    if (cfg::send2csv)
    {
        debug_outln_info(F("Serial as CSV"));
    }

    if (cfg::send2custom)
    {
        debug_outln_info(F("custom API"));
    }

    if (cfg::send2aircms)
    {
        debug_outln_info(F("aircms API"));
    }

    if (cfg::send2influx)
    {
        debug_outln_info(F("custom influx DB"));
    }
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    if (cfg::auto_update)
    {
        debug_outln_info(F("Auto-Update active..."));
    }
}

static void logEnabledDisplays()
{
    if (cfg::has_display || cfg::has_sh1106)
    {
        debug_outln_info(F("Show on OLED..."));
    }
    if (lcd_1602)
    {
        debug_outln_info(F("Show on LCD 1602 ..."));
    }
    if (lcd_2004)
    {
        debug_outln_info(F("Show on LCD 2004 ..."));
    }
}
