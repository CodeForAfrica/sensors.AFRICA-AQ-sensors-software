#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
/*****************************************************************
 * GPS declaration                                               *
 *****************************************************************/
TinyGPSPlus gps;

/*****************************************************************
 * read GPS sensor values                                        *
 *****************************************************************/
static void fetchSensorGPS(String &s)
{
    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), "GPS");

    if (gps.location.isUpdated())
    {
        if (gps.location.isValid())
        {
            last_value_GPS_lat = gps.location.lat();
            last_value_GPS_lon = gps.location.lng();
        }
        else
        {
            last_value_GPS_lat = -200;
            last_value_GPS_lon = -200;
            debug_outln_verbose(F("Lat/Lng INVALID"));
        }
        if (gps.altitude.isValid())
        {
            last_value_GPS_alt = gps.altitude.meters();
            String gps_alt(last_value_GPS_lat);
        }
        else
        {
            last_value_GPS_alt = -1000;
            debug_outln_verbose(F("Altitude INVALID"));
        }
        if (gps.date.isValid())
        {
            char gps_date[16];
            snprintf_P(gps_date, sizeof(gps_date), PSTR("%02d/%02d/%04d"),
                       gps.date.month(), gps.date.day(), gps.date.year());
            last_value_GPS_date = gps_date;
            last_value_GPS_timestamp = gps_date;
        }
        else
        {
            debug_outln_verbose(F("Date INVALID"));
        }
        if (gps.time.isValid())
        {
            char gps_time[20];
            snprintf_P(gps_time, sizeof(gps_time), PSTR("%02d:%02d:%02d.%02d"),
                       gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
            last_value_GPS_time = gps_time;
            last_value_GPS_timestamp += "T";
            last_value_GPS_timestamp += gps_time;
        }
        else
        {
            debug_outln_verbose(F("Time: INVALID"));
        }
    }

    if (send_now)
    {
        debug_outln_info(F("Lat: "), String(last_value_GPS_lat, 6));
        debug_outln_info(F("Lng: "), String(last_value_GPS_lon, 6));
        debug_outln_info(F("Date: "), last_value_GPS_date);
        debug_outln_info(F("Time "), last_value_GPS_time);

        add_Value2Json(s, F("GPS_lat"), String(last_value_GPS_lat, 6));
        add_Value2Json(s, F("GPS_lon"), String(last_value_GPS_lon, 6));
        add_Value2Json(s, F("GPS_height"), F("Altitude: "), last_value_GPS_alt);
        add_Value2Json(s, F("GPS_timestamp"), last_value_GPS_timestamp);
        debug_outln_info(FPSTR(DBG_TXT_SEP));
    }

    if (count_sends > 0 && gps.charsProcessed() < 10)
    {
        debug_outln_error(F("No GPS data received: check wiring"));
        gps_init_failed = true;
    }

    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), "GPS");
}

/*****************************************************************
 * disable unneeded NMEA sentences, TinyGPS++ needs GGA, RMC     *
 *****************************************************************/
static void disable_unneeded_nmea()
{
    serialGPS->println(F("$PUBX,40,GLL,0,0,0,0*5C")); // Geographic position, latitude / longitude
                                                      //	serialGPS->println(F("$PUBX,40,GGA,0,0,0,0*5A"));       // Global Positioning System Fix Data
    serialGPS->println(F("$PUBX,40,GSA,0,0,0,0*4E")); // GPS DOP and active satellites
                                                      //	serialGPS->println(F("$PUBX,40,RMC,0,0,0,0*47"));       // Recommended minimum specific GPS/Transit data
    serialGPS->println(F("$PUBX,40,GSV,0,0,0,0*59")); // GNSS satellites in view
    serialGPS->println(F("$PUBX,40,VTG,0,0,0,0*5E")); // Track made good and ground speed
}

#endif