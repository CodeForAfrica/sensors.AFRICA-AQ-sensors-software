
#ifndef _NETWORK_TIME_H
#define _NETWORK_TIME_H

#if defined(ESP8266)
#include <ctime>
#include <sntp.h>
#endif

#include "../ext_def.h"

static void setupNetworkTime()
{
    // server name ptrs must be persisted after the call to configTime because internally
    // the pointers are stored see implementation of lwip sntp_setservername()
    static char ntpServer1[18], ntpServer2[18], ntpServer3[18];
#if defined(ESP8266)
    settimeofday_cb([]()
                    {
		if (!sntp_time_set) {
			time_t now = time(nullptr);
			debug_outln_info(F("SNTP synced: "), ctime(&now));
			// twoStageOTAUpdate();
			// last_update_attempt = millis();
		}
		sntp_time_set++; });
#endif
    strcpy_P(ntpServer1, NTP_SERVER_1);
    strcpy_P(ntpServer2, NTP_SERVER_2);
    configTime(0, 0, ntpServer1, ntpServer2, ntpServer3);
}

#endif