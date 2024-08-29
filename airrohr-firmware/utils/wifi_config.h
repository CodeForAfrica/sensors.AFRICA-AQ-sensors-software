#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif

#if defined(ESP32)
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#endif

// Function declarations --------------------------------
static void wifiConfig();
static int selectChannelForAp();
static void waitForWifiToConnect(int maxRetries);
static void connectWifi();
static void configureCACertTrustAnchor(WiFiClientSecure *client);
static WiFiClient *getNewLoggerWiFiClient(const LoggerEntry logger);

/*****************************************************************
 * WifiConfig                                                    *
 *****************************************************************/
static void wifiConfig()
{
    debug_outln_info(F("Starting WiFiManager"));
    debug_outln_info(F("AP ID: "), String(cfg::fs_ssid));
    debug_outln_info(F("Password: "), String(cfg::fs_pwd));

    wificonfig_loop = true;

    WiFi.disconnect(true);
    debug_outln_info(F("scan for wifi networks..."));
    count_wifiInfo = WiFi.scanNetworks(false /* scan async */, true /* show hidden networks */);
    delete[] wifiInfo;
    wifiInfo = new struct_wifiInfo[count_wifiInfo];

    for (int i = 0; i < count_wifiInfo; i++)
    {
        String SSID;
        uint8_t *BSSID;

        memset(&wifiInfo[i], 0, sizeof(struct_wifiInfo));
#if defined(ESP8266)
        WiFi.getNetworkInfo(i, SSID, wifiInfo[i].encryptionType,
                            wifiInfo[i].RSSI, BSSID, wifiInfo[i].channel,
                            wifiInfo[i].isHidden);
#else
        WiFi.getNetworkInfo(i, SSID, wifiInfo[i].encryptionType,
                            wifiInfo[i].RSSI, BSSID, wifiInfo[i].channel);
#endif
        SSID.toCharArray(wifiInfo[i].ssid, sizeof(wifiInfo[0].ssid));
    }

    WiFi.mode(WIFI_AP);
    const IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(cfg::fs_ssid, cfg::fs_pwd, selectChannelForAp());
    // In case we create a unique password at first start
    debug_outln_info(F("AP Password is: "), cfg::fs_pwd);

    DNSServer dnsServer;
    // Ensure we don't poison the client DNS cache
    dnsServer.setTTL(0);
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", apIP); // 53 is port for DNS server

    setup_webserver();

    // 10 minutes timeout for wifi config
    last_page_load = millis();
    while ((millis() - last_page_load) < cfg::time_for_wifi_config + 500)
    {
        dnsServer.processNextRequest();
        server.handleClient();
#if defined(ESP8266)
        wdt_reset(); // nodemcu is alive
        MDNS.update();
#endif
        yield();
    }

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);

    dnsServer.stop();
    delay(100);

    debug_outln_info(FPSTR(DBG_TXT_CONNECTING_TO), cfg::wlanssid);

    WiFi.begin(cfg::wlanssid, cfg::wlanpwd);

    debug_outln_info(F("---- Result Webconfig ----"));
    debug_outln_info(F("WLANSSID: "), cfg::wlanssid);
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_info_bool(F("PPD: "), cfg::ppd_read);
    debug_outln_info_bool(F("SDS: "), cfg::sds_read);
    debug_outln_info_bool(F("GSM: "), cfg::gsm_capable);
    debug_outln_info_bool(F("PMS: "), cfg::pms_read);
    debug_outln_info_bool(F("HPM: "), cfg::hpm_read);
    debug_outln_info_bool(F("SPS30: "), cfg::sps30_read);
    debug_outln_info_bool(F("DHT: "), cfg::dht_read);
    debug_outln_info_bool(F("DS18B20: "), cfg::ds18b20_read);
    debug_outln_info_bool(F("HTU21D: "), cfg::htu21d_read);
    debug_outln_info_bool(F("BMP: "), cfg::bmp_read);
    debug_outln_info_bool(F("DNMS: "), cfg::dnms_read);
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_info_bool(F("SensorCommunity: "), cfg::send2dusti);
    debug_outln_info_bool(F("Madavi: "), cfg::send2madavi);
    debug_outln_info_bool(F("CSV: "), cfg::send2csv);
    debug_outln_info(FPSTR(DBG_TXT_SEP));
    debug_outln_info_bool(F("Autoupdate: "), cfg::auto_update);
    debug_outln_info_bool(F("Display: "), cfg::has_display);
    debug_outln_info_bool(F("LCD 1602: "), !!lcd_1602);
    debug_outln_info(F("Debug: "), String(cfg::debug));
    wificonfig_loop = false;
}

static int selectChannelForAp()
{
    std::array<int, 14> channels_rssi;
    std::fill(channels_rssi.begin(), channels_rssi.end(), -100);

    for (unsigned i = 0; i < count_wifiInfo; i++)
    {
        if (wifiInfo[i].RSSI > channels_rssi[wifiInfo[i].channel])
        {
            channels_rssi[wifiInfo[i].channel] = wifiInfo[i].RSSI;
        }
    }

    if ((channels_rssi[1] < channels_rssi[6]) && (channels_rssi[1] < channels_rssi[11]))
    {
        return 1;
    }
    else if ((channels_rssi[6] < channels_rssi[1]) && (channels_rssi[6] < channels_rssi[11]))
    {
        return 6;
    }
    else
    {
        return 11;
    }
}

static void waitForWifiToConnect(int maxRetries)
{
    int retryCount = 0;
    while ((WiFi.status() != WL_CONNECTED) && (retryCount < maxRetries))
    {
        delay(500);
        debug_out(".", DEBUG_MIN_INFO);
        ++retryCount;
    }
}

/*****************************************************************
 * WiFi auto connecting script                                   *
 *****************************************************************/
static void connectWifi()
{
    display_debug(F("Connecting to"), String(cfg::wlanssid));
#if defined(ESP8266)
    // Enforce Rx/Tx calibration
    system_phy_set_powerup_option(1);
    // 20dBM == 100mW == max tx power allowed in europe
    WiFi.setOutputPower(20.0f);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
    delay(100);
#endif
    if (WiFi.getAutoConnect())
    {
        WiFi.setAutoConnect(false);
    }
    if (!WiFi.getAutoReconnect())
    {
        WiFi.setAutoReconnect(true);
    }
    WiFi.mode(WIFI_STA);
    WiFi.hostname(cfg::fs_ssid);
    WiFi.begin(cfg::wlanssid, cfg::wlanpwd); // Start WiFI

    debug_outln_info(FPSTR(DBG_TXT_CONNECTING_TO), cfg::wlanssid);

    waitForWifiToConnect(40);
    debug_outln_info(emptyString);
    if (WiFi.status() != WL_CONNECTED)
    {
        String fss(cfg::fs_ssid);
        display_debug(fss.substring(0, 16), fss.substring(16));
        wifiConfig();
        if (WiFi.status() != WL_CONNECTED)
        {
            waitForWifiToConnect(20);
            debug_outln_info(emptyString);
        }
    }
    debug_outln_info(F("WiFi connected, IP is: "), WiFi.localIP().toString());
    last_signal_strength = WiFi.RSSI();

    if (MDNS.begin(cfg::fs_ssid))
    {
        MDNS.addService("http", "tcp", 80);
        MDNS.addServiceTxt("http", "tcp", "PATH", "/config");
    }
}

#if defined(ESP8266)
BearSSL::X509List x509_dst_root_ca(dst_root_ca_x3);

static void configureCACertTrustAnchor(WiFiClientSecure *client)
{
    constexpr time_t fw_built_year = (__DATE__[7] - '0') * 1000 +
                                     (__DATE__[8] - '0') * 100 +
                                     (__DATE__[9] - '0') * 10 +
                                     (__DATE__[10] - '0');
    if (time(nullptr) < (fw_built_year - 1970) * 365 * 24 * 3600)
    {
        debug_outln_info(F("Time incorrect; Disabling CA verification."));
        client->setInsecure();
    }
    else
    {
        client->setTrustAnchors(&x509_dst_root_ca);
    }
}
#endif

static WiFiClient *getNewLoggerWiFiClient(const LoggerEntry logger)
{

    WiFiClient *_client;
    if (loggerConfigs[logger].session)
    {
        _client = new WiFiClientSecure;
#if defined(ESP8266)
        static_cast<WiFiClientSecure *>(_client)->setSession(loggerConfigs[logger].session);
        static_cast<WiFiClientSecure *>(_client)->setBufferSizes(1024, TCP_MSS > 1024 ? 2048 : 1024);
        switch (logger)
        {
        case Loggeraircms:
        case LoggerInflux:
        case LoggerCustom:
        case LoggerFSapp:
            static_cast<WiFiClientSecure *>(_client)->setInsecure();
            break;
        default:
            configureCACertTrustAnchor(static_cast<WiFiClientSecure *>(_client));
        }
#endif
    }
    else
    {
        _client = new WiFiClient;
    }
    _client->setTimeout(20000);
    return _client;
}

#endif