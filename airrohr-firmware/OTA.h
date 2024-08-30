/*****************************************************************
 * Over The Air Update                                                     *
 *****************************************************************/

#ifndef OTA_H
#define OTA_H
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#endif
// Function declarations
static bool fwDownloadStream(WiFiClientSecure &client, const String &url, Stream *ostream);
static bool fwDownloadStreamFile(WiFiClientSecure &client, const String &url, const String &fname);
static bool launchUpdateLoader(const String &md5);
static void twoStageOTAUpdate();

// Function definitions
static bool fwDownloadStream(WiFiClientSecure &client, const String &url, Stream *ostream)
{

    HTTPClient http;
    int bytes_written = -1;

    http.setTimeout(20 * 1000);
    http.setUserAgent(SOFTWARE_VERSION + ' ' + esp_chipid + ' ' + SDS_version_date() + ' ' +
                      String(cfg::current_lang) + ' ' + String(CURRENT_LANG) + ' ' +
                      String(cfg::use_beta ? F("BETA") : F("")));
    http.setReuse(false);

    debug_outln_verbose(F("HTTP GET: "), String(FPSTR(FW_DOWNLOAD_HOST)) + ':' + String(FW_DOWNLOAD_PORT) + url);

    if (http.begin(client, FPSTR(FW_DOWNLOAD_HOST), FW_DOWNLOAD_PORT, url))
    {
        int r = http.GET();
        debug_outln_verbose(F("GET r: "), String(r));
        last_update_returncode = r;
        if (r == HTTP_CODE_OK)
        {
            bytes_written = http.writeToStream(ostream);
        }
        http.end();
    }

    if (bytes_written > 0)
        return true;

    return false;
}

static bool fwDownloadStreamFile(WiFiClientSecure &client, const String &url, const String &fname)
{

    String fname_new(fname);
    fname_new += F(".new");
    bool downloadSuccess = false;

    File fwFile = SPIFFS.open(fname_new, "w");
    if (fwFile)
    {
        downloadSuccess = fwDownloadStream(client, url, &fwFile);
        fwFile.close();
        if (downloadSuccess)
        {
            SPIFFS.remove(fname);
            SPIFFS.rename(fname_new, fname);
            debug_outln_info(F("Success downloading: "), url);
        }
    }

    if (downloadSuccess)
        return true;

    SPIFFS.remove(fname_new);
    return false;
}

#if defined(ESP8266)
static bool launchUpdateLoader(const String &md5)
{

    File loaderFile = SPIFFS.open(F("/loader.bin"), "r");
    if (!loaderFile)
    {
        return false;
    }

    if (!Update.begin(loaderFile.size(), U_FLASH))
    {
        return false;
    }

    if (md5.length() && !Update.setMD5(md5.c_str()))
    {
        return false;
    }

    if (Update.writeStream(loaderFile) != loaderFile.size())
    {
        return false;
    }
    loaderFile.close();

    if (!Update.end())
    {
        return false;
    }

    debug_outln_info(F("Erasing SDK config."));
    ESP.eraseConfig();

    sensor_restart();
    return true;
}
#endif

static void twoStageOTAUpdate()
{

    if (!cfg::auto_update)
        return;

#if defined(ESP8266)
    debug_outln_info(F("twoStageOTAUpdate"));

    String lang_variant(cfg::current_lang);
    if (lang_variant.length() != 2)
    {
        lang_variant = CURRENT_LANG;
    }
    lang_variant.toLowerCase();

    String fetch_name(F(OTA_BASENAME "/update/latest_"));
    if (cfg::use_beta)
    {
        fetch_name = F(OTA_BASENAME "/beta/latest_");
    }
    fetch_name += lang_variant;
    fetch_name += F(".bin");

    WiFiClientSecure client;
    BearSSL::Session clientSession;

    client.setBufferSizes(1024, TCP_MSS > 1024 ? 2048 : 1024);
    client.setSession(&clientSession);
    configureCACertTrustAnchor(&client);

    String fetch_md5_name(fetch_name);
    fetch_md5_name += F(".md5");

    StreamString newFwmd5;
    if (!fwDownloadStream(client, fetch_md5_name, &newFwmd5))
        return;

    newFwmd5.trim();
    if (newFwmd5 == ESP.getSketchMD5())
    {
        display_debug(FPSTR(DBG_TXT_UPDATE), FPSTR(DBG_TXT_UPDATE_NO_UPDATE));
        debug_outln_verbose(F("No newer version available."));
        return;
    }

    debug_outln_info(F("Update md5: "), newFwmd5);
    debug_outln_info(F("Sketch md5: "), ESP.getSketchMD5());

    // We're entering update phase, kill off everything else
    WiFiUDP::stopAll();
    WiFiClient::stopAllExcept(&client);
    delay(100);

    String firmware_name(F("/firmware.bin"));
    String firmware_md5(F("/firmware.bin.md5"));
    String loader_name(F("/loader.bin"));
    if (!fwDownloadStreamFile(client, fetch_name, firmware_name))
        return;
    if (!fwDownloadStreamFile(client, fetch_md5_name, firmware_md5))
        return;
    if (!fwDownloadStreamFile(client, FPSTR(FW_2ND_LOADER_URL), loader_name))
        return;

    File fwFile = SPIFFS.open(firmware_name, "r");
    if (!fwFile)
    {
        SPIFFS.remove(firmware_name);
        SPIFFS.remove(firmware_md5);
        debug_outln_error(F("Failed reopening fw file.."));
        return;
    }
    size_t fwSize = fwFile.size();
    MD5Builder md5;
    md5.begin();
    md5.addStream(fwFile, fwSize);
    md5.calculate();
    fwFile.close();
    String md5String = md5.toString();

    // Firmware is always at least 128 kB and padded to 16 bytes
    if (fwSize < (1 << 17) || (fwSize % 16 != 0) || newFwmd5 != md5String)
    {
        debug_outln_info(F("FW download failed validation.. deleting"));
        SPIFFS.remove(firmware_name);
        SPIFFS.remove(firmware_md5);
        return;
    }

    StreamString loaderMD5;
    if (!fwDownloadStream(client, String(FPSTR(FW_2ND_LOADER_URL)) + F(".md5"), &loaderMD5))
        return;

    loaderMD5.trim();

    debug_outln_info(F("launching 2nd stage"));
    if (!launchUpdateLoader(loaderMD5))
    {
        debug_outln_error(FPSTR(DBG_TXT_UPDATE_FAILED));
        display_debug(FPSTR(DBG_TXT_UPDATE), FPSTR(DBG_TXT_UPDATE_FAILED));
        SPIFFS.remove(firmware_name);
        SPIFFS.remove(firmware_md5);
        return;
    }
#endif
}

#endif