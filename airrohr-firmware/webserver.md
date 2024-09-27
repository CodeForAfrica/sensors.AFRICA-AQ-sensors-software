
# How the webserver is ***served***

## When the intention is to connect to a WiFi hostpot 
The device will attempt to a connect to WiFi hotspot from the configurations saved in a `config.json` file if any.
Otherwise, the device will set the WiFi mode to Access Point which turns the ESP module into a WiFi hotpost with the SSID and password of choice. 

The webserver is started and when a client connects to the this hotspot, a captive portal will be initiated redirecting the client to the `/config` page.
The user can input device setting and save the configuration. The device restarts itself and the process repeats itself.

If the device had or now successfully connects to another WiFi hotspost i.e. ESP WiFi is on Station Mode, the webserver can be accessed from the IP assigned by the WiFi hotspot/router.

## When the intention is use any communication network other than WiFi
This can be in scenarios where GSM is the primary mode of communation for example.

The WiFi mode is set to AP mode and the webserver is served via `192.168.4.1`. A capative portal is initiated when connecting the ESP's WiFI hotspot and redirects you to the `/config` page. All pages can be accessed by the web browser still.

# Routes/ Web pages

## 1. `/config`
 
 This page is for device configurations such as WiFi settings, Custom API, GPRS APN settings etc.

## 2. `/wifi`

Lists available WiFi hotspots

## 3. `/values`

Lists sensor values

## 4. `/status`
Show device's statuses such as Free Memory, Heap Fragmentation, Last OTA update, NTP info, device uptime etc.

## 5. `/debug`
Set device debug level

## 6. `/removeConfig`
Remove device configs stored in `/config.json` and `/config.json.old` SPIFF files.

## 7. `/reset`
Restarts the sensor kit.

## 8. `/data.json`
Displays sensor data values in JSON format.

## 9. `/images`
Renders the logo.

## 10. `/ota_update`
Page to upload firmware file.

## 11. `/ota_upload`
Route to handle file uploads. Used to handle file(s) used for ota update via the webserver.