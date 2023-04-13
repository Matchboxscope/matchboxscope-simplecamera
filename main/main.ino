#include <WiFiUdp.h>
#include <esp_camera.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include "parsebytes.h"
#include "time.h"
#include <ESPmDNS.h>
// #include "improv.h"
// #include "improvSerial.h"
#include <SD_MMC.h>
#include "device_pref.h"
#include "ArduinoJson.h"

/* This sketch is a extension/expansion/reork of the 'official' ESP32 Camera example
 *  sketch from Expressif:
 *  https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
 *
 *  It is modified to allow control of Illumination LED Lamps's (present on some modules),
 *  greater feedback via a status LED, and the HTML contents are present in plain text
 *  for easy modification.
 *
 *  A camera name can now be configured, and wifi details can be stored in an optional
 *  header file to allow easier updated of the repo.
 *
 *  The web UI has had changes to add the lamp control, rotation, a standalone viewer,
 *  more feeedback, new controls and other tweaks and changes,
 * note: Make sure that you have either selected ESP32 AI Thinker,
 *       or another board which has PSRAM enabled to use high resolution camera modes
 */

/*
 *  FOR NETWORK AND HARDWARE SETTINGS COPY OR RENAME 'myconfig.sample.h' TO 'myconfig.h' AND EDIT THAT.
 *
 * By default this sketch will assume an AI-THINKER ESP-CAM and create
 * an accesspoint called "ESP32-CAM-CONNECT" (password: "InsecurePassword")
 *
 */

// Primary config, or defaults.
struct station
{
    const char ssid[65];
    const char password[65];
    const bool dhcp;
}; // do no edit
#include "myconfig.h"

// Upstream version string
#include "version.h"

// Pin Mappings
#include "camera_pins.h"

// Camera config structure
camera_config_t config;

// Internal filesystem (SPIFFS)
// used for non-volatile camera settings
#include "storage.h"

// Sketch Info
int sketchSize;
int sketchSpace;
String sketchMD5;

// Start with accesspoint mode disabled, wifi setup will activate it if
// no known networks are found, and WIFI_AP_ENABLE has been defined
bool accesspoint = false;

// IP address, Netmask and Gateway, populated when connected
IPAddress ip;
IPAddress net;
IPAddress gw;

// Declare external function from app_httpd.cpp
extern void startCameraServer(int hPort, int sPort);
extern void serialDump();
extern void saveCapturedImageGithub();

// Names for the Camera. (set these in myconfig.h)
char myName[] = CAM_NAME;
char mdnsName[] = MDNS_NAME;
// Ports for http and stream (override in myconfig.h)
int httpPort = 80;
int streamPort = 81;

#define WIFI_WATCHDOG 15000

// Number of known networks in stationList[]
int stationCount = sizeof(stationList) / sizeof(stationList[0]);

// If we have AP mode enabled, ignore first entry in the stationList[]
int firstStation = 0;

// Select between full and simple index as the default.
char default_index[] = "full";

// SD Card
boolean sdInitialized = false;

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;
bool captivePortal = true;
char apName[64] = "Undefined";

// The app and stream URLs
char httpURL[64] = {"Undefined"};
char streamURL[64] = {"Undefined"};

// Counters for info screens and debug
int8_t streamCount = 0;          // Number of currently active streams
unsigned long streamsServed = 0; // Total completed streams
unsigned long imagesServed = 0;  // Total image requests

// Preferences
Preferences pref;
DevicePreferences device_pref(pref, "camera", __DATE__ " " __TIME__);

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

// This will be set to the sensors PID (identifier) during initialisation
// camera_pid_t sensorPID;
int sensorPID;

// Camera module bus communications frequency.
// Originally: config.xclk_freq_mhz = 20000000, but this lead to visual artifacts on many modules.
// See https://github.com/espressif/esp32-camera/issues/150#issuecomment-726473652 et al.
unsigned long xclk = 8;

// initial rotation
// can be set in myconfig.h
int myRotation = 0;
bool isStackAcquired = false;

// minimal frame duration in ms, effectively 1/maxFPS
int minFrameTime = 0;

// Timelapse
int timelapseInterval = -1;
static uint64_t t_old = millis();
bool sendToGithubFlag = false;

// Illumination LAMP and status LED
int lampVal = 0;       // default to off
bool autoLamp = false; // Automatic lamp (auto on while camera running)
int pwmVal = 0;        // default no-value
bool BUSY_SET_LED = false;

int lampChannel = 7; // a free PWM channel (some channels used by camera)
int pwmChannel = 5;
const int pwmfreq = 50000;   // 50K pwm frequency
const int pwmresolution = 9; // duty cycle bit range
const int pwmMax = pow(2, pwmresolution) - 1;

bool filesystem = true;

bool otaEnabled = true;
char otaPassword[] = "";

bool haveTime = false;
const char *ntpServer = "";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

extern bool saveImage(String filename, int lensValue);
// Critical error string; if set during init (camera hardware failure) it
// will be returned for all http requests
String critERR = "";

// Debug flag for stream and capture data
bool debugData;

void debugOn()
{
    debugData = true;
    Serial.println("Camera debug data is enabled (send '{\"d\":1}' for status dump, or any other char to disable debug)");
}

void debugOff()
{
    debugData = false;
    Serial.println("Camera debug data is disabled (send 'd' for status dump, or any other char to enable debug)");
}

// Serial input (debugging controls)
void handleSerial()
{
    if (Serial.available())
    {
        String cmd = Serial.readString(); // Serial.read();
        Serial.print("Serial command");
        Serial.println(cmd);
        const char *mssid = "";
        const char *mpassword = "";
        // Format {"ssid":"Blynk","password":"12345678"}

        StaticJsonDocument<256> doc;
        deserializeJson(doc, cmd);
        if (doc.containsKey("ssid") && doc.containsKey("password"))
        {

            mssid = doc["ssid"];
            mpassword = doc["password"];

            Serial.println("Connecting to Wi-Fi...");
            WiFi.begin(mssid, mpassword);

            int nTrialWifiConnect = 0;
            while (WiFi.status() != WL_CONNECTED)
            {
                nTrialWifiConnect++;
                delay(200);
                Serial.println("Connecting to Wi-Fi...");
                if (nTrialWifiConnect > 10)
                {
                    Serial.println("Failed to connect to Wi-Fi => Rebooting");
                    ESP.restart();
                    break;
                }
            }

            Serial.println("Connected to Wi-Fi!");
            device_pref.setWifiPW(mssid);
            device_pref.setWifiSSID(mpassword);
        }
        if (doc.containsKey("wifiap"))
        {

            mssid = "matchboxscope";
            mpassword = "";

            // Connect to Wi-Fi network with SSID and password
            Serial.print("Setting AP (Access Point)…");
            // Remove the password parameter, if you want the AP (Access Point) to be open
            WiFi.softAP(mssid);

            IPAddress IP = WiFi.softAPIP();
            Serial.print("AP IP address: ");
            Serial.println(IP);
          }
        else if (doc.containsKey("github"))
        {
            // {"github":"send"}
            Serial.println("Sending upload page via json");
            sendToGithubFlag = true;
            // saveCapturedImageGithub();
            Serial.println("Github Upload Requested");
        }
        else if (doc.containsKey("d"))
        {
            // {"d":1}
            log_d("Serial command: %c", cmd);
            serialDump();
        }
    }

    while (Serial.available())
        Serial.read(); // chomp the buffer
}

// Notification LED
void flashLED(int flashtime)
{
    digitalWrite(LED_PIN, LED_ON);  // On at full power.
    delay(flashtime);               // delay
    digitalWrite(LED_PIN, LED_OFF); // turn Off
}


// Lamp Control
void setLamp(int newVal)
{
    if (newVal != -1 and !BUSY_SET_LED)
    {
        BUSY_SET_LED = true;
        // Apply a logarithmic function to the scale.
        int brightness = round((pow(2, (1 + (newVal * 0.02))) - 2) / 6 * pwmMax);
        ledcWrite(lampChannel, brightness);
        Serial.print("Lamp: ");
        Serial.print(newVal);
        Serial.print("%, pwm = ");
        Serial.println(brightness);
        BUSY_SET_LED = false;
        delay(15); // settle time
    }
}

// PWM Control
void setPWM(int newVal)
{
    if (newVal != -1)
    {
        // Apply a logarithmic function to the scale.
        int current = round((pow(2, (1 + (newVal * 0.02))) - 2) / 6 * pwmMax);
        ledcWrite(pwmChannel, current);
        Serial.print("Current: ");
        Serial.print(newVal);
        Serial.print("%, pwm = ");
        Serial.println(current);
    }
    
}

void printLocalTime(bool extraData = false)
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
    }
    else
    {
        Serial.println(&timeinfo, "%H:%M:%S, %A, %B %d %Y");
    }
    if (extraData)
    {
        Serial.printf("NTP Server: %s, GMT Offset: %li(s), DST Offset: %i(s)\r\n", ntpServer, gmtOffset_sec, daylightOffset_sec);
    }
}

void calcURLs()
{
    // Set the URL's
    Serial.println("Setting httpURL");
    if (httpPort != 80)
    {
        sprintf(httpURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], httpPort);
    }
    else
    {
        sprintf(httpURL, "http://%d.%d.%d.%d/", ip[0], ip[1], ip[2], ip[3]);
    }
    sprintf(streamURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], streamPort);
}

void StartCamera()
{
    // Populate camera config structure with hardware and other defaults
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = xclk * 1000000;
    config.pixel_format = PIXFORMAT_JPEG;
    // Low(ish) default framesize and quality
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        delay(100); // need a delay here or the next serial o/p gets missed
        Serial.printf("\r\n\r\nCRITICAL FAILURE: Camera sensor failed to initialise.\r\n\r\n");
        Serial.printf("A full (hard, power off/on) reboot will probably be needed to recover from this.\r\n");
        Serial.printf("Meanwhile; this unit will reboot in 1 minute since these errors sometime clear automatically\r\n");
        // Reset the I2C bus.. may help when rebooting.
        periph_module_disable(PERIPH_I2C0_MODULE); // try to shut I2C down properly in case that is the problem
        periph_module_disable(PERIPH_I2C1_MODULE);
        periph_module_reset(PERIPH_I2C0_MODULE);
        periph_module_reset(PERIPH_I2C1_MODULE);
        // And set the error text for the UI
        critERR = "<h1>Error!</h1><hr><p>Camera module failed to initialise!</p><p>Please reset (power off/on) the camera.</p>";
        critERR += "<p>We will continue to reboot once per minute since this error sometimes clears automatically.</p>";
        // Start a 60 second watchdog timer
        esp_task_wdt_init(60, true);
        esp_task_wdt_add(NULL);
    }
    else
    {
        Serial.println("Camera init succeeded");

        // Get a reference to the sensor
        sensor_t *s = esp_camera_sensor_get();

        // Dump camera module, warn for unsupported modules.
        sensorPID = s->id.PID;
        switch (sensorPID)
        {
        case OV9650_PID:
            Serial.println("WARNING: OV9650 camera module is not properly supported, will fallback to OV2640 operation");
            break;
        case OV7725_PID:
            Serial.println("WARNING: OV7725 camera module is not properly supported, will fallback to OV2640 operation");
            break;
        case OV2640_PID:
            Serial.println("OV2640 camera module detected");
            break;
        case OV3660_PID:
            Serial.println("OV3660 camera module detected");
            break;
        default:
            Serial.println("WARNING: Camera module is unknown and not properly supported, will fallback to OV2640 operation");
        }

        // OV3660 initial sensors are flipped vertically and colors are a bit saturated
        if (sensorPID == OV3660_PID)
        {
            s->set_vflip(s, 1);       // flip it back
            s->set_brightness(s, 1);  // up the blightness just a bit
            s->set_saturation(s, -2); // lower the saturation
        }

        /*
         * Add any other defaults you want to apply at startup here:
         * uncomment the line and set the value as desired (see the comments)
         *
         * these are defined in the esp headers here:
         * https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h#L149
         */

        // s->set_framesize(s, FRAMESIZE_SVGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA|QXGA(ov3660)]);
        // s->set_quality(s, val);       // 10 to 63
        // s->set_brightness(s, 0);      // -2 to 2
        // s->set_contrast(s, 0);        // -2 to 2
        // s->set_saturation(s, 0);      // -2 to 2
        // s->set_special_effect(s, 0);  // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
        // s->set_whitebal(s, 1);        // aka 'awb' in the UI; 0 = disable , 1 = enable
        // s->set_awb_gain(s, 1);        // 0 = disable , 1 = enable
        // s->set_wb_mode(s, 0);         // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
        // s->set_exposure_ctrl(s, 1);   // 0 = disable , 1 = enable
        // s->set_aec2(s, 0);            // 0 = disable , 1 = enable
        // s->set_ae_level(s, 0);        // -2 to 2
        // s->set_aec_value(s, 300);     // 0 to 1200
        // s->set_gain_ctrl(s, 1);       // 0 = disable , 1 = enable
        // s->set_agc_gain(s, 0);        // 0 to 30
        // s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        // s->set_bpc(s, 0);             // 0 = disable , 1 = enable
        // s->set_wpc(s, 1);             // 0 = disable , 1 = enable
        // s->set_raw_gma(s, 1);         // 0 = disable , 1 = enable
        // s->set_lenc(s, 1);            // 0 = disable , 1 = enable
        // s->set_hmirror(s, 0);         // 0 = disable , 1 = enable
        // s->set_vflip(s, 0);           // 0 = disable , 1 = enable
        // s->set_dcw(s, 1);             // 0 = disable , 1 = enable
        // s->set_colorbar(s, 0);        // 0 = disable , 1 = enable
    }
    // We now have camera with default init
}
/*
bool improv_config()
{
    // Return 'true' if SSID and password received within IMPROV_CONFIG_TIMEOUT seconds
    bool connecting = false;

    WiFi.disconnect();
    WiFi.mode(WIFI_STA);

    int VERSION = 0;
    String host = "Matchboxscope";
    String PLATFORMIO_ENV = "unknown";
    int IMPROV_CONNECT_TIMEOUT = 30;

    improv_serial::global_improv_serial.setup(String("PedalinoMini (TM)"), String(VERSION), PLATFORMIO_ENV, String("Device name: ") + String(host));

    unsigned long startCrono = millis();
    unsigned long crono = millis() - startCrono;
    while (!WiFi.isConnected() && crono / 1000 < IMPROV_CONNECT_TIMEOUT)
    {
        improv_serial::global_improv_serial.loop();
        if (!connecting && improv_serial::global_improv_serial.get_state() == improv::STATE_PROVISIONING)
        {
            log_d("Connecting to", improv_serial::global_improv_serial.get_ssid());
            connecting = true;
        }
        if (crono % 200 < 5)
            log_d(crono / 200, IMPROV_CONNECT_TIMEOUT * 5 - 1);
        crono = millis() - startCrono;
    }

    /*
    set_wifi_power_saving_off();
    leds_off();
    display_progress_bar_update(1, 1);

    if (WiFi.isConnected())
    {

        improv_serial::global_improv_serial.loop();

        wifiSSID = WiFi.SSID();
        wifiPassword = WiFi.psk();

        DPRINT("SSID        : %s\n", WiFi.SSID().c_str());
        DPRINT("Password    : %s\n", WiFi.psk().c_str());

        eeprom_update_sta_wifi_credentials(WiFi.SSID(), WiFi.psk());
    }
    else
    {
        improv_serial::global_improv_serial.loop(true);
        DPRINT("WiFi Provisioning timeout\n");
    }

    return WiFi.isConnected();

    return true;
}
*/
void WifiSetup()
{
    // Feedback that we are now attempting to connect
    flashLED(300);
    delay(100);
    flashLED(300);
    Serial.println("Starting WiFi");

    // Disable power saving on WiFi to improve responsiveness
    // (https://github.com/espressif/arduino-esp32/issues/1484)
    WiFi.setSleep(false);

    Serial.print("Known external SSIDs: ");
    if (stationCount > firstStation)
    {
        for (int i = firstStation; i < stationCount; i++)
            Serial.printf(" '%s'", stationList[i].ssid);
    }
    else
    {
        Serial.print("None");
    }
    Serial.println();
    byte mac[6] = {0, 0, 0, 0, 0, 0};
    WiFi.macAddress(mac);
    Serial.printf("MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    int bestStation = -1;
    long bestRSSI = -1024;
    char bestSSID[65] = "";
    uint8_t bestBSSID[6];
    if (stationCount > firstStation)
    {
        // We have a list to scan
        Serial.printf("Scanning local Wifi Networks\r\n");
        int stationsFound = WiFi.scanNetworks();
        Serial.printf("%i networks found\r\n", stationsFound);
        if (stationsFound > 0)
        {
            for (int i = 0; i < stationsFound; ++i)
            {
                // Print SSID and RSSI for each network found
                String thisSSID = WiFi.SSID(i);
                int thisRSSI = WiFi.RSSI(i);
                String thisBSSID = WiFi.BSSIDstr(i);
                Serial.printf("%3i : [%s] %s (%i)", i + 1, thisBSSID.c_str(), thisSSID.c_str(), thisRSSI);
                // Scan our list of known external stations
                for (int sta = firstStation; sta < stationCount; sta++)
                {
                    if ((strcmp(stationList[sta].ssid, thisSSID.c_str()) == 0) ||
                        (strcmp(stationList[sta].ssid, thisBSSID.c_str()) == 0))
                    {
                        Serial.print("  -  Known!");
                        // Chose the strongest RSSI seen
                        if (thisRSSI > bestRSSI)
                        {
                            bestStation = sta;
                            strncpy(bestSSID, thisSSID.c_str(), 64);
                            // Convert char bssid[] to a byte array
                            parseBytes(thisBSSID.c_str(), ':', bestBSSID, 6, 16);
                            bestRSSI = thisRSSI;
                        }
                    }
                }
                Serial.println();
            }
        }
    }
    else
    {
        // No list to scan, therefore we are an accesspoint
        accesspoint = true;
    }

    if (bestStation == -1)
    {
        if (!accesspoint)
        {
            Serial.println("No known networks found, Trying existing one from perferences.. (set via serial..)");
            // try existing SSID from prefes
            // Initiate network connection request (3rd argument, channel = 0 is 'auto')
            WiFi.begin(device_pref.getWifiSSID().c_str(), device_pref.getWifiPW().c_str());

            // Wait to connect, or timeout
            unsigned long start = millis();
            while ((millis() - start <= WIFI_WATCHDOG) && (WiFi.status() != WL_CONNECTED))
            {
                delay(500);
                Serial.print('.');
            }
        }
        else
        {
            Serial.println("AccessPoint mode selected in config");
        }
    }
    else
    {
        Serial.printf("Connecting to Wifi Network %d: [%02X:%02X:%02X:%02X:%02X:%02X] %s \r\n",
                      bestStation, bestBSSID[0], bestBSSID[1], bestBSSID[2], bestBSSID[3],
                      bestBSSID[4], bestBSSID[5], bestSSID);
        // Apply static settings if necesscary
        WiFi.setHostname(mdnsName);

        // Initiate network connection request (3rd argument, channel = 0 is 'auto')
        WiFi.begin(bestSSID, stationList[bestStation].password, 0, bestBSSID);

        // Wait to connect, or timeout
        unsigned long start = millis();
        while ((millis() - start <= WIFI_WATCHDOG) && (WiFi.status() != WL_CONNECTED))
        {
            delay(500);
            Serial.print('.');
        }
        // If we have connected, inform user
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println("Client connection succeeded");
            accesspoint = false;
            // Note IP details
            ip = WiFi.localIP();
            net = WiFi.subnetMask();
            gw = WiFi.gatewayIP();
            Serial.printf("IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
            Serial.printf("Netmask   : %d.%d.%d.%d\r\n", net[0], net[1], net[2], net[3]);
            Serial.printf("Gateway   : %d.%d.%d.%d\r\n", gw[0], gw[1], gw[2], gw[3]);
            calcURLs();
            // Flash the LED to show we are connected
            for (int i = 0; i < 5; i++)
            {
                flashLED(50);
                delay(150);
            }
        }
        else
        {
            Serial.println("Client connection Failed");
            WiFi.disconnect(); // (resets the WiFi scan)

            // try existing SSID from prefes
            // Initiate network connection request (3rd argument, channel = 0 is 'auto')
            WiFi.begin(device_pref.getWifiSSID().c_str(), device_pref.getWifiPW().c_str());

            // Wait to connect, or timeout
            unsigned long start = millis();
            while ((millis() - start <= WIFI_WATCHDOG) && (WiFi.status() != WL_CONNECTED))
            {
                delay(500);
                Serial.print('.');
            }
        }

        // If we still haven't connected switch to AP mode
        if (!WiFi.status() == WL_CONNECTED){
            Serial.println("Client connection not successful - switching to AP Mode: SSID: 'Matchboxscope'");
            
            WiFi.disconnect(); // (resets the WiFi scan)
            WiFi.mode(WIFI_AP);
            WiFi.softAP("Matchboxscope");
            Serial.print("[+] AP Created with IP Gateway ");
            Serial.println(WiFi.softAPIP());

            // Note IP details
            ip = WiFi.localIP();
            net = WiFi.subnetMask();
            gw = WiFi.gatewayIP();
            Serial.printf("IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
            Serial.printf("Netmask   : %d.%d.%d.%d\r\n", net[0], net[1], net[2], net[3]);
            Serial.printf("Gateway   : %d.%d.%d.%d\r\n", gw[0], gw[1], gw[2], gw[3]);
            calcURLs();
            
        }
    }

    if (accesspoint && (WiFi.status() != WL_CONNECTED))
    {
// The accesspoint has been enabled, and we have not connected to any existing networks
#if defined(AP_CHAN)
        Serial.println("Setting up Fixed Channel AccessPoint");
        Serial.print("  SSID     : ");
        Serial.println(stationList[0].ssid);
        Serial.print("  Password : ");
        Serial.println(stationList[0].password);
        Serial.print("  Channel  : ");
        Serial.println(AP_CHAN);
        WiFi.softAP(stationList[0].ssid, stationList[0].password, AP_CHAN);
#else
        Serial.println("Setting up AccessPoint");
        Serial.print("  SSID     : ");
        Serial.println(stationList[0].ssid);
        Serial.print("  Password : ");
        Serial.println(stationList[0].password);
        WiFi.softAP(stationList[0].ssid, stationList[0].password);
#endif
#if defined(AP_ADDRESS)
        // User has specified the AP details; apply them after a short delay
        // (https://github.com/espressif/arduino-esp32/issues/985#issuecomment-359157428)
        delay(100);
        IPAddress local_IP(AP_ADDRESS);
        IPAddress gateway(AP_ADDRESS);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_IP, gateway, subnet);
#endif
        // Note AP details
        ip = WiFi.softAPIP();
        net = WiFi.subnetMask();
        gw = WiFi.gatewayIP();
        strcpy(apName, stationList[0].ssid);
        Serial.printf("IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
        calcURLs();
        // Flash the LED to show we are connected
        for (int i = 0; i < 5; i++)
        {
            flashLED(150);
            delay(50);
        }
        // Start the DNS captive portal if requested
        if (stationList[0].dhcp == true)
        {
            Serial.println("Starting Captive Portal");
            dnsServer.start(DNS_PORT, "*", ip);
            captivePortal = true;
        }
    }
}

void handleSerialTask(void *pvParameters)
{
    Serial.println("Creating task handleSerial");
    Serial.println("You can enter your wifi password and ssid via a json string e.g.{\"ssid\":\"SSID_NAME\",\"password\":\"SSID-PASSWORD\"}");
    while (true)
    {
        handleSerial();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void saveCapturedImageGithubTask(void *pvParameters)
{
    Serial.println("Creating task saveCapturedImageGithubTask");
    while (true)
    {
        if (sendToGithubFlag)
        {
            Serial.println("Sending to Github");
            sendToGithubFlag = false;
            saveCapturedImageGithub();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    // actions done after first flashing
    bool isFirstRun = device_pref.isFirstRun();
    if (isFirstRun)
    {
        device_pref.setTimelapseInterval(-1);
        device_pref.setIsTimelapseAnglerfish(false); 
    }
    isStackAcquired = device_pref.getAcquireStack();

    // only for Anglerfish if already focussed
    bool isTimelapseAnglerfish = device_pref.getIsTimelapseAnglerfish(); // set the global variable for the loop function

    // Debug info
    Serial.println();
    Serial.println("====");
    Serial.print("esp32-cam-webserver: ");
    Serial.println(myName);
    Serial.print("Code Built: ");
    Serial.println(myVer);
    Serial.print("Base Release: ");
    Serial.println(baseVersion);
    Serial.println();


    // Warn if no PSRAM is detected (typically user error with board selection in the IDE)
    if (!psramFound())
    {
        Serial.println("\r\nFatal Error; Halting");
        while (true)
        {
            Serial.println("No PSRAM found; camera cannot be initialised: Please check the board config for your module.");
            delay(5000);
        }
    }

    if (stationCount == 0)
    {
        Serial.println("\r\nFatal Error; Halting");
        while (true)
        {
            Serial.println("No wifi details have been configured; we cannot connect to existing WiFi or start our own AccessPoint, there is no point in proceeding.");
            delay(5000);
        }
    }

    // Start the SPIFFS filesystem before we initialise the camera
    if (filesystem)
    {
        filesystemStart();
        delay(200); // a short delay to let spi bus settle after SPIFFS init
    }

    // Start (init) the camera
    StartCamera();

    // initialize SD card before LED!!
    // We initialize SD_MMC here rather than in setup() because SD_MMC needs to reset the light pin
    // with a different pin mode.
    // 1-bit mode as suggested here:https://dr-mntn.net/2021/02/using-the-sd-card-in-1-bit-mode-on-the-esp32-cam-from-ai-thinker
    if (!SD_MMC.begin("/sdcard", true))
    { // FIXME: this sometimes leads to issues Unix vs. Windows formating - text encoding? Sometimes it copies to "sdcard" => Autoformating does this!!!
        Serial.println("SD Card Mount Failed");
        // FIXME: This should be indicated in the GUI
        sdInitialized = false;
        device_pref.setIsTimelapseAnglerfish(false); // FIXME: if SD card is missing => streaming mode!
        isTimelapseAnglerfish = false;
        // Flash the LED to show SD card is not connected
        for (int i = 0; i < 20; i++){flashLED(50); delay(50);}
        
    }
    else
    {
        sdInitialized = true;
        Serial.println("SD Card Mounted");

        // Check for an SD card
        uint8_t cardType = SD_MMC.cardType();
        if (cardType == CARD_NONE)
        {
            Serial.println("No SD card attached");
        }
        else
        {
            Serial.println(cardType);
        }
    }

    if (!isTimelapseAnglerfish)
    {
        xTaskCreatePinnedToCore(
            saveCapturedImageGithubTask,   /* Function to implement the task */
            "saveCapturedImageGithubTask", /* Name of the task */
            10000,                         /* Stack size in words */
            NULL,                          /* Task input parameter */
            11,                             /* Priority of the task */
            NULL,                          /* Task handle. */
            1);                            /* Core where the task should run */
        xTaskCreatePinnedToCore(
            handleSerialTask,   /* Function to implement the task */
            "handleSerialTask", /* Name of the task */
            10000,              /* Stack size in words */
            NULL,               /* Task input parameter */
            1,                  /* Priority of the task */
            NULL,               /* Task handle. */
            1);                 /* Core where the task should run */
        Serial.println("Tasks created...");
    }
    // loading previous settings
    imagesServed = device_pref.getFrameIndex();
    timelapseInterval = device_pref.getTimelapseInterval();


    // Initialise and set the lamp
    if (lampVal != -1)
    {
        ledcSetup(lampChannel, pwmfreq, pwmresolution); // configure LED PWM channel
        ledcAttachPin(LAMP_PIN, lampChannel);           // attach the GPIO pin to the channel
        if (autoLamp)
            setLamp(0); // set default value
        else
            setLamp(lampVal);
    }
    else
    {
        Serial.println("No lamp, or lamp disabled in config");
    }

    // Initialise and set the PWM output
    if (pwmVal != -1)
    {
        pinMode(PWM_PIN, OUTPUT);
        log_d("PWM pin: %d", PWM_PIN);
        ledcSetup(pwmChannel, pwmfreq, pwmresolution); // configure LED PWM channel
        ledcAttachPin(PWM_PIN, pwmChannel);            // attach the GPIO pin to the channel
        ledcWrite(pwmChannel, 255);                     // set default value to center so that focus or pump are in ground state
        delay(30);
        ledcWrite(pwmChannel, 0);                     // set default value to center so that focus or pump are in ground state
    }
    else
    {
        Serial.println("No PWM, or PWM disabled in config");
    }
    // test LEDs
    pinMode(LED_PIN, OUTPUT);
    // visualize we are "on"
    setLamp(20);
    delay(50);
    setLamp(0);


    // Now load and apply any saved preferences
    if (filesystem)
    {
        delay(200); // a short delay to let spi bus settle after camera init
        loadPrefs(SPIFFS);
        Serial.println("internal files System found and mounted");
    }
    else
    {
        Serial.println("No Internal Filesystem, cannot load or save preferences");
    }

    // before we start the WIFI - check if we are in deep-sleep/anglerfishmode
    initAnglerfish(isTimelapseAnglerfish);

    /*
     * Camera setup complete; initialise the rest of the hardware.
     */

    // Start Wifi and loop until we are connected or have started an AccessPoint
    while ((WiFi.status() != WL_CONNECTED) && !accesspoint)
    {
        WifiSetup();
        delay(1000);
    }

    // Set up OTA
    if (otaEnabled)
    {
        // Start OTA once connected
        Serial.println("Setting up OTA");
        // Port defaults to 3232
        // ArduinoOTA.setPort(3232);
        // Hostname defaults to esp3232-[MAC]
        ArduinoOTA.setHostname(mdnsName);
        // No authentication by default
        if (strlen(otaPassword) != 0)
        {
            ArduinoOTA.setPassword(otaPassword);
            Serial.printf("OTA Password: %s\n\r", otaPassword);
        }
        else
        {
            Serial.printf("\r\nNo OTA password has been set! (insecure)\r\n\r\n");
        }
        ArduinoOTA
            .onStart([]()
                     {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH)
                    type = "sketch";
                else // U_SPIFFS
                    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                    type = "filesystem";
                Serial.println("Start updating " + type);
                // Stop the camera since OTA will crash the module if it is running.
                // the unit will need rebooting to restart it, either by OTA on success, or manually by the user
                Serial.println("Stopping Camera");
                esp_err_t err = esp_camera_deinit();
                critERR = "<h1>OTA Has been started</h1><hr><p>Camera has Halted!</p>";
                critERR += "<p>Wait for OTA to finish and reboot, or <a href=\"control?var=reboot&val=0\" title=\"Reboot Now (may interrupt OTA)\">reboot manually</a> to recover</p>"; })
            .onEnd([]()
                   { Serial.println("\r\nEnd"); })
            .onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
            .onError([](ota_error_t error)
                     {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
        ArduinoOTA.begin();
    }
    else
    {
        Serial.println("OTA is disabled");

        if (!MDNS.begin(mdnsName))
        {
            Serial.println("Error setting up MDNS responder!");
        }
        Serial.println("mDNS responder started");
    }

    // MDNS Config -- note that if OTA is NOT enabled this needs prior steps!
    MDNS.addService("http", "tcp", 80);
    Serial.println("Added HTTP service to MDNS server");

    // Set time via NTP server when enabled
    if (haveTime)
    {
        Serial.print("Time: ");
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        printLocalTime(true);
    }
    else
    {
        Serial.println("Time functions disabled");
    }

    // Gather static values used when dumping status; these are slow functions, so just do them once during startup
    sketchSize = ESP.getSketchSize();
    sketchSpace = ESP.getFreeSketchSpace();
    sketchMD5 = ESP.getSketchMD5();

    // Start the camera server
    startCameraServer(httpPort, streamPort);

    if (critERR.length() == 0)
    {
        Serial.printf("\r\nCamera Ready!\r\nUse '%s' to connect\r\n", httpURL);
        Serial.printf("Stream viewer available at '%sview'\r\n", streamURL);
        Serial.printf("Raw stream URL is '%s'\r\n", streamURL);
#if defined(DEBUG_DEFAULT_ON)
        debugOn();
#else
        debugOff();
#endif
    }
    else
    {
        Serial.printf("\r\nCamera unavailable due to initialisation errors.\r\n\r\n");
    }

    // Info line; use for Info messages; eg 'This is a Beta!' warnings, etc. as necesscary
    // Serial.print("\r\nThis is the 4.1 beta\r\n");

    // As a final init step chomp out the serial buffer in case we have recieved mis-keys or garbage during startup
    while (Serial.available())
        Serial.read();

    // save image to github
    // sendToGithubFlag=true;

    //device_pref.setIsTimelapseAnglerfish(1);
    //device_pref.getIsTimelapseAnglerfish();
}

void acquireFocusStack(String filename, int stepSize = 10, int stepMin = 0, int stepMax = 100)
{
    /*
    Acquire a stack of images 
    */
    for (int iFocus = stepMin; iFocus < stepMax; iFocus += stepSize)
    {
        saveImage(filename + String(imagesServed) + "_z" + String(iFocus), iFocus);
    }
}

void loop()
{

    /*
     *  Just loop forever, reconnecting Wifi As necesscary in client mode
     * The stream and URI handler processes initiated by the startCameraServer() call at the
     * end of setup() will handle the camera and UI processing from now on.
     */

    /*
    if (accesspoint)
    {
        // Accespoint is permanently up, so just loop, servicing the captive portal as needed
        // Rather than loop forever, follow the watchdog, in case we later add auto re-scan.
        unsigned long start = millis();
        while (millis() - start < WIFI_WATCHDOG)
        {
            delay(100);
            if (otaEnabled)
                ArduinoOTA.handle();
            handleSerial();
            if (captivePortal)
                dnsServer.processNextRequest();
        }
    }
    else
    {
        // client mode can fail; so reconnect as appropriate
        static bool warned = false;
        if (WiFi.status() == WL_CONNECTED)
        {
            // We are connected, wait a bit and re-check
            if (warned)
            {
                // Tell the user if we have just reconnected
                Serial.println("WiFi reconnected");
                warned = false;
            }
            // loop here for WIFI_WATCHDOG, turning debugData true/false depending on serial input..
            unsigned long start = millis();
            while (millis() - start < WIFI_WATCHDOG)
            {
                delay(100);
                if (otaEnabled)
                    ArduinoOTA.handle();
                handleSerial();
            }
        }
        else
        {
            // disconnected; attempt to reconnect
            if (!warned)
            {
                // Tell the user if we just disconnected
                WiFi.disconnect(); // ensures disconnect is complete, wifi scan cleared
                Serial.println("WiFi disconnected, retrying");
                warned = true;
            }
            WifiSetup();
        }
    }
    */
    // Timelapse Imaging
    // Perform timelapse imaging
    timelapseInterval = device_pref.getTimelapseInterval();
    if (timelapseInterval > 0 and ((millis() - t_old) > (1000 * timelapseInterval)))
    {
        savePrefs(SPIFFS);
        // https://stackoverflow.com/questions/67090640/errors-while-interacting-with-microsd-card
        log_d("Time to save a new image", timelapseInterval);
        t_old = millis();
        uint32_t frame_index = device_pref.getFrameIndex() + 1;

        // turns on lamp automatically
        // save to SD card if existent

        String filename = "/timelapse_image" + String(imagesServed);
        if (device_pref.getAcquireStack())
        { // FIXME: We could have a switch in the GUI for this settig
            // acquire a stack
            // FIXME: decide which method to use..
            imagesServed++;
            acquireFocusStack(filename, 10);
        }
        else
        {
            // Acquire the image and save
            String filename = "/timelapse_image" + String(imagesServed);
            imagesServed++;
            saveImage(filename, 0);
        }

        // FIXME: we should increase framenumber even if failed - since a corrupted file may lead to issues? (imageSaved)
        device_pref.setFrameIndex(frame_index);
    }

    if (otaEnabled)
        ArduinoOTA.handle();
}

void initAnglerfish(bool isTimelapseAnglerfish)
{

    if (isTimelapseAnglerfish)
    {
        // ONLY IF YOU WANT TO CAPTURE in ANGLERFISHMODE
        Serial.println("In timelapse anglerfish mode.");
        autoLamp = true;
        lampVal = 255;

        // override  camera settings => max framesize
        if(0){
            sensor_t *s = esp_camera_sensor_get();
            s->set_framesize(s, FRAMESIZE_QXGA);
            s->set_quality(s, 10);
        }
    
        // warmup camera
        camera_fb_t *fb = NULL;
        for (int iDummyFrame = 0; iDummyFrame < 5; iDummyFrame++){
            log_d("Capturing dummy frame %i", iDummyFrame);
            fb = esp_camera_fb_get();
            if(!fb) log_e("Camera frame error", false);
            esp_camera_fb_return(fb);
        }
        

        // Save image to SD card
        uint32_t frame_index = device_pref.getFrameIndex() + 1;

        // FIXME: decide which method to use..
        device_pref.setFrameIndex(frame_index);

        // Get the compile date and time as a string
        String compileDate = String(__DATE__) + " " + String(__TIME__);
        // Remove spaces and colons from the compile date and time string
        compileDate.replace(" ", "_");
        compileDate.replace(":", "");

        // Create a filename with the compile date and time string
        String filename = "/data_" + compileDate +"_timelapse_image_anglerfish_" + String(imagesServed);
        
        int stepSize = 10;
        int stepMin = 0;
        int stepMax = 100;
        setLamp(255);
        if (device_pref.getAcquireStack())
            acquireFocusStack(filename, stepSize, stepMin = 0, stepMax = stepMax);
        else
            saveImage(filename, 0);
        imagesServed++;

        // FIXME: we should increase framenumber even if failed - since a corrupted file may lead to issues? 
        device_pref.setFrameIndex(imagesServed);

        // Sleep
        if (timelapseInterval == -1)
            timelapseInterval = 60; // do timelapse every five minutes if not set properly
        Serial.print("Sleeping for ");
        Serial.print(timelapseInterval);
        Serial.println(" s");
        static const uint64_t usPerSec = 1000000; // Conversion factor from microseconds to seconds
        esp_sleep_enable_timer_wakeup(timelapseInterval * usPerSec);
        SD_MMC.end(); // FIXME: may cause issues when file not closed? categoreis: LED/SD-CARD issues

        // After SD Card init? and after the Lens was used?
        // ATTENTIONN: DON'T USE ANY SD-CARD RELATED GPIO!!
        // set a wakeup pin so that we reset the Snow-white deepsleep and turn on the Wifi again: // FIXME: Makes sense?
        // esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 1); //=> GPIO: 4, level: 1
        // Setup interrupt on Touch Pad 3 (GPIO15)
        // touchAttachInterrupt(T3, callbackTouchpad, 40);
        // Configure Touchpad as wakeup source
        // esp_sleep_enable_touchpad_wakeup();
        // Ensure LED is switched off
        pinMode(4, OUTPUT);
        digitalWrite(4, LOW);
        gpio_hold_en(GPIO_NUM_4);
        gpio_deep_sleep_hold_en();
        esp_deep_sleep_start();
        return;
    }
    else
    {
        Serial.println("In livestreaming mode. Connecting to pre-set Wifi");
    }
}