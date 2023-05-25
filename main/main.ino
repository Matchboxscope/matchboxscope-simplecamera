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
#include <SD_MMC.h>
#include "ArduinoJson.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "improv.h"

// camera configuration
#define CAM_NAME "Omniscope"
#define MDNS_NAME "Omniscope"
char default_index[] = "full";
// Primary config, or defaults.
struct station
{
    const char ssid[65];
    const char password[65];
    const bool dhcp;
}; // do no edit

// Pin Mappings
#include "camera_pins.h"

// Camera config structure
camera_config_t config;

// Internal filesystem (SPIFFS)
// used for non-volatile camera settings
#include "storage.h"

//*** Improv
#define MAX_ATTEMPTS_WIFI_CONNECTION 20

uint8_t x_buffer[16];
uint8_t x_position = 0;

int frameWidth = 0;  // Width of the image frame
int frameHeight = 0; // Height of the image frame

// Start with accesspoint mode disabled, wifi setup will activate it if
// no known networks are found, and WIFI_AP_ENABLE has been defined

// IP address, Netmask and Gateway, populated when connected
IPAddress ip;
IPAddress net;
IPAddress gw;
bool is_accesspoint = false;

// Declare external function from app_httpd.cpp
extern void startCameraServer(int hPort, int sPort);

bool onCommandCallback(improv::ImprovCommand cmd);
void getAvailableWifiNetworks();
void set_state(improv::State state);
void onErrorCallback(improv::Error err);
void set_error(improv::Error error);
void send_response(std::vector<uint8_t> &response);

// Names for the Camera
char myName[] = CAM_NAME;
char mdnsName[] = MDNS_NAME;
// Ports for http and stream
int httpPort = 80;
int streamPort = 81;

// settings for ssid/pw if updated from serial
const char *mssid = "omniscope"; // default values
const char *mpassword = "omniscope";

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

// The app and stream URLs
char httpURL[64] = {"Undefined"};
char streamURL[64] = {"Undefined"};

// Counters for info screens and debug
int8_t streamCount = 0;          // Number of currently active streams
unsigned long streamsServed = 0; // Total completed streams
unsigned long imagesServed = 0;  // Total image requests

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

// This will be set to the sensors PID (identifier) during initialisation
// camera_pid_t sensorPID;
int sensorPID;

// Camera module bus communications frequency.
// Originally: config.xclk_freq_mhz = 20000000, but this lead to visual artifacts on many modules.
// See https://github.com/espressif/esp32-camera/issues/150#issuecomment-726473652 et al.
unsigned long xclk = 8;

// initial rotation of the camera image
int myRotation = 0;

// minimal frame duration in ms, effectively 1/maxFPS
int minFrameTime = 0;

bool filesystem = true;

extern bool saveImage(String filename);
// Critical error string; if set during init (camera hardware failure) it
// will be returned for all http requests
String critERR = "";

void initWifi()
{
    /*
     * WIFI-related settings
     */
    // TODO: Try to connect here if credentials are available
    //  load SSID/PW from SPIFFS
    String mssid_tmp = getWifiSSID(SPIFFS);
    String mpassword_tmp = getWifiPW(SPIFFS);

    // try to connect to available Network
    log_d("Try to connect to stored wifi network SSID: %s, PW: %s", mssid_tmp, mpassword_tmp);
    WiFi.begin(mssid_tmp.c_str(), mpassword_tmp.c_str());
    WiFi.setSleep(false);
    log_d("Initi Wifi works");

    int nTrialWifiConnect = 0;
    int nTrialWifiConnectMax = 30;
    int tWaitWifiConnect = 500;
    while (WiFi.status() != WL_CONNECTED)
    {
        log_d("Connecting to Wi-Fi...");
        nTrialWifiConnect++;
        delay(tWaitWifiConnect);

        if (nTrialWifiConnect > nTrialWifiConnectMax)
        {
            WiFi.disconnect(); // (resets the WiFi scan)
            WiFi.mode(WIFI_AP);
            ESP.restart();
        }
    }
}

// Notification LED
void flashLED(int flashtime)
{
#ifdef CAMERA_MODEL_AI_THINKER
    digitalWrite(LED_PIN, LED_ON);  // On at full power.
    delay(flashtime);               // delay
    digitalWrite(LED_PIN, LED_OFF); // turn Off
#endif
}

void blink_led(int d, int times)
{
    for (int j = 0; j < times; j++)
    {
        flashLED(d);
        delay(d);
    }
}

void calcURLs()
{
    // Note AP details
    if (is_accesspoint)
        ip = WiFi.softAPIP();
    else
        ip = WiFi.localIP();
    net = WiFi.subnetMask();
    gw = WiFi.gatewayIP();

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

bool StartCamera()
{
    bool initSuccess = false;
#if defined(CAMERA_MODEL_AI_THINKER)

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
    config.fb_location = CAMERA_FB_IN_PSRAM;

    // Low(ish) default framesize and quality
    config.frame_size = FRAMESIZE_SVGA;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.jpeg_quality = 12;
    config.fb_count = 2;
#elif defined(CAMERA_MODEL_XIAO)
    Serial.println("Xiao Sense Camera initialization");
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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    if (config.pixel_format == PIXFORMAT_JPEG)
    {
        if (psramFound())
        {
            config.jpeg_quality = 10;
            config.fb_count = 2;
            config.grab_mode = CAMERA_GRAB_LATEST;
        }
        else
        {
            // Limit the frame size when PSRAM is not available
            config.frame_size = FRAMESIZE_SVGA;
            config.fb_location = CAMERA_FB_IN_DRAM;
        }
    }
    else
    {
        // Best option for face detection/recognition
        config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
        config.fb_count = 2;
#endif
    }
#endif

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
        initSuccess = false;

        // try it again
        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK)
        {
            log_d("Second attempt worked..");
        }
        else
        {
            log_d("Second attempt to initialize failed too, resetting SPIFFS; probably something went wrong with a weird combination of camera settings");
            removePrefs(SPIFFS);

            ESP.restart();
        }
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
        initSuccess = true;
    }

    // get mean intensities
    camera_fb_t *fb = NULL;
    for (int iDummyFrame = 0; iDummyFrame < 5; iDummyFrame++)
    {
        // FIXME: Look at the buffer for the camera => flush vs. return
        log_d("Capturing dummy frame %i", iDummyFrame);
        fb = esp_camera_fb_get();
        if (!fb)
            log_e("Camera frame error", false);
        esp_camera_fb_return(fb);

        int mean_intensity = get_mean_intensity(fb);

        Serial.print("Mean intensity: ");
        Serial.println(mean_intensity);
    }
    // We now have camera with default init
    return initSuccess;
}

int get_mean_intensity(camera_fb_t *fb)
{
    uint8_t *pixels = fb->buf;
    int num_pixels = fb->width * fb->height;

    float sum_intensity = 0.;
    for (int i = 0; i < num_pixels; i++)
    {
        sum_intensity += pixels[i];
    }
    int mean_intensity = sum_intensity / num_pixels;

    return mean_intensity;
}

#define WIFI_SSID "Blynk"
#define WIFI_PASS "12345678"
void setup()
{
    // Start Serial
    Serial.begin(115200);

    delay(500);
    Serial.println("...............");
    // start wifi AP or connect to AP
    initWifi();

    // propagate URLs to GUI
    calcURLs();

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

    // Start the SPIFFS filesystem before we initialise the camera
    filesystemStart();
    delay(500); // a short delay to let spi bus settle after SPIFFS init

    // Start (init) the camera
    bool camInitSuccess = StartCamera();

    // FIXME: if not successfully initialized, either the cam is broken or we need to fully power it off..not possible yet??
    if (not camInitSuccess)
    {
        log_e("Camera failed to initialize %i", camInitSuccess);
        ESP.restart();
    }

    // actions done on first boot
    bool isFirstRun = isFirstBoot();
    if (isFirstRun)
    {
        // disable any Anglerfish-related settings
        log_d("Remove SPIFFS");
        removePrefs(SPIFFS);

        // set the default settings
        log_d("Write Prefs to SPIFFS");
        writePrefsToSSpiffs(SPIFFS);

        // adjust compiled date to ensure next boot won't be detected as first boot
        log_d("Set compiled date");
        setCompiledDate(SPIFFS);
    }
    // declare LED PIN
    pinMode(LED_PIN, OUTPUT);

    // initialize SD card before LED!!
    // We initialize SD_MMC here rather than in setup() because SD_MMC needs to reset the light pin
    // with a different pin mode.
    // 1-bit mode as suggested here:https://dr-mntn.net/2021/02/using-the-sd-card-in-1-bit-mode-on-the-esp32-cam-from-ai-thinker

    // loading previous settings
    imagesServed = getFrameIndex(SPIFFS);

    // Now load and apply any saved preferences
    if (filesystem)
    {
        delay(200); // a short delay to let spi bus settle after camera init
        loadSpiffsToPrefs(SPIFFS);
        Serial.println("internal files System found and mounted");
    }
    else
    {
        Serial.println("No Internal Filesystem, cannot load or save preferences");
    }

    // MDNS Config -- note that if OTA is NOT enabled this needs prior steps!
    MDNS.addService("http", "tcp", 80);
    Serial.println("Added HTTP service to MDNS server");

    // Start the camera server
    startCameraServer(httpPort, streamPort);

    if (critERR.length() == 0)
    {
        Serial.printf("\r\nCamera Ready!\r\nUse '%s' to connect\r\n", httpURL);
        Serial.printf("Stream viewer available at '%sview'\r\n", streamURL);
        Serial.printf("Raw stream URL is '%s'\r\n", streamURL);
    }
    else
    {
        Serial.printf("\r\nCamera unavailable due to initialisation errors.\r\n\r\n");
    }
}

int WIFI_WATCHDOG = 15000;
void loop()
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
        initWifi();
    }
}
