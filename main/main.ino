#include <WiFiUdp.h>
#include <esp_camera.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <DNSServer.h>
#include "parsebytes.h"
#include "time.h"
#include <ESPmDNS.h>
#include <SD_MMC.h>
#include "ArduinoJson.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "improv.h"
#include "spinlock.h"
#include <base64.h>
#include <Adafruit_NeoPixel.h>
#include <NTPClient.h>
#include <HTTPClient.h>
#include "HTTPClientESP32.h"
#include <ArduinoWebsockets.h>
#include "esp_wifi.h"
#include "FS.h"
#include "SPI.h"
#include "driver/i2c.h"
#include <SD.h>
#include "driver/rtc_io.h"

// Internal filesystem (SPIFFS) used for non-volatile camera settings
#include "storage.h"
#include "version.h"
#include "camera_pins.h"

// Constants and global variables
#define CAM_NAME "UC2xSeeed"
#define MDNS_NAME "UC2xSeeed"
#define HTTP_PORT 80
#define STREAM_PORT 81

// WiFi and Server settings
String baseURL = "NONE";
HTTPClientESP32 httpClient(baseURL);
IPAddress ip, net, gw;
bool is_accesspoint = false;
String uniqueID = "0";

// Camera configuration
camera_config_t config;
int sensorPID;
unsigned long xclk = 20;
int myRotation = 0;
int minFrameTime = 0;
int timelapseInterval = -1;
bool isAutofocusMotorized = true;
static uint64_t t_old = millis();
static uint64_t t_blink_old = millis();
bool sendToGithubFlag = false;
uint32_t frameIndex = 0;

// Lamp and LED settings
int lampVal = 0;
bool autoLamp = false;
int pwmVal = 0;
bool BUSY_SET_LED = false;
int lampChannel = 1;
int pwmChannel = 2;
int pwmfreq = 50000;
int pwmresolution = 9;
const int pwmMax = pow(2, pwmresolution) - 1;

// Filesystem
bool filesystem = true;
boolean sdInitialized = false;

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

// Names for the Camera
char myName[] = CAM_NAME;
char mdnsName[] = MDNS_NAME;

// Ports for http and stream
int httpPort = 80;
int streamPort = 81;

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

// Camera and Network URLs
char httpURL[64] = {"Undefined"};
char streamURL[64] = {"Undefined"};

// Timelapse settings
const char *ntpServer = "";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;
int autofocus_min = -500;
int autofocus_max = 500;
int autofocus_stepsize = 25;

// Error handling
String critERR = "";

// Function declarations
void initWifi(String countryCode);
void setLamp(int newVal);
void setNeopixel(int newVal);
void setPWM(int newVal);
String getThreeDigitID();
void onNewStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);
void debugWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
void calcURLs();
bool StartCamera();
int get_mean_intensity(camera_fb_t *fb);
void grabRawFrameBase64();
void moveFocusRelative(int steps, bool handleEnable = true);
void scanConnectedDevices();
bool checkDeviceStatus(String ip);
void setup();
void loop();
void loadSettings();
int imagesServed = 0;
extern bool saveImage(String filename, int pwmVal);
extern char *GITHUB_TOKEN;
bool isTimelapse = false;

void initSDCard();
void startCameraServer(int hPort, int sPort);
int autoFocus(int minPos, int maxPos, int focusStep);

void setup()
{
    Serial.begin(115200);

    // Warn if no PSRAM is detected
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
    delay(500);

    // Initialize the camera
    if (!StartCamera())
    {
        log_e("Camera failed to initialize");
        ESP.restart();
    }

    // Initialize SD card
    initSDCard();

    // Load settings (e.g. if timelapse was enabled)
    loadSettings();

    // Initialize WiFi // change with WIFI:CN
    String countryCode = getWifiCountry();
    log_i("Country code: %s", countryCode.c_str());
    initWifi(countryCode);

    // Calculate URLs
    calcURLs();

    // Start the camera server
    startCameraServer(HTTP_PORT, STREAM_PORT);

    // Print ready message
    if (critERR.length() == 0)
    {
        Serial.printf("\r\nCamera Ready!\r\nUse '%s' to connect\r\n", httpURL);
        Serial.printf("Stream viewer available at '%sview'\r\n", streamURL);
        Serial.printf("Raw stream URL is '%s'\r\n", streamURL);
    }
    else
    {
        Serial.printf("\r\nCamera unavailable due to initialization errors.\r\n\r\n");
    }
}

void loop()
{
    // If timelapse is enabled and sd card is initialized toggle led every second
    if (sdInitialized && isTimelapse && timelapseInterval > 0 && ((millis() - t_blink_old) > 1000))
    {
        digitalWrite(13, !digitalRead(13));
        t_blink_old = millis();
    }
    // Timelapse Imaging
    if (sdInitialized && isTimelapse && timelapseInterval > 0 && ((millis() - t_old) > (1000 * timelapseInterval)))
    {
        log_i("Taking an Image");
        writePrefsToSSpiffs(SPIFFS);
        t_old = millis();
        frameIndex = getFrameIndex();

        // Save image
        String filename = "/timelapse_image_scope_" + String(uniqueID) + "_" + String(millis()) + "_" + String(imagesServed);
        imagesServed++;
        log_d("Store single image");
        int pwmVal = getPWMVal(SPIFFS);
        saveImage(filename, pwmVal);

        // Set default lamp value for streaming
        setLamp(lampVal);

        // Increment frame number
        setFrameIndex(frameIndex + 1);
    }

    // Check for input on Serial
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("WIFI:CN"))
        {
            Serial.println("Changing WiFi country to China (CN)...");
            // save the new country code to the preferences
            setWifiCountry("CN");
            initWifi("CN");
            //
        }
        else
        {
            Serial.println("Unknown command.");
            Serial.println("Changing WiFi country back to Default (US)...");
            setWifiCountry("US");
            initWifi("US");
        }
    }
}

// Function Definitions
void initWifi(String countryCode)
{
    String ssid = "UC2xSeeed-" + getThreeDigitID();
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);

    // Set country code
    if (countryCode == "CN")
    {
        log_i("Setting WiFi country to China (CN)");
        wifi_country_t my_country = {
            cc : "CN",
            schan : 1,
            nchan : 14,
            max_tx_power : 20
        };
        esp_wifi_set_country(&my_country);
    }
    else
    {
        log_i("Setting WiFi country to US");
        wifi_country_t my_country = {
            cc : "US",
            schan : 1,
            nchan : 11,
            max_tx_power : 20,
            policy : WIFI_COUNTRY_POLICY_AUTO
        };
        esp_wifi_set_country(&my_country);
    }

    WiFi.softAP(ssid.c_str(), "");

    WiFi.onEvent(onNewStationConnected, ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);
    WiFi.onEvent(debugWifiEvent, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
    WiFi.onEvent(debugWifiEvent, ARDUINO_EVENT_WIFI_AP_START);
    WiFi.onEvent(debugWifiEvent, ARDUINO_EVENT_WIFI_AP_STOP);

    Serial.println("AP started");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    is_accesspoint = true;
}

void onNewStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("New device connected to the AP");
    scanConnectedDevices();
    httpClient.setBaseURL(baseURL);
}

void debugWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.print("WiFi Event: ");
    Serial.println(event);
    Serial.print("Info: ");
    // Serial.println(info);
}

void calcURLs()
{
    if (is_accesspoint)
        ip = WiFi.softAPIP();
    else
        ip = WiFi.localIP();
    net = WiFi.subnetMask();
    gw = WiFi.gatewayIP();

    if (HTTP_PORT != 80)
    {
        sprintf(httpURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], HTTP_PORT);
    }
    else
    {
        sprintf(httpURL, "http://%d.%d.%d.%d/", ip[0], ip[1], ip[2], ip[3]);
    }
    sprintf(streamURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], STREAM_PORT);
}

bool StartCamera()
{
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

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.println("Camera initialization failed");
        return false;
    }

    Serial.println("Camera init succeeded");
    sensor_t *s = esp_camera_sensor_get();
    sensorPID = s->id.PID;

    // OV3660 initial sensors are flipped vertically and colors are a bit saturated
    if (sensorPID == OV3660_PID)
    {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }

    // Additional sensor settings can be added here

    // Capture and discard the first few frames to clear any initial buffer
    camera_fb_t *fb = NULL;
    for (int iDummyFrame = 0; iDummyFrame < 2; iDummyFrame++)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            log_e("Camera frame error", false);
        }
        esp_camera_fb_return(fb);
    }
    return true;
}

void setLamp(int newVal)
{
    int current = newVal;
    httpClient.pwm_act(2, current);
    Serial.print("Current: ");
    Serial.println(newVal);
}

void setNeopixel(int newVal)
{
    httpClient.ledarr_act(newVal, newVal, newVal);
    log_d("Setting Neopixel to %d", newVal);
}

void setPWM(int newVal)
{
    int current = newVal;
    httpClient.pwm_act(1, current);
    Serial.print("Current: ");
    Serial.println(newVal);
}

String getThreeDigitID()
{
    uint8_t mac_address[6];
    String s;
    WiFi.macAddress(mac_address);
    for (byte i = 0; i < 6; ++i)
    {
        char buf[3];
        sprintf(buf, "%02X", mac_address[i]);
        s += buf;
    }
    return s;
}

void initSDCard()
{
    if (!SD.begin(21))
    {
        Serial.println("SD Card Mount on XIAO Failed");
        sdInitialized = false;
    }
    else
    {
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE)
        {
            Serial.println("No SD card attached");
            sdInitialized = false;
        }
        else
        {
            sdInitialized = true;
            Serial.println("SD Card Mounted");
            Serial.print("SD Card Type: ");
            if (cardType == CARD_MMC)
            {
                Serial.println("MMC");
            }
            else if (cardType == CARD_SD)
            {
                Serial.println("SDSC");
            }
            else if (cardType == CARD_SDHC)
            {
                Serial.println("SDHC");
            }
            else
            {
                Serial.println("UNKNOWN");
            }
            // indicate that the SD card is available
            ledcWrite(lampChannel, 255);
        }
    }
}

void loadSettings()
{
    imagesServed = getFrameIndex();
    timelapseInterval = getTimelapseInterval(SPIFFS);
    isTimelapse = getisTimelapse();
    ledcSetup(lampChannel, pwmfreq, pwmresolution);
    ledcAttachPin(LAMP_PIN, lampChannel);
    log_d("LED pin: %d", LAMP_PIN);
    if (autoLamp)
        setLamp(0);
    else
        setLamp(lampVal);

    if (PWM_PIN >= 0)
    {
        pinMode(PWM_PIN, OUTPUT);
        log_d("PWM pin: %d", PWM_PIN);
        ledcSetup(pwmChannel, pwmfreq, pwmresolution);
        ledcAttachPin(PWM_PIN, pwmChannel);
        setPWM(255);
        delay(30);
        setPWM(0);
    }

    if (filesystem)
    {
        delay(200);
        loadSpiffsToPrefs(SPIFFS);
        Serial.println("Internal filesystem found and mounted");
    }
    else
    {
        Serial.println("No Internal Filesystem, cannot load or save preferences");
    }
}

void grabRawFrameBase64()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || fb->format != PIXFORMAT_GRAYSCALE)
    {
        Serial.println("Failed to capture image");
        ESP.restart();
    }
    else
    {
        for (int i = 0; i < 10; i++)
        {
            fb->buf[i] = i % 2;
        }
        if (0)
        {
            Serial.write(fb->buf, fb->len);
        }
        else
        {
            String encoded = base64::encode((uint8_t *)fb->buf, fb->len);
            Serial.println(encoded);
        }
    }
    esp_camera_fb_return(fb);
}

void moveFocusRelative(int steps, bool handleEnable)
{
    int stepperid = 1;
    int position = steps;
    int speed = 1000;
    int isabs = 0;
    int isaccel = 1;
    httpClient.motor_act(stepperid, position, speed, isabs, isaccel);
}

void scanConnectedDevices()
{
    Serial.println("Checking connected devices...");
    wifi_sta_list_t wifi_sta_list;
    tcpip_adapter_sta_list_t adapter_sta_list;
    memset(&wifi_sta_list, 0, sizeof(wifi_sta_list_t));
    memset(&adapter_sta_list, 0, sizeof(tcpip_adapter_sta_list_t));

    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

    for (int i = 0; i < adapter_sta_list.num; i++)
    {
        tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
        String ip = IPAddress(station.ip.addr).toString();
        Serial.print("Device IP: ");
        Serial.println(ip);

        if (checkDeviceStatus(ip))
        {
            Serial.println("Device returned UC2");
            baseURL = "http://" + ip;
        }
        else
        {
            Serial.println("Device did not return UC2");
        }
    }
}

bool checkDeviceStatus(String ip)
{
    HTTPClient http;
    http.setTimeout(500);
    String url = "http://" + ip + "/state_get";
    Serial.println("Checking if we are connected to a UC2 Device");
    for (int iTrial = 0; iTrial < 3; iTrial++)
    {
        http.begin(url);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK)
        {
            String payload = http.getString();
            Serial.println(payload);
            http.end();
            return payload.indexOf("UC2") != -1;
        }
        else
        {
            Serial.println("Failed to connect to device");
            http.end();
        }
    }
    return false;
}
