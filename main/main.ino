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
bool isStack = false;
bool isTimelapseGeneral = false;
int minFrameTime = 0;
int timelapseInterval = -1;
int autofocusInterval = 0;
bool isAutofocusMotorized = true;
static uint64_t t_old = millis();
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
void initWifi();
void setLamp(int newVal);
void setNeopixel(int newVal);
void setPWM(int newVal);
String getThreeDigitID();
void onNewStationConnected(WiFiEvent_t event, WiFiEventInfo_t info);
void calcURLs();
bool StartCamera();
void saveCapturedImageGithubTask(void *pvParameters);
int get_mean_intensity(camera_fb_t *fb);
void acquireFocusStack(String filename, int stepSize = 10, int stepMin = 0, int stepMax = 100);
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
void initSDCard();
void startCameraServer(int hPort, int sPort);
int autoFocus(int minPos, int maxPos, int focusStep);

void setup() {
    Serial.begin(115200);

    // Warn if no PSRAM is detected
    if (!psramFound()) {
        Serial.println("\r\nFatal Error; Halting");
        while (true) {
            Serial.println("No PSRAM found; camera cannot be initialised: Please check the board config for your module.");
            delay(5000);
        }
    }

    // Start the SPIFFS filesystem before we initialise the camera
    filesystemStart();
    delay(500);

    // Initialize the camera
    if (!StartCamera()) {
        log_e("Camera failed to initialize");
        ESP.restart();
    }

    // Initialize SD card
    initSDCard();

    // Load settings
    loadSettings();

    // Initialize WiFi
    initWifi();

    // Calculate URLs
    calcURLs();

    // Start the camera server
    startCameraServer(HTTP_PORT, STREAM_PORT);

    // Start the GitHub upload task
    xTaskCreatePinnedToCore(
        saveCapturedImageGithubTask, "saveCapturedImageGithubTask", 8192, NULL, 1, NULL, 0);

    // Print ready message
    if (critERR.length() == 0) {
        Serial.printf("\r\nCamera Ready!\r\nUse '%s' to connect\r\n", httpURL);
        Serial.printf("Stream viewer available at '%sview'\r\n", streamURL);
        Serial.printf("Raw stream URL is '%s'\r\n", streamURL);
    } else {
        Serial.printf("\r\nCamera unavailable due to initialization errors.\r\n\r\n");
    }
}

void loop() {
    // Timelapse Imaging
    if (isTimelapseGeneral && timelapseInterval > 0 && ((millis() - t_old) > (1000 * timelapseInterval))) {
        writePrefsToSSpiffs(SPIFFS);
        t_old = millis();
        frameIndex = getFrameIndex(SPIFFS) + 1;

        // Perform autofocus every n-times
        if (frameIndex % autofocusInterval == 0 && autofocusInterval > 0) {
            log_d("Performing autofocus");
            autoFocus(autofocus_min, autofocus_max, autofocus_stepsize);
        }

        // Save image
        String filename = "/timelapse_image_scope_" + String(uniqueID) + "_" + String(millis()) + "_" + String(imagesServed);
        if (getAcquireStack(SPIFFS)) {
            log_d("Acquiring stack");
            imagesServed++;
            acquireFocusStack(filename, 10);
        } else {
            imagesServed++;
            log_d("Store single image");
            int pwmVal = getPWMVal(SPIFFS);
            saveImage(filename, pwmVal);
        }

        // Set default lamp value for streaming
        setLamp(lampVal);

        // Increment frame number
        setFrameIndex(SPIFFS, frameIndex);
    }
}

// Function Definitions

void initWifi() {
    String ssid = "UC2xSeeed-" + getThreeDigitID();
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), "");

    WiFi.onEvent(onNewStationConnected, ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);

    Serial.println("AP started");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    is_accesspoint = true;
}

void onNewStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("New device connected to the AP");
    scanConnectedDevices();
    httpClient.setBaseURL(baseURL);
}

void calcURLs() {
    if (is_accesspoint)
        ip = WiFi.softAPIP();
    else
        ip = WiFi.localIP();
    net = WiFi.subnetMask();
    gw = WiFi.gatewayIP();

    if (HTTP_PORT != 80) {
        sprintf(httpURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], HTTP_PORT);
    } else {
        sprintf(httpURL, "http://%d.%d.%d.%d/", ip[0], ip[1], ip[2], ip[3]);
    }
    sprintf(streamURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], STREAM_PORT);
}

bool StartCamera() {
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
    if (err != ESP_OK) {
        Serial.println("Camera initialization failed");
        return false;
    }

    Serial.println("Camera init succeeded");
    sensor_t *s = esp_camera_sensor_get();
    sensorPID = s->id.PID;

    // OV3660 initial sensors are flipped vertically and colors are a bit saturated
    if (sensorPID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    }

    // Additional sensor settings can be added here

    // Capture and discard the first few frames to clear any initial buffer
    camera_fb_t *fb = NULL;
    for (int iDummyFrame = 0; iDummyFrame < 2; iDummyFrame++) {
        fb = esp_camera_fb_get();
        if (!fb) {
            log_e("Camera frame error", false);
        }
        esp_camera_fb_return(fb);
    }
    return true;
}

void saveCapturedImageGithub()
{
    /* This works in MAC
  curl -L \
    -X PUT \
    -H "Accept: application/vnd.github+json" \
    -H "Authorization: Bearer TOKEN"\
    -H "X-GitHub-Api-Version: 2022-11-28" \
    https://api.github.com/repos/matchboxscope/matchboxscope-gallery/contents/test2.jpg \
    -d '{"message":"my commit message","committer":{"name":"Monalisa Octocat","email":"octocat@github.com"},"content":"BASE64=="}'


    */

    Serial.println("Performing Saving Capture for Github in Background");
    // choose smaller pixel number
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA|QXGA(ov3660)]);
    if (autoLamp && (lampVal != -1))
    {
        setLamp(lampVal);
        delay(75); // coupled with the status led flash this gives ~150ms for lamp to settle.
    }

    // capture image
    camera_fb_t *fb = NULL;
    for (int i = 0; i < 5; i++)
    {
        // warmup and wb settling
        fb = esp_camera_fb_get();
        esp_camera_fb_return(fb);
    }

    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        return;
    }

    if (autoLamp && (lampVal != -1))
    {
        setLamp(0);
    }

    // Encode the image in base64 format
    Serial.println("Encode image image");
    String base64data = base64::encode(fb->buf, fb->len); // convert buffer to base64
    esp_camera_fb_return(fb);

    // formulate the JSON payload
    const char *token = GITHUB_TOKEN;
    const char *owner = "matchboxscope";
    const char *repo = "matchboxscope-gallery";
    const char *message = "Uploading image from ESP32";
    const char *committer_name = "matchboxscope-bot";
    const char *committer_email = "matchboxscope@gmail.com";
    const char *text_mime_type = "text/plain"; // Set the correct MIME type for your text
    // const char *text_base64_data = "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";

    // Construct the JSON payload
    DynamicJsonDocument doc(16384);
    JsonObject payload = doc.to<JsonObject>();
    payload["message"] = message;
    JsonObject committer = payload.createNestedObject("committer");
    committer["name"] = committer_name;
    committer["email"] = committer_email;
    payload["content"] = base64data.c_str(); // text_base64_data;
    payload["content_type"] = text_mime_type;

    // Serialize the JSON payload to a string
    String payload_string;
    serializeJson(payload, payload_string);

    // Construct a random URL
    uint8_t mac[6];
    WiFi.macAddress(mac);

    // Generate a random number
    int randNum = random(10000);

    // Create the filename string
    char filename[32];
    sprintf(filename, "%02X%02X%02X%02X%02X%02X_%d.jpg", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], randNum);
    ;
    String url = "https://api.github.com/repos/matchboxscope/matchboxscope-gallery/contents/" + String(filename);
    log_d("URL: %s", url);

    // Send the request
    HTTPClient http;
    http.begin(url);
    http.addHeader("Accept", "application/vnd.github+json");
    http.addHeader("Authorization", "Bearer " + String(token));
    http.addHeader("X-GitHub-Api-Version", "2022-11-28");
    http.addHeader("Content-Type", "application/json");
    int http_code = http.sendRequest("PUT", payload_string);

    // Print the response
    /*
    Serial.println(payload);
    Serial.println(payload_string);
    Serial.println(http.getString());
    */
    payload.clear();
    http.end();

    // reset old settings
    // s->set_framesize(s, FRAMESIZE_SVGA);
}

void saveCapturedImageGithubTask(void *pvParameters) {
    while (true) {
        if (sendToGithubFlag) {
            sendToGithubFlag = false;
            saveCapturedImageGithub();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setLamp(int newVal) {
    int current = newVal;
    httpClient.pwm_act(2, current);
    Serial.print("Current: ");
    Serial.println(newVal);
}

void setNeopixel(int newVal) {
    httpClient.ledarr_act(newVal, newVal, newVal);
    log_d("Setting Neopixel to %d", newVal);
}

void setPWM(int newVal) {
    int current = newVal;
    httpClient.pwm_act(1, current);
    Serial.print("Current: ");
    Serial.println(newVal);
}

String getThreeDigitID() {
    uint8_t mac_address[6];
    String s;
    WiFi.macAddress(mac_address);
    for (byte i = 0; i < 6; ++i) {
        char buf[3];
        sprintf(buf, "%02X", mac_address[i]);
        s += buf;
    }
    return s;
}

void initSDCard() {
    if (!SD.begin(21)) {
        Serial.println("SD Card Mount on XIAO Failed");
        sdInitialized = false;
    } else {
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE) {
            Serial.println("No SD card attached");
            sdInitialized = false;
        } else {
            sdInitialized = true;
            Serial.println("SD Card Mounted");
            Serial.print("SD Card Type: ");
            if (cardType == CARD_MMC) {
                Serial.println("MMC");
            } else if (cardType == CARD_SD) {
                Serial.println("SDSC");
            } else if (cardType == CARD_SDHC) {
                Serial.println("SDHC");
            } else {
                Serial.println("UNKNOWN");
            }
        }
    }
}

void loadSettings() {
    imagesServed = getFrameIndex(SPIFFS);
    timelapseInterval = getTimelapseInterval(SPIFFS);
    autofocusInterval = getAutofocusInterval(SPIFFS);
    isTimelapseGeneral = getIsTimelapseGeneral(SPIFFS);
    ledcSetup(lampChannel, pwmfreq, pwmresolution);
    ledcAttachPin(LAMP_PIN, lampChannel);
    log_d("LED pin: %d", LAMP_PIN);
    if (autoLamp) setLamp(0);
    else setLamp(lampVal);

    if (PWM_PIN >= 0) {
        pinMode(PWM_PIN, OUTPUT);
        log_d("PWM pin: %d", PWM_PIN);
        ledcSetup(pwmChannel, pwmfreq, pwmresolution);
        ledcAttachPin(PWM_PIN, pwmChannel);
        setPWM(255);
        delay(30);
        setPWM(0);
    }

    if (filesystem) {
        delay(200);
        loadSpiffsToPrefs(SPIFFS);
        Serial.println("Internal filesystem found and mounted");
    } else {
        Serial.println("No Internal Filesystem, cannot load or save preferences");
    }
}

void acquireFocusStack(String filename, int stepSize, int stepMin, int stepMax) {
    for (int iFocus = stepMin; iFocus < stepMax; iFocus += stepSize) {
        saveImage(filename + String(imagesServed) + "_z" + String(iFocus), iFocus);
    }
}

void grabRawFrameBase64() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || fb->format != PIXFORMAT_GRAYSCALE) {
        Serial.println("Failed to capture image");
        ESP.restart();
    } else {
        for (int i = 0; i < 10; i++) {
            fb->buf[i] = i % 2;
        }
        if (0) {
            Serial.write(fb->buf, fb->len);
        } else {
            String encoded = base64::encode((uint8_t *)fb->buf, fb->len);
            Serial.println(encoded);
        }
    }
    esp_camera_fb_return(fb);
}

void moveFocusRelative(int steps, bool handleEnable) {
    int stepperid = 1;
    int position = steps;
    int speed = 1000;
    int isabs = 0;
    int isaccel = 1;
    httpClient.motor_act(stepperid, position, speed, isabs, isaccel);
}

void scanConnectedDevices() {
    Serial.println("Checking connected devices...");
    wifi_sta_list_t wifi_sta_list;
    tcpip_adapter_sta_list_t adapter_sta_list;
    memset(&wifi_sta_list, 0, sizeof(wifi_sta_list_t));
    memset(&adapter_sta_list, 0, sizeof(tcpip_adapter_sta_list_t));

    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

    for (int i = 0; i < adapter_sta_list.num; i++) {
        tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
        String ip = IPAddress(station.ip.addr).toString();
        Serial.print("Device IP: ");
        Serial.println(ip);

        if (checkDeviceStatus(ip)) {
            Serial.println("Device returned UC2");
            baseURL = "http://" + ip;
        } else {
            Serial.println("Device did not return UC2");
        }
    }
}

bool checkDeviceStatus(String ip) {
    HTTPClient http;
    http.setTimeout(500);
    String url = "http://" + ip + "/state_get";
    Serial.println("Checking if we are connected to a UC2 Device");
    for (int iTrial = 0; iTrial < 3; iTrial++) {
        http.begin(url);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            Serial.println(payload);
            http.end();
            return payload.indexOf("UC2") != -1;
        } else {
            Serial.println("Failed to connect to device");
            http.end();
        }
    }
    return false;
}
