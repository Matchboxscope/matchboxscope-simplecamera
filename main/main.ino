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
#include "spinlock.h"
#include "anglerfishcamsettings.h"
#include <base64.h>
#include <Adafruit_NeoPixel.h>

// Upstream version string
#include "version.h"
// Pin Mappings
#include "camera_pins.h"

//#define ACCELSTEPPER

#ifdef CAMERA_MODEL_XIAO
#include <AccelStepper.h>
#ifdef ACCELSTEPPER
AccelStepper motor(1, STEPPER_MOTOR_STEP, STEPPER_MOTOR_DIR);
#else
AccelStepper motor(8, motorPin1, motorPin3, motorPin2, motorPin4);
#endif
#endif

// camera configuration
#define CAM_NAME "Matchboxscope"
#define MDNS_NAME "Matchboxscope"
// camera module
// #define CAMERA_MODEL_AI_THINKER
// #define CAMERA_MODEL_XIAO

// Primary config, or defaults.
struct station
{
  const char ssid[65];
  const char password[65];
  const bool dhcp;
}; // do no edit

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

// Sketch Info
int sketchSize;
int sketchSpace;
String sketchMD5;

bool isMotorRunningFixedPosition = true;

String uniqueID = "0";

// Start with accesspoint mode disabled, wifi setup will activate it if
// no known networks are found, and WIFI_AP_ENABLE has been defined

// IP address, Netmask and Gateway, populated when connected
IPAddress ip;
IPAddress net;
IPAddress gw;
bool is_accesspoint = false;

// Declare external function from app_httpd.cpp
extern void startCameraServer(int hPort, int sPort);
extern void serialDump();
extern void saveCapturedImageGithub();

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
const char *mssid = "Blynk"; // default values
const char *mpassword = "12345678";

bool isSerialTransferMode = false;

#ifdef NEOPIXEL_PIN
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

#define WIFI_WATCHDOG 15000

// Select between full and simple index as the default.
char default_index[] = "full";

// SD Card
boolean sdInitialized = false;

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
#if defined(CAMERA_MODEL_AI_THINKER)
unsigned long xclk = 8;
#elif defined(CAMERA_MODEL_XIAO)
unsigned long xclk = 20;
#endif

// initial rotation of the camera image
int myRotation = 0;
bool isStack = false;
bool isTimelapseAnglerfish = false;
bool isAnglerfishExposureBracketing = true;// false;
bool isTimelapseGeneral = false;

// minimal frame duration in ms, effectively 1/maxFPS
int minFrameTime = 0;

// Timelapse
int timelapseInterval = -1;
int autofocusInterval = 0;
bool isAutofocusMotorized = true;
static uint64_t t_old = millis();
bool sendToGithubFlag = false;
uint32_t frameIndex = 0;

// Illumination LAMP and status LED
int lampVal = 0;       // default to off
bool autoLamp = false; // Automatic lamp (auto on while camera running)
int pwmVal = 0;        // default no-value
bool BUSY_SET_LED = false;

#if defined(CAMERA_MODEL_AI_THINKER)
int lampChannel = 7; // a free PWM channel (some channels used by camera)
int pwmChannel = 5;
#elif defined(CAMERA_MODEL_XIAO)
int lampChannel = 1; // a free PWM channel (some channels used by camera)
int pwmChannel = 2;
#endif
int pwmfreq = 50000;   // 50K pwm frequency
int pwmresolution = 9; // duty cycle bit range
const int pwmMax = pow(2, pwmresolution) - 1;

bool filesystem = true;

bool otaEnabled = true;
char otaPassword[] = "";

const char *ntpServer = "";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;
int autofocus_min = -500;
int autofocus_max = 500;
int autofocus_stepsize = 25;
extern int autoFocus(int minPos = -300, int maxPos = 300, int focusStep = 25);
extern bool saveImage(String filename, int pwmVal);
// Critical error string; if set during init (camera hardware failure) it
// will be returned for all http requests
String critERR = "";

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

// Neopixel Control
void setNeopixel(int newVal)
{
#ifdef NEOPIXEL_PIN
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(newVal, newVal, newVal));
  }
  log_d("Setting Neopixel to %d", newVal);
  pixels.show(); // Send the updated pixel colors to the hardware.
#endif
}

// PWM Control
void setPWM(int newVal)
{
  if (newVal != -1 and PWM_PIN >= 0)
  {
    // Apply a logarithmic function to the scale.
    int current = newVal;
    ledcWrite(pwmChannel, current);
    Serial.print("Current: ");
    Serial.print(newVal);
    Serial.print("%, pwm = ");
    Serial.println(current);
  }
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

void initWifi()
{
  /*
   * WIFI-related settings
   */
  // TODO: Try to connect here if credentials are available
  //  load SSID/PW from SPIFFS

  int n = WiFi.scanNetworks();
  Serial.println("Scan completed.");
  if (n == 0)
  {
    Serial.println("No network found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks are discovered");
    for (int i = 0; i < n; ++i)
    {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.println(")");
      delay(10);
    }
  }
  Serial.println("");

  // Scan for Wi-Fi networks
  if (0)
  {
    int networkCount = WiFi.scanNetworks();

    if (networkCount == 0)
    {
      Serial.println("No Wi-Fi networks found");
    }
    else
    {
      Serial.print("Found ");
      Serial.print(networkCount);
      Serial.println(" Wi-Fi networks:");

      for (int i = 0; i < networkCount; ++i)
      {
        String ssid = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);
        int encryptionType = WiFi.encryptionType(i);

        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(ssid);
        Serial.print(" Signal strength: ");
        Serial.print(rssi);
        Serial.println(" dBm");
      }
    }
  }
  String mssid_tmp = getWifiSSID(SPIFFS);
  String mpassword_tmp = getWifiPW(SPIFFS);

  uniqueID = getThreeDigitID();
  String ssid = "Matchboxscope-" + uniqueID;

  // if mssid_tmp is "" open access point
  if (mssid_tmp == "")
  {
    // open Access Point with random ID
    WiFi.disconnect(); // (resets the WiFi scan)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), "");
    /*Doesn'T make sense to do this 100x
    for (int iBlink = 0; iBlink < uniqueID; iBlink++)
    {
        setLamp(100);
        delay(50);
        setLamp(0);
        delay(50);
    }*/

    // Print the SSID to the serial monitor
    Serial.print("Access point SSID: ");
    Serial.println(ssid);
    is_accesspoint = true;
  }
  else
  { // try to connect to available Network
    log_d("Try to connect to stored wifi network SSID: %s, PW: %s", mssid_tmp, mpassword_tmp);
    WiFi.begin(mssid_tmp.c_str(), mpassword_tmp.c_str());
    WiFi.setSleep(false);
    log_d("Initi Wifi works");

    int nTrialWifiConnect = 0;
    int nTrialWifiConnectMax = 4;
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

        // open Access Point with random ID
        WiFi.softAP(ssid.c_str(), "");
        // blink_led(100, uniqueID);

        Serial.println("Failed to connect to Wi-Fi => Creating AP");
        // Print the SSID to the serial monitor
        Serial.print("Access point SSID: ");
        Serial.println(ssid);
        is_accesspoint = true;
        break;
      }
    }
  }
}

void setupOTA()
{
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
  if (not getSerialFrameEnabled())
  {
    log_i("Serial Frame Disabled");
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
  }
  else
  {
    log_i("Serial Frame Enabled");
    config.xclk_freq_hz = 20000000;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    log_i("Changing pixel format to grayscale...");
    config.pixel_format = PIXFORMAT_GRAYSCALE; // PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;        // for streaming}

    config.fb_count = 1;
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

    if (isSerialTransferMode)
    {
      sensor_t *s = esp_camera_sensor_get();
      s->set_hmirror(s, 1);
      s->set_vflip(s, 1);

      // enable manual camera settings
      s->set_gain_ctrl(s, 0);     // auto gain off (1 or 0)
      s->set_exposure_ctrl(s, 0); // auto exposure off (1 or 0)
      s->set_aec_value(s, 100);   // set exposure manually (0-1200)
      s->set_agc_gain(s, 0);      // set gain manually (0 - 30)
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
  for (int iDummyFrame = 0; iDummyFrame < 2; iDummyFrame++)
  {
    // FIXME: Look at the buffer for the camera => flush vs. return
    // log_d("Capturing dummy frame %i", iDummyFrame);
    fb = esp_camera_fb_get();
    if (!fb)
      log_e("Camera frame error", false);
    esp_camera_fb_return(fb);
  }
  // We now have camera with default init
  return initSuccess;
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

void setup()
{
  // check if we want to transfer images via serial or not
  isSerialTransferMode = getSerialFrameEnabled();
  if (isSerialTransferMode)
  {
    // Start Serial
    Serial.begin(2000000);
    Serial.println("Serial transfer mode enabled");
    Serial.setTimeout(20);
    Serial.println("Go to this website, connect the camera and try it out yourself: https://matchboxscope.github.io/blog/SerialCamera");
  }
  else
  {
    // Start Serial
    Serial.begin(115200);
    Serial.println("Serial transfer mode disabled");
  }

#ifdef NEOPIXEL_PIN
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(255);
  setNeopixel(0);
#endif
  setNeopixel(255);
  delay(60);
  setNeopixel(0);
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

    // reset anglerfishmode
    setIsTimelapseAnglerfish(false);
    setSerialFrameEnabled(false);
    setFrameIndex(SPIFFS, 0);
  }
  // declare LED PIN
  pinMode(LED_PIN, OUTPUT);

  // check if we want to acquire a stack instead of a single slice
  isStack = getAcquireStack(SPIFFS);

  // only for Anglerfish if already focussed
  isTimelapseAnglerfish = getIsTimelapseAnglerfish(); // set the global variable for the loop function

  // initialize SD card before LED!!
  // We initialize SD_MMC here rather than in setup() because SD_MMC needs to reset the light pin
  // with a different pin mode.
  // 1-bit mode as suggested here:https://dr-mntn.net/2021/02/using-the-sd-card-in-1-bit-mode-on-the-esp32-cam-from-ai-thinker

#if defined(CAMERA_MODEL_AI_THINKER)
  if (!SD_MMC.begin("/sdcard", true))
  { // FIXME: this sometimes leads to issues Unix vs. Windows formating - text encoding? Sometimes it copies to "sdcard" => Autoformating does this!!!
    Serial.println("SD Card Mount Failed");
    // FIXME: This should be indicated in the GUI
    sdInitialized = false;
    setIsTimelapseAnglerfish(false);
    isTimelapseAnglerfish = false;
    // Flash the LED to show SD card is not connected
    blink_led(50, 20);
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
      sdInitialized = false;
    }
    else
    {
      Serial.println(cardType);
    }
  }

#elif defined(CAMERA_MODEL_XIAO)
  // Initialize SD card
  if (!SD.begin(21))
  {
    Serial.println("SD Card Mount on on XIAO Failed");
    sdInitialized = false;
    setIsTimelapseAnglerfish(false);
    isTimelapseAnglerfish = false;
    // Flash the LED to show SD card is not connected
    blink_led(5, 20);
  }
  else
  {
    uint8_t cardType = SD.cardType();

    // Determine if the type of SD card is available
    if (cardType == CARD_NONE)
    {
      Serial.println("No SD card attached");
      sdInitialized = false;
      setIsTimelapseAnglerfish(false);
      isTimelapseAnglerfish = false;
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
    }
  }

#endif
  // Start threads for background frame publishing and serial handling // FIXME: Is this really necessary?
  if (!isTimelapseAnglerfish)
  {
    log_d("Starting saveCapturedImageGithubTask", "");
    xTaskCreatePinnedToCore(
        saveCapturedImageGithubTask,
        "saveCapturedImageGithubTask",
        10000,
        NULL,
        11,
        NULL,
        0);
  }

  // loading previous settings
  imagesServed = getFrameIndex(SPIFFS);
  timelapseInterval = getTimelapseInterval(SPIFFS);
  autofocusInterval = getAutofocusInterval(SPIFFS);

#if defined(CAMERA_MODEL_XIAO)
// not before
// setup stepper
#ifdef ACCELSTEPPER
  pinMode(STEPPER_MOTOR_DIR, OUTPUT);
  if (STEPPER_MOTOR_M1 >= 0)
    pinMode(STEPPER_MOTOR_M1, OUTPUT);
  if (STEPPER_MOTOR_M2 >= 0)
    pinMode(STEPPER_MOTOR_M2, OUTPUT);
  if (STEPPER_MOTOR_M3 >= 0)
    pinMode(STEPPER_MOTOR_M3, OUTPUT);
  if (STEPPER_MOTOR_NOTRESET >= 0)
    pinMode(STEPPER_MOTOR_NOTRESET, OUTPUT);
  if (STEPPER_MOTOR_NOTSLEEP >= 0)
    pinMode(STEPPER_MOTOR_NOTSLEEP, OUTPUT);
  if (STEPPER_MOTOR_NOTRESET >= 0)
    digitalWrite(STEPPER_MOTOR_NOTRESET, HIGH);
  if (STEPPER_MOTOR_NOTSLEEP >= 0)
    digitalWrite(STEPPER_MOTOR_NOTSLEEP, HIGH);
  if (STEPPER_MOTOR_M1 >= 0)
    digitalWrite(STEPPER_MOTOR_M1, HIGH);
  if (STEPPER_MOTOR_M2 >= 0)
    digitalWrite(STEPPER_MOTOR_M2, HIGH);
  if (STEPPER_MOTOR_M3 >= 0)
    digitalWrite(STEPPER_MOTOR_M3, HIGH);
  pinMode(STEPPER_MOTOR_ENABLE, OUTPUT);
  setMotorActive(true);
  motor.setMaxSpeed(STEPPER_MOTOR_SPEED);
  motor.setAcceleration(10000);
  setMotorActive(false);
#else
  // 28byj motor
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(1000);
  motor.setMaxSpeed(500);
  motor.setAcceleration(10000);
  //motor.runToNewPosition(10000);
  //motor.runToNewPosition(-10000);
  setMotorActive(false);
  
#endif
  motor.setSpeed(0);
  motor.setCurrentPosition(0);

#endif

  // Initialise and set the lamp
  ledcSetup(lampChannel, pwmfreq, pwmresolution); // configure LED PWM channel
  ledcAttachPin(LAMP_PIN, lampChannel);           // attach the GPIO pin to the channel
  log_d("LED pin: %d", LAMP_PIN);
  if (autoLamp)
    setLamp(0); // set default value
  else
    setLamp(lampVal);

  // Initialise and set the PWM output
  if (PWM_PIN >= 0)
  {
    pinMode(PWM_PIN, OUTPUT);
    log_d("PWM pin: %d", PWM_PIN);
    ledcSetup(pwmChannel, pwmfreq, pwmresolution); // configure LED PWM channel
    ledcAttachPin(PWM_PIN, pwmChannel);            // attach the GPIO pin to the channel
    setPWM(255);
    delay(30);
    setPWM(0);
  }
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

  // if we only transfer images over serial, we don't need wifi
  if (isSerialTransferMode)
  {
    Serial.println("Serial transfer mode enabled. To disable, type 'r' ");
    return;
  }
  // before we start the WIFI - check if we are in deep-sleep/anglerfishmode
  initAnglerfish(isTimelapseAnglerfish, isAnglerfishExposureBracketing);

  // WIFI-related settings
  Serial.println("...............");
  // start wifi AP or connect to AP

  initWifi();

  setNeopixel(255);
  delay(500);
  setNeopixel(0);

  // propagate URLs to GUI
  calcURLs();

  // setup OTA
  setupOTA();

  // MDNS Config -- note that if OTA is NOT enabled this needs prior steps!
  MDNS.addService("http", "tcp", 80);
  // Serial.println("Added HTTP service to MDNS server");

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

#if defined(CAMERA_MODEL_XIAO)
  if (Serial.available() > 0)
  {
    // String command = Serial.readString(); // Read the command until a newline character is received
    // if (command.length() > 1 && command.charAt(0) == 't')

    String command = Serial.readString();
    if (not isSerialTransferMode && command.length() > 1 && command.charAt(0) == 's')
    {
      // change settings to serial transfer mode
      isSerialTransferMode = true;
      Serial.begin(2000000);
      Serial.setTimeout(20);
      setSerialFrameEnabled(1);
      // we can only switch from JPEG to RAW after a restart
      ESP.restart();
      // TODO: Stop any stream on webservers
    }
    else
    {
      // Check for incoming serial commands
      if (command.length() > 1 && command.charAt(0) == 't')
      {
        // exposure time
        int value = command.substring(1).toInt(); // Extract the numeric part of the command and convert it to an integer
        // Use the value as needed
        // Apply manual settings for the camera
        sensor_t *s = esp_camera_sensor_get();
        s->set_gain_ctrl(s, 0);     // auto gain off (1 or 0)
        s->set_exposure_ctrl(s, 0); // auto exposure off (1 or 0)
        s->set_aec_value(s, value); // set exposure manually (0-1200)
      }
      else if (command.length() > 1 && command.charAt(0) == 'g')
      {
        // gain
        int value = command.substring(1).toInt(); // Extract the numeric part of the command and convert it to an integer

        // Apply manual settings for the camera
        sensor_t *s = esp_camera_sensor_get();
        s->set_gain_ctrl(s, 0);     // auto gain off (1 or 0)
        s->set_exposure_ctrl(s, 0); // auto exposure off (1 or 0)
        s->set_agc_gain(s, value);  // set gain manually (0 - 30)
      }
      else if (command.length() > 0 && command.charAt(0) == 'r')
      {
        // restart and switch back to JPEG mode
        setSerialFrameEnabled(0);
        ESP.restart();
      }
      else
      {
        flushSerial();
        // capture image and return
        grabRawFrameBase64();
      }
      flushSerial();
    }
  }
#endif

  // Timelapse Imaging
  // Perform timelapse imaging
  // timelapseInterval - will be changed by the httpd server
  // isTimelapseGeneral - will be changed by the httpd server //= getIsTimelapseGeneral(SPIFFS);

  if (not isSerialTransferMode)
  {
    if (isTimelapseGeneral and timelapseInterval > 0 and ((millis() - t_old) > (1000 * timelapseInterval)))
    {
      writePrefsToSSpiffs(SPIFFS);
      // https://stackoverflow.com/questions/67090640/errors-while-interacting-with-microsd-card
      log_d("Time to save a new image", timelapseInterval);
      t_old = millis();
      frameIndex = getFrameIndex(SPIFFS) + 1;

      // perform autofocus every n-times
      if (frameIndex % autofocusInterval == 0 and autofocusInterval > 0)
      {
        log_d("Performing autofocus");
        autoFocus(autofocus_min, autofocus_max, autofocus_stepsize);
      }
      // save to SD card if existent
      String filename = "/timelapse_image_scope_" + String(uniqueID) + "_" + String(millis()) + "_" + String(imagesServed);
      if (getAcquireStack(SPIFFS))
      { // FIXME: We could have a switch in the GUI for this settig
        // acquire a stack
        // FIXME: decide which method to use..
        log_d("Acquireing stack");
        imagesServed++;
        acquireFocusStack(filename, 10);
      }
      else
      {
        // Acquire the image and save
        imagesServed++;
        log_d("Store single image");
        int pwmVal = getPWMVal(SPIFFS);
        saveImage(filename, pwmVal);
      }

      // set default lamp value for streaming
      setLamp(lampVal);

      // FIXME: we should increase framenumber even if failed - since a corrupted file may lead to issues? (imageSaved)
      setFrameIndex(SPIFFS, frameIndex);
    }

    if (otaEnabled)
      ArduinoOTA.handle();

#ifdef CAMERA_MODEL_XIAO

    if (not isMotorRunningFixedPosition)
    {
      if (motor.speed() == 0 or motor.isRunning() == 0)
      {
#ifdef ACCELSTEPPER
        setMotorActive(false);
#endif
      }
      else
      {
#ifdef ACCELSTEPPER
        setMotorActive(true);

#endif
      }
      motor.runSpeed();
    }
#endif
  }
}

void loadAnglerfishCamSettings(int tExposure, int mGain)
{
  Serial.println("Resetting Camera Sensor Settings...");

  // Apply manual settings for the camera
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_SVGA); // FRAMESIZE_QVGA);
  s->set_gain_ctrl(s, 0);              // auto gain off (1 or 0)
  s->set_exposure_ctrl(s, 0);          // auto exposure off (1 or 0)
  s->set_agc_gain(s, mGain);           // set gain manually (0 - 30)
  s->set_aec_value(s, tExposure);      // set exposure manually (0-1200)

  // digest the settings => warmup camera
  camera_fb_t *fb = NULL;
  for (int iDummyFrame = 0; iDummyFrame < 2; iDummyFrame++)
  {
    // FIXME: Look at the buffer for the camera => flush vs. return
    // log_d("Capturing dummy frame %i", iDummyFrame);
    fb = esp_camera_fb_get();
    if (!fb)
      log_e("Camera frame error", false);
    esp_camera_fb_return(fb);

    int mean_intensity = get_mean_intensity(fb);
  }
}

bool writePythonProcessingFile()
{
  char filename[] = "/loadTimelapse.py";

  if (SD.exists(filename))
  {
    Serial.println("File already exists!");
    return false;
  }

  File codeFile = SD.open(filename, FILE_WRITE);

  if (codeFile)
  {
    codeFile.println("#%%");
    codeFile.println("import os");
    codeFile.println("import cv2");
    codeFile.println("import numpy as np");
    codeFile.println("import tifffile as tif");
    codeFile.println("");
    codeFile.println("base_dir = '/Volumes/SD'  # Modify this as needed");
    codeFile.println("folder_count = 0");
    codeFile.println("save_dir = '/Users/bene/Downloads'");
    codeFile.println("experiment_name = '2023_10_24-Longtime_Autofocus_Test_1minPerImage'");
    codeFile.println("tiff_filename = os.path.join(save_dir, experiment_name+ '.tif')");
    codeFile.println("allImages = []");
    codeFile.println("nFilesNotFound = 0");
    codeFile.println("while True:");
    codeFile.println("    current_folder = os.path.join(base_dir, str(folder_count))");
    codeFile.println("    if not os.path.exists(current_folder):");
    codeFile.println("        nFilesNotFound+=1");
    codeFile.println("        if nFilesNotFound > 2:");
    codeFile.println("            print('Reached end of folders')");
    codeFile.println("            break");
    codeFile.println("        continue");
    codeFile.println("    nFilesNotFound = 0");
    codeFile.println("    jpg_files = [f for f in os.listdir(current_folder) if f.endswith('.jpg')]");
    codeFile.println("    try:");
    codeFile.println("        image = cv2.imread(os.path.join(current_folder, jpg_files[0]))");
    codeFile.println("        image = np.mean(image, axis=2).astype(np.uint8)");
    codeFile.println("        allImages.append(image)");
    codeFile.println("        print('writing file %s to stack' % os.path.join(current_folder, jpg_files[0]))");
    codeFile.println("    except Exception as e:");
    codeFile.println("        print(e)");
    codeFile.println("        print('Could not find image in folder %s' % current_folder)");
    codeFile.println("    folder_count += 1");
    codeFile.println("tif.imwrite(tiff_filename, np.array(allImages))");
    codeFile.close();
    Serial.println("Code saved to SD card!");
    return true;
  }
  else
  {
    Serial.println("Error opening file for writing.");
    return false;
  }
}

void initAnglerfish(bool isTimelapseAnglerfish, bool isAnglerfishExposureBracketing)
{
  log_d("Anglerfishmode is: %i", isTimelapseAnglerfish);
  if (isTimelapseAnglerfish)
  {
// activate LED FIXME:
#ifdef CAMERA_MODEL_AI_THINKER
    rtc_gpio_hold_dis(GPIO_NUM_4);
#endif
    // Create the processing file on the SD card
    writePythonProcessingFile();

    // ONLY IF YOU WANT TO CAPTURE in ANGLERFISHMODE
    Serial.println("In timelapse anglerfish mode.");

    // override LED intensity settings
    lampVal = 255;
    setLamp(lampVal * autoLamp);

    // Save image to SD card
    uint32_t frameIndex = getFrameIndex(SPIFFS) + 1;
    if (frameIndex > 0 and frameIndex % autofocusInterval == 0 and autofocusInterval > 0)
    {
      log_d("Performing autofocus");
      autoFocus(autofocus_min, autofocus_max, autofocus_stepsize);
    }
    // FIXME: decide which method to use..
    setFrameIndex(SPIFFS, frameIndex);
    // Get the compile date and time as a string
    String compileDate = String(__DATE__) + " " + String(__TIME__);
    // Remove spaces and colons from the compile date and time string
    compileDate.replace(" ", "_");
    compileDate.replace(":", "");
    int stepSize = 2;
    int stepMin = 270;
    int stepMax = 296;
    bool isAcquireStack = getAcquireStack(SPIFFS);
    if (!isAcquireStack)
    {
      stepMin = 0;
      stepMax = 1;
      stepSize = 2;
    }

    long time1 = millis();
    if (1) // for (int iFocus = stepMin; iFocus < stepMax; iFocus += stepSize)
    {
      int iFocus = -1;
      // setPWM(iFocus);
      String folderName = "/" + String(imagesServed);
      // FIXME: If we save single files to the SD card, the time to store them growth with every file
      // workaround for now: We store them in a folders
      // some insights:https://forum.arduino.cc/t/esp32-cam-drastic-slowdown-in-writing-to-the-sd-card-with-increasing-number-of-files/1094767
      log_d("Creating folder %s", folderName);
      SD.mkdir(folderName);
      String filename = folderName + "/data_" + compileDate + "_timelapse_image_anglerfish_" + String(imagesServed) + "_z" + String(iFocus) + "_";

      if (isAnglerfishExposureBracketing)
      {
        /*
        Perform an exposure series in case the AEC doesn't work well
        */
        log_d("Anglerfish: Acquire Exposure Series");
        // under expose
        loadAnglerfishCamSettings(1, 0);
        saveImage(filename + "texp_1", iFocus);

        // over expose
        loadAnglerfishCamSettings(5, 0);
        saveImage(filename + "texp_5", iFocus);

        // over expose
        loadAnglerfishCamSettings(10, 0);
        saveImage(filename + "texp_10", iFocus);

        // over expose
        loadAnglerfishCamSettings(50, 0);
        saveImage(filename + "texp_50", iFocus);

        // over expose
        loadAnglerfishCamSettings(100, 0);
        saveImage(filename + "texp_100", iFocus);

        // over expose
        loadAnglerfishCamSettings(500, 0);
        saveImage(filename + "texp_500", iFocus);

        // even more over expose
        loadAnglerfishCamSettings(1000, 0);
        saveImage(filename + "texp_1000", iFocus);
      }
      else
      {
        /*
        take image using AEC
        */
        // let automatic intensity settle
        camera_fb_t *fb = NULL;
        for (int iDummyFrame = 0; iDummyFrame < 10; iDummyFrame++)
        {
          fb = esp_camera_fb_get();
          if (!fb)
            log_e("Camera frame error", false);
          esp_camera_fb_return(fb);
        }
        saveImage(filename + "_AEC", -1);
      }
    }
    setPWM(0);
    imagesServed++;
    log_d("Acquisition took %i ms", millis() - time1);

    // FIXME: we should increase framenumber even if failed - since a corrupted file may lead to issues?
    setFrameIndex(SPIFFS, imagesServed);

    // Sleep
    if (timelapseInterval == -1)
      timelapseInterval = 60; // do timelapse every five minutes if not set properly
    Serial.print("Sleeping for ");
    Serial.print(timelapseInterval);
    Serial.println(" s");
    static const uint64_t usPerSec = 1000000; // Conversion factor from microseconds to seconds
    SD_MMC.end();                             // FIXME: may cause issues when file not closed? categoreis: LED/SD-CARD issues

    // turn off lamp entirely
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    rtc_gpio_hold_en(GPIO_NUM_4); // Latch value when going into deep sleep

    // go to deep sleep
    esp_sleep_enable_timer_wakeup(timelapseInterval * usPerSec);
    esp_deep_sleep_start();
    return;
  }
  else
  {
    Serial.println("In livestreaming mode. Connecting to pre-set Wifi");
  }
}

void flushSerial()
{
  // flush serial
  while (Serial.available() > 0)
  {
    char c = Serial.read();
  }
}

void grabRawFrameBase64()
{
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();

  if (!fb || fb->format != PIXFORMAT_GRAYSCALE) // PIXFORMAT_JPEG)
  {
    Serial.println("Failed to capture image");
    ESP.restart();
  }
  else
  {
    // Modify the first 10 pixels of the buffer to indicate framesync
    // PRoblem: The reference frame will move over time at random places
    // It'S not clear if this is an issue on the client or server side
    // Solution: To align for it we intoduce a known pattern that we can search for
    // in order to align for this on the client side
    // (actually something funky goes on here: We don't even need to align for that on the client side if we introduce these pixels..)
    for (int i = 0; i < 10; i++)
    {
      fb->buf[i] = i % 2; // Alternates between 0 and 1
    }
    // delay(40);

    // String encoded = base64::encode(fb->buf, fb->len);
    // Serial.write(encoded.c_str(), encoded.length());
    if (0)
    {
      Serial.write(fb->buf, fb->len);
    }
    else
    {
      // Encode the buffer in base64 and send it
      String encoded = base64::encode((uint8_t *)fb->buf, fb->len);
      Serial.println(encoded);
      // free(encoded); // Remember to free the encoded buffer after using it
    }
    // Serial.println();
  }

  esp_camera_fb_return(fb);
}

// move focus using accelstepper
void moveFocusRelative(int steps, bool handleEnable = true)
{
#ifdef CAMERA_MODEL_XIAO
// a very bad idea probably, but otherwise we may have concurancy with the loop function
  if (handleEnable)
    setMotorActive(true);
  // log_i("Moving focus %d steps, currentposition %d", motor.currentPosition() + steps, motor.currentPosition());

  // run motor to new position with relative movement
  motor.setSpeed(STEPPER_MOTOR_SPEED);
  motor.runToNewPosition(motor.currentPosition() + steps);
  if (handleEnable)
    setMotorActive(false);
  #endif
}

int getCurrentMotorPos()
{
#ifdef CAMERA_MODEL_XIAO
  return motor.currentPosition();
#endif
}

void setSpeed(int speed)
{
#ifdef CAMERA_MODEL_XIAO
  motor.setSpeed(speed);
#endif
}

void setMotorActive(bool isActive)
{
#ifdef CAMERA_MODEL_XIAO
#ifdef ACCELSTEPPER
  // low means active
  digitalWrite(STEPPER_MOTOR_ENABLE, !isActive);
#else
  digitalWrite(motorPin1, isActive);
  digitalWrite(motorPin2, isActive);
  digitalWrite(motorPin3, isActive);
  digitalWrite(motorPin4, isActive);
#endif
#endif
}
