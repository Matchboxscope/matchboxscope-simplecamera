# 1 "/var/folders/4w/k4yhf14j7xsbp2jd85yk555r0000gn/T/tmp9du4sk_4"
#include <Arduino.h>
# 1 "/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/main/main.ino"
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



#define NUMPIXELS 16

#define STEPPER_MOTOR 
#define STEPPER_MOTOR_STEPS 200
#define STEPPER_MOTOR_SPEED 10000
#define STEPPER_MOTOR_DIR D2
#define STEPPER_MOTOR_STEP D1
#define STEPPER_MOTOR_ENABLE D0

#ifdef CAMERA_MODEL_XIAO
#include <AccelStepper.h>
AccelStepper motor(1, STEPPER_MOTOR_STEP, STEPPER_MOTOR_DIR);
#endif


#define CAM_NAME "Matchboxscope"
#define MDNS_NAME "Matchboxscope"





struct station
{
  const char ssid[65];
  const char password[65];
  const bool dhcp;
};


#include "version.h"


#include "camera_pins.h"


camera_config_t config;



#include "storage.h"


#define MAX_ATTEMPTS_WIFI_CONNECTION 20

uint8_t x_buffer[16];
uint8_t x_position = 0;

int frameWidth = 0;
int frameHeight = 0;


int sketchSize;
int sketchSpace;
String sketchMD5;

String uniqueID = "0";





IPAddress ip;
IPAddress net;
IPAddress gw;
bool is_accesspoint = false;


extern void startCameraServer(int hPort, int sPort);
extern void serialDump();
extern void saveCapturedImageGithub();

bool onCommandCallback(improv::ImprovCommand cmd);
void getAvailableWifiNetworks();
void set_state(improv::State state);
void onErrorCallback(improv::Error err);
void set_error(improv::Error error);
void send_response(std::vector<uint8_t> &response);


char myName[] = CAM_NAME;
char mdnsName[] = MDNS_NAME;

int httpPort = 80;
int streamPort = 81;


const char *mssid = "Blynk";
const char *mpassword = "12345678";

#ifdef NEOPIXEL
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(NUMPIXELS, PWM_PIN, NEO_GRB + NEO_KHZ800);
#endif

#define WIFI_WATCHDOG 15000


char default_index[] = "full";


boolean sdInitialized = false;


const byte DNS_PORT = 53;
DNSServer dnsServer;


char httpURL[64] = {"Undefined"};
char streamURL[64] = {"Undefined"};


int8_t streamCount = 0;
unsigned long streamsServed = 0;
unsigned long imagesServed = 0;


char myVer[] PROGMEM = __DATE__ " @ " __TIME__;



int sensorPID;




#if defined(CAMERA_MODEL_AI_THINKER)
unsigned long xclk = 8;
#elif defined(CAMERA_MODEL_XIAO)
unsigned long xclk = 20;
#endif


int myRotation = 0;
bool isStack = false;
bool isTimelapseAnglerfish = false;
bool isTimelapseGeneral = false;


int minFrameTime = 0;


int timelapseInterval = -1;
static uint64_t t_old = millis();
bool sendToGithubFlag = false;
uint32_t frameIndex = 0;


int lampVal = 0;
bool autoLamp = false;
int pwmVal = 0;
bool BUSY_SET_LED = false;

#if defined(CAMERA_MODEL_AI_THINKER)
int lampChannel = 7;
int pwmChannel = 5;
#elif defined(CAMERA_MODEL_XIAO)
int lampChannel = 1;
int pwmChannel = 2;
#endif
const int pwmfreq = 50000;
const int pwmresolution = 9;
const int pwmMax = pow(2, pwmresolution) - 1;

bool filesystem = true;

bool otaEnabled = false;
char otaPassword[] = "";

const char *ntpServer = "";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;

extern bool saveImage(String filename, int pwmVal);


String critERR = "";
void handleSerial();
void flashLED(int flashtime);
void blink_led(int d, int times);
void setLamp(int newVal);
void setPWM(int newVal);
String getThreeDigitID();
void initWifi();
void setupOTA();
void calcURLs();
bool StartCamera();
void handleSerialTask(void *pvParameters);
void saveCapturedImageGithubTask(void *pvParameters);
int get_mean_intensity(camera_fb_t *fb);
void setup();
void loop();
void loadAnglerfishCamSettings(int tExposure, int mGain);
void initAnglerfish(bool isTimelapseAnglerfish);
void moveFocus(int steps);
void setSpeed(int speed);
#line 200 "/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/main/main.ino"
void handleSerial()
{
  if (Serial.available())
  {
    uint8_t b = Serial.read();

    if (parse_improv_serial_byte(x_position, b, x_buffer, onCommandCallback, onErrorCallback))
    {
      x_buffer[x_position++] = b;
    }
    else
    {
      x_position = 0;
    }

    while (Serial.available())
      Serial.read();
  }
}

bool connectWifi(std::string ssid, std::string password)
{
  uint8_t count = 0;
  WiFi.begin(ssid.c_str(), password.c_str());
  while (WiFi.status() != WL_CONNECTED)
  {
    blink_led(500, 1);
    if (count > MAX_ATTEMPTS_WIFI_CONNECTION)
    {
      WiFi.disconnect();
      return false;
    }
    count++;
  }
  return true;
}


void flashLED(int flashtime)
{
#ifdef CAMERA_MODEL_AI_THINKER
  digitalWrite(LED_PIN, LED_ON);
  delay(flashtime);
  digitalWrite(LED_PIN, LED_OFF);
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


void setLamp(int newVal)
{
  if (newVal != -1 and !BUSY_SET_LED)
  {
    BUSY_SET_LED = true;

    int brightness = round((pow(2, (1 + (newVal * 0.02))) - 2) / 6 * pwmMax);
    ledcWrite(lampChannel, brightness);
    Serial.print("Lamp: ");
    Serial.print(newVal);
    Serial.print("%, pwm = ");
    Serial.println(brightness);
    BUSY_SET_LED = false;
    delay(15);
  }
}


void setPWM(int newVal)
{
#ifdef NEOPIXEL
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(newVal, newVal, newVal));
  }
  pixels.show();
#else
  if (newVal != -1)
  {

    int current = newVal;
    ledcWrite(pwmChannel, current);
    Serial.print("Current: ");
    Serial.print(newVal);
    Serial.print("%, pwm = ");
    Serial.println(current);
  }
#endif
}

String getThreeDigitID()
{

  uint8_t mac_address[6];
  WiFi.macAddress(mac_address);


  uint32_t combined_mac = (mac_address[0] << 24) |
                         (mac_address[1] << 16) |
                         (mac_address[2] << 8) |
                         mac_address[3];


  uint32_t five_digit_number = combined_mac % 100000;


  return String(five_digit_number);

}

void initWifi()
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
  String mssid_tmp = getWifiSSID(SPIFFS);
  String mpassword_tmp = getWifiPW(SPIFFS);

  uniqueID = getThreeDigitID();
  String ssid = "Matchboxscope-" + uniqueID;


  if (mssid_tmp == "")
  {

    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), "");
# 375 "/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/main/main.ino"
    Serial.print("Access point SSID: ");
    Serial.println(ssid);
    is_accesspoint = true;
  }
  else
  {
    log_d("Try to connect to stored wifi network SSID: %s, PW: %s", mssid_tmp, mpassword_tmp);
    WiFi.begin(mssid_tmp.c_str(), mpassword_tmp.c_str());
    WiFi.setSleep(false);
    log_d("Initi Wifi works");

    int nTrialWifiConnect = 0;
    int nTrialWifiConnectMax = 2;
    int tWaitWifiConnect = 500;
    while (WiFi.status() != WL_CONNECTED)
    {
      log_d("Connecting to Wi-Fi...");
      nTrialWifiConnect++;
      delay(tWaitWifiConnect);

      if (nTrialWifiConnect > nTrialWifiConnectMax)
      {
        WiFi.disconnect();
        WiFi.mode(WIFI_AP);


        WiFi.softAP(ssid.c_str(), "");


        Serial.println("Failed to connect to Wi-Fi => Creating AP");

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

  if (otaEnabled)
  {

    Serial.println("Setting up OTA");



    ArduinoOTA.setHostname(mdnsName);

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
                else

                    type = "filesystem";
                Serial.println("Start updating " + type);


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

  if (is_accesspoint)
    ip = WiFi.softAPIP();
  else
    ip = WiFi.localIP();
  net = WiFi.subnetMask();
  gw = WiFi.gatewayIP();


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

      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }
  else
  {

    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }
#endif


  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    delay(100);
    Serial.printf("\r\n\r\nCRITICAL FAILURE: Camera sensor failed to initialise.\r\n\r\n");
    Serial.printf("A full (hard, power off/on) reboot will probably be needed to recover from this.\r\n");
    Serial.printf("Meanwhile; this unit will reboot in 1 minute since these errors sometime clear automatically\r\n");

    periph_module_disable(PERIPH_I2C0_MODULE);
    periph_module_disable(PERIPH_I2C1_MODULE);
    periph_module_reset(PERIPH_I2C0_MODULE);
    periph_module_reset(PERIPH_I2C1_MODULE);

    critERR = "<h1>Error!</h1><hr><p>Camera module failed to initialise!</p><p>Please reset (power off/on) the camera.</p>";
    critERR += "<p>We will continue to reboot once per minute since this error sometimes clears automatically.</p>";

    esp_task_wdt_init(60, true);
    esp_task_wdt_add(NULL);
    initSuccess = false;


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


    sensor_t *s = esp_camera_sensor_get();


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


    if (sensorPID == OV3660_PID)
    {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
      s->set_saturation(s, -2);
    }
# 688 "/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/main/main.ino"
    initSuccess = true;
  }


  camera_fb_t *fb = NULL;
  for (int iDummyFrame = 0; iDummyFrame < 2; iDummyFrame++)
  {

    log_d("Capturing dummy frame %i", iDummyFrame);
    fb = esp_camera_fb_get();
    if (!fb)
      log_e("Camera frame error", false);
    esp_camera_fb_return(fb);

    int mean_intensity = get_mean_intensity(fb);

    Serial.print("Mean intensity: ");
    Serial.println(mean_intensity);
  }

  return initSuccess;
}

void handleSerialTask(void *pvParameters)
{
  Serial.println("Creating task handleSerial");

  Serial.println("Navigate to https://matchboxscope.github.io/firmware/FLASH.html and enter the Wifi SSID/PW through the GUI");
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

  Serial.begin(115200);
#ifdef NEOPIXEL
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    pixels.show();
  }
  delay(60);
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
  }
  delay(60);
  pixels.begin();
#endif


  if (!psramFound())
  {
    Serial.println("\r\nFatal Error; Halting");
    while (true)
    {
      Serial.println("No PSRAM found; camera cannot be initialised: Please check the board config for your module.");
      delay(5000);
    }
  }


  filesystemStart();
  delay(500);


  bool camInitSuccess = StartCamera();


  if (not camInitSuccess)
  {
    log_e("Camera failed to initialize %i", camInitSuccess);
    ESP.restart();
  }

#if defined(CAMERA_MODEL_XIAO)

    digitalWrite(STEPPER_MOTOR_DIR, HIGH);

    motor.setMaxSpeed(STEPPER_MOTOR_SPEED);
    motor.setAcceleration(1000);
    motor.setSpeed(STEPPER_MOTOR_SPEED);
    pinMode(STEPPER_MOTOR_ENABLE, OUTPUT);
    digitalWrite(STEPPER_MOTOR_ENABLE, LOW);
    motor.runToNewPosition(5);
    motor.runToNewPosition(-5);
    digitalWrite(STEPPER_MOTOR_ENABLE, HIGH);
#endif


  bool isFirstRun = isFirstBoot();
  if (isFirstRun)
  {

    log_d("Remove SPIFFS");
    removePrefs(SPIFFS);


    log_d("Write Prefs to SPIFFS");
    writePrefsToSSpiffs(SPIFFS);


    log_d("Set compiled date");
    setCompiledDate(SPIFFS);


    setIsTimelapseAnglerfish(false);
  }

  pinMode(LED_PIN, OUTPUT);


  isStack = getAcquireStack(SPIFFS);


  isTimelapseAnglerfish = getIsTimelapseAnglerfish();






#if defined(CAMERA_MODEL_AI_THINKER)
  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.println("SD Card Mount Failed");

    sdInitialized = false;
    setIsTimelapseAnglerfish(false);
    isTimelapseAnglerfish = false;

    blink_led(50, 20);
  }
  else
  {
    sdInitialized = true;
    Serial.println("SD Card Mounted");


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

  if (!SD.begin(21))
  {
    Serial.println("Card Mount Failed");
    sdInitialized = false;
    setIsTimelapseAnglerfish(false);
    isTimelapseAnglerfish = false;

    blink_led(50, 20);
  }
  else
  {
    uint8_t cardType = SD.cardType();


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


  imagesServed = getFrameIndex(SPIFFS);
  timelapseInterval = getTimelapseInterval(SPIFFS);


  ledcSetup(lampChannel, pwmfreq, pwmresolution);
  ledcAttachPin(LAMP_PIN, lampChannel);
  log_d("LED pin: %d", LAMP_PIN);
  if (autoLamp)
    setLamp(0);
  else
    setLamp(lampVal);

#ifndef NEOPIXEL

  pinMode(PWM_PIN, OUTPUT);
  log_d("PWM pin: %d", PWM_PIN);
  ledcSetup(pwmChannel, pwmfreq, pwmresolution);
  ledcAttachPin(PWM_PIN, pwmChannel);
  ledcWrite(pwmChannel, 255);
  delay(30);
  ledcWrite(pwmChannel, 0);
#endif



  setLamp(20);
  delay(50);
  setLamp(0);


  if (filesystem)
  {
    delay(200);
    loadSpiffsToPrefs(SPIFFS);
    Serial.println("internal files System found and mounted");
  }
  else
  {
    Serial.println("No Internal Filesystem, cannot load or save preferences");
  }


  initAnglerfish(isTimelapseAnglerfish);


  Serial.println("...............");

  initWifi();


  calcURLs();


  setupOTA();


  MDNS.addService("http", "tcp", 80);
  Serial.println("Added HTTP service to MDNS server");


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



  for (int iFocus = stepMin; iFocus < stepMax; iFocus += stepSize)
  {
    saveImage(filename + String(imagesServed) + "_z" + String(iFocus), iFocus);
  }
}

void loop()
{
# 1050 "/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/main/main.ino"
  if (isTimelapseGeneral and timelapseInterval > 0 and ((millis() - t_old) > (1000 * timelapseInterval)))
  {
    writePrefsToSSpiffs(SPIFFS);

    log_d("Time to save a new image", timelapseInterval);
    t_old = millis();
    frameIndex = getFrameIndex(SPIFFS) + 1;




    String filename = "/timelapse_image_scope_" + String(uniqueID) + "_" + String(millis()) + "_" + String(imagesServed);
    if (getAcquireStack(SPIFFS))
    {


      log_d("Acquireing stack");
      imagesServed++;
      acquireFocusStack(filename, 10);
    }
    else
    {

      imagesServed++;
      int pwmVal = getPWMVal(SPIFFS);
      saveImage(filename, pwmVal);
    }


    setLamp(lampVal);


    setFrameIndex(SPIFFS, frameIndex);
  }

  if (otaEnabled)
    ArduinoOTA.handle();

   #ifdef CAMERA_MODEL_XIAO
    motor.runSpeed();
    #endif


}

void loadAnglerfishCamSettings(int tExposure, int mGain)
{
  Serial.println("Resetting Camera Sensor Settings...");


  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_gain_ctrl(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_agc_gain(s, mGain);
  s->set_aec_value(s, tExposure);


  camera_fb_t *fb = NULL;
  for (int iDummyFrame = 0; iDummyFrame < 2; iDummyFrame++)
  {


    fb = esp_camera_fb_get();
    if (!fb)
      log_e("Camera frame error", false);
    esp_camera_fb_return(fb);

    int mean_intensity = get_mean_intensity(fb);
  }
}

void initAnglerfish(bool isTimelapseAnglerfish)
{
  log_d("Anglerfishmode is: %i", isTimelapseAnglerfish);
  if (isTimelapseAnglerfish)
  {

#ifdef CAMERA_MODEL_AI_THINKER
    rtc_gpio_hold_dis(GPIO_NUM_4);
#endif

    Serial.println("In timelapse anglerfish mode.");


    lampVal = 255;
    setLamp(lampVal * autoLamp);


    uint32_t frameIndex = getFrameIndex(SPIFFS) + 1;


    setFrameIndex(SPIFFS, frameIndex);

    String compileDate = String(__DATE__) + " " + String(__TIME__);

    compileDate.replace(" ", "_");
    compileDate.replace(":", "");

    int stepSize = 2;
    int stepMin = 270;
    int stepMax = 296;
    bool isAcquireStack = getAcquireStack(SPIFFS);
    if (isAcquireStack)
    {
      stepMin = 0;
      stepMax = 1;
      stepSize = 2;
    }

    long time1 = millis();
    for (int iFocus = stepMin; iFocus < stepMax; iFocus += stepSize)
    {
      String folderName = "/" + String(imagesServed);



      SD.mkdir(folderName);
      String filename = folderName + "/data_" + compileDate + "_timelapse_image_anglerfish_" + String(imagesServed) + "_z" + String(iFocus) + "_";

      setPWM(iFocus);
      log_d("Anglerfish: Acquire Exposure Series");

      loadAnglerfishCamSettings(1, 0);
      saveImage(filename + "texp_1", iFocus);


      loadAnglerfishCamSettings(5, 0);
      saveImage(filename + "texp_5", iFocus);


      loadAnglerfishCamSettings(10, 0);
      saveImage(filename + "texp_10", iFocus);


      loadAnglerfishCamSettings(50, 0);
      saveImage(filename + "texp_50", iFocus);


      loadAnglerfishCamSettings(100, 0);
      saveImage(filename + "texp_100", iFocus);


      loadAnglerfishCamSettings(500, 0);
      saveImage(filename + "texp_500", iFocus);
    }
    setPWM(0);
    imagesServed++;
    log_d("Acquisition took %i ms", millis() - time1);


    setFrameIndex(SPIFFS, imagesServed);


    if (timelapseInterval == -1)
      timelapseInterval = 60;
    Serial.print("Sleeping for ");
    Serial.print(timelapseInterval);
    Serial.println(" s");
    static const uint64_t usPerSec = 1000000;
    SD_MMC.end();


    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    rtc_gpio_hold_en(GPIO_NUM_4);


    esp_sleep_enable_timer_wakeup(timelapseInterval * usPerSec);
    esp_deep_sleep_start();
    return;
  }
  else
  {
    Serial.println("In livestreaming mode. Connecting to pre-set Wifi");
  }
}
# 1235 "/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/main/main.ino"
std::vector<std::string> getLocalUrl()
{
  return {


      String("http://" + WiFi.localIP().toString()).c_str()};
}

void onErrorCallback(improv::Error err)
{
  blink_led(2000, 3);
}

bool onCommandCallback(improv::ImprovCommand cmd)
{

  switch (cmd.command)
  {
  case improv::Command::GET_CURRENT_STATE:
  {
    if ((WiFi.status() == WL_CONNECTED))
    {
      set_state(improv::State::STATE_PROVISIONED);
      std::vector<uint8_t> data = improv::build_rpc_response(improv::GET_CURRENT_STATE, getLocalUrl(), false);
      send_response(data);
    }
    else
    {
      set_state(improv::State::STATE_AUTHORIZED);
    }

    break;
  }

  case improv::Command::WIFI_SETTINGS:
  {
    if (cmd.ssid.length() == 0)
    {
      set_error(improv::Error::ERROR_INVALID_RPC);
      break;
    }

    set_state(improv::STATE_PROVISIONING);

    if (connectWifi(cmd.ssid, cmd.password))
    {

      blink_led(100, 3);


      Serial.println("Connected to Wi-Fi!");
      setWifiPW(SPIFFS, cmd.ssid.c_str());
      setWifiSSID(SPIFFS, cmd.password.c_str());

      set_state(improv::STATE_PROVISIONED);
      std::vector<uint8_t> data = improv::build_rpc_response(improv::WIFI_SETTINGS, getLocalUrl(), false);
      send_response(data);

    }
    else
    {
      set_state(improv::STATE_STOPPED);
      set_error(improv::Error::ERROR_UNABLE_TO_CONNECT);
    }

    break;
  }

  case improv::Command::GET_DEVICE_INFO:
  {
    std::vector<std::string> infos = {

        "ImprovWiFiDemo",

        "1.0.0",

        "ESP32",

        "SimpleWebServer"};
    std::vector<uint8_t> data = improv::build_rpc_response(improv::GET_DEVICE_INFO, infos, false);
    send_response(data);
    break;
  }

  case improv::Command::GET_WIFI_NETWORKS:
  {
    getAvailableWifiNetworks();
    break;
  }

  default:
  {
    set_error(improv::ERROR_UNKNOWN_RPC);
    return false;
  }
  }

  return true;
}

void getAvailableWifiNetworks()
{
  int networkNum = WiFi.scanNetworks();

  for (int id = 0; id < networkNum; ++id)
  {
    std::vector<uint8_t> data = improv::build_rpc_response(
        improv::GET_WIFI_NETWORKS, {WiFi.SSID(id), String(WiFi.RSSI(id)), (WiFi.encryptionType(id) == WIFI_AUTH_OPEN ? "NO" : "YES")}, false);
    send_response(data);
    delay(1);
  }

  std::vector<uint8_t> data =
      improv::build_rpc_response(improv::GET_WIFI_NETWORKS, std::vector<std::string>{}, false);
  send_response(data);
}

void set_state(improv::State state)
{

  std::vector<uint8_t> data = {'I', 'M', 'P', 'R', 'O', 'V'};
  data.resize(11);
  data[6] = improv::IMPROV_SERIAL_VERSION;
  data[7] = improv::TYPE_CURRENT_STATE;
  data[8] = 1;
  data[9] = state;

  uint8_t checksum = 0x00;
  for (uint8_t d : data)
    checksum += d;
  data[10] = checksum;

  Serial.write(data.data(), data.size());
}

void send_response(std::vector<uint8_t> &response)
{
  std::vector<uint8_t> data = {'I', 'M', 'P', 'R', 'O', 'V'};
  data.resize(9);
  data[6] = improv::IMPROV_SERIAL_VERSION;
  data[7] = improv::TYPE_RPC_RESPONSE;
  data[8] = response.size();
  data.insert(data.end(), response.begin(), response.end());

  uint8_t checksum = 0x00;
  for (uint8_t d : data)
    checksum += d;
  data.push_back(checksum);

  Serial.write(data.data(), data.size());
}

void set_error(improv::Error error)
{
  std::vector<uint8_t> data = {'I', 'M', 'P', 'R', 'O', 'V'};
  data.resize(11);
  data[6] = improv::IMPROV_SERIAL_VERSION;
  data[7] = improv::TYPE_ERROR_STATE;
  data[8] = 1;
  data[9] = error;

  uint8_t checksum = 0x00;
  for (uint8_t d : data)
    checksum += d;
  data[10] = checksum;

  Serial.write(data.data(), data.size());
}

bool isMotorRunning=false;

void moveFocus(int steps)
{
  #ifdef CAMERA_MODEL_XIAO
  if (not isMotorRunning){
  isMotorRunning=true;
  digitalWrite(STEPPER_MOTOR_ENABLE, LOW);

  motor.setMaxSpeed(STEPPER_MOTOR_SPEED);
  motor.setAcceleration(1000);
  motor.setSpeed(STEPPER_MOTOR_SPEED);
  motor.move(steps);
  while(motor.distanceToGo() != 0){
    motor.run();
  }

  isMotorRunning=false;
  motor.setSpeed(0);
  }
  #endif
}



void setSpeed(int speed){
  #ifdef CAMERA_MODEL_XIAO
  motor.setSpeed(speed);
  #endif
}