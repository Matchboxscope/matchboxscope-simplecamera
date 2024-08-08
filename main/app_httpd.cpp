// Original Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <base64.h>
#include <Arduino.h>
#include <WiFi.h>
#include "camera_pins.h"

#include <esp_http_server.h>
#include "esp_http_client.h"
#include <esp_timer.h>
#include <esp_camera.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <SD_MMC.h>
#include <SD.h>
#include <SPIFFS.h>
#include <FS.h>

#include "index_ov2640.h"
#include "index_ov3660.h"
#include "index_other.h"
#include "index_holo.h"
#include "index_gallery.h"
#include "css.h"
#include "favicons.h"
#include "logo.h"
#include "storage.h"

#include <NTPClient.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoWebsockets.h>
#include "HTTPClientESP32.h"

// only inlcude if file available
#define HAS_GITHUB
#ifdef HAS_GITHUB
#include "githubtoken.h"
#elif
const char *GITHUB_TOKEN = "";
#endif

// Functions from the main .ino
extern void setLamp(int newVal);
extern void setPWM(int newVal);
extern void loadSpiffsToPrefs(fs::FS &fs);
extern void moveFocusRelative(int val, bool handleEnable);
extern void setNeopixel(int);

void saveCapturedImageGithub();

// Select between full and simple index as the default.
char default_index[] = "full";

camera_fb_t *convolution(camera_fb_t *input);
bool saveImage(String filename, int pwmVal = -1);
int autoFocus(int minPos, int maxPos, int focusStep);

// External variables declared in the main .ino
extern char myName[];
extern char myVer[];
extern char baseVersion[];
extern IPAddress ip;
extern IPAddress net;
extern IPAddress gw;
extern bool isAutofocusMotorized;
extern bool is_accesspoint;
extern int httpPort;
extern int streamPort;
extern char httpURL[];
extern char streamURL[];
extern char default_index[];
extern unsigned long imagesServed;
extern int myRotation;
extern int minFrameTime;
extern int lampVal;
extern int pwmVal;
extern bool filesystem;
extern String critERR;

extern unsigned long xclk;
extern int sensorPID;
extern int sdInitialized;
extern bool isTimelapseGeneral;
extern bool isStack;
extern int lampChannel;
const int pwmfreq = 50000; // 50K pwm frequency
extern int pwmresolution;

bool IS_STREAM_PAUSE = false;

extern int timelapseInterval;
extern int autofocusInterval;
extern bool sendToGithubFlag;
const char *indexFileName = "/index.txt";
typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void startCameraServer(int hPort, int sPort);
void stopCameraServer();
void loadSettings();
int autoFocus(int minPos = -300, int maxPos = 300, int focusStep = 25);

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// Webserver functions
uint16_t websocket_server_port = 80;
using namespace websockets;
extern WebsocketsClient client;
bool socketConnected = false;

// Flag that can be set to kill all active streams
bool streamKill;

/*
Websocket Stuff
*/
void onEventsCallback(WebsocketsEvent event, String data)
{
    if (event == WebsocketsEvent::ConnectionOpened)
    {
        Serial.println("Websocket Connection Opened");
        socketConnected = true;
    }
    else if (event == WebsocketsEvent::ConnectionClosed)
    {
        Serial.println("Websocket Connection Closed");
        socketConnected = false;
        ESP.restart();
    }
}

void onMessageCallback(WebsocketsMessage message)
{
    String data = message.data();
    int index = data.indexOf("=");
    if (index != -1)
    {
        String key = data.substring(0, index);
        String value = data.substring(index + 1);

        if (key == "MOVE_FOCUS")
        {
            Serial.println("Moving focus");
        }

        Serial.print("Key: ");
        Serial.println(key);
        Serial.print("Value: ");
        Serial.println(value);
    }
}

/*
Server Stuff
*/

esp_err_t bitmap_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    camera_config_t config;
    config.pixel_format = PIXFORMAT_RAW;

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->pixformat = PIXFORMAT_RAW;

    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("CAPTURE: failed to acquire frame");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_RGB565)
    {
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }
    else
    {
        res = ESP_FAIL;
        Serial.println("Capture Error: Non-RGB565 image returned by camera module");
    }

    esp_camera_fb_return(fb);
    fb = NULL;

    sensor->pixformat = PIXFORMAT_JPEG;

    return res;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    Serial.println("Capture Requested");
    setLamp(lampVal);
    delay(75); // coupled with the status led flash this gives ~150ms for lamp to settle.

    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("CAPTURE: failed to acquire frame");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    // camera_fb_t* fb = convolution(fb_);

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG)
    {
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }
    else
    {
        res = ESP_FAIL;
        Serial.println("Capture Error: Non-JPEG image returned by camera module");
    }
    esp_camera_fb_return(fb);
    // esp_camera_fb_return(fb_);
    fb = NULL;

    // save to SD card if existent
    String filename = "/image" + String(imagesServed);
    saveImage(filename);

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\r\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    imagesServed++;
    setFrameIndex(SPIFFS, imagesServed);
    return res;
}

bool IS_STREAMING = false;

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    streamKill = false;

    Serial.println("Stream requested");
    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        Serial.println("STREAM: failed to set HTTP response type");
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (res == ESP_OK)
    {
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    // handle timout from browser
    int64_t last_successful_frame_time = esp_timer_get_time();
    const int64_t TIMEOUT_THRESHOLD = 2000000; // 2 seconds in microseconds

    while (true)
    {

        IS_STREAMING = true;
        // in case we need to pause the stream since we save the images?
        while (IS_STREAM_PAUSE)
        {
            delay(10);
            IS_STREAMING = false;
        }

        fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("STREAM: failed to acquire frame");
            res = ESP_FAIL;
        }
        else
        {
            if (fb->format != PIXFORMAT_JPEG)
            {
                Serial.println("STREAM: Non-JPEG frame returned by camera module");
                res = ESP_FAIL;
            }
            else
            {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
                // Serial.println(_jpg_buf_len);
            }
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        else
        {
            log_d("STREAM: corrupted frame");
            // ESP.restart();
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            last_successful_frame_time = esp_timer_get_time(); // Update the time when a frame is successfully sent
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            // This is the error exit point from the stream loop.
            // We end the stream here only if a Hard failure has been encountered or the connection has been interrupted.
            Serial.printf("Stream failed, code = %i : %s\r\n", res, esp_err_to_name(res));
            break;
        }
        if ((res != ESP_OK) || streamKill)
        {
            // We end the stream here when a kill is signalled.
            Serial.printf("Stream killed\r\n");
            break;
        }
        int64_t frame_time = esp_timer_get_time() - last_frame;
        frame_time /= 1000;
        int32_t frame_delay = (minFrameTime > frame_time) ? minFrameTime - frame_time : 0;
        delay(frame_delay);

        last_frame = esp_timer_get_time();
        int64_t current_time = esp_timer_get_time();
        if (current_time - last_successful_frame_time > TIMEOUT_THRESHOLD)
        {
            Serial.printf("Stream timeout\r\n");
            break; // End the stream if the timeout threshold is exceeded
        }
    }

    Serial.println("Stream ended");
    last_frame = 0;
    IS_STREAMING = false;
    return res;
}

bool isStreaming = false;

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;
    char variable[32] = {
        0,
    };
    char value[32] = {
        0,
    };

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1)
    {
        buf = (char *)malloc(buf_len);
        if (!buf)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK)
            {
            }
            else
            {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        }
        else
        {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    }
    else
    {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    if (critERR.length() > 0)
        return httpd_resp_send_500(req);

    int val = atoi(value);
    sensor_t *s = esp_camera_sensor_get();
    int res = 0;
    Serial.print("CMD: var= ");
    Serial.print(variable);
    Serial.println(value);
    if (!strcmp(variable, "framesize"))
    {
        if (s->pixformat == PIXFORMAT_JPEG)
            res = s->set_framesize(s, (framesize_t)val);
    }
    else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "xclk"))
    {
        xclk = val;
        res = s->set_xclk(s, LEDC_TIMER_0, val);
    }
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "stack"))
    {
        Serial.print("Changing stack is enable to: ");
        Serial.println(val);
        setAcquireStack(SPIFFS, val);
        isStack = val;
    }
    else if (!strcmp(variable, "isTimelapse"))
    {
        Serial.print("Changing timelapse is enable to: ");
        Serial.println(val);
        isTimelapseGeneral = val;
        setIsTimelapseGeneral(SPIFFS, val);
    }
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val);
    else if (!strcmp(variable, "rotate"))
        myRotation = val;
    else if (!strcmp(variable, "min_frame_time"))
        minFrameTime = val;
    else if (!strcmp(variable, "lamp") && (lampVal != -1))
    {
        lampVal = constrain(val, 0, 100);
    }
    // control focus motor
    else if (!strcmp(variable, "mFocusUp"))
    {
        val = 50;
        Serial.print("Moving focus up");
        Serial.println(val);
        moveFocusRelative(val, true);
    }
    else if (!strcmp(variable, "mFocusDown"))
    {
        val = -50;
        Serial.print("Moving focus down");
        Serial.println(val);
        moveFocusRelative(val, true);
    }
    else if (!strcmp(variable, "mAutoFocus"))
    {
        Serial.print("Autofocus with default settings");
        autoFocus(-500, 500, 25);
    }
    else if (!strcmp(variable, "autofocus"))
    { // http://192.168.4.1/control?var=autofocus&val=-100;100;2
        // http://192.168.137.108//control?var=autofocus&val=419;512;2

        int posMin = -100;
        int posMax = 100;
        int posSteps = 20;
        // Extract three integers separated by semicolons.
        int parsedValues = sscanf(value, "%d;%d;%d", &posMin, &posMax, &posSteps);
        if (parsedValues == 3)
        { // Ensure all three integers were found and parsed
            printf("posMin: %d, posMax: %d, posSteps: %d\n", posMin, posMax, posSteps);
        }
        else
        {
            // Handle error: the format was not as expected.
            printf("Failed to parse all values.\n");
        }
        Serial.print("Moving focus at speed");
        Serial.println(val);
        autoFocus(posMin, posMax, posSteps);
    }
    else if (!strcmp(variable, "focusSlider"))
    {
        // http://192.168.137.198/control?var=focusSlider&val=1000
        Serial.print("Moving focus at speed");
        Serial.println(val);
    }
    else if (!strcmp(variable, "pwm") && (pwmVal != -1))
    {
        ledcSetup(lampChannel, pwmfreq, pwmresolution); // configure LED PWM channel
        ledcAttachPin(LAMP_PIN, lampChannel);           // attach the GPIO pin to the channel

        pwmVal = val; // constrain(val, 0, 100);
        setPWMVal(SPIFFS, pwmVal);
        setPWM(pwmVal);
    }
    else if (!strcmp(variable, "neopixel"))
    {
        setNeopixel(val);
    }
    else if (!strcmp(variable, "timelapseInterval"))
    {
        Serial.print("Changing timelapse interval to: ");
        Serial.println(val);
        timelapseInterval = val;
        setTimelapseInterval(SPIFFS, timelapseInterval);
    }
    else if (!strcmp(variable, "autofocusInterval"))
    {
        Serial.print("Changing autofocus interval to: ");
        Serial.println(val);
        autofocusInterval = val;
        setAutofocusInterval(SPIFFS, autofocusInterval);
    }
    else if (!strcmp(variable, "save_prefs"))
    {
        if (filesystem)
        {
            writePrefsToSSpiffs(SPIFFS);
            printPrefs(SPIFFS);
        }
        else
        {
            log_d("No filesystem, not saving prefs");
        }
    }
    else if (!strcmp(variable, "clear_prefs"))
    {
        if (filesystem)
            removePrefs(SPIFFS);
    }
    else if (!strcmp(variable, "reboot"))
    {
        if (lampVal != -1)
            setLamp(0); // kill the lamp; otherwise it can remain on during the soft-reboot
        if (pwmVal != -1)
            setPWM(0);
        esp_task_wdt_init(3, true); // schedule a a watchdog panic event for 3 seconds in the future
        esp_task_wdt_add(NULL);
        periph_module_disable(PERIPH_I2C0_MODULE); // try to shut I2C down properly
        periph_module_disable(PERIPH_I2C1_MODULE);
        periph_module_reset(PERIPH_I2C0_MODULE);
        periph_module_reset(PERIPH_I2C1_MODULE);
        Serial.print("REBOOT requested");
    }
    else
    {
        res = -1;
    }
    if (res)
    {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t mac_handler(httpd_req_t *req)
{
    uint8_t mac[6];
    WiFi.macAddress(mac);
    // Create the filename string
    char macaddress[32];
    sprintf(macaddress, "%02X%02X%02X%02X%02X%02X.jpg", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Construct a random URL
    static char json_response[1024];
    char *p = json_response;
    *p++ = '{';
    // Do not get attempt to get sensor when in error; causes a panic..
    p += sprintf(p, "\"mac\":%d,", macaddress);

    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t status_handler(httpd_req_t *req)
{

    static char json_response[1024];
    char *p = json_response;
    *p++ = '{';

    // FIXME: We should have an indicator if SD card was properly mounted

    // Do not get attempt to get sensor when in error; causes a panic..
    if (critERR.length() == 0)
    {
        sensor_t *s = esp_camera_sensor_get();
        p += sprintf(p, "\"lamp\":%d,", lampVal);
        p += sprintf(p, "\"min_frame_time\":%d,", minFrameTime);
        p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
        p += sprintf(p, "\"quality\":%u,", s->status.quality);
        p += sprintf(p, "\"xclk\":%u,", xclk);
        p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
        p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
        p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
        p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
        p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
        p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
        p += sprintf(p, "\"awb\":%u,", s->status.awb);
        p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
        p += sprintf(p, "\"aec\":%u,", s->status.aec);
        p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
        p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
        p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
        p += sprintf(p, "\"agc\":%u,", s->status.agc);
        p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
        p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
        p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
        p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
        p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
        p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
        p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
        p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
        p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
        p += sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
        p += sprintf(p, "\"cam_name\":\"%s\",", myName);
        p += sprintf(p, "\"code_ver\":\"%s\",", myVer);
        p += sprintf(p, "\"rotate\":\"%d\",", myRotation);
        p += sprintf(p, "\"stream_url\":\"%s\",", streamURL);
        p += sprintf(p, "\"sd_card\":%d,", sdInitialized);
        // p += sprintf(p, "\"compiled_date\":%llu,", compileDate);
        p += sprintf(p, "\"isStack\":\"%u\",", isStack);
        p += sprintf(p, "\"images_served\":\"%u\",", imagesServed);
        p += sprintf(p, "\"isTimelapseGeneral\":\"%u\",", isTimelapseGeneral);
        p += sprintf(p, "\"focusSlider\":\"%d\",", 1);
        p += sprintf(p, "\"timelapseInterval\":\"%d\",", timelapseInterval);
        p += sprintf(p, "\"autofocusInterval\":\"%d\"", autofocusInterval);
    }
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t info_handler(httpd_req_t *req)
{
    static char json_response[256];
    char *p = json_response;
    *p++ = '{';
    p += sprintf(p, "\"cam_name\":\"%s\",", myName);
    p += sprintf(p, "\"rotate\":\"%d\",", myRotation);
    p += sprintf(p, "\"stream_url\":\"%s\"", streamURL);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t favicon_16x16_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)favicon_16x16_png, favicon_16x16_png_len);
}

static esp_err_t favicon_32x32_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)favicon_32x32_png, favicon_32x32_png_len);
}

static esp_err_t favicon_ico_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)favicon_ico, favicon_ico_len);
}

static esp_err_t logo_svg_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)logo_svg, logo_svg_len);
}

static esp_err_t dump_handler(httpd_req_t *req)
{
    Serial.println("\r\nDump requested via Web");
    static char dumpOut[2000] = "";
    char *d = dumpOut;
    // Header
    d += sprintf(d, "<html><head><meta charset=\"utf-8\">\n");
    d += sprintf(d, "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n");
    d += sprintf(d, "<title>%s - Status</title>\n", myName);
    d += sprintf(d, "<link rel=\"icon\" type=\"image/png\" sizes=\"32x32\" href=\"/favicon-32x32.png\">\n");
    d += sprintf(d, "<link rel=\"icon\" type=\"image/png\" sizes=\"16x16\" href=\"/favicon-16x16.png\">\n");
    d += sprintf(d, "<link rel=\"stylesheet\" type=\"text/css\" href=\"/style.css\">\n");
    d += sprintf(d, "</head>\n");
    d += sprintf(d, "<body>\n");
    d += sprintf(d, "<img src=\"/logo.svg\" style=\"position: relative; float: right;\">\n");
    if (critERR.length() > 0)
    {
        d += sprintf(d, "%s<hr>\n", critERR.c_str());
    }
    d += sprintf(d, "<h1>ESP32 Cam Webserver</h1>\n");
    // Module
    d += sprintf(d, "Name: %s<br>\n", myName);
    d += sprintf(d, "Firmware: %s (base: %s)<br>\n", myVer, baseVersion);
    d += sprintf(d, "ESP sdk: %s<br>\n", ESP.getSdkVersion());
    // Network
    d += sprintf(d, "<h2>WiFi</h2>\n");
    d += sprintf(d, "Mode: Client<br>\n");
    String ssidName = WiFi.SSID();
    d += sprintf(d, "SSID: %s<br>\n", ssidName.c_str());
    d += sprintf(d, "Rssi: %i<br>\n", WiFi.RSSI());
    String bssid = WiFi.BSSIDstr();
    d += sprintf(d, "BSSID: %s<br>\n", bssid.c_str());
    d += sprintf(d, "IP address: %d.%d.%d.%d<br>\n", ip[0], ip[1], ip[2], ip[3]);
    if (!is_accesspoint)
    {
        d += sprintf(d, "Netmask: %d.%d.%d.%d<br>\n", net[0], net[1], net[2], net[3]);
        d += sprintf(d, "Gateway: %d.%d.%d.%d<br>\n", gw[0], gw[1], gw[2], gw[3]);
    }
    d += sprintf(d, "Http port: %i, Stream port: %i<br>\n", httpPort, streamPort);
    byte mac[6];
    WiFi.macAddress(mac);
    d += sprintf(d, "MAC: %02X:%02X:%02X:%02X:%02X:%02X<br>\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // System
    d += sprintf(d, "<h2>System</h2>\n");

    int64_t sec = esp_timer_get_time() / 1000000;
    int64_t upDays = int64_t(floor(sec / 86400));
    int upHours = int64_t(floor(sec / 3600)) % 24;
    int upMin = int64_t(floor(sec / 60)) % 60;
    int upSec = sec % 60;

    d += sprintf(d, "Up: %" PRId64 ":%02i:%02i:%02i (d:h:m:s)<br>\n", upDays, upHours, upMin, upSec);
    d += sprintf(d, "CPU Freq: %i MHz, Xclk Freq: %i MHz<br>\n", ESP.getCpuFreqMHz(), xclk);
    d += sprintf(d, "<span title=\"NOTE: Internal temperature sensor readings can be innacurate on the ESP32-c1 chipset, and may vary significantly between devices!\">");
    d += sprintf(d, "Heap: %i, free: %i, min free: %i, max block: %i<br>\n", ESP.getHeapSize(), ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
    if (psramFound())
    {
        d += sprintf(d, "Psram: %i, free: %i, min free: %i, max block: %i<br>\n", ESP.getPsramSize(), ESP.getFreePsram(), ESP.getMinFreePsram(), ESP.getMaxAllocPsram());
    }
    else
    {
        d += sprintf(d, "Psram: <span style=\"color:red;\">Not found</span>, please check your board configuration.<br>\n");
        d += sprintf(d, "- High resolution/quality images & streams will show incomplete frames due to low memory.<br>\n");
    }
    if (filesystem && (SPIFFS.totalBytes() > 0))
    {
        d += sprintf(d, "Spiffs: %i, used: %i<br>\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());
    }
    else
    {
        d += sprintf(d, "Spiffs: <span style=\"color:red;\">No filesystem found</span>, please check your board configuration.<br>\n");
        d += sprintf(d, "- saving and restoring camera settings will not function without this.<br>\n");
    }

    // Footer
    d += sprintf(d, "<br><div class=\"input-group\">\n");
    d += sprintf(d, "<button title=\"Instant Refresh; the page reloads every minute anyway\" onclick=\"location.replace(document.URL)\">Refresh</button>\n");
    d += sprintf(d, "<button title=\"Force-stop all active streams on the camera module\" ");
    d += sprintf(d, "onclick=\"let throwaway = fetch('stop');setTimeout(function(){\nlocation.replace(document.URL);\n}, 200);\">Kill Stream</button>\n");
    d += sprintf(d, "<button title=\"Close this page\" onclick=\"javascript:window.close()\">Close</button>\n");
    d += sprintf(d, "</div>\n</body>\n");
    // A javascript timer to refresh the page every minute.
    d += sprintf(d, "<script>\nsetTimeout(function(){\nlocation.replace(document.URL);\n}, 60000);\n");
    d += sprintf(d, "</script>\n</html>\n");
    *d++ = 0;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, dumpOut, strlen(dumpOut));
}

static esp_err_t stop_handler(httpd_req_t *req)
{
    Serial.println("\r\nStream stop requested via Web");
    streamKill = true;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t style_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)style_css, style_css_len);
}

static esp_err_t streamviewer_handler(httpd_req_t *req)
{

    Serial.println("Stream viewer requested");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)streamviewer_html, streamviewer_html_len);
}

static esp_err_t error_handler(httpd_req_t *req)
{
    Serial.println("Sending error page");
    std::string s(error_html);
    size_t index;
    while ((index = s.find("<APPURL>")) != std::string::npos)
        s.replace(index, strlen("<APPURL>"), httpURL);
    while ((index = s.find("<CAMNAME>")) != std::string::npos)
        s.replace(index, strlen("<CAMNAME>"), myName);
    while ((index = s.find("<ERRORTEXT>")) != std::string::npos)
        s.replace(index, strlen("<ERRORTEXT>"), critERR.c_str());
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)s.c_str(), s.length());
}

esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP GET EVENT DATA: %s", (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

void saveCapturedImageiNaturalist()
{
    /* Write a function that uploads an image using the iNaturalist REST API
     */
    Serial.println("Performing Saving Capture for iNaturalist in Background");
    // choose smaller pixel number
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA]
}

static esp_err_t uploadgithub_handler(httpd_req_t *req)
{
    streamKill = true;
    Serial.println("Sending upload page");
    sendToGithubFlag = true;
    Serial.println("Github Upload Requested");
    return 0;
}

// HTTP handler to serve the file for download
// e.g. http://192.168.4.1/downloadfile?filename=image661.jpg
esp_err_t downloadfile_handler(httpd_req_t *req)
{
    log_d("Downloading file");
    log_d("Request URI: %s", req->uri);

    char filename[128];
    size_t filename_len = httpd_req_get_url_query_len(req) + 1;
    log_d("filename_len %i", filename_len);
    if (filename_len > 1)
    {
        char *buf = (char *)malloc(filename_len);
        log_d("filename: %c", buf);
        if (httpd_req_get_url_query_str(req, buf, filename_len) == ESP_OK)
        {
            if (httpd_query_key_value(buf, "filename", filename, sizeof(filename)) == ESP_OK)
            {
                free(buf);

                fs::FS &fs = SD;
                String filePath = "/" + String(filename);
                log_d("Download file path: %s", filePath.c_str());
                File file = fs.open(filePath);
                if (!file)
                {
                    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
                    return ESP_FAIL;
                }

                httpd_resp_set_type(req, "application/octet-stream");
                httpd_resp_set_hdr(req, "Content-Disposition", ("attachment; filename=" + String(filename)).c_str());
                httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

                const size_t bufferSize = 512;
                uint8_t buffer[bufferSize];
                size_t bytesRead;

                while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0)
                {
                    if (httpd_resp_send_chunk(req, (const char *)buffer, bytesRead) != ESP_OK)
                    {
                        file.close();
                        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File download failed");
                        return ESP_FAIL;
                    }
                }

                file.close();
                httpd_resp_send_chunk(req, NULL, 0); // Send the last chunk to close the stream
                return ESP_OK;
            }
        }
        free(buf);
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
    return ESP_FAIL;
}

static esp_err_t files_handler(httpd_req_t *req)
{
    // Scanning files on the SD card
    fs::FS &fs = SD;
    File file = fs.open(indexFileName);
    if (!file)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    const size_t bufferSize = 512;
    uint8_t buffer[bufferSize];
    size_t bytesRead;

    while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0)
    {
        if (httpd_resp_send_chunk(req, (const char *)buffer, bytesRead) != ESP_OK)
        {
            file.close();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File streaming failed");
            return ESP_FAIL;
        }
    }

    file.close();
    httpd_resp_send_chunk(req, NULL, 0); // Send the last chunk to close the stream
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;
    char view[32] = {
        0,
    };

    // See if we have a specific target (full/simple/portal) and serve as appropriate
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1)
    {
        buf = (char *)malloc(buf_len);
        if (!buf)
        {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
        {
            if (httpd_query_key_value(buf, "view", view, sizeof(view)) == ESP_OK)
            {
            }
            else
            {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        }
        else
        {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    }
    else
    {
        // no target specified; default.
        strcpy(view, default_index);
    }

    if (strncmp(view, "simple", sizeof(view)) == 0)
    {
        Serial.println("Simple index page requested");
        if (critERR.length() > 0)
            return error_handler(req);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "identity");
        return httpd_resp_send(req, (const char *)index_simple_html, index_simple_html_len);
    }
    else if (strncmp(view, "full", sizeof(view)) == 0)
    {
        Serial.println("Full index page requested");
        if (critERR.length() > 0)
            return error_handler(req);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "identity");
        if (sensorPID == OV3660_PID)
        {
            return httpd_resp_send(req, (const char *)index_ov3660_html, index_ov3660_html_len);
        }
        return httpd_resp_send(req, (const char *)index_ov2640_html, index_ov2640_html_len);
    }
    else if (strncmp(view, "portal", sizeof(view)) == 0)
    {
        // Prototype captive portal landing page.
        Serial.println("Portal page requested");
        std::string s(portal_html);
        size_t index;
        while ((index = s.find("<APPURL>")) != std::string::npos)
            s.replace(index, strlen("<APPURL>"), httpURL);
        while ((index = s.find("<STREAMURL>")) != std::string::npos)
            s.replace(index, strlen("<STREAMURL>"), streamURL);
        while ((index = s.find("<CAMNAME>")) != std::string::npos)
            s.replace(index, strlen("<CAMNAME>"), myName);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "identity");
        return httpd_resp_send(req, (const char *)s.c_str(), s.length());
    }
    else
    {
        Serial.print("Unknown page requested: ");
        Serial.println(view);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
}

static esp_err_t gallery_handler(httpd_req_t *req)
{
    Serial.println("Gallery index page requested");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)index_gallery_html, index_gallery_html_len);
}

void startCameraServer(int hPort, int sPort)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20; // we use more than the default 8 (on port 80)

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL};
    httpd_uri_t index_gallery_html_uri = {
        .uri = "/gallery",
        .method = HTTP_GET,
        .handler = gallery_handler,
        .user_ctx = NULL};
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL};
    httpd_uri_t mac_uri = {
        .uri = "/mac",
        .method = HTTP_GET,
        .handler = mac_handler,
        .user_ctx = NULL};
    httpd_uri_t cmd_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL};
    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL};
    httpd_uri_t bitmap_uri = {
        .uri = "/bitmap",
        .method = HTTP_GET,
        .handler = bitmap_handler,
        .user_ctx = NULL};
    httpd_uri_t style_uri = {
        .uri = "/style.css",
        .method = HTTP_GET,
        .handler = style_handler,
        .user_ctx = NULL};
    httpd_uri_t favicon_16x16_uri = {
        .uri = "/favicon-16x16.png",
        .method = HTTP_GET,
        .handler = favicon_16x16_handler,
        .user_ctx = NULL};
    httpd_uri_t favicon_32x32_uri = {
        .uri = "/favicon-32x32.png",
        .method = HTTP_GET,
        .handler = favicon_32x32_handler,
        .user_ctx = NULL};
    httpd_uri_t favicon_ico_uri = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = favicon_ico_handler,
        .user_ctx = NULL};
    httpd_uri_t logo_svg_uri = {
        .uri = "/logo.svg",
        .method = HTTP_GET,
        .handler = logo_svg_handler,
        .user_ctx = NULL};
    httpd_uri_t dump_uri = {
        .uri = "/dump",
        .method = HTTP_GET,
        .handler = dump_handler,
        .user_ctx = NULL};
    httpd_uri_t stop_uri = {
        .uri = "/stop",
        .method = HTTP_GET,
        .handler = stop_handler,
        .user_ctx = NULL};
    httpd_uri_t stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};
    httpd_uri_t streamviewer_uri = {
        .uri = "/view",
        .method = HTTP_GET,
        .handler = streamviewer_handler,
        .user_ctx = NULL};
    httpd_uri_t info_uri = {
        .uri = "/info",
        .method = HTTP_GET,
        .handler = info_handler,
        .user_ctx = NULL};
    httpd_uri_t error_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = error_handler,
        .user_ctx = NULL};
    httpd_uri_t viewerror_uri = {
        .uri = "/view",
        .method = HTTP_GET,
        .handler = error_handler,
        .user_ctx = NULL};
    httpd_uri_t files_uri = {
        .uri = "/files",
        .method = HTTP_GET,
        .handler = files_handler,
        .user_ctx = NULL};
    httpd_uri_t downloadfile_uri = {
        .uri = "/downloadfile",
        .method = HTTP_GET,
        .handler = downloadfile_handler,
        .user_ctx = NULL};
    httpd_uri_t uploadgithub_uri = {
        .uri = "/uploadgithub",
        .method = HTTP_GET,
        .handler = uploadgithub_handler,
        .user_ctx = NULL};

    // Request Handlers; config.max_uri_handlers (above) must be >= the number of handlers
    config.server_port = hPort;
    config.ctrl_port = hPort;
    // Serial.printf("Starting web server on port: '%d'\r\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK)
    {
        if (critERR.length() > 0)
        {
            httpd_register_uri_handler(camera_httpd, &error_uri);
        }
        else
        {
            httpd_register_uri_handler(camera_httpd, &index_uri);
            httpd_register_uri_handler(camera_httpd, &cmd_uri);
            httpd_register_uri_handler(camera_httpd, &status_uri);
            httpd_register_uri_handler(camera_httpd, &capture_uri);
            httpd_register_uri_handler(camera_httpd, &index_gallery_html_uri);
        }

        httpd_register_uri_handler(camera_httpd, &style_uri);
        httpd_register_uri_handler(camera_httpd, &favicon_16x16_uri);
        httpd_register_uri_handler(camera_httpd, &favicon_32x32_uri);
        httpd_register_uri_handler(camera_httpd, &favicon_ico_uri);
        httpd_register_uri_handler(camera_httpd, &logo_svg_uri);
        httpd_register_uri_handler(camera_httpd, &dump_uri);
        httpd_register_uri_handler(camera_httpd, &stop_uri);
        httpd_register_uri_handler(camera_httpd, &uploadgithub_uri);
        httpd_register_uri_handler(camera_httpd, &mac_uri);
        httpd_register_uri_handler(camera_httpd, &files_uri);
        httpd_register_uri_handler(camera_httpd, &downloadfile_uri);
    }

    config.server_port = sPort;
    config.ctrl_port = sPort;
    // Serial.printf("Starting stream server on port: '%d'\r\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        if (critERR.length() > 0)
        {
            httpd_register_uri_handler(camera_httpd, &error_uri);
            httpd_register_uri_handler(camera_httpd, &viewerror_uri);
        }
        else
        {
            httpd_register_uri_handler(stream_httpd, &stream_uri);
            httpd_register_uri_handler(stream_httpd, &info_uri);
            httpd_register_uri_handler(stream_httpd, &streamviewer_uri);
        }
        httpd_register_uri_handler(stream_httpd, &favicon_16x16_uri);
        httpd_register_uri_handler(stream_httpd, &favicon_32x32_uri);
        httpd_register_uri_handler(stream_httpd, &favicon_ico_uri);
    }
}

int autoFocus(int minPos, int maxPos, int focusStep)
{
    if (isAutofocusMotorized)
    {

        int maxFocusValue = 0;

        // Move to start position
        int bestPosition = minPos;
        int range = maxPos - minPos;
        moveFocusRelative(minPos, false);

        // Scan through range and measure focus quality
        int lastFBSize = 0;
        int focusValue = 0;
        for (int iFocus = 0; iFocus <= range / focusStep; iFocus++)
        {
            if (iFocus > 0)
                moveFocusRelative(focusStep, false);
            // Pause the stream for a moment
            IS_STREAM_PAUSE = true;

            // wait until stream pauses
            while (IS_STREAMING)
            {
                delay(10);
            }

            for (int iFrame = 0; iFrame < 100; iFrame++)
            {
                // Capture image and get the focus quality
                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb)
                {
                    Serial.println("Camera capture failed");
                    return -1;
                }

                focusValue = fb->len;
                esp_camera_fb_return(fb);
                if (focusValue != lastFBSize)
                { // sometmies the same image is returned twice..not sure why..
                    break;
                }
                else
                {
                    focusValue = -1;
                }
            }
            IS_STREAM_PAUSE = false;
            Serial.println("Focus value; " + String(focusValue) + "; at position; " + String(minPos + iFocus * focusStep) + "; of " + String(range) + "");
            lastFBSize = focusValue;
            // Check if this focus is better than previous best
            if (focusValue > maxFocusValue)
            {
                maxFocusValue = focusValue;
                bestPosition = iFocus * focusStep;
            }
        }

        // Move back to best position
        moveFocusRelative(-range, false);
        delay(1000);
        moveFocusRelative(bestPosition, false);
        Serial.println("Autofocus complete. Best position: " + String(bestPosition));
        return bestPosition;
    }
    else
    {
        // This is for voice-coil based autofocus
        int maxFocusValue = 0;
        int bestPosition = 0;
        int range = maxPos - minPos;
        int lastFBSize = 0;
        int focusValue = 0;

        // Scan through range and measure focus quality
        for (int i = 0; i <= range / focusStep; i++)
        {
            log_i("Autofocus: %d", i);
            setPWM(minPos + i * focusStep);
            delay(20);
            // Pause the stream for a moment
            IS_STREAM_PAUSE = true;

            // wait until stream pauses
            while (IS_STREAMING)
            {
                delay(10);
            }

            // Capture image and get the focus quality
            for (int j = 0; j < 5; j++)
            {
                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb)
                {
                    Serial.println("Camera capture failed");
                    return -1;
                }

                focusValue = fb->len;
                esp_camera_fb_return(fb);
                if (focusValue != lastFBSize)
                {
                    break;
                }
            }
            IS_STREAM_PAUSE = false;
            lastFBSize = focusValue;
            Serial.println("Focus value: " + String(focusValue) + " at position " + String(minPos + i * focusStep) + " of " + String(range) + "");
            // Check if this focus is better than previous best
            if (focusValue > maxFocusValue)
            {
                maxFocusValue = focusValue;
                bestPosition = i;
            }
        }

        // Move back to best position
        setPWM(minPos + bestPosition * focusStep);
        Serial.println("Autofocus complete. Best position: " + String(bestPosition));
        return bestPosition;
    }
}

bool saveImage(String filename, int pwmVal)
{
    // Pause the stream for a moment
    IS_STREAM_PAUSE = true;

    // wait until stream pauses
    while (IS_STREAMING)
    {
        delay(10);
    }
 
    // set PWM value e.g.
    if (pwmVal > -1)
    {
        setPWM(pwmVal);
    }

    bool res = false;
    // TODO: We need to stop the stream here!
    if (sdInitialized)
    { // Do not attempt to save anything to a non-existig SD card

        camera_fb_t *fb = NULL;

        Serial.println("Capture Requested for SD card save");
        int64_t fr_start = esp_timer_get_time();

        fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("CAPTURE: failed to acquire frame");
            res = false;
            esp_camera_fb_return(fb);
            fb = NULL;
            return res;
        }

        size_t fb_len = 0;
        if (fb->format == PIXFORMAT_JPEG)
        {
            fb_len = fb->len;
            res = true;
        }
        else
        {
            res = false;
            Serial.println("Capture Error: Non-JPEG image returned by camera module");
        }

        int64_t fr_end = esp_timer_get_time();

        // Save image to disk
        // https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera/blob/56580ab8e438d82a91fcbe47a8412cf9d29a6b76/take_photos/take_photos.ino#L33
        fs::FS &fs = SD;
        String extension = ".jpg";
        File imgFile = fs.open((filename + extension).c_str(), FILE_WRITE);
        if (!imgFile)
        {
            Serial.println("SD: Failed to open file in writing mode");
            // reset frame buffer
            esp_camera_fb_return(fb);
            fb = NULL;
            // Resume the stream
            IS_STREAM_PAUSE = false;
            return false;
        }
        else
        {
            imgFile.write(fb->buf, fb->len);
            Serial.println("Saved " + filename);
        }
        imgFile.close();

        File indexFile;
        if (!fs.exists(indexFileName))
        {
            // create index file
            log_d("Creating %s", indexFileName);
            indexFile = fs.open(indexFileName, FILE_WRITE);
        }
        else
        {
            log_d("Appending to %s", indexFileName);
            indexFile = fs.open(indexFileName, FILE_APPEND);
        }

        // Append a string to the file
        log_d("Writing to index.txt - %s", filename + extension);
        indexFile.println(filename);
        indexFile.close();

        // reset frame buffer
        esp_camera_fb_return(fb);
        fb = NULL;

    }
    else
    {
        res = false;
    }

    // Resume the stream
    IS_STREAM_PAUSE = false;

    return res;
}

/*

Image Processing


*/

camera_fb_t *convolution(camera_fb_t *input)
{
    // Create a new dummy frame buffer for the result

    int FRAME_WIDTH = input->width;
    int FRAME_HEIGHT = input->height;
    size_t frame_size = FRAME_WIDTH * FRAME_HEIGHT * 3; // Assuming pixel format is PIXFORMAT_RGB565

    camera_fb_t *frame_buffer = (camera_fb_t *)malloc(sizeof(camera_fb_t));
    if (!frame_buffer)
    {
        log_e("Failed to allocate frame buffer");
        return NULL;
    }

    frame_buffer->width = FRAME_WIDTH;
    frame_buffer->height = FRAME_HEIGHT;
    frame_buffer->len = frame_size;
    frame_buffer->buf = (uint8_t *)malloc(frame_size);
    if (!frame_buffer->buf)
    {
        log_e("Failed to allocate frame buffer memory");
        free(frame_buffer);
        return NULL;
    }

    // Set the result frame buffer parameters
    frame_buffer->width = input->width;
    frame_buffer->height = input->height;
    frame_buffer->len = input->len;
    frame_buffer->format = input->format;
    frame_buffer->buf = new uint8_t[frame_buffer->len];

    // Perform convolution on each pixel of the input frame buffer
    int kernelSize = 7;
    float kernel[kernelSize][kernelSize] = {
        {0.0009, 0.0060, 0.0217, 0.0447, 0.0217, 0.0060, 0.0009},
        {0.0060, 0.0401, 0.1466, 0.3040, 0.1466, 0.0401, 0.0060},
        {0.0217, 0.1466, 0.5352, 1.1065, 0.5352, 0.1466, 0.0217},
        {0.0447, 0.3040, 1.1065, 2.2945, 1.1065, 0.3040, 0.0447},
        {0.0217, 0.1466, 0.5352, 1.1065, 0.5352, 0.1466, 0.0217},
        {0.0060, 0.0401, 0.1466, 0.3040, 0.1466, 0.0401, 0.0060},
        {0.0009, 0.0060, 0.0217, 0.0447, 0.0217, 0.0060, 0.0009}};

    for (int y = kernelSize / 2; y < input->height - kernelSize / 2; y++)
    {
        for (int x = kernelSize / 2; x < input->width - kernelSize / 2; x++)
        {
            float sum = 0;

            // Apply the convolution kernel to the pixel neighborhood
            for (int ky = 0; ky < kernelSize; ky++)
            {
                for (int kx = 0; kx < kernelSize; kx++)
                {
                    int pixel = input->buf[(y + ky - kernelSize / 2) * input->width + (x + kx - kernelSize / 2)];
                    sum += kernel[ky][kx] * pixel;
                }
            }

            // Ensure the pixel value is within the valid range
            sum = min(max(sum, (float)0.), (float)255.);

            // Set the convolved pixel value in the result frame buffer
            frame_buffer->buf[y * input->width + x] = sum;
        }
    }

    return frame_buffer;
}