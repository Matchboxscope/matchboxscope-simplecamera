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

#include <esp_http_server.h>
#include "esp_http_client.h"
#include <esp_timer.h>
#include <esp_camera.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <SD_MMC.h>
#include <SPIFFS.h>
#include <FS.h>
#include "device_pref.h"

#include "index_ov2640.h"
#include "index_ov3660.h"
#include "index_other.h"
#include "css.h"
#include "favicons.h"
#include "logo.h"
#include "storage.h"

#include "githubtoken.h"

// Functions from the main .ino
extern void flashLED(int flashtime);
extern void setLamp(int newVal);
extern void setPWM(int newVal);
extern void printLocalTime(bool extraData);
extern void loadPrefs(fs::FS &fs);

bool saveImage(String filename, int lensValue=-1);

// External variables declared in the main .ino
extern char myName[];
extern char myVer[];
extern char baseVersion[];
extern IPAddress ip;
extern IPAddress net;
extern IPAddress gw;
extern bool accesspoint;
extern char apName[];
extern bool captivePortal;
extern int httpPort;
extern int streamPort;
extern char httpURL[];
extern char streamURL[];
extern char default_index[];
extern int8_t streamCount;
extern unsigned long streamsServed;
extern unsigned long imagesServed;
extern int myRotation;
extern int minFrameTime;
extern int lampVal;
extern bool autoLamp;
extern int pwmVal;
extern bool filesystem;
extern String critERR;
extern bool debugData;
extern bool haveTime;
extern int sketchSize;
extern int sketchSpace;
extern String sketchMD5;
extern bool otaEnabled;
extern char otaPassword[];
extern unsigned long xclk;
extern int sensorPID;
extern int sdInitialized;

//extern DevicePreferences device_pref_http;
Preferences pref_http;
DevicePreferences device_pref_http(pref_http, "camera", __DATE__ " " __TIME__);


extern int timelapseInterval;
extern bool sendToGithubFlag;
typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// Flag that can be set to kill all active streams
bool streamKill;

#ifdef __cplusplus
extern "C"
{
#endif
    uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

void serialDump()
{
    Serial.println();
    // Module
    Serial.printf("Name: %s\r\n", myName);
    if (haveTime)
    {
        Serial.print("Time: ");
        printLocalTime(true);
    }
    Serial.printf("Firmware: %s (base: %s)\r\n", myVer, baseVersion);
    float sketchPct = 100 * sketchSize / sketchSpace;
    Serial.printf("Sketch Size: %i (total: %i, %.1f%% used)\r\n", sketchSize, sketchSpace, sketchPct);
    Serial.printf("MD5: %s\r\n", sketchMD5.c_str());
    Serial.printf("ESP sdk: %s\r\n", ESP.getSdkVersion());
    if (otaEnabled)
    {
        if (strlen(otaPassword) != 0)
        {
            Serial.printf("OTA: Enabled, Password: %s\n\r", otaPassword);
        }
        else
        {
            Serial.printf("OTA: Enabled, No Password! (insecure)\n\r");
        }
    }
    else
    {
        Serial.printf("OTA: Disabled\n\r");
    }
    // Network
    if (accesspoint)
    {
        if (captivePortal)
        {
            Serial.printf("WiFi Mode: AccessPoint with captive portal\r\n");
        }
        else
        {
            Serial.printf("WiFi Mode: AccessPoint\r\n");
        }
        Serial.printf("WiFi SSID: %s\r\n", apName);
    }
    else
    {
        Serial.printf("WiFi Mode: Client\r\n");
        String ssidName = WiFi.SSID();
        Serial.printf("WiFi Ssid: %s\r\n", ssidName.c_str());
        Serial.printf("WiFi Rssi: %i\r\n", WiFi.RSSI());
        String bssid = WiFi.BSSIDstr();
        Serial.printf("WiFi BSSID: %s\r\n", bssid.c_str());
    }
    Serial.printf("WiFi IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
    if (!accesspoint)
    {
        Serial.printf("WiFi Netmask: %d.%d.%d.%d\r\n", net[0], net[1], net[2], net[3]);
        Serial.printf("WiFi Gateway: %d.%d.%d.%d\r\n", gw[0], gw[1], gw[2], gw[3]);
    }
    Serial.printf("WiFi Http port: %i, Stream port: %i\r\n", httpPort, streamPort);
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.printf("WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // System
    int64_t sec = esp_timer_get_time() / 1000000;
    int64_t upDays = int64_t(floor(sec / 86400));
    int upHours = int64_t(floor(sec / 3600)) % 24;
    int upMin = int64_t(floor(sec / 60)) % 60;
    int upSec = sec % 60;
    int McuTc = (temprature_sens_read() - 32) / 1.8; // celsius
    int McuTf = temprature_sens_read();              // fahrenheit
    Serial.printf("System up: %" PRId64 ":%02i:%02i:%02i (d:h:m:s)\r\n", upDays, upHours, upMin, upSec);
    Serial.printf("Active streams: %i, Previous streams: %lu, Images captured: %lu\r\n", streamCount, streamsServed, imagesServed);
    Serial.printf("CPU Freq: %i MHz, Xclk Freq: %i MHz\r\n", ESP.getCpuFreqMHz(), xclk);
    Serial.printf("MCU temperature : %i C, %i F  (approximate)\r\n", McuTc, McuTf);
    Serial.printf("Heap: %i, free: %i, min free: %i, max block: %i\r\n", ESP.getHeapSize(), ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
    if (psramFound())
    {
        Serial.printf("Psram: %i, free: %i, min free: %i, max block: %i\r\n", ESP.getPsramSize(), ESP.getFreePsram(), ESP.getMinFreePsram(), ESP.getMaxAllocPsram());
    }
    else
    {
        Serial.printf("Psram: Not found; please check your board configuration.\r\n");
        Serial.printf("- High resolution/quality settings will show incomplete frames to low memory.\r\n");
    }
    // Filesystems
    if (filesystem && (SPIFFS.totalBytes() > 0))
    {
        Serial.printf("Spiffs: %i, used: %i\r\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());
    }
    else
    {
        Serial.printf("Spiffs: No filesystem found, please check your board configuration.\r\n");
        Serial.printf("- Saving and restoring camera settings will not function without this.\r\n");
    }
    Serial.println("Preferences file: ");
    dumpPrefs(SPIFFS);
    if (critERR.length() > 0)
    {
        Serial.printf("\r\n\r\nAn error or halt has occurred with Camera Hardware, see previous messages.\r\n");
        Serial.printf("A reboot is required to recover from this.\r\nError message: (html)\r\n %s\r\n\r\n", critERR.c_str());
    }
    Serial.println();
    return;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    Serial.println("Capture Requested");
    if (autoLamp && (lampVal != -1))
    {
        setLamp(lampVal);
        delay(75); // coupled with the status led flash this gives ~150ms for lamp to settle.
    }
    flashLED(75); // little flash of status LED

    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("CAPTURE: failed to acquire frame");
        httpd_resp_send_500(req);
        if (autoLamp && (lampVal != -1))
            setLamp(0);
        return ESP_FAIL;
    }

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
    fb = NULL;

    // save to SD card if existent
    String filename = "/image" + String(imagesServed) + ".jpg";
    saveImage(filename);

    int64_t fr_end = esp_timer_get_time();
    if (debugData)
    {
        Serial.printf("JPG: %uB %ums\r\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    }
    imagesServed++;
    device_pref_http.setFrameIndex(imagesServed);
    if (autoLamp && (lampVal != -1))
    {
        setLamp(0);
    }
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    streamKill = false;

    Serial.println("Stream requested");
    if (autoLamp && (lampVal != -1))
        setLamp(lampVal);
    streamCount = 1; // at present we only have one stream handler, so values are 0 or 1..
    flashLED(75);    // double flash of status LED
    delay(75);
    flashLED(75);

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        streamCount = 0;
        if (autoLamp && (lampVal != -1))
            setLamp(0);
        Serial.println("STREAM: failed to set HTTP response type");
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (res == ESP_OK)
    {
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    while (true)
    {
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
            }
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
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

        if (debugData)
        {
            Serial.printf("MJPG: %uB %ums, delay: %ums, framerate (%.1ffps)\r\n",
                          (uint32_t)(_jpg_buf_len),
                          (uint32_t)frame_time, frame_delay, 1000.0 / (uint32_t)(frame_time + frame_delay));
        }
        last_frame = esp_timer_get_time();
    }

    streamsServed++;
    streamCount = 0;
    if (autoLamp && (lampVal != -1))
        setLamp(0);
    Serial.println("Stream ended");
    last_frame = 0;
    return res;
}

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

    flashLED(75);

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
    else if (!strcmp(variable, "autolamp") && (lampVal != -1))
    {
        autoLamp = val;
        if (autoLamp)
        {
            if (streamCount > 0)
                setLamp(lampVal);
            else
                setLamp(0);
        }
        else
        {
            setLamp(lampVal);
        }
    }
    else if (!strcmp(variable, "lamp") && (lampVal != -1))
    {
        lampVal = constrain(val, 0, 100);
        if (autoLamp)
        {
            if (streamCount > 0)
                setLamp(lampVal);
            else
                setLamp(0);
        }
        else
        {
            setLamp(lampVal);
        }
    }
    else if (!strcmp(variable, "pwm") && (pwmVal != -1))
    {
        pwmVal = constrain(val, 0, 100);
        setPWM(pwmVal);
    }
    else if (!strcmp(variable, "timelapseInterval"))
    {
        Serial.print("Changing timelapse interval to: ");
        Serial.println(val);
        timelapseInterval = val;
        device_pref_http.setTimelapseInterval(timelapseInterval);
    }
    else if (!strcmp(variable, "save_prefs"))
    {
        if (filesystem)
            savePrefs(SPIFFS);
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
        while (true)
        {
            flashLED(50);
            delay(150);
            Serial.print('.');
        }
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


static esp_err_t mac_handler(httpd_req_t *req){
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


    // Do not get attempt to get sensor when in error; causes a panic..
    if (critERR.length() == 0)
    {
        sensor_t *s = esp_camera_sensor_get();
        p += sprintf(p, "\"lamp\":%d,", lampVal);
        p += sprintf(p, "\"autolamp\":%d,", autoLamp);
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
        p += sprintf(p, "\"anglerfishSlider\":\"%d\"", 1);
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
    flashLED(75);
    Serial.println("\r\nDump requested via Web");
    serialDump();
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
    float sketchPct = 100 * sketchSize / sketchSpace;
    d += sprintf(d, "Sketch Size: %i (total: %i, %.1f%% used)<br>\n", sketchSize, sketchSpace, sketchPct);
    d += sprintf(d, "MD5: %s<br>\n", sketchMD5.c_str());
    d += sprintf(d, "ESP sdk: %s<br>\n", ESP.getSdkVersion());
    // Network
    d += sprintf(d, "<h2>WiFi</h2>\n");
    if (accesspoint)
    {
        if (captivePortal)
        {
            d += sprintf(d, "Mode: AccessPoint with captive portal<br>\n");
        }
        else
        {
            d += sprintf(d, "Mode: AccessPoint<br>\n");
        }
        d += sprintf(d, "SSID: %s<br>\n", apName);
    }
    else
    {
        d += sprintf(d, "Mode: Client<br>\n");
        String ssidName = WiFi.SSID();
        d += sprintf(d, "SSID: %s<br>\n", ssidName.c_str());
        d += sprintf(d, "Rssi: %i<br>\n", WiFi.RSSI());
        String bssid = WiFi.BSSIDstr();
        d += sprintf(d, "BSSID: %s<br>\n", bssid.c_str());
    }
    d += sprintf(d, "IP address: %d.%d.%d.%d<br>\n", ip[0], ip[1], ip[2], ip[3]);
    if (!accesspoint)
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
    if (haveTime)
    {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo))
        {
            char timeStringBuff[50]; // 50 chars should be enough
            strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S, %A, %B %d %Y", &timeinfo);
            // print like "const char*"
            d += sprintf(d, "Time: %s<br>\n", timeStringBuff);
        }
    }
    int64_t sec = esp_timer_get_time() / 1000000;
    int64_t upDays = int64_t(floor(sec / 86400));
    int upHours = int64_t(floor(sec / 3600)) % 24;
    int upMin = int64_t(floor(sec / 60)) % 60;
    int upSec = sec % 60;
    int McuTc = (temprature_sens_read() - 32) / 1.8; // celsius
    int McuTf = temprature_sens_read();              // fahrenheit

    d += sprintf(d, "Up: %" PRId64 ":%02i:%02i:%02i (d:h:m:s)<br>\n", upDays, upHours, upMin, upSec);
    d += sprintf(d, "Active streams: %i, Previous streams: %lu, Images captured: %lu<br>\n", streamCount, streamsServed, imagesServed);
    d += sprintf(d, "CPU Freq: %i MHz, Xclk Freq: %i MHz<br>\n", ESP.getCpuFreqMHz(), xclk);
    d += sprintf(d, "<span title=\"NOTE: Internal temperature sensor readings can be innacurate on the ESP32-c1 chipset, and may vary significantly between devices!\">");
    d += sprintf(d, "MCU temperature : %i &deg;C, %i &deg;F</span>\n<br>", McuTc, McuTf);
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
    flashLED(75);
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
    flashLED(75);
    Serial.println("Stream viewer requested");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)streamviewer_html, streamviewer_html_len);
}

static esp_err_t error_handler(httpd_req_t *req)
{
    flashLED(75);
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
    flashLED(75); // little flash of status LED

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
    //const char *text_base64_data = "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";

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
    String url = "https://api.github.com/repos/matchboxscope/matchboxscope-gallery/contents/"+String(filename);
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
    //s->set_framesize(s, FRAMESIZE_SVGA);

}

static esp_err_t uploadgithub_handler(httpd_req_t *req)
{
    streamKill = true;
    Serial.println("Sending upload page");
    sendToGithubFlag = true;
    Serial.println("Github Upload Requested");
    return 0;
}

static esp_err_t anglerfish_handler(httpd_req_t *req)
{

    Serial.println("Entering the Anglerfish mode");
    for(int iFlash = 0; iFlash<10; iFlash++) flashLED(75);
    Serial.println("Going into deepsleep mode");

    // save all settings from gui
    savePrefs(SPIFFS);
  
    // this will set the anglerfish into a periodic deep-sleep awake timelapse
    device_pref_http.setIsTimelapse(true);
    device_pref_http.isTimelapse();
    static char json_response[1024];
    char *p = json_response;
    *p++ = '{';
    p += sprintf(p, "You have enabled long-time timelpase - remove the SD card to wake up from deepsleep mode #Schneewittchen...");
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_response, strlen(json_response));

    SD_MMC.end(); // FIXME: may cause issues when file not closed? categoreis: LED/SD-CARD issues
    delay(2000);
    ESP.restart();
    return 0;
}


static esp_err_t index_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;
    char view[32] = {
        0,
    };

    flashLED(75);
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
        // If captive portal is active send that instead
        if (captivePortal)
        {
            strcpy(view, "portal");
        }
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

void startCameraServer(int hPort, int sPort)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16; // we use more than the default 8 (on port 80)

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
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
    httpd_uri_t uploadgithub_uri = {
        .uri = "/uploadgithub",
        .method = HTTP_GET,
        .handler = uploadgithub_handler,
        .user_ctx = NULL};
    httpd_uri_t anglerfish_uri = {
        .uri = "/anglerfishmode",
        .method = HTTP_GET,
        .handler = anglerfish_handler,
        .user_ctx = NULL};

    // Request Handlers; config.max_uri_handlers (above) must be >= the number of handlers
    config.server_port = hPort;
    config.ctrl_port = hPort;
    Serial.printf("Starting web server on port: '%d'\r\n", config.server_port);
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
        httpd_register_uri_handler(camera_httpd, &anglerfish_uri);
    }

    config.server_port = sPort;
    config.ctrl_port = sPort;
    Serial.printf("Starting stream server on port: '%d'\r\n", config.server_port);
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

bool saveImage(String filename, int lensValue)
{
    // load camera preferences into camera
    loadPrefs(SPIFFS);

    // set PWM value e.g. 
    if (lensValue>-1){
        log_d("Setting PWM Value");
        setPWM(lensValue);
    }

    bool res = false;
    // TODO: We need to stop the stream here!
    if (sdInitialized)
    { // Do not attempt to save anything to a non-existig SD card

        camera_fb_t *fb = NULL;
        
        Serial.println("Capture Requested for SD card save");
        if (autoLamp && (lampVal != -1))
        {
            setLamp(lampVal);
            delay(75); // coupled with the status led flash this gives ~150ms for lamp to settle.
        }
        flashLED(75); // little flash of status LED

        int64_t fr_start = esp_timer_get_time();

        fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("CAPTURE: failed to acquire frame");
            if (autoLamp && (lampVal != -1))
                setLamp(0);
            res = false;
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
        if (debugData)
        {
            Serial.printf("JPG: %uB %ums\r\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
        }
        if (autoLamp && (lampVal != -1))
        {
            setLamp(0);
        }

        // Save image to disk
        fs::FS &fs = SD_MMC;
        String extension = ".jpg";
        File imgFile = fs.open((filename+extension).c_str(), FILE_WRITE);
        if (!imgFile)
        {
            Serial.println("Failed to open file in writing mode");
            return false;
        }
        else
        {
            imgFile.write(fb->buf, fb->len);
            Serial.println("Saved " + filename);
        }
        imgFile.close();

        // reset frame buffer
        esp_camera_fb_return(fb);
        fb = NULL;
    }
    else
    {
        res = false;
    }
    return res;
}