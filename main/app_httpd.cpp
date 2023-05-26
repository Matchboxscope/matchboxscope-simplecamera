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

#include "index_ov2640.h"
#include "index_ov3660.h"
#include "index_other.h"
#include "index_holo.h"
#include "css.h"
#include "favicons.h"
#include "logo.h"
#include "storage.h"

// Functions from the main .ino
extern void flashLED(int flashtime);
extern void loadSpiffsToPrefs(fs::FS &fs);


camera_fb_t *convolution(camera_fb_t *input);
bool saveImage(String filename);

// External variables declared in the main .ino
extern char myName[];
extern char myVer[];
extern IPAddress ip;
extern IPAddress net;
extern IPAddress gw;
extern bool is_accesspoint;
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
extern bool filesystem;
extern String critERR;
extern unsigned long xclk;
extern int sensorPID;



bool IS_STREAM_PAUSE = false;

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
    flashLED(75); // little flash of status LED

    int64_t fr_start = esp_timer_get_time();

    // globally change frame resolution to lower the bandwidth
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_UXGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA]


     // get mean intensities
    for (int iDummyFrame = 0; iDummyFrame < 5; iDummyFrame++)
    {
        // FIXME: Look at the buffer for the camera => flush vs. return
        log_d("Capturing dummy frame %i", iDummyFrame);
        fb = esp_camera_fb_get();
        if (!fb)
            log_e("Camera frame error", false);
        esp_camera_fb_return(fb);

    }
    
    fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("CAPTURE: failed to acquire frame");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    log_d("Acquire second frame");
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
    String filename = "/image" + String(imagesServed) + ".jpg";
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

    // globally change frame resolution to lower the bandwidth
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA]

    streamKill = false;

    Serial.println("Stream requested");
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
    }

    streamsServed++;
    streamCount = 0;
    Serial.println("Stream ended");
    last_frame = 0;
    return res;
}

bool isStreaming = false;

static esp_err_t stream_handler_XIAO(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128];

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    isStreaming = true;

    while (true)
    {

        fb = esp_camera_fb_get();
        if (!fb)
        {
            log_e("Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            _timestamp.tv_sec = fb->timestamp.tv_sec;
            _timestamp.tv_usec = fb->timestamp.tv_usec;
            if (fb->format != PIXFORMAT_JPEG)
            {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted)
                {
                    log_e("JPEG compression failed");
                    res = ESP_FAIL;
                }
            }
            else
            {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
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
            log_e("Send frame failed");
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t frame_time = fr_end - last_frame;
        frame_time /= 1000;
    }

    isStreaming = false;

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
    else if (!strcmp(variable, "ssid"))
    {
        Serial.print("Changing SSID to: ");
        Serial.println(value);
        setWifiSSID(SPIFFS, value);
    }
    else if (!strcmp(variable, "password"))
    {
        Serial.print("Changing password to: ");
        Serial.println(value);
        setWifiPW(SPIFFS, value);
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
        // p += sprintf(p, "\"compiled_date\":%llu,", compileDate);
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
    d += sprintf(d, "Active streams: %i, Previous streams: %lu, Images captured: %lu<br>\n", streamCount, streamsServed, imagesServed);
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

void saveCapturedImageiNaturalist()
{
    /* Write a function that uploads an image using the iNaturalist REST API
     */
    Serial.println("Performing Saving Capture for iNaturalist in Background");
    // choose smaller pixel number
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_VGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA]
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

static esp_err_t holo_handler(httpd_req_t *req)
{
    Serial.println("Holo index page requested");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "identity");
    return httpd_resp_send(req, (const char *)index_holo_html, index_holo_html_len);
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
    httpd_uri_t index_holo_html_uri = {
        .uri = "/holo",
        .method = HTTP_GET,
        .handler = holo_handler,
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
            httpd_register_uri_handler(camera_httpd, &index_holo_html_uri);
        }

        httpd_register_uri_handler(camera_httpd, &style_uri);
        httpd_register_uri_handler(camera_httpd, &favicon_16x16_uri);
        httpd_register_uri_handler(camera_httpd, &favicon_32x32_uri);
        httpd_register_uri_handler(camera_httpd, &favicon_ico_uri);
        httpd_register_uri_handler(camera_httpd, &logo_svg_uri);
        httpd_register_uri_handler(camera_httpd, &dump_uri);
        httpd_register_uri_handler(camera_httpd, &stop_uri);
        httpd_register_uri_handler(camera_httpd, &mac_uri);
        
        httpd_register_uri_handler(camera_httpd, &bitmap_uri);
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

bool saveImage(String filename)
{
    // Pause the stream for a moment
    IS_STREAM_PAUSE = true;

    // wait until stream pauses
    while (IS_STREAMING)
    {
        delay(10);
    }

    bool res = false;
    // TODO: We need to stop the stream here!
    res = false;

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



