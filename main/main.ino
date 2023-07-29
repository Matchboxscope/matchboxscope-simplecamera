#include "esp_camera.h"
#include <base64.h>

#define BAUD_RATE 2000000

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

#define LED_GPIO_NUM 21

void grabImage();
void cameraInit();

void setup()
{
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(20);
  cameraInit();
}


/* setting expsorue time: t1000
setting gain: g1
getting frame: \n
restarting: r0 */
void loop()
{
  // Check for incoming serial commands
  if (Serial.available() > 0)
  {
    String command = Serial.readString(); // Read the command until a newline character is received
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
    else if (command.length() > 0 && command.charAt(0) == 'd')
    {
      // return buffer length
      camera_fb_t *fb = NULL;
      fb = esp_camera_fb_get();

      Serial.println(fb->len);
       esp_camera_fb_return(fb);
    }
    else if (command.length() > 0 && command.charAt(0) == 'r')
    {
      // restart
      ESP.restart();
    }
    else
    {
      flushSerial();
      // capture image and return
      grabImage();
    }

    flushSerial();
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

void cameraInit()
{

  camera_config_t config;
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
  config.xclk_freq_hz = 20000000;
  // config.frame_size = FRAMESIZE_QVGA;
  // config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;

  config.pixel_format = PIXFORMAT_GRAYSCALE; // PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;        // for streaming}

  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);
  s->set_vflip(s, 1);

  // enable manual camera settings
  s->set_gain_ctrl(s, 0);     // auto gain off (1 or 0)
  s->set_exposure_ctrl(s, 0); // auto exposure off (1 or 0)
  s->set_aec_value(s, 100);   // set exposure manually (0-1200)
  s->set_agc_gain(s, 0);      // set gain manually (0 - 30)
}
void grabImage()
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
    for(int i = 0; i < 10; i++){
    fb->buf[i] = i % 2;  // Alternates between 0 and 1
    }
    // add tail to create some kind of check pattern
    for(int i = 0; i < 10; i++){ // for some very weird reason this seems to stabilize any frame missyncing on Windows - absolutely no clue - this does not even need to be decoded or backshifted?! Just its existence is enough?!!!
    fb->buf[i+ fb->len/2-10] = i%3;  // Alternates between 0 and 1
    }

    // add tail to create some kind of check pattern
    for(int i = 0; i < 10; i++){
    fb->buf[i+ fb->len-10] = 1;  // Alternates between 0 and 1
    }
    // delay(40);

    // String encoded = base64::encode(fb->buf, fb->len);
    // Serial.write(encoded.c_str(), encoded.length());
    Serial.write(fb->buf, fb->len);
    //Serial.println();
  }

  esp_camera_fb_return(fb);
}
