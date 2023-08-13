/*
 *   Pin definitions for some common ESP-CAM modules
 *
 *   Select the module to use 
 *   Defaults to AI-THINKER CAM module
 *
 */

#define LED_ON LOW   // - Pin is inverted.
#define LED_OFF HIGH //


#if defined(CAMERA_MODEL_AI_THINKER)
//
// AI Thinker
// https://github.com/SeeedDocument/forum_doc/raw/master/reg/ESP32_CAM_V1.6.pdf
//
#define PWM_PIN 12
#define LED_PIN 33 // Status led
#define LAMP_PIN 4   // LED FloodLamp.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

    #elif defined(CAMERA_MODEL_XIAO)
//
// ESP XIAO
// https://dl.espressif.com/dl/schematics/ESP-WROVER-KIT_SCH-2.pdf
//
#define LED_PIN LED_BUILTIN // Status led
#define LAMP_PIN 21   // LED FloodLamp.
#define PWM_PIN 4
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

#define LED_GPIO_NUM 0
#else
// Well.
// that went badly...
#error "Camera model not selected"
#endif
