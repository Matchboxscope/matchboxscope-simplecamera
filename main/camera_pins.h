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
#define NEOPIXEL_PIN 4
#define NUMPIXELS 16
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
#define PWM_PIN  -1 //D8
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

// Motor Pins


// For wired version (Anglerfish)
/*
#define ACCELSTEPPER 1
#define STEPPER_MOTOR_DIR D2
#define STEPPER_MOTOR_STEP D1
#define STEPPER_MOTOR_ENABLE D0
#define STEPPER_MOTOR_M1 -1
#define STEPPER_MOTOR_M2 -1
#define STEPPER_MOTOR_M3 -1
#define STEPPER_MOTOR_NOTRESET -1
#define STEPPER_MOTOR_NOTSLEEP -1
#define NUMPIXELS 16
#define NEOPIXEL_PIN D6
//#define NEOPIXEL_PIN -1

//For piggy packed version
#define STEPPER_MOTOR
#define STEPPER_MOTOR_STEPS 200
#define STEPPER_MOTOR_SPEED 20000
*/

#define STEPPER 1
#define motorPin1 D7
#define motorPin2 D8
#define motorPin3 D9
#define motorPin4 D10
#define NUMPIXELS 16
#define NEOPIXEL_PIN D6
#define STEPPER_MOTOR
#define STEPPER_MOTOR_STEPS 200
#define STEPPER_MOTOR_SPEED 20000
//#define NEOPIXEL_PIN -1


/*
#define NUMPIXELS 16
#define NEOPIXEL_PIN D9
#define STEPPER_MOTOR_DIR D7
#define STEPPER_MOTOR_STEP D0
#define STEPPER_MOTOR_ENABLE D6
#define STEPPER_MOTOR_M1 -1
#define STEPPER_MOTOR_M2 -1
#define STEPPER_MOTOR_M3 -1
#define STEPPER_MOTOR_NOTRESET D2
#define STEPPER_MOTOR_NOTSLEEP D1
#define STEPPER_MOTOR
#define STEPPER_MOTOR_STEPS 200
#define STEPPER_MOTOR_SPEED 20000
*/
// For Board version
/*
#define STEPPER_MOTOR_DIR D8
#define STEPPER_MOTOR_STEP D7
#define STEPPER_MOTOR_ENABLE D9
#define STEPPER_MOTOR_M1 -1
#define STEPPER_MOTOR_M2 -1
#define STEPPER_MOTOR_M3 -1
#define STEPPER_MOTOR_NOTRESET -1
#define STEPPER_MOTOR_NOTSLEEP -1
#define NUMPIXELS 16
#define NEOPIXEL_PIN -0
*/

/*
// for solder-less version
en d6
m1 D5
m2 d4
m3 d3
notreset d2
notsleep d1
stp d0
dir d7
*/



#define LED_GPIO_NUM 0
#else
// Well.
// that went badly...
#error "Camera model not selected"
#endif
