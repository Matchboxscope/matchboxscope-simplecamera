#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  4 22:35:28 2023

@author: bene
"""
#%%

import numpy as np
import serial
import matplotlib.pyplot as plt
H=240//2
W=320//2

# Open the serial connection
ser = serial.Serial('/dev/cu.usbmodem11101', 100000, timeout=.3)  # Adjust the port and baud rate accordingly


# %%
while 1:
    # Receive the byte array
    byte_array_length = H * W * 2  # Size of uint16_t is 2 bytes
    try:
        received_data = ser.read(int(byte_array_length*2))
        frames = received_data.split(b'###NEWFRAME###')[1]
        # Convert the byte array to a 2D NumPy array
        np_array = np.frombuffer(frames, dtype=np.uint16)[0:int(H*W)]
        np_array = np_array.reshape((int(H), int(W)))
        
        plt.imshow(np_array), plt.show()
    except:
        pass
    
#%%
# Create an empty 2D NumPy array to store the frames
frames = []

# Read frames continuously
while 1:
    # Read until a line break is encountered
    line = ser.readline().decode().strip()
    
    # Check if the line break is encountered
    if line == '':
        # Convert the received frames to a 2D NumPy array
        np_array = np.array(frames, dtype=np.uint16)
        np_array = np_array.reshape((-1, W))  # Reshape to the desired dimensions
        
        # Process the received frames
        # ...

        # Clear the frames list
        frames = []
    else:
        # Split the line by spaces to get individual values
        values = line.split()
        
        # Convert the values to integers and add them to the frames list
        frame = [int(value) for value in values]
        frames.append(frame)
        
#%%
while(1):
    # Read the data from serial
    try:
        ser.write((' ').encode())
      
        data = ser.read_until(b'---').decode().strip()
        
        # Remove the leading '+++' and trailing '---' markers
        data = data.split('+++')[-1].split('---')[0]#data.strip('+++').strip('---')
        
        # Split the data into rows
        rows = data.split("\r\n")[1:-1] 
        
        # Initialize an empty 4x4 NumPy array
        matrix = np.empty((int(H), int(W)), dtype=np.int16)
        
        # Fill the matrix with the parsed values
        for i, row in enumerate(rows):
            elements = row.split(',')[0:-1]
            matrix[i] = [int(element) for element in elements]
        
        # Print the resulting matrix
        plt.imshow(matrix), plt.show()
        plt.imsave('test.png', matrix)
    except Exception as e:
        ser.flushInput()
        ser.flushOutput()
        print(e)
        #break
    matrix_ = matrix


#%%
byte_array_length = H * W * 2 + 30 # extra delimeter
while(1):
    # Read the data from serial
    if 1:
        ser.write((' ').encode())
      
        received_data = ser.read(byte_array_length)
        data = received_data.split(b"+++")[-1].split(b'---')[0]
        
        # Convert the byte array to a 2D NumPy array
        np_array = np.frombuffer(data, dtype=np.uint16)
        np_array = np_array.reshape((int(H), int(W)))

        # Print the resulting matrix
        plt.imshow(np_array), plt.show()
#        plt.imsave('test.png', matrix)
    
    
  #%%  
#include "esp_camera.h"

#define FRAME_SIZE FRAMESIZE_QVGA
#define WIDTH 320
#define HEIGHT 240
#define BLOCK_SIZE 2
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.2
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 1

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

uint16_t proc_frame[H][W] = {0};
uint16_t current_frame[H][W] = {0};

bool setup_camera(framesize_t);
bool capture_still();
void convolve_gaussian();
void find_max_pix();
void print_frame(uint16_t frame[H][W]);
int max_x = 0;
int max_y = 0;

/**

*/
void setup()
{
    Serial.begin(1000000);
    cameraInit();

    // flush serial
      while (Serial.available() > 0) {
    char c = Serial.read();
  }
}

/**

*/
void loop()
{

    if (Serial.available() > 0)
    {
        delay(50);
        Serial.println(Serial.read()); // Read the incoming command until a newline character is encountered

        if (!capture_still())
        {
            Serial.println("Failed capture");
            delay(3000);

            return;
        }

        // convolve_gaussian();
    }

    // find_max_pix();
    /*
    Serial.print("My X: ");
    Serial.print(max_x);
    Serial.print("My Y: ");
    Serial.print(max_y);

    Serial.println("=================");
    */
}

/**

*/
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
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA; // for streaming}

    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.println("Camera init failed with error: " + err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_hmirror(s, 1);
    s->set_vflip(s, 1);
    s->set_special_effect(s, 2); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
}

/**
   Capture image and do down-sampling
*/
bool capture_still()
{
    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (!frame_buffer)
        return false;

    // set all 0s in current frame
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] = 0;

    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++)
    {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if 1
    // Convert the 2D array to a byte array
    uint8_t* byte_array = (uint8_t*)current_frame;
    size_t byte_array_length = H * W * sizeof(uint16_t);

    // Send the byte array over serial
    Serial.print("+++");
    Serial.write(byte_array, byte_array_length);
    Serial.print("---");

/*
    Serial.println("+++");
    print_frame(current_frame);
    Serial.println("---");
    */
#endif

    esp_camera_fb_return(frame_buffer);
    return true;
}

//  Different kernels:
// http://blog.dzl.dk/2019/06/08/compact-gaussian-interpolation-for-small-displays/
// Sigma=1
#define P0 (0.077847)
#define P1 (0.123317 + 0.077847)
#define P2 (0.195346 + 0.123317 + 0.123317 + 0.077847)
/*
  //Sigma=0.5
  #define P0 (0.024879)
  #define P1 (0.107973+0.024879)
  #define P2 (0.468592+0.107973+0.107973+0.024879)
*/
/*
  //Sigma=2
  #define P0 (0.102059)
  #define P1 (0.115349+0.102059)
  #define P2 (0.130371+0.115349+0.115349+0.102059)
*/

const float kernel[4][4] =
    {
        {P0, P1, P1, P2},
        {P1, P0, P2, P1},
        {P1, P2, P0, P1},
        {P2, P1, P1, P0}};

/*
   Convolve the image with a gaussian and find the maximum position of the laserbeam
   Inspired by: http://www.songho.ca/dsp/convolution/convolution.html
*/
void convolve_gaussian()
{
    // find center position of kernel (half of kernel size)
    int kCols = 4; // size kernel x
    int kRows = 4; // size kernel y
    int kCenterX = kCols / 2;
    int kCenterY = kRows / 2;

    for (int y = 0; y < H; y++) // rows
    {
        for (int x = 0; x < W; x++) // columns
        {
            proc_frame[y][x] = 0;           // Initiliaze with zero
            for (int m = 0; m < kRows; ++m) // kernel rows
            {
                int mm = kRows - 1 - m;         // row index of flipped kernel
                for (int n = 0; n < kCols; ++n) // kernel columns
                {
                    int nn = kCols - 1 - n; // column index of flipped kernel

                    // index of input signal, used for checking boundary
                    int yy = y + (kCenterY - mm);
                    int xx = x + (kCenterX - nn);

                    // ignore input samples which are out of bound
                    if (yy >= 0 && yy < H && xx >= 0 && xx < W)
                        proc_frame[y][x] += current_frame[yy][xx] * kernel[mm][nn];
                }
            }
        }
    }

#if 0
    // Convert the 2D array to a byte array
    uint8_t* byte_array = (uint8_t*)proc_frame;
    size_t byte_array_length = H * W * sizeof(uint16_t);

    // Send the byte array over serial
    Serial.print("+++");
    Serial.write(byte_array, byte_array_length);
    Serial.print("---");
    // flush serial
      while (Serial.available() > 0) {
    char c = Serial.read();
  }
    /*
    
    Serial.println("+++");
    print_frame(proc_frame);
    Serial.println("---");
    */
#endif
}

/**
   Copy current frame to previous
*/
void find_max_pix()
{
    int mymax = 0;

    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            if (proc_frame[y][x] > mymax)
            {
                mymax = proc_frame[y][x];
                max_x = x;
                max_y = y;
            }
        }
    }
}

/**
   For serial debugging
   @param frame
*/
void print_frame(uint16_t frame[H][W])
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            int myval = 0;
            if (frame[y][x] > 0)
                myval = frame[y][x];
            Serial.print(myval);
            Serial.print(',');
        }

        Serial.println();
    }
}