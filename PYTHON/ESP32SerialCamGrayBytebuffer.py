#%%
import serial
import time
import serial.tools.list_ports
from PIL import Image
import base64
import io
import numpy as np
import matplotlib.pyplot as plt
import tifffile as tif

import cv2

def search_sequence_numpy(arr,seq):
    """ Find sequence in an array using NumPy only.

    Parameters
    ----------    
    arr    : input 1D array
    seq    : input 1D array

    Output
    ------    
    Output : 1D Array of indices in the input array that satisfy the 
    matching of input sequence in the input array.
    In case of no match, an empty list is returned.
    """

    # Store sizes of input array and sequence
    arr = np.array(arr)
    seq = np.array(seq)
    Na, Nseq = arr.size, seq.size

    # Range of sequence
    r_seq = np.arange(Nseq)

    # Create a 2D array of sliding indices across the entire length of input array.
    # Match up with the input sequence & get the matching starting indices.
    M = (arr[np.arange(Na-Nseq+1)[:,None] + r_seq] == seq).all(1)

    # Get the range of those indices as final output
    if M.any() >0 and M.any():
        positionList =  np.where(np.convolve(M,np.ones((Nseq),dtype=int))>0)[0]
        if len(positionList) == Nseq:
              return positionList[0]
        else:
              return -1
    else:
        return -1        # No match found
    
def connect_to_usb_device(manufacturer="Espressif", timeout=1):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.manufacturer == manufacturer or port.manufacturer == "Microsoft":
            try:
                # ATTENTION: Windows does not like a higher BAUDRATE than 100000
                ser = serial.Serial(port.device, baudrate=2000000, timeout=timeout)
                print(f"Connected to device: {port.description}")
                ser.write_timeout = timeout
                time.sleep(.5)
                return ser
            except serial.SerialException:
                print(f"Failed to connect to device: {port.description}")
    print("No matching USB device found.")
    return None

# Specify the manufacturer to connect to
manufacturer = 'Espressif'

# Connect to the USB device
serialdevice = connect_to_usb_device(manufacturer)


#%%
iError = 0

t0 = time.time()
message = ""
imageString = ""

#cv2.startWindowThread()

serialdevice.write(('t10\n').encode())
serialdevice.readline()
frame_size = 320 * 240
syncPatternPosition = 38684
while True:
  try:
        #read image and decode
        #serialdevice.write(b"")
        serialdevice.write(('\n').encode())
        # don't read to early
        time.sleep(.01)

        # Read a frame from the serial port
        frame_bytes = serialdevice.read(frame_size)

        # Convert the bytes to a numpy array
        frame_flat = np.frombuffer(frame_bytes, dtype=np.uint8)
        
        # find 0,1,0,1... pattern to sync
        pattern = (0,1,0,1,0,1,0,1,0,1)
        
        indexPattern = search_sequence_numpy(frame_flat,pattern)

        if indexPattern >=0:

          frame_flat_shifted =  np.roll(frame_flat,-indexPattern)
          #print(indexPattern)

          # reshape into 2D image
          frame = np.zeros(frame_size, dtype=np.uint8)
          frame[0:frame_flat_shifted.shape[0]]=frame_flat_shifted
          pattern = (0,1,2,0,1,2,0,1,2,0)
        
          indexPattern = search_sequence_numpy(frame_flat,pattern)
          print(indexPattern)
          if 1:# indexPattern == syncPatternPosition:
            frame = frame.reshape((240, 320))
            t0 = time.time()
            cv2.imshow("image", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break    
            #frame = np.mean(frame,-1)
            #cv2.waitKey(-1)
            #plt.imshow(image), plt.show()
            #serialdevice.flushInput()
            #serialdevice.flushOutput()
            #tif.imsave("test_stack_esp32.tif", image, append=True)
          else:
              # skip frame and restart device
              serialdevice.write(('r\n').encode())
              serialdevice.readline()
              time.sleep(.4)



        
  except Exception as e:
      print("Error")
      print(e)
      serialdevice.flushInput()
      serialdevice.flushOutput()
      iError += 1
      #serialdevice.reset_input_buffer()
      # reset device here 
      if not iError % 20 and iError >1:
            try:
                # close the device - similar to hard reset
                serialdevice.setDTR(False)
                serialdevice.setRTS(True)
                time.sleep(.1)
                serialdevice.setDTR(False)
                serialdevice.setRTS(False)
                time.sleep(.5)
                #serialdevice.close()
            except Exception as e: pass
            serialdevice = connect_to_usb_device()
            nTrial = 0
      
    
print(iError)

#%%
        
        
''' ESP CODE
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

int Nx = 320;
int Ny = 240;
int Nroi = 50;
int x = 320 / 2;
int y = 240 / 2;
bool isStreaming = true;

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

  config.fb_count = 1;

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
    // delay(40);

    // String encoded = base64::encode(fb->buf, fb->len);
    // Serial.write(encoded.c_str(), encoded.length());
    Serial.write(fb->buf, fb->len);
    //Serial.println();
  }

  esp_camera_fb_return(fb);
}
'''
