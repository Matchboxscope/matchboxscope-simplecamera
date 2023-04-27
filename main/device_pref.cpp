// Arduino ESP32 headers (in alphabetical order!)
#include <Arduino.h>
#include "device_pref.h"
// First-run detection







// Timelapse frame indexing

// Camera settings (MAKE SURE NAME IS SHORT!
static const char exposuretimeKey[] = "cameraExpT";
static const char gainKey[] = "cameraGain";
static const char framesizeKey[] = "cameraFS";
static const char timelapseIntervalKey[] = "cameraTLI";
static const char cameraEffectKey[] = "cameraFX";

// CAMERA - exposureTime
void DevicePreferences::setCameraExposureTime(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(exposuretimeKey, value);
  preferences.end();
}

uint32_t DevicePreferences::getCameraExposureTime() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(exposuretimeKey, 0);
  preferences.end();
  return value;
}

// CAMERA - stream framesize
void DevicePreferences::setCameraFramesize(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(framesizeKey, value);
  preferences.end();
}

uint32_t DevicePreferences::getCameraFramesize() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(framesizeKey, 0);
  preferences.end();
  return value;
}



// CAMERA - stream framesize
void DevicePreferences::setTimelapseInterval(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(timelapseIntervalKey, value);
  Serial.print("Setting Timelapse Interval to: ");
  Serial.println(value);  
  preferences.end();
}

uint32_t DevicePreferences::getTimelapseInterval() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(timelapseIntervalKey, -1);
  //Serial.print("Timelapse Interval is: ");
  //Serial.println(value);
  preferences.end();
  return value;
}
