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
// CAMERA - GAIN
void DevicePreferences::setCameraGain(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(gainKey, value);
  preferences.end();
}

uint32_t DevicePreferences::getCameraGain() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(gainKey, 0);
  preferences.end();
  return value;
}

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

// STACK - acquire stack?
static const char isStackKey[] = "isStack";
void DevicePreferences::setAcquireStack(bool value) {
  preferences.begin(group_name, false);
  preferences.putBool(isStackKey, value);
  Serial.print("Setting stack enable to: ");
  Serial.println(value);  
  preferences.end();
}

bool DevicePreferences::getAcquireStack(void) {
  preferences.begin(group_name, true);
  bool value = preferences.getBool(isStackKey, false);
  Serial.print("Is Acquire Stack is: ");
  Serial.println(value);
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

// CAMERA - camera effect
void DevicePreferences::setCameraEffect(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(cameraEffectKey, value);
  preferences.end();
}

uint32_t DevicePreferences::getCameraEffect() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(cameraEffectKey, 2); // 2=> mono
  preferences.end();
  return value;
}
