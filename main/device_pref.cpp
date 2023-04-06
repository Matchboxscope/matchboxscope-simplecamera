// Arduino ESP32 headers (in alphabetical order!)
#include <Arduino.h>
#include "device_pref.h"
// First-run detection

static const char dateKey[] = "date";

bool DevicePreferences::isFirstRun() {
  preferences.begin(group_name, false);
  String stored_date = preferences.getString(dateKey, "");  // FIXME

  Serial.println("Stored date:");
  Serial.println(stored_date);
  Serial.println("Compiled date:");
  Serial.println(compiled_date);

  Serial.print("First run? ");
  if (!stored_date.equals(compiled_date)) {
    Serial.println("yes");
  } else {
    Serial.println("no");
  }

  preferences.putString(dateKey, compiled_date); // FIXME?
  preferences.end();
  return !stored_date.equals(compiled_date);
}

// Timelapse mode

static const char timelapseKey[] = "is_timelapse";
bool DevicePreferences::isTimelapse() {
  preferences.begin(group_name, true);
  bool value = preferences.getBool(timelapseKey, false);
  preferences.end();
  return value;
}

void DevicePreferences::setIsTimelapse(bool value) {
  preferences.begin(group_name, false);
  preferences.putBool(timelapseKey, value);
  preferences.end();
}

// Wifi settings
static const char wifissidKey[] = "wifi_ssid";
void DevicePreferences::setWifiSSID(String value) {
  preferences.begin(group_name, true);
  preferences.putString(value.c_str(), "Test");
  preferences.end();
}

String DevicePreferences::getWifiSSID() {
  preferences.begin(group_name, true);
  String value = preferences.getString(wifissidKey, "Blynk");
  preferences.end();
  return value;
}

static const char wifipwKey[] = "wifi_pw";
void DevicePreferences::setWifiPW(String value) {
  preferences.begin(group_name, true);
  preferences.putString(value.c_str(), wifissidKey);
  preferences.end();
}

String DevicePreferences::getWifiPW() {
  preferences.begin(group_name, true);
  String value = preferences.getString(wifipwKey, "Blynk");
  preferences.end();
  return value;
}

// Timelapse frame indexing

static const char frameIndexKey[] = "frame_index";

uint32_t DevicePreferences::getFrameIndex() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(frameIndexKey, 0);
  preferences.end();
  return value;
}

void DevicePreferences::setFrameIndex(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(frameIndexKey, value);
  preferences.end();
}

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

// CAMERA - stream framesize
void DevicePreferences::setTimelapseInterval(uint32_t value) {
  preferences.begin(group_name, false);
  preferences.putUInt(timelapseIntervalKey, value);
  preferences.end();
}

uint32_t DevicePreferences::getTimelapseInterval() {
  preferences.begin(group_name, true);
  uint32_t value = preferences.getUInt(timelapseIntervalKey, -1);
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
