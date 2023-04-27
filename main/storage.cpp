#include "esp_camera.h"
#include "jsonlib.h"
#include "storage.h"
#include <ArduinoJson.h>

// These are defined in the main .ino file
extern void flashLED(int flashtime);
extern int myRotation;   // Rotation
extern int lampVal;      // The current Lamp value
extern bool autoLamp;    // Automatic lamp mode
extern int xclk;         // Camera module clock speed
extern int minFrameTime; // Limits framerate
/*
 * Useful utility when debugging...
 */

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  Serial.printf("Listing SPIFFS directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void dumpPrefs(fs::FS &fs)
{
  if (fs.exists(PREFERENCES_FILE))
  {
    // Dump contents for debug
    File file = fs.open(PREFERENCES_FILE, FILE_READ);
    int countSize = 0;
    while (file.available() && countSize <= PREFERENCES_MAX_SIZE)
    {
      Serial.print(char(file.read()));
      countSize++;
    }
    Serial.println("");
    file.close();
  }
  else
  {
    Serial.printf("%s not found, nothing to dump.\r\n", PREFERENCES_FILE);
  }
}

void savePrefs(fs::FS &fs)
{
  // FIXME: Merge with loadPrefs() to avoid duplication
  if (fs.exists(PREFERENCES_FILE))
  {
    Serial.printf("Updating %s\r\n", PREFERENCES_FILE);
  }
  else
  {
    Serial.printf("Creating %s\r\n", PREFERENCES_FILE);
  }
  File file = fs.open(PREFERENCES_FILE, FILE_WRITE);

  sensor_t *s = esp_camera_sensor_get();

  // save it to an arduinojson document
  DynamicJsonDocument jsonDoc(1024);
  jsonDoc["lamp"] = lampVal;
  jsonDoc["autolamp"] = autoLamp;
  jsonDoc["framesize"] = s->status.framesize;
  jsonDoc["quality"] = s->status.quality;
  jsonDoc["xclk"] = xclk;
  jsonDoc["min_frame_time"] = minFrameTime;
  jsonDoc["brightness"] = s->status.brightness;
  jsonDoc["contrast"] = s->status.contrast;
  jsonDoc["saturation"] = s->status.saturation;
  jsonDoc["special_effect"] = s->status.special_effect;
  jsonDoc["wb_mode"] = s->status.wb_mode;
  jsonDoc["awb"] = s->status.awb;
  jsonDoc["awb_gain"] = s->status.awb_gain;
  jsonDoc["aec"] = s->status.aec;
  jsonDoc["aec2"] = s->status.aec2;
  jsonDoc["ae_level"] = s->status.ae_level;
  jsonDoc["aec_value"] = s->status.aec_value;
  jsonDoc["agc"] = s->status.agc;
  jsonDoc["agc_gain"] = s->status.agc_gain;
  jsonDoc["gainceiling"] = s->status.gainceiling;
  jsonDoc["bpc"] = s->status.bpc;
  jsonDoc["wpc"] = s->status.wpc;
  jsonDoc["raw_gma"] = s->status.raw_gma;
  jsonDoc["lenc"] = s->status.lenc;
  jsonDoc["vflip"] = s->status.vflip;
  jsonDoc["hmirror"] = s->status.hmirror;
  jsonDoc["dcw"] = s->status.dcw;
  jsonDoc["colorbar"] = s->status.colorbar;
  jsonDoc["rotate"] = String(myRotation);

  // FIXME: ADD ALL THE values from the json document to variabels!
  log_d("Serialize");
  serializeJsonPretty(jsonDoc, Serial); // print the JSON object to Serial
  String jsonString;
  serializeJson(jsonDoc, jsonString);

  // Write the JSON string to the file
  file.print(jsonString);
  dumpPrefs(SPIFFS);
}

void removePrefs(fs::FS &fs)
{
  if (fs.exists(PREFERENCES_FILE))
  {
    Serial.printf("Removing %s\r\n", PREFERENCES_FILE);
    if (!fs.remove(PREFERENCES_FILE))
    {
      Serial.println("Error removing preferences");
    }
  }
  else
  {
    Serial.println("No saved preferences file to remove");
  }
}

void filesystemStart()
{
  Serial.println("Starting internal SPIFFS filesystem");
  while (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
  {
    // if we sit in this loop something is wrong;
    // if no existing spiffs partition exists one should be automagically created.
    Serial.println("SPIFFS Mount failed, this can happen on first-run initialisation");
    Serial.println("If it happens repeatedly check if a SPIFFS partition is present for your board?");
    for (int i = 0; i < 10; i++)
    {
      flashLED(100); // Show SPIFFS failure
      delay(100);
    }
    delay(1000);
    Serial.println("Retrying..");
  }
  listDir(SPIFFS, "/", 0);
}

/* Functions from Device Pref*/

// Read the device preferences from the filesystem and return a arduinojson dictionary

DynamicJsonDocument readConfig(fs::FS &fs)
{
  DynamicJsonDocument jsonDoc(1024); // create a dynamic JSON document with a capacity of 1024 bytes
  // FIXME: ADD ALL THE values from the json document to variabels!
  if (fs.exists(PREFERENCES_FILE))
  {
    // read file into a string
    String prefs;
    Serial.printf("Loading preferences from file %s\r\n", PREFERENCES_FILE);
    File file = fs.open(PREFERENCES_FILE, FILE_READ);
    if (!file)
    {
      Serial.println("Failed to open preferences file for reading, maybe corrupt, removing");
      removePrefs(SPIFFS);
      return jsonDoc;
    }
    size_t size = file.size();
    if (size > PREFERENCES_MAX_SIZE)
    {
      Serial.println("Preferences file size is too large, maybe corrupt, removing");
      removePrefs(SPIFFS);
      return jsonDoc;
    }
    while (file.available())
    {
      prefs += char(file.read());
      if (prefs.length() > size)
      {
        // corrupted SPIFFS files can return data beyond their declared size.
        Serial.println("Preferences file failed to load properly, appears to be corrupt, removing");
        removePrefs(SPIFFS);
        return jsonDoc;
      }
    }

    // parse the string into a JSON object
    DeserializationError error = deserializeJson(jsonDoc, prefs);
    if (error)
    {
      Serial.println("Failed to parse JSON file");
      return jsonDoc; // return an empty JSON object
    }

    return jsonDoc; // return the parsed JSON object
  }
  else
  {
    Serial.println("No preferences file found, returning empty JSON object");
    return jsonDoc; // return an empty JSON object
  }
}

void loadPrefs(fs::FS &fs)
{

  // read if file exists
  if (fs.exists(PREFERENCES_FILE))
  {
    // Define the size of the JSON document
    const size_t capacity = JSON_OBJECT_SIZE(10) + 140;

    // Create a DynamicJsonDocument
    DynamicJsonDocument doc(capacity);

    // Parse the JSON document
    doc = readConfig(SPIFFS);

    // Get sensor reference
    sensor_t *s = esp_camera_sensor_get();

    // Process local settings
    if (lampVal >= 0)
    {
      int lampValPref = doc["lamp"].as<int>();
      if (lampValPref >= 0)
        lampVal = lampValPref;
    }
    minFrameTime = doc["min_frame_time"].as<int>();
    autoLamp = doc["autolamp"].as<bool>();
    int xclkPref = doc["xclk"].as<int>();
    if (xclkPref >= 2)
      xclk = xclkPref;
    myRotation = doc["rotate"].as<int>();

    // Process camera settings
    s->set_framesize(s, (framesize_t)doc["framesize"].as<int>());
    s->set_quality(s, doc["quality"].as<int>());
    s->set_xclk(s, LEDC_TIMER_0, xclk);
    s->set_brightness(s, doc["brightness"].as<int>());
    s->set_contrast(s, doc["contrast"].as<int>());
    s->set_saturation(s, doc["saturation"].as<int>());
    s->set_special_effect(s, doc["special_effect"].as<int>());
    s->set_wb_mode(s, doc["wb_mode"].as<int>());
    s->set_whitebal(s, doc["awb"].as<int>());
    s->set_awb_gain(s, doc["awb_gain"].as<int>());
    s->set_exposure_ctrl(s, doc["aec"].as<int>());
    s->set_aec2(s, doc["aec2"].as<int>());
    s->set_ae_level(s, doc["ae_level"].as<int>());
    s->set_aec_value(s, doc["aec_value"].as<int>());
    s->set_gain_ctrl(s, doc["agc"].as<int>());
    s->set_agc_gain(s, doc["agc_gain"].as<int>());
    s->set_gainceiling(s, (gainceiling_t)doc["gainceiling"].as<int>());
    s->set_bpc(s, doc["bpc"].as<int>());
    s->set_wpc(s, doc["wpc"].as<int>());
    s->set_raw_gma(s, doc["raw_gma"].as<int>());
    s->set_lenc(s, doc["lenc"].as<int>());
    s->set_vflip(s, doc["vflip"].as<int>());
    s->set_hmirror(s, doc["hmirror"].as<int>());
    s->set_dcw(s, doc["dcw"].as<int>());
    s->set_colorbar(s, doc["colorbar"].as<int>());

    // close the file
    dumpPrefs(SPIFFS);
  }
  else
  {
    Serial.printf("Preference file %s not found; using system defaults.\r\n", PREFERENCES_FILE);
  }
}

bool isFirstBoot(fs::FS &fs)
{
  // get compiled date/time
  static const char compiled_date[] PROGMEM = __DATE__ " " __TIME__;

  // FIXME: What if stored_date is not set yet?
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  String stored_date = mConfig["stored_date"];

  Serial.print("First run? ");
  if (!stored_date.equals(compiled_date))
  {
    Serial.println("yes");
  }
  else
  {
    Serial.println("no");
  }
  mConfig["stored_date"] = compiled_date;
  savePrefs(SPIFFS);
  return !stored_date.equals(compiled_date);

  return !stored_date.equals(compiled_date);
}

bool getIsTimelapseAnglerfish(fs::FS &fs)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  if (mConfig.containsKey("isTimelapseAnglerfish"))
    return mConfig["isTimelapseAnglerfish"];
  else
    return false;
}

void setIsTimelapseAnglerfish(fs::FS &fs, bool isTimelapseAnglerfish)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  mConfig["isTimelapseAnglerfish"] = isTimelapseAnglerfish;
  savePrefs(SPIFFS);
}


static const char wifissidKey[] = "wifi_ssid";
String getWifiSSID(fs::FS &fs)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  if (mConfig.containsKey(wifissidKey))
    return mConfig[wifissidKey];
  else
    return "";
}

void setWifiSSID(fs::FS &fs, String value)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  mConfig[wifissidKey] = value;
  savePrefs(SPIFFS);
}

static const char wifipwKey[] = "wifi_pw";
String getWifiPW(fs::FS &fs)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  if (mConfig.containsKey(wifipwKey))
    return mConfig[wifipwKey];
  else
    return "";
}

void setWifiPW(fs::FS &fs, String value)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  mConfig[wifipwKey] = value;
  savePrefs(SPIFFS);
}

static const char frameIndexKey[] = "frame_index";
uint32_t getFrameIndex(fs::FS &fs)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  if (mConfig.containsKey(frameIndexKey))
    return mConfig[frameIndexKey];
  else
    return -1;
}

void setFrameIndex(fs::FS &fs, int value)
{
  DynamicJsonDocument mConfig = readConfig(SPIFFS);
  mConfig[frameIndexKey] = value;
  savePrefs(SPIFFS);
}

