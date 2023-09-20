#include "FS.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>

#define FORMAT_SPIFFS_IF_FAILED true
#define PREFERENCES_MAX_SIZE 500

#define PREFERENCES_FILE "/esp32cam-preferences.json"

extern void printPrefs(fs::FS &fs);
extern void loadSpiffsToPrefs(fs::FS &fs);
extern DynamicJsonDocument readPrefs(fs::FS &fs);
extern void removePrefs(fs::FS &fs);
extern void writePrefsToSSpiffs(fs::FS &fs);
extern void writeJsonToSSpiffs(fs::FS &fs, DynamicJsonDocument doc);
extern bool isFirstBoot();

// Anglerfish Modes 
extern bool getIsTimelapseAnglerfish(); // in preferences not spiffs!
extern void setIsTimelapseAnglerfish(bool value); // in preferences not spiffs!
extern String getWifiSSID(fs::FS &fs);
extern void setWifiSSID(fs::FS &fs, String value);
extern String getWifiPW(fs::FS &fs);
extern void setWifiPW(fs::FS &fs, String value);
extern void setSerialFrameEnabled(int value);
extern uint32_t getSerialFrameEnabled();

extern uint32_t getFrameIndex(fs::FS &fs);
extern void setFrameIndex(fs::FS &fs, int value);
extern bool getAcquireStack(fs::FS &fs);
extern void setAcquireStack(fs::FS &fs, bool value);
uint32_t getTimelapseInterval(fs::FS &fs);
void setTimelapseInterval(fs::FS &fs, uint32_t value);
uint32_t getAutofocusInterval(fs::FS &fs);
void setAutofocusInterval(fs::FS &fs, uint32_t value);
void setCompiledDate(fs::FS &fs);
bool getIsTimelapseGeneral(fs::FS &fs);
void setIsTimelapseGeneral(fs::FS &fs, bool value);
void setCompiledDate(fs::FS &fs);
uint32_t getPWMVal(fs::FS &fs);
void setPWMVal(fs::FS &fs, uint32_t value);


extern void filesystemStart();
