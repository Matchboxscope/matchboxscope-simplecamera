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
extern bool isFirstBoot(fs::FS &fs);

// Anglerfish Modes 
extern bool getIsTimelapseAnglerfish(fs::FS &fs);
extern void setIsTimelapseAnglerfish(fs::FS &fs, bool value);
extern String getWifiSSID(fs::FS &fs);
extern void setWifiSSID(fs::FS &fs, String value);
extern String getWifiPW(fs::FS &fs);
extern void setWifiPW(fs::FS &fs, String value);

extern uint32_t getFrameIndex(fs::FS &fs);
extern void setFrameIndex(fs::FS &fs, int value);
extern bool getAcquireStack(fs::FS &fs);
extern void setAcquireStack(fs::FS &fs, bool value);
uint32_t getTimelapseInterval(fs::FS &fs);
void setTimelapseInterval(fs::FS &fs, uint32_t value);
void setCompiledDate(fs::FS &fs);
bool getIsTimelapseGeneral(fs::FS &fs);
void setIsTimelapseGeneral(fs::FS &fs, bool value);

extern void filesystemStart();
