#include "FS.h"
#include "SPIFFS.h"

#define FORMAT_SPIFFS_IF_FAILED true
#define PREFERENCES_MAX_SIZE 500

#define PREFERENCES_FILE "/esp32cam-preferences.json"

extern void dumpPrefs(fs::FS &fs);
extern void loadPrefs(fs::FS &fs);
extern void removePrefs(fs::FS &fs);
extern void savePrefs(fs::FS &fs);
extern bool isFirstBoot(fs::FS &fs);

// Anglerfish Modes 
extern bool getIsTimelapseAnglerfish(fs::FS &fs);
extern void setIsTimelapseAnglerfish(fs::FS &fs, bool value);
extern String getWifiSSID(fs::FS &fs);
extern void setWifiSSID(fs::FS &fs, String value);
extern String getWifiPW(fs::FS &fs);
extern void setWifiPW(fs::FS &fs, String value);

extern void filesystemStart();
