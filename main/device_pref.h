// Arduino ESP32 headers
#include <Preferences.h>

class DevicePreferences {
  public:
    explicit DevicePreferences(
        Preferences &preferences, const char *group_name, const char *compiled_date
    ) :
      preferences(preferences), group_name(group_name), compiled_date(compiled_date) {}
    
    bool getIsTimelapseAnglerfish();
    void setIsTimelapseAnglerfish(bool value);
    void setWifiSSID(String value);
    void setWifiPW(String value);
    String getWifiSSID();    
    String getWifiPW();

    uint32_t getFrameIndex();
    void setFrameIndex(uint32_t value);
    uint32_t getCameraExposureTime();
    uint32_t getCameraGain();
    void setCameraExposureTime(uint32_t value);
    void setCameraGain(uint32_t value);  
    uint32_t getTimelapseInterval();
    void setTimelapseInterval(uint32_t value);  
    uint32_t getCameraFramesize();
    void setCameraFramesize(uint32_t value);  
    void setCameraEffect(uint32_t value);
    uint32_t getCameraEffect();

    void setAcquireStack(bool value);
    bool getAcquireStack();



  private:
    Preferences &preferences;
    const char *group_name;
    const String compiled_date;
};
