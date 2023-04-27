// Arduino ESP32 headers
#include <Preferences.h>

class DevicePreferences {
  public:
    explicit DevicePreferences(
        Preferences &preferences, const char *group_name, const char *compiled_date
    ) :
      preferences(preferences), group_name(group_name), compiled_date(compiled_date) {}
    
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




  private:
    Preferences &preferences;
    const char *group_name;
    const String compiled_date;
};
