#ifndef HTTPClientESP32_h
#define HTTPClientESP32_h

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

class HTTPClientESP32 {
public:
    HTTPClientESP32(const String& baseURL);
    void sendPostRequest(const String& endpoint, const JsonDocument& jsonDoc);
    void sendPostRequest(const String& endpoint, const String payload);
    void setBaseURL(const String& baseURL);
    void sendGetRequest(const String& endpoint);
    void pwm_act(int channel, int value);
    void ledarr_act(int r, int g, int b);
    void motor_act(int stepperid, int position, int speed, int isabs, int isaccel);
    
private:
    String _ssid;
    String _password;
    String _baseURL;
};

#endif