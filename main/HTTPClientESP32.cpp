#include "HTTPClientESP32.h"

HTTPClientESP32::HTTPClientESP32(const String& baseURL) 
    : _baseURL(baseURL) {}

void HTTPClientESP32::sendPostRequest(const String& endpoint, const JsonDocument& jsonDoc) {
    if (_baseURL == "NONE")
        return;
    HTTPClient http;
    http.begin(_baseURL + endpoint); 
    http.addHeader("Content-Type", "application/json");

    String payload;
    serializeJson(jsonDoc, payload);

    int httpResponseCode = http.POST(payload);
    Serial.println("Sending POST to "+ _baseURL + endpoint);
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
    } else {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
    }

    http.end();
}

void HTTPClientESP32::setBaseURL(const String& baseURL)
{
    _baseURL = baseURL;
}

void HTTPClientESP32::sendGetRequest(const String& endpoint) {
    if (_baseURL == "NONE")
        return;

    HTTPClient http;
    http.begin(_baseURL + endpoint); 

    int httpResponseCode = http.GET();
    Serial.println("Sending GET to "+ _baseURL + endpoint);
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
    } else {
        Serial.print("Error on sending GET: ");
        Serial.println(httpResponseCode);
    }

    http.end();
}

void HTTPClientESP32::sendPostRequest(const String& endpoint, const String payload) {
    if (_baseURL == "NONE")
        return;

    HTTPClient http;
    http.begin(_baseURL + endpoint); 
    http.addHeader("Content-Type", "application/json");


    int httpResponseCode = http.POST(payload);
    Serial.println("Sending POST to "+ _baseURL + endpoint);
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
    } else {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
    }

    http.end();
}


void HTTPClientESP32::motor_act(int stepperid, int position, int speed, int isabs, int isaccel){
    // Prepare a JSON payload using ArduinoJson
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["motor"]["steppers"][0]["stepperid"] = stepperid;
    jsonDoc["motor"]["steppers"][0]["position"] = position;
    jsonDoc["motor"]["steppers"][0]["speed"] = speed;
    jsonDoc["motor"]["steppers"][0]["isabs"] = isabs;
    jsonDoc["motor"]["steppers"][0]["isaccel"] = isaccel;
    log_d("motor_act");
    // Send POST request
    sendPostRequest("/motor_act", jsonDoc);
}

void HTTPClientESP32::ledarr_act(int r, int g, int b){
    // Send an RGB value to turn all leds on/off at a certain colour
    /* curl -X POST http://192.168.4.2/ledarr_act 
     -H "Content-Type: application/json" 
     -d '{"led": { "LEDArrMode": 1, "led_array": [ { "b": 255, "g": 255, "id": 0, "r": 255 }] }}'
     */
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["led"]["LEDArrMode"] = 1;
    jsonDoc["led"]["led_array"][0]["id"] = 0;
    jsonDoc["led"]["led_array"][0]["r"] = r;
    jsonDoc["led"]["led_array"][0]["g"] = g;
    jsonDoc["led"]["led_array"][0]["b"] = b;
    log_i("ledarr_act");
    sendPostRequest("/ledarr_act", jsonDoc);

}