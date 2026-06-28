#include <Arduino.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h> 
#include <ESP32Ping.h>   
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_task_wdt.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_NeoPixel.h>
#include <vector>
#include <esp_sntp.h>
#include <TelnetStream.h>

#include "secrets.h" // Local credentials (ignored by Git)

#ifndef ESP_FIRMWARE_PASS
#error "ESP_FIRMWARE_PASS is not defined. Did you export the environment variable?"
#endif

// a logging macro to send output to both USB and Wi-Fi simultaneously
#define LOG_PRINT(x)    { Serial.print(x); TelnetStream.print(x); }
#define LOG_PRINTLN(x)  { Serial.println(x); TelnetStream.println(x); }
#define LOG_PRINTF(...) { Serial.printf(__VA_ARGS__); TelnetStream.printf(__VA_ARGS__); }

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

// --- Hardware & Pin Definitions ---
#define STATUS_LED 15
#define WDT_TIMEOUT_S 120

const int PIN_RIGHT = 21; 
const int PIN_LEFT  = 22; 
const int BME_CS    = 17;   
const int BME_MOSI  = 18;
const int BME_SCK   = 19;  
const int BME_MISO  = 20; 

const int PIN_NEOPIXELS = 1; 
const int PIN_BUZZER    = 0; 
const int NUM_PIXELS    = 22;
const int LOGICAL_STEPS = 9;

// --- Network & Configuration ---
const char* ssid       = WIFI_SSID;
const char* wifi_password   = WIFI_PASSWORD;

const char* firmware_upload_password = ESP_FIRMWARE_PASS;  // keep in sync with platformio.ini
const char* statsd_ip  = "10.1.1.1";
const int statsd_port  = 8125;


// --- Global Objects & Variables ---
WiFiUDP udp;
WebServer server(9980);
Preferences prefs;
Adafruit_BME680 bme(BME_CS);
Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEOPIXELS, NEO_GRB + NEO_KHZ800);

SemaphoreHandle_t configMutex;
SemaphoreHandle_t buzzerMutex;
QueueHandle_t motorStateQueue;

unsigned long cycle = 0;

volatile float currentTemp = -100;
volatile float spikeFactor = 1.0; // 1.0 broad,  20.0 spiky
volatile bool prepareForFlash = false; // Flash-mode safety flag
volatile bool isBroken = false;
bool wasBroken = false; 
volatile bool isAnyDown = false;

// LAN Core Services State (Dynamic)
std::vector<IPAddress> coreServices;
std::vector<bool> serviceStatus;
volatile bool initialScanComplete = false;

int GAMMA_NOTES_COUNT = 8;
int cMajorGamma[] = {523, 587, 659, 698, 784, 880, 988, 1047};
             //      0C   1D   2E   3F   4G   5A   6B   7C 
TimerHandle_t buzzerTimer;

enum MotorState {
    STATE_DUMMY
};

enum MotorRotate {
    ROTATE_LEFT,
    ROTATE_RIGHT,
    ROTATE_IDLE
};

struct ColorPoint { float temp; uint8_t r, g, b; };

ColorPoint palette[] = {
    {15.0, 0, 255, 255},     // Cyan
    {20.0, 0, 0, 255},       // Deep Blue
    {25.0, 255, 0, 176},     // Pink
    {30.0, 255, 44, 0},      // Orange
    {35.0, 255, 255, 0},      // Yellow
    {40.0, 255, 255, 255  }   // White
};

// --- DYNAMIC LUT CONFIGURATION ---
struct RGB { uint8_t r, g, b; };
RGB* tempLUT = nullptr; // Pointer for dynamic heap allocation

const float LUT_RESOLUTION = 0.1f; // Tweak this anytime
float LUT_MIN_TEMP = 0.0f;
int LUT_SIZE = 0;
float LUT_INV_RES = 0.0f; 

// The heavy interpolation function (Used ONLY during setup)
void calculateTemperatureRGB(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    int numPoints = sizeof(palette) / sizeof(palette[0]);
    if (t <= palette[0].temp) { r = palette[0].r; g = palette[0].g; b = palette[0].b; return; }
    if (t >= palette[numPoints - 1].temp) { r = palette[numPoints - 1].r; g = palette[numPoints - 1].g; b = palette[numPoints - 1].b; return; }

    for (int i = 0; i < numPoints - 1; i++) {
        if (t >= palette[i].temp && t <= palette[i+1].temp) {
            float range = palette[i+1].temp - palette[i].temp;
            float pct = (t - palette[i].temp) / range;
            r = palette[i].r + pct * (palette[i+1].r - palette[i].r);
            g = palette[i].g + pct * (palette[i+1].g - palette[i].g);
            b = palette[i].b + pct * (palette[i+1].b - palette[i].b);
            return;
        }
    }
    r = 0; g = 0; b = 0;
}

// Call this EXACTLY ONCE inside your setup() function
void prebakeLUT() {
    int numPoints = sizeof(palette) / sizeof(palette[0]);
    
    // Automatically extract boundaries from your palette array
    LUT_MIN_TEMP = palette[0].temp;
    float LUT_MAX_TEMP = palette[numPoints - 1].temp;
    
    // Calculate required array size and the inverse resolution for fast math
    LUT_SIZE = (int)((LUT_MAX_TEMP - LUT_MIN_TEMP) / LUT_RESOLUTION) + 1;
    LUT_INV_RES = 1.0f / LUT_RESOLUTION; 
    
    // Dynamically allocate contiguous RAM for the LUT
    tempLUT = new RGB[LUT_SIZE];
    
    // Bake the colors
    for (int i = 0; i < LUT_SIZE; i++) {
        float t = LUT_MIN_TEMP + (i * LUT_RESOLUTION);
        calculateTemperatureRGB(t, tempLUT[i].r, tempLUT[i].g, tempLUT[i].b);
    }
}

// The O(1) inline lookup for the main render loop
inline void getTemperatureRGB(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    // Fast multiplication using the precomputed inverse resolution
    int idx = (int)((t - LUT_MIN_TEMP) * LUT_INV_RES);
    
    // Clamp bounds
    if (idx < 0) idx = 0;
    if (idx >= LUT_SIZE) idx = LUT_SIZE - 1;
    
    r = tempLUT[idx].r;
    g = tempLUT[idx].g;
    b = tempLUT[idx].b;
}

// --- Helper Functions ---
void ledOn() { digitalWrite(STATUS_LED, LOW); }
void ledOff() { digitalWrite(STATUS_LED, HIGH); }
void buzzerStopCallback(TimerHandle_t xTimer) {
    ledcWriteTone(PIN_BUZZER, 0); 
}

void beepAsync(int freq, int durationMs) {
    if (prepareForFlash) return; 
    
    ledcWriteTone(PIN_BUZZER, freq); 
    xTimerChangePeriod(buzzerTimer, pdMS_TO_TICKS(durationMs), 0); 
}

void getSection(int cut, int &startIdx, int &endIdx) {
    if (cut < 0) cut = 0;
    if (cut >= LOGICAL_STEPS) cut = LOGICAL_STEPS - 1;
    
    startIdx = (cut * NUM_PIXELS) / LOGICAL_STEPS;
    endIdx = ((cut + 1) * NUM_PIXELS) / LOGICAL_STEPS - 1;
}

void showProgress(int cut) {
    int startIdx, endIdx;
    getSection(cut, startIdx, endIdx);

    beepAsync(cMajorGamma[cut % GAMMA_NOTES_COUNT], 100);

    for (int i = 0; i < NUM_PIXELS; i++) {
        strip.setPixelColor(NUM_PIXELS - i - 1, i <= endIdx ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0));
    }            
    strip.show();
}

void waitProgress(int cut) {
    Serial.print(".");
    int startIdx, endIdx;
    getSection(cut, startIdx, endIdx);

    for(int i = startIdx; i <= endIdx; i++) {
        strip.setPixelColor(NUM_PIXELS - i - 1, strip.Color(255, 255, 0));
    }
    ledcWriteTone(PIN_BUZZER, cMajorGamma[cut % GAMMA_NOTES_COUNT]);
    strip.show();
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    for(int i = startIdx; i <= endIdx; i++) {
        strip.setPixelColor(NUM_PIXELS - i - 1, strip.Color(0, 0, 0));
    }
    strip.show();

    ledcWriteTone(PIN_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(2900)); 
}

void playStatusChord(bool isUp) {
    if (prepareForFlash) return; 
    
    int indicesUp[]   = {0, 2, 4};       
    int indicesDown[] = {6, 3, 1}; 
    
    for(int i = 0; i < 3; i++) {
        if (prepareForFlash) break; 
        int targetFreq = cMajorGamma[isUp ? indicesUp[i] : indicesDown[i]];
        ledcWriteTone(PIN_BUZZER, targetFreq);
        vTaskDelay(pdMS_TO_TICKS(180)); 
    }
    ledcWriteTone(PIN_BUZZER, 0);
}

void performHourlyAction(int hour) {
    ledcWriteTone(PIN_BUZZER, 2048);
    delay(160); 
    ledcWriteTone(PIN_BUZZER, 0);
    delay(80); 
    ledcWriteTone(PIN_BUZZER, 2048);
    delay(80); 
    ledcWriteTone(PIN_BUZZER, 0);
    // Calculates a linear scale from 1.0 at 12:00 to 20.0 at 00:00
    float spikeFactor = 1.0f + (19.0f / 12.0f) * abs(hour - 12);
    LOG_PRINTLN("hourly action fired!");
}


// --- Web Server Setup ---
void setupWebServer() {
    server.on("/api/ips", HTTP_POST, []() {
        if (prepareForFlash) {
            server.send(503, "text/plain", "System is locked for flashing.");
            return;
        }
        
        if (!server.hasArg("plain")) {
            server.send(400, "text/plain", "Body not received");
            return;
        }

        JsonDocument doc; 
        DeserializationError error = deserializeJson(doc, server.arg("plain"));
        
        if (error || !doc["ips"].is<JsonArray>()) {
            server.send(400, "text/plain", "Invalid JSON format.");
            return;
        }

        JsonArray newIps = doc["ips"];
        
        xSemaphoreTake(configMutex, portMAX_DELAY);
        coreServices.clear();
        serviceStatus.clear();
        
        int validIps = 0;
        for(size_t i = 0; i < newIps.size(); i++) {
            String ipStr = newIps[i].as<String>();
            IPAddress ip;
            if (ip.fromString(ipStr)) {
                coreServices.push_back(ip);
                serviceStatus.push_back(false);
                prefs.putString(("ip" + String(validIps)).c_str(), ipStr);
                validIps++;
            }
        }
        prefs.putInt("num_ips", validIps);
        initialScanComplete = false;
        xSemaphoreGive(configMutex);

        server.send(200, "application/json", "{\"status\":\"success\", \"loaded\":" + String(validIps) + "}");
        playStatusChord(true);
    });
    
    server.on("/api/brick", HTTP_POST, []() {
        prepareForFlash = true;
        server.send(200, "application/json", "{\"status\":\"flash_mode_active\", \"message\":\"Hardware suspended. Ready for firmware upload.\"}");
        playStatusChord(true);
    });

    server.on("/api/gamma", HTTP_POST, []() {
        for (int i=0; i < LOGICAL_STEPS; i++) {
            showProgress(i);
            vTaskDelay(pdMS_TO_TICKS(120));
        }
        server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Hope you had some fun!.\"}");
    });

    server.on("/api/hourly", HTTP_POST, []() {
        performHourlyAction(12);
        server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Hope you had some fun!.\"}");
    });

    server.on("/api/play", HTTP_POST, []() {
        if (!server.hasArg("freq") || !server.hasArg("duration")) {
            server.send(400, "application/json", "{\"status\":\"error\", \"message\":\"Missing 'freq' or 'duration'\"}");
            return;
        }

        int freq = server.arg("freq").toInt();
        int duration = server.arg("duration").toInt();

        if (freq <= 0 || duration <= 0) {
            server.send(400, "application/json", "{\"status\":\"error\", \"message\":\"Values must be greater than 0\"}");
            return;
        }

        if (duration > 1000) {
            duration = 1000;
        }

        beepAsync(freq, duration);
        server.send(200, "application/json", "{\"status\":\"success\", \"message\":\"Hope you had some fun!.\"}");
    });

    server.begin();
}

// --- FreeRTOS Tasks ---
void serverTask(void *pvParameters) {
    for (;;) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void pingTask(void *pvParameters) {
    size_t currentIndex = 0;

    for (;;) {
        if (prepareForFlash) vTaskSuspend(NULL); 

        if (isBroken) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (WiFi.status() == WL_CONNECTED) {
            IPAddress targetIp;
            bool skip = false;
            
            xSemaphoreTake(configMutex, portMAX_DELAY);
            if (coreServices.empty()) {
                skip = true;
            } else {
                if (currentIndex >= coreServices.size()) currentIndex = 0;
                targetIp = coreServices[currentIndex];
            }
            xSemaphoreGive(configMutex);

            if (skip) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            bool isUp = Ping.ping(targetIp, 1); 
            bool triggerChord = false;
            bool chordType = false;
            
            xSemaphoreTake(configMutex, portMAX_DELAY);
            if (!coreServices.empty() && currentIndex < coreServices.size() && coreServices[currentIndex] == targetIp) {
                if (isUp != serviceStatus[currentIndex]) {
                    serviceStatus[currentIndex] = isUp;
                    if (initialScanComplete) {
                        triggerChord = true;
                        chordType = isUp;
                    }
                }
                currentIndex++;
                if (currentIndex >= coreServices.size()) {
                    currentIndex = 0;
                    if (!initialScanComplete) initialScanComplete = true; 
                }
            } else {
                currentIndex = 0; 
            }
            xSemaphoreGive(configMutex);

            if (triggerChord) {
                playStatusChord(chordType);
                isAnyDown = !chordType;
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        } else {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

void ledDrawServices(int serviceCount, uint32_t* failColors) {
    strip.clear();
    if (serviceCount > 0) {
        int gapSize = 2; 
        int totalGaps = serviceCount - 1;
        int sectionSize = (NUM_PIXELS - (totalGaps * gapSize)) / serviceCount;
        
        if (sectionSize < 1) { 
            sectionSize = 1; 
            gapSize = 1; 
        }

        int currentPixel = 0;
        for (int i = 0; i < serviceCount; i++) {
            for (int p = 0; p < sectionSize && currentPixel < NUM_PIXELS; p++) {
                strip.setPixelColor(NUM_PIXELS - currentPixel - 1, failColors[i]);
                currentPixel++;
            }
            currentPixel += gapSize;
        }
    }
    strip.show();
}

float smoothTemp = -100;

void ledDrawTemperature() {
    if (currentTemp < 0) {
        return;
    }

    if (smoothTemp < 0.0) {
        smoothTemp = currentTemp; 
    } 

    // Adjust the 0.01 multiplier if you want the scrolling to lag/catch up faster.
    smoothTemp += (currentTemp - smoothTemp) * 0.001; 

    float viewportWidthCelsius = 0.25; 

    float cameraLeftEdgeTemp = smoothTemp - (viewportWidthCelsius / 2.0);
    float degreesPerPixel = viewportWidthCelsius / (float)NUM_PIXELS;

    // Wave frequencies in Temperature Space (Radians per Degree Celsius)
    float onFreq = 47.;     


    for (int i = 0; i < NUM_PIXELS; i++) {
        
        // x = the absolute coordinate on the infinite Celsius canvas
        float x = cameraLeftEdgeTemp + ((float)i * degreesPerPixel);

        // 1. Fetch the permanent color painted at x
        uint8_t baseR, baseG, baseB;
        getTemperatureRGB(x, baseR, baseG, baseB);

        // 2. Evaluate the permanent waves molded into x
        // --- ON STATE LOGIC ---
        float sinVal = sin(x * onFreq);
        float cosVal = cos(x * onFreq);
        float valOn = (sinVal/(1. + spikeFactor * cosVal * cosVal)) - 0.82; 
        if (valOn > 1.0) valOn = 1.0;
        if (valOn < -1.0) valOn = -1.0;  
        float finalBrightness = (valOn + 1.0) / 2.0; 
        finalBrightness *= 0.66;

        float finalR = (baseR * finalBrightness);
        float finalG = (baseG * finalBrightness);
        float finalB = (baseB * finalBrightness);

        if (finalR > 255) finalR = 255;
        if (finalG > 255) finalG = 255;
        if (finalB > 255) finalB = 255;

        strip.setPixelColor(i, strip.Color((uint8_t)finalR, (uint8_t)finalG, (uint8_t)finalB));
    }

    strip.show();

}

void ledTask(void *pvParameters) {

    float phase = 0.0;
    uint32_t failColors[NUM_PIXELS];

    for (;;) {
        if (prepareForFlash) vTaskSuspend(NULL); 

        if (isBroken) {
            strip.fill(strip.Color(255, 0, 0)); 
            strip.show();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        bool allUp = true;
        bool initComplete = false;
        int serviceCount = 0;

        xSemaphoreTake(configMutex, portMAX_DELAY);
        initComplete = initialScanComplete;
        serviceCount = serviceStatus.size();
        for(int i = 0; i < serviceCount && i < NUM_PIXELS; i++) {
            failColors[i] = serviceStatus[i] ? strip.Color(0, 255, 0) : strip.Color(255, 0, 0);
            if(!serviceStatus[i]) allUp = false;
        }
        if (serviceCount == 0) allUp = true;
        xSemaphoreGive(configMutex);

        if (!initComplete && serviceCount > 0) allUp = false; 

        if (allUp) {
            ledDrawTemperature();
            vTaskDelay(pdMS_TO_TICKS(33)); 
        } else {
            ledDrawServices(serviceCount, failColors);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
    }
}

bool isMotorPanic() {
    return isBroken || isAnyDown;
}

void motorRegularOrPanicWait(int regularWaitMs) {
    int waitChunkMs = 500;
    //  check for panic during the wait
    int chunkCount = regularWaitMs / waitChunkMs;
    vTaskDelay(pdMS_TO_TICKS(waitChunkMs));
    while (chunkCount --> 0 && !isMotorPanic()) {
        vTaskDelay(pdMS_TO_TICKS(waitChunkMs));
    }
    
}

void motorRotate(MotorRotate mode) {
    if (mode == ROTATE_LEFT) {
        digitalWrite(PIN_RIGHT, LOW); digitalWrite(PIN_LEFT, HIGH); 
    } else if (mode == ROTATE_RIGHT) {
        digitalWrite(PIN_RIGHT, HIGH); digitalWrite(PIN_LEFT, LOW); 
    } else {
        digitalWrite(PIN_RIGHT, LOW); digitalWrite(PIN_LEFT, LOW); 
    }
}

void motorTask(void *pvParameters) {
    MotorState currentState = STATE_DUMMY; 
    MotorState incomingState;
  
    for (;;) {
        if (prepareForFlash) vTaskSuspend(NULL); 

        if (isMotorPanic()) {
            motorRotate(ROTATE_RIGHT);
            motorRegularOrPanicWait(0); 
            motorRotate(ROTATE_IDLE);
            motorRegularOrPanicWait(0);
        } else {
            motorRotate(ROTATE_RIGHT);
            motorRegularOrPanicWait(3000); 
            motorRotate(ROTATE_LEFT);
            motorRegularOrPanicWait(20000); 
            motorRotate(ROTATE_RIGHT);
            motorRegularOrPanicWait(3000); 
            motorRotate(ROTATE_IDLE);
            motorRegularOrPanicWait(90000); 

            motorRotate(ROTATE_LEFT);
            motorRegularOrPanicWait(3000); 
            motorRotate(ROTATE_RIGHT);
            motorRegularOrPanicWait(20000); 
            motorRotate(ROTATE_LEFT);
            motorRegularOrPanicWait(3000); 
            motorRotate(ROTATE_IDLE);
            motorRegularOrPanicWait(90000);
        } 
    }
}

void otaTask(void *pvParameters) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.handle();
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void hourlyTask(void *pvParameters) {
  vTaskDelay(10000 / portTICK_PERIOD_MS);

  for(;;) {
    struct tm timeinfo;
    
    if (!getLocalTime(&timeinfo)) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      LOG_PRINTLN("HA: Can't get time on tick");
      continue;
    }

    uint32_t seconds_until_next = ((59 - timeinfo.tm_min) * 60) + (60 - timeinfo.tm_sec);

    if (timeinfo.tm_min == 0 &&  timeinfo.tm_sec == 0) {
      performHourlyAction(timeinfo.tm_hour);
      vTaskDelay(2000 / portTICK_PERIOD_MS); 
    } else {
      uint32_t sleep_time_ms = 20; 
      
      if (seconds_until_next > 180) {
        seconds_until_next = 180;
      }

      if (seconds_until_next > 2) {
        sleep_time_ms = (seconds_until_next - 2) * 1000;
      }
      
      vTaskDelay(sleep_time_ms / portTICK_PERIOD_MS);
    }
  }
}

// --- Initialization Block ---
void initHardware() {
    Serial.begin(115200);
    strip.begin();
    strip.clear();
    strip.show();

    buzzerTimer = xTimerCreate("BuzzerTimer", pdMS_TO_TICKS(180), pdFALSE, (void *)0, buzzerStopCallback);
    
    pinMode(PIN_RIGHT, OUTPUT);
    pinMode(PIN_LEFT, OUTPUT);
    digitalWrite(PIN_RIGHT, LOW);
    digitalWrite(PIN_LEFT, LOW);
    
    ledcAttach(PIN_BUZZER, 2000, 8); 
    ledcWriteTone(PIN_BUZZER, 0);
    
    pinMode(STATUS_LED, OUTPUT);
    ledOn();
    
    showProgress(0);
    
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true 
    };
    esp_task_wdt_reconfigure(&twdt_config);
    esp_task_wdt_add(NULL);
    
    showProgress(1);
}

void initWiFi() {
    WiFi.begin(ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        waitProgress(2);
    }
    Serial.printf("\nWiFi Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    TelnetStream.begin();
    esp_task_wdt_reset(); 
    showProgress(2);
}

void initOTA() {
    ArduinoOTA.setHostname("esp32-rotatotron");
    ArduinoOTA.setPort(1725);
    ArduinoOTA.setPassword(firmware_upload_password); 

    ArduinoOTA.onStart([]() {
        digitalWrite(PIN_RIGHT, LOW);
        digitalWrite(PIN_LEFT, LOW);
        ledcWriteTone(PIN_BUZZER, 0);
        strip.clear();
        strip.show();
        ledOn(); 
        prepareForFlash = true;
        Serial.println("\nOTA Update Started...");
    });
    
    ArduinoOTA.onEnd([]() {
        playStatusChord(true);
        Serial.println("\nOTA Update Finished. Rebooting...");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    xTaskCreate(otaTask, "OTATask", 4096, NULL, 1, NULL);
    showProgress(3);
}

void initServices() {
    motorStateQueue = xQueueCreate(2, sizeof(MotorState));
    configMutex = xSemaphoreCreateMutex();
    buzzerMutex = xSemaphoreCreateMutex();

    prefs.begin("netconf", false);
    xSemaphoreTake(configMutex, portMAX_DELAY);
    
    int numIps = prefs.getInt("num_ips", 0);
    if (numIps == 0) {
        coreServices.push_back(IPAddress(10, 1, 1, 1));
        coreServices.push_back(IPAddress(10, 1, 1, 8));
        coreServices.push_back(IPAddress(10, 1, 1, 7));
        coreServices.push_back(IPAddress(10, 1, 1, 9));
        serviceStatus.assign(4, false);
    } else {
        for(int i = 0; i < numIps; i++) {
            String savedIp = prefs.getString(("ip" + String(i)).c_str(), "");
            IPAddress ip;
            if (ip.fromString(savedIp)) {
                coreServices.push_back(ip);
                serviceStatus.push_back(false);
            }
        }
    }
    xSemaphoreGive(configMutex);

    sntp_set_sync_interval(900 * 1000);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

    struct tm timeinfo;
    LOG_PRINTLN("Waiting for NTP sync");
    while (!getLocalTime(&timeinfo)) {
        waitProgress(4);
    }

    esp_task_wdt_reset();
    showProgress(4);
}

void initSensors() {
    SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
    if (!bme.begin()) {
        playStatusChord(false);
        delay(500); 
        while(true) {
            waitProgress(5);
        }
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); 
    showProgress(5);
}

void initTasks() {
    xTaskCreate(motorTask,  "MotorTask",  2048, NULL, 1, NULL);
    xTaskCreate(pingTask,   "PingTask",   4096, NULL, 1, NULL);
    xTaskCreate(ledTask,    "LEDTask",    4096, NULL, 1, NULL);
    xTaskCreate(serverTask, "ServerTask", 4096, NULL, 1, NULL);
    xTaskCreate(hourlyTask, "HourlyTask", 4096, NULL, 1, NULL);

    ledOn();
    esp_task_wdt_reset();
    showProgress(7);
    
    setupWebServer();
    showProgress(8);
}

// --- Main Setup ---
void setup() {
    initHardware();
    initWiFi();
    initOTA();
    prebakeLUT();
    initServices();
    initSensors();
    initTasks();
}

// --- Main Loop ---
void loop() {
    if (prepareForFlash) {
        static bool shutdownComplete = false;
        if (!shutdownComplete) {
            vTaskDelay(pdMS_TO_TICKS(300)); 
            
            digitalWrite(PIN_RIGHT, LOW);
            digitalWrite(PIN_LEFT, LOW);
            ledcWriteTone(PIN_BUZZER, 0);
            
            strip.clear();
            strip.show();
            
            ledOn(); 
            
            shutdownComplete = true;
            Serial.println("System Locked. Hardware secured. Ready for Flash.");
        }
        
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    ledOn();
    bool isWifiConnected = (WiFi.status() == WL_CONNECTED);

    if(! isWifiConnected) {
        isBroken = true;
    } else if (!bme.performReading()) {
        isBroken = true;
    } else {
        isBroken =  bme.temperature < 1;
    }

    if (isBroken != wasBroken) {
        wasBroken = isBroken;
        playStatusChord(!isBroken);
    }

    if (isBroken) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    currentTemp = bme.temperature;

    String tags = "|#family:esp32";
    char packet[256];
    
    snprintf(packet, sizeof(packet), 
             "esp32.wifi_rssi:%d|g%s\n"
             "esp32.cycle:%lu|g%s\n"
             "esp32.uptime:%.2f|g%s\n"
             "esp32.temperature:%.2f|g%s\n"
             "esp32.humidity:%.2f|g%s\n"
             "esp32.pressure:%.2f|g%s\n"
             "esp32.gas:%.2f|g%s",
             abs(WiFi.RSSI()), tags.c_str(),
             ++cycle, tags.c_str(),
             log(cycle) * 10000.0, tags.c_str(),
             bme.temperature, tags.c_str(),
             bme.humidity, tags.c_str(),
             bme.pressure / 133.322, tags.c_str(),
             bme.gas_resistance / 1000.0, tags.c_str());

    udp.beginPacket(statsd_ip, statsd_port);
    udp.print(packet);
    udp.endPacket();
    
    ledOff();
    LOG_PRINTF("tick %lu temp: %f\n", cycle, bme.temperature);
    esp_task_wdt_reset(); 
 
    vTaskDelay(pdMS_TO_TICKS(3000)); 
}