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
const int NUM_PIXELS    = 9;

// --- Network & Configuration ---
const char* ssid       = "mywifinet";
const char* password   = "mywifipass";
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

volatile float currentTemp = 0.0;
volatile bool prepareForFlash = false; // Flash-mode safety flag
volatile bool isBroken = false;
bool wasBroken = false; 

// LAN Core Services State (Dynamic)
std::vector<IPAddress> coreServices;
std::vector<bool> serviceStatus;
volatile bool initialScanComplete = false;

int GAMMA_NOTES_COUNT = 8;
int cMajorGamma[] = {523, 587, 659, 698, 784, 880, 988, 1047};
// Notes:            C5   D5   E5   F5   G5   A5   B5   C6

TimerHandle_t buzzerTimer;

enum MotorState {
    STATE_DUMMY
};

struct ColorPoint { float temp; uint8_t r, g, b; };
ColorPoint palette[] = {
    {10.0, 0, 0, 255},       // Deep Blue
    {15.0, 0, 255, 255},     // Cyan
    {20.0, 255, 105, 180},   // Pink
    {25.0, 255, 0, 0},       // Red
    {30.0, 255, 165, 0},     // Orange
    {40.0, 255, 255, 0}      // Yellow
};

// --- Helper Functions ---
void ledOn() { digitalWrite(STATUS_LED, LOW); }
void ledOff() { digitalWrite(STATUS_LED, HIGH); }
void buzzerStopCallback(TimerHandle_t xTimer) {
    ledcWriteTone(PIN_BUZZER, 0); 
}

void beepAsync(int freq, int durationMs) {
    if (prepareForFlash) return; 
    
    ledcWriteTone(PIN_BUZZER, freq); // 1. Start the sound immediately
    
    // 2. Set the stopwatch duration and start it. 
    // (A wait time of 0 means don't block if the timer queue is full)
    xTimerChangePeriod(buzzerTimer, pdMS_TO_TICKS(durationMs), 0); 
}

void showProgress(int cut) {
    if (cut < 0) {
        cut = 0;
    }
    if (cut >= NUM_PIXELS) {
        cut = NUM_PIXELS -1;
    }

    beepAsync(cMajorGamma[cut % GAMMA_NOTES_COUNT], 100);

    for (int i = 0; i < NUM_PIXELS; i++) {
        strip.setPixelColor(NUM_PIXELS -i -1, i<=cut ? strip.Color(0, 255,0) : strip.Color(0, 0, 0));
    }            
    strip.show();
}

void playStatusChord(bool isUp) {
    if (prepareForFlash) return; // Abort if flashing
    
    int indicesUp[]   = {0, 2, 4};       // C5, E5, G5 (Ascending Major)
    int indicesDown[] = {5, 3, 1, 0, 0}; // A5, F5, D5, C5, C5 (Descending/Failure)
    
    for(int i = 0; i < (isUp ? 3 : 5); i++) {
        if (prepareForFlash) break; 
        
        // Grab the note frequency from the gamma array using the index
        int targetFreq = cMajorGamma[isUp ? indicesUp[i] : indicesDown[i]];
        
        ledcWriteTone(PIN_BUZZER, targetFreq);
        vTaskDelay(pdMS_TO_TICKS(180)); 
    }
    ledcWriteTone(PIN_BUZZER, 0);
}

void getTemperatureRGB(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    if (t <= 10.0) { r = 0; g = 0; b = 255; return; }
    if (t >= 40.0) { r = 255; g = 255; b = 0; return; }

    for (int i = 0; i < 5; i++) {
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

// --- API Endpoint Setup ---
void setupWebServer() {
    // curl -X POST http://10.1.1.4:9980/api/ips -H "Content-Type: application/json" -d '{"ips": ["10.1.1.1", "10.1.1.8", "10.1.1.9", "10.1.1.7"]}'
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
    

    // curl -X POST http://10.1.1.4:9980/api/brick
    server.on("/api/brick", HTTP_POST, []() {
        prepareForFlash = true;

        server.send(200, "application/json", "{\"status\":\"flash_mode_active\", \"message\":\"Hardware suspended. Ready for firmware upload.\"}");
        playStatusChord(true);
    });

    // curl -X POST http://10.1.1.4:9980/api/gamma
    server.on("/api/gamma", HTTP_POST, []() {
        for (int i=0; i < NUM_PIXELS; i++) {
            showProgress(i);
            vTaskDelay(pdMS_TO_TICKS(120));
        }
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
        if (prepareForFlash) vTaskSuspend(NULL); // Commit task suicide if flashing

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

            if (triggerChord) playStatusChord(chordType);
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        } else {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

void ledTask(void *pvParameters) {
    float smoothTemp = 0.0; 
    float smoothVelocity = 0.0;
    float phase = 0.0;

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
        std::vector<bool> currentStatus;

        xSemaphoreTake(configMutex, portMAX_DELAY);
        initComplete = initialScanComplete;
        currentStatus = serviceStatus; 
        xSemaphoreGive(configMutex);

        if (currentStatus.empty()) {
            allUp = true; 
        } else {
            for (bool status : currentStatus) {
                if (!status) { allUp = false; break; }
            }
        }

        if (!initComplete && !currentStatus.empty()) allUp = false; 

        if (allUp) {
            if (currentTemp > 0) {
                // The ONLY conditional required is to snap the baseline on the very first boot 
                // so it doesn't take 5 minutes to mathematically crawl from 0°C to 25°C.
                if (smoothTemp == 0.0) {
                    smoothTemp = currentTemp; 
                } 

                // 1. Calculate the tension between the raw sensor and the smoothed display state
                float delta = currentTemp - smoothTemp;

                // 2. Heavy Inertial Temperature (Drives the color transition)
                // Climbs toward the raw reading at 1% of the distance per frame.
                smoothTemp += delta * 0.01; 

                // 3. Heavy Inertial Velocity (Drives the sine wave phase)
                // The wave speed is a direct factor of the temperature tension.
                float targetVelocity = delta * 4.0; // Multiplier defines sensitivity to sudden spikes
                
                // Momentum bleeds in/out at 5% per frame for smooth spin-up and spin-down
                smoothVelocity += (targetVelocity - smoothVelocity) * 0.05; 

                // Apply velocity to the phase
                phase += smoothVelocity;

                // --- RENDER ---
                uint8_t baseR, baseG, baseB;
                getTemperatureRGB(smoothTemp, baseR, baseG, baseB);

                for (int i = 0; i < NUM_PIXELS; i++) {
                    float val = sin((float)i * 0.8 + phase);
                    if (val > 1.0) val = 1.0;
                    if (val < -1.0) val = -1.0;  
                    float brightness = (val + 1.0) / 2.0; 
                    
                    strip.setPixelColor(i, strip.Color(
                        (uint8_t)(baseR * brightness), 
                        (uint8_t)(baseG * brightness), 
                        (uint8_t)(baseB * brightness)
                    ));
                }
                strip.show();
            }
            vTaskDelay(pdMS_TO_TICKS(33)); 
        } else {
            strip.clear();
            for (size_t i = 0; i < currentStatus.size() && i < NUM_PIXELS; i++) {
                strip.setPixelColor(i, currentStatus[i] ? strip.Color(0, 255, 0) : strip.Color(255, 0, 0));
            }
            strip.show();
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
    }
}

void motorTask(void *pvParameters) {
    MotorState currentState = STATE_DUMMY; 
    MotorState incomingState;
  

    for (;;) {
        if (prepareForFlash) vTaskSuspend(NULL); // Commit task suicide if flashing

        //BaseType_t stateReceived = xQueueReceive(motorStateQueue, &incomingState, currentTimeout);

        digitalWrite(PIN_RIGHT, HIGH); digitalWrite(PIN_LEFT, LOW); 
        vTaskDelay(pdMS_TO_TICKS(3000)); 
        digitalWrite(PIN_RIGHT, LOW); digitalWrite(PIN_LEFT, HIGH); 
        vTaskDelay(pdMS_TO_TICKS(20000)); 
        digitalWrite(PIN_RIGHT, LOW); digitalWrite(PIN_LEFT, LOW); 
        vTaskDelay(pdMS_TO_TICKS(90000)); 

        digitalWrite(PIN_RIGHT, LOW); digitalWrite(PIN_LEFT, HIGH); 
        vTaskDelay(pdMS_TO_TICKS(3000)); 
        digitalWrite(PIN_RIGHT, HIGH); digitalWrite(PIN_LEFT, LOW); 
        vTaskDelay(pdMS_TO_TICKS(20000)); 
        digitalWrite(PIN_RIGHT, LOW); digitalWrite(PIN_LEFT, LOW); 
        vTaskDelay(pdMS_TO_TICKS(90000)); 
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

// --- Main Setup ---
void setup() {
    Serial.begin(115200);
    strip.begin();
    strip.clear();
    strip.show();
    
    buzzerTimer = xTimerCreate(
        "BuzzerTimer",         // Internal name
        pdMS_TO_TICKS(180),    // Default duration
        pdFALSE,               // pdFALSE = One-shot (don't repeat)
        (void *)0,             // Timer ID (unused here)
        buzzerStopCallback     // The function to run when time is up
    );
    
    pinMode(PIN_RIGHT, OUTPUT);
    pinMode(PIN_LEFT, OUTPUT);
    digitalWrite(PIN_RIGHT, LOW);
    digitalWrite(PIN_LEFT, LOW);
    ledcAttach(PIN_BUZZER, 2000, 8); 
    ledcWriteTone(PIN_BUZZER, 0);
    
    pinMode(STATUS_LED, OUTPUT);
    ledOn();
    showProgress(0);

    // watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true 
    };
    esp_task_wdt_reconfigure(&twdt_config);
    esp_task_wdt_add(NULL);
    showProgress(1);

    // wifi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        ledcWriteTone(PIN_BUZZER, cMajorGamma[3]);
        delay(100); 
        ledcWriteTone(PIN_BUZZER, 0);
        delay(2900);
    }
    esp_task_wdt_reset(); 
    showProgress(2);

    // --- Arduino OTA Setup ---
    ArduinoOTA.setHostname("esp32-rotatotron");
    ArduinoOTA.setPassword(password); // Re-using your WiFi password for OTA auth

    ArduinoOTA.onStart([]() {
        // Forcefully drop all hardware lines to 0 to prevent brownouts
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
    // Spawn the OTA listener task
    xTaskCreate(otaTask, "OTATask", 4096, NULL, 1, NULL);
    showProgress(3);

    // tasks 1
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
    showProgress(4);

    // bme
    SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
    if (!bme.begin()) {
        playStatusChord(false);
        delay(500); // Let the chord finish
        while(true) {
            ledcWriteTone(PIN_BUZZER, cMajorGamma[3]);
            delay(100); 
            ledcWriteTone(PIN_BUZZER, 0);
            delay(2900);
        }
    }

    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); 
    showProgress(5);
    
    setupWebServer();
    showProgress(6);


    // tasks 2
    xTaskCreate(motorTask,  "MotorTask",  2048, NULL, 1, NULL);
    xTaskCreate(pingTask,   "PingTask",   4096, NULL, 1, NULL);
    xTaskCreate(ledTask,    "LEDTask",    4096, NULL, 1, NULL);
    xTaskCreate(serverTask, "ServerTask", 4096, NULL, 1, NULL);

    ledOn();
    esp_task_wdt_reset();
    showProgress(7);
}

// --- Main Loop ---
void loop() {
    // --- The Flash Shutdown Override ---
    if (prepareForFlash) {
        static bool shutdownComplete = false;
        if (!shutdownComplete) {
            vTaskDelay(pdMS_TO_TICKS(300)); // Allow running tasks 300ms to exit loops and suspend
            
            // Forcefully drop all hardware lines to 0
            digitalWrite(PIN_RIGHT, LOW);
            digitalWrite(PIN_LEFT, LOW);
            ledcWriteTone(PIN_BUZZER, 0);
            
            strip.clear();
            strip.show();
            
            ledOn(); // Keep the onboard LED solid on to indicate it's ready
            
            shutdownComplete = true;
            Serial.println("System Locked. Hardware secured. Ready for Flash.");
        }
        
        // Feed watchdog to prevent kernel panic, do nothing else.
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
        //MotorState newState = isWifiConnected ? STATE_WIFI_CONNECTED : STATE_WIFI_DISCONNECTED;
        //xQueueSend(motorStateQueue, &newState, 0);
        wasBroken = isBroken;
        playStatusChord(!isBroken);
    }

    if (isBroken) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    currentTemp = bme.temperature;

    String tags = "|#family:esp32";
    String packet = "";
    
    packet += "esp32.wifi_rssi:"   + String(abs(WiFi.RSSI()))          + "|g" + tags + "\n";
    packet += "esp32.cycle:"       + String(++cycle)                   + "|g" + tags + "\n";
    packet += "esp32.uptime:"      + String(log(cycle) * 10000.0)      + "|g" + tags + "\n";
    packet += "esp32.temperature:" + String(bme.temperature)           + "|g" + tags + "\n";
    packet += "esp32.humidity:"    + String(bme.humidity)              + "|g" + tags + "\n";
    packet += "esp32.pressure:"    + String(bme.pressure / 133.322)    + "|g" + tags + "\n";
    packet += "esp32.gas:"         + String(bme.gas_resistance / 1000) + "|g" + tags;

    udp.beginPacket(statsd_ip, statsd_port);
    udp.print(packet);
    udp.endPacket();
    
    ledOff();
    esp_task_wdt_reset(); 
 
    vTaskDelay(pdMS_TO_TICKS(3000)); 
}