#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define LM35_PIN 34 
#define MH_PIN 13 
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64  
#define OLED_RESET -1  
#define BATTERY_CAPACITY_AH 7.0  

// Define the wheel circumference in meters (adjust as needed)
#define WHEEL_CIRCUMFERENCE 2.0  

const char* ssid     = "Hedy fidelity";
const char* password = "Onecaster2020";
const char* serverName = "http://192.168.7.231:5000/predict";

const int timeSteps = 10;
const int featureDim = 5;

Adafruit_INA219 ina219; 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float speedRPM = 0;  
volatile unsigned long lastPulseTime = 0;    

float soc = 100.0; 
unsigned long prevSocTime = 0; 

float sensorBuffer[timeSteps][featureDim];
int bufferIndex = 0;

float lastPredictedRange = -1;

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
    }
    Serial.println("Reconnected");
  }
}

void IRAM_ATTR countPulse() {
  unsigned long now = micros();
  if (now - lastPulseTime > 1000) {
    pulseCount++;
    lastPulseTime = now;
  }
}

void i2cScanner() {
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("I2C scan done\n");
}

float getBatterySoC(float voltage) {
  if (voltage >= 11.6) return 100;
  if (voltage >= 11.3) return 90;
  if (voltage >= 11.0) return 80;
  if (voltage >= 10.7) return 70;
  if (voltage >= 10.4) return 60;
  if (voltage >= 10.1) return 50;
  if (voltage >= 9.8) return 40;
  if (voltage >= 9.5) return 30;
  if (voltage >= 9.2) return 20;
  if (voltage >= 8.9)  return 10;
  return 0;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Booting...");

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Attempting to connect...");
  }
  Serial.println("Connected to WiFi");

  Wire.begin(21, 22);
  Wire.setClock(100000);
  Serial.println("I2C initialized at 100kHz");
  i2cScanner();

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
  } else {
    ina219.setCalibration_32V_1A();
    Serial.println("INA219 initialized");
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Starting...");
    display.display();
    Serial.println("SSD1306 initialized");
  }

  pinMode(MH_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MH_PIN), countPulse, FALLING);
  Serial.println("MH Sensor interrupt attached");

  analogSetAttenuation(ADC_11db);

  float initVoltage = ina219.getBusVoltage_V();
  soc = getBatterySoC(initVoltage);
  prevSocTime = millis();
  Serial.print("Initial SoC: ");
  Serial.print(soc);
  Serial.println(" %");

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  reconnectWiFi();

  int rawValue = analogRead(LM35_PIN);
  Serial.print("LM35 raw ADC value: ");
  Serial.println(rawValue);
  float voltageLM35 = rawValue * (3.3 / 4095.0);
  float calibrationFactor = 176.0;
  float temperature = voltageLM35 * calibrationFactor;
  Serial.print("Calculated Temp: ");
  Serial.print(temperature);
  Serial.println(" C");

  float busVoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  if (isnan(current_mA)) {
    Serial.println("Warning: INA219 current reading is NaN. Check wiring and load!");
  }
  Serial.println("INA219 read complete");

  if (current_mA < -1000 || current_mA > 1000) {
    Serial.println("Current reading out of expected range, skipping data send.");
  }

  unsigned long currentSocTime = millis();
  float deltaT_hours = (currentSocTime - prevSocTime) / 3600000.0;
  float current_A = current_mA / 1000.0;
  if (prevSocTime > 0) {
    soc -= (current_A * deltaT_hours) / BATTERY_CAPACITY_AH * 100;
    if (soc < 0) soc = 0;
    if (soc > 100) soc = 100;
  }
  prevSocTime = currentSocTime;
  if (fabs(current_A) < 0.05) {
    float voltageSoC = getBatterySoC(busVoltage);
    soc = (soc * 0.8) + (voltageSoC * 0.2);
  }

  unsigned long elapsedTime = currentTime - lastTime;
  if (elapsedTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(MH_PIN));
    if (pulseCount > 10000) {
      Serial.println("Pulse count too high, resetting");
      pulseCount = 0;
    }
  
    speedRPM = (pulseCount * 60000.0) / elapsedTime;
    pulseCount = 0;
    lastTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(MH_PIN), countPulse, FALLING);
  }
  Serial.println("Speed calculated");

  float speedKMH = speedRPM * WHEEL_CIRCUMFERENCE * 60.0 / 1000.0;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("T:"); display.print(temperature); display.println(" C");
  display.print("V:"); display.print(busVoltage); display.println(" V");
  display.print("C:"); display.print(current_mA); display.println(" mA");
  display.print("P:"); display.print(power_mW); display.println(" mW");
  display.print("S:"); display.print(speedKMH); display.println(" km/h");
  display.print("SoC:"); display.print(soc); display.println(" %");
  display.print("Range: ");
  if(lastPredictedRange >= 0) {
    display.print(lastPredictedRange);
    display.println(" Km");
  } else {
    display.println("N/A");
  }
  display.display();
  Serial.println("Display updated");

  Serial.print("Voltage: ");
  Serial.print(busVoltage);
  Serial.println(" V");
  Serial.print("Current: ");
  Serial.print(current_mA);
  Serial.println(" mA");
  Serial.print("Power: ");
  Serial.print(power_mW);
  Serial.println(" mW");
  Serial.print("Speed: ");
  Serial.print(speedKMH);
  Serial.println(" km/h");
  Serial.print("SoC: ");
  Serial.print(soc);
  Serial.println(" %");
  Serial.println("----------------");

  sensorBuffer[bufferIndex][0] = busVoltage;
  sensorBuffer[bufferIndex][1] = current_mA;
  sensorBuffer[bufferIndex][2] = temperature;
  sensorBuffer[bufferIndex][3] = speedKMH;
  sensorBuffer[bufferIndex][4] = soc;
  bufferIndex++;

  if(bufferIndex >= timeSteps) {
    if(WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverName);
      http.addHeader("Content-Type", "application/json");
      
      DynamicJsonDocument doc(2048); 
      JsonArray inputArray = doc.createNestedArray("inputs");
      for (int i = 0; i < timeSteps; i++) {
        JsonArray row = inputArray.createNestedArray();
        for (int j = 0; j < featureDim; j++) {
          row.add(sensorBuffer[i][j]);
        }
      }
      
      String jsonString;
      serializeJson(doc, jsonString);
      
      int httpResponseCode = http.POST(jsonString);
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("Response: " + response);
        
        DynamicJsonDocument respDoc(512);
        DeserializationError error = deserializeJson(respDoc, response);
        if (!error) {
          JsonArray predictedArray = respDoc["predicted_range"].as<JsonArray>();
          if (predictedArray.size() > 0) {
            JsonArray innerArray = predictedArray[0];
            lastPredictedRange = innerArray[0];
            Serial.print("Predicted Range: ");
            Serial.println(lastPredictedRange);
          }
        } else {
          Serial.print("Error parsing JSON: ");
          Serial.println(error.c_str());
        }
      } else {
        Serial.println("Error on sending POST: " + String(httpResponseCode));
      }
      
      http.end();
    } else {
      Serial.println("WiFi not connected. Cannot send data.");
    }
    bufferIndex = 0;
  }

  Serial.print(temperature, 2);
  Serial.print(",");
  Serial.print(busVoltage, 2);
  Serial.print(",");
  Serial.print(current_mA, 2);
  Serial.print(",");
  Serial.print(power_mW, 2);
  Serial.print(",");
  Serial.print(speedKMH, 2);
  Serial.print(",");
  Serial.print(soc, 2);
  Serial.print(",");
  Serial.println(lastPredictedRange, 2);
  
  delay(5000);
}