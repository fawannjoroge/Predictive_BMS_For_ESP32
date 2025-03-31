#include <Arduino.h>
#include <unity.h>

// Define missing constant for display initialization.
#define SSD1306_SWITCHCAPVCC 0x02

// --- Dummy (Mock) Classes for Sensors ---

// Dummy DHT sensor that always returns valid readings.
class DummyDHT {
public:
  void begin() {}
  float readHumidity() { return 50.0; }
  float readTemperature() { return 25.0; }
};

// Dummy INA219 sensor that simulates initialization and readings.
class DummyINA219 {
public:
  bool begin() { return true; }
  void setCalibration_32V_1A() {}
  float getShuntVoltage_mV() { return 10.0; }
  float getBusVoltage_V() { return 12.0; }
  float getCurrent_mA() { return 100.0; }
  float getPower_mW() { return 1200.0; }
};

// Dummy SSD1306 display that simulates a working display.
class DummySSD1306 {
public:
  bool begin(uint8_t vcc, uint8_t addr) { (void)vcc; (void)addr; return true; }
  void clearDisplay() {}
  void display() {}
  void setTextSize(uint8_t size) { (void)size; }
  void setTextColor(uint16_t color) { (void)color; }
  void setCursor(uint8_t x, uint8_t y) { (void)x; (void)y; }
  void print(const char* str) { (void)str; }
  void println(const char* str) { (void)str; }
  void print(float val) { (void)val; }
  void println(float val) { (void)val; }
};

// --- Instantiate Dummy Sensor Objects ---
DummyDHT dht;
DummyINA219 ina219;
DummySSD1306 display;

// Variables used for pulse counting and speed calculation.
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float speedRPM = 0;

// Simulated pulse counter function.
void countPulse() {
  pulseCount++;
}

// --- Test Cases ---

// Test that the DHT sensor returns valid readings.
void test_dht_sensor_reading() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  TEST_ASSERT_FALSE(isnan(humidity));
  TEST_ASSERT_FALSE(isnan(temperature));

  TEST_ASSERT_TRUE(humidity >= 0.0 && humidity <= 100.0);
  TEST_ASSERT_TRUE(temperature >= -40.0 && temperature <= 80.0);
}

// Test that INA219 readings are within expected ranges.
void test_ina219_readings() {
  float busVoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();

  TEST_ASSERT_TRUE(busVoltage >= 0.0 && busVoltage <= 32.0);
  TEST_ASSERT_TRUE(current_mA >= 0.0 && current_mA <= 1000.0);
}

// Test that the speed calculation is correct.
void test_speed_calculation() {
  pulseCount = 60;
  unsigned long elapsedTime = 1000;  // Simulate 1 second
  speedRPM = (pulseCount * 60000.0) / elapsedTime;
  TEST_ASSERT_EQUAL_FLOAT(3600.0, speedRPM);
}

// Test that the display initializes correctly.
void test_display_initialization() {
  TEST_ASSERT_TRUE(display.begin(SSD1306_SWITCHCAPVCC, 0x3C));
}

// Test that INA219 initializes correctly.
void test_ina219_initialization() {
  TEST_ASSERT_TRUE(ina219.begin());
}

// Test the pulse counter functionality.
void test_pulse_counter() {
  pulseCount = 0;
  countPulse();
  TEST_ASSERT_EQUAL_UINT32(1, pulseCount);
  countPulse();
  TEST_ASSERT_EQUAL_UINT32(2, pulseCount);
}

// --- Setup and Run Unity Tests ---
void setup() {
  UNITY_BEGIN();
  
  // Simulated sensor initialization.
  dht.begin();
  ina219.begin();
  ina219.setCalibration_32V_1A();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  // Run tests.
  RUN_TEST(test_dht_sensor_reading);
  RUN_TEST(test_ina219_readings);
  RUN_TEST(test_speed_calculation);
  RUN_TEST(test_display_initialization);
  RUN_TEST(test_ina219_initialization);
  RUN_TEST(test_pulse_counter);
  
  UNITY_END();
}

void loop() {
  // Empty for testing.
}
// --- End of Test Code ---