#include <Arduino.h> // Essential for ESP32 functions
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1327.h>
#include <Adafruit_SH110X.h>
#include "Adafruit_MAX31856.h"
#include <PID_v1.h>
#include <WiFi.h>
#include "esp32-hal-timer.h"
#include <PubSubClient.h>

#define DEFAULT_I2C_PORT &Wire1



// Images 
static const unsigned char PROGMEM image_wifi_full_bits[] = {0x01,0xf0,0x00,0x07,0xfc,0x00,0x1e,0x0f,0x00,0x39,0xf3,0x80,0x77,0xfd,0xc0,0xef,0x1e,0xe0,0x5c,0xe7,0x40,0x3b,0xfb,0x80,0x17,0x1d,0x00,0x0e,0xee,0x00,0x05,0xf4,0x00,0x03,0xb8,0x00,0x01,0x50,0x00,0x00,0xe0,0x00,0x00,0x40,0x00,0x00,0x00,0x00};
static const unsigned char PROGMEM image_weather_temperature_bits[] = {0x1c,0x00,0x22,0x02,0x2b,0x05,0x2a,0x02,0x2b,0x38,0x2a,0x60,0x2b,0x40,0x2a,0x40,0x2a,0x60,0x49,0x38,0x9c,0x80,0xae,0x80,0xbe,0x80,0x9c,0x80,0x41,0x00,0x3e,0x00};
static const unsigned char PROGMEM image_wifi_not_connected_bits[] = {0x21,0xf0,0x00,0x16,0x0c,0x00,0x08,0x03,0x00,0x25,0xf0,0x80,0x42,0x0c,0x40,0x89,0x02,0x20,0x10,0xa1,0x00,0x23,0x58,0x80,0x04,0x24,0x00,0x08,0x52,0x00,0x01,0xa8,0x00,0x02,0x04,0x00,0x00,0x42,0x00,0x00,0xa1,0x00,0x00,0x40,0x80,0x00,0x00,0x00};
static const unsigned char PROGMEM image_FaceNormal_bits[] = {0x00,0x00,0x00,0x00,0x3c,0x00,0x01,0xe0,0x7a,0x00,0x03,0xd0,0x7e,0x00,0x03,0xf0,0x7e,0x00,0x03,0xf0,0x7e,0x00,0x03,0xf0,0x3c,0x00,0x01,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x40,0x00,0x00,0x10,0x40,0x00,0x00,0x10,0x40,0x00,0x00,0x08,0x80,0x00,0x00,0x07,0x00,0x00};
static const unsigned char PROGMEM image_weather_wind_bits[] = {0x00,0x00,0x00,0x00,0x00,0x30,0x03,0x88,0x04,0x44,0x04,0x44,0x00,0x44,0x00,0x88,0xff,0x32,0x00,0x00,0xad,0x82,0x00,0x60,0x00,0x10,0x00,0x10,0x01,0x20,0x00,0xc0};
static const unsigned char PROGMEM image_Restoring_bits[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0x00,0x00,0x00,0x07,0x01,0x80,0x00,0x00,0x18,0x00,0x40,0x00,0x00,0x20,0x7e,0x40,0x00,0x00,0x43,0xff,0xc0,0x00,0x00,0x87,0x81,0x80,0x00,0x00,0x8c,0x00,0x02,0x00,0x01,0x10,0x00,0x05,0x00,0x01,0x10,0x00,0x08,0x80,0x02,0x20,0x00,0x10,0x40,0x02,0x20,0x00,0x20,0x20,0x04,0x40,0x00,0x40,0x10,0x04,0x40,0x00,0x80,0x08,0x04,0x40,0x00,0xf8,0xf8,0x7c,0x7c,0x00,0xf8,0xf8,0x40,0x04,0x00,0x08,0x80,0x60,0x0c,0x00,0x08,0x80,0x70,0x1c,0x00,0x11,0x00,0x38,0x38,0x00,0x11,0x00,0x1c,0x70,0x00,0x22,0x00,0x0e,0xe0,0x00,0x22,0x00,0x07,0xc0,0x00,0xc4,0x00,0x03,0x80,0x00,0x84,0x00,0x00,0x06,0x07,0x08,0x00,0x00,0x09,0xf8,0x10,0x00,0x00,0x08,0x00,0x70,0x00,0x00,0x0e,0x03,0xe0,0x00,0x00,0x07,0xff,0x80,0x00,0x00,0x01,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const unsigned char PROGMEM image_poopface_bits[] = {0x00,0x00,0x00,0x00,0x03,0x00,0x06,0x00,0x31,0x80,0x0c,0x60,0x18,0x80,0x08,0xc0,0x0c,0x00,0x01,0x80,0x1e,0x00,0x03,0xc0,0x3c,0x00,0x01,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x07,0x0d,0x87,0x00,0x01,0x18,0xc4,0x00,0x01,0x0d,0x84,0x00,0x00,0x07,0x00,0x00};


// OLED variables
bool oledAvailable = false;
#define OLED_ADDR   0x3D
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET, 1000000, 100000);

// Fan PWM output variables
#define FPIN_OUT 15
#define FPWM_Res 8
#define FPWM_Freq 250000  // 250 kHz
int FPWM_DutyCycle = 0;
int FanSpeed = 0;
int FanStep = 10;

// Thermocouple variables
bool thermocoupleAvailable = false;
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(32);
int curTemp = 0;
int tarTemp = 0;
unsigned long LastReadTime = 0;
int TempStep = 5;

// Motor variables
int OutStep = 26;
int OutDir = 25;
int OutEnable = 27;
bool MotorEnable = false;
float MotorRev = 800;
int MotorSpeed = 0;  // RPM
int MotorStep = 25;
bool ignoreRobotMotorSpeed = false;

// Hardware timer for motor control
hw_timer_t* timer = NULL;
bool stepState = false; // Tracks the state of the step pin

void updateMotorSpeed(int newSpeed);

// Timer interrupt callback function
void IRAM_ATTR onTimer() {
  digitalWrite(OutStep, stepState); // Toggle step pin
  stepState = !stepState; // Invert step state for next interrupt
}

//WIFI Setup---------------------------------------------
WiFiClient tcp;
IPAddress static_IP(192, 168, 0, 205);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);

bool wifiConnected = false;
const char* ssid = "simplifiedintegration";
const char* password = "acadia2024";

//MQTT Setup Section-------------------------------------
const char* mqtt_server = "192.168.0.200";
PubSubClient mqtt(tcp);
const int port = 1883;
const char* topic_motorEnable = "extruder/motorEnable";
const char* topic_motorSpeed = "extruder/motorSpeed";
const char* topic_fanSpeed = "extruder/fanSpeed";
const char* topic_tarTemp = "extruder/tarTemp";
const char* topic_curTemp = "extruder/curTemp";


// Variables to hold sensor readings from MQTT
volatile int32_t rawMotorSpeed = 0;
volatile int32_t rawFanSpeed = 0;
volatile int32_t rawTarTemp = 0;
volatile bool rawMotorEnable = false;

// Update setup test variables
unsigned long lastOLEDUpdate = 0;
const unsigned long oledUpdateInterval = 5000; // 5 seconds in milliseconds
bool needImmediateUpdate = false;

// Sensor update variables
unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 50; // Update interval of 50 ms

// Variables for the last state to detect changes
int32_t lastMotorSpeed = -1;
int32_t lastFanSpeed = -1;
int32_t lastTarTemp = -1;
bool lastMotorEnable = false;
int lastCurTemp = -999; // An unlikely initial value

void setup() {
  // Serial setup
  Serial.begin(115200);

  // Initialize WiFi
  setupWiFi();
  // Initialize MQTT
  

//   // Start I2C communication
// #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || \
//     defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || \
//     defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || \
//     defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
//   // ESP32 is kinda odd in that secondary ports must be manually
//   // assigned their pins with setPins()!
//   Wire.setPins(SDA1, SCL1);
// #endif

  // Setup thermocouple
  Serial.println("MAX31856 thermocouple setup");
  if (!thermocouple.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);
  thermocouple.setConversionMode(MAX31856_CONTINUOUS);
  if (thermocouple.conversionComplete()) {
    curTemp = thermocouple.readThermocoupleTemperature();
    Serial.println(curTemp);
    thermocoupleAvailable = true;
  } else {
    Serial.println("Conversion not complete!");
  }

  // Fan PWM output using new LEDC API
  // Using ledcAttach to replace ledcSetup and ledcAttachPin
  ledcAttach(FPIN_OUT, FPWM_Freq, FPWM_Res);
  // Initialize the PWM duty cycle
  ledcWrite(FPIN_OUT, FPWM_DutyCycle);
  Serial.println("Cooling Fan initialized successfully.");

  // OLED setup
  oledAvailable = display.begin(OLED_ADDR, true); // Initialize OLED and check if it's available
  if (oledAvailable) {
    delay(2000);
    display.display(); // Show whatever is initially on the buffer (usually Adafruit splash screen)
    display.clearDisplay(); // Clear the display buffer
    Serial.println("OLED display initialized successfully.");
  } else {
    Serial.println("Failed to initialize OLED display. Proceeding without OLED.");
  }

  // Motor setup
  pinMode(OutDir, OUTPUT);
  pinMode(OutStep, OUTPUT);
  pinMode(OutEnable, OUTPUT);
  MotorEnable = rawMotorEnable;  // Initialize motor enable state
  lastMotorEnable = MotorEnable;
  digitalWrite(OutEnable, MotorEnable);  // Activate extruder motor
  digitalWrite(OutDir, true);     // Anti-Clockwise
  Serial.println("Motor Enable: " + String(MotorEnable));

  // Initialize hardware timer for motor control
  uint32_t timerFrequency = 1000000; // 1 MHz frequency for microsecond precision
  timer = timerBegin(timerFrequency);

  // timerSetPrescaler(timer, 80); // Set prescaler to 80 (1us per tick)
  // timerSetCountUp(timer, true); // Set count direction to up
  timerAttachInterrupt(timer, &onTimer);
  uint64_t alarm_value = 1000000; // Initial alarm value (1 second)
  bool autoreload = true;         // Auto-reload the timer
  uint64_t reload_count = 0;      // 0 for unlimited reloads
  timerAlarm(timer, alarm_value, autoreload, reload_count);
  Serial.println("Motor setup OK");

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);
}

void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();
  unsigned long currentMillis = millis();
  
  // Update sensor readings every sensorUpdateInterval (50 ms)
  if (currentMillis - lastSensorUpdate >= sensorUpdateInterval) {
    lastSensorUpdate = currentMillis; // Update the last update time

    // Read current temperature from thermocouple
    curTemp = round(thermocouple.readThermocoupleTemperature());

    // Constrain the raw values to their respective limits
    MotorSpeed = constrain(rawMotorSpeed, 0, 500);
    FanSpeed = constrain(rawFanSpeed, 0, 100);
    tarTemp = constrain(rawTarTemp, 0, 500);

    // Check if there's a change in any value
    bool updated = false;

    if (MotorSpeed != lastMotorSpeed) {
      lastMotorSpeed = MotorSpeed;
      updateMotorSpeed(MotorSpeed); // Update motor speed
      updated = true;
    }

    if (FanSpeed != lastFanSpeed) {
      lastFanSpeed = FanSpeed;
      updateFan(); // Update fan speed
      updated = true;
    }

    if (tarTemp != lastTarTemp) {
      lastTarTemp = tarTemp;
      // Implement temperature control logic here if needed
      updated = true;
    }

    if (curTemp != lastCurTemp) {
      lastCurTemp = curTemp;
      // You can publish current temperature if needed
      updated = true;
    }

    if (MotorEnable != lastMotorEnable) {
      lastMotorEnable = MotorEnable;
      digitalWrite(OutEnable, MotorEnable); // Update motor enable state
      updated = true;
    }

    if (updated) {
      needImmediateUpdate = true; // Mark for immediate OLED update
      updateOLED();
      Serial.println("Values updated!");
    }
  }

  // Update OLED display if needed
  if (needImmediateUpdate || currentMillis - lastOLEDUpdate >= oledUpdateInterval) {
    updateOLED();
    lastOLEDUpdate = currentMillis;
    needImmediateUpdate = false;
  }
}

// Function to set up WiFi connection
void setupWiFi() {
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  if (!WiFi.config(static_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);

  int maxRetries = 10; // Maximum number of retries for WiFi connection
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < maxRetries) {
    delay(1000);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi. Proceeding without network connectivity.");
    wifiConnected = false;
  }
}

// Function to update the OLED display
void updateOLED(void) {
  if (!oledAvailable) return; // Ensure OLED is available

  display.clearDisplay();
  display.setTextSize(1);
  // Draw a dividing line
  display.drawLine(0, 18, 128, 18, SSD1327_WHITE);

  // Display WiFi status with an icon based on connection status
  if (wifiConnected) {
    // Display WiFi connected icon
    display.drawBitmap(109, 0, image_wifi_full_bits, 19, 16, SSD1327_WHITE);
    display.setCursor(1, 100); // Position for IP address
    display.print(WiFi.localIP());
  } else {
    // Display WiFi not connected icon
    display.drawBitmap(109, 1, image_wifi_not_connected_bits, 19, 16, SSD1327_WHITE);
    display.setCursor(1, 100);
    display.print("No WiFi");
  }

  // Display temperature and fan speed information
  display.setTextSize(1);
  display.setTextColor(SSD1327_WHITE);

  // Temperature icon and text
  display.drawBitmap(3, 21, image_weather_temperature_bits, 16, 16, SSD1327_WHITE);
  display.setCursor(23, 26); // Adjusted for "curTemp / tarTemp" text
  if (thermocoupleAvailable) {
    display.print(curTemp);
    display.print("/");
    display.print(tarTemp);
  } else {
    display.print("N/A");
  }

  // Fan speed icon and text
  display.drawBitmap(2, 43, image_weather_wind_bits, 15, 16, SSD1327_WHITE);
  display.setCursor(27, 49); // Adjusted for "FanSpeed" text
  display.print(FanSpeed);

  // RPM text
  display.setTextSize(2); // Larger text size for emphasis
  display.setCursor(49, 75); // Position for "RPM" text
  if (MotorEnable) {
    display.print(MotorSpeed);  // Display motor speed
  } else {
    display.print("OFF");       // Motor is not enabled, display OFF
  }

  // Additional UI elements
  display.drawBitmap(2, 64, image_Restoring_bits, 38, 32, SSD1327_WHITE);
   if ( MotorSpeed >= 1 ){
      display.drawBitmap(46, 2, image_poopface_bits, 29, 14, SSD1327_WHITE);
    }
    else{
      display.drawBitmap(46, 2, image_FaceNormal_bits, 29, 14, SSD1327_WHITE);
    }
    display.setTextSize(1);
    display.drawLine(0, 97, 127, 97,  SSD1327_WHITE);
    display.drawLine(0, 110, 127, 110,  SSD1327_WHITE);
    display.setCursor(35, 112); // "NeoStruder 0.2" text position
    display.print("NeoStruder 0.2");
    display.setCursor(35, 120); // "by Luis Pacheco" text position
    display.print("by Luis Pacheco");

  // Update the display with the new information
  display.display();
}

// Function to update the fan speed based on FanSpeed variable
void updateFan() {
  FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
  ledcWrite(FPIN_OUT, FPWM_DutyCycle); // Use pin number instead of channel
}

// Function to update the motor speed based on MotorSpeed variable
void updateMotorSpeed(int newSpeed) {
  MotorSpeed = newSpeed; // Update the global MotorSpeed variable with the new speed

  // Calculate interval in microseconds for one step
  unsigned long interval;
  if (MotorSpeed > 0) {
    float stepsPerSecond = MotorSpeed * MotorRev / 60.0; // Convert RPM to steps per second
    interval = (1000000.0 / stepsPerSecond) / 2; // Interval in microseconds for half a step (toggle)
  } else {
    interval = ULONG_MAX; // Effectively stops the motor
  }

  // Update timer alarm value to change the speed
  timerAlarm(timer, interval, true, 0);
}
void callback(char* topic, byte* payload, unsigned int length) {
   
   char messageBuffer[length + 1];
    memcpy(messageBuffer, payload, length);
    messageBuffer[length] = '\0'; // Null-terminate the buffer

   if(strcmp(topic,topic_motorEnable) == 0){
        MotorEnable = (bool)atoi(messageBuffer);
        Serial.println("Motor Enable: "); 
        Serial.println(MotorEnable);
    }
    else if(strcmp(topic,topic_motorSpeed) == 0){
        rawMotorSpeed = atoi(messageBuffer);
        Serial.println("Motor Speed");
        Serial.println(rawMotorSpeed );
    }
    else if(strcmp(topic,topic_fanSpeed) == 0){
        rawFanSpeed = atoi(messageBuffer);
        Serial.println("Motor Speed");
        Serial.println(rawFanSpeed); 
    }
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
       mqtt.subscribe(topic_motorEnable);
       mqtt.subscribe(topic_motorSpeed);
       mqtt.subscribe(topic_fanSpeed);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// void reconnect() {
//   while (!mqtt.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     if (mqtt.connect("ESP32Client2")) {
//       Serial.println("connected");
//       mqtt.subscribe("device/motorEnable");
//       mqtt.subscribe("device/motorSpeed");
//       mqtt.subscribe("device/fanSpeed");
//       mqtt.subscribe("device/tarTemp");
//     } else {
//       Serial.print("failed, rc=");
//       Serial.print(mqtt.state());
//       Serial.println(" try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }
