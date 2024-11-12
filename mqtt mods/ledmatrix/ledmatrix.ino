#include <Arduino.h> // Essential for ESP32 functions
#include <PicoMQTT.h>
#include <Adafruit_GFX.h>       // Graphics library
#include <Adafruit_NeoPixel.h>  // NeoPixel library
#include <Adafruit_NeoMatrix.h> // Bridges GFX and NeoPixel

// WIFI Setup
WiFiClient tcp;
IPAddress static_IP(192, 168, 0, 202);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);

bool wifiConnected = false;
const char* ssid = "simplifiedintegration";
const char* password = "acadia2024";

// MQTT setup
PicoMQTT::Client mqtt("192.168.0.200");  // Broker IP address

#define PIN A3
#define MATRIX_WIDTH 5
#define MATRIX_HEIGHT 5
#define LED_COUNT (MATRIX_WIDTH * MATRIX_HEIGHT)

// NeoMatrix declaration for a 5x5 matrix
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, PIN,
  NEO_MATRIX_TOP + NEO_MATRIX_RIGHT + NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB + NEO_KHZ800);

// Variables for state
bool led_enable = false;
uint32_t led_colors[LED_COUNT]; // Array to store the colors of all LEDs

// Variables to track the last state
bool last_led_enable = false;
uint32_t last_led_colors[LED_COUNT];

// Function prototypes
void setupWifi();
void setupMqtt();
void setupLed();
void onMqttMessage(const char* topic, PicoMQTT::IncomingPacket& packet);
void updateLED();
void checkChanges();

void setup() {
  Serial.begin(115200);
  setupWifi();
  setupMqtt();
  setupLed();
}

void loop() {
  mqtt.loop(); // Handle MQTT communication
  checkChanges();
}

void setupWifi() {
  // Configure static IP
  if (!WiFi.config(static_IP, gateway, subnet)) {
    Serial.println("Failed to configure Static IP");
  }

  // Connect to WiFi
  Serial.printf("Connecting to WiFi %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void setupMqtt() {
  mqtt.subscribe("ledmatrix/enable", onMqttMessage);
  mqtt.subscribe("ledmatrix/colors", onMqttMessage); // New topic for RGB values
  mqtt.begin();
  Serial.println("ledmatrix/enable");
  Serial.println("ledmatrix/colors");
}

void setupLed() {
  matrix.begin(); // Initialize NeoMatrix
  matrix.setBrightness(255); // Set brightness level

  // Set all LEDs to white
  for (int i = 0; i < LED_COUNT; i++) {
    int x = i % MATRIX_WIDTH;
    int y = i / MATRIX_WIDTH;
    matrix.drawPixel(x, y, matrix.Color(255, 255, 255)); // White color
  }
  matrix.show(); // Update the display
}

void onMqttMessage(const char* topic, PicoMQTT::IncomingPacket& packet) {
  String addr = String(topic);
  String valueStr = "";
  while (packet.available()) {
    valueStr += (char)packet.read(); // Append each character to the string
  }

  if (addr == "ledmatrix/enable") {
    led_enable = (valueStr == "1");
    Serial.print("Enable: ");
    Serial.println(led_enable);
  } else if (addr == "ledmatrix/colors") {
    // Parse the array of RGB values
    int ledIndex = 0;
    int start = 0;
    int comma1, comma2, semicolon;

    // Parse the comma-separated RGB values
    while (ledIndex < LED_COUNT) {
      comma1 = valueStr.indexOf(',', start);
      comma2 = valueStr.indexOf(',', comma1 + 1);
      semicolon = valueStr.indexOf(';', comma2 + 1);

      if (comma1 == -1 || comma2 == -1) break; // Exit if the format is incorrect

      int r = valueStr.substring(start, comma1).toInt();
      int g = valueStr.substring(comma1 + 1, comma2).toInt();
      int b = valueStr.substring(comma2 + 1, semicolon).toInt();

      // Store the color in the array
      led_colors[ledIndex] = matrix.Color(r, g, b);

      // Move to the next LED
      start = semicolon + 1;
      ledIndex++;
    }

    Serial.println("RGB values updated.");
  }
}

void updateLED() {
  if (led_enable) {
    for (int i = 0; i < LED_COUNT; i++) {
      int x = i % MATRIX_WIDTH;
      int y = i / MATRIX_WIDTH;
      matrix.drawPixel(x, y, led_colors[i]);
    }
  } else {
    matrix.fillScreen(0); // Turn off all LEDs
  }
  matrix.show();
}

void checkChanges() {
  if (last_led_enable != led_enable) {
    updateLED();
    last_led_enable = led_enable;
  }

  bool colorsChanged = false;
  for (int i = 0; i < LED_COUNT; i++) {
    if (last_led_colors[i] != led_colors[i]) {
      colorsChanged = true;
      last_led_colors[i] = led_colors[i];
    }
  }

  if (colorsChanged) {
    updateLED();
  }
}