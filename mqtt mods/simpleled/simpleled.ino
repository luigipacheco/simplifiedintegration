#include <Arduino.h> // Essential for ESP32 functions
#include <PicoMQTT.h>
#include <Adafruit_NeoPixel.h>

// WIFI Setup
WiFiClient tcp;
IPAddress static_IP(192, 168, 0, 201);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);

bool wifiConnected = false;
const char* ssid = "simplifiedintegration";
const char* password = "acadia2024";

// MQTT setup
PicoMQTT::Client mqtt("192.168.0.200");  // Broker IP address

// LED setup
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Variables for state
bool led_enable = false;
uint32_t led_color = 0; // Variable to store the color as a single value

// Variables to track the last state
bool last_led_enable = false;
uint32_t last_led_color = 0;

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
  mqtt.subscribe("led/enable", onMqttMessage);
  mqtt.subscribe("led/color", onMqttMessage);
  mqtt.begin();
  Serial.println("mqtt topics");
  Serial.println("bool led/enable");
  Serial.println("r,g,b led/color");

}

void setupLed() {
  #if defined(NEOPIXEL_POWER)
  // If this board has a power control pin, set it to output and high to enable the NeoPixels
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif
  pixels.begin(); // Initialize NeoPixel strip
  pixels.setBrightness(255); // Set brightness level
}

void onMqttMessage(const char* topic, PicoMQTT::IncomingPacket& packet) {
  String addr = String(topic);
  String valueStr = "";
  while (packet.available()) {
    valueStr += (char)packet.read(); // Append each character to the string
  }

  // Handle the topic and update the corresponding variable
  if (addr == "led/enable") {
    led_enable = (valueStr == "1");
    Serial.print("Enable: ");
    Serial.println(led_enable);
  } else if (addr == "led/color") {
    // Parse RGB values from the received string
    int comma1 = valueStr.indexOf(',');
    int comma2 = valueStr.indexOf(',', comma1 + 1);

    if (comma1 != -1 && comma2 != -1) {
      int r = valueStr.substring(0, comma1).toInt();
      int g = valueStr.substring(comma1 + 1, comma2).toInt();
      int b = valueStr.substring(comma2 + 1).toInt();

      // Combine RGB values into a single 32-bit color
      led_color = pixels.Color(r, g, b);

      Serial.print("Color: R=");
      Serial.print(r);
      Serial.print(", G=");
      Serial.print(g);
      Serial.print(", B=");
      Serial.println(b);
    }
  }
}

void updateLED() {
  if (led_enable) {
    pixels.fill(led_color); // Set the LED color
  } else {
    pixels.fill(0); // Turn off the LED
  }
  pixels.show();
}

void checkChanges() {
  // Check if there are changes in the LED variables
  if (last_led_color != led_color || last_led_enable != led_enable) {
    updateLED();
    // Update the last state variables
    last_led_color = led_color;
    last_led_enable = led_enable;
  }
}