#include <PicoMQTT.h>
#include <WiFi.h>  // Make sure to include the WiFi library

#if __has_include("config.h")
#include "config.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "simplifiedintegration"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "acadia2024"
#endif

// Set your desired static IP, gateway, and subnet
IPAddress local_IP(192, 168, 0, 200);  // Static IP
IPAddress gateway(192, 168, 0, 1);    // Gateway (usually your router IP)
IPAddress subnet(255, 255, 255, 0);   // Subnet mask

PicoMQTT::Server mqtt(1883);  // Set the MQTT port to 1883

void setup() {
    // Setup serial
    Serial.begin(115200);

    // Configure static IP
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("Failed to configure Static IP");
    }

    // Connect to WiFi
Serial.printf("WIFI_SSIDConnecting to WiFi %s\n");  
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting...");
    }
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

    // Start MQTT server on port 1883
    mqtt.begin();
}

void loop() {
    mqtt.loop();
}