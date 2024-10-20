#include <Arduino.h>
#include "esp_system.h"   // Library for system information
#include "esp_bt.h"       // Library to handle Bluetooth

// Structure to store all relevant device information
struct DeviceInfo {
  String chipVersion;      // ESP32 chip version
  String macBluetooth;     // Bluetooth MAC address
  String chipID;           // Unique chip ID
  int cpuFreqMHz;          // CPU frequency in MHz
  int flashSize;           // Flash memory size in bytes
  size_t freeRAM;          // Amount of free RAM in bytes
  long uptime;             // Time since the device was powered on (in milliseconds)
};

DeviceInfo info;

void sendDeviceInfo();
void listenForCommands();

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Retrieve ESP32 chip version
  info.chipVersion = String(esp_get_idf_version());

  // Get Bluetooth MAC address
  uint8_t btMac[6]; 
  esp_read_mac(btMac, ESP_MAC_BT);
  info.macBluetooth = String(btMac[0], HEX) + ":" + String(btMac[1], HEX) + ":" +
                      String(btMac[2], HEX) + ":" + String(btMac[3], HEX) + ":" +
                      String(btMac[4], HEX) + ":" + String(btMac[5], HEX);

  // Get WiFi MAC address (even without connection)
  uint8_t wifiMac[6];
  esp_read_mac(wifiMac, ESP_MAC_WIFI_STA);  // Read the WiFi MAC address

  // Get unique chip ID
  uint64_t chipid = ESP.getEfuseMac();  // Unique chip ID based on MAC
  info.chipID = String((uint16_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);

  // Get CPU frequency
  info.cpuFreqMHz = getCpuFrequencyMhz();

  // Get Flash memory size
  info.flashSize = ESP.getFlashChipSize();

  // Get the amount of free RAM
  info.freeRAM = esp_get_free_heap_size();

  // Get the uptime (time since power-on)
  info.uptime = millis();

  sendDeviceInfo();
}

void loop() {
  listenForCommands(); // Listen for commands from the Serial Monitor
}

// Function to send device information in a CSV format
void sendDeviceInfo() {
  Serial.print("DEVICE_INFO,"); // Identifier
  Serial.print("chipVersion:"); 
  Serial.print(info.chipVersion);
  Serial.print(",macBluetooth:"); 
  Serial.print(info.macBluetooth);
  Serial.print(",chipID:"); 
  Serial.print(info.chipID);
  Serial.print(",cpuFreqMHz:"); 
  Serial.print(info.cpuFreqMHz);
  Serial.print(",flashSizeKB:"); 
  Serial.print(info.flashSize / 1024);  // Convert to KB
  Serial.print(",freeRAMKB:"); 
  Serial.print(info.freeRAM / 1024);  // Convert to KB
  Serial.print(",uptimeSeconds:"); 
  Serial.println(info.uptime / 1000);  // Convert to seconds
}

// Function to listen for commands from the Serial Monitor
void listenForCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace

    // If the command is "get_info", send the device info
    if (command.equalsIgnoreCase("get_info()")) {
      sendDeviceInfo();
    }
  }
}
