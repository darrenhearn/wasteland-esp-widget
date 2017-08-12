//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "MqttUtil.h"

typedef uint8_t Pinout;

namespace pinout {
  const Pinout kOneWire = 2;  // 1-Wire bus

  // The SPI and I2C libraries already assume these pinouts when using a hardware-based implementation.
  // Including them here as a convenient reference -- from http://esp8266.github.io/Arduino/versions/2.3.0/doc/reference.html
  const Pinout SPI_SCLK = 14;
  const Pinout SPI_MISO = 12;
  const Pinout SPI_MOSI = 13;
  const Pinout SPI_SS = 15;
  const Pinout I2C_SCL = 5;
  const Pinout I2C_SDA = 4;
}

// REMINDER: MQTT is disabled for now -- see DISABLED in MqttUtil.cpp

struct WiFiCredentials {
  char* ssid;
  char* password;
};

const WiFiCredentials kHearnetWiFi = {"hearnet", "twasbrilligandtheslithytoves"};
const WiFiCredentials kEieioWiFi = {"eieio", "sustainablepat"};
const WiFiCredentials kWakefieldWiFi = {"QL8XJ", "MVFVJFHFVRF7TSJD"};
const WiFiCredentials* kDefaultWiFi = &kHearnetWiFi;
const char* kMqttServer = "mqtt-farm.hearnnet.org";
const uint16_t kMqttPort = 1883;

//OneWire one_wire_controller(pinout::kOneWire);
//DallasTemperature one_wire_thermal(&one_wire_controller);
WiFiClient network_client;
PubSubClient mqtt_client(kMqttServer, kMqttPort, network_client);
MqttUtil mqtt_util(&mqtt_client);

void setup() {
  Serial.begin(115200);
  InitWiFi(kDefaultWiFi);
  //one_wire_thermal.setWaitForConversion(false);
  //one_wire_thermal.begin();
  Wire.begin();
  InitMeasurement();
}

void InitWiFi(const WiFiCredentials* const credentials) {
  WiFi.begin(credentials->ssid, credentials->password);
  
  Serial.print("Connecting to wireless");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
}

void InitMeasurement() {
  //one_wire_thermal.requestTemperatures();
}

namespace ds2482 {
  const uint8_t kAddress = 0x18;
  const uint8_t kCmdReset = 0xF0;
  const uint8_t kCmdWriteConfig = 0xD2;
  const uint8_t kCmdChannelSelect = 0xC3;  // Only available for the DS2482-800
  const uint8_t kCmdSetReadPointer = 0xE1;
  const uint8_t kCmd1WireReset = 0xB4;
  const uint8_t kCmd1WireWriteByte = 0xA5;
  const uint8_t kCmd1WireReadByte = 0x96;
  const uint8_t kCmd1WireSingleBit = 0x87;
  const uint8_t kCmd1WireTriplet = 0x78;
}

void ReadAndSendMeasurements() {
  mqtt_util.Refresh();
  unsigned long timestamp = millis();

  /*
  uint8_t num_thermal = one_wire_thermal.getDeviceCount();
  for(uint8_t t = 0; t < num_thermal; t++) {
    DeviceAddress t_addr;
    if(one_wire_thermal.getAddress(t_addr, t)) {
      mqtt_util.PublishTemperature(timestamp, t, one_wire_thermal.getTempC(t_addr));
    } else {
      mqtt_util.PublishTemperature(timestamp, t, 0.0);
    }
  }
  */

  Wire.beginTransmission(ds2482::kAddress);
  Wire.write(
  InitMeasurement();
}

void loop() {
  ReadAndSendMeasurements();
  delay(500);
}

