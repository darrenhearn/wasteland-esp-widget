#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "MqttUtil.h"
#include "HardwarePins.h"
#include "Device.h"
#include "DeviceBus.h"
#include "SensorDevice.h"

// todo: support list<Device>
// todo: refactor Sensor class's measurements_ from Measurement** to list<Measurement*> (likely better as array<Measurement*> or similar)
// todo: build topics for the JSON messages -- maybe /customer_id/marketing_name/esp8266_id/device_model_or_classname/device_label/device_id/device_serial_num/ (device_label doesn't exist yet)
// todo: overload ByteArray to support indexing (i.e. for ByteArray b, make b[0] equivalent to b.GetBytes()[0])
// todo: support crc checks for OneWireTemperatureSensor
// todo: generic OneWireTemperatureSensor class, covering MAX31820, DS18*20, MAX31850 (thermocouple) device families
// todo: factory method to auto-discover and return list<Device*> for OneWireTemperatureSensor
// todo: configurable resolution for OneWireTemperatureSensor
// todo: solve the full (4th order) CVD equation or find an approximation to cover temperatures below freezing for MAX31865
// todo: look at std::ostream instead of Printable
// todo: break Serial dependency for debugging (maybe create Printable objects?)
// todo: switch from debugging to generic logging (to include DEBUG, TRACE and other levels)
// todo: independent class to support debugging (e.g. Debug class with debug levels and helper methods)
// todo: const-correctness (decompose this into areas and prioritize)
// todo: auto-increment/assign Device ids
// todo: use flash space for strings (e.g. Measurement unit names)
// todo: fault test cycle support for MAX31865
// todo: strategy for error codes on Device communication (set/clear an error state in the object, exceptions, other?)
//
// done: 2017-10-17: avoid recording initial measurement until measurement/conversion complete -- using last_update_request_timestamp_ in Measurement (*not* in Sensor)
// done: 2017-10-16: decompose Refresh() methods into more discrete operations
// done: 2017-10: basic support for MAX31865 (Pt100 probe)
// done: 2017-10: basic SPI interface with debugging
// done: 2017-09: basic support for MAX31820
// done: 2107-09: basic 1-Wire interface with debugging
// done: 2017-08: JSON library integration

struct WiFiCredentials {
  char* ssid;
  char* password;
};

// REMINDER: MQTT is effectively disabled for now -- search for DISABLED in MqttUtil.cpp

const WiFiCredentials kWiFi1 = {"some_ssid", "some_psk"};
const WiFiCredentials kWiFi2 = {"some_other_ssid", "some_other_psk");
// and so on ...
const WiFiCredentials kDefaultWifi = WiFi1;
const char* kMqttServer = "mqtt-hostname";
const uint16_t kMqttPort = 1883;

OneWireDeviceBus one_wire_controller(pinout::kOneWire, 1);
SpiDeviceBus spi_controller(pinout::kSpiSs, 1);

WiFiClient network_client;
//PubSubClient mqtt_client(kMqttServer, kMqttPort, network_client);
//MqttUtil mqtt_util(&mqtt_client);

Max31820* ambient_temperature_sensor;
Max31865* pt_temperature_sensor;

void InitWiFi(const WiFiCredentials& credentials) {
  WiFi.begin(credentials.ssid, credentials.password);
  
  Serial.print("Connecting to wireless");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
}

void InitMeasurement(void) {
  one_wire_controller.reset_search();
  one_wire_controller.target_search(Max31820::kOneWireFamilyCode);
  
  DeviceAddress address;
  if (!one_wire_controller.search(address)) {
    Serial.println("no devices found");
  } else {
    ambient_temperature_sensor = new Max31820(1, &one_wire_controller, address, 2);
  }
  
  pt_temperature_sensor = new Max31865(2, &spi_controller, 430, 100, 2);
}

void setup() {
  Serial.begin(115200);
  //InitWiFi(kDefaultWiFi);
  InitMeasurement();
}

void RefreshAndSendMeasurements(Sensor* s) {
  s->RefreshMeasurements();
  
  Measurement** measurements = s->GetMeasurements();
  for(size_t m = 0; m < s->GetNumMeasurements(); m++) {
    StaticJsonBuffer<250> json_buffer;
    JsonObject& json_root = json_buffer.createObject();
    measurements[m]->GetJson(json_root);
    json_root.printTo(Serial);
    Serial.println("");
  }
}

void loop() {
  RefreshAndSendMeasurements(ambient_temperature_sensor);
  RefreshAndSendMeasurements(pt_temperature_sensor);
  delay(1000);
}

