#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "MqttUtil.h"

typedef uint16_t AnalogValue;
typedef uint8_t Pinout;

namespace pinout {
  const Pinout kOneWire = 2;  // OneWire bus
  const Pinout kAnalog = A0;   // Analog input
  const Pinout kAnalogSelect0 = 12;  // 1:4 analog multiplexer select (LSB)
  const Pinout kAnalogSelect1 = 13;  // 1:4 analog multiplexer select (MSB)
}

namespace analog_inputs {
  const uint8_t kAmbientTemperature = 0;
  const uint8_t kExternalTemperature = 1;
  const uint8_t kDrain = 3;
}

// REMINDER: MQTT is effectively disabled for now -- search for DISABLED in MqttUtil.cpp

struct WiFiCredentials {
  char* ssid;
  char* password;
};

const WiFiCredentials kWiFi1 = {"some_ssid", "some_psk"};
const WiFiCredentials kWiFi2 = {"some_other_ssid", "some_other_psk");
// and so on ...
const WiFiCredentials* kDefaultWiFi = &kWiFi1;
const char* kMqttServer = "mqtt-hostname";
const uint16_t kMqttPort = 1883;

OneWire one_wire_controller(pinout::kOneWire);
DallasTemperature thermocouples(&one_wire_controller);
WiFiClient network_client;
PubSubClient mqtt_client(kMqttServer, kMqttPort, network_client);
MqttUtil mqtt_util(&mqtt_client);

uint8_t state_id;
unsigned long last_state_change_timestamp;

void setup() {
  Serial.begin(115200);
  pinMode(pinout::kAnalogSelect0, OUTPUT);
  pinMode(pinout::kAnalogSelect1, OUTPUT);
  InitWiFi(kDefaultWiFi);
  //thermocouples.setWaitForConversion(false);
  thermocouples.begin();
  InitMeasurement();
  state_id = 3;
  last_state_change_timestamp = millis();
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

void AnalogSelect(uint8_t source) {
  digitalWrite(pinout::kAnalogSelect0, source & B01);
  digitalWrite(pinout::kAnalogSelect1, source & B10);
}

void InitMeasurement(void) {
  thermocouples.requestTemperatures();
  AnalogSelect(analog_inputs::kDrain);
}

void AnalogConnect(uint8_t analog_input) {
  analog_input == analog_inputs::kAmbientTemperature ? state_id = 0 : state_id = 2;
  AnalogSelect(analog_input);
  last_state_change_timestamp = millis();
}

float ConvertAnalogValueToVolts(AnalogValue val) {
  // ESP8266 ADC has 10-bit resolution, returning 1023 for the 1.0V maximum input
  return val / 1023.0;
}

float ConvertAnalogValueToCelsius_TMP36(AnalogValue val) {
  // TMP36 temperature sensor outputs 750mV at 25C and changes by +/-10mV per +/-1C
  float volts = ConvertAnalogValueToVolts(val);
  return 100*(volts - 0.750) + 25;
}

void ReadAndSendMeasurements(uint8_t analog_input) {
  analog_input == analog_inputs::kAmbientTemperature ? state_id = 1 : state_id = 3;
  
  unsigned long timestamp = millis();
  mqtt_util.Refresh();
  
  uint8_t num_thermocouples = thermocouples.getDeviceCount();
  for(uint8_t t = 0; t < num_thermocouples; t++) {
    DeviceAddress t_addr;
    if(thermocouples.getAddress(t_addr, t)) {
      mqtt_util.PublishTemperature(timestamp, t, thermocouples.getTempC(t_addr));
    } else {
      mqtt_util.PublishTemperature(timestamp, t, 0.0);
    }
  }
  
  float analog_temp = ConvertAnalogValueToCelsius_TMP36(analogRead(pinout::kAnalog));
  mqtt_util.PublishTemperature(timestamp, num_thermocouples + analog_input, analog_temp);

  InitMeasurement();
  
  last_state_change_timestamp = millis();
}


void loop() {
  unsigned long timestamp = millis();
  unsigned long delta = timestamp - last_state_change_timestamp;
  
  switch(state_id) {
    case 3:
      if(delta >= 250) AnalogConnect(analog_inputs::kAmbientTemperature);
      break;
    case 0:
      if(delta >= 1000) ReadAndSendMeasurements(analog_inputs::kAmbientTemperature);
      break;
    case 1:
      if(delta >= 250) AnalogConnect(analog_inputs::kExternalTemperature);
      break;
    case 2:
      if(delta >= 1000) ReadAndSendMeasurements(analog_inputs::kExternalTemperature);
      break;
  }
}

