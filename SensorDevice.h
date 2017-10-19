#ifndef SENSOR_DEVICE_H
#define SENSOR_DEVICE_H

#include <cstdint>
#include <cstddef>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Device.h"
#include "DeviceBus.h"

class Measurement {
 public:
  enum Unit {
    kUnitKelvin = 0,
    kUnitCelsius,
    kUnitFahrenheit,
    kUnitRadian,
    kUnitDegree,
    kUnitMeter,
    kUnitCentimeter,
    kUnitMillimeter,
    kUnitKilogram,
    kUnitGram,
    kUnitSecond,
    kUnitMillisecond,
    kUnitMeterPerSecond,
    kUnitMeterPerSecondPerSecond
  };
  
 private:
  unsigned long last_update_request_timestamp_;
  unsigned long last_update_timestamp_;
  const unsigned long update_timeout_millis_;
  float* value_;
  const size_t value_length_;
  const Unit unit_;
  const char* label_;
  
 public:
  Measurement(size_t value_length, Unit unit, const char* label, unsigned long update_timeout_millis)
    : last_update_request_timestamp_(0),
      last_update_timestamp_(0),
      update_timeout_millis_(update_timeout_millis),
      value_(new float[value_length]()),  // Note: "new T[]()" performs value-initialization (with T as float, all array elements are set to 0)
      value_length_(value_length),
      unit_(unit),
      label_(label)
  {}
  
  ~Measurement() {
    delete[] value_;
  }
  
  // Is there measurement data available, i.e. should we expect anything meaningful from GetValue()?
  bool IsAvailable() {
    return last_update_timestamp_ > 0;
  }
  
  // Is there a pending update for this measurement, i.e. is the parent sensor in the process of gathering the next batch of data?
  bool UpdatePending() {
    return last_update_request_timestamp_ > last_update_timestamp_;
  }
  
  // Have we hit the timeout for a pending update?
  bool UpdateTimedOut() {
    return millis() - last_update_request_timestamp_ > update_timeout_millis_;
  }
  
  void SetValue(const float* value) {
    memcpy(value_, value, value_length_ * sizeof(float));
    last_update_timestamp_ = millis();
  }
  
  void SetValue(const float value) {
    value_[0] = value;
    last_update_timestamp_ = millis();
  }
  
  void RequestUpdate() {
    last_update_request_timestamp_ = millis();
  }
  
  void Reset() {
    last_update_request_timestamp_ = 0;
    last_update_timestamp_ = 0;
    for(size_t v = 0; v < value_length_; v++) {
      value_[v] = 0;
    }
  }
  
  const float* GetValue() {
    return value_;
  }
  
  size_t GetValueLength() {
    return value_length_;
  }
  
  Unit GetUnit() {
    return unit_;
  }
  
  const char* GetUnitString() {
    switch (unit_) {
      case kUnitCelsius: return "degree C";
      case kUnitFahrenheit: return "degree F";
      case kUnitRadian: return "radian";
      case kUnitDegree: return "degree";
      case kUnitMeter: return "m";
      case kUnitCentimeter: return "cm";
      case kUnitMillimeter: return "mm";
      case kUnitKilogram: return "kg";
      case kUnitGram: return "g";
      case kUnitSecond: return "s";
      case kUnitMillisecond: return "ms";
      case kUnitMeterPerSecond: return "m/s";
      case kUnitMeterPerSecondPerSecond: return "m/s^2";
      default: return "unknown";
    }
  }
  
  const char* GetLabel() {
    return label_;
  }
  
  void GetJson(JsonObject& json_root) {
    json_root["label"] = label_;
    json_root["timestamp"] = last_update_timestamp_;
    if (IsAvailable()) {
      JsonArray& value = json_root.createNestedArray("value");
      for(size_t v = 0; v < value_length_; v++) {
        value.add(value_[v]);
      }
    } else {
      json_root["value"] = RawJson("null");
    }
    json_root["unit"] = GetUnitString();
  }
};

class Sensor : public Device {
 protected:
  Measurement** measurements_;
  const size_t num_measurements_;
  
 public:
  Sensor(unsigned long id, size_t num_measurements)
    : Device(id),
      measurements_(new Measurement*[num_measurements]),
      num_measurements_(num_measurements) {
    
    for(unsigned int m = 0; m < num_measurements; m++) {
      measurements_[m] = nullptr;
    }
  }
  
  ~Sensor() { delete[] measurements_; }
  
  Measurement** GetMeasurements() { return measurements_; }
  
  size_t GetNumMeasurements() { return num_measurements_; }
  
  // Refresh() is what drives the sensor -- the idea is that this:
  //   - Requests measurement from a sensor
  //   - Watches timing/status indicators to determine if the requested measurement is ready
  //   - When the measurement is ready, saves it to a local Measurement object
  //   - If appropriate, launches new request
  virtual bool RefreshMeasurements() = 0;
  
  virtual bool Reset() {
    for (size_t m = 0; m < GetNumMeasurements(); m++) {
      measurements_[m]->Reset();
    }
  }
};


class Max31820 : public Sensor {
 // MAX31820 1-Wire Ambient Temperature Sensor -- https://datasheets.maximintegrated.com/en/ds/MAX31820.pdf
 
 private:
  const static unsigned int kTemperatureLatencyMillis = 1000;
  
  const static uint8_t kCommandConvertTemperature = 0x44;
  const static uint8_t kCommandReadScratchpad = 0xbe;
  
  const static uint8_t kScratchpadTemperatureLsb = 0;
  const static uint8_t kScratchpadTemperatureMsb = 1;
  const static uint8_t kScratchpadAlarmHighTemperature = 2;
  const static uint8_t kScratchpadAlarmLowTemperature = 3;
  const static uint8_t kScratchpadConfig = 4;
  const static uint8_t kScratchpadCrc = 8;
  
  Measurement temperature_;
  ByteArray address_;
  OneWireDeviceBus* bus_;
  uint8_t debug_;
  
 public:
  const static uint8_t kOneWireFamilyCode = 0x28;
  
  Max31820(unsigned long id, OneWireDeviceBus* bus, DeviceAddress address, uint8_t debug=0)
    : Sensor(id, 1),
      temperature_(1, Measurement::kUnitCelsius, "temperature", kTemperatureLatencyMillis),
      address_((uint8_t*)address, 8),
      bus_(bus),
      debug_(debug) {
    
    measurements_[0] = &temperature_;
    
    if (debug_ > 0) {
      Serial.print(millis());
      Serial.print(": DEBUG: Max31820 id=");
      Serial.print(id_);
      Serial.print(": setup with address 0x");
      Serial.println(address_.GetString());
    }
  }
  
  void RequestRawTemperatureData() {
    bus_->reset();
    bus_->select(address_.GetBytes());
    bus_->write(kCommandConvertTemperature);
    temperature_.RequestUpdate();
  }
  
  uint16_t ReadRawTemperatureData() {
    bus_->reset();
    bus_->select(address_.GetBytes());
    bus_->write(kCommandReadScratchpad);
    uint16_t temperature_lsb = bus_->read();
    uint16_t temperature_msb = bus_->read();
    return (temperature_msb << 8) | temperature_lsb;
  }
  
  float ConvertRawTemperatureToMeasurement(uint16_t raw_temperature) {
    float temperature = 0.0625 * (float)raw_temperature;
    temperature_.SetValue(temperature);
  }
  
  void ReadAndRefreshTemperature() {
    uint16_t raw_temperature = ReadRawTemperatureData();
    ConvertRawTemperatureToMeasurement(raw_temperature);
    RequestRawTemperatureData();
  }
  
  bool RefreshMeasurements() {
    if (debug_ > 1) {
      Serial.print(millis()); Serial.print(": DEBUG: Max31820 id="); Serial.print(id_);
      Serial.print(": refresh with (IsAvailable, UpdatePending, UpdateTimedOut)=(");
      Serial.print(temperature_.IsAvailable()); Serial.print(", "); Serial.print(temperature_.UpdatePending()); Serial.print(", "); Serial.print(temperature_.UpdateTimedOut());
      Serial.println(")");
    }
    
    if (temperature_.IsAvailable() && temperature_.UpdatePending() && temperature_.UpdateTimedOut()) {
      ReadAndRefreshTemperature();
    } else if (temperature_.IsAvailable() && ! temperature_.UpdatePending()) {
      RequestRawTemperatureData();
    } else if (! temperature_.IsAvailable() && temperature_.UpdatePending() && temperature_.UpdateTimedOut()) {
      ReadAndRefreshTemperature();
    } else if (! temperature_.IsAvailable() && ! temperature_.UpdatePending()) {
      RequestRawTemperatureData();
    } else {
      if (debug_ > 0) { Serial.print(millis()); Serial.println(": ERROR: Max31820 id="); Serial.print(id_); Serial.print(": unknown condition for refresh"); }
    }
    
    return true;
  }
  
  bool Reset() {
    Sensor::Reset();
    bus_->reset();
    return true;
  }
};


class Max31865 : public Sensor {
 // MAX31865 RTD-to-Digital Converter -- https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
 
 private:
  const static unsigned int kTemperatureLatencyMillis = 100;
  
  // To write to a register instead of read from it, bitwise-or this with the register addresses below
  const static uint8_t kRegisterModeWrite = 0x80;
  
  // Register addresses for reads
  const static uint8_t kRegisterConfig = 0x00;
  const static uint8_t kRegisterRtdResistanceMsb = 0x01;
  const static uint8_t kRegisterRtdResistanceLsb = 0x02;
  const static uint8_t kRegisterHighFaultThresholdMsb = 0x03;
  const static uint8_t kRegisterHighFaultThresholdLsb = 0x04;
  const static uint8_t kRegisterLowFaultThresholdMsb = 0x05;
  const static uint8_t kRegisterLowFaultThresholdLsb = 0x06;
  const static uint8_t kRegisterFaultStatus = 0x07;
  
  // Bit definitions within the config byte -- "Alternate" comments indicate what happens when the bit is low
  const static uint8_t kConfigBiasEnable = 0x80;              // Alternate: Bias voltage disabled (must be enabled ahead of 1-shot conversion request)
  const static uint8_t kConfigAutoConversionEnable = 0x40;    // Alternate: Only take measurement on 1-shot conversion request
  const static uint8_t kConfig1ShotConversionRequest = 0x20;  // Alternate: No-op (i.e. don't request a 1-shot conversion) 
  const static uint8_t kConfig3WireRtd = 0x10;                // Alternate: Configure for a 2- or 4-wire RTD
  const static uint8_t kConfig50HzFilter = 0x01;              // Alternate: Use 60 Hz filter
  
  Measurement temperature_;
  const uint16_t adc_reference_resistance_ohms_;
  const uint16_t rtd_zero_celsius_resistance_ohms_;
  SpiDeviceBus* bus_;
  uint8_t debug_;
  
 public:
  Max31865(unsigned long id, SpiDeviceBus* bus, uint16_t adc_reference_resistance_ohms=430, uint16_t rtd_zero_celsius_resistance_ohms=100, uint8_t debug=0)
    : Sensor(id, 1),
      temperature_(1, Measurement::kUnitCelsius, "temperature", kTemperatureLatencyMillis),
      adc_reference_resistance_ohms_(adc_reference_resistance_ohms),
      rtd_zero_celsius_resistance_ohms_(rtd_zero_celsius_resistance_ohms),
      bus_(bus),
      debug_(debug) {
    
    measurements_[0] = &temperature_;
    
    Reset();
    
    if (debug_ > 0) {
      uint8_t config = GetConfiguration();
      Serial.print(millis());
      Serial.print(": DEBUG: Max31865 id=");
      Serial.print(id_);
      Serial.print(": setup with configuration 0x");
      Serial.println(ByteArray(config).GetString());
    }
  }
  
  uint8_t GetConfiguration() {
    bus_->Reset();
    bus_->Write8(kRegisterConfig);
    uint8_t config = bus_->Read8();
    bus_->EndTransaction();
    return config;
  }
  
  void RequestRawTemperatureData() {
    uint8_t config = GetConfiguration();
    bus_->Reset();
    bus_->Write8(kRegisterConfig | kRegisterModeWrite);
    bus_->Write8(config | kConfig1ShotConversionRequest);
    bus_->EndTransaction();
    temperature_.RequestUpdate();
  }
  
  uint16_t ReadRawTemperatureData() {
    // Raw data is RTD-to-reference resistance ratio (but lsb is a fault status bit)
    bus_->Reset();
    bus_->Write8(kRegisterRtdResistanceMsb);
    uint16_t resistance_ratio_msb = bus_->Read8();
    uint16_t resistance_ratio_lsb = bus_->Read8();
    bus_->EndTransaction();
    return (resistance_ratio_msb << 8) | resistance_ratio_lsb;
  }
  
  float ConvertRawTemperatureToMeasurement(uint16_t raw_temperature) {
    // Drop the fault status bit (lsb)
    uint16_t rtd_resistance_ratio = (raw_temperature >> 1);
    
    // Solve the Callendar-Van Dusen (CVD) equation for temperature given the resistances -- see datasheet or
    // https://en.wikipedia.org/wiki/Callendar%E2%80%93Van_Dusen_equation for details.  Note that we only solve
    // the quadratic version of the equation (i.e. we assume the "c" coefficient of CVD is 0), so these results
    // are valid only for temperatures above 0 celsius.
    float resistance_term = 1 - (rtd_resistance_ratio * adc_reference_resistance_ohms_) / (32768.0 * rtd_zero_celsius_resistance_ohms_);
    float a = 3.90830e-3;
    float b = -5.77500e-7;
    float temperature = (-a + sqrt(a*a - 4*b*resistance_term)) / (2*b);
    temperature_.SetValue(temperature);
  }
  
  void ReadAndRefreshTemperature() {
    uint16_t raw_temperature = ReadRawTemperatureData();
    boolean fault = raw_temperature & 0x01;
    if (debug_ > 0 && fault) { Serial.print(millis()); Serial.print(": ERROR: Max31865 id="); Serial.print(id_); Serial.print(": fault status "); Serial.println(fault); }
    ConvertRawTemperatureToMeasurement(raw_temperature);
    RequestRawTemperatureData();
  }
  
  bool RefreshMeasurements() {
    if (debug_ > 1) {
      Serial.print(millis()); Serial.print(": DEBUG: Max31865 id="); Serial.print(id_);
      Serial.print(": refresh with (IsAvailable, UpdatePending, UpdateTimedOut)=(");
      Serial.print(temperature_.IsAvailable()); Serial.print(", "); Serial.print(temperature_.UpdatePending()); Serial.print(", "); Serial.print(temperature_.UpdateTimedOut());
      Serial.println(")");
    }
    
    if (temperature_.IsAvailable() && temperature_.UpdatePending() && temperature_.UpdateTimedOut()) {
      ReadAndRefreshTemperature();
    } else if (temperature_.IsAvailable() && ! temperature_.UpdatePending()) {
      RequestRawTemperatureData();
    } else if (! temperature_.IsAvailable() && temperature_.UpdatePending() && temperature_.UpdateTimedOut()) {
      ReadAndRefreshTemperature();
    } else if (! temperature_.IsAvailable() && ! temperature_.UpdatePending()) {
      RequestRawTemperatureData();      
    } else {
      if (debug_ > 0) { Serial.print(millis()); Serial.print(": ERROR: Max31865 id="); Serial.print(id_); Serial.print(": unknown condition for refresh"); }
    }
    
    return true;
  }
  
  bool Reset() {
    Sensor::Reset();
    bus_->Reset();
    bus_->Write8(kRegisterConfig | kRegisterModeWrite);
    bus_->Write8(kConfigBiasEnable | kConfig3WireRtd);
    bus_->EndTransaction();
    return true;
  }
};

#endif
