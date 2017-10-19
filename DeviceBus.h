#ifndef DEVICE_BUS_H
#define DEVICE_BUS_H

#include <cstdint>
#include <cstddef>
#include "Device.h"
#include "HardwarePins.h"

#include <OneWire.h>
#include <SPI.h>

// TODO
// - Create InterfaceBus base class
// - Virtual methods for AttachDevice(GenericAddress addr), EnumerateDevices()
// - Create derived classes for SPI, I2C and 1Wire

// Probably change approach a bit -- no compelling case to have DeviceBus maintain a list of attached
// devices, especially since some devices may be for reading, others for writing; different subclasses
// of device that may support different functionality; etc.
//
// Instead, have the registration function simply attach each Device to its DeviceBus (i.e. device maintains
// pointer to associated bus).  Then the user can assemble Devices into appropriate collections by function
// (e.g. sensors for reading from on each loop, SD card for writing data on each loop), taking advantage
// of a generic Device container (list<>, vector<>, etc.) that still exposes Device subclass functionality.

class OneWireDeviceBus : public OneWire {
  private:
    uint8_t debug_;

  public:
    OneWireDeviceBus(Pinout one_wire_pin, bool debug = 0)
      : OneWire(one_wire_pin),
        debug_(debug) {}

    uint8_t read() {
      uint8_t byte_read = OneWire::read();
      if (debug_) {
        Serial.print(millis());
        Serial.print(": DEBUG: OneWireDeviceBus: read 0x");
        Serial.println(ByteArray(byte_read).GetString());
      }
      return byte_read;
    }

    void select(const uint8_t rom[8]) {
      if (debug_) {
        Serial.print(millis());
        Serial.print(": DEBUG: OneWireDeviceBus: selecting device address 0x");
        Serial.println(ByteArray(rom, 8).GetString());
      }
      OneWire::select(rom);
    }

    void write(uint8_t v, uint8_t power = 0) {
      if (debug_) {
        Serial.print(millis());
        Serial.print(": DEBUG: OneWireDeviceBus: writing 0x");
        Serial.println(ByteArray(v).GetString());
      }
      OneWire::write(v, power);
    }
};


class SpiDeviceBus {
 private:
  SPISettings config_;
  const Pinout spi_ss_pin_;
  boolean transaction_in_progress_;
  uint8_t debug_;
  
 public:
  SpiDeviceBus(Pinout spi_ss_pin, bool debug=0)
    : config_(500000, MSBFIRST, SPI_MODE1),
      spi_ss_pin_(spi_ss_pin),
      transaction_in_progress_(false),
      debug_(debug) {
    
    pinMode(spi_ss_pin_, OUTPUT);
    digitalWrite(spi_ss_pin_, 1);
    SPI.begin();
  }
  
  void Write8(uint8_t byte_write) {
    if (! transaction_in_progress_) {
      SPI.beginTransaction(config_);
      digitalWrite(spi_ss_pin_, 0);
      transaction_in_progress_ = true;
    }
    if (debug_) {
      Serial.print(millis());
      Serial.print(": DEBUG: SpiDeviceBus: writing 0x");
      Serial.println(ByteArray(byte_write).GetString());
    }
    SPI.write(byte_write);
  }
  
  uint8_t Read8() {
    if (! transaction_in_progress_) {
      SPI.beginTransaction(config_);
      digitalWrite(spi_ss_pin_, 0);
      transaction_in_progress_ = true;
    }
    uint8_t byte_read = SPI.transfer(0);
    if (debug_) {
      Serial.print(millis());
      Serial.print(": DEBUG: SpiDeviceBus: read 0x");
      Serial.println(ByteArray(byte_read).GetString());
    }
    return byte_read;
  }

  void Reset() {
    EndTransaction();
  }
  
  void EndTransaction() {
    digitalWrite(spi_ss_pin_, 1);
    SPI.endTransaction();
    transaction_in_progress_ = false;
  }
};

#endif
