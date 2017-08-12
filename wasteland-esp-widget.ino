#include <Wire.h>
#include <twi.h>

typedef uint8_t Pinout;

namespace pinout {
  const Pinout kOneWire = 2;  // 1-Wire bus
  
  // The SPI and I2C libraries already assume these pinouts when using a hardware-based implementation, but including
  // them here as a convenient reference -- originally from http://esp8266.github.io/Arduino/versions/2.3.0/doc/reference.html
  // UPDATE: Apparently ESP8266 has no hardware support for I2C -- see http://bbs.espressif.com/viewtopic.php?t=1032
  const Pinout kSpiSclk = 14;
  const Pinout kSpiMiso = 12;
  const Pinout kSpiMosi = 13;
  const Pinout kSpiSs = 15;
  const Pinout kI2cScl = 5;
  const Pinout kI2cSda = 4;
}

namespace onewire {
  const uint8_t kReadRom = 0x33;
  const uint8_t kMatchRom = 0x55;
  const uint8_t kSearchRom = 0xF0;
}

class Ds2482 {
  public:
  enum Commands {
    kCmdReset = 0xF0,
    kCmdWriteConfig = 0xD2,
    kCmdChannelSelect = 0xC3,  // Only available for the DS2482-800
    kCmdSetReadPointer = 0xE1,
    kCmd1WireReset = 0xB4,
    kCmd1WireWriteByte = 0xA5,
    kCmd1WireReadByte = 0x96,
    kCmd1WireSingleBit = 0x87,
    kCmd1WireTriplet = 0x78
  };
  
  enum Config {
    kConfigActivePullup = 0x01,
    kConfigPresencePulseMasking = 0x02,
    kConfigStrongPullup = 0x04,
    kConfig1WireSpeed = 0x08
  };
  
  enum Status {
    kStatus1WireBusy = 0x01,
    kStatusPresencePulseDetect = 0x02,
    kStatusShortDetect = 0x04,
    kStatusReset = 0x10
  };
  
  Ds2482(TwoWire* i2c_interface, uint8_t i2c_address = 0x18) :
    i2c_interface_(i2c_interface),
    i2c_address_(i2c_address)
  {};
  
  uint8_t I2cRead8(bool send_stop = true) {
    uint8_t bytes_read = i2c_interface_->requestFrom(i2c_address_, (size_t)1, send_stop);
    if (bytes_read > 0) return i2c_interface_->read();
    else return 0;
  }
  
  // Reset DS2482 state-machine and terminate any 1-Wire communication.  This also resets the configuration
  // (all options disabled) -- see SetConfig function to reconfigure.
  // Returns true for a successful reset.
  bool Reset() {
    i2c_interface_->beginTransmission(i2c_address_);
    i2c_interface_->write(kCmdReset);
    uint8_t i2c_status = i2c_interface_->endTransmission(0);
    
    if (i2c_status != 0) return false;
    
    // DS2482 application notes (http://pdfserv.maximintegrated.com/en/an/AN3684.pdf) use this logic
    // to confirm a successful reset.
    //
    // TODO: Understand the logic here.  I think the 0x10 comparison comes from the fact that kStatusReset = 0x10,
    // but I don't understand the mask.  Also, I gather it's coincidence that the raw value returned by the read (0x18)
    // matches the I2C address I'm using for my DS2482?
    return (I2cRead8() & 0xF7) == 0x10;
  }
  
  // Set DS2482 configuration for optional features.  Pass the desired config as a single byte by combining
  // flags with bitwise-or -- see the Config enum for a list of all supported flags.
  // Example: SetConfig(kConfigActivePullup | kConfig1WireSpeed) // Enables active pull-up and high speed on the 1-Wire bus
  // Returns true for a succesful config update (i.e. reading-back the config matches the requested config).
  bool SetConfig(uint8_t config) {
    i2c_interface_->beginTransmission(i2c_address_);
    i2c_interface_->write(kCmdWriteConfig);
    // DS2482 expects its four config flags in the lower nibble and the inverse of the config flags in the upper nibble
    i2c_interface_->write(config | (~config << 4));
    uint8_t i2c_status = i2c_interface_->endTransmission(0);
    
    if (i2c_status != 0) return false;
    
    return (I2cRead8() == config);
  }
  
  bool OneWireReset() {
    const uint8_t kPollLimit = 200;
    uint8_t poll_count = 0;
    
    i2c_interface_->beginTransmission(i2c_address_);
    i2c_interface_->write(kCmd1WireReset);
    uint8_t i2c_status = i2c_interface_->endTransmission(0);
    
    if (i2c_status != 0) return false;
    
    // TODO: Check if this logic is broken if the DS2482 immediately gets a response back on
    // the 1-Wire bus -- seems like we might be issuing an extra I2cRead8() in that case
    uint8_t reset_status = I2cRead8();
    do {
      reset_status = I2cRead8(reset_status & kStatus1WireBusy);
    } while ((reset_status & kStatus1WireBusy) && (poll_count++ < kPollLimit));

    // TODO: Understand if there's a way to do this without hacking core_esp8266_si2c.  At first glance,
    // the Wire library demands that you know whether or not you want to send an I2C stop at each
    // transaction -- but here we don't know if we can send the stop or not until we see what we got.
    // For now, I've removed the "static" modifier on twi_write_stop() to expose its functionality,
    // but I need to clean that up.
    twi_write_stop();
    
    if (poll_count >= kPollLimit) return false;
    if (reset_status & kStatusShortDetect) return false;
    if (reset_status & kStatusPresencePulseDetect) return true;
    Serial.println("No 1-Wire devices detected -- but successfully reset 1-Wire bus");
    return false;
  }
  
  private:
  TwoWire* i2c_interface_;
  uint8_t i2c_address_;
};

Ds2482 ds2482(&Wire, 0x18);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.print("Reset return = ");  Serial.println(ds2482.Reset());
}

void loop() {
  Serial.print("Reset return = ");  Serial.println(ds2482.Reset());  delay(4000);
  Serial.print("Config return = ");  Serial.println(ds2482.SetConfig(0));  delay(4000);
  Serial.print("1-Wire reset return = ");  Serial.println(ds2482.OneWireReset());  delay(4000);
}

