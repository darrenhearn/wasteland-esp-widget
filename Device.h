#ifndef DEVICE_H
#define DEVICE_H

#include <cstdint>
#include <cstddef>
#include <ArduinoJson.h>

class Device {
 protected:
  const unsigned long id_;
 
 public:
  enum ErrorCode {
    kErrorCodeSuccess = 0,
    kErrorCodeCommunicationFailure = 1,
    kErrorCodeIncorrectDeviceForAddress = 2,
    kErrorCodeCrcFailed = 3
  };
  
  Device(unsigned long id) : id_(id) {}
  unsigned long GetId() { return id_; }
};

class ByteArray {
 private:
  const uint8_t* bytes_;
  const size_t length_;
  char* string_;
  
  void ConvertBytesToString() {
    for (uint8_t elt = 0; elt < length_; elt++) {
      snprintf(&string_[2*elt], 3, "%02x", bytes_[elt]);
    }    
  }
  
 public:
  ByteArray(uint8_t byte) : ByteArray(&byte, 1) {}
  
  ByteArray(const uint8_t* bytes, size_t length)
    : bytes_(new uint8_t[length]),
      length_(length),
      string_(new char[2*length + 1]) {
    
    memcpy((uint8_t*) bytes_, bytes, length_);
    ConvertBytesToString();
  }
  
  ~ByteArray() {
    delete[] bytes_;
    delete[] string_;
  }
  
  const uint8_t* const GetBytes() { return bytes_; }
  
  uint8_t GetByte(size_t position) {
    if (position >= length_) return 0;
    return bytes_[position];
  }
  
  const size_t GetLength() { return length_; }
  
  const char* const GetString() { return string_; }
};

#endif
