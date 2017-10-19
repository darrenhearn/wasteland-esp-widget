#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

typedef uint8_t Pinout;

namespace pinout {
  // 1-Wire master
  const Pinout kOneWire = 2;

  // SPI master -- see http://esp8266.github.io/Arduino/versions/2.3.0/doc/reference.html for hardware-based SPI pin assignments
  const Pinout kSpiSclk = 14;
  const Pinout kSpiMiso = 12;
  const Pinout kSpiMosi = 13;
  const Pinout kSpiSs = 4;
}

#endif
