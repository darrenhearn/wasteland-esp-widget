# wasteland-esp-widget
Playing with ESP8266 as Arduino device

Setting up this branch for tinkering with:
- 1-Wire thermocouple amp (MAX31850K) on pin 2
- Analog mux (74HC4051) feeding to A0 with address selection on pins 12 and 13
- Analog noise reduction (maybe) by draining A0 to ground through a resistor (available on highest address of mux) between measurements
- WiFi and MQTT (insecure)
- C++
