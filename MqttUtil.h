#ifndef MQTTUTIL_H
#define MQTTUTIL_H

#include "PubSubClient.h"

class MqttUtil {
 public:
  MqttUtil(PubSubClient* mqtt_client, uint8_t max_retries = 10, uint16_t retry_delay = 1000, size_t message_buffer_size = 200)
    : mqtt_client_(mqtt_client),
      max_retries_(max_retries),
      retry_delay_(retry_delay),
      message_buffer_size_(message_buffer_size),
      message_buffer_(new char[message_buffer_size]) {}
  
  boolean Connect();
  boolean Refresh();
  boolean PublishTemperature(unsigned long timestamp, const char* const device, float temperature);
  
 private:
  PubSubClient* mqtt_client_;
  uint8_t max_retries_;
  uint16_t retry_delay_;
  
  static const size_t kTopicBufferSize = 256;
  char topic_buffer_[kTopicBufferSize];
  
  size_t message_buffer_size_;
  char* message_buffer_;
  
  static const size_t kValueBufferSize = 33;
  char value_buffer_[kValueBufferSize];
};

#endif
