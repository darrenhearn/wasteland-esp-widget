#include "MqttUtil.h"

boolean MqttUtil::Connect() {
  uint8_t retries = 0;
  Serial.print("Connecting to MQTT broker");
  while(! mqtt_client_->connected() && retries <= max_retries_) {
    if(! mqtt_client_->connect("stickyWidgetSessionId")) {
      retries++;
      delay(retry_delay_);
      Serial.print(".");
    }
  }
  Serial.println("done");
  return mqtt_client_->connected();
}

boolean MqttUtil::Refresh() {
  // DISABLED by virtue of this immediate return
  return true;
  if(! Connect()) {
    return false;
  }
  mqtt_client_->loop();
}

boolean MqttUtil::PublishTemperature(unsigned long timestamp, const char* const device, float temperature) {
  snprintf(topic_buffer_, kTopicBufferSize, "temperature/%s", device);
  dtostrf(temperature, 4, 2, value_buffer_);
  snprintf(message_buffer_, message_buffer_size_, "{\"timestamp\": %ld, \"value\": %s, \"units\": \"celsius degrees\"}", timestamp, value_buffer_);
  Serial.print(topic_buffer_);
  Serial.println(message_buffer_);
  // DISABLED by virtue of this constant return value instead of invoking publish
  return true;
  //return mqtt_client_->publish(topic_buffer_, message_buffer_);
}

