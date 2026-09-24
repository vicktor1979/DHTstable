#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include <DHTStable.h>

namespace esphome::dht {

enum DHTModel : uint8_t {
  DHT_MODEL_AUTO_DETECT = 0,
  DHT_MODEL_DHT11,
  DHT_MODEL_DHT22,
  DHT_MODEL_AM2120,
  DHT_MODEL_AM2302,
  DHT_MODEL_RHT03,
  DHT_MODEL_SI7021,
  DHT_MODEL_DHT22_TYPE2
};

class DHT final : public PollingComponent {
 public:
  void set_dht_model(DHTModel model);

  void set_pin(InternalGPIOPin *pin) { this->t_pin_ = pin; }

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) {
    this->temperature_sensor_ = temperature_sensor;
  }

  void set_humidity_sensor(sensor::Sensor *humidity_sensor) {
    this->humidity_sensor_ = humidity_sensor;
  }

  void setup() override;
  void dump_config() override;
  void update() override;

 protected:
  static constexpr uint8_t MAX_READ_ATTEMPTS = 3;
  static constexpr uint8_t MAX_TIMEOUT_ATTEMPTS = 2;
  static constexpr uint8_t OFFLINE_AFTER_TIMEOUT_CYCLES = 3;
  static constexpr uint32_t RETRY_DELAY_MS = 2200;

  enum class ReadStatus : uint8_t {
    OK = 0,
    TIMEOUT,
    CHECKSUM,
    INVALID_VALUE,
    OTHER_ERROR,
  };

  void read_attempt_(uint8_t attempt);
  void publish_reading_(float temperature, float humidity, uint8_t attempt);
  void finish_failed_cycle_(ReadStatus status, uint8_t attempt);
  ReadStatus read_sensor_(float *temperature, float *humidity, bool report_errors);

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  InternalGPIOPin *t_pin_{nullptr};

  DHTModel model_{DHT_MODEL_AUTO_DETECT};
  bool is_auto_detect_{false};
  bool retry_in_progress_{false};
  bool sensor_offline_{false};

  uint8_t consecutive_timeout_cycles_{0};

  DHTStable dht_stable_;
};

}  // namespace esphome::dht
