#include "dht.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::dht {

static const char *const TAG = "dht";

void DHT::setup() {
  // Let ESPHome apply the configured GPIO flags first.
  this->t_pin_->digital_write(true);
  this->t_pin_->setup();
  this->t_pin_->digital_write(true);

  // Preserve ESPHome's pin pull-up setting inside DHTStable.
  const bool pullup = (this->t_pin_->get_flags() & gpio::FLAG_PULLUP) != 0;
  this->dht_stable_.setPullup(pullup);

  // DHTStable can disable IRQs for the complete transaction, including
  // the sensor wake-up delay. Keep this disabled to avoid long IRQ-off
  // periods, especially with DHT11.
  this->dht_stable_.setDisableIRQ(false);
}

void DHT::dump_config() {
  ESP_LOGCONFIG(TAG,
                "DHTStable:\n"
                "  %sModel: %s\n"
                "  Internal pull-up: %s",
                this->is_auto_detect_ ? LOG_STR_LITERAL("Auto-detected ") : "",
                this->model_ == DHT_MODEL_DHT11 ? LOG_STR_LITERAL("DHT11")
                                                : LOG_STR_LITERAL("DHT22 or equivalent"),
                ONOFF(this->t_pin_->get_flags() & gpio::FLAG_PULLUP));

  LOG_PIN("  Pin: ", this->t_pin_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

void DHT::update() {
  float temperature;
  float humidity;
  bool success;

  if (this->model_ == DHT_MODEL_AUTO_DETECT) {
    // Match ESPHome's normal behaviour: try DHT22 first.
    this->model_ = DHT_MODEL_DHT22;
    success = this->read_sensor_(&temperature, &humidity, false);

    if (!success) {
      // Try DHT11 on the next polling cycle.
      this->model_ = DHT_MODEL_DHT11;
      return;
    }
  } else {
    success = this->read_sensor_(&temperature, &humidity, true);
  }

  if (success) {
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(temperature);

    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(humidity);

    this->status_clear_warning();
  } else {
    ESP_LOGW(TAG, "Invalid readings");

    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(NAN);

    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(NAN);

    this->status_set_warning();
  }
}

void DHT::set_dht_model(DHTModel model) {
  this->model_ = model;
  this->is_auto_detect_ = model == DHT_MODEL_AUTO_DETECT;
}

bool DHT::read_sensor_(float *temperature, float *humidity, bool report_errors) {
  *temperature = NAN;
  *humidity = NAN;

  const uint8_t pin = this->t_pin_->get_pin();

  int result;
  switch (this->model_) {
    case DHT_MODEL_DHT11:
      result = this->dht_stable_.read11(pin);
      break;

    case DHT_MODEL_AM2302:
      result = this->dht_stable_.read2302(pin);
      break;

    case DHT_MODEL_DHT22:
    case DHT_MODEL_AM2120:
    case DHT_MODEL_RHT03:
    case DHT_MODEL_SI7021:
    case DHT_MODEL_DHT22_TYPE2:
    default:
      result = this->dht_stable_.read22(pin);
      break;
  }

  if (result != DHTLIB_OK) {
    if (report_errors) {
      if (result == DHTLIB_ERROR_CHECKSUM) {
        ESP_LOGW(TAG, "DHTStable checksum error");
      } else if (result == DHTLIB_ERROR_TIMEOUT) {
        ESP_LOGW(TAG, "DHTStable timeout");
      } else {
        ESP_LOGW(TAG, "DHTStable error: %d", result);
      }
    }
    return false;
  }

  *temperature = this->dht_stable_.getTemperature();
  *humidity = this->dht_stable_.getHumidity();

  ESP_LOGV(TAG, "DHTStable read: %.2f °C, %.2f %%", *temperature, *humidity);
  return true;
}

}  // namespace esphome::dht
