#include "dht.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cmath>

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

  // Avoid disabling IRQs for the whole DHT transaction.
  this->dht_stable_.setDisableIRQ(false);
}

void DHT::dump_config() {
  ESP_LOGCONFIG(TAG,
                "DHTStable:\n"
                "  %sModel: %s\n"
                "  Internal pull-up: %s\n"
                "  Read attempts: %u\n"
                "  Retry delay: %lu ms",
                this->is_auto_detect_ ? LOG_STR_LITERAL("Auto-detected ") : "",
                this->model_ == DHT_MODEL_DHT11 ? LOG_STR_LITERAL("DHT11")
                                                : LOG_STR_LITERAL("DHT22 or equivalent"),
                ONOFF(this->t_pin_->get_flags() & gpio::FLAG_PULLUP),
                static_cast<unsigned>(MAX_READ_ATTEMPTS),
                static_cast<unsigned long>(RETRY_DELAY_MS));

  LOG_PIN("  Pin: ", this->t_pin_);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

void DHT::update() {
  // Do not start another measurement cycle while a retry cycle is active.
  if (this->retry_in_progress_) {
    ESP_LOGD(TAG, "Read cycle already in progress, skipping scheduled update");
    return;
  }

  // For AUTO_DETECT, start by trying DHT22 timing.
  if (this->is_auto_detect_ && this->model_ == DHT_MODEL_AUTO_DETECT) {
    this->model_ = DHT_MODEL_DHT22;
  }

  this->retry_in_progress_ = true;
  this->read_attempt_(1);
}

void DHT::read_attempt_(uint8_t attempt) {
  float temperature = NAN;
  float humidity = NAN;

  // Only print the low-level DHTStable error on the final attempt.
  const bool report_errors = attempt >= MAX_READ_ATTEMPTS;
  const bool success = this->read_sensor_(&temperature, &humidity, report_errors);

  if (success) {
    this->publish_reading_(temperature, humidity, attempt);
    return;
  }

  if (attempt < MAX_READ_ATTEMPTS) {
    ESP_LOGD(TAG,
             "DHT read failed (attempt %u/%u), retrying in %lu ms",
             static_cast<unsigned>(attempt),
             static_cast<unsigned>(MAX_READ_ATTEMPTS),
             static_cast<unsigned long>(RETRY_DELAY_MS));

    this->set_timeout("dht_retry", RETRY_DELAY_MS, [this, attempt]() {
      this->read_attempt_(attempt + 1);
    });
    return;
  }

  this->retry_in_progress_ = false;

  // Keep the last successfully published values.
  // Do not publish NAN, because Home Assistant would show Unknown.
  ESP_LOGW(TAG,
           "DHT read failed after %u attempts - keeping previous values",
           static_cast<unsigned>(MAX_READ_ATTEMPTS));

  this->status_set_warning();

  // Preserve the old AUTO_DETECT behaviour: after repeated DHT22 failures,
  // try DHT11 timing on the next regular polling cycle.
  if (this->is_auto_detect_ && this->model_ == DHT_MODEL_DHT22) {
    ESP_LOGW(TAG, "Auto-detect: switching to DHT11 timing for next update");
    this->model_ = DHT_MODEL_DHT11;
  }
}

void DHT::publish_reading_(float temperature, float humidity, uint8_t attempt) {
  this->retry_in_progress_ = false;

  if (attempt > 1) {
    ESP_LOGD(TAG,
             "DHT read succeeded on attempt %u/%u",
             static_cast<unsigned>(attempt),
             static_cast<unsigned>(MAX_READ_ATTEMPTS));
  }

  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);

  if (this->humidity_sensor_ != nullptr)
    this->humidity_sensor_->publish_state(humidity);

  this->status_clear_warning();
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

  const float new_temperature = this->dht_stable_.getTemperature();
  const float new_humidity = this->dht_stable_.getHumidity();

  // Never publish NaN/Inf even if the library returned DHTLIB_OK.
  if (!std::isfinite(new_temperature) || !std::isfinite(new_humidity)) {
    if (report_errors) {
      ESP_LOGW(TAG, "DHTStable returned non-finite value");
    }
    return false;
  }

  *temperature = new_temperature;
  *humidity = new_humidity;

  ESP_LOGV(TAG,
           "DHTStable read: %.2f °C, %.2f %%",
           *temperature,
           *humidity);
  return true;
}

}  // namespace esphome::dht
