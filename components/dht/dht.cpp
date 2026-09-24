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
                "  Max read attempts: %u\n"
                "  Max timeout attempts: %u\n"
                "  Offline after timeout cycles: %u\n"
                "  Retry delay: %lu ms",
                this->is_auto_detect_ ? LOG_STR_LITERAL("Auto-detected ") : "",
                this->model_ == DHT_MODEL_DHT11 ? LOG_STR_LITERAL("DHT11")
                                                : LOG_STR_LITERAL("DHT22 or equivalent"),
                ONOFF(this->t_pin_->get_flags() & gpio::FLAG_PULLUP),
                static_cast<unsigned>(MAX_READ_ATTEMPTS),
                static_cast<unsigned>(MAX_TIMEOUT_ATTEMPTS),
                static_cast<unsigned>(OFFLINE_AFTER_TIMEOUT_CYCLES),
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

  // In offline mode there is deliberately only one probe per update interval.
  const bool report_errors = this->sensor_offline_ || attempt >= MAX_READ_ATTEMPTS;
  const ReadStatus status = this->read_sensor_(&temperature, &humidity, report_errors);

  if (status == ReadStatus::OK) {
    this->publish_reading_(temperature, humidity, attempt);
    return;
  }

  if (this->sensor_offline_) {
    // One lightweight presence probe only. Keep the last known good value.
    this->retry_in_progress_ = false;

    if (status == ReadStatus::TIMEOUT) {
      ESP_LOGD(TAG, "DHT sensor still offline - keeping previous values");
    } else {
      // A non-timeout response means the device/data line is responding again.
      // Leave offline mode so the next normal update gets the full retry policy.
      this->sensor_offline_ = false;
      this->consecutive_timeout_cycles_ = 0;
      ESP_LOGI(TAG, "DHT sensor is responding again - normal retry mode restored");
    }

    this->status_set_warning();
    return;
  }

  // Timeout usually means no response at all. Retry it only once.
  const uint8_t max_attempts =
      status == ReadStatus::TIMEOUT ? MAX_TIMEOUT_ATTEMPTS : MAX_READ_ATTEMPTS;

  if (attempt < max_attempts) {
    ESP_LOGD(TAG,
             "DHT read failed (attempt %u/%u), retrying in %lu ms",
             static_cast<unsigned>(attempt),
             static_cast<unsigned>(max_attempts),
             static_cast<unsigned long>(RETRY_DELAY_MS));

    this->set_timeout("dht_retry", RETRY_DELAY_MS, [this, attempt]() {
      this->read_attempt_(attempt + 1);
    });
    return;
  }

  this->finish_failed_cycle_(status, attempt);
}

void DHT::finish_failed_cycle_(ReadStatus status, uint8_t attempt) {
  this->retry_in_progress_ = false;

  if (status == ReadStatus::TIMEOUT) {
    if (this->consecutive_timeout_cycles_ < 255)
      this->consecutive_timeout_cycles_++;

    ESP_LOGW(TAG,
             "DHT timeout cycle %u/%u - keeping previous values",
             static_cast<unsigned>(this->consecutive_timeout_cycles_),
             static_cast<unsigned>(OFFLINE_AFTER_TIMEOUT_CYCLES));

    if (this->consecutive_timeout_cycles_ >= OFFLINE_AFTER_TIMEOUT_CYCLES) {
      this->sensor_offline_ = true;
      ESP_LOGW(TAG,
               "DHT sensor appears offline - only one probe will be made per update interval");
    }
  } else {
    // It did respond, so it is not considered physically absent.
    this->consecutive_timeout_cycles_ = 0;

    ESP_LOGW(TAG,
             "DHT read failed after %u attempts - keeping previous values",
             static_cast<unsigned>(attempt));
  }

  this->status_set_warning();

  // Preserve AUTO_DETECT behaviour: after a failed DHT22 cycle,
  // try DHT11 timing on the next regular polling cycle.
  if (this->is_auto_detect_ && this->model_ == DHT_MODEL_DHT22) {
    ESP_LOGW(TAG, "Auto-detect: switching to DHT11 timing for next update");
    this->model_ = DHT_MODEL_DHT11;
  }
}

void DHT::publish_reading_(float temperature, float humidity, uint8_t attempt) {
  this->retry_in_progress_ = false;

  if (this->sensor_offline_) {
    ESP_LOGI(TAG, "DHT sensor recovered");
  } else if (attempt > 1) {
    ESP_LOGD(TAG,
             "DHT read succeeded on attempt %u/%u",
             static_cast<unsigned>(attempt),
             static_cast<unsigned>(MAX_READ_ATTEMPTS));
  }

  this->sensor_offline_ = false;
  this->consecutive_timeout_cycles_ = 0;

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

DHT::ReadStatus DHT::read_sensor_(float *temperature, float *humidity, bool report_errors) {
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
    if (result == DHTLIB_ERROR_TIMEOUT) {
      if (report_errors)
        ESP_LOGW(TAG, "DHTStable timeout");
      return ReadStatus::TIMEOUT;
    }

    if (result == DHTLIB_ERROR_CHECKSUM) {
      if (report_errors)
        ESP_LOGW(TAG, "DHTStable checksum error");
      return ReadStatus::CHECKSUM;
    }

    if (report_errors)
      ESP_LOGW(TAG, "DHTStable error: %d", result);

    return ReadStatus::OTHER_ERROR;
  }

  const float new_temperature = this->dht_stable_.getTemperature();
  const float new_humidity = this->dht_stable_.getHumidity();

  // Never publish NaN/Inf even if the library returned DHTLIB_OK.
  if (!std::isfinite(new_temperature) || !std::isfinite(new_humidity)) {
    if (report_errors)
      ESP_LOGW(TAG, "DHTStable returned non-finite value");
    return ReadStatus::INVALID_VALUE;
  }

  *temperature = new_temperature;
  *humidity = new_humidity;

  ESP_LOGV(TAG,
           "DHTStable read: %.2f °C, %.2f %%",
           *temperature,
           *humidity);

  return ReadStatus::OK;
}

}  // namespace esphome::dht
