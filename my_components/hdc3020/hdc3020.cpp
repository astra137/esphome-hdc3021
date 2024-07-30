#include "hdc3020.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace hdc3020 {

// ESPHome component for HDC302x relative humidity and temperature sensors
//
// Product: https://www.ti.com/product/HDC3020
// Datasheet: https://www.ti.com/lit/ds/symlink/hdc3020.pdf
//
// Reference: Example code by Josh Wyatt
// https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/1190688/faq-hdc3020-is-there-an-energia-or-arduino-example-available

static const char *const TAG = "hdc3020";

void HDC3020Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HDC3020...");

  // Use Manufacturer ID command as a setup check, but anything could work
  // https://github.com/SndrSchnklshk/HDC302x/issues/4
  const uint8_t data[2] = {0x37, 0x81};

  if (!this->write(data, 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "HDC3020 setup command write error");
    this->status_set_warning();
    return;
  }

  uint16_t raw_manufacturer_id;

  if (this->read(reinterpret_cast<uint8_t *>(&raw_manufacturer_id), 2) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "HDC3020 setup command read error");
    this->status_set_warning();
    return;
  }

  raw_manufacturer_id = i2c::i2ctohs(raw_manufacturer_id);

  if (raw_manufacturer_id != 0x3000u) {
    ESP_LOGW(TAG, "HDC3020 setup unexpected manufacturer ID error");
    this->status_set_warning();
    return;
  }
}

void HDC3020Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HDC3020:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with HDC3020 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_);
  LOG_SENSOR("  ", "Humidity", this->humidity_);
}

void HDC3020Component::update() {
  // See HDC302x Command Table on datasheet page 18, Table 7-4
  const uint8_t data[2] = {
      0x24,  // MSB 0x24 => Trigger-On Demand Mode
      0x00   // LSB 0x00 => Low Power Mode 0 (lowest noise)
  };

  if (this->write(data, 2) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  // In LPM0, typical measurement duration is 12.5 ms
  // 15 ms is chosen from Josh Wyatt's example code
  delay(15);

  uint8_t buffer[6] = {0};

  if (this->read(buffer, 6) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  // Section 7.5.5 - Retrieve Multi Data Result
  // Reinterpret temperature and humidity MSB and LSB, ignoring CRC bytes [2] and [5]
  uint16_t raw_temperature = (buffer[0] << 8) + buffer[1];
  uint16_t raw_humidity = (buffer[3] << 8) + buffer[4];

  // Section 7.3.3, formulas (1) and (2)
  float humidity = (((float) raw_humidity) / 65535.) * 100.;
  float temperature = (((float) raw_temperature) / 65535.) * (175.) - 45.;

  this->humidity_->publish_state(humidity);
  this->temperature_->publish_state(temperature);
  this->status_clear_warning();
}

float HDC3020Component::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace hdc3020
}  // namespace esphome
