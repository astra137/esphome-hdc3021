#include "hdc3020.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace hdc3020 {

// ESPHome component for HDC302x relative humidity and temperature sensors
//
// Product: https://www.ti.com/product/HDC3020
// Datasheet: https://www.ti.com/lit/ds/symlink/hdc3020.pdf
// User Guide: https://www.ti.com/lit/ug/snau265c/snau265c.pdf
//
// Reference: Example code by Josh Wyatt
// https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/1190688/faq-hdc3020-is-there-an-energia-or-arduino-example-available

static const char *const TAG = "hdc3020";

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

void HDC3020Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HDC3020...");

  {
    // Verify Manufacturer ID
    // See dicussion at https://github.com/SndrSchnklshk/HDC302x/issues/4
    const uint8_t data[2] = {0x37, 0x81};

    if (this->write(data, 2) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "HDC3020 setup \"manufacturer ID\" write error");
      this->status_set_warning();
      return;
    }

    uint8_t manufacturer_id[3] = {0};

    if (this->read(manufacturer_id, 3) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "HDC3020 setup \"manufacturer ID\" read error");
      this->status_set_warning();
      return;
    }

    if (manufacturer_id[0] != 0x30 || manufacturer_id[1] != 0x00) {
      ESP_LOGW(TAG, "HDC3020 setup unexpected manufacturer error");
      this->status_set_warning();
      return;
    }
  }

  // Read offset, compare to configured offset, then update if necessary
  // See Datasheet 7.5.7.5 - Programmable Measurement Offset
  uint8_t humidity_offset = convert_offset_byte(12.5, humidity_offset_);
  uint8_t temperature_offset = convert_offset_byte(10.9375, temperature_offset_);
  bool previous_offset_matches = false;

  {
    const uint8_t cmd[2] = {0xA0, 0x04};
    if (this->write(cmd, 2) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "HDC3020 setup \"get offset\" write error");
      this->status_set_warning();
      return;
    }

    uint8_t offsets[3] = {0};
    if (this->read(offsets, 3) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "HDC3020 setup \"get offset\" read error");
      this->status_set_warning();
      return;
    }

    ESP_LOGCONFIG(TAG, "HDC3020 previous offset values RH=%02x T=%02x", offsets[0], offsets[1]);

    previous_offset_matches = offsets[0] == humidity_offset && offsets[1] == temperature_offset;
  }

  if (!previous_offset_matches) {
    ESP_LOGCONFIG(TAG, "HDC3020 updating offset values RH=%02x T=%02x", humidity_offset, temperature_offset);
    const uint8_t msg[2] = {humidity_offset, temperature_offset};
    const uint8_t cmd[5] = {0xA0, 0x04, humidity_offset, temperature_offset, calc_crc_byte(msg, 2)};
    if (this->write(cmd, 5) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "HDC3020 setup \"set offset\" write error");
      this->status_set_warning();
      return;
    }
    // 77ms taken from Datasheet 7.5.7.5.6 - Verify a Programmed Offset Value
    // It also satisfies the suggested 50ms delay after setting offsets
    delay(77);
  }
}

void HDC3020Component::update() {
  // See HDC302x Command Table on Datasheet page 18, Table 7-4
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

  // See Datasheet 7.5.5 - Retrieve Multi Data Result
  // Reinterpret temperature and humidity MSB and LSB, ignoring CRC bytes [2] and [5]
  uint16_t raw_temperature = (buffer[0] << 8) + buffer[1];
  uint16_t raw_humidity = (buffer[3] << 8) + buffer[4];

  // See Datasheet 7.3.3, formulas (1) and (2)
  float humidity = (((float) raw_humidity) / 65535.) * 100.;
  float temperature = (((float) raw_temperature) / 65535.) * (175.) - 45.;

  this->humidity_->publish_state(humidity);
  this->temperature_->publish_state(temperature);
  this->status_clear_warning();
}

float HDC3020Component::get_setup_priority() const { return setup_priority::DATA; }

// See Datasheet 7.5.7.5 - Programmable Measurement Offset
uint8_t convert_offset_byte(float base, float value) {
  float epsilon = base / 64.;  // precision of smallest (7th) bit
  if (value < epsilon && value > -epsilon) {
    return 0;
  }
  uint8_t byte = 0;
  if (value < 0.0) {
    value = -value;
  } else {
    byte |= 1 << 7;
  }
  for (int i = 6; i >= 0; i--) {
    if (value >= base) {
      byte |= 1 << i;
      value -= base;
    }
    base /= 2.0;
  }
  if (value * 2.0 > epsilon) {
    // So far, the value as been truncated similar to floor()
    // Round upward if more than half way to the next value.
    byte++;
  }
  return byte;
}

// See User Guide 3.3.1 - CRC C Code
uint8_t calc_crc_byte(const uint8_t *msg, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t byte = 0; byte < len; byte++) {
    crc ^= msg[byte];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

}  // namespace hdc3020
}  // namespace esphome
