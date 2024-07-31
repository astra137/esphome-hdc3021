#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace hdc3020 {

class HDC3020Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_temperature(sensor::Sensor *temperature) { temperature_ = temperature; }
  void set_humidity(sensor::Sensor *humidity) { humidity_ = humidity; }

  void set_temperature_offset(float offset) { temperature_offset_ = offset; }
  void set_humidity_offset(float offset) { humidity_offset_ = offset; }

  void setup() override;
  void dump_config() override;
  void update() override;

  float get_setup_priority() const override;

 protected:
  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *humidity_{nullptr};
  float temperature_offset_ = 0;
  float humidity_offset_ = 0;
};

uint8_t convert_offset_byte(float base, float value);

uint8_t calc_crc_byte(const uint8_t *msg, size_t len);

}  // namespace hdc3020
}  // namespace esphome
