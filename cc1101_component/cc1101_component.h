#ifndef CC1101_COMPONENT_H
#define CC1101_COMPONENT_H

#include "esphome.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"

namespace esphome {
namespace cc1101_component {

struct tmeter_data {
  int liters;
  int reads_counter;
  int battery_left;
  int time_start;
  int time_end;
};

class CC1101Component : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_frequency(float f);
  void set_meter_year(uint8_t y) { meter_year_ = y; }
  void set_meter_serial(uint32_t s) { meter_serial_ = s; }
  void update_meter_data();

  void set_gdo0_pin(GPIOPin *pin) { gdo0_pin_ = pin; }

  void set_liters_sensor(sensor::Sensor *sensor) { liters_sensor_ = sensor; }
  void set_counter_sensor(sensor::Sensor *sensor) { counter_sensor_ = sensor; }
  void set_battery_sensor(sensor::Sensor *sensor) { battery_sensor_ = sensor; }
  void set_time_start_sensor(sensor::Sensor *sensor) { time_start_sensor_ = sensor; }
  void set_time_end_sensor(sensor::Sensor *sensor) { time_end_sensor_ = sensor; }
  void set_rssi_sensor(sensor::Sensor *sensor) { rssi_sensor_ = sensor; }
  void set_tuned_frequency_sensor(sensor::Sensor *sensor) { tuned_frequency_sensor_ = sensor; }
  void set_json_sensor(text_sensor::TextSensor *sensor) { json_sensor_ = sensor; }
  void set_timestamp_sensor(text_sensor::TextSensor *sensor) { timestamp_sensor_ = sensor; }
  void set_time(time::RealTimeClock *time) { time_ = time; }

  std::string get_json() { return json_data_; }
  std::string get_timestamp() { return timestamp_str_; }

 protected:
  void cc1101_init(float freq);
  void cc1101_reset_();
  void cc1101_configure_rf_0_(float freq);
  void cc1101_rec_mode_();
  void tx_wakeup_and_request_(const uint8_t *txbuffer, int tx_len, bool quiet);
  int8_t rssi_to_dbm_(uint8_t rssi_reg);
  void setMHZ(float mhz);
  uint8_t halRfReadReg(uint8_t addr);
  void halRfWriteReg(uint8_t addr, uint8_t val);
  void SPIWriteBurstReg(uint8_t addr, uint8_t *data, uint8_t len);
  void SPIReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t len);
  void CC1101_CMD(uint8_t cmd);
  tmeter_data get_meter_data(int stage1_tmo_ms = 150, int stage2_tmo_ms = 700, bool quiet = false);
  int receive_radian_frame(int size_byte, int rx_tmo_ms, uint8_t *rx_buffer, int rx_buffer_size);
  uint8_t decode_4bitpbit_serial(uint8_t *rx_buffer, int rx_total_bytes, uint8_t *decoded_buffer);
  tmeter_data parse_meter_report(uint8_t *decoded_buffer, uint8_t size);
  int encode2serial_1_3(uint8_t *input, int input_len, uint8_t *output);
  int Make_Radian_Master_req(uint8_t *output, uint8_t year, uint32_t serial);
  uint16_t crc_kermit(const uint8_t *input, size_t len);
  int probe_frequency_rssi_dbm_(float freq_mhz, int window_ms);

  GPIOPin *gdo0_pin_;
  float frequency_ = 433.82f;
  uint8_t meter_year_ = 12;
  uint32_t meter_serial_ = 123456;
  tmeter_data meter_data_ = {0, 0, 0, 0, 0};
  bool radio_ready_ = false;
  std::string json_data_;
  std::string timestamp_str_;
  int last_rssi_dbm_ = -127;
  float last_tuned_frequency_mhz_ = NAN;

  sensor::Sensor *liters_sensor_{nullptr};
  sensor::Sensor *counter_sensor_{nullptr};
  sensor::Sensor *battery_sensor_{nullptr};
  sensor::Sensor *time_start_sensor_{nullptr};
  sensor::Sensor *time_end_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};
  sensor::Sensor *tuned_frequency_sensor_{nullptr};
  text_sensor::TextSensor *json_sensor_{nullptr};
  text_sensor::TextSensor *timestamp_sensor_{nullptr};
  time::RealTimeClock *time_{nullptr};
};

}  // namespace cc1101_component
}  // namespace esphome

#endif