#include "cc1101_component.h"
#include "esphome/core/log.h"
#include <cstdarg>

namespace esphome {
namespace cc1101_component {

static const char *TAG = "cc1101_component";

static uint8_t RF_config_u8 = 0xFF;
static uint8_t RF_Test_u8 = 0;
static uint8_t PA[] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t CC1101_status_state = 0;
static uint8_t CC1101_status_FIFO_FreeByte = 0;
static uint8_t CC1101_status_FIFO_ReadByte = 0;
static bool debug_out = false;

#define WRITE_SINGLE_BYTE 0x00
#define WRITE_BURST 0x40
#define READ_SINGLE_BYTE 0x80
#define READ_BURST 0xC0

#define IOCFG2 0x00
#define IOCFG1 0x01
#define IOCFG0 0x02
#define FIFOTHR 0x03
#define SYNC1 0x04
#define SYNC0 0x05
#define PKTLEN 0x06
#define PKTCTRL1 0x07
#define PKTCTRL0 0x08
#define ADDRR 0x09
#define CHANNR 0x0A
#define FSCTRL1 0x0B
#define FSCTRL0 0x0C
#define FREQ2 0x0D
#define FREQ1 0x0E
#define FREQ0 0x0F
#define MDMCFG4 0x10
#define MDMCFG3 0x11
#define MDMCFG2 0x12
#define MDMCFG1 0x13
#define MDMCFG0 0x14
#define DEVIATN 0x15
#define MCSM2 0x16
#define MCSM1 0x17
#define MCSM0 0x18
#define FOCCFG 0x19
#define BSCFG 0x1A
#define AGCCTRL2 0x1B
#define AGCCTRL1 0x1C
#define AGCCTRL0 0x1D
#define WOREVT1 0x1E
#define WOREVT0 0x1F
#define WORCTRL 0x20
#define FREND1 0x21
#define FREND0 0x22
#define FSCAL3 0x23
#define FSCAL2 0x24
#define FSCAL1 0x25
#define FSCAL0 0x26
#define RCCTRL1 0x27
#define RCCTRL0 0x28
#define FSTEST 0x29
#define PTEST 0x2A
#define AGCTEST 0x2B
#define TEST2 0x2C
#define TEST1 0x2D
#define TEST0 0x2E
#define SRES 0x30
#define SFSTXON 0x31
#define SXOFF 0x32
#define SCAL 0x33
#define SRX 0x34
#define STX 0x35
#define SIDLE 0x36
#define SWOR 0x38
#define SPTX 0x39
#define SFRX 0x3A
#define SFTX 0x3B
#define SWORRST 0x3C
#define SNOP 0x3D
#define TX_FIFO_ADDR 0x3F
#define PATABLE_ADDR 0x3E

// Status registers live at 0xF0..0xFF (datasheet: burst bit must be set; using 0xFx addresses matches Arduino code)
#define PARTNUM_ADDR 0xF0
#define VERSION_ADDR 0xF1
#define FREQEST_ADDR 0xF2
#define LQI_ADDR 0xF3
#define RSSI_ADDR 0xF4
#define MARCSTATE_ADDR 0xF5
#define TXBYTES_ADDR 0xFA
#define RXBYTES_ADDR 0xFB

#define RX_FIFO_ADDR 0x3F
#define RXBYTES_MASK 0x7F
#define TX_LOOP_OUT 300
#define CRC_START_KERMIT 0x0000
#define CRC_POLY_KERMIT 0x8408

static uint8_t crc_tab_init = 0;
static uint16_t crc_tab[256];

void init_crc_tab(void) {
  uint16_t i, j, crc, c;
  for (i = 0; i < 256; i++) {
    crc = 0;
    c = i;
    for (j = 0; j < 8; j++) {
      if ((crc ^ c) & 0x0001) crc = (crc >> 1) ^ CRC_POLY_KERMIT;
      else crc = crc >> 1;
      c = c >> 1;
    }
    crc_tab[i] = crc;
  }
  crc_tab_init = 1;
}

uint16_t CC1101Component::crc_kermit(const uint8_t *input, size_t len) {
  if (!crc_tab_init) init_crc_tab();
  uint16_t crc = CRC_START_KERMIT;
  for (size_t a = 0; a < len; a++) {
    uint16_t short_c = 0x00ff & input[a];
    uint16_t tmp = crc ^ short_c;
    crc = (crc >> 8) ^ crc_tab[tmp & 0xff];
  }
  return ((crc & 0xff00) >> 8) | ((crc & 0x00ff) << 8);
}

int CC1101Component::encode2serial_1_3(uint8_t *input, int input_len, uint8_t *output) {
  // Ported 1:1 from the working Arduino implementation.
  int bytepos = 0;
  int bitpos;
  int j = 0;

  for (int i = 0; i < (input_len * 8); i++) {
    if (i % 8 == 0) {
      if (i > 0) {
        // Insert stop bits (3)
        bytepos = j / 8;
        bitpos = j % 8;
        output[bytepos] |= 1 << (7 - bitpos);
        j++;

        bytepos = j / 8;
        bitpos = j % 8;
        output[bytepos] |= 1 << (7 - bitpos);
        j++;

        bytepos = j / 8;
        bitpos = j % 8;
        output[bytepos] |= 1 << (7 - bitpos);
        j++;
      }

      // Insert start bit (0)
      bytepos = j / 8;
      bitpos = j % 8;
      output[bytepos] &= ~(1 << (7 - bitpos));
      j++;
    }

    // NOTE: bitpos is LSB-first (matches Arduino).
    bytepos = i / 8;
    bitpos = i % 8;
    uint8_t mask = 1 << bitpos;

    if ((input[bytepos] & mask) > 0) {
      bytepos = j / 8;
      bitpos = 7 - (j % 8);
      output[bytepos] |= 1 << bitpos;
    } else {
      bytepos = j / 8;
      bitpos = 7 - (j % 8);
      output[bytepos] &= ~(1 << bitpos);
    }
    j++;
  }

  // Insert additional stop bits until end of byte
  while (j % 8 > 0) {
    bytepos = j / 8;
    bitpos = 7 - (j % 8);
    output[bytepos] |= 1 << bitpos;
    j++;
  }

  output[bytepos + 1] = 0xFF;
  return bytepos + 2;
}

int CC1101Component::Make_Radian_Master_req(uint8_t *output, uint8_t year, uint32_t serial) {
  uint8_t to_encode[19] = {0x13, 0x10, 0x00, 0x45, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x45, 0x20, 0x0A, 0x50, 0x14, 0x00, 0x0A, 0x40, 0xFF, 0xFF};
  uint8_t synch_pattern[9] = {0x50, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF};
  to_encode[4] = year;
  to_encode[5] = (serial >> 16) & 0xFF;
  to_encode[6] = (serial >> 8) & 0xFF;
  to_encode[7] = serial & 0xFF;
  uint16_t crc = crc_kermit(to_encode, sizeof(to_encode) - 2);
  to_encode[sizeof(to_encode) - 2] = (crc >> 8) & 0xFF;
  to_encode[sizeof(to_encode) - 1] = crc & 0xFF;
  memcpy(output, synch_pattern, 9);
  int encoded_len = encode2serial_1_3(to_encode, 19, output + 9);
  return 9 + encoded_len;
}

tmeter_data CC1101Component::parse_meter_report(uint8_t *data, uint8_t len) {
  // Match the working Arduino implementation: decoded buffer is binary, not ASCII.
  tmeter_data result;
  memset(&result, 0, sizeof(result));

  if (len >= 22) {
    result.liters = (int) data[18] + (int) data[19] * 256 + (int) data[20] * 65536 + (int) data[21] * 16777216;
  }
  if (len >= 49) {
    result.reads_counter = data[48];
    result.battery_left = data[31];
    result.time_start = data[44];
    result.time_end = data[45];
  }
  return result;
}

// Remove start/stop bits and decode oversampled bits (4 samples per bit), ported from the Arduino version.
uint8_t CC1101Component::decode_4bitpbit_serial(uint8_t *rx_buffer, int rx_total_bytes, uint8_t *decoded_buffer) {
  uint16_t i, j, k;
  uint8_t bit_cnt = 0;
  int8_t bit_cnt_flush_s8 = 0;
  uint8_t bit_pol = 0;
  uint8_t dest_bit_cnt = 0;
  uint8_t dest_byte_cnt = 0;
  uint8_t current_rx_byte;

  if (rx_total_bytes <= 0) return 0;
  bit_pol = (rx_buffer[0] & 0x80);

  for (i = 0; i < (uint16_t) rx_total_bytes; i++) {
    current_rx_byte = rx_buffer[i];
    for (j = 0; j < 8; j++) {
      if ((current_rx_byte & 0x80) == bit_pol) {
        bit_cnt++;
      } else if (bit_cnt == 1) {
        bit_pol = current_rx_byte & 0x80;
        bit_cnt = bit_cnt_flush_s8 + 1;
      } else {
        bit_cnt_flush_s8 = bit_cnt;
        bit_cnt = (bit_cnt + 2) / 4;
        bit_cnt_flush_s8 = bit_cnt_flush_s8 - (bit_cnt * 4);

        for (k = 0; k < bit_cnt; k++) {
          if (dest_bit_cnt < 8) {
            decoded_buffer[dest_byte_cnt] = decoded_buffer[dest_byte_cnt] >> 1;
            decoded_buffer[dest_byte_cnt] |= bit_pol;
          }
          dest_bit_cnt++;

          if ((dest_bit_cnt == 10) && (!bit_pol)) {
            return dest_byte_cnt;
          }
          if ((dest_bit_cnt >= 11) && (!bit_pol)) {
            dest_bit_cnt = 0;
            dest_byte_cnt++;
          }
        }

        bit_pol = current_rx_byte & 0x80;
        bit_cnt = 1;
      }
      current_rx_byte = current_rx_byte << 1;
    }
  }
  return dest_byte_cnt;
}

void CC1101Component::CC1101_CMD(uint8_t cmd) {
  this->enable();
  this->write_byte(cmd);
  this->disable();
}

uint8_t CC1101Component::halRfReadReg(uint8_t addr) {
  this->enable();
  this->write_byte(addr | READ_SINGLE_BYTE);
  uint8_t val = this->read_byte();
  this->disable();
  return val;
}

void CC1101Component::halRfWriteReg(uint8_t addr, uint8_t val) {
  this->enable();
  this->write_byte(addr);
  this->write_byte(val);
  this->disable();
}

void CC1101Component::SPIWriteBurstReg(uint8_t addr, uint8_t *data, uint8_t len) {
  this->enable();
  this->write_byte(addr | WRITE_BURST);
  this->write_array(data, len);
  this->disable();
}

void CC1101Component::SPIReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t len) {
  this->enable();
  this->write_byte(addr | READ_BURST);
  this->read_array(buffer, len);
  this->disable();
}

void CC1101Component::setMHZ(float mhz) {
  // Ported from the working Arduino setMHZ() to avoid subtle rounding differences.
  uint8_t freq2 = 0;
  uint8_t freq1 = 0;
  uint16_t freq0 = 0;

  for (bool done = false; !done;) {
    if (mhz >= 26.0f) {
      mhz -= 26.0f;
      freq2 += 1;
    } else if (mhz >= 0.1015625f) {
      mhz -= 0.1015625f;
      freq1 += 1;
    } else if (mhz >= 0.00039675f) {
      mhz -= 0.00039675f;
      freq0 += 1;
    } else {
      done = true;
    }
  }
  // Carry (should be rare, but keep it correct).
  freq1 = static_cast<uint8_t>(freq1 + (freq0 >> 8));
  freq0 &= 0xFF;

  halRfWriteReg(FREQ2, freq2);
  halRfWriteReg(FREQ1, freq1);
  halRfWriteReg(FREQ0, static_cast<uint8_t>(freq0));
}

void CC1101Component::cc1101_reset_() {
  CC1101_CMD(SRES);
  delay(1);
  CC1101_CMD(SFTX);
  CC1101_CMD(SFRX);
}

void CC1101Component::cc1101_configure_rf_0_(float freq) {
  // Ported from Arduino's cc1101_configureRF_0().
  RF_config_u8 = 0;

  halRfWriteReg(IOCFG2, 0x0D);  // Serial data output
  halRfWriteReg(IOCFG0, 0x06);  // Assert on sync, deassert end of packet
  halRfWriteReg(FIFOTHR, 0x47);
  halRfWriteReg(SYNC1, 0x55);
  halRfWriteReg(SYNC0, 0x00);

  halRfWriteReg(PKTLEN, 38);
  halRfWriteReg(PKTCTRL1, 0x00);
  halRfWriteReg(PKTCTRL0, 0x00);
  halRfWriteReg(FSCTRL1, 0x08);

  setMHZ(freq);

  // Read back programmed frequency to catch mismatches.
  const uint32_t fw = (static_cast<uint32_t>(halRfReadReg(FREQ2)) << 16) |
                      (static_cast<uint32_t>(halRfReadReg(FREQ1)) << 8) |
                      (static_cast<uint32_t>(halRfReadReg(FREQ0)));
  const float actual_mhz = (static_cast<float>(fw) * 26.0f) / 65536.0f;
  this->last_tuned_frequency_mhz_ = actual_mhz;
  if (this->tuned_frequency_sensor_ != nullptr) this->tuned_frequency_sensor_->publish_state(actual_mhz);
  ESP_LOGD(TAG, "FREQ regs: FREQ2=0x%02X FREQ1=0x%02X FREQ0=0x%02X -> %.6f MHz (requested %.6f)",
           (uint8_t) (fw >> 16), (uint8_t) (fw >> 8), (uint8_t) (fw), actual_mhz, freq);

  halRfWriteReg(MDMCFG4, 0xF6);
  halRfWriteReg(MDMCFG3, 0x83);
  halRfWriteReg(MDMCFG2, 0x02);
  halRfWriteReg(MDMCFG1, 0x00);
  halRfWriteReg(MDMCFG0, 0x00);
  halRfWriteReg(DEVIATN, 0x15);
  halRfWriteReg(MCSM1, 0x00);
  halRfWriteReg(MCSM0, 0x18);
  halRfWriteReg(FOCCFG, 0x1D);
  halRfWriteReg(BSCFG, 0x1C);
  halRfWriteReg(AGCCTRL2, 0xC7);
  halRfWriteReg(AGCCTRL1, 0x00);
  halRfWriteReg(AGCCTRL0, 0xB2);
  halRfWriteReg(WORCTRL, 0xFB);
  halRfWriteReg(FREND1, 0xB6);
  halRfWriteReg(FSCAL3, 0xE9);
  halRfWriteReg(FSCAL2, 0x2A);
  halRfWriteReg(FSCAL1, 0x00);
  halRfWriteReg(FSCAL0, 0x1F);
  halRfWriteReg(TEST2, 0x81);
  halRfWriteReg(TEST1, 0x35);
  halRfWriteReg(TEST0, 0x09);

  SPIWriteBurstReg(PATABLE_ADDR, PA, 8);
}

void CC1101Component::cc1101_init(float freq) {
  ESP_LOGD(TAG, "Starting CC1101 init at %f MHz", freq);
  cc1101_reset_();
  cc1101_configure_rf_0_(freq);
}

void CC1101Component::cc1101_rec_mode_() {
  uint8_t marcstate;
  CC1101_CMD(SIDLE);
  CC1101_CMD(SRX);
  marcstate = 0xFF;
  while ((marcstate != 0x0D) && (marcstate != 0x0E) && (marcstate != 0x0F)) {
    marcstate = halRfReadReg(MARCSTATE_ADDR);
    delay(1);
  }
}

void CC1101Component::set_frequency(float f) {
  this->frequency_ = f;
  if (!this->radio_ready_) return;
  cc1101_init(this->frequency_);
}

void CC1101Component::tx_wakeup_and_request_(const uint8_t *txbuffer, int tx_len, bool quiet) {
  halRfWriteReg(MDMCFG2, 0x00);   // clear to not send preamble and sync
  halRfWriteReg(PKTCTRL0, 0x02);  // infinite packet len

  const uint8_t wupbuffer[8] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
  int wup2send = 77;

  // Mirror the Arduino sketch's TX pacing more closely.
  CC1101_CMD(SIDLE);
  CC1101_CMD(SFTX);
  SPIWriteBurstReg(TX_FIFO_ADDR, (uint8_t *) wupbuffer, 8);
  wup2send--;
  CC1101_CMD(STX);
  delay(10);  // calibration settle

  bool request_written = false;
  int loops = 0;
  uint8_t marc = halRfReadReg(MARCSTATE_ADDR) & 0x1F;
  if (!quiet && marc != 0x13) {
    ESP_LOGW(TAG, "TX: unexpected MARCSTATE after STX: 0x%02X", marc);
  }

  while (loops < TX_LOOP_OUT && marc == 0x13) {  // 0x13 == TX
    const uint8_t txbytes_raw = halRfReadReg(TXBYTES_ADDR);
    if (txbytes_raw & 0x80) {
      if (!quiet) ESP_LOGW(TAG, "TX: TXFIFO underflow");
      break;
    }
    const int txbytes = txbytes_raw & 0x7F;
    const int free_bytes = 64 - txbytes;

    if (wup2send > 0) {
      if (free_bytes <= 10) {
        delay(20);
        loops += 2;
      }
      if (free_bytes >= 8) {
        SPIWriteBurstReg(TX_FIFO_ADDR, (uint8_t *) wupbuffer, 8);
        wup2send--;
      }
    } else if (!request_written) {
      // Give the FIFO time to drain enough to accept the request (Arduino uses a fixed 130ms).
      delay(130);
      int guard = 0;
      while (guard < 60) {
        const int fb = 64 - (halRfReadReg(TXBYTES_ADDR) & 0x7F);
        if (fb >= tx_len) break;
        delay(5);
        guard++;
      }
      SPIWriteBurstReg(TX_FIFO_ADDR, (uint8_t *) txbuffer, tx_len);
      request_written = true;
    }

    delay(10);
    loops++;
    marc = halRfReadReg(MARCSTATE_ADDR) & 0x1F;
  }

  if (!request_written && !quiet) ESP_LOGW(TAG, "TX: request not written");

  // Flush TX FIFO (Arduino relies on this to clear state back to IDLE).
  CC1101_CMD(SFTX);
  halRfWriteReg(MDMCFG2, 0x02);
  halRfWriteReg(PKTCTRL0, 0x00);
}

int8_t CC1101Component::rssi_to_dbm_(uint8_t rssi_reg) {
  // Ported from Arduino cc1100_rssi_convert2dbm.
  if (rssi_reg >= 128) return ((int8_t) (rssi_reg - 256) / 2) - 74;
  return ((int8_t) rssi_reg / 2) - 74;
}

int CC1101Component::probe_frequency_rssi_dbm_(float freq_mhz, int window_ms) {
  // Send a meter request at the given frequency, then sample RSSI for a short window.
  // This helps scanning even when full frame decode isn't achieved.
  cc1101_init(freq_mhz);

  uint8_t txbuffer[100];
  memset(txbuffer, 0, sizeof(txbuffer));
  const int tx_len = Make_Radian_Master_req(txbuffer, meter_year_, meter_serial_);

  tx_wakeup_and_request_(txbuffer, tx_len, true);
  delay(43);
  CC1101_CMD(SFRX);
  cc1101_rec_mode_();

  int max_rssi = -127;
  const uint32_t start = millis();
  while ((int32_t) (millis() - start) < window_ms) {
    const int rssi = this->rssi_to_dbm_(halRfReadReg(RSSI_ADDR));
    if (rssi > max_rssi) max_rssi = rssi;

    // Small hint of activity: FIFO gets any bytes.
    const uint8_t rxbytes = halRfReadReg(RXBYTES_ADDR) & RXBYTES_MASK;
    if (rxbytes > 0) {
      // If anything arrived, weight the score slightly upward.
      if (max_rssi < -30) max_rssi += 3;
    }
    delay(5);
  }

  CC1101_CMD(SFRX);
  CC1101_CMD(SIDLE);
  this->last_rssi_dbm_ = max_rssi;
  return max_rssi;
}

int CC1101Component::receive_radian_frame(int size_byte, int rx_tmo_ms, uint8_t *rx_buffer, int rx_buffer_size) {
  uint8_t byte_in_rx = 0;
  uint16_t total_byte = 0;
  uint16_t radian_frame_size_byte = ((size_byte * (8 + 3)) / 8) + 1;
  int tmo = 0;

  if ((int) (radian_frame_size_byte * 4) > rx_buffer_size) {
    ESP_LOGW(TAG, "RX buffer too small (%d) need >= %d", rx_buffer_size, radian_frame_size_byte * 4);
    return 0;
  }

  CC1101_CMD(SFRX);
  halRfWriteReg(MCSM1, 0x0F);
  halRfWriteReg(MDMCFG2, 0x02);
  halRfWriteReg(SYNC1, 0x55);
  halRfWriteReg(SYNC0, 0x50);
  halRfWriteReg(MDMCFG4, 0xF6);
  halRfWriteReg(MDMCFG3, 0x83);
  halRfWriteReg(PKTLEN, 1);
  cc1101_rec_mode_();

  while ((!this->gdo0_pin_->digital_read()) && (tmo < rx_tmo_ms)) {
    delay(1);
    tmo++;
  }
  if (tmo >= rx_tmo_ms) {
    ESP_LOGD(TAG, "RX stage1: GDO0 never asserted (tmo=%dms)", rx_tmo_ms);
    return 0;
  }

  while ((byte_in_rx == 0) && (tmo < rx_tmo_ms)) {
    delay(5);
    tmo += 5;
    byte_in_rx = (halRfReadReg(RXBYTES_ADDR) & RXBYTES_MASK);
    if (byte_in_rx) {
      SPIReadBurstReg(RX_FIFO_ADDR, &rx_buffer[0], byte_in_rx);
    }
  }
  if (tmo >= rx_tmo_ms || byte_in_rx == 0) {
    ESP_LOGD(TAG, "RX stage1: sync seen but no FIFO bytes (tmo=%dms)", rx_tmo_ms);
    return 0;
  }

  // Cache RSSI at first sync detection for scan scoring.
  this->last_rssi_dbm_ = this->rssi_to_dbm_(halRfReadReg(RSSI_ADDR));

  // Switch to "end of sync + start bit" and higher data rate.
  halRfWriteReg(SYNC1, 0xFF);
  halRfWriteReg(SYNC0, 0xF0);
  halRfWriteReg(MDMCFG4, 0xF8);
  halRfWriteReg(MDMCFG3, 0x83);
  halRfWriteReg(PKTCTRL0, 0x02);
  CC1101_CMD(SFRX);
  cc1101_rec_mode_();

  total_byte = 0;
  while ((!this->gdo0_pin_->digital_read()) && (tmo < rx_tmo_ms)) {
    delay(1);
    tmo++;
  }
  if (tmo >= rx_tmo_ms) {
    ESP_LOGD(TAG, "RX stage2: GDO0 never asserted (tmo=%dms)", rx_tmo_ms);
    return 0;
  }

  while ((total_byte < (uint16_t) (radian_frame_size_byte * 4)) && (tmo < rx_tmo_ms)) {
    delay(5);
    tmo += 5;
    byte_in_rx = (halRfReadReg(RXBYTES_ADDR) & RXBYTES_MASK);
    if (byte_in_rx) {
      SPIReadBurstReg(RX_FIFO_ADDR, &rx_buffer[total_byte], byte_in_rx);
      total_byte += byte_in_rx;
    }
  }
  if (tmo >= rx_tmo_ms || total_byte == 0) {
    ESP_LOGD(TAG, "RX stage2: no/insufficient FIFO bytes (got=%u, need~=%u, tmo=%dms)", total_byte,
             (uint16_t) (radian_frame_size_byte * 4), rx_tmo_ms);
    return 0;
  }

  CC1101_CMD(SFRX);
  CC1101_CMD(SIDLE);

  // Restore defaults.
  halRfWriteReg(MDMCFG4, 0xF6);
  halRfWriteReg(MDMCFG3, 0x83);
  halRfWriteReg(PKTCTRL0, 0x00);
  halRfWriteReg(PKTLEN, 38);
  halRfWriteReg(SYNC1, 0x55);
  halRfWriteReg(SYNC0, 0x00);

  return total_byte;
}

tmeter_data CC1101Component::get_meter_data(int stage1_tmo_ms, int stage2_tmo_ms, bool quiet) {
  tmeter_data data = {0};
  uint8_t txbuffer[100];
  memset(txbuffer, 0, sizeof(txbuffer));
  int tx_len = Make_Radian_Master_req(txbuffer, meter_year_, meter_serial_);
  if (tx_len != 39) {
    ESP_LOGW(TAG, "TX: unexpected request len=%d (expected 39)", tx_len);
  }
  if (!quiet) {
    ESP_LOGD(TAG, "TX: request len=%d (year=%u serial=%u)", tx_len, this->meter_year_, (unsigned) this->meter_serial_);
  }
  tx_wakeup_and_request_(txbuffer, tx_len, quiet);
  delay(43);
  uint8_t rxBuffer[1000];
  int rxBuffer_size = receive_radian_frame(0x12, stage1_tmo_ms, rxBuffer, sizeof(rxBuffer));
  if (!quiet && rxBuffer_size == 0) ESP_LOGW(TAG, "Timeout on first receive");
  delay(50);
  rxBuffer_size = receive_radian_frame(0x7C, stage2_tmo_ms, rxBuffer, sizeof(rxBuffer));
  if (rxBuffer_size) {
    if (!quiet) ESP_LOGD(TAG, "RX: raw bytes=%d", rxBuffer_size);
    uint8_t meter_data[200];
    memset(meter_data, 0, sizeof(meter_data));
    int meter_data_size = decode_4bitpbit_serial(rxBuffer, rxBuffer_size, meter_data);
    if (!quiet) ESP_LOGD(TAG, "RX: decoded bytes=%d", meter_data_size);
    data = parse_meter_report(meter_data, meter_data_size);
  } else {
    if (!quiet) ESP_LOGW(TAG, "Timeout on data receive");
  }
  return data;
}

void CC1101Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CC1101...");
  this->spi_setup();
  if (this->gdo0_pin_ != nullptr) this->gdo0_pin_->setup();
  this->radio_ready_ = true;
  cc1101_init(frequency_);
}

void CC1101Component::loop() {
  // No background loop work.
}

void CC1101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CC1101 Component");
  LOG_SENSOR("  ", "Liters", liters_sensor_);
  LOG_SENSOR("  ", "Counter", counter_sensor_);
  LOG_SENSOR("  ", "Battery", battery_sensor_);
  LOG_SENSOR("  ", "Time Start", time_start_sensor_);
  LOG_SENSOR("  ", "Time End", time_end_sensor_);
  LOG_SENSOR("  ", "RSSI", rssi_sensor_);
  LOG_SENSOR("  ", "Tuned Frequency", tuned_frequency_sensor_);
  LOG_TEXT_SENSOR("  ", "JSON", json_sensor_);
  LOG_TEXT_SENSOR("  ", "Timestamp", timestamp_sensor_);
}

void CC1101Component::update_meter_data() {
  // Fail-safe: only allow reads during the permitted local-time window.
  // Local time is determined by the configured ESPHome timezone.
  if (this->time_ == nullptr) {
    ESP_LOGW(TAG, "Read blocked: time source not configured");
    return;
  }

  auto local_time = this->time_->now();
  if (!local_time.is_valid()) {
    ESP_LOGW(TAG, "Read blocked: time not synced yet");
    return;
  }

  if (local_time.hour < 7 || local_time.hour > 17) {
    ESP_LOGW(TAG, "Read blocked: outside allowed hours (local time %02d:%02d, allowed 07:00-17:59)", local_time.hour,
             local_time.minute);
    return;
  }

  ESP_LOGI(TAG, "Meter read start (freq=%.6f MHz, year=%u, serial=%u)", this->frequency_, this->meter_year_, (unsigned) this->meter_serial_);
  cc1101_init(this->frequency_);
  meter_data_ = get_meter_data();
  ESP_LOGI(TAG, "Meter read result: liters=%d counter=%d battery(months)=%d window=%d-%d", meter_data_.liters, meter_data_.reads_counter, meter_data_.battery_left, meter_data_.time_start, meter_data_.time_end);
  if (this->rssi_sensor_ != nullptr) this->rssi_sensor_->publish_state(this->last_rssi_dbm_);
  if (liters_sensor_) liters_sensor_->publish_state(meter_data_.liters);
  if (counter_sensor_) counter_sensor_->publish_state(meter_data_.reads_counter);
  if (battery_sensor_) battery_sensor_->publish_state(std::min(static_cast<int>(meter_data_.battery_left * 100 / 120), 100));
  if (time_start_sensor_) time_start_sensor_->publish_state(meter_data_.time_start);
  if (time_end_sensor_) time_end_sensor_->publish_state(meter_data_.time_end);
  if (json_sensor_ && timestamp_sensor_) {
    auto current_time = this->time_->utcnow();
    char ts[32];
    sprintf(ts, "%04d-%02d-%02dT%02d:%02d:%02dZ", current_time.year, current_time.month, current_time.day_of_month, current_time.hour, current_time.minute, current_time.second);
    timestamp_str_ = ts;
    char json[256];
    sprintf(json, "{\"liters\":%d,\"counter\":%d,\"battery\":%d,\"time_start\":%d,\"time_end\":%d,\"timestamp\":\"%s\"}",
            meter_data_.liters, meter_data_.reads_counter, meter_data_.battery_left, meter_data_.time_start, meter_data_.time_end, ts);
    json_data_ = json;
    json_sensor_->publish_state(json_data_);
    timestamp_sensor_->publish_state(timestamp_str_);
  } else {
    ESP_LOGW(TAG, "JSON or Timestamp sensor not configured");
  }
  if (meter_data_.liters == 0 && meter_data_.reads_counter == 0) ESP_LOGW(TAG, "Failed to read meter");
}

}  // namespace cc1101_component
}  // namespace esphome