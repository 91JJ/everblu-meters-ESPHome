# EverBlue / Cyble Water Meter (ESPHome + CC1101)

An ESPHome project that reads an EverBlue/Cyble-style wireless water meter using a CC1101 433 MHz transceiver and publishes the decoded values to Home Assistant. (Migrated from https://github.com/genestealer/everblu-meters-esp8266-improved)

<img width="483" height="443" alt="image" src="https://github.com/user-attachments/assets/6ce67af7-5e77-485a-9c3b-5fc93d8c9508" />


This repo includes:
- `water_meter.yaml`: the ESPHome configuration (entities, buttons, WiFi, etc.)
- `cc1101_component/`: a custom ESPHome external component implementing the CC1101 radio setup + meter request/receive/decoder

## What it does

- Sends the meter “wake-up” burst + request frame over 433 MHz
- Receives the meter response and decodes the oversampled serial bitstream
- Publishes meter values as sensors:
  - Total liters
  - Read counter
  - Battery estimate
  - Time window (start/end)
  - JSON summary + timestamp
- Publishes useful diagnostics:
  - CC1101 last RSSI (dBm)
  - CC1101 tuned frequency (MHz)
  - WiFi signal, uptime, and WiFi info

## Hardware

- ESP8266 NodeMCU (ESP-12E/12F) running ESPHome
- CC1101 transceiver module (433 MHz capable)


## ESPHome configuration

The main config is in `water_meter.yaml`.

Key points:
- The custom component is loaded via:
  ```yaml
  external_components:
    - source:
        type: local
        path: .
  ```
- The CC1101 component block:
  ```yaml
  cc1101_component:
    id: cc1101
    cs_pin: D8
    gdo0_pin:
      number: D1
      mode: INPUT_PULLUP
  ```

### Meter settings

Three template numbers are exposed in Home Assistant for configuration:
- `Frequency` (default `433.82 MHz`)
- `Meter Year` (default `12`)
- `Meter Serial / ID` (default `123456`)

A `Read Meter Now` button triggers a read on demand.

### Read schedule (UK local time)

Reads are permitted only between **07:00 and 17:59 UK local time**.

- `water_meter.yaml` uses SNTP with `timezone: Europe/London`.
- Automatic reads are controlled by `Refresh Rate` (hours):
  - `0` = disabled (default, to protect meter battery)
  - `1..168` = read every N hours (max 7 days), only during the allowed hours
- A fail-safe in the component blocks **any** read attempt outside the allowed hours (including manual button presses) and logs a warning.

Note: the published `Water Meter Timestamp` and the JSON `timestamp` are in **UTC** (with a trailing `Z`).

## Entities in Home Assistant

**Main sensors**
- `Water Meter Liters` (L)
- `Water Meter Counter` (reads)
- `Water Meter Battery` (%)
- `Water Meter Time Start` (h)
- `Water Meter Time End` (h)
- `Water Meter Timestamp`
- `Water Meter JSON`

**Config** (entity category: config)
- `1. Frequency`
- `2. Meter Serial / ID`
- `3. Meter Year`
- `4. Refresh Rate`
- `5. Read Meter Now`

**Diagnostics** (entity category: diagnostic)
- `CC1101 Last RSSI` (dBm)
- `CC1101 Tuned Frequency` (MHz)
- `Device Status`
- `WiFi Signal`
- `Uptime`
- `Device Info`
- `Reset Reason`
- `Heap Free`
- `Heap Fragmentation`
- `Heap Max Block`
- `Loop Time`
- `CPU Frequency`
- `Reboot Device`
- `IP Address`, `Connected SSID`, `Connected BSSID`, `MAC Address`

## Build 
### Wiring (NodeMCU → CC1101)

This project uses the ESPHome SPI bus and a CS pin:

- `D5` → `SCK`
- `D7` → `MOSI`
- `D6` → `MISO`
- `D8` → `CSN` (chip select)
- `D1` → `GDO0`
- `3V3` → `VCC`
- `GND` → `GND`

Notes:
- Use **3.3 V only** for the CC1101.
- Keep wires short (especially SCK/MOSI/MISO) for reliability.

### Config
1. Download the contents of the 'cc1101_component' folder and place it in your 'custom_components' for ESPHome.
2. Download the 'water_meter.yaml' file and add it to the root directory for ESPHome.
3. Configure the 'water_meter.yaml' file with your config.


## How it works (high level)

1. **CC1101 configuration**
   - Programs the CC1101 registers for the 2.4 kbps 2-FSK mode used by these meters.
   - Tunes to the configured frequency.

2. **Transmit request**
   - Sends a ~2s wake-up pattern (`0x55` bursts).
   - Sends a meter request frame derived from year + serial.

3. **Receive + decode**
   - Uses GDO0 (sync detect) to time reception.
   - Reads raw bytes from the RX FIFO.
   - Decodes the oversampled bitstream and extracts fields.

4. **Publish**
   - Updates sensors and publishes a JSON snapshot with a timestamp.

## Troubleshooting

- **All zeros / timeouts**
  - Double-check wiring (CS/SCK/MOSI/MISO/GDO0).
  - Confirm the frequency, meter year, and meter serial are correct.
  - Ensure the CC1101 is powered from 3.3 V.

- **Unstable reads**
  - Shorten SPI wires.
  - Move the antenna / module location (433 MHz is sensitive to placement).

- **Frequency looks like 433.820007**
  - Normal float rounding/printing on ESP8266.

## Development notes

- The ESPHome custom component lives in `cc1101_component/`.

## Disclaimer

This is a hobby project and depends on meter model/firmware and RF conditions. Use at your own risk.
