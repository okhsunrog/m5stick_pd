# OxiFOC Remote Stick

M5StickC Plus firmware with AXP192 power management, an ST7789 display and a
diagnostic BLE peripheral. The diagnostic service does not control a motor.
The display uses async SPI FIFO transfers at 20 MHz and a statically initialized
framebuffer. The 72 KiB heap includes 64 KiB reclaimed from the bootloader.

## Hardware and dependency versions

- Target: ESP32, M5StickC Plus, 4 MB flash.
- `esp-hal`, `esp-radio` and the companion Espressif crates are pinned to
  `c72d6805fce0764bbe97d1e97a47896c6d4744bf` (upstream HEAD checked on 2026-11-15).
- `trouble-host` is pinned to `042bd1c3bdc5980f66d28e0e698b6e579b15da4e`.
- `lcd-async` is pinned to `ab6b71d6`.

**Experimental ESP32 v1.1 opt-in:** current upstream `esp-hal` supports ESP32
v3.0 and newer. `.cargo/config.toml` explicitly lowers
`ESP_HAL_CONFIG_MIN_CHIP_REVISION` to `101` for the connected v1.1 device.
This removes the revision check; it does not restore upstream support or
silicon errata workarounds. Light sleep, BLE modem sleep and the second CPU
are not enabled by this application.

## Build and run

Install the Espressif Rust toolchain named `esp` and `espflash`, then:

```sh
cargo build --release --locked
cargo run --release --locked
```

The Cargo runner flashes the firmware and opens a defmt serial monitor. Specify
`ESPFLASH_PORT` when several serial devices are attached.

## BLE diagnostics

The device advertises as `OxiFOC Remote Stick` with a static-random address
derived from its factory MAC. It accepts one connection. Pairing and encryption
are not configured.

Service UUID: `c5a10000-7d3e-4f71-9a52-3b64e80f2401`.

| Characteristic UUID | Properties | Value |
| --- | --- | --- |
| `c5a10001-7d3e-4f71-9a52-3b64e80f2401` | Read, write, notify | Four-byte little-endian echo register; notifies after a successful write |
| `c5a10002-7d3e-4f71-9a52-3b64e80f2401` | Read, notify | Four-byte little-endian uptime in seconds; updates every second while connected |

On a Linux computer with BlueZ, a powered Bluetooth adapter and `uv`:

```sh
uv run scripts/ble_smoke.py
```

The script discovers the physical device, validates service properties and its
GAP name, checks echo reads/writes/notifications and rejection of a malformed
write, observes increasing uptime notifications, and disconnects/reconnects
over three connection cycles. It emits JSON lines and exits unsuccessfully on any failed check.
Use `--address` to select a device, and `--cycles`, `--writes`, `--seconds` to
adjust the test duration.

The test verifies BLE diagnostics on the selected hardware. It does not qualify
this unsupported silicon revision, motor control, pairing, encryption, sleep,
power consumption or the display's visual appearance.

### Physical test on 2026-11-15

The connected ESP32 v1.1 passed three connection cycles in 66.19 seconds:
60 echo writes with matching notifications, readback checks, 54 increasing uptime
notifications, and six rejected writes with invalid lengths (1 and 5 bytes).
After disconnecting, advertising resumed. The final UART capture contained no
panics or error logs. Power management and display initialization completed;
the display's appearance was not visually checked.
