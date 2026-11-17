# OxiFOC Remote Stick

M5StickC Plus firmware with AXP192 power management, an ST7789 display and the
`oxifoc-remote` BLE link to `oxifoc-bridge`. It exists to compare the ESP32
radio with the nRF52840 remote on the same bridge. It has no throttle and does
not command a motor.
The display uses async SPI FIFO transfers at 20 MHz and a statically initialized
framebuffer. The 72 KiB heap includes 64 KiB reclaimed from the bootloader.

## Power button

Holding the Power button for four seconds requests hardware shutdown through
AXP192. Startup enables this behavior and selects the four-second threshold in
register `0x36`, preserving the other PEK settings. Both fields are read back
and checked before startup continues. Readback was confirmed on the connected
board; physical button-hold timing was not measured. Buttons A and B have no
assigned actions.

## Screen

The screen refreshes once per second:

- `LINK UP` or `SEARCHING`: whether an ergot session to the bridge is active.
- `Att` / `Fail`: connection attempts, and links that dropped before the GATT
  client came up (for example 0x3E, failed establishment).
- `Sess` / `Loss`: ergot sessions that became active, and sessions that ended
  with a link loss.
- `Recov`: time from the last link loss to the next active session, in ms.
- `Bat`: AXP192 voltage-based SoC estimate, battery voltage and `CHG`, `USB` or
  `BAT` status. SoC follows M5Unified's estimate and is not a fuel gauge.

The UART log also prints the AXP192 battery, charge, discharge and VBUS
currents every second.

## Hardware and dependency versions

- Target: ESP32, M5StickC Plus, 4 MB flash.
- `esp-hal`, `esp-radio` and the companion Espressif crates are pinned to
  `c72d6805fce0764bbe97d1e97a47896c6d4744bf` (upstream HEAD checked on 2026-11-15).
- `trouble-host` is pinned to `da44b34f67e1ee8efe977a5a975804b8b21962d4`, the
  same revision and features as `oxifoc-remote`.
- `oxifoc-core` comes from `../oxifoc` (path dependency) for the BLE UUIDs and
  the ICD.
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

## Bridge link

`src/link.rs` is a port of the BLE part of `oxifoc-remote/src/remote.rs`:

- BLE central with a static-random address derived from the factory MAC. It
  connects only to the bridge's fixed address.
- Connection interval 15–20 ms, latency 0, supervision timeout 1 s.
- A lost session reconnects immediately; the first three failed attempts retry
  at once, longer runs back off from 50 ms to 1 s.
- OxiFOC GATT service from `oxifoc_core::ble`, ergot `DirectEdge` over it,
  `HardwareInfo` and ergot device-info servers.

The bridge accepts one connection, so power off the nRF52840 remote during a
comparison. The ESP32 TX power is the controller default; this esp-radio
revision does not expose it.

### Comparison on 2026-11-17

Same bridge and placement, single CPU core.

| | nRF52840 remote | This stick |
|---|---|---|
| Bridge reset: link loss → session active | 287–312 ms, 0 failed establishments in 7 | 280–419 ms, median ≈319 ms |
| Power-on: connect start → session active | ≈490 ms, 0 failures in 10 | 300–380 ms, 0 failures in 8 |
| Failed establishments, all runs | 0 of 17 | 2 of 17 |

Only reconnects were measured, not range, hand blocking, RSSI or long runs.
The stick reaches its first attempt about 2.5 s after boot because of PMIC and
display initialization delays.
