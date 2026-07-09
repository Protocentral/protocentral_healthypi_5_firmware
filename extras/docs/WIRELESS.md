# Wireless on the HealthyPi 5 (BLE & Wi-Fi)

> **TL;DR — where is the WiFi/BLE code?** Not on the RP2040. The RP2040 has **no
> radio**. Wireless is done by the on-board **ESP32-C3 co-processor**, which runs
> a separate firmware. The RP2040 just streams data to it over a UART
> ("HealthyBridge"). This doc explains the two-chip model and how to bring it up.

## Why there is no "BLE sketch" for the RP2040

The HealthyPi 5's main MCU is a **plain RP2040** — not a Pico W — so it has no
Bluetooth and no Wi-Fi silicon. You cannot call `BLEDevice.begin()` here; there
is nothing for it to talk to.

Instead the board carries an **ESP32-C3**, which has *both* BLE and Wi-Fi radios.
It runs its own firmware and does all the wireless work. The two chips are wired
together on the PCB by a single UART.

```
  ┌──────────────────┐   HealthyBridge     ┌──────────────────┐
  │      RP2040       │   UART1 @ 921600    │     ESP32-C3     │
  │  main MCU         │  ───AA55|…|CRC───►  │  wireless co-MCU │
  │  • sensors (SPI)  │   GP24 TX / 25 RX   │  • BLE GATT      │ ──BLE──►  📱 app
  │  • DSP / vitals   │   GP27 RTS/26 CTS   │  • Wi-Fi AP/STA  │ ──WiFi─►  🖥 browser
  │  • USB / SD / LCD │  ◄──status/cmds───  │                  │
  └──────────────────┘                     └──────────────────┘
     healthypi5_next_arduino  (this repo)     healthybridge-esp32  (separate)
```

**BLE and Wi-Fi are identical on the RP2040 side.** The same HealthyBridge frames
go out UART1 either way; which radio carries them is decided entirely on the
ESP32 (and by the host that connects — a phone over BLE, a browser over Wi-Fi).
That is why this repo has **one** wireless example, not two.

## The two pieces you flash

| Chip | Firmware | Role |
|---|---|---|
| **RP2040** | this repo (`healthypi5_next_arduino`) | acquire + stream over HealthyBridge |
| **ESP32-C3** | [`Protocentral/healthybridge-esp32`](https://github.com/Protocentral/healthybridge-esp32) | own the BLE/Wi-Fi radios, relay the stream |

You do **not** re-implement BLE/Wi-Fi in this repo. Reuse the ESP32 firmware
unchanged — the HealthyBridge frame format is shared verbatim by both sides so
they cannot drift.

## The HealthyBridge link

- **Transport:** UART1 (arduino-pico `Serial2`), **921600 8N1, RTS/CTS**.
- **Pins** (from `board.h`, already routed on the PCB — nothing to wire):

  | Signal | RP2040 GPIO | ESP32-C3 side |
  |---|---|---|
  | TX  | GP24 | RX |
  | RX  | GP25 | TX |
  | RTS | GP27 | CTS |
  | CTS | GP26 | RTS |

- **Frame** (little-endian):
  `SYNC(0xAA55) | TYPE(1) | FLAGS(1) | LENGTH(2) | SEQ(2) | PAYLOAD | CRC16(2)`,
  CRC-16/CCITT (poly 0x1021, init 0xFFFF) over `TYPE..PAYLOAD`.
- **Directions:** RP2040 → ESP32 sends `BIOSIG` (waveforms), `VITALS`, `BATTERY`.
  ESP32 → RP2040 sends `STATUS` (BLE/Wi-Fi up) and forwards phone commands.

The canonical implementation is in the `HealthyPi5` library:
[`HPIBridge.h`](../libraries/HealthyPi5/src/HPIBridge.h) /
[`HPIBridge.cpp`](../libraries/HealthyPi5/src/HPIBridge.cpp).

## Where it's used in this repo

| Where | What it does |
|---|---|
| [`examples/Tutorials/10_Wireless_Bridge`](../examples/Tutorials/10_Wireless_Bridge) | standalone teaching sketch: sends `VITALS` frames, mirrors them to USB. Read this to learn the wire format. |
| [`examples/Applications/HealthyPi5_NEXT`](../examples/Applications/HealthyPi5_NEXT) | full firmware: the library's `BridgeSink` streams biosignals + vitals + battery and parses status, drop-newest, off the dual-core spine. Hardware-validated with BLE **and** Wi-Fi. |

## Bringing it up

1. Flash this repo's firmware to the RP2040 (`./scripts/upload.sh next`, or the
   teaching demo `./scripts/upload.sh wireless --monitor`).
2. Flash [`healthybridge-esp32`](https://github.com/Protocentral/healthybridge-esp32)
   to the on-board ESP32-C3 (see that repo's instructions).
3. **BLE:** open the HealthyPi mobile app (or any BLE scanner) and connect to the
   advertised device to see live vitals/waveforms.
4. **Wi-Fi:** follow the ESP32 firmware's Wi-Fi/SoftAP instructions to view the
   stream in a browser.

## Troubleshooting

- **`hbtx`/frames = 0, drops climbing** (production `HPI_INSTR` line, or the
  teaching sketch's `[frames= drops=]`): the ESP32 isn't draining the UART — it's
  unflashed, held in reset, or CTS is deasserted. The RP2040 side deliberately
  **drops** rather than blocking, so acquisition never stalls.
- **Bench-testing without an ESP32:** in `10_Wireless_Bridge` set
  `USE_FLOW_CONTROL 0` and sniff GP24 with a logic analyser / 921600 USB-UART
  adapter to confirm the `AA 55 …` framing.
- **Brownout on Wi-Fi start:** the ESP32's Wi-Fi SoftAP draws a current spike on
  the shared rail; a marginal supply can reset the board when Wi-Fi comes up.
  Power from a solid 5 V source.
