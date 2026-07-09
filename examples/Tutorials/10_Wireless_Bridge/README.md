# 10 — Wireless via the ESP32-C3 (HealthyBridge)

**The one thing to understand:** the RP2040 on the HealthyPi 5 has **no radio.**
It is a plain RP2040 (not a Pico W), so there is no BLE or Wi-Fi on this chip.
All wireless is done by the on-board **ESP32-C3 co-processor**, which runs its
own firmware and owns the entire BLE/Wi-Fi stack.

So this sketch **never calls a BLE API.** Its whole job is to hand vitals to the
ESP32 over a wire — the **HealthyBridge** UART link — and let the ESP32 relay
them to a phone (BLE) or a browser (Wi-Fi). BLE and Wi-Fi are the *same* on the
RP2040 side: identical frames go out UART1; the radio is chosen on the ESP32.
That is why there is **one** wireless example, not a "BLE" and a "Wi-Fi" one.

```
  ┌─────────────┐   HealthyBridge      ┌────────────┐    BLE / Wi-Fi     📱 / 🖥
  │   RP2040    │   UART1 @ 921600     │  ESP32-C3  │  ───────────────►  phone /
  │ (this .ino) │ ───AA55|…|CRC16───►  │ (own f/w)  │                    browser
  └─────────────┘   GP24/25 +RTS/CTS   └────────────┘
   acquires vitals    frames on a wire    owns the radios
```

## What it does

- Brings up the three sensors (same as example 07): HR (MAX30001), SpO2
  (AFE4400), temperature (MAX30205).
- Once per second, packs them into a **HealthyBridge `VITALS` frame** and writes
  it to `Serial2` (UART1) — byte-for-byte what the ESP32-C3 firmware expects.
- Mirrors each transmission to the USB Serial Monitor (115200) so you can watch
  the link — including a dropped-frame counter — even with no ESP32 attached.

The frame is built inline so you can read the whole packet in one file:
`SYNC(0xAA55) | TYPE | FLAGS | LENGTH | SEQ | PAYLOAD | CRC16`, CRC-16/CCITT over
`TYPE..PAYLOAD`. The **canonical** definition (all frame types, the batched
biosignal stream, RX/status parsing, drop-newest transport) lives in the
`HealthyPi5` library — [`HPIBridge.h`](../../../libraries/HealthyPi5/src/HPIBridge.h) /
`HPIBridge.cpp` — which the production firmware
[`../../Applications/HealthyPi5_NEXT`](../../Applications/HealthyPi5_NEXT) uses.

## The UART1 wiring (RP2040 ↔ ESP32-C3, on-board)

| Signal | RP2040 GPIO | Direction |
|---|---|---|
| TX  | 24 | RP2040 → ESP32 |
| RX  | 25 | RP2040 ← ESP32 |
| RTS | 27 | flow control |
| CTS | 26 | flow control |

Baud is **921600, 8N1, RTS/CTS**. These pins come from `board.h`
(`HPI_PIN_UART1_*`) and are already routed on the board — you don't wire
anything; the two MCUs are connected on the PCB.

## Seeing the data wirelessly

The RP2040 side is only half the system. To get vitals onto a phone or browser,
flash the **companion ESP32-C3 firmware** to the on-board ESP32:

- **Firmware:** [`Protocentral/healthybridge-esp32`](https://github.com/Protocentral/healthybridge-esp32)
- **BLE:** connect with the HealthyPi mobile app (or any BLE scanner) to see the
  advertised vitals service.
- **Wi-Fi:** the ESP32 exposes the stream over its Wi-Fi path (SoftAP / browser).

See [`docs/WIRELESS.md`](../../../docs/WIRELESS.md) for the full two-chip picture,
how to flash the ESP32, and how BLE vs Wi-Fi are selected.

## Bench-testing without an ESP32

Set `USE_FLOW_CONTROL 0` at the top of the sketch. Without CTS gating, frames
still go out **GP24** (Serial2 TX) so you can capture them with a logic analyser
or a 921600-baud USB-UART adapter and confirm the `AA 55 …` framing + CRC. With
the ESP32 present, keep it `1`.

## Build / flash

```bash
./scripts/upload.sh wireless --monitor     # build + flash + open the USB console
./scripts/build.sh  wireless               # build only
```

## Next step

This sends only the ~1 Hz `VITALS` frame — the clearest thing to learn from. The
library's `BridgeSink` additionally batches the **biosignal waveform** into
`BIOSIG` frames and parses **status** frames coming back from the ESP32 (BLE
connected, Wi-Fi up). See `HPIBridge.cpp` and the production firmware for the
full-duplex version.
