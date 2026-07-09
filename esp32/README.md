# ESP32-C3 firmware has moved

The HealthyPi 5 ESP32-C3 (BLE / Wi-Fi "HealthyBridge") firmware now lives in its
own repository:

**https://github.com/Protocentral/healthybridge-esp32**

The RP2040 firmware in this repository streams to it over the HealthyBridge UART
link (BLE and Wi-Fi share the same RP2040-side code — the radio is chosen on the
ESP32). See the **Wireless** section of the top-level [`README`](../README.md) and
[`extras/docs/WIRELESS.md`](../extras/docs/WIRELESS.md).

The previous in-repo ESP32 sketch is preserved on the `v1-legacy` tag.
