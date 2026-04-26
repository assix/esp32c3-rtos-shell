# esp32c3-rtos-shell

A shell-style firmware for ESP32-C3 built with ESP-IDF + FreeRTOS.

It provides:
- USB serial shell
- Telnet shell on port 23
- SPIFFS-backed file commands
- OLED dashboard + status LED
- Wi-Fi connection state machine
- Network tools (`ping`, `wget`, `ip`, `wifi`)
- Tiny Python-style runner (`python` / `py`)

## What This Is

This project is **not Linux**. It is an embedded firmware environment that offers Linux-like shell commands on top of FreeRTOS.

## Hardware

- ESP32-C3
- SSD1306 OLED (I2C)
- Status LED

Pin defaults are in [main/config.h.example](main/config.h.example).

## Quick Start

1. Install ESP-IDF (v6.x+ recommended).
2. Edit [main/config.h](main/config.h) with your Wi-Fi SSID/password and shell credentials.
3. Build and flash:

```bash
. /path/to/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Shell Commands

Core commands include:
- `help`, `uname`, `free`
- `pwd`, `ls`, `cd`, `mkdir`, `touch`, `cat`, `rm`, `write`, `append`, `df`
- `ip`, `wifi`, `ping`, `wget`
- `python`, `py`

## Python Runner Notes

The built-in Python runner is intentionally lightweight.
- Supports a subset of function-style calls (`print`, `sleep`, `ls`, `cat`, `mkdir`, `rm`, `touch`, `wget`, etc.).
- It is **not full CPython or full MicroPython**.

## Security Notes

- Telnet is plaintext; use on trusted LANs only.

## License

MIT. See [LICENSE](LICENSE).
