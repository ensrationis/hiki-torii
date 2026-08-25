# hiki-torii

E-ink status gateway for the [Hiki](https://openclaw.ai) home assistant — an ESP32-C6
driving a 4.2″ e-paper panel that shows air quality and the state of the home node
at a glance, without a screen to unlock or an app to open.

![Hiki](hiki-ava.png)

## What it does

- Reads **CO₂, temperature and humidity** from a Sensirion SCD4x over I²C
- Publishes them to MQTT and announces itself through **Home Assistant MQTT discovery**,
  so the sensors appear in HA with no manual configuration
- Subscribes to the Hiki node's own topics and renders its health, gateway status and
  killswitch state — including a **QR code of the killswitch address** and the current
  block number
- Switches to a dedicated **isolated screen** when the killswitch reports that state
- Three buttons page between the home screen and two detail screens, which time out
  back to home on their own

## Hardware

| Part | Notes |
|---|---|
| ESP32-C6 | built as `esp32-c6-devkitc-1` |
| 4.2″ e-paper, 400×300 | Waveshare EPD_4in2 V2; driver vendored in `lib/epd` |
| Sensirion SCD4x | CO₂ / temperature / humidity, I²C |
| 3 push buttons | UP, SET, DOWN |

### Pinout

| Signal | GPIO |
|---|---|
| I²C SDA | 19 |
| I²C SCL | 18 |
| Button UP | 10 |
| Button SET | 2 |
| Button DOWN | 3 |

E-paper SPI pins are defined in `lib/epd/DEV_Config.h`.

## Build and flash

Built with [PlatformIO](https://platformio.org/). The platform is pinned to a
[pioarduino](https://github.com/pioarduino/platform-espressif32) release, since
ESP32-C6 needs a newer Arduino core than upstream `platform-espressif32` ships.

```bash
cp src/config.h.example src/config.h
$EDITOR src/config.h          # Wi-Fi, MQTT broker, device id

pio run                        # build
pio run -t upload              # flash
pio device monitor             # serial log, 115200
```

`src/config.h` is gitignored — the example file is the template:

```c
#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"
#define MQTT_SERVER   "192.168.1.100"
#define MQTT_PORT     1883
#define DEVICE_ID     "torii-ink"
```

Dependencies are resolved by PlatformIO from `platformio.ini`: SparkFun SCD4x,
PubSubClient and ricmoo/QRCode.

## MQTT

Published, retained discovery configs plus live state:

| Topic | Payload |
|---|---|
| `<DEVICE_ID>/sensor/co2` | ppm |
| `<DEVICE_ID>/sensor/temperature` | °C |
| `<DEVICE_ID>/sensor/humidity` | % |

Subscribed:

| Topic | Used for |
|---|---|
| `hiki/health` | node health shown on the home screen |
| `hiki/killswitch/status` | killswitch state, address, block number, chain connectivity |
| `hiki/gateway/health` | gateway indicator |

## Timings

| Interval | Value |
|---|---|
| Sensor read and publish | 120 s |
| Home screen re-render | 60 s |
| Full e-ink waveform refresh | every 5 screen transitions |
| Detail screen auto-return | 25 s |

Partial refreshes are used between full ones to keep the panel responsive; the
periodic full refresh is what clears e-paper ghosting. A task watchdog resets the
device if the main loop stalls.

## Avatar bitmap

`tools/png_to_bitmap.py` converts a PNG into the packed C header in
`src/hiki_bitmaps.h` (MSB-first, 1 = white). Pillow is the only dependency:

```bash
python3 tools/png_to_bitmap.py hiki-ava.png
```

The `tools/preview_*.png` files are dithering previews for comparing conversion
settings before committing a new avatar.

## License

[MIT](LICENSE) © 2026 Sergei Lonshakov
