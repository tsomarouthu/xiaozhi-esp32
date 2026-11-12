# SoftToy S3-Zero board

Custom xiaozhi board for a talking soft toy: **Waveshare ESP32-S3-Zero (N8R8)** with an
**INMP441** I2S microphone and a **MAX98357A** I2S amplifier + speaker. No display.

Derived from `bread-compact-wifi` (same no-codec simplex-I2S audio), re-pinned and stripped of
the OLED. See `docs/board-build-plan.md` in the `softToy` repo for the full design rationale.

## Pin map (locked)

| Function | Pin | Notes |
|---|---|---|
| Mic WS (LRCL) | GPIO4 | INMP441 |
| Mic SCK (BCLK) | GPIO5 | |
| Mic DIN (DOUT/SD) | GPIO6 | L/R -> GND = left channel |
| Amp DIN | GPIO7 | MAX98357A |
| Amp BCLK | GPIO8 | |
| Amp LRC | GPIO9 | |
| Amp SD (enable) | GPIO10 | driven HIGH = enable + LEFT slot |
| Status LED | GPIO21 | onboard WS2812, **RGB** order |
| BOOT button | GPIO0 | Wi-Fi config / toggle chat |
| Push-to-talk button | GPIO18 | to GND, internal pull-up; hold 5 s = restart (was GPIO11 on the bench) |

Mic VDD -> 3V3, amp Vin -> 5V, amp GAIN -> NC (9 dB default).

## Board-specific details

- **RGB LED** — `RgbSingleLed` (local copy of `SingleLed`) sets the WS2812 colour order to
  `LED_STRIP_COLOR_COMPONENT_FMT_RGB`; the stock `SingleLed` uses GRB, which shows green as red
  on this board.
- **Amp enable** — `NoAudioCodecSimplex` has no PA-enable pin, so the board drives the MAX98357A
  `SD` pin (GPIO10) high at init.
- **`WIFI_PS_NONE`** — `SetPowerSaveLevel()` is overridden to always force `PERFORMANCE`
  (radio fully awake). Required: the USB power bank's low-current auto-shutoff otherwise kills
  the toy within ~1 minute (proven in `soak_test/`).

- **Offline mode & self-recovery** — when it can't reach Wi-Fi or the server, the toy *says*
  why (embedded clips in `sounds/<lang>/`, rendered by `offline_clips/` in the softToy repo) via
  the `GetAlertSound()` override, at most once per 2 min. Setup mode it fell into by itself (home
  Wi-Fi saved) is silent and reboots after 3 min with no phone connected (10 min if entered via
  BOOT). Holding push-to-talk 5 s turns the LED white; releasing restarts. Details:
  `docs/features/feature_offline-mode.md` in the softToy repo.

## Build

```bash
idf.py set-target esp32s3        # first time only
python scripts/build.py softtoy-s3zero --language en-US --wake-word nihaoxiaozhi
idf.py -p COM4 flash
```

`config.json` applies 8 MB flash, `partitions/v2/8m.csv` (two OTA slots), and octal PSRAM.
Firmware version is `PROJECT_VER` in the repo-root `CMakeLists.txt`; the OTA model key is
`board.type` = `softtoy-s3zero` (server looks for `data/bin/softtoy-s3zero_<version>.bin`).
