#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// --- Audio sample rates ---
// Mic path is 16 kHz (server STT decodes 16 kHz Opus); TTS playback is 24 kHz.
#define AUDIO_INPUT_SAMPLE_RATE   16000
#define AUDIO_OUTPUT_SAMPLE_RATE  24000

// INMP441 mic and MAX98357A amp are separate I2S devices -> simplex mode.
#define AUDIO_I2S_METHOD_SIMPLEX

// INMP441 microphone (L/R -> GND = left channel). Pin map locked 2026-11-07.
#define AUDIO_I2S_MIC_GPIO_WS    GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK   GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN   GPIO_NUM_6

// MAX98357A amplifier.
#define AUDIO_I2S_SPK_GPIO_DOUT  GPIO_NUM_7    // amp DIN
#define AUDIO_I2S_SPK_GPIO_BCLK  GPIO_NUM_8
#define AUDIO_I2S_SPK_GPIO_LRCK  GPIO_NUM_9
#define AUDIO_CODEC_PA_PIN       GPIO_NUM_10   // amp SD: HIGH = enable output + select LEFT slot

// Onboard WS2812 RGB status LED (Waveshare ESP32-S3-Zero). Colour order is RGB,
// not the WS2812 default GRB -> handled by RgbSingleLed (see rgb_single_led.cc).
#define BUILTIN_LED_GPIO         GPIO_NUM_21

// Buttons (both wired to GND, internal pull-up, active low).
#define BOOT_BUTTON_GPIO         GPIO_NUM_0    // onboard BOOT: Wi-Fi config / toggle chat
#define ASSIST_BUTTON_GPIO       GPIO_NUM_18   // tactile button: push-to-talk; hold 5 s = restart
                                               // (final map, docs/wiring.md; was GPIO11 on the bench)
                                               // Also in config.json (CONFIG_BOOTLOADER_NUM_PIN_APP_TEST:
                                               // held 5 s at boot = recovery app) and recovery_app/.

// Shared I2C bus (DRV2605L haptics 0x5A + LIS3DH accelerometer 0x19). Pull-ups are on the
// breakouts, so the ESP32's internal pull-ups stay off. See docs/wiring.md "I2C bus".
#define SOFTTOY_I2C_SDA_GPIO     GPIO_NUM_15
#define SOFTTOY_I2C_SCL_GPIO     GPIO_NUM_16
#define SOFTTOY_I2C_FREQ_HZ      400000
#define SOFTTOY_I2C_ADDR_DRV2605L 0x5A
#define SOFTTOY_I2C_ADDR_LIS3DH   0x19

// Capacitive touch pads, copper tape through 470 ohm each (docs/wiring.md). On the ESP32-S3 the touch
// channel number equals the GPIO number. Thresholds and debounce live in touch_pads.cc.
#define SOFTTOY_TOUCH_GPIO_BELLY       1
#define SOFTTOY_TOUCH_GPIO_RIGHT_HAND  2
#define SOFTTOY_TOUCH_GPIO_LEFT_FOOT   11
#define SOFTTOY_TOUCH_GPIO_RIGHT_FOOT  12
#define SOFTTOY_TOUCH_GPIO_HEAD        13
#define SOFTTOY_TOUCH_GPIO_TAIL        14

// Boot haptics self-test: plays each effect once ~300 ms apart right after boot, so the owner
// can feel them without a trigger. ON for the integration run; set to 0 once checked.
#define SOFTTOY_HAPTICS_SELFTEST 0   // off for the final build (2026-11-13); its drive pulses ticked the speaker

// Mic-less fallback (2026-11-13, mic not working): every paw press (push-to-talk GPIO18 or the
// onboard BOOT button) sends [wake: button], even inside a session, so the server's
// button_only mode (control/birthday.txt) replays the birthday greeting each press. Plain
// push-to-talk is off. Set to 0 to restore the conversational behaviour once the mic works.
#define SOFTTOY_BUTTON_ONLY 1

#endif // _BOARD_CONFIG_H_
