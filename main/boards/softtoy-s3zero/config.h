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

#endif // _BOARD_CONFIG_H_
