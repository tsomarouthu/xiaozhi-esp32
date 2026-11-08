#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "rgb_single_led.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <lvgl.h>
#include <esp_lvgl_port.h>

#define TAG "SoftToyS3Zero"

// SoftToy S3-Zero: Waveshare ESP32-S3-Zero (N8R8) + INMP441 mic + MAX98357A amp.
// No display. Simplex I2S (separate mic and amp buses), no external codec.
class SoftToyS3ZeroBoard : public WifiBoard {
private:
    Button boot_button_;
    Button assist_button_;

    // MAX98357A SD pin: drive HIGH to enable output (HIGH also selects the LEFT I2S slot).
    // NoAudioCodecSimplex has no PA-enable parameter, so the amp is enabled here and left on.
    void InitializeAmplifier() {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = 1ULL << AUDIO_CODEC_PA_PIN;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(AUDIO_CODEC_PA_PIN, 1);
    }

    // This board has no screen (GetDisplay() returns NoDisplay). The framework still
    // assumes LVGL is initialized: the assets loader allocates an LVGL font from the
    // assets partition (the same partition the wake-word models load from), and that
    // allocation crashes if LVGL's memory pool was never set up. Every other board
    // initializes LVGL inside its display driver; a headless board must do the minimal
    // equivalent here. No display is registered, so nothing is rendered — this just makes
    // lv_malloc() valid so the (unused) font allocates and is then harmlessly ignored.
    void InitializeLvglHeadless() {
        lv_init();
        lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
        port_cfg.task_priority = 1;
#if CONFIG_SOC_CPU_CORES_NUM > 1
        port_cfg.task_affinity = 1;
#endif
        lvgl_port_init(&port_cfg);
    }

    void InitializeButtons() {
        // Onboard BOOT button: enter Wi-Fi config on first boot, otherwise toggle chat.
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        // Tactile button on GPIO11: push-to-talk (hold to listen, release to send).
        assist_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        assist_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });
    }

public:
    SoftToyS3ZeroBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        assist_button_(ASSIST_BUTTON_GPIO) {
        InitializeLvglHeadless();
        InitializeAmplifier();
        InitializeButtons();
    }

    virtual Led* GetLed() override {
        static RgbSingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        static NoDisplay display;
        return &display;
    }

    // The toy is powered by a USB power bank whose low-current auto-shutoff kills the
    // device within ~1 minute when Wi-Fi modem-sleep is active (proven in soak_test/).
    // Force the radio fully awake (WIFI_PS_NONE) no matter what idle level the app asks
    // for, so the bank never auto-shuts off. Trades some idle current for reliability
    // (still gave >=18 h on a 5000 mAh bank).
    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        (void)level;
        WifiBoard::SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    }
};

DECLARE_BOARD(SoftToyS3ZeroBoard);
