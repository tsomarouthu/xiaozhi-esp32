#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "rgb_single_led.h"
#include "settings.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_system.h>

#define TAG "SoftToyS3Zero"

// SoftToy S3-Zero: Waveshare ESP32-S3-Zero (N8R8) + INMP441 mic + MAX98357A amp.
// No display. Simplex I2S (separate mic and amp buses), no external codec.
class SoftToyS3ZeroBoard : public WifiBoard {
private:
    Button boot_button_;
    Button assist_button_;

    // WiFi-liveness watchdog state (see StartWifiWatchdog).
    esp_timer_handle_t wifi_watchdog_timer_ = nullptr;
    int64_t last_wifi_ok_us_ = 0;
    static constexpr int64_t kWifiWatchdogCheckUs = 30LL * 1000000;      // check every 30 s
    static constexpr int64_t kWifiDownRebootUs    = 5LL * 60 * 1000000;  // reboot after 5 min down

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
        InitializeWakeVolume();
        StartWifiWatchdog();
    }

    // Wake-volume policy (2026-11-10): the toy always powers on quiet, and although the
    // child can raise it by voice within a session, it returns to the quiet default on
    // every wake/boot. So we force the NVS "output_volume" key that AudioCodec::Start()
    // reads (stock default 70) back to WAKE_VOLUME each boot. NoAudioCodec volume is
    // square-law: factor = (vol/100)^2, so dial 14 -> ~0.02 linear (the level settled in
    // voice_test). The quietest the child can go is dial 10 (~0.01); the louder ceiling is 60.
    //
    // Only the wake default lives here. The step size (+20 per "louder"), the quiet floor
    // (10) and the louder ceiling (60, above which the toy asks the child to have the
    // moderator raise it) live in the server prompt ("the Brain"), so the moderator can
    // change them without reflashing.
    static constexpr int WAKE_VOLUME = 14;   // 0-100 dial; square-law -> ~0.02 linear (voice_test level)
    void InitializeWakeVolume() {
        Settings settings("audio", true);
        settings.SetInt("output_volume", WAKE_VOLUME);
    }

    // Minimal WiFi-liveness watchdog. The stock app already returns to idle on a dropped
    // conversation and WiFi retries on its own, but nothing recovers a WiFi link that never
    // comes back: the toy would sit idle and unreachable with no one to power-cycle it. A
    // periodic timer checks STA association; if it stays disconnected for kWifiDownRebootUs
    // (5 min), it reboots to recover. It resets the clock during startup and SoftAP
    // provisioning (STA is legitimately not connected then), so it never reboots mid-setup.
    void StartWifiWatchdog() {
        last_wifi_ok_us_ = esp_timer_get_time();  // grace period from boot
        esp_timer_create_args_t args = {};
        args.callback = [](void* arg) {
            static_cast<SoftToyS3ZeroBoard*>(arg)->CheckWifiLiveness();
        };
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "wifi_wdt";
        if (esp_timer_create(&args, &wifi_watchdog_timer_) == ESP_OK) {
            esp_timer_start_periodic(wifi_watchdog_timer_, kWifiWatchdogCheckUs);
        }
    }

    void CheckWifiLiveness() {
        auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateStarting || state == kDeviceStateWifiConfiguring) {
            last_wifi_ok_us_ = esp_timer_get_time();  // don't count while booting/provisioning
            return;
        }
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            last_wifi_ok_us_ = esp_timer_get_time();  // associated -> healthy
            return;
        }
        int64_t down_us = esp_timer_get_time() - last_wifi_ok_us_;
        if (down_us >= kWifiDownRebootUs) {
            ESP_LOGW(TAG, "WiFi down for %lld s, rebooting to recover",
                     (long long)(down_us / 1000000));
            esp_restart();
        }
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
