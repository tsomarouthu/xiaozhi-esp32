#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "rgb_single_led.h"
#include "softtoy_i2c.h"
#include "haptics.h"
#include "lis3dh.h"
#include "markers.h"
#include "touch_pads.h"
#include "settings.h"
#include "assets/lang_config.h"
#include "ssid_manager.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <lvgl.h>
#include <esp_lvgl_port.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <atomic>
#include <cstring>
#include <string_view>

#define TAG "SoftToyS3Zero"

// Offline status clips, spoken in the toy's own voice with no server. Embedded from
// sounds/<lang>/ by main/CMakeLists.txt (BOARD_SOUNDS); rendered by offline_clips/ in the
// softToy repo. See docs/features/feature_offline-mode.md.
extern const char st_offline_wifi_start[] asm("_binary_st_offline_wifi_ogg_start");
extern const char st_offline_wifi_end[] asm("_binary_st_offline_wifi_ogg_end");
extern const char st_offline_server_start[] asm("_binary_st_offline_server_ogg_start");
extern const char st_offline_server_end[] asm("_binary_st_offline_server_ogg_end");
static const std::string_view OGG_ST_OFFLINE_WIFI{
    st_offline_wifi_start, static_cast<size_t>(st_offline_wifi_end - st_offline_wifi_start)};
static const std::string_view OGG_ST_OFFLINE_SERVER{
    st_offline_server_start, static_cast<size_t>(st_offline_server_end - st_offline_server_start)};

// Status LED plus the haptics state hook. xiaozhi calls Led::OnStateChanged() from the main loop
// on every device-state change (and on VAD changes while Listening), before the state handler
// starts or stops audio capture, so it is the board's state-change hook. We forward to the RGB LED,
// then drive the haptics from real state edges only (see Haptics for the mic gate):
//   -> Listening : tick, finished before capture starts
//   Listening -> : tick ~80 ms later, after capture has stopped
//   -> Speaking  : gentle purr until Speaking ends (cut at once if Listening starts)
class HapticStateLed : public Led {
public:
    explicit HapticStateLed(RgbSingleLed& led) : led_(led) {}
    void OnStateChanged() override {
        led_.OnStateChanged();
        DeviceState state = Application::GetInstance().GetDeviceState();
        if (state == prev_) return;   // VAD-only call: no edge
        auto& haptics = Haptics::GetInstance();
        if (state == kDeviceStateListening) {
            haptics.Purr(false);
            haptics.BuzzListenStart();
        } else if (prev_ == kDeviceStateListening) {
            haptics.BuzzListenEnd();
        }
        if (state == kDeviceStateSpeaking) {
            haptics.Purr(true);
        } else if (prev_ == kDeviceStateSpeaking) {
            haptics.Purr(false);
        }
        Markers::NoteDeviceState(state);   // for "no pick-up within 10 min after a session"
        prev_ = state;
    }
private:
    RgbSingleLed& led_;
    DeviceState prev_ = kDeviceStateUnknown;
};

// SoftToy S3-Zero: Waveshare ESP32-S3-Zero (N8R8) + INMP441 mic + MAX98357A amp.
// No display. Simplex I2S (separate mic and amp buses), no external codec.
class SoftToyS3ZeroBoard : public WifiBoard {
private:
    // Hold push-to-talk this long to restart the toy (the child's own "fix it" gesture).
    static constexpr uint16_t kRestartHoldMs = 5000;

    Button boot_button_;
    Button assist_button_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;   // shared bus; also available as SoftToyI2c::Bus()

    // WiFi-liveness watchdog state (see StartWifiWatchdog).
    esp_timer_handle_t wifi_watchdog_timer_ = nullptr;
    int64_t last_wifi_ok_us_ = 0;
    static constexpr int64_t kWifiWatchdogCheckUs = 30LL * 1000000;      // check every 30 s
    static constexpr int64_t kWifiDownRebootUs    = 5LL * 60 * 1000000;  // reboot after 5 min down

    // Setup-mode timeout (see CheckWifiLiveness). The stock firmware falls into the SoftAP
    // setup mode when saved Wi-Fi isn't found within 60 s of boot, and then stays there for
    // good; out of range (car, garden) that means a toy that never comes back by itself.
    int64_t config_idle_since_us_ = 0;   // 0 = not in setup mode
    static constexpr int64_t kAutoConfigRebootUs = 3LL * 60 * 1000000;    // fell in by itself
    static constexpr int64_t kUserConfigRebootUs = 10LL * 60 * 1000000;   // parent pressed BOOT
    std::atomic<bool> config_requested_{false};  // setup mode was asked for via BOOT

    // Push-to-talk state.
    std::atomic<bool> restart_armed_{false};     // held >= kRestartHoldMs: restart on release
    std::atomic<bool> ptt_swallowed_{false};     // press answered with a clip, not a turn
    std::atomic<bool> ptt_marker_started_{false};  // press began a [wake: button] session: swallow the release

    // Server-down memory for Markers' "online?" check. The server is only ever noticed as
    // unreachable when a talk attempt or the OTA check fails; that alert (see GetAlertSound) sets
    // this, and for the next kServerDownMemoryUs start-a-session markers count as offline.
    std::atomic<int64_t> server_down_until_us_{0};
    static constexpr int64_t kServerDownMemoryUs = 60LL * 1000000;

    // Offline clip rate limit: speak at most once per window, the short beep in between,
    // so a child pressing repeatedly isn't lectured every time.
    std::atomic<int64_t> last_offline_clip_us_{0};
    static constexpr int64_t kOfflineClipRepeatUs = 2LL * 60 * 1000000;

    static bool HasSavedWifi() {
        return !SsidManager::GetInstance().GetSsidList().empty();
    }

    static bool WifiAssociated() {
        wifi_ap_record_t ap_info;
        return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
    }

    static bool StartsWith(const char* text, const char* prefix) {
        return text != nullptr && strncmp(text, prefix, strlen(prefix)) == 0;
    }

    // Setup mode that the toy fell into on its own (home Wi-Fi not found), as opposed to
    // a parent deliberately setting up a network or a brand-new toy with none saved.
    bool InAutomaticSetupMode() const {
        auto state = Application::GetInstance().GetDeviceState();
        return (state == kDeviceStateWifiConfiguring || state == kDeviceStateAudioTesting) &&
               !config_requested_ && HasSavedWifi();
    }

    // Which clip explains why the toy can't talk: Wi-Fi itself, or the server behind it.
    static std::string_view OfflineClip() {
        return WifiAssociated() ? OGG_ST_OFFLINE_SERVER : OGG_ST_OFFLINE_WIFI;
    }

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

    // "Can the toy reach the server right now?" for Markers (offline -> M6's local clips).
    // Cheapest reliable check that already exists: Wi-Fi associated (as the offline status clips
    // use) and no recent server-unreachable alert. In a live session (Listening) it is obviously up.
    bool IsOnline() {
        if (Application::GetInstance().GetDeviceState() == kDeviceStateListening) return true;
        if (!WifiAssociated()) return false;
        return esp_timer_get_time() >= server_down_until_us_.load();
    }

    // One shared I2C bus (GPIO15/16) for the DRV2605L and the LIS3DH. Logs a bus scan.
    // A missing device only disables that feature; it never blocks boot.
    void InitializeI2cAndHaptics() {
        i2c_bus_ = SoftToyI2c::Init();
        Haptics::GetInstance().Init(i2c_bus_);
        Markers::Init([this]() { return IsOnline(); });
        Lis3dh::GetInstance().Init(i2c_bus_);
        TouchPads::Init();   // M4: needs Haptics + Markers; a failure only disables touch
    }

    void InitializeButtons() {
        // Onboard BOOT button: enter Wi-Fi config on first boot, otherwise toggle chat.
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                config_requested_ = true;   // a person asked for setup: longer timeout, chime on
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        // Tactile push-to-talk button (GPIO18): press = listen, release = send.
        // Held for kRestartHoldMs: the LED turns white ("let go now") and releasing restarts
        // the toy. Restart happens on *release*, so a stuck or shorted button can't boot-loop.
        assist_button_.OnPressDown([this]() {
            auto& app = Application::GetInstance();
            if (InAutomaticSetupMode()) {
                // Stock behaviour here is the mic echo test, which looks like "working" to a
                // child but can't talk. Explain instead; hold-to-restart still works.
                ptt_swallowed_ = true;
                app.Schedule([]() { Application::GetInstance().PlaySound(OGG_ST_OFFLINE_WIFI); });
                return;
            }
            // No session open: send [wake: button] and start one (the toy speaks first, then the
            // conversation continues hands-free) and swallow the matching release. Session already
            // open / speaking / anything else: the app rejects it and the press is plain
            // push-to-talk as before. The decision runs on the main task; the release below is
            // scheduled too, so it is always ordered after it.
            Markers::Request(Markers::Id::kWakeButton, [this](bool accepted) {
                if (accepted) {
                    ptt_marker_started_ = true;
                } else {
                    Application::GetInstance().StartListening();
                }
            });
        });
        assist_button_.OnLongPress([this]() {
            ESP_LOGW(TAG, "Push-to-talk held %u ms: release to restart", kRestartHoldMs);
            restart_armed_ = true;
            StatusLed().SetOverrideColor(16, 16, 16);   // white = let go now
        });
        assist_button_.OnPressUp([this]() {
            if (restart_armed_) {
                ESP_LOGW(TAG, "Restart requested by push-to-talk hold");
                esp_restart();
            }
            if (ptt_swallowed_.exchange(false)) {
                return;
            }
            Application::GetInstance().Schedule([this]() {
                if (ptt_marker_started_.exchange(false)) {
                    return;   // the press started a [wake: button] session: nothing to stop
                }
                Application::GetInstance().StopListening();
            });
        });
    }

    static RgbSingleLed& StatusLed() {
        static RgbSingleLed led(BUILTIN_LED_GPIO);
        return led;
    }

public:
    SoftToyS3ZeroBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        assist_button_(ASSIST_BUTTON_GPIO, false, kRestartHoldMs) {
        InitializeLvglHeadless();
        InitializeI2cAndHaptics();
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
    // provisioning (STA is legitimately not connected then).
    //
    // Setup mode gets its own timeout: if a home network is saved and no phone is connected
    // to the setup hotspot, reboot after kAutoConfigRebootUs (3 min) so the toy goes back to
    // looking for home Wi-Fi; kUserConfigRebootUs (10 min) if a parent asked for setup via
    // BOOT. A phone on the hotspot holds it open; with no saved network it never times out.
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
        int64_t now = esp_timer_get_time();
        if (state == kDeviceStateWifiConfiguring || state == kDeviceStateAudioTesting) {
            last_wifi_ok_us_ = now;  // the 5-min Wi-Fi-down rule doesn't apply in setup mode
            CheckSetupModeTimeout(now);
            return;
        }
        config_idle_since_us_ = 0;
        if (state == kDeviceStateStarting) {
            last_wifi_ok_us_ = now;  // don't count while booting
            return;
        }
        if (WifiAssociated()) {
            last_wifi_ok_us_ = now;  // associated -> healthy
            return;
        }
        int64_t down_us = now - last_wifi_ok_us_;
        if (down_us >= kWifiDownRebootUs) {
            ESP_LOGW(TAG, "WiFi down for %lld s, rebooting to recover",
                     (long long)(down_us / 1000000));
            esp_restart();
        }
    }

    void CheckSetupModeTimeout(int64_t now) {
        wifi_sta_list_t stations = {};
        bool phone_connected = esp_wifi_ap_get_sta_list(&stations) == ESP_OK && stations.num > 0;
        if (!HasSavedWifi() || phone_connected || config_idle_since_us_ == 0) {
            config_idle_since_us_ = now;  // nothing to go back to, someone is setting up, or just entered
            return;
        }
        int64_t limit_us = config_requested_ ? kUserConfigRebootUs : kAutoConfigRebootUs;
        int64_t idle_us = now - config_idle_since_us_;
        if (idle_us >= limit_us) {
            ESP_LOGW(TAG, "Setup mode idle for %lld s with home Wi-Fi saved, rebooting to retry it",
                     (long long)(idle_us / 1000000));
            esp_restart();
        }
    }

    virtual Led* GetLed() override {
        static HapticStateLed led(StatusLed());
        return &led;
    }

    // Headless toy: say *why* it can't talk instead of the generic exclamation beep, and keep
    // the setup-mode chime quiet when the toy only fell into setup mode by itself (otherwise
    // it would chime every ~4 min on a car trip). Everything else keeps its stock sound.
    virtual std::string_view GetAlertSound(const char* status, const char* message,
                                           const char* emotion, std::string_view sound) override {
        (void)status;
        if (strcmp(emotion, "gear") == 0) {   // "Wi-Fi configuration mode" alert
            return (!config_requested_ && HasSavedWifi()) ? std::string_view{} : sound;
        }
        bool offline =
            strcmp(emotion, "cloud_off") == 0 ||   // OTA/version check failed: server unreachable
            (strcmp(emotion, "cancel") == 0 &&     // talk attempt failed to reach the server
             (StartsWith(message, Lang::Strings::SERVER_NOT_CONNECTED) ||
              StartsWith(message, Lang::Strings::SERVER_TIMEOUT) ||
              StartsWith(message, Lang::Strings::SERVER_ERROR)));
        if (!offline) {
            return sound;   // e.g. firmware upgrade failed: keep the stock beep
        }
        int64_t now = esp_timer_get_time();
        if (WifiAssociated()) {
            server_down_until_us_ = now + kServerDownMemoryUs;   // Wi-Fi is up, so it's the server
        }
        int64_t last = last_offline_clip_us_;
        if (last != 0 && now - last < kOfflineClipRepeatUs) {
            return sound;   // said it recently: just the beep
        }
        last_offline_clip_us_ = now;
        return OfflineClip();
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
