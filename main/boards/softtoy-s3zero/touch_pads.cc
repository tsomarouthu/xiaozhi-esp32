#include "touch_pads.h"
#include "haptics.h"
#include "markers.h"
#include "config.h"

#include <driver/touch_sens.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <mutex>

#define TAG "Touch"

// One log line per pad every 2 s: raw (smoothed) reading, baseline, delta, press threshold and state.
// This is the calibration tool (docs/ONBOARD-CHECKLIST.md, M4). Set to 0 once the pads are tuned.
#ifndef SOFTTOY_TOUCH_CALIB_LOG
#define SOFTTOY_TOUCH_CALIB_LOG 0    // off for the final build (2026-11-13); pads calibrated in the stuffed toy
#endif

// ============================ TUNING: every touch constant lives here ============================
// BENCH VALUES. The pads are loose copper tape on the bench, powered from USB. Capacitance depends
// on the fabric, the stuffing, the pad area and the supply, so RE-CALIBRATE after sewing the pads
// into the plush, with the toy on the power bank (a USB-powered and a bank-powered toy read
// differently). Procedure: ONBOARD-CHECKLIST.md "M4 calibration".
//
// On the ESP32-S3 a touch RAISES the reading. "delta" = smooth - baseline. A pad is pressed when
// delta >= max(baseline * press_permille / 1000, kMinAbsDelta), released when delta falls below
// kReleaseFracPct percent of that (hysteresis).
namespace cfg {
struct PadCfg {
    int gpio;                 // = touch channel on the S3
    const char* name;         // log name = marker name
    Markers::Id marker;       // kCount = none (belly)
    int press_permille;       // press threshold as 1/1000 of the baseline (30 = 3.0 %)
    bool giggle;              // feet: giggle buzz instead of the plain buzz
    bool buzz;                // local buzz-back on press (belly: none, M5 purrs on a hug)
};
// Order is only for logs. Start with 3 % everywhere; large pads (belly) usually move more, so
// the same percentage is easier to hit there. Raise a pad that false-triggers, lower one that
// needs a hard press.
// Bench, board 2, before stuffing (2026-11-13): neighbour coupling ~3-4 %; finger on foil gave
// head +45, tail +44, left foot +39, right foot +33, right hand +12, belly ~+7 %. 3 % caught the
// coupling, so raised per pad. Re-calibrate in the stuffed toy on the power bank.
const PadCfg kPads[] = {
    {SOFTTOY_TOUCH_GPIO_BELLY,      "belly",      Markers::Id::kCount,          50, false, false},
    {SOFTTOY_TOUCH_GPIO_RIGHT_HAND, "right_hand", Markers::Id::kTouchRightHand, 70, false, true},
    {SOFTTOY_TOUCH_GPIO_LEFT_FOOT,  "left_foot",  Markers::Id::kTouchLeftFoot,  80, true,  true},
    {SOFTTOY_TOUCH_GPIO_RIGHT_FOOT, "right_foot", Markers::Id::kTouchRightFoot, 80, true,  true},
    {SOFTTOY_TOUCH_GPIO_HEAD,       "head",       Markers::Id::kTouchHead,      80, false, true},
    {SOFTTOY_TOUCH_GPIO_TAIL,       "tail",       Markers::Id::kTouchTail,      80, false, true},
};
constexpr int kPadCount = sizeof(kPads) / sizeof(kPads[0]);

constexpr int kPollMs = 20;                 // sample period (50 Hz)
// delta must stay over the press level this many samples before the press counts (300 ms).
// Was 3 (60 ms). Stuffed board 2 (2026-11-13): touching the right foot coupled +22 % into the right
// hand (wires run together), and the hand crossed its threshold ~220 ms before the foot did, so
// it won. With 300 ms the foot is already over its threshold when the hand's cross-talk check
// runs, and the stronger pad wins. Raised again to 1 s (owner's decision, permanent): only a
// touch held >= 1 s counts, so brushes while she carries or hugs the toy don't trigger; quick
// pats are ignored on purpose. 1 s also gives the cross-talk check plenty of margin.
constexpr int kPressSamples = 50;
constexpr int kReleaseSamples = 5;          // ... and under the release level this many (100 ms)
constexpr int kReleaseFracPct = 50;         // release level = this % of the press threshold
constexpr int32_t kMinAbsDelta = 40;        // counts: a floor so a tiny baseline can't make a hair-trigger
constexpr int kStartSettleMs = 300;         // ignore the first samples after start
constexpr int kLearnSamples = 25;           // baseline = mean of this many untouched samples (0.5 s)
constexpr float kAdaptUp = 0.002f;          // slow baseline drift (time constant ~10 s), untouched pads only
constexpr float kAdaptDown = 0.02f;         // faster when the reading falls (a pad released before learning)
constexpr int kStuckMs = 120 * 1000;        // touched this long = treat as drift: re-learn, report a release
constexpr int kXtalkMs = 80;                // a second pad within this of another's press must be stronger to count
constexpr int kMotorTailMs = 100;           // no NEW press while the motor runs, or for this long after
constexpr int kCalibEveryMs = 2000;
constexpr int kTaskStack = 4096;
constexpr int kTaskPrio = 4;
}  // namespace cfg
// =================================================================================================

namespace {

enum class St : uint8_t { kLearning, kIdle, kTouched, kSuppressed, kDead };

struct Pad {
    touch_channel_handle_t chan = nullptr;
    St st = St::kLearning;
    float base = 0;
    float learn_sum = 0;
    int learn_n = 0;
    int above_n = 0;
    int below_n = 0;
    int32_t raw = 0;
    int32_t delta = 0;
    int32_t thr = 1;
    int64_t press_us = 0;
    bool logged_sup = false;
};

Pad g_pad[cfg::kPadCount];
std::atomic<bool> g_started{false};
std::atomic<bool> g_belly_touched{false};
std::atomic<int64_t> g_belly_press_us{0};
std::mutex g_cb_mutex;
std::function<void(bool)> g_belly_cb;

[[maybe_unused]] const char* StName(St s) {   // used only by the calibration log
    switch (s) {
        case St::kLearning: return "learning";
        case St::kIdle: return "idle";
        case St::kTouched: return "TOUCHED";
        case St::kSuppressed: return "suppressed";
        case St::kDead: return "dead";
    }
    return "?";
}

void BellyEdge(bool touched) {
    if (touched) {
        g_belly_press_us = esp_timer_get_time();
        g_belly_touched = true;
    } else {
        g_belly_touched = false;
    }
    std::function<void(bool)> cb;
    {
        std::lock_guard<std::mutex> lock(g_cb_mutex);
        cb = g_belly_cb;
    }
    if (cb) cb(touched);
}

int32_t PressThreshold(const Pad& p, const cfg::PadCfg& c) {
    int32_t t = (int32_t)(p.base * (float)c.press_permille / 1000.0f);
    return t < cfg::kMinAbsDelta ? cfg::kMinAbsDelta : t;
}

void OnPress(int i, int64_t now) {
    Pad& p = g_pad[i];
    const cfg::PadCfg& c = cfg::kPads[i];
    p.st = St::kTouched;
    p.below_n = 0;
    p.press_us = now;
    ESP_LOGI(TAG, "PRESS %s (GPIO%d) raw=%d base=%d delta=%d (+%d.%d%%) thr=%d",
             c.name, c.gpio, (int)p.raw, (int)p.base, (int)p.delta,
             (int)(p.delta * 100 / (int)p.base), (int)((p.delta * 1000 / (int)p.base) % 10),
             (int)p.thr);
    if (c.marker == Markers::Id::kCount) {
        ESP_LOGI(TAG, "%s: no marker and no buzz of its own (hug input for M5)", c.name);
        BellyEdge(true);
        return;
    }
    if (c.buzz) {
        if (c.giggle) Haptics::GetInstance().Giggle();
        else Haptics::GetInstance().Buzz();
    }
    Markers::Request(c.marker);   // logs "marker <text> sent|dropped(<reason>)"
}

void OnRelease(int i, int64_t now, bool forced) {
    Pad& p = g_pad[i];
    const cfg::PadCfg& c = cfg::kPads[i];
    ESP_LOGI(TAG, "RELEASE %s after %d ms%s", c.name, (int)((now - p.press_us) / 1000),
             forced ? " (stuck: re-learning baseline)" : "");
    p.st = St::kIdle;
    p.above_n = 0;
    p.below_n = 0;
    if (c.marker == Markers::Id::kCount) BellyEdge(false);
}

// Called once per sample for every pad that has a valid reading.
void Step(int64_t now, bool motor_ok) {
    // Pass 1: deltas and thresholds.
    for (int i = 0; i < cfg::kPadCount; i++) {
        Pad& p = g_pad[i];
        if (p.st == St::kDead || p.st == St::kLearning) continue;
        p.delta = p.raw - (int32_t)p.base;
        p.thr = PressThreshold(p, cfg::kPads[i]);
    }
    auto ratio = [](const Pad& p) { return (float)p.delta / (float)p.thr; };
    auto over = [](const Pad& p) { return p.delta >= p.thr; };

    // Pass 2: state machine.
    for (int i = 0; i < cfg::kPadCount; i++) {
        Pad& p = g_pad[i];
        const cfg::PadCfg& c = cfg::kPads[i];
        if (p.st == St::kDead || p.st == St::kLearning) continue;
        bool under = p.delta * 100 < p.thr * cfg::kReleaseFracPct;

        switch (p.st) {
        case St::kIdle:
            if (over(p) && motor_ok) {
                if (++p.above_n >= cfg::kPressSamples) {
                    // Cross-talk arbitration: only the strongest relative delta wins.
                    bool weaker = false;
                    for (int j = 0; j < cfg::kPadCount && !weaker; j++) {
                        if (j == i) continue;
                        const Pad& q = g_pad[j];
                        if (q.st == St::kDead || q.st == St::kLearning) continue;
                        if (over(q) && ratio(q) > ratio(p)) weaker = true;           // stronger one is up now
                        if (q.st == St::kTouched && now - q.press_us < cfg::kXtalkMs * 1000LL &&
                            ratio(q) >= ratio(p)) weaker = true;                      // just pressed, not weaker
                    }
                    if (weaker) {
                        p.st = St::kSuppressed;
                        p.below_n = 0;
                        ESP_LOGI(TAG, "cross-talk: %s ignored (weaker than a neighbour) raw=%d delta=%d",
                                 c.name, (int)p.raw, (int)p.delta);
                    } else {
                        OnPress(i, now);
                    }
                }
            } else {
                p.above_n = 0;
            }
            if (p.st == St::kIdle && p.delta * 2 < p.thr) {   // untouched: drift slowly
                float k = p.delta < 0 ? cfg::kAdaptDown : cfg::kAdaptUp;
                p.base += ((float)p.raw - p.base) * k;
            }
            break;
        case St::kSuppressed:
            if (under) {
                if (++p.below_n >= cfg::kReleaseSamples) { p.st = St::kIdle; p.above_n = 0; }
            } else {
                p.below_n = 0;
            }
            break;
        case St::kTouched:
            if (under) {
                if (++p.below_n >= cfg::kReleaseSamples) OnRelease(i, now, false);
            } else {
                p.below_n = 0;
            }
            if (p.st == St::kTouched && now - p.press_us > cfg::kStuckMs * 1000LL) {
                p.base = (float)p.raw;   // held for minutes: the "touch" is the new normal
                OnRelease(i, now, true);
            }
            break;
        default:
            break;
        }
    }
}

void Task(void*) {
    int64_t start_us = esp_timer_get_time();
    int calib_ticks = 0;
    int read_fail = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(cfg::kPollMs));
        int64_t now = esp_timer_get_time();
        if (now - start_us < cfg::kStartSettleMs * 1000LL) continue;

        // Read every pad.
        bool all_ok = true;
        for (int i = 0; i < cfg::kPadCount; i++) {
            Pad& p = g_pad[i];
            if (p.st == St::kDead) continue;
            uint32_t v[TOUCH_SAMPLE_CFG_NUM] = {};
            if (touch_channel_read_data(p.chan, TOUCH_CHAN_DATA_TYPE_SMOOTH, v) != ESP_OK || v[0] == 0) {
                all_ok = false;
                continue;
            }
            p.raw = (int32_t)v[0];
            if (p.st == St::kLearning) {
                p.learn_sum += (float)p.raw;
                if (++p.learn_n >= cfg::kLearnSamples) {
                    p.base = p.learn_sum / (float)p.learn_n;
                    p.st = St::kIdle;
                    ESP_LOGI(TAG, "%s (GPIO%d) baseline %d, press thr +%d", cfg::kPads[i].name,
                             cfg::kPads[i].gpio, (int)p.base, (int)PressThreshold(p, cfg::kPads[i]));
                }
            }
        }
        if (!all_ok && ++read_fail == 50) {   // ~1 s of failures, say it once
            ESP_LOGW(TAG, "some touch channels return no data");
        }
        if (all_ok) read_fail = 0;

        auto& h = Haptics::GetInstance();
        bool motor_ok = !h.IsPlaying() && h.MotorQuietFor(cfg::kMotorTailMs);
        Step(now, motor_ok);

#if SOFTTOY_TOUCH_CALIB_LOG
        calib_ticks += cfg::kPollMs;
        if (calib_ticks >= cfg::kCalibEveryMs) {
            calib_ticks = 0;
            for (int i = 0; i < cfg::kPadCount; i++) {
                const Pad& p = g_pad[i];
                if (p.st == St::kLearning || p.st == St::kDead) continue;
                int pm = (int)(p.delta * 1000 / (int)p.base);   // per mille of the baseline
                ESP_LOGI(TAG, "calib %-10s GPIO%-2d raw=%d base=%d delta=%d thr=%d (%d.%d%%) state=%s",
                         cfg::kPads[i].name, cfg::kPads[i].gpio, (int)p.raw, (int)p.base, (int)p.delta,
                         (int)p.thr, pm / 10, pm < 0 ? -(pm % 10) : pm % 10, StName(p.st));
            }
        }
#else
        (void)calib_ticks;
#endif
    }
}

}  // namespace

namespace TouchPads {

void Init() {
    if (g_started.exchange(true)) return;

    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = {
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(500, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2)};
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(1, sample_cfg);
    touch_sensor_handle_t sens = nullptr;
    esp_err_t err = touch_sensor_new_controller(&sens_cfg, &sens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "touch controller init failed (%s): touch pads disabled", esp_err_to_name(err));
        return;
    }

    // The hardware threshold/callbacks are unused (the polling task does its own); keep a large
    // value so the driver never has anything to report.
    touch_channel_config_t chan_cfg = {};
    chan_cfg.active_thresh[0] = 1000000;
    chan_cfg.charge_speed = TOUCH_CHARGE_SPEED_7;
    chan_cfg.init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT;
    for (int i = 0; i < cfg::kPadCount; i++) {
        err = touch_sensor_new_channel(sens, cfg::kPads[i].gpio, &chan_cfg, &g_pad[i].chan);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "touch channel %d (%s) failed (%s): touch pads disabled",
                     cfg::kPads[i].gpio, cfg::kPads[i].name, esp_err_to_name(err));
            return;
        }
    }
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    err = touch_sensor_config_filter(sens, &filter_cfg);
    if (err == ESP_OK) err = touch_sensor_enable(sens);
    if (err == ESP_OK) err = touch_sensor_start_continuous_scanning(sens);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "touch sensor start failed (%s): touch pads disabled", esp_err_to_name(err));
        return;
    }
    if (xTaskCreate(Task, "touch", cfg::kTaskStack, nullptr, cfg::kTaskPrio, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "touch task not created: touch pads disabled");
        return;
    }
    ESP_LOGI(TAG, "Touch ready: %d pads, polled every %d ms, press %d samples / release %d samples",
             cfg::kPadCount, cfg::kPollMs, cfg::kPressSamples, cfg::kReleaseSamples);
}

bool BellyTouched() { return g_belly_touched.load(); }

uint32_t BellyHeldMs() {
    if (!g_belly_touched.load()) return 0;
    int64_t d = esp_timer_get_time() - g_belly_press_us.load();
    return d > 0 ? (uint32_t)(d / 1000) : 0;
}

void SetBellyCallback(std::function<void(bool touched)> cb) {
    std::lock_guard<std::mutex> lock(g_cb_mutex);
    g_belly_cb = std::move(cb);
}

}  // namespace TouchPads
