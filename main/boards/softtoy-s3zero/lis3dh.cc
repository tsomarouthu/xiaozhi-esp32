#include "lis3dh.h"
#include "haptics.h"
#include "markers.h"
#include "config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdlib>
#include <new>

#define TAG "Motion"

// Print one line every 2 s with x/y/z, the derived values and the detector state, plus one line
// per impulse (the raw material for tuning hit / double-tap). Set to 0 once tuned.
#ifndef SOFTTOY_MOTION_CALIB_LOG
#define SOFTTOY_MOTION_CALIB_LOG 0   // off for the final build (2026-11-13)
#endif

// ============================ TUNING: every threshold lives here ============================
// Units: mg = thousandths of g (1000 mg = gravity). Time in ms. "dyn" = |a - gravity estimate|,
// the part of the acceleration that is not gravity; "act" = dyn smoothed over ~100 ms ("how much
// is it moving right now"). Defaults are a first guess for a 9-year-old handling a plush; refine
// with the SOFTTOY_MOTION_CALIB_LOG lines.
namespace cfg {
// --- Mounting: which sensor axis points UP when the toy sits or stands upright. ---------------
// Read the calibration line with the toy upright and set these so that "up" reads about +1000.
constexpr int kUpAxis = 2;        // 0 = x, 1 = y, 2 = z
constexpr int kUpSign = +1;       // +1 or -1

constexpr int kSampleMs = 5;                 // 200 Hz
constexpr float kGravityAlpha = 0.03f;       // gravity low-pass (time constant ~165 ms)
constexpr float kActAlpha = 0.05f;           // activity smoothing (~100 ms)
constexpr int kWarmupMs = 600;               // ignore everything this long after (re)start

// --- Rest / pick-up --------------------------------------------------------------------------
constexpr float kRestActMg = 20.0f;          // act below this = lying still on a surface (noise ~5-10)
constexpr float kRestBreakMg = 60.0f;        // act above this = not at rest (between: hold the count)
constexpr float kRestTiltDeg = 10.0f;        // tilting more than this restarts the rest count
constexpr int kArmRestMs = 60 * 1000;        // a pick-up can only fire after this much stillness
constexpr float kPickupActMg = 100.0f;       // act above this ...
constexpr int kPickupMoveMs = 400;           // ... for this long = sustained motion = pick-up
constexpr float kPickupTiltDeg = 30.0f;      // or: tilted this far from the rest orientation ...
constexpr int kPickupTiltMs = 300;           // ... for this long

// --- Shake: repeated high-g reversals ---------------------------------------------------------
constexpr float kShakeAxisMg = 900.0f;       // an axis must swing beyond +-this (about 1.8 g peak to peak)
constexpr int kShakeReversals = 4;           // this many reversals on one axis ...
constexpr int kShakeWindowMs = 1000;         // ... within this window
constexpr float kShakeQuietActMg = 150.0f;   // after a shake: act must stay below this ...
constexpr int kShakeQuietMs = 500;           // ... for this long before another shake can fire

// --- Impulses: hit (one sharp spike) and tap ------------------------------------------------
constexpr float kImpulseStartMg = 300.0f;    // dyn rises above this = impulse begins
constexpr float kImpulseEndMg = 150.0f;      // falls below this = impulse over
constexpr int kImpulseMaxMs = 60;            // longer than this is not a spike (carry, swing, shake)
constexpr float kHitMinMg = 2500.0f;         // spike this big = hit (sensor saturates at ~4000)
constexpr int kHitSettleMs = 250;            // a hit is confirmed after this, if no shake followed
constexpr float kTapMinMg = 300.0f;          // spike between this and kHitMinMg = tap candidate
constexpr float kTapCalmActMg = 40.0f;       // taps only count when the toy was calm before ...
constexpr int kTapCalmMs = 300;              // ... for this long (so footsteps / carrying aren't taps)
constexpr int kTapGapMinMs = 80;             // second tap start: this long after the first ...
constexpr int kTapGapMaxMs = 400;            // ... and no more than this
constexpr int kTapSettleMs = 150;            // double-tap is confirmed after this quiet time (a 3rd tap cancels)

// --- Orientation ----------------------------------------------------------------------------
constexpr float kInvertUpMg = -600.0f;       // up-axis reading below this = upside down ...
constexpr float kInvertClearMg = -300.0f;    // ... re-arms above this
constexpr int kInvertHoldMs = 1000;
constexpr float kUprightMg = 800.0f;         // up-axis reading above this = upright
constexpr int kUprightMs = 500;              // upright for this long counts as "was upright"
constexpr float kLieUpMaxMg = 500.0f;        // lying: up-axis reading below this (> ~60 deg from upright) ...
constexpr float kLieStillActMg = 40.0f;      // ... and act below this ...
constexpr int kLieHoldMs = 2000;             // ... for this long, after having been upright

// --- Held (in hand) ------------------------------------------------------------------------
// Buckets of 200 ms; a bucket is "alive" if its peak dyn is within [kHeldMinPeakMg, kHeldMaxPeakMg]
// (hand tremor, breathing, small adjustments: more than a surface, less than a shake). The toy is
// held when >= kHeldBuckets of the last 15 buckets (3 s) are alive.
constexpr int kHeldBucketMs = 200;
constexpr float kHeldMinPeakMg = 25.0f;
constexpr float kHeldMaxPeakMg = 1500.0f;
constexpr int kHeldBuckets = 9;
constexpr int kHeldGateLatchMs = 30 * 1000;  // keep the last answer this long while the motor gate discards reads

constexpr int kPollMs = 20;                  // FIFO drain period (<= 32 samples at 200 Hz = 160 ms)
}  // namespace cfg
// =============================================================================================

// Registers (datasheet DS: LIS3DH)
static constexpr uint8_t REG_WHO_AM_I = 0x0F;     // 0x33
static constexpr uint8_t REG_TEMP_CFG = 0x1F;
static constexpr uint8_t REG_CTRL1 = 0x20;        // ODR[7:4] | LPen | Zen Yen Xen
static constexpr uint8_t REG_CTRL2 = 0x21;
static constexpr uint8_t REG_CTRL3 = 0x22;
static constexpr uint8_t REG_CTRL4 = 0x23;        // BDU | FS[5:4] | HR
static constexpr uint8_t REG_CTRL5 = 0x24;        // FIFO_EN bit 6
static constexpr uint8_t REG_OUT_X_L = 0x28;
static constexpr uint8_t REG_FIFO_CTRL = 0x2E;    // FM[7:6]: 10 = stream
static constexpr uint8_t REG_FIFO_SRC = 0x2F;     // FSS[4:0] = samples stored
static constexpr uint8_t kWhoAmI = 0x33;
static constexpr uint8_t kCtrl1_200Hz_xyz = 0x67;  // 200 Hz, normal power, XYZ on
static constexpr uint8_t kCtrl4_bdu_4g_hr = 0x98;  // BDU, +-4 g, high resolution (12 bit): 2 mg/digit
static constexpr float kMgPerDigit = 2.0f;

namespace {

float Dot(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
float Norm(const float* a) { return sqrtf(Dot(a, a)); }
float AngleDeg(const float* a, const float* b) {
    float n = Norm(a) * Norm(b);
    if (n < 1.0f) return 0.0f;
    float c = Dot(a, b) / n;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c) * 57.29578f;
}

}  // namespace

// The gesture detector. Fed one sample at a time (5 ms apart) by the task; knows nothing about I2C.
struct Lis3dh::Impl {
    i2c_master_dev_handle_t dev = nullptr;
    Lis3dh* owner = nullptr;

    // --- filters ---
    bool have_lp = false;
    float lp[3] = {0, 0, 0};       // gravity estimate
    float act = 0;                 // smoothed dynamic acceleration
    int64_t t = 0;                 // ms, sample clock
    int warmup = 0;

    // --- rest / pick-up ---
    float ref[3] = {0, 0, 1000};   // orientation while resting
    int rest_ms = 0;
    bool armed = false;
    int move_ms = 0, tilt_ms = 0;

    // --- shake ---
    int8_t axis_sign[3] = {0, 0, 0};
    int64_t rev_t[3][6] = {};
    int rev_n[3] = {0, 0, 0};
    bool shake_latched = false;
    int shake_quiet_ms = 0;

    // --- impulses ---
    bool in_impulse = false;
    bool impulse_long = false;
    float impulse_peak = 0;
    int64_t impulse_start = 0;
    int calm_ms = 0;               // continuous calm (act < kTapCalmActMg)
    int calm_at_start = 0;
    int64_t hit_deadline = -1;     // pending hit, fire at this time
    int tap_count = 0;             // taps in the current sequence (0, 1, 2)
    int64_t tap_first = 0;
    int64_t tap_deadline = -1;     // pending double-tap, fire at this time

    // --- orientation ---
    int inv_ms = 0;
    bool inv_fired = false;
    int upright_ms = 0;
    bool was_upright = false;
    int lie_ms = 0;

    // --- held ---
    float bucket_peak = 0;
    int bucket_ms = 0;
    uint16_t held_hist = 0;
    int64_t gated_since = -1;

    // --- logging ---
    int64_t last_calib_log = 0;

    void ResetTransients() {
        // Called when reads were discarded (motor gate): forget everything short-term. The pick-up
        // arming is cleared too: a motor effect means the toy is being interacted with.
        have_lp = false;
        warmup = cfg::kWarmupMs;
        act = 0;
        rest_ms = 0; armed = false; move_ms = tilt_ms = 0;
        for (int i = 0; i < 3; i++) { axis_sign[i] = 0; rev_n[i] = 0; }
        in_impulse = false; impulse_long = false;
        hit_deadline = -1; tap_count = 0; tap_deadline = -1;
        inv_ms = 0; upright_ms = 0; lie_ms = 0; calm_ms = 0;
        shake_quiet_ms = 0;
    }

    void Fire(const char* name, Markers::Id id) {
        ESP_LOGI(TAG, "gesture: %s", name);
        Markers::Request(id);
    }

    void NoteGated(int64_t now_ms) {
        if (gated_since < 0) gated_since = now_ms;
        if (now_ms - gated_since > cfg::kHeldGateLatchMs) {
            owner->held_ = false;
            held_hist = 0;
        }
    }

    // One sample, in mg. dt is cfg::kSampleMs.
    void Process(const float a[3]) {
        constexpr int dt = cfg::kSampleMs;
        t += dt;
        gated_since = -1;
        if (!have_lp) {
            for (int i = 0; i < 3; i++) lp[i] = a[i];
            have_lp = true;
            for (int i = 0; i < 3; i++) ref[i] = a[i];
        }
        float d[3];
        for (int i = 0; i < 3; i++) {
            lp[i] += cfg::kGravityAlpha * (a[i] - lp[i]);
            d[i] = a[i] - lp[i];
        }
        float dm = Norm(d);
        act += cfg::kActAlpha * (dm - act);
        if (warmup > 0) {
            warmup -= dt;
            return;
        }
        float up = cfg::kUpSign * lp[cfg::kUpAxis];

        UpdateHeld(dm);
        UpdateCalm();
        bool shake_now = UpdateShake(d);
        UpdateImpulses(dm, shake_now);
        UpdateOrientation(up);
        UpdatePickup();
        MaybeLogCalib(a, up);
    }

    // ---- Held: sliding 3 s window of 200 ms buckets (see cfg) ----
    void UpdateHeld(float dm) {
        if (dm > bucket_peak) bucket_peak = dm;
        bucket_ms += cfg::kSampleMs;
        if (bucket_ms < cfg::kHeldBucketMs) return;
        bool alive = bucket_peak >= cfg::kHeldMinPeakMg && bucket_peak <= cfg::kHeldMaxPeakMg;
        held_hist = (uint16_t)(((held_hist << 1) | (alive ? 1 : 0)) & 0x7FFF);
        bucket_peak = 0;
        bucket_ms = 0;
        owner->held_ = __builtin_popcount(held_hist) >= cfg::kHeldBuckets;
    }

    void UpdateCalm() {
        if (act < cfg::kTapCalmActMg) calm_ms += cfg::kSampleMs;
        else calm_ms = 0;
    }

    // ---- Shake: >= kShakeReversals swings beyond +-kShakeAxisMg on one axis within the window ----
    // Returns true on the sample where a shake fires.
    bool UpdateShake(const float d[3]) {
        constexpr int dt = cfg::kSampleMs;
        if (shake_latched) {
            if (act < cfg::kShakeQuietActMg) shake_quiet_ms += dt;
            else shake_quiet_ms = 0;
            if (shake_quiet_ms >= cfg::kShakeQuietMs) shake_latched = false;
            return false;
        }
        for (int i = 0; i < 3; i++) {
            int8_t s = d[i] > cfg::kShakeAxisMg ? 1 : (d[i] < -cfg::kShakeAxisMg ? -1 : 0);
            if (s == 0) continue;
            if (axis_sign[i] != 0 && s != axis_sign[i]) {
                // reversal: keep the last kShakeReversals timestamps
                if (rev_n[i] == cfg::kShakeReversals) {
                    for (int k = 1; k < cfg::kShakeReversals; k++) rev_t[i][k - 1] = rev_t[i][k];
                    rev_n[i]--;
                }
                rev_t[i][rev_n[i]++] = t;
                if (rev_n[i] == cfg::kShakeReversals &&
                    t - rev_t[i][0] <= cfg::kShakeWindowMs) {
                    FireShake();
                    return true;
                }
            }
            axis_sign[i] = s;
        }
        return false;
    }

    void FireShake() {
        shake_latched = true;
        shake_quiet_ms = 0;
        for (int i = 0; i < 3; i++) { rev_n[i] = 0; axis_sign[i] = 0; }
        hit_deadline = -1;          // a shake is not also a hit or a tap
        tap_count = 0; tap_deadline = -1;
        in_impulse = false;
        Fire("shake", Markers::Id::kShake);
    }

    // ---- Impulses: hit = one sharp spike; double-tap = two small spikes from calm ----
    void UpdateImpulses(float dm, bool shake_now) {
        if (shake_now || shake_latched) {
            in_impulse = false;
            return;
        }
        // Pending decisions.
        if (hit_deadline >= 0 && t >= hit_deadline) {
            hit_deadline = -1;
            Fire("hit", Markers::Id::kHit);
        }
        if (tap_deadline >= 0 && t >= tap_deadline) {
            tap_deadline = -1;
            tap_count = 0;
            Fire("double_tap", Markers::Id::kDoubleTap);
        }
        if (tap_count == 1 && t - tap_first > cfg::kTapGapMaxMs) tap_count = 0;

        if (!in_impulse) {
            if (dm > cfg::kImpulseStartMg) {
                in_impulse = true;
                impulse_long = false;
                impulse_peak = dm;
                impulse_start = t;
                calm_at_start = calm_ms;
            }
            return;
        }
        if (dm > impulse_peak) impulse_peak = dm;
        if (!impulse_long && t - impulse_start > cfg::kImpulseMaxMs) impulse_long = true;
        if (dm > cfg::kImpulseEndMg) return;   // still inside the impulse
        in_impulse = false;
        if (impulse_long) return;              // too long to be a spike
#if SOFTTOY_MOTION_CALIB_LOG
        ESP_LOGI(TAG, "impulse peak=%d mg dur=%d ms calm_before=%d ms", (int)impulse_peak,
                 (int)(t - impulse_start), calm_at_start);
#endif
        if (impulse_peak >= cfg::kHitMinMg) {
            tap_count = 0; tap_deadline = -1;
            if (hit_deadline < 0) hit_deadline = t + cfg::kHitSettleMs;
        } else if (impulse_peak >= cfg::kTapMinMg) {
            OnTap();
        }
    }

    void OnTap() {
        if (hit_deadline >= 0) return;   // rebound of a hit
        if (tap_count == 0) {
            if (calm_at_start < cfg::kTapCalmMs) return;   // not calm before: carrying, walking
            tap_count = 1;
            tap_first = impulse_start;
        } else if (tap_count == 1) {
            int64_t gap = impulse_start - tap_first;
            if (gap < cfg::kTapGapMinMs) return;            // same bounce
            if (gap > cfg::kTapGapMaxMs) {                  // too slow: this one starts a new pair
                tap_count = 0;
                OnTap();
                return;
            }
            tap_count = 2;
            tap_deadline = t + cfg::kTapSettleMs;
        } else {
            // third tap: a triple is not a double-tap
            tap_count = 0;
            tap_deadline = -1;
        }
    }

    // ---- Orientation: upside-down (held ~1 s) and lie-down (still ~2 s after being upright) ----
    void UpdateOrientation(float up) {
        constexpr int dt = cfg::kSampleMs;
        // upside-down
        if (up < cfg::kInvertUpMg) {
            if (inv_ms < cfg::kInvertHoldMs) inv_ms += dt;
            if (inv_ms >= cfg::kInvertHoldMs && !inv_fired) {
                inv_fired = true;
                Fire("upside_down", Markers::Id::kUpsideDown);
            }
        } else if (up > cfg::kInvertClearMg) {
            inv_ms = 0;
            inv_fired = false;
        }
        // upright memory
        if (up > cfg::kUprightMg) {
            if (upright_ms < cfg::kUprightMs) upright_ms += dt;
            if (upright_ms >= cfg::kUprightMs) was_upright = true;
        } else {
            upright_ms = 0;
        }
        // lie-down: neither upright nor upside-down, and still
        bool lying = up < cfg::kLieUpMaxMg && up > cfg::kInvertUpMg && act < cfg::kLieStillActMg;
        if (lying) {
            if (lie_ms < cfg::kLieHoldMs) lie_ms += dt;
            if (lie_ms >= cfg::kLieHoldMs && was_upright) {
                was_upright = false;
                Fire("lie_down", Markers::Id::kLieDown);
            }
        } else {
            lie_ms = 0;
        }
    }

    // ---- Pick-up: 60 s of stillness arms it; then sustained motion or a tilt fires it once ----
    void UpdatePickup() {
        constexpr int dt = cfg::kSampleMs;
        if (!armed) {
            float tilt = AngleDeg(lp, ref);
            if (act < cfg::kRestActMg && tilt < cfg::kRestTiltDeg) {
                rest_ms += dt;
            } else if (act > cfg::kRestBreakMg || tilt >= cfg::kRestTiltDeg) {
                rest_ms = 0;
                for (int i = 0; i < 3; i++) ref[i] = lp[i];
            }
            if (rest_ms >= cfg::kArmRestMs) {
                armed = true;
                move_ms = tilt_ms = 0;
                ESP_LOGI(TAG, "pick-up armed (still for %d s)", cfg::kArmRestMs / 1000);
            }
            return;
        }
        move_ms = act > cfg::kPickupActMg ? move_ms + dt : 0;
        tilt_ms = AngleDeg(lp, ref) > cfg::kPickupTiltDeg ? tilt_ms + dt : 0;
        if (move_ms >= cfg::kPickupMoveMs || tilt_ms >= cfg::kPickupTiltMs) {
            armed = false;
            rest_ms = 0;
            for (int i = 0; i < 3; i++) ref[i] = lp[i];
            Fire("pickup", Markers::Id::kPickup);
        }
    }

    void MaybeLogCalib(const float a[3], float up) {
#if SOFTTOY_MOTION_CALIB_LOG
        if (t - last_calib_log < 2000) return;
        last_calib_log = t;
        ESP_LOGI(TAG, "x=%d y=%d z=%d up=%d act=%d rest=%ds armed=%d held=%d upright=%d tilt=%d",
                 (int)a[0], (int)a[1], (int)a[2], (int)up, (int)act, rest_ms / 1000, armed,
                 (int)owner->held_.load(), was_upright, (int)AngleDeg(lp, ref));
#else
        (void)a; (void)up;
#endif
    }
};

Lis3dh& Lis3dh::GetInstance() {
    static Lis3dh l;
    return l;
}

static bool Write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, 50) == ESP_OK;
}

static bool Read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* out, size_t n) {
    // Bit 7 of the sub-address = auto-increment for multi-byte reads.
    uint8_t r = n > 1 ? (uint8_t)(reg | 0x80) : reg;
    return i2c_master_transmit_receive(dev, &r, 1, out, n, 50) == ESP_OK;
}

void Lis3dh::Init(i2c_master_bus_handle_t bus) {
    if (impl_ != nullptr) return;
    if (bus == nullptr) {
        ESP_LOGW(TAG, "No I2C bus: motion gestures disabled");
        return;
    }
    if (i2c_master_probe(bus, SOFTTOY_I2C_ADDR_LIS3DH, 50) != ESP_OK) {
        ESP_LOGW(TAG, "LIS3DH not found at 0x%02X: motion gestures disabled", SOFTTOY_I2C_ADDR_LIS3DH);
        return;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = SOFTTOY_I2C_ADDR_LIS3DH;
    dev_cfg.scl_speed_hz = SOFTTOY_I2C_FREQ_HZ;
    i2c_master_dev_handle_t dev = nullptr;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        ESP_LOGW(TAG, "Could not add the LIS3DH to the bus: motion gestures disabled");
        return;
    }
    uint8_t who = 0;
    if (!Read(dev, REG_WHO_AM_I, &who, 1) || who != kWhoAmI) {
        ESP_LOGW(TAG, "LIS3DH WHO_AM_I=0x%02X (expected 0x%02X): motion gestures disabled", who, kWhoAmI);
        i2c_master_bus_rm_device(dev);
        return;
    }
    bool ok = Write(dev, REG_CTRL1, 0x00)                 // power down while configuring
              && Write(dev, REG_TEMP_CFG, 0x00)
              && Write(dev, REG_CTRL2, 0x00)              // no high-pass filter
              && Write(dev, REG_CTRL3, 0x00)              // INT1 unused
              && Write(dev, REG_CTRL4, kCtrl4_bdu_4g_hr)
              && Write(dev, REG_CTRL5, 0x40)              // FIFO enable
              && Write(dev, REG_FIFO_CTRL, 0x00)          // bypass -> clears the FIFO
              && Write(dev, REG_FIFO_CTRL, 0x80)          // stream mode
              && Write(dev, REG_CTRL1, kCtrl1_200Hz_xyz);
    impl_ = new (std::nothrow) Impl();
    if (!ok || impl_ == nullptr) {
        ESP_LOGW(TAG, "LIS3DH configuration failed: motion gestures disabled");
        i2c_master_bus_rm_device(dev);
        delete impl_;
        impl_ = nullptr;
        return;
    }
    impl_->dev = dev;
    impl_->owner = this;
    impl_->ResetTransients();
    if (xTaskCreate(TaskThunk, "lis3dh", 6144, this, 2, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Could not start the motion task: motion gestures disabled");
        return;   // leaked deliberately (boot-time, once)
    }
    available_ = true;
    ESP_LOGI(TAG, "LIS3DH ready (WHO_AM_I 0x33, 200 Hz, +-4 g, FIFO stream, polled every %d ms, INT1 unused)",
             cfg::kPollMs);
}

void Lis3dh::TaskThunk(void* arg) {
    auto* self = static_cast<Lis3dh*>(arg);
    Impl* m = self->impl_;
    auto& haptics = Haptics::GetInstance();
    vTaskDelay(pdMS_TO_TICKS(1000));   // let the app finish constructing before we touch it
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(cfg::kPollMs));
        uint8_t src = 0;
        if (!Read(m->dev, REG_FIFO_SRC, &src, 1)) continue;
        int n = src & 0x1F;
        if (src & 0x40) n = 32;   // overrun: the FIFO is full

        // Gate: with the motor running (or just stopped) discard everything and start over.
        if (!haptics.MotorQuietFor(Haptics::kAccelTailMs)) {
            Write(m->dev, REG_FIFO_CTRL, 0x00);   // bypass: empties the FIFO
            Write(m->dev, REG_FIFO_CTRL, 0x80);   // back to stream
            m->ResetTransients();
            m->NoteGated(esp_timer_get_time() / 1000);
            continue;
        }
        static uint8_t raw[32][6];   // off-stack
        int got = 0;
        for (; got < n; got++) {
            if (!Read(m->dev, REG_OUT_X_L, raw[got], 6)) break;
        }
        // Re-check after reading: if the motor started meanwhile, these samples are suspect.
        if (!haptics.MotorQuietFor(Haptics::kAccelTailMs)) {
            m->ResetTransients();
            continue;
        }
        for (int i = 0; i < got; i++) {
            float a[3];
            for (int k = 0; k < 3; k++) {
                int16_t v = (int16_t)((raw[i][2 * k + 1] << 8) | raw[i][2 * k]);
                a[k] = (float)(v >> 4) * kMgPerDigit;
            }
            m->Process(a);
        }
    }
}
