#include "reaction_clips.h"
#include "application.h"

#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <atomic>
#include <string_view>

#define TAG "ReactionClips"

// Weak so a clip that has not been rendered yet leaves the build intact (it just never plays).
#define RC_DECLARE(name)                                                                     \
    extern const char rc_##name##_start[] asm("_binary_rc_" #name "_ogg_start") __attribute__((weak)); \
    extern const char rc_##name##_end[] asm("_binary_rc_" #name "_ogg_end") __attribute__((weak))
#define RC(name) Clip{rc_##name##_start, rc_##name##_end}

RC_DECLARE(touch_right_hand_1);
RC_DECLARE(touch_left_foot_1);
RC_DECLARE(touch_right_foot_1);
RC_DECLARE(touch_head_1);
RC_DECLARE(touch_head_2);
RC_DECLARE(touch_tail_1);
RC_DECLARE(shake_1);
RC_DECLARE(upside_down_1);
RC_DECLARE(hit_1);
RC_DECLARE(lie_down_1);
RC_DECLARE(double_tap_1);
RC_DECLARE(hug_1);
RC_DECLARE(hug_2);
RC_DECLARE(pickup_1);
RC_DECLARE(pickup_2);
RC_DECLARE(back_1);

namespace ReactionClips {
namespace {

constexpr int64_t kMsUs = 1000LL;
constexpr int64_t kGlobalGapMs = 20 * 1000;        // at most one reaction clip per 20 s
constexpr int64_t kBackWindowMs = 10 * 60 * 1000;  // greet only if told "offline" this recently

struct Clip {
    const char* start;
    const char* end;
    bool ok() const { return start != nullptr && end != nullptr && end > start; }
    std::string_view view() const { return {start, (size_t)(end - start)}; }
};

struct Variants {
    Clip a;
    Clip b;
};

Variants For(Markers::Id id) {
    using Markers::Id;
    switch (id) {
        case Id::kTouchRightHand: return {RC(touch_right_hand_1), {}};
        case Id::kTouchLeftFoot:  return {RC(touch_left_foot_1), {}};
        case Id::kTouchRightFoot: return {RC(touch_right_foot_1), {}};
        case Id::kTouchHead:      return {RC(touch_head_1), RC(touch_head_2)};
        case Id::kTouchTail:      return {RC(touch_tail_1), {}};
        case Id::kShake:          return {RC(shake_1), {}};
        case Id::kUpsideDown:     return {RC(upside_down_1), {}};
        case Id::kHit:            return {RC(hit_1), {}};
        case Id::kLieDown:        return {RC(lie_down_1), {}};
        case Id::kDoubleTap:      return {RC(double_tap_1), {}};
        case Id::kHug:            return {RC(hug_1), RC(hug_2)};
        case Id::kPickup:         return {RC(pickup_1), RC(pickup_2)};
        default:                  return {};
    }
}

// Opus 24 kbps ~ 3 KB/s; add a margin for the start-up latency.
int64_t ApproxMs(const Clip& c) { return (int64_t)(c.end - c.start) * 1000 / 2500 + 600; }

std::atomic<int64_t> g_busy_until_us{0};
std::atomic<int64_t> g_last_clip_us{0};      // 0 = none yet
std::atomic<int64_t> g_told_offline_us{0};   // 0 = not told

bool QueueClip(const Clip& c, const char* what) {
    int64_t now = esp_timer_get_time();
    g_busy_until_us = now + ApproxMs(c) * kMsUs;
    g_last_clip_us = now;
    std::string_view view = c.view();
    Application::GetInstance().Schedule([view, what]() {
        // Re-check on the main task: only ever play into a quiet, idle toy.
        if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
            ESP_LOGI(TAG, "%s clip skipped (not idle)", what);
            return;
        }
        Application::GetInstance().PlaySound(view);
    });
    ESP_LOGI(TAG, "%s clip queued (%u bytes)", what, (unsigned)view.size());
    return true;
}

}  // namespace

bool PlayFor(Markers::Id id) {
    Variants v = For(id);
    const Clip* pick = nullptr;
    if (v.a.ok() && v.b.ok()) pick = (esp_random() & 1) ? &v.b : &v.a;
    else if (v.a.ok()) pick = &v.a;
    else if (v.b.ok()) pick = &v.b;
    if (pick == nullptr) {
        ESP_LOGI(TAG, "no clip for %s (not rendered?)", Markers::Text(id));
        return false;
    }
    int64_t now = esp_timer_get_time();
    if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
        ESP_LOGI(TAG, "%s: no clip (device not idle)", Markers::Text(id));
        return false;
    }
    if (now < g_busy_until_us.load()) {
        ESP_LOGI(TAG, "%s: no clip (another clip still playing)", Markers::Text(id));
        return false;
    }
    int64_t last = g_last_clip_us.load();
    if (last != 0 && now - last < kGlobalGapMs * kMsUs) {
        ESP_LOGI(TAG, "%s: no clip (global 20 s limit)", Markers::Text(id));
        return false;
    }
    QueueClip(*pick, Markers::Text(id));
    g_told_offline_us = now;   // a reaction made while offline counts as "she saw it was offline"
    return true;
}

bool PlayBackOnlineIfDue() {
    int64_t told = g_told_offline_us.load();
    int64_t now = esp_timer_get_time();
    if (told == 0 || now - told > kBackWindowMs * kMsUs) return false;
    Clip c = RC(back_1);
    if (!c.ok()) {
        ESP_LOGI(TAG, "back-online clip not rendered; skipping");
        return false;
    }
    if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle ||
        now < g_busy_until_us.load()) {
        return false;   // not a quiet moment; the window keeps the greeting due
    }
    g_told_offline_us = 0;   // once only
    ESP_LOGI(TAG, "back online after being offline: Yay clip");
    return QueueClip(c, "back_online");
}

void NoteStatusClipSpoken(int64_t approx_ms) {
    int64_t now = esp_timer_get_time();
    g_told_offline_us = now;
    int64_t until = now + approx_ms * kMsUs;
    if (until > g_busy_until_us.load()) g_busy_until_us = until;
}

}  // namespace ReactionClips
