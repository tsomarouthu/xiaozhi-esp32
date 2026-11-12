#include "markers.h"
#include "haptics.h"
#include "application.h"
#include "reaction_clips.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <mutex>

#define TAG "Markers"

namespace Markers {
namespace {

constexpr int64_t kNever = INT64_MIN;
constexpr int64_t kSecUs = 1000000LL;
constexpr int64_t kGlobalGapUs = 3 * kSecUs;                // between any two markers (button exempt)
constexpr int64_t kPickupSessionQuietUs = 10 * 60 * kSecUs;  // no pick-up this soon after a session
constexpr int64_t kHitSuppressHeadUs = 300 * 1000LL;         // a hit wins over a head pat

struct Info {
    const char* text;
    MarkerStart start;
    int64_t limit_us;    // per-marker minimum gap between sent markers
    bool exempt;         // skips every limit and the online check (the button)
};

// Table order = enum Id. Rates and MarkerStart are the M0 conventions (INTEGRATION-PLAN.md).
const Info kInfo[] = {
    {"[wake: button]",        MarkerStart::kStartOnly,        0,                 true},
    {"[greeting: pickup]",    MarkerStart::kStartOnly,        3 * 3600 * kSecUs, false},
    {"[hug]",                 MarkerStart::kStartOrInSession, 60 * kSecUs,       false},
    {"[touch: right_hand]",   MarkerStart::kInSessionOnly,    10 * kSecUs,       false},
    {"[touch: left_foot]",    MarkerStart::kInSessionOnly,    10 * kSecUs,       false},
    {"[touch: right_foot]",   MarkerStart::kInSessionOnly,    10 * kSecUs,       false},
    {"[touch: head]",         MarkerStart::kInSessionOnly,    10 * kSecUs,       false},
    {"[touch: tail]",         MarkerStart::kInSessionOnly,    10 * kSecUs,       false},
    {"[motion: shake]",       MarkerStart::kInSessionOnly,    15 * kSecUs,       false},
    {"[motion: upside_down]", MarkerStart::kInSessionOnly,    15 * kSecUs,       false},
    {"[motion: hit]",         MarkerStart::kInSessionOnly,    15 * kSecUs,       false},
    {"[motion: lie_down]",    MarkerStart::kInSessionOnly,    15 * kSecUs,       false},
    {"[motion: double_tap]",  MarkerStart::kInSessionOnly,    15 * kSecUs,       false},
};
static_assert(sizeof(kInfo) / sizeof(kInfo[0]) == (size_t)Id::kCount, "marker table out of sync");

std::mutex g_mutex;
std::function<bool()> g_is_online;
int64_t g_last_sent[(size_t)Id::kCount];     // per marker; set to kNever by EnsureInit()
int64_t g_last_any_sent = kNever;            // for the global gap
int64_t g_last_hit = kNever;
int64_t g_session_end = kNever;
bool g_session_active = false;
bool g_inflight = false;                     // a non-button request is waiting for done()
bool g_inited = false;

void EnsureInit() {   // caller holds g_mutex
    if (g_inited) return;
    for (auto& t : g_last_sent) t = kNever;
    g_inited = true;
}

bool Elapsed(int64_t now, int64_t since, int64_t span) {
    return since == kNever || now - since >= span;
}

// The haptic that goes with a motion marker once it is accepted. Pick-up gets none (the
// listen-start tick already marks the session); touch and hug haptics are M4/M5's.
void MotionHaptic(Id id) {
    auto& h = Haptics::GetInstance();
    switch (id) {
        case Id::kShake:      h.Wiggle(); break;
        case Id::kUpsideDown: h.Giggle(); break;
        case Id::kHit:        h.Buzz(); break;
        case Id::kLieDown:    h.Heartbeat(); break;
        case Id::kDoubleTap:  h.Buzz(); break;
        default: break;
    }
}

}  // namespace

const char* Text(Id id) { return kInfo[(size_t)id].text; }

void Init(std::function<bool()> is_online) {
    std::lock_guard<std::mutex> lock(g_mutex);
    EnsureInit();
    g_is_online = std::move(is_online);
}

void NoteDeviceState(DeviceState state) {
    bool active = state == kDeviceStateListening || state == kDeviceStateSpeaking ||
                  state == kDeviceStateConnecting;
    std::lock_guard<std::mutex> lock(g_mutex);
    EnsureInit();
    if (g_session_active && !active) {
        g_session_end = esp_timer_get_time();   // a turn (or the session) just ended
    }
    g_session_active = active;
}

void Consume(Id id) {
    std::lock_guard<std::mutex> lock(g_mutex);
    EnsureInit();
    int64_t now = esp_timer_get_time();
    g_last_sent[(size_t)id] = now;
    g_last_any_sent = now;
}

void OnOfflineMarker(Id id) {
    // M6: play the marker's local clip (random variant) if the device is idle, no other local clip
    // is playing and the global 20 s reaction limit allows. Only a played clip uses up the
    // marker's own rate limit and the 3 s global gap.
    if (ReactionClips::PlayFor(id)) {
        Consume(id);
        ESP_LOGI(TAG, "offline marker %s: clip played", kInfo[(size_t)id].text);
    } else {
        ESP_LOGI(TAG, "offline marker %s: no clip", kInfo[(size_t)id].text);
    }
}

void Request(Id id, std::function<void(bool accepted)> done) {
    const Info& info = kInfo[(size_t)id];
    int64_t now = esp_timer_get_time();
    const char* reason = nullptr;
    std::function<bool()> online;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        EnsureInit();
        if (id == Id::kHit) {
            g_last_hit = now;   // recorded even if the hit itself is rate-limited: it still happened
        }
        if (!info.exempt) {
            if (id == Id::kTouchHead && !Elapsed(now, g_last_hit, kHitSuppressHeadUs)) {
                reason = "hit_wins";
            } else if (g_inflight) {
                reason = "busy";
            } else if (!Elapsed(now, g_last_any_sent, kGlobalGapUs)) {
                reason = "global_gap";
            } else if (!Elapsed(now, g_last_sent[(size_t)id], info.limit_us)) {
                reason = "rate";
            } else if (id == Id::kPickup &&
                       (g_session_active || !Elapsed(now, g_session_end, kPickupSessionQuietUs))) {
                reason = "after_session";
            }
            if (reason == nullptr) {
                online = g_is_online;
            }
        }
    }
    if (reason != nullptr) {
        ESP_LOGI(TAG, "marker %s dropped(%s)", info.text, reason);
        if (done) done(false);
        return;
    }

    if (online && !online()) {
        // Offline: the limits above were passed, so this marker would have been sent. M6 plays
        // a local clip and, as always, the marker's haptic fires locally (motion markers here;
        // touch and hug haptics are fired by their own modules). The mic gate is inside Haptics.
        ESP_LOGI(TAG, "marker %s dropped(offline)", info.text);
        OnOfflineMarker(id);
        MotionHaptic(id);
        if (done) done(false);
        return;
    }

    if (!info.exempt) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_inflight = true;
    }
    Application::GetInstance().SendEventMarker(
        info.text, info.start, [id, done = std::move(done)](bool accepted) {
            const Info& i = kInfo[(size_t)id];
            if (!i.exempt) {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_inflight = false;
                if (accepted) {
                    int64_t t = esp_timer_get_time();
                    g_last_sent[(size_t)id] = t;
                    g_last_any_sent = t;
                }
            }
            if (accepted) {
                ESP_LOGI(TAG, "marker %s sent", i.text);
                MotionHaptic(id);
            } else {
                ESP_LOGI(TAG, "marker %s dropped(app)", i.text);
            }
            if (done) done(accepted);
        });
}

}  // namespace Markers
