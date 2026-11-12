#include "hug.h"
#include "haptics.h"
#include "lis3dh.h"
#include "markers.h"
#include "touch_pads.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "Hug"

namespace {
// ---- Tuning (edit here only) ----
constexpr uint32_t kHugHoldMs = 1500;      // belly must be touched this long, continuously, while held
constexpr uint32_t kHugMaxMs = 60 * 1000;  // safety cap on one purr; a new hug needs a belly release first
constexpr uint32_t kReleaseGraceMs = 300;  // belly must stay released this long to end the hug
constexpr uint32_t kPollMs = 50;

void HugTask(void*) {
    bool active = false;
    bool armed = true;               // false after a capped hug until the belly is released
    int64_t start_us = 0;
    int64_t released_since_us = 0;   // 0 = belly currently touched
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
        const bool belly = TouchPads::BellyTouched();
        const int64_t now = esp_timer_get_time();
        if (!active) {
            if (!belly) {
                armed = true;
                continue;
            }
            if (armed && TouchPads::BellyHeldMs() >= kHugHoldMs && Lis3dh::GetInstance().IsHeld()) {
                active = true;
                armed = false;
                start_us = now;
                released_since_us = 0;
                ESP_LOGI(TAG, "hug start (belly %u ms + held): purr on, sending [hug]",
                         (unsigned)TouchPads::BellyHeldMs());
                Haptics::GetInstance().Purr(true, Haptics::PurrOwner::kHug);
                Markers::Request(Markers::Id::kHug);
            }
            continue;
        }
        // Active hug.
        const char* why = nullptr;
        if (belly) {
            released_since_us = 0;
        } else if (released_since_us == 0) {
            released_since_us = now;
        } else if ((now - released_since_us) / 1000 >= (int64_t)kReleaseGraceMs) {
            why = "belly released";
        }
        if (why == nullptr && (now - start_us) / 1000 >= (int64_t)kHugMaxMs) why = "max duration";
        if (why != nullptr) {
            Haptics::GetInstance().Purr(false, Haptics::PurrOwner::kHug);
            ESP_LOGI(TAG, "hug end (%s) after %lld ms", why, (long long)((now - start_us) / 1000));
            active = false;
            // After a cap the belly is still down: stay disarmed until it is released.
            armed = (why[0] == 'b');
        }
    }
}
}  // namespace

namespace Hug {
void Init() {
    if (xTaskCreate(HugTask, "hug", 3072, nullptr, 2, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Could not start hug task: hug disabled");
        return;
    }
    ESP_LOGI(TAG, "Hug ready: belly %u ms + held, max %u s", (unsigned)kHugHoldMs, (unsigned)(kHugMaxMs / 1000));
}
}  // namespace Hug
