#include "haptics.h"
#include "drv2605l.h"
#include "config.h"
#include "application.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <new>

#define TAG "Haptics"

// ---- Effect recipes (DRV2605L ROM library 1 = ERM, TS2200 A). Edit the numbers here only. ----
// ROM ids used: 1 Strong Click 100%, 7 Soft Bump 100%, 8 Soft Bump 60%, 13 Soft Fuzz 60%,
// 27/28/29 Short Double Click Strong 100/80/60%, 52 Pulsing Strong 1 100%, 53 Pulsing Strong 2 60%.
// Wait(ms) entries are pauses in the sequencer.
static const uint8_t kSeqBuzz[]      = {1};                                    // one crisp "I felt that"
static const uint8_t kSeqEdge[]      = {1};                                    // listen start/stop tick (kept separate to tune)
static const uint8_t kSeqPurrStep[]  = {13, Drv2605l::Wait(60), 13};           // soft fuzz pair; looped for the purr
static const uint8_t kSeqGiggle[]    = {27, Drv2605l::Wait(60), 28, Drv2605l::Wait(60), 29};  // three tapering double-clicks
static const uint8_t kSeqHeartbeat[] = {7, Drv2605l::Wait(120), 8, Drv2605l::Wait(500), 7, Drv2605l::Wait(120), 8};  // lub-dub, lub-dub
static const uint8_t kSeqWiggle[]    = {52, Drv2605l::Wait(40), 53, Drv2605l::Wait(40), 52};   // excited pulsing
static constexpr uint32_t kPurrGapMs = 140;        // pause between purr steps
static constexpr uint32_t kMaxEffectMs = 2500;     // safety: abort a sequence that never ends
static constexpr uint32_t kSelfTestGapMs = 300;

#define SEQ(a) (a), (uint8_t)sizeof(a)

Haptics& Haptics::GetInstance() {
    static Haptics h;
    return h;
}

bool Haptics::MotorAllowed() {
    return Application::GetInstance().GetDeviceState() != kDeviceStateListening;
}

void Haptics::Init(i2c_master_bus_handle_t bus) {
    if (queue_ != nullptr) return;
    Drv2605l* drv = new (std::nothrow) Drv2605l();
    if (drv == nullptr || !drv->Init(bus)) {
        delete drv;
        return;   // logged once by the driver; every call stays a no-op
    }
    drv_ = drv;
    queue_ = xQueueCreate(8, sizeof(Cmd));
    edge_done_ = xSemaphoreCreateBinary();
    if (queue_ == nullptr || edge_done_ == nullptr ||
        xTaskCreate(TaskThunk, "haptics", 4096, this, 3, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Could not start haptics task: haptics disabled");
        return;   // available_ stays false; objects are leaked deliberately (boot-time, once)
    }
    available_ = true;
#if SOFTTOY_HAPTICS_SELFTEST
    ESP_LOGI(TAG, "Boot self-test: buzz, giggle, heartbeat, wiggle, purr (300 ms apart)");
    Enqueue(Cmd::kSelfTest, true);   // edge=true: no gate lookup (Application may still be constructing)
#endif
}

bool Haptics::Enqueue(Cmd cmd, bool edge) {
    if (!available_) return false;
    if (!edge && cmd != Cmd::kPurr && !MotorAllowed()) {
        ESP_LOGD(TAG, "Dropped effect %d: mic is capturing", (int)cmd);
        return false;
    }
    pending_++;
    last_active_us_ = esp_timer_get_time();
    // Edge-start jumps the queue so it never waits behind an effect.
    BaseType_t ok = (cmd == Cmd::kEdgeStart) ? xQueueSendToFront(queue_, &cmd, 0)
                                              : xQueueSendToBack(queue_, &cmd, 0);
    if (ok != pdTRUE) {
        pending_--;
        return false;
    }
    return true;
}

void Haptics::Buzz()      { Enqueue(Cmd::kBuzz, false); }
void Haptics::Giggle()    { Enqueue(Cmd::kGiggle, false); }
void Haptics::Heartbeat() { Enqueue(Cmd::kHeartbeat, false); }
void Haptics::Wiggle()    { Enqueue(Cmd::kWiggle, false); }

void Haptics::Purr(bool on, PurrOwner owner) {
    if (!available_) return;
    const uint8_t bit = static_cast<uint8_t>(owner);
    const uint8_t keep = static_cast<uint8_t>(~bit);
    const uint8_t owners = on ? static_cast<uint8_t>(purr_owners_.fetch_or(bit) | bit)
                              : static_cast<uint8_t>(purr_owners_.fetch_and(keep) & keep);
    if (owners == 0) {
        purr_wanted_ = false;   // no owner left: the task aborts within ~10 ms
        return;
    }
    if (!on) return;                           // another owner still wants it: keep purring
    if (!MotorAllowed()) {                     // dropped: mic is capturing
        purr_owners_.fetch_and(keep);
        return;
    }
    if (purr_wanted_.exchange(true)) return;   // already purring
    if (!Enqueue(Cmd::kPurr, false)) {
        purr_wanted_ = false;
        purr_owners_.fetch_and(keep);
    }
}

void Haptics::BuzzListenStart() {
    if (!available_) return;
    purr_wanted_ = false;
    purr_owners_ = 0;   // Listening cuts the purr for every owner
    edge_start_cancel_ = false;
    xSemaphoreTake(edge_done_, 0);   // clear a stale give
    if (!Enqueue(Cmd::kEdgeStart, true)) return;
    // Hold capture back until the click has finished (a strong click is ~50 ms).
    if (xSemaphoreTake(edge_done_, pdMS_TO_TICKS(200)) != pdTRUE) {
        edge_start_cancel_ = true;   // too slow: skip it rather than buzz into the mic
        ESP_LOGW(TAG, "Listen-start buzz timed out, skipped");
    }
}

void Haptics::BuzzListenEnd() {
    if (!available_) return;
    Enqueue(Cmd::kEdgeEnd, true);
}

bool Haptics::MotorQuietFor(uint32_t ms) const {
    if (!available_) return true;
    if (IsPlaying()) return false;
    return (esp_timer_get_time() - last_active_us_.load()) >= (int64_t)ms * 1000;
}

void Haptics::MarkActive() { last_active_us_ = esp_timer_get_time(); }

void Haptics::TaskThunk(void* arg) { static_cast<Haptics*>(arg)->Task(); }

void Haptics::Task() {
    Cmd cmd;
    for (;;) {
        if (xQueueReceive(queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            Handle(cmd);
        }
    }
}

// Play one sequence and wait for it to finish. Gated effects are cut if the mic gate closes;
// a purr step is also cut when Purr(false) arrives.
void Haptics::PlayAndWait(const uint8_t* seq, uint8_t n, bool respect_gate, uint32_t max_ms) {
    auto* drv = static_cast<Drv2605l*>(drv_);
    if (respect_gate && !MotorAllowed()) return;
    MarkActive();
    if (!drv->Play(seq, n)) {
        ESP_LOGW(TAG, "Effect start failed (I2C)");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    uint32_t waited = 10;
    while (drv->Busy() && waited < max_ms) {
        if (respect_gate && !MotorAllowed()) {
            drv->Stop();
            break;
        }
        if (seq == kSeqPurrStep && !purr_wanted_) {
            drv->Stop();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
    if (waited >= max_ms) drv->Stop();
    MarkActive();
}

void Haptics::RunPurr() {
    if (purring_.exchange(true)) return;   // already looping (nested command): nothing to do
    while (purr_wanted_ && MotorAllowed()) {
        PlayAndWait(SEQ(kSeqPurrStep), true, kMaxEffectMs);
        // Gap between steps; a queued command wakes us early and is served inline.
        Cmd next;
        if (xQueueReceive(queue_, &next, pdMS_TO_TICKS(kPurrGapMs)) == pdTRUE) {
            Handle(next);
        }
    }
    purr_wanted_ = false;
    purring_ = false;
}

void Haptics::Handle(Cmd cmd) {
    switch (cmd) {
        case Cmd::kBuzz:
            PlayAndWait(SEQ(kSeqBuzz), true, kMaxEffectMs);
            break;
        case Cmd::kGiggle:
            PlayAndWait(SEQ(kSeqGiggle), true, kMaxEffectMs);
            break;
        case Cmd::kHeartbeat:
            PlayAndWait(SEQ(kSeqHeartbeat), true, kMaxEffectMs);
            break;
        case Cmd::kWiggle:
            PlayAndWait(SEQ(kSeqWiggle), true, kMaxEffectMs);
            break;
        case Cmd::kEdgeStart:
            // Runs while the state is already Listening but capture has not started yet.
            if (!edge_start_cancel_.exchange(false)) {
                PlayAndWait(SEQ(kSeqEdge), false, 200);
            }
            xSemaphoreGive(edge_done_);
            break;
        case Cmd::kEdgeEnd:
            vTaskDelay(pdMS_TO_TICKS(kEdgeEndDelayMs));   // capture has stopped by now
            if (MotorAllowed()) {                          // not Listening again already
                PlayAndWait(SEQ(kSeqEdge), false, 200);
            }
            break;
        case Cmd::kPurr:
            // The loop is tracked by purring_, not pending_.
            pending_--;
            RunPurr();
            MarkActive();
            return;
        case Cmd::kSelfTest: {
            vTaskDelay(pdMS_TO_TICKS(2000));   // let Application finish constructing (the gate reads its state)
            const uint8_t* seqs[] = {kSeqBuzz, kSeqGiggle, kSeqHeartbeat, kSeqWiggle, kSeqPurrStep, kSeqPurrStep};
            const uint8_t lens[] = {sizeof(kSeqBuzz), sizeof(kSeqGiggle), sizeof(kSeqHeartbeat),
                                    sizeof(kSeqWiggle), sizeof(kSeqPurrStep), sizeof(kSeqPurrStep)};
            for (int i = 0; i < 6; i++) {
                PlayAndWait(seqs[i], lens[i], true, kMaxEffectMs);
                vTaskDelay(pdMS_TO_TICKS(kSelfTestGapMs));
            }
            break;
        }
    }
    pending_--;
}
