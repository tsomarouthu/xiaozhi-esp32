#ifndef _HAPTICS_H_
#define _HAPTICS_H_

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <cstdint>

// Haptics for the soft toy (DRV2605L + coin ERM motor). Every call just queues a command to a
// small task, so callers never block on I2C. If the DRV2605L is missing everything is a no-op.
//
// MIC GATE (single rule, shared with M3-M5): the motor never runs while the mic is capturing,
// i.e. while the device state is Listening. MotorAllowed() is that rule. Effects requested
// while it is false are DROPPED (not deferred), and a running purr is cut the moment Listening
// starts. The only exceptions are the two edge buzzes (BuzzListenStart / BuzzListenEnd):
// start fires and finishes before the state handler starts capture (the caller waits for it),
// end fires ~kEdgeEndDelayMs after leaving Listening, when capture has stopped.
//
// ACCELEROMETER GATE (for M3): do not read the LIS3DH while !MotorQuietFor(kAccelTailMs).
class Haptics {
public:
    static Haptics& GetInstance();

    static constexpr uint32_t kAccelTailMs = 150;    // LIS3DH tail after the motor stops
    static constexpr uint32_t kEdgeEndDelayMs = 80;  // wait for capture to stop before end buzz

    // Start the task and the DRV2605L on `bus`. Missing chip = logs once, no task, all no-ops.
    void Init(i2c_master_bus_handle_t bus);
    bool Available() const { return available_; }

    // The mic gate. true = the motor may run now (device state is not Listening).
    static bool MotorAllowed();

    // Effects (gated by MotorAllowed(); dropped otherwise).
    void Buzz();                // short click: touch buzz-back, pick-up, reward
    // Purr has two owners that can both want it (M2 speaking purr, M5 hug purr). It runs while at
    // least one owner wants it and stops when none does; off is always honoured. Listening cuts it
    // and clears every owner (the mic gate), so a hug purr does not resume after a turn.
    enum class PurrOwner : uint8_t { kSpeaking = 1, kHug = 2 };
    void Purr(bool on, PurrOwner owner = PurrOwner::kSpeaking);
    void Giggle();              // three quick double-clicks: foot tickle
    void Heartbeat();           // lub-dub: bedtime / calm
    void Wiggle();              // excited pulsing: homecoming / shake

    // Edge buzzes, allowed around Listening. Called by the board's state hook only.
    // Start waits (<= ~150 ms, main task) so capture begins after the motor has stopped.
    void BuzzListenStart();
    void BuzzListenEnd();

    // True from the moment an effect is queued until it has finished.
    bool IsPlaying() const { return pending_.load() > 0 || purring_.load(); }
    // True if the motor has been idle (and nothing queued) for at least `ms`.
    bool MotorQuietFor(uint32_t ms) const;

private:
    Haptics() = default;
    enum class Cmd : uint8_t { kBuzz, kGiggle, kHeartbeat, kWiggle, kEdgeStart, kEdgeEnd, kPurr, kSelfTest };
    static void TaskThunk(void* arg);
    void Task();
    bool Enqueue(Cmd cmd, bool edge);
    void PlayAndWait(const uint8_t* seq, uint8_t n, bool respect_gate, uint32_t max_ms);
    void Handle(Cmd cmd);
    void RunPurr();
    void MarkActive();

    bool available_ = false;
    QueueHandle_t queue_ = nullptr;
    SemaphoreHandle_t edge_done_ = nullptr;
    std::atomic<int> pending_{0};
    std::atomic<bool> purring_{false};
    std::atomic<bool> purr_wanted_{false};
    std::atomic<uint8_t> purr_owners_{0};   // bitmask of PurrOwner
    std::atomic<bool> edge_start_cancel_{false};
    std::atomic<int64_t> last_active_us_{0};
    void* drv_ = nullptr;   // Drv2605l*, owned by the task
};

#endif  // _HAPTICS_H_
