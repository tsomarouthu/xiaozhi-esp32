#ifndef _LIS3DH_H_
#define _LIS3DH_H_

#include <driver/i2c_master.h>
#include <atomic>
#include <cstdint>

// LIS3DH accelerometer (I2C 0x19 on the shared bus) plus the gesture detector for the soft toy.
// Polled by its own small task (200 Hz, +-4 g, 12-bit, FIFO stream mode, drained every 20 ms).
// INT1 (GPIO17) is deliberately unused: the toy never sleeps (WIFI_PS_NONE) and every gesture,
// including double-tap, is detected in software, so there is nothing to wake.
//
// Gestures (marker via Markers::Request, see markers.h): pick-up, shake, upside-down, hit,
// lie-down, double-tap. All thresholds and their comments are at the top of lis3dh.cc.
//
// ACCELEROMETER GATE (M2): reads are skipped and discarded unless the motor has been quiet for
// Haptics::kAccelTailMs, and the detectors restart afterwards, so a buzz can never be a gesture.
//
// Missing chip (WHO_AM_I != 0x33): one warning, no task, every call is a no-op; boot never blocks.
class Lis3dh {
public:
    static Lis3dh& GetInstance();

    // Probe + configure + start the polling task on the shared bus (SoftToyI2c::Bus()).
    void Init(i2c_master_bus_handle_t bus);
    bool Available() const { return available_; }

    // True while the toy looks "in hand": small continuous motion over a sliding 3 s window
    // (see "Held" in lis3dh.cc). For M5's hug fusion (held + sustained belly pad). While the
    // motor gate discards reads (e.g. the hug purr) the last answer is latched for up to 30 s.
    // Thread-safe.
    bool IsHeld() const { return held_.load(); }

private:
    Lis3dh() = default;
    struct Impl;
    static void TaskThunk(void* arg);
    bool available_ = false;
    std::atomic<bool> held_{false};
    Impl* impl_ = nullptr;
};

#endif  // _LIS3DH_H_
