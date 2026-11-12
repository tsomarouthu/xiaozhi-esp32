#ifndef _DRV2605L_H_
#define _DRV2605L_H_

#include <driver/i2c_master.h>
#include <cstdint>

// Minimal DRV2605L haptic driver: ERM coin motor, ROM library 1, internal trigger, effects
// played through the 8-slot waveform sequencer. Blocking I2C: call from one task only
// (the Haptics task). If the chip is missing, Init() returns false and every call is a no-op.
class Drv2605l {
public:
    static constexpr uint8_t kMaxSlots = 8;
    // Sequencer slot values: 1..123 = ROM effect id, kWait(ms) = pause (10 ms units, <=1270 ms).
    static constexpr uint8_t Wait(uint16_t ms) { return 0x80 | (uint8_t)((ms > 1270 ? 1270 : ms) / 10); }

    // Probe 0x5A on the bus and configure ERM / library 1. Returns false (and logs once) if absent.
    bool Init(i2c_master_bus_handle_t bus);
    bool present() const { return present_; }

    // Load up to kMaxSlots entries (zero-terminated if fewer) and fire GO.
    bool Play(const uint8_t* seq, uint8_t count);
    // GO bit still set = sequence running.
    bool Busy();
    // Abort the current sequence.
    void Stop();

private:
    i2c_master_dev_handle_t dev_ = nullptr;
    bool present_ = false;
    bool Write(uint8_t reg, uint8_t val);
    bool Read(uint8_t reg, uint8_t* val);
};

#endif  // _DRV2605L_H_
