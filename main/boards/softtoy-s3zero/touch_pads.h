#ifndef _TOUCH_PADS_H_
#define _TOUCH_PADS_H_

#include <cstdint>
#include <functional>

// Capacitive touch pads for the soft toy (M4): six copper-tape pads, each through 470 ohm, on the
// ESP32-S3 touch sensor (GPIO n = touch channel n). Pins come from docs/wiring.md via config.h.
//
// A small polling task reads the driver's smoothed data every 20 ms and does its own per-pad
// baseline, threshold, debounce, hysteresis and cross-talk arbitration (all constants are in one
// block at the top of touch_pads.cc). A debounced touch does, on the polling task:
//   - hand / head / tail: Haptics::Buzz(); feet: Haptics::Giggle(); then Markers::Request(<pad>)
//   - belly: no marker and no buzz of its own; it only feeds the belly API below (M5: [hug]).
// Haptics are mic-gated by Haptics itself (dropped while Listening). If the touch driver fails to
// start, Init() logs once and everything stays a no-op; boot is never blocked.
namespace TouchPads {

// Start the sensor and the polling task. Call once, after Markers::Init() and Haptics Init().
void Init();

// Belly pad (GPIO1), debounced and hysteresis-filtered. M5 fuses this with the LIS3DH "held"
// state into [hug]. All three are safe to call from any task.
bool BellyTouched();        // true from the debounced press until the debounced release
uint32_t BellyHeldMs();     // continuous touch time in ms; 0 when not touched

// Optional edge callback: `touched` = true on debounced press, false on release. It runs on the
// touch polling task (every 20 ms at most), so it must not block. Pass nullptr to clear.
void SetBellyCallback(std::function<void(bool touched)> cb);

}  // namespace TouchPads

#endif  // _TOUCH_PADS_H_
