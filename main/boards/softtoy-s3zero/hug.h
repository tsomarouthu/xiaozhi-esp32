#ifndef _HUG_H_
#define _HUG_H_

// Hug detector (M5). A hug = the LIS3DH says the toy is HELD (Lis3dh::IsHeld()) AND the belly pad
// (TouchPads) has been touched continuously for kHugHoldMs. One small task polls every 50 ms.
//   - Quick belly pat (< kHugHoldMs): nothing.   Pick-up alone (no belly): nothing.
//   - Belly on a table (not held): nothing.
// On a hug: Haptics purr (owner kHug, works online and offline), and one Markers::Request(kHug)
// (so a hug from idle may start a session online; offline M6 plays a clip). The purr is mic-gated:
// if the hug starts a session and the toy goes to Listening, the purr stops and does not resume.
// The hug ends on belly release (with a short grace for sensor glitches) or after kHugMaxMs.
// While the purr runs the LIS3DH is not read (motor gate), so IsHeld() is only a latched value
// there; that is why the end is decided by the belly, not by IsHeld().
namespace Hug {
// Start the polling task. Call after Haptics, Markers, Lis3dh and TouchPads init.
void Init();
}

#endif  // _HUG_H_
