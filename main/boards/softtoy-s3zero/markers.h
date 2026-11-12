#ifndef _MARKERS_H_
#define _MARKERS_H_

#include <functional>
#include <cstdint>
#include "device_state.h"

// Board-level event-marker dispatcher (M3; reused by M4 touch and M5 hug).
//
// Every touch / motion / pick-up / button event goes through Markers::Request(). It applies the
// run-mode rules from docs/INTEGRATION-PLAN.md "Marker conventions" and then hands the text to
// Application::SendEventMarker() with the MarkerStart that M0 assigned to that marker:
//   - per-marker rate limits and a 3 s global gap between any two markers (the button is exempt);
//   - pick-up: 1 per 3 h and none within 10 min after a session ends;
//   - a hit suppresses a head pat within ~300 ms;
//   - a limit is used up only when the app reports done(true) (sent / a start was begun);
//   - offline (Wi-Fi or server down): nothing is sent, OnOfflineMarker() is called instead (M6 hook);
//   - every decision is logged on one line: "marker <text> sent|dropped(<reason>)".
// Thread-safe: Request() may be called from any task.
namespace Markers {

enum class Id : uint8_t {
    kWakeButton,        // [wake: button]       kStartOnly, no limits
    kPickup,            // [greeting: pickup]   kStartOnly
    kHug,               // [hug]                kStartOrInSession, 60 s
    kTouchRightHand,    // [touch: right_hand]  kInSessionOnly, 10 s per pad
    kTouchLeftFoot,     // [touch: left_foot]
    kTouchRightFoot,    // [touch: right_foot]
    kTouchHead,         // [touch: head]        (dropped within 300 ms of a hit)
    kTouchTail,         // [touch: tail]
    kShake,             // [motion: shake]      kInSessionOnly, 15 s
    kUpsideDown,        // [motion: upside_down]
    kHit,               // [motion: hit]
    kLieDown,           // [motion: lie_down]
    kDoubleTap,         // [motion: double_tap]
    kCount
};

// The exact marker text for an id (ASCII, no double quote or backslash).
const char* Text(Id id);

// Board start-up: `is_online` answers "can the toy reach the server right now?" (Wi-Fi up and the
// server not known to be down). Without it everything counts as online.
void Init(std::function<bool()> is_online);

// Ask to send a marker. `done(accepted)` is optional: accepted=true means sent / a session was
// begun (runs on the main task); accepted=false means dropped, by the dispatcher (runs at once on
// the calling task) or by the app (main task). Local haptic reactions of the motion markers are
// fired here when the marker is accepted; touch and hug haptics belong to M4/M5.
void Request(Id id, std::function<void(bool accepted)> done = nullptr);

// Feed device-state edges (the board's state hook). Used for "10 min after a session ends".
void NoteDeviceState(DeviceState state);

// M6 hook: called instead of sending when the toy is offline and the marker passed its rate limit.
// Empty for now (logs). M6 plays the local clip here and may call Consume(id) when it did.
void OnOfflineMarker(Id id);

// Use up `id`'s rate limit and the global gap now (for M6 when it plays an offline clip).
void Consume(Id id);

}  // namespace Markers

#endif  // _MARKERS_H_
