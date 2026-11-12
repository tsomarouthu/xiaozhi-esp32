#ifndef _REACTION_CLIPS_H_
#define _REACTION_CLIPS_H_

#include <cstdint>
#include "markers.h"

// M6: short spoken reactions the toy plays by itself when a marker fires while it cannot reach the
// server. Clips are rendered by offline_clips/make_clips.ps1 (ids rc_*) and embedded through
// BOARD_SOUNDS; a clip that was never rendered is simply skipped (weak symbols), haptics still fire.
namespace ReactionClips {

// Play a random clip for `id` if allowed: device Idle, no other local clip still playing, and at
// least 20 s since the last reaction clip. Returns true if a clip was queued.
bool PlayFor(Markers::Id id);

// "Yay, I'm back!" once, only if the toy told her it was offline in the last 10 minutes.
// Returns true if played. Call from an idle, online moment.
bool PlayBackOnlineIfDue();

// A local status clip (offline wifi/server) was just spoken: remember it for the "back online"
// greeting and keep reaction clips from playing over it.
void NoteStatusClipSpoken(int64_t approx_ms);

}  // namespace ReactionClips

#endif  // _REACTION_CLIPS_H_
