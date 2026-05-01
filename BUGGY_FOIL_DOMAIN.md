# Tow Buggy + Foil Surfing — Domain Context for BREmote V2.5-Evo

## What the Buggy Is
The tow buggy ("buggy" or "boogie") is a motorized watercraft — roughly the size
and role of a small powered surfboard or mini jet ski. It does not carry a rider.
It tows the foil surfer behind. The user holds the remote control (TX) at all
times while in the water. The buggy is the motor; the foiler is the payload.

## Steering Types
Three distinct steering modes exist in this system. Understanding the difference
is critical for safe code changes.

**Tow steering (passive):** During the tow phase, the foiler is behind the buggy
on the tow rope. If the foiler leans or angles right, rope tension pulls the buggy
tail right — the buggy turns left. Vice versa for left. This is pure physics with
no code involvement. The foiler steers the buggy by body angle and rope tension.

**BREmote steering (manual):** The user operates the remote's steering toggle
directly. This is explicit user input. No autonomy involved. Active in normal
operation and optionally active during RTM as course correction (see below).

**Autonomous steering:** GPS and compass-guided heading corrections applied by
the firmware. Used exclusively in RTM (Return to Me) mode, where the buggy must
navigate back to the foiler's GPS coordinates. Default ON during RTM. The user
may apply BREmote steering during RTM to course-correct — this is called
course correction and is the only manual override during RTM.

FM (Follow Me) mode is designed to use autonomous steering when engaged — GPS-guided
  heading corrections to trail the foiler, combined with distance-based throttle gating.
  See FM section below. Note: FM autonomous steering is not yet implemented (Priority 9, future work).

## The Tow Phase
At the start of a session, the foiler holds the tow rope attached to the back of
the buggy. The buggy accelerates and pulls the foiler up to speed. During this phase:
- The buggy is ahead; the foiler is directly behind on the rope.
- Steering is via tow rope only (passive, see above). No BREmote steering needed.
- The user holds the throttle trigger to maintain speed.
- Releasing the trigger stops the motor immediately.

## The Whip / Wave Catch
When a wave approaches, the foiler uses the buggy's speed and rope tension to
slingshot ("whip") into the wave at an angle. The sequence:
1. Foiler angles toward the wave using rope tension.
2. Buggy momentum and rope snap accelerate the foiler onto the wave face.
3. Foiler releases the rope and begins foiling independently on the wave.
4. Rope goes slack. Buggy continues forward on its last heading, now unmanned.

This is the critical transition: from this moment, the foiler and buggy are
separated and moving independently.

## Post-Release Geometry
After the rope is released:
- The foiler is typically lateral to or ahead of the buggy, riding the wave.
- The buggy continues forward until the user releases the throttle trigger.
- The foiler may end up in any direction relative to the buggy — assume any
  geometry is possible after release.

## Follow Me (FM) Mode — Operational Model
FM is designed for the post-release phase. Once the foiler has caught the wave
and separated from the buggy, FM allows the buggy to trail the foiler.

**FM uses autonomous steering when engaged.** The buggy applies GPS-guided heading
corrections to maintain a configured lateral offset relative to the foiler (behind-left,
directly behind, behind-right — set via SPIFFS before the session). Throttle is gated
by follow distance: the buggy only moves when the foiler is beyond the minimum follow
distance, and throttle ramps gradually to avoid sudden engagement.

*Note: FM autonomous steering and GPS following are not yet implemented in firmware.
The current code supports FM mode selection (F0-F3) via gesture and displays the R5
proximity bar. Full autonomous FM following is Priority 9 (future work).*

**Throttle model (non-negotiable safety rule):**
- The user must hold the throttle trigger for the buggy to move — always.
- FM is a distance-based throttle gate. It can only reduce user throttle, never add.
- Hard stop zone (inner radius): if the buggy gets closer to the foiler than this
  threshold, throttle is cut to zero completely. The buggy stops. Hard limit,
  no override.
- Throttle ramp (activation gate): when FM activates (foiler exits the minimum
  follow distance), throttle ramps from 0 to the configured maximum over 3–4
  seconds. This prevents sudden full-speed engagement that could flip or wheelie
  the buggy. Maximum FM throttle is SPIFFS-configurable (default ~75%).
  Implementation TBD: ramp may trigger at the bubble boundary or on FM activation —
  to be decided in Priority 9.
- User releases trigger → motor stops immediately, regardless of FM state.

FM activates only after the foiler has cleared the minimum follow distance.

## Return to Me (RTM) Mode — Operational Model
RTM is used when the foiler has ridden far down the wave (or around a point) and
the buggy is stranded a significant distance away. The foiler activates RTM and
the buggy navigates back to the foiler's GPS coordinates using autonomous steering.

RTM requires large physical separation between foiler and buggy by definition.
RTM and FM are physically incompatible states:
- FM requires proximity (buggy following at safe distance behind foiler).
- RTM requires large separation (buggy returning from far away).
This geometric incompatibility reinforces the code-level mutual exclusion of FM
and RTM. If RTM arms while FM is active, FM must be explicitly disarmed and RX
must be notified (0xF2/0 packet) before RTM proceeds.

## Safety Philosophy (Non-Negotiable for All Code)
1. The buggy ONLY moves when the user physically holds the throttle trigger.
2. Autonomous modes (RTM, FM) can ONLY subtract from user throttle, never add.
3. Releasing the trigger stops the motor at the hardware level — no mode overrides this.
4. Manual control must always work even if RTM, FM, GPS, or compass systems fail.
5. Any code change that allows motor movement without active user throttle input
   must be rejected immediately before any further discussion.

## Edge Cases the System Does Not Solve
- Foiler falls and cannot hold the remote: trigger released, buggy stops.
- Buggy hit by a wave and changes heading: RTM self-corrects via GPS/compass;
  FM's intended design includes heading correction; current implementation is throttle-distance gating only (Priority 9 pending).
- Foiler ends up behind the buggy after a turn: hard stop zone prevents collision
  approach; FM has no directional awareness beyond distance.
- Rope wrap or entanglement: outside scope of software — operational procedure.
- Foiler whips into wave at full throttle while inside hard stop zone: buggy does
  not move. Foiler must clear the hard stop radius before throttle engages.
