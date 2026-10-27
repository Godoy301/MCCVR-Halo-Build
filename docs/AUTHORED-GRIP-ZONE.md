# Follow-up: larger acquisition and side-by-side authority

User accepts source6735313e Magnum placement and reacquisition recovery. New
log confirms use of authored zones (e.g. endpoint -0.090,-0.005,-0.359 with
mostly misses and no fallback), consistent with a restrictive acquisition area.

Expand long-grip sphere9cm to a primary-forward capsule:30cm centre segment,
12cm radius,54cm total length (original barrel interval72cm). Retain authored
centre, owner checks and existing grip retention. Short-grip box expands to
+/-22cm lateral,+/-14cm vertical,18cm forward/16cm rearward. No per-weapon list.

Keep distance Authority unchanged (zero<=16cm/full>=30cm). Multiply by a local
primary-controller directional guard: exactly zero for lateral separation at
<=8cm fore/aft and<=12cm vertical. Smooth transition returns full authority by
16cm fore/aft; vertical/dominantly longitudinal arrangements also fade out the
lateral guard. Uses physical grip samples, orientation transformed into primary
local space, so headset/world heading does not choose the rule. Applies through
the existing pre-calibration blend, preserving connected support and stock on/off.

New geometry/boundary/authority tests pass;307 extracted production aim checks
pass. Windows compilation/headset validation still pending. Commit only.

# Follow-up: acquisition availability and close-hand aiming

User log shows repeated hit=0/miss=0/fallback~400 intervals alternating with
fresh authored hits/misses; final intervals have only misses. This proves zone
availability changes, not a stuck durable latch. Previous producer stopped
publishing while attached and erased readiness during settling. Its100ms expiry
then reopened the legacy forward barrel zone, explaining far-front reacquisition.

Retain the last settled point for the exact owner while observing its live
primary invocation, including attachment. Pending learning never overwrites a
calibrated point. Owner changes clear readiness. Fresh reads still require100ms
heartbeat; known supported-but-pending/stale owners return no acquisition instead
of switching geometry. Completely unknown owners retain legacy fallback. Outside
attempts never mutate the publication; grip state machine is unchanged.

Use fresh input-frame controller grip positions where available, aim positions
otherwise. Authored separation <=24cm selects a generic close-grip volume:
+/-16cm lateral, +/-10cm vertical,10cm forward/12cm rearward. Longer separation
keeps the9cm authored sphere. These are initial ergonomic bounds, not measured
Magnum constants. No names/tags identify pistols. Logs now include local endpoint
XYZ for diagnosing classification and position in the next headset session.

All H3 standard-handed two-hand aim gets a frozen sample distance weight:
0 at <=16cm, smoothstep to1 at30cm. Blend solved orientation toward primary
before calibration, including Virtual Stock paths, while preserving grip/latch
and position. Other titles keep factor1. Traced/untraced calculations agree;
trace final direction reflects the blend. No native weapon animations changed.

Validation:307 production aim checks pass via extracted shipping solver using
minimal OpenXR value-struct stubs on Linux (not a Windows DLL build). New
publication/geometry/authority regressions and persistent-support wiring guard
pass. Headset confirmation pending, particularly short-grip classification and
repeated outside/inside attempts. No compilation workflow triggered.

Historical first-candidate design follows; availability and geometry rules above
supersede the prior behavior:

# Authored support-grip acquisition: first Halo 3 adapter

Branch two-handed-grip, based on the user-created branch (6e13ded0). Goal:
acquire pistol support near the dominant hand without per-weapon offsets.
Existing base and persistent paths require 0.08 < forward < 0.80m with 0.09m
lateral radius, regardless of weapon. Keep those paths as fallback.

H3 ReconstructVisiblePaletteSource already has exact local primary invocation
identity, untouched native wrist matrices, and desired primary wrist world pose.
The attached assembly carries the original wrist relation. Compute the left
wrist through desiredRight * inverse(authoredRight) * authoredLeft. Express its
separation from desiredRight in the controller basis (game forward/left/up to
OpenXR -Z/-X/+Y), divide by current world scale. Cosmetic assembly translation
is excluded: physical controller separation must not inherit gun offsets.
This uses the authored wrist, not a claim that every mod has a grip marker.

Publish through bounded atomic sequence/fields with title/generation/unit/weapon.
Consumer requires exact KnownPresent owner and <=100ms age. Initial acquisition
waits1500ms plus500ms within3cm; movement restarts the quiet window. No permanent
weapon reference cache. Model wrist/count changes and sample gaps reset learning.
This is a stability heuristic, not native reload detection. A quiet reload
plateau can qualify temporarily; later movement invalidates it and idle recovers.

Both acquisition paths try a 9cm sphere at the authored endpoint, with existing
zone-right nudge. Compare physical wrist positions, without legacy palm-forward
offset (which would displace pistol acquisition). Hold/toggle retention, release,
owner-switch handling, dual-wield inhibition and aim solver remain unchanged.
No source-tag writes/new hooks or per-weapon names. Initial scope H3 primary,
standard handedness; other titles/left-handed mode keep fallback. No promise of
all-game support yet. Nearby-controller aim jitter is a separate next-stage
issue; aim authority is deliberately unchanged for this acquisition test.

Portable tests pass for pistol/rifle separation, invalid inputs, identity/age,
stabilization, reload recovery and original fallback (69 existing checks).
Windows compilation and headset behavior pending. Test Magnum/Plasma Pistol,
AR/SMG, reload immediately after pickup, release/reacquire, switching weapons,
hold/toggle and virtual stock on/off. H3 authored grip zone hit/miss/fallback
counters report every5s on the existing cold tick. No build requested yet;
existing weapons-alignment workflow is branch-specific and not changed.
