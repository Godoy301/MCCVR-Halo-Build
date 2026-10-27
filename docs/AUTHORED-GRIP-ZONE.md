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
