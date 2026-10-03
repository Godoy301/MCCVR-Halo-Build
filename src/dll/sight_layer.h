#pragma once

// SIGHT LAYER -- the title-agnostic half of trace-and-reaim for every title other than Reach (Reach keeps
// its own lane, reach_sight_trace.inl, which this generalises). It is only plumbing between threads:
//
//   1. the aim path (the input poll, or whichever thread builds the controller aim for that title)
//      publishes the INTENT RAY: the rendered centre eye it starts at, the controller's direction, and
//      the ENGINE EYE the title aims its shots from -- all in the title's own world units;
//   2. the title's simulation-thread hook traces that ray with the title's own collision query and
//      publishes the range, which the layer also SMOOTHS for the aim: the bend goes as 1/range, so the
//      inverse range is averaged over ~120 ms (a sweep off a near edge would otherwise snap the aim in
//      one trace), and a miss counts as the trace's far end, as Reach's aim does (SightSmoothInverseRange);
//   3. the aim path bends its aim so the engine's line from the engine eye passes through the point at
//      that range (aim_solve::converge_direction), actuates it, and marks the aim in effect as converged;
//   4. the compositor draws the reticle on the eye-anchored ray at the traced depth while it is.
//
// A title that cannot trace simply never publishes a result, and its aim path keeps its old behaviour:
// the fallback runs only when this cannot resolve. The player key converged_aim = 0 turns the whole layer
// off (no ray, no trace, no convergence): every title then aims as it did before it.

#include "../common/runtime_types.h"
#include "../common/aim_solve_logic.h"
#include "../common/converge_policy.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

// The intent ray is traced 1 km out, like Reach's and Halo: Campaign Evolved's (a Blam world unit is 10 ft).
inline constexpr float kSightWorldUnitsPerMetre = 1.0f / 3.048f;
inline constexpr float kSightTraceLengthWu = 1000.0f * kSightWorldUnitsPerMetre;
inline constexpr float kSightCmPerWorldUnit = 304.8f;
// A trace result older than this is not used (the trace runs every ~30 ms): no fresh result means the
// simulation stalled or the trace is off, and the shot genuinely cannot be resolved.
inline constexpr uint64_t kSightFreshMs = 150;
inline constexpr uint64_t kSightTraceIntervalMs = 30;
// A published ray older than this is not traced (and asking is what keeps the trace running).
inline constexpr uint64_t kSightRayFreshMs = 100;
// Start 5 cm down the ray so the trace clears the head it starts in; ranges are still from the ray origin.
inline constexpr float kSightTraceSkipWu = 0.05f * kSightWorldUnitsPerMetre;
// The time constant of the inverse-range average the aim converges on (Reach's: reach_aim_provider.cpp).
inline constexpr float kSightRangeTauMs = 120.0f;

// One trace's step of the range the aim converges on, kept as the INVERSE range (the bend goes as 1/range)
// and averaged over kSightRangeTauMs, so a sweep off a near edge onto a far wall does not snap the aim in
// one trace. A MISS means the target is at least the whole trace away, so it feeds the far end: a sustained
// miss converges far, exactly as Reach's aim does (Game_ReachSightSample hands its provider the trace
// length on a miss). Holding the last range instead kept Halo 4's aim converged on a 1.6 m spawn hit
// through minutes of sky, bending every far shot by the eye-to-hand parallax (FA, Forge, 2026-09-29).
// `continuous` false (a new generation, or a gap in the traces) starts afresh from this sample.
inline float SightSmoothInverseRange(float invPerWu, bool continuous, bool hit, float rangeWu, float dtSeconds)
{
    if (!continuous)
        invPerWu = 0.0f;
    const float sampleWu = hit && std::isfinite(rangeWu) && rangeWu > 0.0f ? rangeWu : kSightTraceLengthWu;
    return aim_solve::ema_inverse_range(invPerWu, sampleWu,
        aim_solve::ema_alpha(kSightRangeTauMs, std::clamp(dtSeconds, 0.001f, 0.1f)));
}

struct SightRay
{
    uint32_t generation = 0;
    uint64_t sampleMs = 0;
    float head[3]{};        // the rendered centre eye: the intent ray starts here
    float dir[3]{};         // unit controller direction
    float eye[3]{};         // the engine eye the title aims its shots from
    int32_t unit = -1;      // the local unit the ray belongs to (the trace ignores it)
    int32_t ignoreB = -1;   // further objects the trace passes through (seated: the vehicle), or -1
    int32_t ignoreC = -1;
};

struct SightTrace
{
    uint32_t generation = 0;
    uint64_t sampleMs = 0;
    bool hit = false;
    float rangeWu = 0.0f;         // this trace's own result; on a miss the trace length
    float convergeRangeWu = 0.0f; // the smoothed range the aim converges on (a miss counts as the far end); 0 = no trace yet
    // The range the aim was bent through: what a shot check must measure against.
    float AimRangeWu() const { return convergeRangeWu > 0.0f ? convergeRangeWu : rangeWu; }
};

// The player key converged_aim: false turns the layer off (no ray, no trace, no convergence).
bool Sight_AimEnabled();

void Sight_PublishRay(GameTitle title, const SightRay& ray);
bool Sight_ReadRay(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightRay& out);
// The simulation-thread trace driver's rate limit: true at most once per kSightTraceIntervalMs per title.
bool Sight_TraceDue(GameTitle title, uint64_t nowMs);
void Sight_PublishTrace(GameTitle title, uint32_t generation, bool hit, float rangeWu, uint64_t costTicks);
bool Sight_ReadTrace(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightTrace& out);
void Sight_NoteTraceFault(GameTitle title);
bool Sight_TraceFaulted(GameTitle title);
void Sight_ResetTitle(GameTitle title);   // a new title generation: forget faults and stale state

// The aim now in effect for `title` is a converged one. `unitsPerVrMetre` is the title's current VR world
// scale (world units per metre of VR space), which the compositor needs to put the reticle at the depth
// the traced surface appears at.
void Sight_MarkConverged(GameTitle title, float unitsPerVrMetre);
bool Sight_ConvergedLive(GameTitle title);
// ...and it is WRITTEN: this update the game's aim IS the converged aim (direct drive, Halo 2's aim write,
// CE's control packet), not a target the stick servo is still turning the aim toward at the game's turn
// rate. The traced reticle needs only the first; the zoom lens needs both (Sight_ScopeConvergedLive).
void Sight_MarkAimWritten(GameTitle title);
bool Sight_AimWrittenLive(GameTitle title);

// One-shot convergence for a title that aims with a direction: bend `dir` (unit, world) from `head`
// through the point at the smoothed traced range, as seen from `eye`. False (out untouched) when there is
// no fresh trace for this generation, no hit yet, or the geometry cannot resolve. A bent aim steeper than
// `maxPitchDeg` (the steepest pitch the caller's actuation can reach) is HELD AT THAT PITCH and still
// returned as converged (2026-10-01): a close ceiling or floor is natural geometry, not our code failing.
// `outWhy`, when given, receives why it declined (our code's reasons only: converge::ConvergeOff).
// `railHold`, when given, is the caller's hysteresis for that clamp (aim_solve::RailHold: one per actuation loop,
// owned by the thread that calls; a call that does not clamp, including every decline, forgets it).
bool Sight_ConvergedDirection(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                              const float eye[3], float out[3], float maxPitchDeg = 90.0f,
                              float* outRangeWu = nullptr, bool* outHit = nullptr,
                              converge::ConvergeOff* outWhy = nullptr, aim_solve::RailHold* railHold = nullptr);

// For the compositor (any thread): the active title's traced range along its eye-anchored intent ray, in
// VR METRES, and whether it hit -- only while that title's aim in effect is converged. Never Reach
// (Game_ReachSightReticleRange owns Reach).
bool Game_SightReticleRange(float& outRangeM, bool& outHit);

// SCOPE CONVERGENCE: a zoom lens at the RENDERED eye looks along the intent ray, which from that eye passes
// through the point the converged aim is bent through -- so the lens centre is the reticle's point, where
// looking along the game's aim (bent to leave from the engine eye) would run parallel to the shot line and
// off it by the eye's distance from the character. Only while the player key scope_converge is on and the
// converged aim is in effect AND written (Sight_ConvergedLive, Sight_AimWrittenLive): under the stick servo
// the shot follows a game aim still turning toward the target, and the lens keeps showing that aim.
// A DIRECTION, never a point: the lens origin is this frame's eye while the ray's head was sampled on another
// thread and frame, so a look-at point would swing the lens by the head's motion since that sample over the
// range, magnified by the zoom. Sight_ScopeConvergedDirection hands back the ray the aim path published
// (Halo 3, ODST, Halo 4); a title that builds its own ray from the lens origin asks Sight_ScopeConvergedLive
// (Halo 2). False: the lens keeps its own direction, as before. Any thread.
bool Sight_ScopeConvergedLive(GameTitle title);
bool Sight_ScopeConvergedDirection(GameTitle title, float outDir[3]);

// SHOTS FROM THE MUZZLE (the player key shots_from_muzzle; aim_solve::muzzle_shot_direction). Each title's
// barrel-shot transaction already moves a shot's origin to the visible muzzle and runs it through the
// title's own obstruction check (that is the barrel aim, gun_barrel_aim, which also takes the barrel's
// direction). With this key instead, the shot keeps that origin but flies through the CONVERGED POINT --
// the intent ray's point at the range the converged aim is bent through -- so it lands where the shot from
// the engine eye would have, and the tracer and projectile come out of the gun.
// Sight_MuzzleShotsConverge: the key is on and the barrel aim (which wins) is off.
// Sight_AimMuzzleShot: after the title's obstruction check moved the muzzle from `before` to `position`,
// replace `direction` and `up` and return true; false leaves all three untouched, and the title then
// refuses the barrel shot, so it leaves the eye exactly as before -- when the aim in effect is not
// converged, the ray or trace is stale, or the geometry fails a rail. The intent ray and engine eye are
// the published ray's (Halo 3, ODST, Halo 4), the caller's own (...FromRay: Halo 2, CE), or the caller's
// converged point itself (...Through: Reach, whose aim lane is its own). Any thread; counted per title.
// ...FromRay's pointDistance, when given, receives how far that point lies from the muzzle (world units) on
// success: a title whose firing assist aims at a point along the shot (Halo 2) aims at that one, from the
// same trace sample the direction was built from.
bool Sight_MuzzleShotsConverge();
bool Sight_AimMuzzleShot(GameTitle title, uint32_t generation, const float before[3], const float position[3],
                         float direction[3], float up[3]);
bool Sight_AimMuzzleShotFromRay(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                                const float eye[3], const float before[3], const float position[3],
                                float direction[3], float up[3], float* pointDistance = nullptr);
bool Sight_AimMuzzleShotThrough(GameTitle title, const float point[3], const float rayDir[3], const float eye[3],
                                const float before[3], const float position[3], float direction[3], float up[3]);
// A title that could not even ask (no converged point of its own): counted with the rest.
void Sight_NoteMuzzleShotUnconverged(GameTitle title);
// A seated redirect refused DEFENSIVELY because the gun points elsewhere, naturally (a seat held at its stop,
// a rate-capped turret): the shot leaves as the game fires it. The convergence claim is not touched.
void Sight_NoteMuzzleShotHeld(GameTitle title);

// Cold worker: one status line per title that is tracing, every 2 s.
void Sight_Report();

#if HALOMCCVR_DEV_DIAGNOSTICS
// [assess-only begin]
// DEV ASSESSMENT RAY (honest_reticle_assess; dev builds only, src/common/assess_reticle_logic.h). The reticle the
// assessment draws is the point the game's shot really reaches: from the game's own shot origin along the game's
// own aim, to the first surface. The aim path publishes that ray here (only while the key draws this title); the
// title's simulation-thread trace, in the same tick and with the same ignores as the intent ray, traces it too
// and publishes the result in a slot of its own. Nothing else reads either: the intent ray, its trace, the
// smoothed range the aim converges on, the scope, the shots and the writes are untouched.
struct SightAssessRay
{
    uint32_t generation = 0;
    uint64_t sampleMs = 0;
    float origin[3]{};   // the game's shot origin (the engine eye on foot; the seat's eye in a seat)
    float dir[3]{};      // unit, the game's own aim
};
void Sight_PublishAssessRay(GameTitle title, const SightAssessRay& ray);
bool Sight_ReadAssessRay(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightAssessRay& out);
// rangeWu: from the ray's origin; on a miss the caller passes 0 and a read reports the trace's far end.
void Sight_PublishAssessTrace(GameTitle title, uint32_t generation, bool hit, float rangeWu);
bool Sight_ReadAssessTrace(GameTitle title, uint32_t generation, uint64_t maxAgeMs, bool& hit, float& rangeWu);
// The assessment trace's OWN fault latch. A native fault in the assessment's trace switches off only the assessment
// trace for that title, until the title reinstalls (Sight_ResetTitle, or Reach's install); the title's intent trace
// (Sight_NoteTraceFault / its own faulted flag) belongs to the aim and is never touched by the assessment.
void Sight_NoteAssessTraceFault(GameTitle title);
bool Sight_AssessTraceFaulted(GameTitle title);
void Sight_ResetAssessTrace(GameTitle title);
// [assess-only end]
#endif

// CONVERGE-OFF WITNESS (dev; empty stubs in player builds, converge_policy.h). Natural in-game limits never
// switch the converged aim off any more (2026-10-01); these lines are how a headset session proves it.
// Sight_NoteConvergeState: each aim poll in gameplay (Halo 3, ODST and Halo 4 from Game_ComputeAimStick,
// Reach from its provider) says whether the converged aim is in effect and, if not, the FIRST reason -- which
// is always our code's or a state convergence does not apply to.
// Sight_NoteNatural: the natural causes that USED to switch it off, counted only -- no decision reads them.
// Sight_NaturalHeld: one of those held this title's aim off the point within `windowMs` (the dev engine-trace
// witnesses skip such polls, so their miss statistics measure our solve, not a seat's stop).
// Sight_ReportConverge (cold worker, every loop): an edge line when a title's converged aim goes off or comes
// back (rate-limited), and every 2 s per title
//   SIGHT <title> converge-off <off>/<polls> polls: <Reason>=<n>, ...
//   SIGHT <title> natural (no switch-off) in <n> polls: latchHeld=.. outcomeMiss=.. stall=.. turret=.. ...
void Sight_NoteConvergeState(GameTitle title, bool live, converge::ConvergeOff why);
void Sight_NoteNatural(GameTitle title, const converge::NaturalEvents& events);
bool Sight_NaturalHeld(GameTitle title, uint64_t windowMs);
void Sight_ReportConverge();

// SHOT WITNESS (dev; compiled out of player builds): a title's firing path reports each local shot's ray --
// origin and direction in world units, as the engine adjusted them -- and the cold worker logs, per title
// every 2 s, where those shots pass the surface under the reticle (the intent ray's latest trace) and the
// converged point, in the intent ray's own frame (up / right, mm), and where they left from relative to the
// engine eye and the rendered eye. "The shots land below the reticle" is then a number, and so is its cause.
void Sight_NoteShot(GameTitle title, uint32_t generation, const float origin[3], const float direction[3]);
void Sight_ReportShots();

// ENGINE-RAY CENSUS (dev; the callers gate it with the dev key sight_ray_census): which of the title's own
// collision queries is its crosshair / shot query? A title's collision hook reports each query the game
// makes with its caller's RVA. One that starts within 0.3 m of the solve's engine eye and runs within 2 deg
// of the eye -> traced-point direction counts as ALIGNED, with how far the traced point sits off its line.
// Needs the title's ray and trace published. Compare windows with and without shots: a caller that
// appears only on shots is the shot's own query. Compiled out of player builds (empty stubs).
void Sight_NoteEngineRay(GameTitle title, uint32_t generation, uint32_t callerRva, uint32_t profile,
                         uint64_t flags, const float start[3], const float end[3]);
// Cold worker, every 5 s: the census lines of every title that fed it.
void Sight_ReportEngineRays();
