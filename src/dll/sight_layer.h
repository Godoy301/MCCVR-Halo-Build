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

// One-shot convergence for a title that aims with a direction: bend `dir` (unit, world) from `head`
// through the point at the smoothed traced range, as seen from `eye`. False (out untouched) when there is
// no fresh trace for this generation, no hit yet, or the geometry cannot resolve -- including a bent aim
// steeper than `maxPitchDeg`, which must be the steepest pitch the caller's actuation can reach (a
// clamped aim is not a converged one).
bool Sight_ConvergedDirection(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                              const float eye[3], float out[3], float maxPitchDeg = 90.0f,
                              float* outRangeWu = nullptr, bool* outHit = nullptr);

// For the compositor (any thread): the active title's traced range along its eye-anchored intent ray, in
// VR METRES, and whether it hit -- only while that title's aim in effect is converged. Never Reach
// (Game_ReachSightReticleRange owns Reach).
bool Game_SightReticleRange(float& outRangeM, bool& outHit);

// SCOPE CONVERGENCE (scope_logic.h ComputeScopeLookAtDirection): the point a title's zoom camera looks AT --
// the intent ray at the smoothed traced range, the point the aim in effect is bent through -- while that aim
// is converged (Sight_ConvergedLive) and the player key scope_converge is on. Sight_ScopeConvergedPoint reads
// the ray the aim path published (Halo 3, ODST, Halo 4) and can hand back its head, the rendered eye; a title
// that builds its ray itself passes it in (Halo 2). False when there is none: the zoom keeps its own
// direction, as before. Any thread.
bool Sight_ScopeConvergedPoint(GameTitle title, float outPoint[3], float outHead[3] = nullptr);
bool Sight_ScopeConvergedPointOnRay(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                                    float outPoint[3]);
// A converged point nearer the zoom camera than this (world units) is not aimed at: the camera keeps its own
// direction rather than swing on a point it is nearly standing on.
inline constexpr float kSightScopeLookAtMinimumWu = 0.1f * kSightWorldUnitsPerMetre;

// Cold worker: one status line per title that is tracing, every 2 s.
void Sight_Report();

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
