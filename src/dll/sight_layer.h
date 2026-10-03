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
#include "../common/shot_lag_logic.h"
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

// THE SIGHT CLOCK (2026-10-02): every stamp and every age in this layer -- rays, traces, the converged and written
// marks, the intent point -- is QueryPerformanceCounter time, in milliseconds (Sight_NowMs) or microseconds
// (Sight_NowUs). GetTickCount64 ticks in 15.6 ms steps, which is half of the 25-40 ms lag the shot-lag witness
// measures. The ray publishers (Sight_PublishRay, the auxiliary rays) stamp `sampleMs` THEMSELVES at the publish,
// so no caller's clock can reach a reader; the trace driver's `nowMs` is Sight_NowMs() too. sight_layer.cpp reads no
// other clock (tools/check_sight_clock.py, a CTest guard).
uint64_t Sight_NowMs();
uint64_t Sight_NowUs();

struct SightRay
{
    uint32_t generation = 0;
    uint64_t sampleMs = 0;  // Sight_NowMs(), stamped by Sight_PublishRay (a caller's value is ignored)
    float head[3]{};        // the rendered centre eye: the intent ray starts here
    float dir[3]{};         // unit controller direction
    float eye[3]{};         // the engine eye the title aims its shots from
    int32_t unit = -1;      // the local unit the ray belongs to (the trace ignores it)
    int32_t ignoreB = -1;   // further objects the trace passes through (seated: the vehicle), or -1
    int32_t ignoreC = -1;
    // The engine eye's velocity (world units per second) over ~100 ms, estimated on the publishing thread from the
    // published eyes (shot_lag::EyeTrack, Sight_PublishRay fills it); eyeVelValid false = unknown, never 0.
    float eyeVel[3]{};
    bool eyeVelValid = false;
    // When `eye` was sampled (Sight_NowUs; 0 = at the publish). A title whose eye comes from a render passes that
    // render's stamp, so the velocity is measured on the eye's own clock.
    uint64_t eyeStampUs = 0;
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
// The world units per VR metre the title's converged aim last stamped (Sight_MarkConverged): the scale
// Game_SightReticleRange divided the traced range by. 0 before the first converged poll.
float Sight_UnitsPerVrMetre(GameTitle title);

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
// converged, the ray or trace is stale, or the geometry refuses (aim_solve::MuzzleShot, counted by reason; no
// distance-from-the-eye limit: the muzzle's identity is each title's weapon_muzzle receipt). The intent ray and engine eye are
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
bool Sight_AimMuzzleShotThrough(GameTitle title, const float point[3], const float rayDir[3],
                                const float before[3], const float position[3], float direction[3], float up[3]);
// A title that could not even ask (no converged point of its own): counted with the rest.
void Sight_NoteMuzzleShotUnconverged(GameTitle title);
// A seated redirect refused DEFENSIVELY because the gun points elsewhere, naturally (a seat held at its stop,
// a rate-capped turret): the shot leaves as the game fires it. The convergence claim is not touched.
void Sight_NoteMuzzleShotHeld(GameTitle title);

// THE INTENT POINT (shot_lag_logic.h; 2026-10-02). The aim path publishes its converged intent as a WORLD POINT P,
// not only as the angles it solved -- angles from a render-time eye land a frame or more later as a line parallel to
// the one meant, moved by the eye's travel in between (Reach: mean 93 mm, p90 187, max 307). A shot site that knows
// its own origin aims through P from there (Sight_ReaimThroughPoint). `point` comes from ONE frame-coherent
// hand/camera pair where the title has one (Halo 3, ODST: the render that used the hand's head sample); `solvePoint`
// is the point the published angles were bent through (the same unless the pair was older). Halo 3, ODST and Halo 4
// publish from Game_ComputeAimStick; Reach keeps its own lane (ReachSightReadActuated). Any thread reads.
struct SightIntent
{
    shot_lag::IntentPoint p{};
    float solvePoint[3]{};
};
void Sight_PublishIntent(GameTitle title, const SightIntent& intent);
bool Sight_ReadIntent(GameTitle title, SightIntent& out);
// The intent's verdict for a shot site now: fresh within maxAgeMs, this generation, the converged aim live, no
// natural hold, the solve not held at the pitch rail. `out` is filled whenever an intent was read.
shot_lag::IntentUse Sight_IntentForShot(GameTitle title, uint64_t maxAgeMs, bool naturalHeld, SightIntent& out);
inline constexpr uint64_t kSightIntentShotMaxAgeMs = 150;

// SHOT RE-AIM SITES (every build): which of a title's shot sites re-aimed a query or shot through the intent point.
enum class SightShotSite : uint8_t
{
    CrosshairAssist = 0,   // the title's own crosshair / firing-assist collision query
    UnitAdjust,            // Reach's unit-adjust (the shot's own origin and direction)
    Muzzle,                // shots from the muzzle (Sight_AimMuzzleShot)
    Count
};
// Re-aims `vector` (from `start`) through `point`: true and `vector` replaced when it applied; false leaves it as the
// game built it. Counted per title and site with the reason (Sight_Report logs "SHOT REAIM" every 2 s). `seated`: a
// seat's query, counted apart as well -- with the seat's record chosen upstream of replication (Halo 3,
// seat_upstream_logic.h) the game's own line should already pass through P, so a seated re-aim that still acts (and
// its bend) is the MULTIPLAYER RESIDUAL: what a host firing from the record alone would miss by.
bool Sight_ReaimThroughPoint(GameTitle title, SightShotSite site, const float start[3], float vector[3],
                             const float point[3], bool seated = false);
// A shot site that did not try (the intent was not usable): counted with its reason.
void Sight_NoteReaimSkipped(GameTitle title, SightShotSite site, shot_lag::IntentUse why);

// SHOT LAG WITNESS (dev; empty in player builds). Per shot, after the native call decided its final ray: the miss of
// the shot line from the point the player meant, split along / across the player's travel, with the eye's speed and
// the stale-eye term (shot origin minus the eye the solve used). One window per title and PATH (how the shot was
// aimed), logged by the cold worker every 2 s:
//   SHOT LAG <title> [<path>; <mode>] (dev): N shots; speed 0-1 m/s: n, miss X mm (along a, across c); 1-4: ...;
//     4-10: ...; 10+: ...; mean lag axis along travel B mm; stale eye along travel S mm (implied dt T ms);
//     at range R m; seated=0/1 seat=<id>
// `mode`: the fire's own `predicted` flag where the title passes one (Halo 3 / ODST distributed simulation predicts
// local shots outside the campaign), and the measured simulation tick. Read-only: nothing decides on it.
enum class SightShotPath : uint8_t
{
    Native = 0,        // the game's own aim, as the record put it
    CrosshairAssist,   // the crosshair / firing-assist query (after any re-aim)
    Muzzle,            // shots from the muzzle
    UnitAdjust,        // Reach's unit-adjust, after any re-aim
    Count
};
// `reference` = P (world units); `solveEye` = the eye the solve used (may be null); `eyeVel` (wu/s, may be null).
// `predicted`: -1 unknown, else the fire's flag.
void Sight_NoteShotLag(GameTitle title, SightShotPath path, const float origin[3], const float direction[3],
                       const float reference[3], const float* solveEye, const float* eyeVel, int predicted,
                       bool seated, int32_t seat);
// The same, with the reference, eye and velocity from this title's published intent point (else its ray and trace).
void Sight_NoteShotLagFromIntent(GameTitle title, SightShotPath path, const float origin[3], const float direction[3],
                                 int predicted, bool seated, int32_t seat);
// The same for a title that publishes no intent (Halo 2, CE): the point is the caller's own intent ray (head, dir) at
// the range the aim converged on, the eye its own; the velocity is measured from these eyes by the witness itself.
void Sight_NoteShotLagFromRay(GameTitle title, SightShotPath path, const float origin[3], const float direction[3],
                              const float head[3], const float rayDir[3], const float eye[3], bool seated);
// A simulation tick of `title` seen by a per-tick hook (dev): the witness reports the measured tick period.
void Sight_NoteSimTick(GameTitle title);
// How the aim path built this title's intent point (dev): 0 this poll's hand with the render of its own frame, 1 an
// earlier hand with the render of its frame (this poll's frame not rendered yet), 2 no kept hand has a rendered
// frame (the solve's own pair), 3 the title publishes no camera frames (Halo 4).
void Sight_NoteIntentPairing(GameTitle title, int kind);

// Cold worker: one status line per title that is tracing, every 2 s.
void Sight_Report();

// AN AUXILIARY RAY: one the title's simulation-thread trace runs beside the intent ray, in the same tick, with the
// same native query, flags and ignores, into a slot of its own -- from a shot origin along a direction the game
// really aims, to the first surface. Nothing the aim converges on reads it: the intent ray, its trace, the smoothed
// range, the scope, the shots and the writes are untouched.
struct SightAuxRay
{
    uint32_t generation = 0;
    uint64_t sampleMs = 0;   // Sight_NowMs(), stamped by the publish (a caller's value is ignored)
    float origin[3]{};   // the game's shot origin (the engine eye on foot; the seat's eye in a seat)
    float dir[3]{};      // unit, the direction the game aims along
};

// SEAT LIMIT RAY (all builds; src/common/seat_limit_reticle_logic.h). While a seat's reticle is clamped at a natural
// angular stop (a passenger's cone, a turret's or a tank gun's pitch stop), the aim path publishes the ray the gun
// really aims along -- the seat's engine eye along the limit-clamped direction -- and the title's trace runs it as an
// auxiliary ray, so the reticle sits on the surface the gun's shots meet. Only while a seat is clamped: nothing is
// traced otherwise. rangeWu: from the ray's origin; on a miss the caller passes 0 and a read reports the far end.
void Sight_PublishLimitRay(GameTitle title, const SightAuxRay& ray);
bool Sight_ReadLimitRay(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightAuxRay& out);
void Sight_PublishLimitTrace(GameTitle title, uint32_t generation, bool hit, float rangeWu);
bool Sight_ReadLimitTrace(GameTitle title, uint32_t generation, uint64_t maxAgeMs, bool& hit, float& rangeWu);
// The seat's reticle is clamped at a natural stop (stamped by each aim poll that clamps it; live for 150 ms). While
// it holds, a muzzle redirect is refused (the shot leaves along the game's clamped aim, through the clamped mark,
// never past the stop) and the zoom lens keeps the game's aim instead of the hand's ray. Any thread.
void Sight_MarkSeatLimitClamped(GameTitle title);
bool Sight_SeatLimitClampedLive(GameTitle title);
// Dev witness (empty in player builds): each clamped poll's edges and overshoot, and every 2 s per title
//   SEAT LIMIT <title>: reticle clamped N polls (yaw edge Y, pitch edge P, gun band G), max overshoot X deg
// `gunBand`: a look-steered driver's shot held at its guns' elevation band (Halo 3; halo3_vehicle_gun_band.inl).
void Sight_NoteSeatLimit(GameTitle title, bool yawEdge, bool pitchEdge, float overshootDeg, bool gunBand = false);

#if HALOMCCVR_DEV_DIAGNOSTICS
// [assess-only begin]
// DEV ASSESSMENT RAY (honest_reticle_assess; dev builds only, src/common/assess_reticle_logic.h). The reticle the
// assessment draws is the point the game's shot really reaches: from the game's own shot origin along the game's
// own aim, to the first surface. The aim path publishes that ray here (only while the key draws this title); the
// title's simulation-thread trace traces it as an auxiliary ray (above) into slots of its OWN: it never shares a
// slot or a latch with the seat limit ray, and a player build does not contain it.
using SightAssessRay = SightAuxRay;
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
