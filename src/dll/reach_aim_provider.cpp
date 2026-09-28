// Reach's aim provider (see reach_aim_provider.h). The engine-free core picks the actuation rung from
// caps(); this file supplies Reach's facts and actuates the rung it picks:
//   WriteState -- direct drive (reach_direct_drive=3): the converged setpoint is published to the
//                 sim-thread hook on haloreach+0x6375C, which writes player_control.desired_angles
//                 (upstream of the derived aim vector and of replication); the aim stick is held
//                 neutral so there is exactly one driver.
//   StickLoop  -- the shipped closed loop, Game_ComputeAimStick, unchanged.
// Convergence (reach_sight_trace.inl): TraceReaim while a FRESH traced hit exists -- the native shot
// stays at Reach's own origin and the setpoint is bent through the traced point -- else
// OriginRelocation, the older exact path (the firing site moves the shot onto the hand ray).
// Engine-bound (reads g_config and game.cpp's Reach state); the pure core it feeds is
// src/common/aim_solve_logic.h + aim_provider.h, and the write guards the hook applies are
// src/common/aim_write_logic.h.

#include "reach_aim_provider.h"

#include "../common/aim_provider.h"
#include "../common/reach_aim_basis.h"   // Reach <-> core units and angles (unit-tested)
#include "game.h"   // Game_ComputeAimStick (StickLoop) + the Game_ReachDirectDrive* WriteState seam
#include "../common/config.h"
#include "../common/log.h"

#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace {
using namespace aim_solve;
using reach_aim_basis::kCmPerWorldUnit;   // 304.8: a Reach world unit is 10 ft
// Trace-and-reaim bends the aim only on a range measured this recently. A miss, a stale trace or a
// gap never bends it on a held or assumed range: the older hand shot ray is exact at every range, so
// it is the right fallback whenever the range is not known.
constexpr uint64_t kFreshHitMs = 200;

const char* rung_name(Actuation how) {
    switch (how) {
    case Actuation::WriteState: return "WriteState";
    case Actuation::StickLoop:  return "StickLoop";
    default:                    return "None";
    }
}

class ReachAimProvider final : public IAimProvider {
public:
    const char* title() const override { return "reach"; }

    // The core reads caps() more than once per poll; every read sees this poll's snapshot, so a
    // cross-thread flip mid-poll cannot pair a trace-and-reaim intent with a relocation actuation.
    AimCaps caps() const override { return snapshot; }

    AimCaps compute_caps() const {
        AimCaps c;
        // Rung 1 is player_control.desired_angles, the accumulator's own record: after stick
        // conditioning, before the aim vector is derived, and persistent (one write steers the sim).
        // Offered only when opted in (reach_direct_drive=3), with the hook proven by the arm-gate AND
        // demonstrably running (its heartbeat -- a hook that stopped would otherwise leave the stick
        // neutral with nothing writing), and on foot -- vehicles and cinematics keep the stick loop,
        // which is proven there. Co-op host-follow is UNVERIFIED, so actuate() never claims a
        // replicated write.
        c.can_write_state       = g_config.reach_direct_drive == 3 && Game_ReachDirectDriveReady() &&
                                  Game_ReachDirectDriveOnFoot();
        c.has_stick_loop        = true;   // Reach steers via Game_ComputeAimStick, like Halo 3
        // TRACE-AND-REAIM (reach_sight_trace.inl): only with a fresh traced HIT and only while direct
        // drive can actuate the bent aim (a stick loop cannot hold an aim that bends with range).
        c.has_sightline_trace   = fresh_hit && c.can_write_state;
        c.has_origin_relocation = true;   // R-V10/R-V27 firing-site eye substitution, foot + vehicle
        c.has_fixed_range_reaim = true;   // ComputeAimStick steers through the crosshair-distance point
        c.has_barrel_marker     = g_config.gun_barrel_aim != 0;
        c.camera_follows_aim    = true;   // Reach writes the aim, so the view follows the hand
        return c;
    }

    // With trace-and-reaim the intent is the RENDERED hand/reticle ray itself (origin and direction in
    // world units -> cm): the core bends it through the traced point. Otherwise it is the aim
    // Game_ComputeAimStick recorded during this poll (the mount-calibrated controller ray steered
    // through the crosshair-distance point, already in Reach's world basis -- which IS the core's
    // basis: Blam is Z-up, yaw about +Z from +X). Either way the servo must have recorded an aim this
    // poll -- that is the gate for every aim precondition (VR aim off, pause, tracking).
    bool intent_ray(Ray* out) override {
        if (!out) return false;
        float yaw = 0.0f, pitch = 0.0f;
        if (!Game_ReachTakeDesiredAim(yaw, pitch)) return false;
        if (snapshot.has_sightline_trace) {
            out->origin = reach_aim_basis::CmFromWorld(sight.origin);
            out->dir    = Vec3{sight.dir[0], sight.dir[1], sight.dir[2]};
            return true;
        }
        out->origin = Vec3{};
        out->dir    = reach_aim_basis::DirFromGameAngles(yaw, pitch);
        return true;
    }

    // The range the sim-thread trace measured along that ray, in cm (fresh hits only; see caps).
    bool trace_impact_cm(const Ray&, float* out_cm) override {
        if (!out_cm || !snapshot.has_sightline_trace) return false;
        *out_cm = sight.rangeWu * kCmPerWorldUnit;
        return true;
    }
    // Ray origin minus the engine eye (Reach's first-person shot origin), world basis, cm.
    bool origin_delta(Vec3* out) override {
        if (!out || !snapshot.has_sightline_trace) return false;
        *out = reach_aim_basis::DeltaCm(sight.origin, sight.eye);
        last_delta_cm = std::sqrt(out->x * out->x + out->y * out->y + out->z * out->z);
        return true;
    }

    // WriteState publishes the setpoint for the hook's next sim tick, together with whether the core
    // actually bent it. The HOOK records that bit when it writes desired_angles, and the firing site
    // reads it from there, so the stand-down follows the angles really in effect, on the sim thread.
    // StickLoop was already computed by the servo this poll (ProduceStick hands its stick to the input
    // hook), so it reports whether the servo produced one.
    ActuateResult actuate(Actuation how, const Angles& converged) override {
        last_how = how;
        last_angles = converged;
        last_converged = false;
        if (how == Actuation::WriteState) {
            last_converged = state && state->converged && snapshot.has_sightline_trace;
            float yawRad = 0.0f, pitchRad = 0.0f;
            reach_aim_basis::GameAnglesFromCore(converged, yawRad, pitchRad);
            Game_ReachDirectDrivePublish(yawRad, pitchRad, last_converged);
            if (last_converged)
                Game_ReachNoteConvergedSample(sight, state->range_cm / kCmPerWorldUnit);
            return { true, false };   // not a replicated write until co-op host-follow is verified
        }
        if (how == Actuation::StickLoop) return { servo_ok, false };
        return {};
    }

    void apply_engine_convergence(Converge cv, const Ray&) override { last_conv = cv; }

    AimCaps   snapshot{};
    bool      servo_ok    = false;               // set by ProduceStick before each solve
    bool      sight_ok    = false;               // ditto: a fresh trace-and-reaim sample this poll
    bool      fresh_hit   = false;               // ...and it carries a hit measured within kFreshHitMs
    ReachSightSample sight{};
    const SolveState* state = nullptr;
    Actuation last_how    = Actuation::None;
    Converge  last_conv   = Converge::DirectionOnly;
    Angles    last_angles{};
    float     last_delta_cm = 0.0f;
    bool      last_converged = false;
};

// Status for the cold worker (never logged from the input hook: file I/O and the log lock stay out of
// the game's XInput path).
struct ProviderStatus {
    std::atomic<uint32_t> polls{0}, writes{0}, converged{0}, freshHits{0};
    std::atomic<int> rung{0}, converge{0};
    std::atomic<float> yawDeg{0.0f}, pitchDeg{0.0f}, rangeM{0.0f}, deltaCm{0.0f};
} g_status;

} // namespace

bool ReachAimProvider_ProduceStick(float& outRx, float& outRy) {
    static ReachAimProvider provider;
    static SolveState state;
    static bool traceReaimLastPoll = false;
    static LARGE_INTEGER lastPoll{};

    // 1. The stick servo runs first, every poll: it records this poll's desired aim (the intent the core
    //    takes) and computes the StickLoop rung's stick, so falling back costs nothing and is immediate.
    float rx = 0.0f, ry = 0.0f;
    provider.servo_ok = Game_ComputeAimStick(rx, ry);
    provider.last_how = Actuation::None;
    provider.state = &state;

    // 2. This poll's trace-and-reaim sample and the caps snapshot the whole solve will see.
    provider.sight_ok = Game_ReachSightSample(provider.sight);
    const uint64_t now = GetTickCount64();
    provider.fresh_hit = provider.sight_ok && provider.sight.hit && provider.sight.traceSampleMs &&
                         now >= provider.sight.traceSampleMs && now - provider.sight.traceSampleMs <= kFreshHitMs;
    provider.snapshot = provider.compute_caps();
    // A range smoothed before a gap describes some earlier view: start it afresh from the first hit.
    if (!provider.snapshot.has_sightline_trace || !traceReaimLastPoll)
    {
        state.range_cm = 0.0f;
        state.inv_range_per_cm = 0.0f;
    }
    traceReaimLastPoll = provider.snapshot.has_sightline_trace;

    // 3. The core picks the rung, converges, and actuates through the provider, with the measured poll
    //    interval (the input hook runs at the game's XInput rate, not a fixed one). Range tau matches
    //    CampE's convergence smoothing.
    LARGE_INTEGER qpc{}, freq{};
    QueryPerformanceCounter(&qpc);
    QueryPerformanceFrequency(&freq);
    float dt = 1.0f / 90.0f;
    if (lastPoll.QuadPart && freq.QuadPart && qpc.QuadPart > lastPoll.QuadPart)
        dt = std::fmax(0.001f, std::fmin(0.1f, float(double(qpc.QuadPart - lastPoll.QuadPart) / double(freq.QuadPart))));
    lastPoll = qpc;
    const ActuateResult res = solve_and_actuate(provider, state, ConvergeRails{}, dt, 120.0f);
    const bool wroteState = res.applied && provider.last_how == Actuation::WriteState;
    if (!wroteState)
        Game_ReachDirectDriveRelease();   // no WriteState this poll: the hook must stop writing now

    g_status.polls.fetch_add(1, std::memory_order_relaxed);
    if (wroteState) g_status.writes.fetch_add(1, std::memory_order_relaxed);
    if (wroteState && provider.last_converged) g_status.converged.fetch_add(1, std::memory_order_relaxed);
    if (provider.fresh_hit) g_status.freshHits.fetch_add(1, std::memory_order_relaxed);
    g_status.rung.store(static_cast<int>(provider.last_how), std::memory_order_relaxed);
    g_status.converge.store(static_cast<int>(select_converge(provider.snapshot)), std::memory_order_relaxed);
    g_status.yawDeg.store(provider.last_angles.yaw_deg, std::memory_order_relaxed);
    g_status.pitchDeg.store(provider.last_angles.pitch_deg, std::memory_order_relaxed);
    g_status.rangeM.store(state.range_cm * 0.01f, std::memory_order_relaxed);
    g_status.deltaCm.store(provider.last_delta_cm, std::memory_order_relaxed);

    if (wroteState) {   // one driver: the state write owns aim, so the stick says nothing
        outRx = 0.0f;
        outRy = 0.0f;
        return true;
    }
    if (provider.servo_ok) {
        outRx = rx;
        outRy = ry;
    }
    return provider.servo_ok;
}

void ReachAimProvider_Report() {
    static uint64_t last_ms = 0;
    const uint64_t now = GetTickCount64();
    if (now - last_ms < 2000) return;
    last_ms = now;
    const uint32_t polls = g_status.polls.exchange(0, std::memory_order_relaxed);
    if (!polls) return;   // the provider is not running (another title, a menu, or the default config)
    const int rung = g_status.rung.load(std::memory_order_relaxed);
    // ready = hook installed + arm-gate armed + hook heartbeat fresh (Game_ReachDirectDriveReady).
    // why = the failing on-foot conditions (Game_ReachDirectDriveOnFootWhy's bits; 0 = on foot).
    // converge: 0 trace-and-reaim, 1 origin relocation, 2 fixed range, 3 direction only.
    LOG("AIMPROVIDER reach: rung=%s (WriteState %u/%u polls, converged %u, fresh hits %u) mode=%d ready=%d "
        "onfoot=%d why=0x%02X converge=%d range=%.2f m eye->ray %.1f cm yaw=%.1f pitch=%.1f",
        rung_name(static_cast<Actuation>(rung)), g_status.writes.exchange(0, std::memory_order_relaxed), polls,
        g_status.converged.exchange(0, std::memory_order_relaxed),
        g_status.freshHits.exchange(0, std::memory_order_relaxed), g_config.reach_direct_drive,
        Game_ReachDirectDriveReady() ? 1 : 0, Game_ReachDirectDriveOnFoot() ? 1 : 0,
        static_cast<unsigned>(Game_ReachDirectDriveOnFootWhy()),
        g_status.converge.load(std::memory_order_relaxed), g_status.rangeM.load(std::memory_order_relaxed),
        g_status.deltaCm.load(std::memory_order_relaxed), g_status.yawDeg.load(std::memory_order_relaxed),
        g_status.pitchDeg.load(std::memory_order_relaxed));
}
