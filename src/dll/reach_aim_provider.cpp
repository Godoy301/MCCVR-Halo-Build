// Reach's aim provider (see reach_aim_provider.h). The engine-free core picks the actuation rung from
// caps(); this file supplies Reach's facts and actuates the rung it picks:
//   WriteState -- direct drive (reach_direct_drive=3): the converged setpoint is published to the
//                 sim-thread hook on haloreach+0x6375C, which writes player_control.desired_angles
//                 (upstream of the derived aim vector and of replication); the aim stick is held
//                 neutral so there is exactly one driver.
//   StickLoop  -- the shipped closed loop, Game_ComputeAimStick, unchanged.
// Engine-bound (reads g_config and game.cpp's Reach state); the pure core it feeds is
// src/common/aim_solve_logic.h + aim_provider.h, and the write guards the hook applies are
// src/common/aim_write_logic.h.

#include "reach_aim_provider.h"

#include "../common/aim_provider.h"
#include "game.h"   // Game_ComputeAimStick (StickLoop) + the Game_ReachDirectDrive* WriteState seam
#include "../common/config.h"
#include "../common/log.h"

#include <windows.h>
#include <cmath>
#include <cstdint>

namespace {
using namespace aim_solve;

constexpr float kDegToRad = 0.017453292519943295f;

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

    AimCaps caps() const override {
        AimCaps c;
        // Rung 1 is player_control.desired_angles, the accumulator's own record: after stick
        // conditioning, before the aim vector is derived, and persistent (one write steers the sim).
        // Offered only when opted in (reach_direct_drive=3), with the hook proven by the arm-gate, and on
        // foot -- vehicles and cinematics keep the stick loop, which is proven there. Co-op host-follow
        // is UNVERIFIED, so actuate() never claims a replicated write.
        c.can_write_state       = g_config.reach_direct_drive == 3 && Game_ReachDirectDriveReady() &&
                                  Game_ReachDirectDriveOnFoot();
        c.has_stick_loop        = true;   // Reach steers via Game_ComputeAimStick, like Halo 3
        c.has_sightline_trace   = false;  // no aim trace into Blam collision yet
        c.has_origin_relocation = true;   // R-V10/R-V27 firing-site eye substitution, foot + vehicle
        c.has_fixed_range_reaim = true;   // ComputeAimStick steers through the crosshair-distance point
        c.has_barrel_marker     = g_config.gun_barrel_aim != 0;
        c.camera_follows_aim    = true;   // Reach writes the aim, so the view follows the hand
        return c;
    }

    // The aim Game_ComputeAimStick recorded during this poll (ProduceStick runs it first): the
    // mount-calibrated controller ray steered through the crosshair-distance point, already mapped into
    // Reach's world basis -- which IS the core's basis (Blam is Z-up, yaw about +Z from +X; the servo
    // compares it against atan2(y,x)/asin(z) of the game's own aim vector). Direction only: the origin
    // is reconciled in the engine (OriginRelocation), so it stays zero.
    bool intent_ray(Ray* out) override {
        if (!out) return false;
        float yaw = 0.0f, pitch = 0.0f;
        if (!Game_ReachTakeDesiredAim(yaw, pitch)) return false;
        const float cp = std::cos(pitch);
        out->origin = Vec3{};
        out->dir    = Vec3{cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch)};
        return true;
    }

    bool trace_impact_cm(const Ray&, float*) override { return false; }  // no aim trace on Reach
    bool origin_delta(Vec3*) override { return false; }                  // Reach uses origin relocation

    // WriteState publishes the converged setpoint for the hook's next sim tick. StickLoop was already
    // computed by the servo this poll (ProduceStick hands its stick to the input hook), so it reports
    // whether the servo produced one.
    ActuateResult actuate(Actuation how, const Angles& converged) override {
        last_how = how;
        last_angles = converged;
        if (how == Actuation::WriteState) {
            Game_ReachDirectDrivePublish(converged.yaw_deg * kDegToRad, converged.pitch_deg * kDegToRad);
            return { true, false };   // not a replicated write until co-op host-follow is verified
        }
        if (how == Actuation::StickLoop) return { servo_ok, false };
        return {};
    }

    void apply_engine_convergence(Converge cv, const Ray&) override { last_conv = cv; }

    bool      servo_ok    = false;               // set by ProduceStick before each solve
    Actuation last_how    = Actuation::None;
    Converge  last_conv   = Converge::DirectionOnly;
    Angles    last_angles{};
};

} // namespace

bool ReachAimProvider_ProduceStick(float& outRx, float& outRy) {
    static ReachAimProvider provider;
    static SolveState state;

    // 1. The stick servo runs first, every poll: it records this poll's desired aim (the intent the core
    //    takes) and computes the StickLoop rung's stick, so falling back costs nothing and is immediate.
    float rx = 0.0f, ry = 0.0f;
    provider.servo_ok = Game_ComputeAimStick(rx, ry);
    provider.last_how = Actuation::None;

    // 2. The core picks the rung, converges, and actuates through the provider. ~90 Hz aim tick; the
    //    range tau matches CampE's convergence smoothing.
    const ActuateResult res = solve_and_actuate(provider, state, ConvergeRails{}, 1.0f / 90.0f, 120.0f);
    const bool wroteState = res.applied && provider.last_how == Actuation::WriteState;
    if (!wroteState)
        Game_ReachDirectDriveRelease();   // no WriteState this poll: the hook must stop writing now

    static uint64_t last_ms = 0;
    static unsigned polls = 0, writes = 0;
    ++polls;
    writes += wroteState ? 1u : 0u;
    const uint64_t now = GetTickCount64();
    if (now - last_ms >= 2000) {
        last_ms = now;
        const AimCaps c = provider.caps();
        LOG("AIMPROVIDER reach: rung=%s (WriteState %u/%u polls) mode=%d armed=%d onfoot=%d converge=%d "
            "barrel=%d yaw=%.1f pitch=%.1f",
            rung_name(provider.last_how), writes, polls, g_config.reach_direct_drive,
            Game_ReachDirectDriveReady() ? 1 : 0, Game_ReachDirectDriveOnFoot() ? 1 : 0,
            static_cast<int>(select_converge(c)), c.has_barrel_marker ? 1 : 0,
            provider.last_angles.yaw_deg, provider.last_angles.pitch_deg);
        polls = 0;
        writes = 0;
    }

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
