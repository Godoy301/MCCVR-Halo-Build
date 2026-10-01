#pragma once

// CONVERGENCE POLICY -- when the converged aim (the traced reticle, the converged mark, the scope's
// convergence and the muzzle redirect that rides on it) is switched OFF, and what is only WITNESSED.
//
// The user's decision (2026-10-01): convergence and direct drive are never switched off for a NATURAL,
// in-game cause. A seat stop, a turret's rate cap, an aim the game is still integrating toward the hand --
// in flat Halo those already send a shot somewhere the reticle is not (a stuck vehicle's shots do), and the
// game shows it the way it always has: tracers and impacts. We get close enough to the game's own behaviour
// that a second, VR-only signal (the old "honest reticle" jumping onto the gun) is not wanted.
//
// What still switches convergence off is a failure of OUR code or a state the converged aim does not apply
// to: an address that did not resolve, a trace that faulted or went stale, a seat we have not implemented, an
// aim feedback we cannot read. Those are the ConvergeOff reasons below.
//
// The natural causes are still COUNTED (NaturalWitness, dev lines only) so a headset session can prove they
// now leave convergence on. They are never an input to a decision: ClaimSeatedConverged takes no natural
// cause by construction (tests/converge_policy_tests.cpp pins its signature).
//
// Zero engine dependencies; unit-tested in tests/converge_policy_tests.cpp.

#include <atomic>
#include <cmath>
#include <cstdint>

namespace converge {

// Why the converged aim is not live this poll. Every one is our code's or a non-gameplay state's -- never a
// natural in-game limit (those are NaturalEvents, witnessed only).
enum class ConvergeOff : uint8_t {
    None,
    AimOff,              // VR aim, converged_aim or the title's aim feature is off
    NoTraceFlags,        // the crosshair trace's flags did not resolve
    TraceFaulted,        // the trace raised a native fault and latched off
    TraceStale,          // no fresh trace (a hitch, a load, a pause)
    RayStale,            // no fresh intent ray
    NoRange,             // the trace returned no usable range
    PitchRail,           // the bent pitch would pass the aim's pitch rail (Reach keeps its rail; see aim_solve)
    SeatUnnamed,         // a seat whose vehicle the sampler could not name (the trace cannot look past it)
    SeatViewNotOwned,    // a seat whose first-person view is not ours (a chase camera)
    SeatNotImplemented,  // a seat this title does not converge yet
    DriveNotArmed,       // the direct drive has not proven its record yet
    CallerMapUnproven,   // the aim caller map is unproven (every call counts)
    AimFeedbackInvalid,  // the game's aim could not be read as a finite direction
    MarkStale,           // the converged mark is older than its freshness window
    Count
};

inline const char* ConvergeOffName(ConvergeOff why)
{
    switch (why) {
    case ConvergeOff::None: return "None";
    case ConvergeOff::AimOff: return "AimOff";
    case ConvergeOff::NoTraceFlags: return "NoTraceFlags";
    case ConvergeOff::TraceFaulted: return "TraceFaulted";
    case ConvergeOff::TraceStale: return "TraceStale";
    case ConvergeOff::RayStale: return "RayStale";
    case ConvergeOff::NoRange: return "NoRange";
    case ConvergeOff::PitchRail: return "PitchRail";
    case ConvergeOff::SeatUnnamed: return "SeatUnnamed";
    case ConvergeOff::SeatViewNotOwned: return "SeatViewNotOwned";
    case ConvergeOff::SeatNotImplemented: return "SeatNotImplemented";
    case ConvergeOff::DriveNotArmed: return "DriveNotArmed";
    case ConvergeOff::CallerMapUnproven: return "CallerMapUnproven";
    case ConvergeOff::AimFeedbackInvalid: return "AimFeedbackInvalid";
    case ConvergeOff::MarkStale: return "MarkStale";
    case ConvergeOff::Count: break;
    }
    return "?";
}

// A SEATED converged aim is claimed (the mark, so the traced reticle and the scope's convergence) whenever the
// aim was solved through the traced point and the game's aim feedback is a finite direction -- exactly as on
// foot. Nothing about the seat's limits, a turret's rate cap or how far the game's aim trails the target
// takes part: those are natural, and the game shows them itself.
inline bool ClaimSeatedConverged(bool solved, bool aimFeedbackFinite) { return solved && aimFeedbackFinite; }

// The game's aim feedback as a direction: finite and not degenerate.
inline bool AimFeedbackFinite(float x, float y, float z)
{
    const float length = std::sqrt(x * x + y * y + z * z);
    return std::isfinite(length) && length > 1.0e-4f;
}

// DEV ASSESSMENT ONLY (honest_reticle_assess, parsed in dev builds only): draw the reticle on the game's real
// aim instead, to judge by eye how far the game trails the hand. 0 = off, 1 = on foot only, 2 = everywhere.
// It moves where the reticle is DRAWN and nothing else: the mark, the shots, the write and the drive mode
// never read it.
inline bool AssessDrawsGameAim(int assess, bool seated) { return assess >= 2 || (assess == 1 && !seated); }

// ---- natural causes: witnessed, never decided on -------------------------------------------------------

// What happened this poll that USED to switch convergence off (or put the reticle on the gun).
struct NaturalEvents {
    bool latchHeld = false;     // the seat-limit latch held a written axis at the game's clamp
    bool outcomeMiss = false;   // the game's aim stayed off the written target (OutcomeGate's old trigger)
    bool stall = false;         // the servo's error stalled past the old 250 ms stall check
    bool turretSlew = false;    // a rate-capped turret seat (its reticle used to ride the barrel)
    bool reachLimited = false;  // Reach's seat latch held the write at a limit
    bool yawShortH4 = false;    // a yaw-only seat whose game yaw stayed off the writes (Halo 4's slope frame)
};

struct NaturalCounts {
    uint32_t polls = 0;
    uint32_t latchHeld = 0, outcomeMiss = 0, stall = 0, turretSlew = 0, reachLimited = 0, yawShortH4 = 0;
};

// Lock-free counters, one per natural cause; any thread may note, the cold worker takes them every report.
struct NaturalWitness {
    std::atomic<uint32_t> polls{0};
    std::atomic<uint32_t> latchHeld{0}, outcomeMiss{0}, stall{0}, turretSlew{0}, reachLimited{0}, yawShortH4{0};

    void note(const NaturalEvents& e)
    {
        polls.fetch_add(1, std::memory_order_relaxed);
        if (e.latchHeld) latchHeld.fetch_add(1, std::memory_order_relaxed);
        if (e.outcomeMiss) outcomeMiss.fetch_add(1, std::memory_order_relaxed);
        if (e.stall) stall.fetch_add(1, std::memory_order_relaxed);
        if (e.turretSlew) turretSlew.fetch_add(1, std::memory_order_relaxed);
        if (e.reachLimited) reachLimited.fetch_add(1, std::memory_order_relaxed);
        if (e.yawShortH4) yawShortH4.fetch_add(1, std::memory_order_relaxed);
    }

    NaturalCounts take()
    {
        NaturalCounts c;
        c.polls = polls.exchange(0, std::memory_order_relaxed);
        c.latchHeld = latchHeld.exchange(0, std::memory_order_relaxed);
        c.outcomeMiss = outcomeMiss.exchange(0, std::memory_order_relaxed);
        c.stall = stall.exchange(0, std::memory_order_relaxed);
        c.turretSlew = turretSlew.exchange(0, std::memory_order_relaxed);
        c.reachLimited = reachLimited.exchange(0, std::memory_order_relaxed);
        c.yawShortH4 = yawShortH4.exchange(0, std::memory_order_relaxed);
        return c;
    }
};

// The old servo stall check, kept as a witness: the error is "stalled" when it is at least `stall_rad`, not
// shrinking by more than `converging_rad` a poll, and has stayed so for `hold_ms` while seated.
struct StallWitness {
    float stall_rad = 0.12f;          // ~7 deg, far beyond the loop's ordinary lag
    float converging_rad = 0.005f;
    unsigned long long hold_ms = 250;

    unsigned long long since_ms = 0;
    float previous = 0.0f;

    void reset() { since_ms = 0; previous = 0.0f; }

    bool step(bool seated, float error_rad, unsigned long long now_ms)
    {
        const bool finite = std::isfinite(error_rad);
        if (!seated || !finite || error_rad < stall_rad || error_rad < previous - converging_rad)
            since_ms = 0;
        else if (!since_ms)
            since_ms = now_ms ? now_ms : 1;
        previous = finite ? error_rad : 0.0f;
        return since_ms && now_ms >= since_ms && now_ms - since_ms >= hold_ms;
    }
};

} // namespace converge
