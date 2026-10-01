#pragma once

// SEAT DRIVE POLICY -- which axes of a vehicle seat's aim the direct-drive hook may WRITE, rather than
// leave to the stick servo (or to the steering author, for a driver whose stick or wheel steers).
//
// Every MCC title keeps a seat's aim in the same player_control record, updated by the same accumulator
// as on foot; the seat's limits are applied inside that accumulator (mapped 2026-10-01 for Halo 3, ODST,
// Halo 4, Halo 2 and Reach). So a seat can be written exactly like on foot, axis by axis, once the policy
// below says the hand owns that axis. Reach's per-seat rules (Game_ReachDirectDriveAimWhy) are the model:
//   * a passenger, a gunner, a turret, or a driver whose vehicle does NOT steer by the look (a tank: the
//     look aims the turret) -- the hand owns both axes;
//   * a driver whose vehicle steers toward the look (Warthog, Ghost, Banshee...) -- the hand owns both axes
//     only at level 2 ("drivers aim with the hand"); level 1 leaves that seat on the servo;
//   * the same driver while the stick or wheel steers (vehicle_view_follow): the heading is the steering
//     author's, never written; the climb/dive (pitch) still follows the hand, unless both hands are on the
//     wheel, which aims nothing;
//   * an unidentified seat, or a seat whose first-person view is not ours: nothing is written.
// What the policy allows is not yet what the hook writes: each seat stint still proves, axis by axis, that
// its record holds the game's aim before the first write (legacy_direct_drive.inl).
//
// Zero engine dependencies; unit-tested in tests/seat_drive_policy_tests.cpp.

#include <cmath>

namespace seat_drive {

struct Seat {
    bool identified = false;        // the vehicle and seat role are known
    bool view_owned = false;        // our first-person seat view is up (the intent ray is the hand's)
    bool driver = false;            // the seat that drives the vehicle
    bool look_steered = false;      // the vehicle turns toward the driver's aim
    bool authors_steering = false;  // the stick or wheel steers this driver now (vehicle_view_follow)
    bool wheel = false;             // both hands are on the wheel: the hands aim nothing
};

struct Axes {
    bool yaw = false;
    bool pitch = false;
    bool any() const { return yaw || pitch; }
};

// level: reach_vehicle_aim (0 = every seat on the servo, 1 = all but look-steered drivers, 2 = all).
// seat_drive_on: seat_direct_drive.
inline Axes Decide(const Seat& s, int level, bool seat_drive_on)
{
    Axes a;
    if (!seat_drive_on || level <= 0 || !s.identified || !s.view_owned)
        return a;
    if (!s.driver || !s.look_steered) {
        a.yaw = a.pitch = true;
        return a;
    }
    if (level < 2)
        return a;
    if (s.authors_steering) {
        a.pitch = !s.wheel;
        return a;
    }
    a.yaw = a.pitch = true;
    return a;
}

// Whether the seat-limit latch may read the game's pitch moves as a limit. A look-steered driver's pitch is
// levelled by the accumulator itself as the vehicle moves (autolevel, speed-scaled), which the latch would
// take for a limit and hold the aim short. A tank driver's look aims a turret with a real pitch stop, like a
// gunner's (Reach latches those too): it latches, as every other seat does.
inline bool PitchLatches(const Seat& s) { return !s.driver || !s.look_steered; }

// One seat stint, as the hook sees it: a key that changes when the occupant changes seat or vehicle, so a
// check passed in one seat is never carried into the next.
inline unsigned SeatKey(int parent_handle, int seat_index, bool mounted)
{
    return static_cast<unsigned>(parent_handle) * 2654435761u ^ (static_cast<unsigned>(seat_index + 1) << 1) ^
           (mounted ? 1u : 0u);
}

// PER-AXIS SEAT CHECK -- each seat stint proves, axis by axis, that its record holds the aim the game
// shows before that axis is written. Yaw passes on `needed` consecutive updates within `tol`. Pitch passes
// on `needed` SETTLED updates within `tol` (the game's aim pitch moved less than `settle` since the last
// update: a rate-capped barrel still slewing is not a disagreement) within `budget_ms` of updates after yaw
// passed; a pitch that has not passed by then stays on the servo for the rest of the stint. Pitch is never
// verified before yaw, and a slow yaw never uses up pitch's budget (a pitch that agreed all along passes the
// moment yaw does). The budget counts only time the game is updating (a gap -- a pause menu -- adds at most
// `gap_ms`). A REPEATED aim sample (the sim ticking faster than the aim is published) is not evidence: it
// neither counts nor resets.
struct StintCheck {
    float tol = 0.03f;          // 1.7 deg
    float settle = 0.00175f;    // 0.1 deg per update
    unsigned needed = 30;
    unsigned long long budget_ms = 3000;
    unsigned long long gap_ms = 100;

    bool yaw_ok = false, pitch_ok = false, pitch_failed = false;
    unsigned yaw_count = 0, pitch_count = 0;
    unsigned long long elapsed_ms = 0, last_ms = 0, yaw_ok_at_ms = 0;
    bool have_last = false;
    float last_aim_pitch = 0.0f;

    void reset()
    {
        yaw_ok = pitch_ok = pitch_failed = false;
        yaw_count = pitch_count = 0;
        elapsed_ms = last_ms = yaw_ok_at_ms = 0;
        have_last = false;
        last_aim_pitch = 0.0f;
    }

    // One update of the stint. `fresh`: the aim was published recently. `new_sample`: the aim is not the one
    // the previous update already judged. Yaw differences wrap.
    void step(bool fresh, float native_yaw, float native_pitch, float aim_yaw, float aim_pitch,
              unsigned long long now_ms, bool new_sample = true)
    {
        if (last_ms && now_ms >= last_ms)
            elapsed_ms += (now_ms - last_ms) < gap_ms ? (now_ms - last_ms) : gap_ms;
        last_ms = now_ms ? now_ms : 1;
        const bool finite = fresh && std::isfinite(native_yaw) && std::isfinite(native_pitch) &&
                            std::isfinite(aim_yaw) && std::isfinite(aim_pitch);
        if (!yaw_ok && (new_sample || !finite)) {
            const bool agree = finite && std::fabs(std::remainder(native_yaw - aim_yaw, 6.28318531f)) < tol;
            yaw_count = agree ? yaw_count + 1 : 0;
            if (yaw_count >= needed) {
                yaw_ok = true;
                yaw_ok_at_ms = elapsed_ms;
            }
        }
        if (!pitch_ok && !pitch_failed) {
            const bool settled = finite && new_sample && have_last && std::fabs(aim_pitch - last_aim_pitch) < settle;
            if (settled) {
                const bool agree = std::fabs(native_pitch - aim_pitch) < tol;
                pitch_count = agree ? pitch_count + 1 : 0;
            }
            if (yaw_ok && pitch_count >= needed) pitch_ok = true;
            else if (yaw_ok && elapsed_ms - yaw_ok_at_ms > budget_ms) pitch_failed = true;
        }
        if (finite && new_sample) {
            have_last = true;
            last_aim_pitch = aim_pitch;
        }
    }
};

// HONEST RETICLE BY OUTCOME. A written seat's aim can still fall short downstream of the record (a gun's
// rate cap, an aiming clip), where the seat-limit latch cannot see it. While the game's aim misses the
// written target by more than `enter` for `hold_ms`, the reticle shows the game's aim and no converged
// shot is claimed; it returns to the target once the miss is under `exit`. A latch hold is honest at once.
struct OutcomeGate {
    float enter = 0.035f;       // 2 deg
    float exit = 0.0175f;       // 1 deg
    unsigned long long hold_ms = 120;

    bool honest = false;
    unsigned long long since_ms = 0;

    void reset() { honest = false; since_ms = 0; }

    bool step(float miss_rad, bool latch_limited, unsigned long long now_ms)
    {
        if (latch_limited || !std::isfinite(miss_rad)) {
            honest = true;
            since_ms = 0;
            return honest;
        }
        if (honest) {
            if (miss_rad < exit) {
                honest = false;
                since_ms = 0;
            }
            return honest;
        }
        if (miss_rad <= enter) {
            since_ms = 0;
            return honest;
        }
        if (!since_ms) since_ms = now_ms ? now_ms : 1;
        if (now_ms >= since_ms && now_ms - since_ms >= hold_ms) honest = true;
        return honest;
    }
};

} // namespace seat_drive
