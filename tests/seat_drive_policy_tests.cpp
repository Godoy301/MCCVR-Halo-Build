// Seat drive policy (src/common/seat_drive_policy.h): which axes of a vehicle seat the direct-drive hook may
// write, each seat stint's per-axis check, and the honest-reticle gate by outcome.
#include "seat_drive_policy.h"
#include <cmath>
#include <cstdio>

namespace
{
unsigned checks{}, failures{};
void Check(bool ok, const char* what)
{
    ++checks;
    if (!ok)
    {
        ++failures;
        std::printf("FAIL: %s\n", what);
    }
}

using seat_drive::Axes;
using seat_drive::Decide;
using seat_drive::Seat;

Seat Passenger() { Seat s; s.identified = s.view_owned = true; return s; }
Seat LookDriver() { Seat s = Passenger(); s.driver = s.look_steered = true; return s; }
Seat TankDriver() { Seat s = Passenger(); s.driver = true; return s; }

void TestPolicy()
{
    // Off switches and unknowns: nothing written.
    Check(!Decide(Passenger(), 2, false).any(), "seat_direct_drive=0 writes no seat");
    Check(!Decide(Passenger(), 0, true).any(), "reach_vehicle_aim=0 writes no seat");
    Seat s = Passenger(); s.identified = false;
    Check(!Decide(s, 2, true).any(), "an unidentified seat is never written");
    s = Passenger(); s.view_owned = false;
    Check(!Decide(s, 2, true).any(), "a seat whose view is not ours is never written");

    // Passengers, gunners, turrets and tank drivers: both axes, at levels 1 and 2.
    for (int level = 1; level <= 2; ++level)
    {
        Axes a = Decide(Passenger(), level, true);
        Check(a.yaw && a.pitch, "a passenger is written on both axes");
        a = Decide(TankDriver(), level, true);
        Check(a.yaw && a.pitch, "a driver whose look aims a turret is written on both axes");
    }

    // Look-steered drivers.
    Check(!Decide(LookDriver(), 1, true).any(), "level 1 keeps a look-steered driver on the servo");
    Axes a = Decide(LookDriver(), 2, true);
    Check(a.yaw && a.pitch, "level 2: a look-steered driver (follow off) is written on both axes");
    s = LookDriver(); s.authors_steering = true;
    a = Decide(s, 2, true);
    Check(!a.yaw && a.pitch, "the stick steers: the heading is the author's, the climb follows the hand");
    s.wheel = true;
    Check(!Decide(s, 2, true).any(), "both hands on the wheel: nothing is written");
    s.wheel = false;
    Check(!Decide(s, 1, true).any(), "level 1, stick steering: still the servo");

    // The latch reads no driver pitch (autolevel); every other seat latches.
    Check(!seat_drive::PitchLatches(LookDriver()), "a look-steered driver's pitch never latches (autolevel)");
    Check(seat_drive::PitchLatches(TankDriver()), "a tank driver's pitch latches at the turret's stop");
    Check(seat_drive::PitchLatches(Passenger()), "a passenger's pitch latches");

    // Seat keys change with the seat, the vehicle and the mount.
    Check(seat_drive::SeatKey(0x10002, 0, false) != seat_drive::SeatKey(0x10002, 1, false), "seat index keys");
    Check(seat_drive::SeatKey(0x10002, 0, false) != seat_drive::SeatKey(0x20003, 0, false), "vehicle keys");
    Check(seat_drive::SeatKey(0x10002, 0, false) != seat_drive::SeatKey(0x10002, 0, true), "mount keys");
}

void TestStintCheck()
{
    using seat_drive::StintCheck;
    {   // agreeing record at rest: yaw then pitch pass at the 30th update
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 29; ++i, t += 16)
            c.step(true, 0.5f, 0.1f, 0.51f, 0.11f, t);
        Check(!c.yaw_ok && !c.pitch_ok, "29 updates are not enough");
        c.step(true, 0.5f, 0.1f, 0.51f, 0.11f, t);
        Check(c.yaw_ok, "yaw passes on 30 agreeing updates");
        // pitch had one fewer settled update (the first has no previous aim)
        t += 16;
        c.step(true, 0.5f, 0.1f, 0.51f, 0.11f, t);
        Check(c.pitch_ok && !c.pitch_failed, "pitch passes on 30 settled agreeing updates");
    }
    {   // yaw wraps across +-pi
        StintCheck c;
        for (int i = 0; i < 30; ++i)
            c.step(true, 3.13f, 0.0f, -3.13f, 0.0f, 1000 + 16 * i);
        Check(c.yaw_ok, "a yaw agreement across the wrap counts");
    }
    {   // a pitch offset (driver+gunner, record 13 deg off the gun): yaw passes, pitch fails at the budget
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 400 && !c.pitch_failed; ++i, t += 16)
            c.step(true, 0.2f, 0.0f, 0.2f, 0.23f, t);
        Check(c.yaw_ok && !c.pitch_ok && c.pitch_failed, "a pitch that disagrees stays on the servo");
        // yaw passed at its 30th update (464 ms in); pitch then had three seconds of updates
        Check(t > 1000 + 464 + 3000 && t < 1000 + 464 + 3200, "the pitch budget is three seconds after yaw");
    }
    {   // a slow yaw (a gun slewing onto the hand) does not use up an agreeing pitch's budget
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 250; ++i, t += 16)           // 4 s: yaw never 30 in a row, pitch settled and agreeing
            c.step(true, (i % 2) ? 0.05f : 0.0f, 0.1f, 0.0f, 0.1f, t);
        Check(!c.yaw_ok && !c.pitch_ok && !c.pitch_failed, "no pitch before yaw, and no budget spent waiting");
        for (int i = 0; i < 30; ++i, t += 16)
            c.step(true, 0.0f, 0.1f, 0.0f, 0.1f, t);
        Check(c.yaw_ok && c.pitch_ok && !c.pitch_failed, "pitch passes the moment the slow yaw does");
    }
    {   // agreement must be consecutive
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 29; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.0f, t);
        c.step(true, 0.2f, 0.0f, 0.0f, 0.0f, t); t += 16;
        for (int i = 0; i < 29; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.0f, t);
        Check(!c.yaw_ok && c.yaw_count == 29, "one disagreement restarts the yaw count");
    }
    {   // a repeated aim sample neither counts nor resets
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 20; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.0f, t);
        for (int i = 0; i < 20; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.0f, t, false);
        Check(c.yaw_count == 20, "a repeated agreeing sample does not count");
        c.step(true, 0.2f, 0.2f, 0.0f, 0.0f, t, false); t += 16;
        Check(c.yaw_count == 20 && c.pitch_count == 19, "a repeated disagreeing sample does not reset");
        for (int i = 0; i < 10; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.0f, t);
        Check(c.yaw_ok, "new samples finish the count");
    }
    {   // a slewing barrel: unsettled updates neither count nor reset
        StintCheck c;
        unsigned long long t = 1000;
        float p = 0.0f;
        for (int i = 0; i < 30; ++i, t += 16)
            c.step(true, 0.0f, p, 0.0f, p, t);
        Check(c.yaw_ok && !c.pitch_ok, "pitch is still counting (29 settled)");
        for (int i = 0; i < 20; ++i, t += 16)
        {
            p += 0.01f;                       // the barrel slews 0.6 deg per update; record lags 1.7+ deg
            c.step(true, 0.0f, p - 0.05f, 0.0f, p, t);
        }
        Check(!c.pitch_ok && c.pitch_count == 29, "a slewing barrel is not a disagreement");
        c.step(true, 0.0f, p, 0.0f, p, t);    // settles back onto the record
        Check(c.pitch_ok, "one more settled agreement passes pitch");
    }
    {   // a pause after yaw passed: the gap adds at most 100 ms to pitch's budget
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 30; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.3f, t);
        Check(c.yaw_ok && !c.pitch_failed, "(setup) yaw passed, pitch disagreeing");
        t += 60000;
        c.step(true, 0.0f, 0.0f, 0.0f, 0.3f, t);
        Check(!c.pitch_failed && c.elapsed_ms - c.yaw_ok_at_ms <= 100, "a minute paused is not pitch's budget");
    }
    {   // a stale or non-finite sample neither counts nor resets pitch
        StintCheck c;
        unsigned long long t = 1000;
        for (int i = 0; i < 20; ++i, t += 16) c.step(true, 0.0f, 0.0f, 0.0f, 0.0f, t);
        c.step(false, 0.0f, 0.5f, 0.0f, 0.0f, t); t += 16;
        c.step(true, 0.0f, std::nanf(""), 0.0f, 0.0f, t); t += 16;
        Check(c.pitch_count == 19, "a stale or non-finite sample leaves the pitch count");
    }
    {   // a pause: the gap adds at most 100 ms to the budget
        StintCheck c;
        c.step(true, 0.0f, 0.0f, 0.0f, 0.3f, 1000);
        c.step(true, 0.0f, 0.0f, 0.0f, 0.3f, 61000);
        Check(!c.pitch_failed && c.elapsed_ms == 100, "a minute in a menu is not three seconds of seat");
    }
    {   // stale aim: nothing counts
        StintCheck c;
        for (int i = 0; i < 40; ++i)
            c.step(false, 0.0f, 0.0f, 0.0f, 0.0f, 1000 + 16 * i);
        Check(!c.yaw_ok && !c.pitch_ok, "a stale aim proves nothing");
        c.reset();
        Check(!c.yaw_ok && !c.have_last && !c.elapsed_ms, "reset forgets the stint");
    }
}

void TestOutcomeGate()
{
    using seat_drive::OutcomeGate;
    {
        OutcomeGate g;
        Check(!g.step(0.01f, false, 1000), "on target: the target's reticle");
        Check(!g.step(0.05f, false, 1016), "a miss must persist (hand flick lag)");
        Check(!g.step(0.05f, false, 1100), "not yet 120 ms");
        Check(g.step(0.05f, false, 1140), "a persistent miss shows the game's aim");
        Check(g.step(0.025f, false, 1160), "between exit and enter: stays honest");
        Check(!g.step(0.01f, false, 1180), "back under one degree: the target again");
    }
    {
        OutcomeGate g;
        Check(g.step(0.0f, true, 1000), "a latch hold is honest at once");
        Check(!g.step(0.0f, false, 1016), "and releases as soon as the aim is on the target");
        Check(g.step(std::nanf(""), false, 1032), "a non-finite miss is never claimed as converged");
    }
    {
        OutcomeGate g;
        g.step(0.05f, false, 1000);
        g.step(0.01f, false, 1050);           // dipped back inside: the hold restarts
        Check(!g.step(0.05f, false, 1130), "the hold restarts after the miss closes");
        Check(g.step(0.05f, false, 1260), "and then completes");
    }
}
} // namespace

int main()
{
    TestPolicy();
    TestStintCheck();
    TestOutcomeGate();
    std::printf("Seat drive policy: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
