// Convergence policy (src/common/converge_policy.h): what switches the converged aim off (our code only), the
// dev assessment key's scope, and the natural-cause witnesses that are never an input to a decision.
#include "converge_policy.h"
#include <cmath>
#include <cstdio>
#include <type_traits>

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

using namespace converge;

// THE REGRESSION GUARD for the 2026-10-01 decision. The seated claim's signature is two facts of our own
// code -- solved, and a readable aim feedback -- so no natural cause can be handed to it. Adding a parameter
// (a latch, a stall, a turret, an outcome miss) fails this compile-time check before it can fail a player.
static_assert(std::is_same_v<decltype(&ClaimSeatedConverged), bool (*)(bool, bool)>,
              "the seated converged claim must take no natural in-game cause");

void TestClaimIgnoresNaturalCauses()
{
    // Every combination of the natural causes that used to switch a seat's convergence off, witnessed as they
    // are in the game; the claim is the same as with none of them.
    NaturalWitness witness;
    unsigned combinations = 0, changed = 0;
    for (unsigned bits = 0; bits < 64; ++bits)
    {
        NaturalEvents e;
        e.latchHeld = (bits & 1u) != 0;
        e.outcomeMiss = (bits & 2u) != 0;
        e.stall = (bits & 4u) != 0;
        e.turretSlew = (bits & 8u) != 0;
        e.reachLimited = (bits & 16u) != 0;
        e.yawShortH4 = (bits & 32u) != 0;
        for (int solved = 0; solved < 2; ++solved)
        {
            for (int finite = 0; finite < 2; ++finite)
            {
                witness.note(e);
                ++combinations;
                const bool claim = ClaimSeatedConverged(solved != 0, finite != 0);
                if (claim != (solved && finite))
                    ++changed;
            }
        }
    }
    Check(combinations == 256 && changed == 0, "no natural cause changes the seated converged claim");
    {
        unsigned held = 0;
        for (unsigned bits = 0; bits < 64; ++bits)
        {
            NaturalEvents e;
            e.latchHeld = (bits & 1u) != 0;
            e.outcomeMiss = (bits & 2u) != 0;
            e.stall = (bits & 4u) != 0;
            e.turretSlew = (bits & 8u) != 0;
            e.reachLimited = (bits & 16u) != 0;
            e.yawShortH4 = (bits & 32u) != 0;
            held += NaturalHoldsAimOff(e) ? 1u : 0u;
        }
        Check(held == 63, "any natural cause marks the poll held off the point (for the dev witnesses only)");
    }
    const NaturalCounts counts = witness.take();
    Check(counts.polls == 256 && counts.latchHeld == 128 && counts.outcomeMiss == 128 && counts.stall == 128 &&
              counts.turretSlew == 128 && counts.reachLimited == 128 && counts.yawShortH4 == 128,
          "the witness counts every natural cause it is shown");
    const NaturalCounts again = witness.take();
    Check(again.polls == 0 && again.latchHeld == 0 && again.yawShortH4 == 0, "taking the counts resets them");

    // Our code's failures still decline.
    Check(ClaimSeatedConverged(true, true), "solved with a readable aim: claimed, as on foot");
    Check(!ClaimSeatedConverged(false, true), "not solved (no fresh trace, an unnamed seat): not claimed");
    Check(!ClaimSeatedConverged(true, false), "an unreadable aim feedback: not claimed");
}

void TestAimFeedback()
{
    Check(AimFeedbackFinite(1.0f, 0.0f, 0.0f), "a unit aim is finite");
    Check(AimFeedbackFinite(0.0f, 0.0f, 3.0f), "an unnormalised aim is still a direction");
    Check(!AimFeedbackFinite(0.0f, 0.0f, 0.0f), "a zero aim is not a direction");
    Check(!AimFeedbackFinite(std::nanf(""), 0.0f, 1.0f), "a NaN aim is not a direction");
    Check(!AimFeedbackFinite(INFINITY, 0.0f, 0.0f), "an infinite aim is not a direction");
}

void TestAssessment()
{
    Check(!AssessDrawsGameAim(0, false) && !AssessDrawsGameAim(0, true), "0: never");
    Check(AssessDrawsGameAim(1, false) && !AssessDrawsGameAim(1, true), "1: on foot only");
    Check(AssessDrawsGameAim(2, false) && AssessDrawsGameAim(2, true), "2: everywhere");
    Check(!AssessDrawsGameAim(-1, false), "a negative key is off");
    // The assessment key is not an input of the claim: the claim is pinned to two facts above, so this
    // documents the separation rather than computing it.
    Check(ClaimSeatedConverged(true, true) == true, "the claim stands whatever the assessment draws");
}

void TestStallWitness()
{
    {
        StallWitness s;
        Check(!s.step(true, 0.2f, 1000), "a fresh stall is not yet a stall");
        Check(!s.step(true, 0.2f, 1200), "not before 250 ms");
        Check(s.step(true, 0.2f, 1250), "held 250 ms: stalled");
        Check(!s.step(false, 0.2f, 1300), "on foot never stalls");
    }
    {
        StallWitness s;
        s.step(true, 0.3f, 1000);
        Check(!s.step(true, 0.2f, 1300), "a shrinking error is converging, not stalled");
        Check(!s.step(true, 0.1f, 1400), "under 0.12 rad is ordinary lag");
        Check(!s.step(true, std::nanf(""), 1500), "a non-finite error is no evidence");
    }
}

void TestNames()
{
    bool all = true;
    for (unsigned i = 0; i < static_cast<unsigned>(ConvergeOff::Count); ++i)
        all = all && ConvergeOffName(static_cast<ConvergeOff>(i))[0] != '?';
    Check(all, "every ConvergeOff reason has a name");
}

void TestReachStamps()
{
    // The converged stamp follows the write and the converged setpoint; a seat limit never withdraws it.
    unsigned changed = 0;
    for (int wrote = 0; wrote < 2; ++wrote)
        for (int conv = 0; conv < 2; ++conv)
        {
            const ReachUpdateStamps free = ReachStampUpdate(wrote != 0, conv != 0, false);
            const ReachUpdateStamps held = ReachStampUpdate(wrote != 0, conv != 0, true);
            if (free.converged != held.converged)
                ++changed;
            Check(free.converged == (wrote && conv), "Reach: converged = wrote and a converged setpoint");
            Check(!free.heldAtLimit, "Reach: not limited, not held");
            Check(held.heldAtLimit == (wrote != 0), "Reach: held only when this update wrote");
        }
    Check(changed == 0, "Reach: a seat limit does not change the converged claim (2026-10-01)");
    Check(ReachStampUpdate(true, true, true).converged, "Reach: held at a seat's stop and still converged");
    Check(!ReachStampUpdate(false, true, true).converged, "Reach: nothing written, nothing claimed");

    Check(!StampLive(0, 1000, 100), "a stamp never set is not live");
    Check(StampLive(1000, 1100, 100), "inside the window");
    Check(!StampLive(1000, 1101, 100), "past the window");
    Check(!StampLive(1000, 999, 100), "a clock that went backwards is not live");
}

void TestSeatMuzzleRefusal()
{
    Check(SeatMuzzleRefuses(false, false) == SeatMuzzleRefusal::None, "free seat: the redirect may run");
    Check(SeatMuzzleRefuses(true, false) == SeatMuzzleRefusal::HeldAtLimit, "held at the stop: refused");
    Check(SeatMuzzleRefuses(false, true) == SeatMuzzleRefusal::Turret,
          "a turret refuses even when not latched (a rate cap is not a latch)");
    Check(SeatMuzzleRefuses(true, true) == SeatMuzzleRefusal::Turret, "a held turret reports the turret");
    // The refusal is a separate decision from the claim: the claim's signature takes neither input.
    Check(ClaimSeatedConverged(true, true), "the claim stands whatever the muzzle guard decides");
}
} // namespace

int main()
{
    TestClaimIgnoresNaturalCauses();
    TestAimFeedback();
    TestAssessment();
    TestStallWitness();
    TestReachStamps();
    TestSeatMuzzleRefusal();
    TestNames();
    std::printf("Converge policy: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
