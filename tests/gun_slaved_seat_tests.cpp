// Gun-slaved seats (src/common/gun_slaved_seat_logic.h): the seat flag, the occupant-aim binding's agreement rules,
// the servo's feedback choice, the pacer that stops a servo closing on a slewing gun from nodding, the gun-settle
// gate of the crosshair witness, the slew rate and the nod witness.
#include "gun_slaved_seat_logic.h"
#include "aim_servo_logic.h"

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

using namespace gun_slaved;

void TestSeatFlag()
{
    Check(CameraSlavedToGun(0x00101198u), "the Warthog turret gunner's seat (0x00101198) is gun-slaved");
    Check(!CameraSlavedToGun(0x00101118u), "the same seat without bit 7 is not");
    Check(!CameraSlavedToGun(0x00000040u) && !CameraSlavedToGun(0x00000100u), "bits 6 and 8 are not bit 7");
}

UnitAimSites Halo3Sites()
{
    UnitAimSites s;
    s.cameraForward = 0x1AC; s.cameraForwardZ = 0x1B4; s.cameraSeatStride = 0xD4; s.cameraFlagShift = 7;
    s.controlForward = 0x1AC; s.controlForwardZ = 0x1B4; s.controlPitchWrite = 0xD8; s.controlYawWrite = 0xD4;
    return s;
}

void TestBinding()
{
    const char* why = nullptr;
    Check(ResolveUnitAimOffset(Halo3Sites(), 0xD4, 0xD4, why) == 0x1AC && !why, "Halo 3's two consumers agree: +0x1AC");
    UnitAimSites odst = Halo3Sites();
    odst.cameraForward = odst.controlForward = 0x1A4;
    odst.cameraForwardZ = odst.controlForwardZ = 0x1AC;
    Check(ResolveUnitAimOffset(odst, 0xD4, 0xD4, why) == 0x1A4, "ODST's agree on its own offset, +0x1A4");

    UnitAimSites s = Halo3Sites();
    s.cameraFlagShift = 6;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why) && why, "a camera that tests another bit binds nothing");
    s = Halo3Sites(); s.cameraSeatStride = 0xD8;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why), "a seat stride that is not the seat block's binds nothing");
    s = Halo3Sites(); s.controlForward = 0x1B8; s.controlForwardZ = 0x1C0;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why), "consumers reading neighbouring vectors bind nothing");
    s = Halo3Sites(); s.cameraForwardZ = 0x1B0;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why), "a z that is not beside x/y binds nothing");
    s = Halo3Sites(); s.controlYawWrite = 0x138; s.controlPitchWrite = 0x13C;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why), "a reset that writes another record's angles binds nothing");
    s = Halo3Sites();
    Check(!ResolveUnitAimOffset(s, 0xD4, 0x138, why), "nor against a title whose record is elsewhere");
    s = Halo3Sites(); s.cameraForward = s.controlForward = 0x1AD; s.cameraForwardZ = s.controlForwardZ = 0x1B5;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why), "a misaligned offset binds nothing");
    s = Halo3Sites(); s.cameraForward = s.controlForward = 0x40; s.cameraForwardZ = s.controlForwardZ = 0x48;
    Check(!ResolveUnitAimOffset(s, 0xD4, 0xD4, why), "an offset in the object header binds nothing");
}

void TestAngles()
{
    float yaw = 0, pitch = 0;
    const float level[3]{0.0f, 1.0f, 0.0f};
    Check(AnglesFromVector(level, yaw, pitch) && std::fabs(yaw - 1.5707963f) < 1e-5f && std::fabs(pitch) < 1e-6f,
          "a unit vector along +y is yaw 90 deg, level");
    const float up[3]{0.7071068f, 0.0f, 0.7071068f};
    Check(AnglesFromVector(up, yaw, pitch) && std::fabs(pitch - 0.7853982f) < 1e-5f, "45 deg up");
    const float shortVec[3]{0.5f, 0.0f, 0.0f};
    Check(!AnglesFromVector(shortVec, yaw, pitch), "a vector that is not a normal is not this field");
    const float zero[3]{0.0f, 0.0f, 0.0f};
    Check(!AnglesFromVector(zero, yaw, pitch), "zeroes are no sample");
    const float bad[3]{std::nanf(""), 0.0f, 1.0f};
    Check(!AnglesFromVector(bad, yaw, pitch), "a NaN is no sample");
    Check(std::fabs(AngleBetweenDeg(0.0f, 0.0f, 0.0f, 0.2268928f) - 13.0f) < 0.01f, "13 deg of pitch apart");
    Check(std::fabs(AngleBetweenDeg(3.13f, 0.0f, -3.13f, 0.0f) - 1.3284f) < 0.01f, "yaw across the wrap");
    Check(std::fabs(AngleBetweenDeg(0.0f, 1.5f, 3.0f, 1.5f) - 8.08f) < 0.1f, "near vertical, yaw counts by cos(pitch)");
    Check(std::isnan(AngleBetweenDeg(std::nanf(""), 0.0f, 0.0f, 0.0f)), "NaN in, NaN out");
}

void TestOffsetWatch()
{
    {   // a gun lagging the record while it moves, then catching up: never revoked
        OffsetWatch w;
        float gun = 0.0f;
        for (int i = 0; i < 200; ++i)
        {
            const float record = i < 60 ? 0.005f * i : 0.3f;   // the hand sweeps 17 deg, then holds
            gun += std::fabs(record - gun) < 0.018f ? record - gun : (record > gun ? 0.018f : -0.018f);
            w.step(record, gun);
        }
        Check(!w.revoked && w.count == 0, "a gun that lags and then lands is no offset");
    }
    {   // a gun that comes to rest 13 deg off the record: revoked after 30 still samples
        OffsetWatch w;
        int at = -1;
        for (int i = 0; i < 100 && at < 0; ++i)
            if (w.step(0.3f, 0.3f - 0.227f)) at = i;
        Check(at == 30, "a gun at rest 13 deg off the record revokes the occupant's proof on the 30th still sample");
        Check(w.step(0.3f, 0.3f), "and the revocation holds for the stint");
    }
    {   // a gap while the gun still moves is no evidence; one in-band sample at rest restarts the count
        OffsetWatch w;
        for (int i = 0; i < 25; ++i) w.step(0.3f, 0.0f);
        Check(w.count == 24, "(setup) 24 still disagreements");
        w.step(0.3f, 0.02f);
        Check(w.count == 24, "a moving gun neither counts nor resets");
        w.step(0.3f, 0.29f);
        w.step(0.3f, 0.29f);
        Check(w.count == 0 && !w.revoked, "back within 3 deg at rest: the count restarts");
    }
    {   // yaw wraps
        OffsetWatch w;
        w.wrap = true;
        for (int i = 0; i < 60; ++i) w.step(6.28f, -0.002f);
        Check(!w.revoked, "a record at 2pi and a gun just below 0 are the same heading");
    }
    {   // NaN: no sample
        OffsetWatch w;
        for (int i = 0; i < 60; ++i) w.step(std::nanf(""), 0.0f);
        Check(!w.revoked && !w.have, "a NaN is no sample");
    }
}

void TestServoFeedback()
{
    Check(ServoFeedback(true, true, kServoTrustSamples) == Feedback::Occupant,
          "a gun-slaved seat whose occupant has agreed closes on the occupant");
    Check(ServoFeedback(true, true, kServoTrustSamples - 1) == Feedback::Gun, "not yet trusted: the gun");
    Check(ServoFeedback(true, false, 30) == Feedback::Gun, "no fresh occupant sample: the gun");
    Check(ServoFeedback(false, true, 30) == Feedback::Gun, "any other seat: the camera, as before");
}

// A simulated turret axis: the stick integrates the record (rate per poll at full deflection, with ToRawStick's 27.5%
// floor), the gun slews toward the record at a capped rate, and the servo sees its feedback `latency` polls late
// (the input path, the control update, the tick). `occupant`: the feedback is the record's twin (the occupant's aim)
// rather than the gun.
struct TurretSim
{
    float stickRate = 0.05f;   // rad per poll at full deflection (~340 deg/s at 120 Hz)
    float gunRate = 0.018f;    // rad per poll (~125 deg/s)
    int latency = 2;
    bool occupant = false;
    float record = 0.0f, gun = 0.0f;
    float gunSeen[8]{}, recordSeen[8]{};
    float feedback() const { return occupant ? recordSeen[0] : gunSeen[0]; }
    void poll(float command)
    {
        float c = command;
        if (c != 0.0f && std::fabs(c) < kAimServoStickFloor) c = c > 0 ? kAimServoStickFloor : -kAimServoStickFloor;
        record += c * stickRate;
        const float toward = record - gun;
        gun += std::fabs(toward) <= gunRate ? toward : (toward > 0 ? gunRate : -gunRate);
        for (int i = 0; i < latency; ++i) { gunSeen[i] = gunSeen[i + 1]; recordSeen[i] = recordSeen[i + 1]; }
        gunSeen[latency] = gun;
        recordSeen[latency] = record;
    }
};

struct ServoRun
{
    unsigned gunReversals = 0, handReversals = 0;
    float finalErrDeg = 0.0f, maxTrackErrDeg = 0.0f;
};

// `paced`: the gun-slaved pacer; otherwise the parking servo exactly as the turret seat has always run it.
ServoRun RunServo(TurretSim sim, bool paced, bool movingHand)
{
    AimServoAxis axis;
    ServoPacer pacer;
    NodWitness gunNod, handNod;
    float lastCommand = 0.0f;
    ServoRun run;
    const int polls = 900;
    for (int i = 0; i < polls; ++i)
    {
        const float target = movingHand ? 0.35f + 0.2f * std::sin(i * 0.02f) : 0.35f;
        const float feedback = sim.feedback();
        AimServoObserve(axis, feedback, lastCommand);
        const float error = target - feedback;
        float command = AimServoCommand(axis, error, 12.0f, paced ? pacer.enter(kAimServoRestEnterRadians)
                                                                  : kAimServoRestEnterRadians,
                                        paced ? pacer.exit(kAimServoRestExitRadians) : kAimServoRestExitRadians);
        if (paced)
            command = pacer.pace(feedback, error, command, kAimServoStickFloor);
        sim.poll(command);
        lastCommand = std::fabs(command);
        if (i >= polls / 3)
        {
            gunNod.step(sim.gun);
            handNod.step(target);
            const float e = std::fabs(target - sim.gun) * kRadToDeg;
            if (e > run.maxTrackErrDeg) run.maxTrackErrDeg = e;
        }
        run.finalErrDeg = std::fabs(target - sim.gun) * kRadToDeg;
    }
    float swing = 0.0f;
    gunNod.take(run.gunReversals, swing);
    handNod.take(run.handReversals, swing);
    return run;
}

void TestPacer()
{
    {   // THE FAULT, reproduced: the parking servo closing on a gun it sees two polls late nods without end
        TurretSim sim;
        const ServoRun r = RunServo(sim, false, false);
        Check(r.gunReversals >= 10, "(the fault) a parking servo closing on a lagging gun nods: it keeps reversing");
    }
    // The pacer lands a 20 deg flick and stays, at every stick rate and latency tried, on either feedback.
    const float rates[]{0.02f, 0.03f, 0.05f, 0.08f, 0.12f};
    const int latencies[]{1, 2, 4, 6};
    bool gunAll = true, occupantAll = true;
    for (float rate : rates)
        for (int latency : latencies)
        {
            TurretSim sim;
            sim.stickRate = rate;
            sim.latency = latency;
            const ServoRun g = RunServo(sim, true, false);
            gunAll = gunAll && g.gunReversals == 0 && g.finalErrDeg < 1.0f;
            sim.occupant = true;
            const ServoRun o = RunServo(sim, true, false);
            occupantAll = occupantAll && o.gunReversals == 0 && o.finalErrDeg < 1.0f;
        }
    Check(gunAll, "paced on the gun: it lands within 1 deg and never reverses, at every rate and latency");
    Check(occupantAll, "paced on the occupant's aim: the same");
    {   // a hand that moves: the gun follows it, reversing only where the hand does
        TurretSim sim;
        const ServoRun r = RunServo(sim, true, true);
        Check(r.gunReversals <= r.handReversals && r.maxTrackErrDeg < 3.0f,
              "paced, the gun follows a moving hand within 3 deg and reverses only with it");
    }

    ServoPacer p;
    Check(p.pace(0.0f, 0.3f, 1.0f, kAimServoStickFloor) == 1.0f && p.waiting, "a command goes out as one pulse");
    Check(p.pace(0.0f, 0.3f, 1.0f, kAimServoStickFloor) == 0.0f && p.holds == 1, "then it waits for an answer");
    Check(p.pace(0.02f, 0.28f, 1.0f, kAimServoStickFloor) == 0.0f, "the feedback moving: still waiting");
    Check(p.pace(0.0202f, 0.2798f, 1.0f, kAimServoStickFloor) == 1.0f, "moved and at rest: the next pulse");
    ServoPacer t;
    t.timeoutPolls = 3;
    (void)t.pace(0.0f, 0.3f, 1.0f, kAimServoStickFloor);
    for (int i = 0; i < 2; ++i) (void)t.pace(0.0f, 0.3f, 1.0f, kAimServoStickFloor);
    Check(t.pace(0.0f, 0.3f, 1.0f, kAimServoStickFloor) == 1.0f, "a feedback that never answers is asked again");
    ServoPacer q;
    (void)q.pace(0.0f, 0.01f, 0.12f, kAimServoStickFloor);    // a floor pulse from rest
    (void)q.pace(0.005f, 0.005f, 0.0f, kAimServoStickFloor);  // it moves 0.29 deg
    (void)q.pace(0.005f, 0.005f, 0.0f, kAimServoStickFloor);  // and rests: that is the quantum
    Check(std::fabs(q.quantum - 0.005f) < 1e-6f, "a floor pulse's measured effect is the quantum");
    Check(q.enter(0.001f) == 0.003f && q.exit(0.001f) == 0.0075f, "the rest band widens to it");
    Check(q.enter(0.01f) == 0.01f, "never narrower than the servo's own band");
    ServoPacer w;
    w.wrap = true;
    (void)w.pace(3.13f, 0.1f, 0.0f, kAimServoStickFloor);
    Check(w.pace(-3.13f, 0.1f, 1.0f, kAimServoStickFloor) == 0.0f, "yaw wraps: +pi to -pi is arriving, not leaving");
    ServoPacer n;
    Check(n.pace(std::nanf(""), 0.1f, 0.5f, kAimServoStickFloor) == 0.5f, "no feedback: the command as it was");
}

void TestGunSettle()
{
    GunSettle s;
    Check(!s.step(0.05f, 1000), "3 deg off: unsettled");
    Check(!s.step(0.01f, 1100), "inside 1 deg: not yet 120 ms");
    Check(s.episodeDone && s.lastSettleMs == 100, "the gun took 100 ms to land");
    Check(s.step(0.012f, 1220), "120 ms inside: settled");
    Check(!s.step(0.02f, 1230), "it leaves the band: unsettled at once");
    Check(!s.step(std::nanf(""), 1240), "nothing written to compare: unsettled");
    s.episodeDone = false;
    (void)s.step(0.001f, 1300);
    Check(!s.episodeDone, "no episode is timed across a gap with no write");
    s.reset();
    Check(!s.settled && !s.inBand && s.tol == 0.01745f, "reset keeps the band");
}

void TestSlewRate()
{
    SlewRate r;
    r.step(0.0f, 0.0f, 1000);
    const float rate = r.step(0.0f, 0.0349066f, 1016);   // 2 deg in 16 ms
    Check(std::fabs(rate - 125.0f) < 1.0f && std::fabs(r.peakDegPerS - 125.0f) < 1.0f, "2 deg in 16 ms: 125 deg/s");
    Check(r.step(0.0f, 0.0f, 2000) == 0.0f, "a gap over 100 ms is no rate");
    Check(r.step(0.0f, 0.0f, 2000) == 0.0f, "a repeated timestamp is no rate");
}

void TestNodWitness()
{
    NodWitness n;
    for (int i = 0; i < 300; ++i)
        n.step(0.1f * std::sin(i * 0.1f));   // a 11.5 deg peak-to-peak nod, period ~63 polls
    unsigned rev = 0;
    float swing = 0.0f;
    n.take(rev, swing);
    Check(rev >= 8 && rev <= 10, "a sine nod reverses twice a period");
    Check(std::fabs(swing - 0.2f) < 0.01f, "its swing is peak to peak");
    n.take(rev, swing);
    Check(rev == 0 && swing == 0.0f, "take empties the window");
    NodWitness q;
    for (int i = 0; i < 300; ++i)
        q.step(0.3f + 0.001f * ((i % 2) ? 1.0f : -1.0f));   // 0.1 deg of chatter
    q.take(rev, swing);
    Check(rev == 0, "chatter inside the deadband is no nod");
    NodWitness ramp;
    for (int i = 0; i < 100; ++i) ramp.step(i * 0.01f);
    ramp.take(rev, swing);
    Check(rev == 0, "a steady climb is no reversal");
}
} // namespace

int main()
{
    TestSeatFlag();
    TestBinding();
    TestAngles();
    TestOffsetWatch();
    TestServoFeedback();
    TestPacer();
    TestGunSettle();
    TestSlewRate();
    TestNodWitness();
    std::printf("Gun-slaved seats: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
