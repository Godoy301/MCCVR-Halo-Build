#pragma once
// GUN-SLAVED SEATS (Halo 3 / ODST seat tag flag bit 7, "first person camera slaved to gun"): the pure half.
//
// What the game does. The first-person camera function (Halo 3 0x17DAEC, ODST 0x1AC670) takes the camera's
// forward from the unit's own aim vector (Halo 3 unit+0x1AC, ODST unit+0x1A4) -- unless the unit sits in a seat
// whose tag flags carry bit 7, where it takes the position and forward from a marker on the PARENT, the gun. The
// Warthog turret's gunner seat (flags 0x00101198) is one. So in that seat the camera forward this mod reads as
// "the game's aim" (g_aimFwd) is the GUN, and the gun slews toward the aim the record asks for at its own rate
// (~125-140 deg/s on the Warthog turret, measured). Comparing the record against it -- the seat's per-axis proof,
// the arm gate, the stick servo before the proof -- compares it against a quantity that lags by design: turret
// stints spent 1-2 s unwritten, short ones never wrote, and with the pitch left on the stick servo the servo
// closed its loop on the lagging gun, ran the record past the target, and the gun nodded.
//
// What the record really drives is the OCCUPANT's own aim vector, the same one the camera reads on foot. The
// control update proves that relation itself: it re-derives the record's desired angles from exactly that vector
// (Halo 3 0xF7082, ODST 0x112C5D: yaw = atan2(y, x), pitch from z, written to the record's +0xD4 / +0xD8). So in a
// gun-slaved seat the record is proven against the occupant's aim; the gun stays the outcome (witnesses only).
//
// Everything here is engine-free and unit-tested (tests/gun_slaved_seat_tests.cpp).

#include <cmath>
#include <cstdint>

namespace gun_slaved
{
constexpr float kPi = 3.14159265f;
constexpr float kTwoPi = 6.28318531f;
constexpr float kRadToDeg = 57.2957795f;

// ---- the seat ------------------------------------------------------------------------------------------------
// Bit 7 of a seat's tag flags (H3EK unit seat flags: "first person camera slaved to gun"). Halo 3 reads it from
// the occupied seat's ORIGINAL flags (the C20 patch keeps them; it only clears bit 4), ODST from the live seat word.
constexpr uint32_t kCameraSlavedToGunBit = 1u << 7;
constexpr bool CameraSlavedToGun(uint32_t seatFlags) noexcept
{
    return (seatFlags & kCameraSlavedToGunBit) != 0;
}

// ---- the binding: where the occupant's aim vector lives ------------------------------------------------------
// Two independent consumers, each found by a unique signature, must agree on the offset: the camera function's
// forward read (with the seat stride and the bit it tests, so the bit is proven to be the one this file names)
// and the control update's reset of the record from the same vector (whose two writes must land on the record's
// own yaw / pitch, the angles the direct drive writes). The z component is read 8 bytes on in both.
struct UnitAimSites
{
    int32_t cameraForward = 0;     // movsd xmm0, [unit + X]
    int32_t cameraForwardZ = 0;    // mov eax, [unit + X + 8]
    int32_t cameraSeatStride = 0;  // imul rdx, rax, stride (the seat block's element size)
    uint8_t cameraFlagShift = 0;   // shr eax, N; test r?b, al  (N = 7)
    int32_t controlForward = 0;    // movsd xmm0, [unit + X]
    int32_t controlForwardZ = 0;   // mov eax, [unit + X + 8]
    int32_t controlPitchWrite = 0; // movss [record + P], xmm0
    int32_t controlYawWrite = 0;   // movss [record + Y], xmm0
};

// The offset, or 0 with `why` naming the first disagreement. `seatStride` and `recordAngles` are the mod's own
// proven constants (the seat block stride, the record's desired-angle pair).
inline uint32_t ResolveUnitAimOffset(const UnitAimSites& s, int32_t seatStride, int32_t recordAngles,
                                     const char*& why) noexcept
{
    why = nullptr;
    if (s.cameraFlagShift != 7)
        why = "the camera function does not test seat flag bit 7";
    else if (s.cameraSeatStride != seatStride)
        why = "the camera function's seat stride is not the seat block's";
    else if (s.cameraForwardZ != s.cameraForward + 8 || s.controlForwardZ != s.controlForward + 8)
        why = "a consumer does not read the vector's z beside its x/y";
    else if (s.cameraForward != s.controlForward)
        why = "the camera and the control update read different fields";
    else if (s.controlYawWrite != recordAngles || s.controlPitchWrite != recordAngles + 4)
        why = "the control update's reset does not write the record's desired angles";
    else if (s.cameraForward < 0x100 || s.cameraForward > 0x800 || (s.cameraForward & 3) != 0)
        why = "the offset is outside a unit's aim block";
    return why ? 0u : static_cast<uint32_t>(s.cameraForward);
}

// The game keeps the vector normalised (it asserts so). Anything else is not this vector: no sample.
inline bool AnglesFromVector(const float v[3], float& yaw, float& pitch) noexcept
{
    if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
        return false;
    const float length = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!(length > 0.95f && length < 1.05f))
        return false;
    yaw = std::atan2(v[1], v[0]);
    const float z = v[2] / length;
    pitch = std::asin(z > 1.0f ? 1.0f : (z < -1.0f ? -1.0f : z));
    return std::isfinite(yaw) && std::isfinite(pitch);
}

// The angle between two directions given as (yaw, pitch), in degrees. NaN when either is not finite.
inline float AngleBetweenDeg(float yawA, float pitchA, float yawB, float pitchB) noexcept
{
    if (!std::isfinite(yawA) || !std::isfinite(pitchA) || !std::isfinite(yawB) || !std::isfinite(pitchB))
        return std::nanf("");
    const float ca = std::cos(pitchA), cb = std::cos(pitchB);
    float dot = ca * std::cos(yawA) * cb * std::cos(yawB) + ca * std::sin(yawA) * cb * std::sin(yawB) +
                std::sin(pitchA) * std::sin(pitchB);
    dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
    return std::acos(dot) * kRadToDeg;
}

// ---- the outcome guard: lag, or a real offset? ---------------------------------------------------------------
// The occupant's aim proves that the record holds the aim the record drives. It cannot prove that the GUN then
// points there: a seat whose gun marker sat at a constant angle from the occupant's aim would pass the occupant's
// check, be written, and leave the gun off the hand -- where the camera check (failing on that offset, by design)
// had kept the axis on the servo, closing on the gun. Lag and a real offset part company when the gun comes to REST:
// lag is gone then, an offset is not. So once an axis is written on the occupant's proof alone, this watch steps on
// every new sample of that axis: with the gun and the record both at rest (each moved under `rest` since the last
// sample) a gap over `tol` counts, anything under resets, and `needed` such samples in a row REVOKE the occupant's
// proof of that axis for the rest of the stint (the camera's check alone decides it again: today's behaviour).
// A gun at a stop the record shares (the accumulator clamps both to the seat's range) shows no gap.
struct OffsetWatch
{
    float tol = 0.0524f;     // 3 deg: farther than this, a gun at rest is not lagging
    float rest = 0.00175f;   // 0.1 deg per sample: at rest
    unsigned needed = 30;
    bool wrap = false;

    bool have = false;
    float lastGun = 0.0f, lastRecord = 0.0f;
    unsigned count = 0;
    bool revoked = false;

    void reset() noexcept
    {
        have = revoked = false;
        count = 0;
        lastGun = lastRecord = 0.0f;
    }

    bool step(float record, float gun) noexcept
    {
        if (!std::isfinite(record) || !std::isfinite(gun))
        {
            have = false;
            return revoked;
        }
        const auto diff = [this](float a, float b) {
            const float d = a - b;
            return wrap ? std::remainder(d, kTwoPi) : d;
        };
        if (have && std::fabs(diff(gun, lastGun)) < rest && std::fabs(diff(record, lastRecord)) < rest)
        {
            if (std::fabs(diff(record, gun)) > tol)
            {
                if (++count >= needed)
                    revoked = true;
            }
            else
            {
                count = 0;
            }
        }
        lastGun = gun;
        lastRecord = record;
        have = true;
        return revoked;
    }
};

// ---- the stick servo's feedback ------------------------------------------------------------------------------
// Where a gun-slaved seat's axis is still steered (before its proof, or after a failed one), the servo closes on
// the occupant's aim once that aim has agreed with the record on this axis for `trust` consecutive judged samples
// of this stint (the record is what the stick moves, so this feedback has no lag). Otherwise -- no binding, no
// fresh sample, a disagreement, any other seat -- it closes on the camera's forward exactly as before.
enum class Feedback : uint8_t
{
    Gun = 0,       // the camera forward (g_aimFwd); in a gun-slaved seat, the gun
    Occupant = 1,  // the occupant's own aim, proven against the record this stint
};
constexpr unsigned kServoTrustSamples = 8;
constexpr Feedback ServoFeedback(bool gunSlaved, bool occupantFresh, unsigned agreeingSamples) noexcept
{
    return gunSlaved && occupantFresh && agreeingSamples >= kServoTrustSamples ? Feedback::Occupant : Feedback::Gun;
}

// THE PACER: a servo in a gun-slaved seat (whichever feedback it closes on) commands in PULSES. The stick moves the
// record, and the feedback answers only after the input path, the control update, the tick and -- for the gun --
// its slew: polls later. A continuous command keeps pushing through that latency, the record runs past the target,
// the feedback arrives late, the error flips sign and the loop drives back: the nod. The rest hysteresis
// (AimServoCommand) cannot catch it, because the overshoot is the latency's, not the stick's quantum (its measured
// quantum read 0.0000 in every turret stint of the 2026-10-02 log: the gun never moved on the poll after a command).
// So the pacer lets one poll's command through, then holds zero until the feedback has moved and come to rest (or
// `timeoutPolls` pass: a stop that never answers is asked again), and holds while the feedback is still travelling
// toward the target. It also measures what one floor-level pulse moved the feedback by, from rest -- the quantum
// AimServoObserve cannot see through the latency -- and widens the rest band to it (capped), so the last pulse
// cannot hunt around the target either. Per axis; the yaw axis wraps. Engaged only in gun-slaved seats.
struct ServoPacer
{
    float moving = 0.0009f;        // ~0.05 deg per poll: less is at rest
    unsigned timeoutPolls = 24;    // ~200 ms at 120 polls/s
    float quantumCap = 0.035f;     // 2 deg: the widest the measured quantum may make the rest band
    bool wrap = false;

    bool have = false;
    float last = 0.0f;
    bool waiting = false, moved = false, floorPulse = false;
    unsigned waited = 0;
    float pulseFrom = 0.0f;
    float quantum = 0.0f;          // radians: one floor pulse's measured effect
    unsigned holds = 0;            // polls whose non-zero command was held (the caller's dev line takes it)

    void reset() noexcept
    {
        have = waiting = moved = floorPulse = false;
        waited = 0;
        last = pulseFrom = quantum = 0.0f;
        holds = 0;
    }

    float enter(float base) const noexcept { return quantum * 0.6f > base ? quantum * 0.6f : base; }
    float exit(float base) const noexcept { return quantum * 1.5f > base ? quantum * 1.5f : base; }

    // `command`: AimServoCommand's (with enter()/exit() as its rest bands); `floorMagnitude`: the deflection the
    // stick mapping floors smaller commands to. Returns the command to issue this poll.
    float pace(float feedback, float error, float command, float floorMagnitude) noexcept
    {
        if (!std::isfinite(feedback) || !std::isfinite(error))
        {
            have = waiting = false;
            return command;   // nothing to pace against: the servo as it was
        }
        float d = have ? feedback - last : 0.0f;
        if (wrap)
            d = std::remainder(d, kTwoPi);
        last = feedback;
        have = true;
        const bool movingNow = std::fabs(d) > moving;
        if (waiting)
        {
            ++waited;
            if (movingNow)
                moved = true;
            if ((moved && !movingNow) || waited >= timeoutPolls)
            {
                waiting = false;
                if (floorPulse && moved)
                {
                    float step = feedback - pulseFrom;
                    if (wrap)
                        step = std::remainder(step, kTwoPi);
                    step = std::fabs(step);
                    if (step > quantumCap)
                        step = quantumCap;
                    // Biased low: a pulse measured with an earlier command still in flight reads too far.
                    quantum = (quantum == 0.0f || step < quantum) ? step : quantum * 0.8f + step * 0.2f;
                }
            }
            else
            {
                if (command != 0.0f)
                    ++holds;
                return 0.0f;
            }
        }
        if (command == 0.0f)
            return 0.0f;
        if (movingNow && (d > 0.0f) == (error > 0.0f))
        {
            ++holds;   // still arriving
            return 0.0f;
        }
        waiting = true;
        moved = false;
        waited = 0;
        floorPulse = std::fabs(command) <= floorMagnitude && !movingNow;
        pulseFrom = feedback;
        return command;
    }
};

// ---- the outcome: has the gun landed where it was written? --------------------------------------------------
// The engine-crosshair witness scores a turret only once the gun has settled: within `tol` of the last write for
// `hold_ms`. Each unsettled episode's length (from leaving the band to re-entering it) is the gun's settle time.
struct GunSettle
{
    float tol = 0.01745f;                 // 1 deg
    unsigned long long hold_ms = 120;

    bool inBand = false;
    bool settled = false;
    unsigned long long inSinceMs = 0, outSinceMs = 0;
    unsigned long long lastSettleMs = 0;  // the last completed episode
    bool episodeDone = false;             // a new lastSettleMs since the caller last took it

    void reset() noexcept { *this = GunSettle{tol, hold_ms}; }

    // `missRad`: the gun's angle from the nearer of the last two writes. NaN (no write to compare): not settled,
    // and no episode is timed across it.
    bool step(float missRad, unsigned long long nowMs) noexcept
    {
        if (!nowMs)
            nowMs = 1;
        if (!std::isfinite(missRad))
        {
            inBand = settled = false;
            inSinceMs = outSinceMs = 0;
            return false;
        }
        if (missRad > tol)
        {
            if (inBand || !outSinceMs)
                outSinceMs = nowMs;
            inBand = settled = false;
            inSinceMs = 0;
            return false;
        }
        if (!inBand)
        {
            inBand = true;
            inSinceMs = nowMs;
            if (outSinceMs && nowMs >= outSinceMs)
            {
                lastSettleMs = nowMs - outSinceMs;
                episodeDone = true;
            }
            outSinceMs = 0;
        }
        settled = nowMs >= inSinceMs && nowMs - inSinceMs >= hold_ms;
        return settled;
    }
};

// The gun's angular speed between successive polls (deg/s), and the peak of it.
struct SlewRate
{
    bool have = false;
    float yaw = 0.0f, pitch = 0.0f;
    unsigned long long ms = 0;
    float peakDegPerS = 0.0f;

    void reset() noexcept { *this = SlewRate{}; }

    float step(float yawRad, float pitchRad, unsigned long long nowMs) noexcept
    {
        float rate = 0.0f;
        if (!std::isfinite(yawRad) || !std::isfinite(pitchRad))
        {
            have = false;
            return rate;
        }
        if (have && nowMs > ms && nowMs - ms <= 100)
        {
            const float deg = AngleBetweenDeg(yaw, pitch, yawRad, pitchRad);
            if (std::isfinite(deg))
            {
                rate = deg * 1000.0f / static_cast<float>(nowMs - ms);
                if (rate > peakDegPerS)
                    peakDegPerS = rate;
            }
        }
        if (!have || nowMs != ms)
        {
            yaw = yawRad;
            pitch = pitchRad;
            ms = nowMs;
        }
        have = true;
        return rate;
    }
};

// THE NOD WITNESS: direction reversals of one angle (the gun's pitch), with hysteresis, and the largest swing
// between two successive turning points. A hand that nods reverses too, so the caller runs one on the hand's
// target beside it: a gun that reverses where the hand does not is the loop's own nod.
struct NodWitness
{
    float deadband = 0.0035f;   // 0.2 deg: a smaller wobble is not a reversal
    bool have = false;
    int dir = 0;                // +1 rising, -1 falling, 0 not yet moving
    float peak = 0.0f;          // the running extreme in `dir`
    bool haveTurn = false;
    float lastTurn = 0.0f;
    unsigned reversals = 0;
    float maxSwing = 0.0f;      // radians, peak to peak

    void reset() noexcept { *this = NodWitness{deadband}; }

    void step(float v) noexcept
    {
        if (!std::isfinite(v))
        {
            have = false;
            dir = 0;
            haveTurn = false;
            return;
        }
        if (!have)
        {
            have = true;
            dir = 0;
            peak = v;
            return;
        }
        if (dir == 0)
        {
            if (v - peak > deadband) { dir = 1; peak = v; }
            else if (peak - v > deadband) { dir = -1; peak = v; }
            return;
        }
        if (dir > 0)
        {
            if (v > peak) peak = v;
            else if (peak - v > deadband) { turn(); dir = -1; peak = v; }
        }
        else
        {
            if (v < peak) peak = v;
            else if (v - peak > deadband) { turn(); dir = 1; peak = v; }
        }
    }

    // The counts since the last take (the trend state is kept, so a reversal straddling a report is not lost).
    void take(unsigned& outReversals, float& outMaxSwingRad) noexcept
    {
        outReversals = reversals;
        outMaxSwingRad = maxSwing;
        reversals = 0;
        maxSwing = 0.0f;
    }

private:
    void turn() noexcept
    {
        ++reversals;
        if (haveTurn)
        {
            const float swing = std::fabs(peak - lastTurn);
            if (swing > maxSwing)
                maxSwing = swing;
        }
        lastTurn = peak;
        haveTurn = true;
    }
};
} // namespace gun_slaved
