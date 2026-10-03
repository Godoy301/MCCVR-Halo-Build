// Seated aim upstream of replication (src/common/seat_upstream_logic.h): the stock query model and its exact inverse,
// the gain learned from the game's own answers, the write that was in effect at a tick, the seated recomposition's
// fail-closed rules, and the stock-line witness.
#include "seat_upstream_logic.h"

#include <cmath>
#include <cstdio>
#include <cstring>

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
constexpr float kWuPerM = shot_lag::kWuPerMetre;

float LineMissWu(const float origin[3], const float direction[3], const float point[3])
{
    const float l = shot_lag::Length(direction);
    const float d[3]{direction[0] / l, direction[1] / l, direction[2] / l};
    const float w[3]{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    const float t = shot_lag::Dot(w, d);
    const float m[3]{w[0] - d[0] * t, w[1] - d[1] * t, w[2] - d[2] * t};
    return shot_lag::Length(m);
}

void TestModelAndInverse()
{
    const float r[3]{1.0f, 0.0f, 0.0f};
    const float still[3]{};
    float q[3]{};
    Check(seat_upstream::StockQuery(r, still, 0.016f, q) && std::fabs(q[0] - 1.0f) < 1e-6f,
          "at rest the stock query is the record");
    // A Hornet-like strafe: 17 m/s sideways, 1/g = 190 m/s -> the query leaves the record by atan(17/190) ~ 5.1 deg,
    // against the travel.
    const float v[3]{0.0f, 17.0f * kWuPerM, 0.0f};
    const float g = 1.0f / (190.0f * kWuPerM);
    Check(seat_upstream::StockQuery(r, v, g, q), "the model answers");
    const float dev = seat_upstream::AngleDeg(r, q);
    Check(std::fabs(dev - std::atan(17.0f / 190.0f) * shot_lag::kRadToDeg) < 0.02f,
          "a 17 m/s strafe at 1/g = 190 m/s moves the query ~5.1 deg");
    Check(q[1] < 0.0f, "the query leaves the record against the travel");

    // The inverse is exact: the record it returns maps back onto the wanted direction.
    const float wants[][3]{{1.0f, 0.0f, 0.0f}, {0.6f, 0.7f, -0.2f}, {0.1f, -0.9f, 0.4f}, {-0.3f, 0.2f, 0.93f}};
    const float vels[][3]{{0, 0, 0}, {0, 17.0f * kWuPerM, 0}, {12.0f * kWuPerM, -9.0f * kWuPerM, 3.0f * kWuPerM},
                          {40.0f * kWuPerM, 0, 0}};
    const float gains[]{0.0f, g, 1.0f / (440.0f * kWuPerM), 1.0f / (60.0f * kWuPerM)};
    for (const auto& want : wants)
        for (const auto& vel : vels)
            for (const float gain : gains)
            {
                float rec[3]{}, back[3]{};
                const auto why = seat_upstream::RecordForQuery(want, vel, gain, rec);
                if (why != seat_upstream::Invert::Solved)
                {
                    // Only possible when the velocity across the line is at least 1/g.
                    float w[3]{want[0], want[1], want[2]};
                    seat_upstream::Normalize(w);
                    const float along = shot_lag::Dot(w, vel);
                    const float across2 = shot_lag::Dot(vel, vel) - along * along;
                    Check(gain * gain * across2 >= 0.999f, "the inverse refuses only an unreachable query");
                    continue;
                }
                Check(std::fabs(shot_lag::Length(rec) - 1.0f) < 1e-5f, "the inverse's record is a unit vector");
                Check(seat_upstream::StockQuery(rec, vel, gain, back), "its stock query answers");
                float w[3]{want[0], want[1], want[2]};
                seat_upstream::Normalize(w);
                Check(seat_upstream::AngleDeg(w, back) < 1.0e-3f, "the inverse's record maps exactly onto the query");
            }
    float out[3]{9, 9, 9};
    const float zero[3]{};
    Check(seat_upstream::RecordForQuery(zero, v, g, out) == seat_upstream::Invert::Degenerate && out[0] == 9.0f,
          "a zero query is refused and writes nothing");
    const float nan[3]{NAN, 0, 0};
    Check(seat_upstream::RecordForQuery(nan, v, g, out) == seat_upstream::Invert::NonFinite, "non-finite refused");
    // 1/g below the speed across the line: no record reaches it (the game could not aim there either).
    const float fast[3]{0.0f, 300.0f * kWuPerM, 0.0f};
    Check(seat_upstream::RecordForQuery(r, fast, g, out) == seat_upstream::Invert::NoSolution,
          "a query the model cannot reach is refused");
}

void TestGainFit()
{
    // The game's answers under a known gain, at varied velocities and records: the fit recovers it.
    const float g = 1.0f / (190.0f * kWuPerM);
    seat_upstream::GainFit fit;
    Check(!fit.Learned(seat_upstream::kGainMinWeight), "nothing learned before a sample");
    unsigned seed = 12345u;
    const auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return float((seed >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f;
    };
    for (int i = 0; i < 200; ++i)
    {
        float rec[3]{1.0f, 0.3f * rnd(), 0.2f * rnd()};
        seat_upstream::Normalize(rec);
        const float vel[3]{6.0f * rnd() * kWuPerM, 18.0f * rnd() * kWuPerM, 3.0f * rnd() * kWuPerM};
        float q[3]{};
        seat_upstream::StockQuery(rec, vel, g, q);
        fit.Add(rec, q, vel, seat_upstream::kGainMinCrossWuPerS, seat_upstream::kGainDecay);
    }
    Check(fit.Learned(seat_upstream::kGainMinWeight), "learned after a strafe");
    Check(std::fabs(fit.Gain() / g - 1.0f) < 1e-3f, "the fit recovers the game's gain");

    // At rest no sample informs it: none used, nothing learned.
    seat_upstream::GainFit still;
    const float rec[3]{1, 0, 0}, q[3]{1, 0, 0}, slow[3]{0.0f, 1.0f * kWuPerM, 0.0f};
    for (int i = 0; i < 50; ++i)
        still.Add(rec, q, slow, seat_upstream::kGainMinCrossWuPerS, seat_upstream::kGainDecay);
    Check(!still.Learned(seat_upstream::kGainMinWeight) && still.used == 0 && still.ignored == 50,
          "slow samples (under 2 m/s across) carry no information");

    // A query that ignores velocity (a passenger whose aim is the record) fits g ~ 0.
    seat_upstream::GainFit none;
    const float fastAcross[3]{0.0f, 15.0f * kWuPerM, 0.0f};
    for (int i = 0; i < 60; ++i)
        none.Add(rec, rec, fastAcross, seat_upstream::kGainMinCrossWuPerS, seat_upstream::kGainDecay);
    Check(none.Learned(seat_upstream::kGainMinWeight) && std::fabs(none.Gain()) < 1e-7f, "no velocity term fits 0");

    // A weapon change re-learns: the recent answers dominate after ~1/(1 - decay) samples.
    seat_upstream::GainFit change;
    const float g2 = 1.0f / (440.0f * kWuPerM);
    for (int pass = 0; pass < 2; ++pass)
        for (int i = 0; i < 300; ++i)
        {
            float r2[3]{1.0f, 0.2f * rnd(), 0.1f * rnd()};
            seat_upstream::Normalize(r2);
            const float vel[3]{0.0f, 16.0f * kWuPerM * (i % 2 ? 1.0f : -1.0f), 0.0f};
            float q2[3]{};
            seat_upstream::StockQuery(r2, vel, pass ? g2 : g, q2);
            change.Add(r2, q2, vel, seat_upstream::kGainMinCrossWuPerS, seat_upstream::kGainDecay);
        }
    Check(std::fabs(change.Gain() / g2 - 1.0f) < 0.01f, "a new gain replaces the old within a few hundred samples");
}

void TestWriteRing()
{
    seat_upstream::WriteRing ring;
    const float a[3]{1, 0, 0}, b[3]{0, 1, 0}, c[3]{0, 0, 1};
    Check(!ring.InEffect(10, 7), "empty: nothing in effect");
    ring.Note(10, 7, a);
    Check(!ring.InEffect(10, 7), "a write no tick has consumed is not in effect");
    Check(ring.InEffect(11, 7) && ring.InEffect(11, 7)->dir[0] == 1.0f, "consumed by the next tick");
    ring.Note(11, 7, b);   // a pass with no tick in between...
    ring.Note(11, 7, c);   // ...then another write at the same count
    Check(ring.InEffect(11, 7)->dir[0] == 1.0f, "writes of a pass with no tick yet leave the older one in effect");
    Check(ring.InEffect(12, 7)->dir[2] == 1.0f, "after a tick the newest write is in effect");
    Check(!ring.InEffect(12, 8), "another seat's write is never this seat's");
    ring.Note(12, 7, nullptr);   // a stick call that wrote nothing (disarmed, unproven axis): the record is unknown
    Check(!ring.InEffect(13, 7), "a call that left a record we did not write blocks the pairing");
    ring.Note(13, 7, a);
    Check(ring.InEffect(14, 7) && ring.InEffect(14, 7)->dir[0] == 1.0f, "the next write pairs again");
    // Wrap-safe tick counts.
    seat_upstream::WriteRing wrap;
    wrap.Note(0xFFFFFFFFu, 3, a);
    Check(wrap.InEffect(0u, 3) != nullptr, "the tick counter's wrap is handled");
}

seat_upstream::SeatRecomposeInput HornetInput()
{
    seat_upstream::SeatRecomposeInput in{};
    in.enabled = true;
    in.seated = true;
    in.intentRead = true;
    in.generation = 5;
    in.nowUs = 1'000'000;
    in.maxAgeUs = 150'000;
    shot_lag::IntentPoint& p = in.intent;
    p.generation = 5;
    p.stampUs = 990'000;
    p.seated = true;
    // P 53 m ahead of a render eye at the origin; the solve published the angles from that eye.
    const float point[3]{53.0f * kWuPerM, 0.0f, 0.0f};
    std::memcpy(in.point, point, sizeof(point));
    p.solvedYaw = 0.0f;
    p.solvedPitch = 0.0f;
    in.setYaw = 0.0f;
    in.setPitch = 0.0f;
    // The tick's camera 0.8 m ahead and 0.5 m right of the render eye, strafing at 17 m/s.
    in.cameraRead = true;
    in.camera[0] = 0.8f * kWuPerM;
    in.camera[1] = -0.5f * kWuPerM;
    in.camera[2] = 0.0f;
    in.cameraVel[1] = 17.0f * kWuPerM;
    in.cameraVelValid = true;
    in.tickS = 1.0f / 60.0f;
    in.gain = 1.0f / (190.0f * kWuPerM);
    return in;
}

void TestRecomposeSeat()
{
    seat_upstream::SeatRecomposeInput in = HornetInput();
    float yaw = 0.0f, pitch = 0.0f, bend = 0.0f;
    Check(seat_upstream::RecomposeSeat(in, yaw, pitch, bend) == seat_upstream::SeatRecompose::Recomposed,
          "a fresh seated intent with this pass's camera recomposes");
    // The stock query of the new record, from the camera a tick on, passes through P.
    float rec[3]{}, q[3]{};
    seat_upstream::DirFromAngles(yaw, pitch, rec);
    seat_upstream::StockQuery(rec, in.cameraVel, in.gain, q);
    const float cam[3]{in.camera[0], in.camera[1] + in.cameraVel[1] * in.tickS, in.camera[2]};
    Check(LineMissWu(cam, q, in.point) * shot_lag::kMmPerWu < 1.0f, "the stock line from the tick's camera hits P");
    // Before: the published angles' stock line missed by metres (the Hornet's 3-4 m at 53 m).
    float set[3]{}, q0[3]{};
    seat_upstream::DirFromAngles(in.setYaw, in.setPitch, set);
    seat_upstream::StockQuery(set, in.cameraVel, in.gain, q0);
    Check(LineMissWu(cam, q0, in.point) * shot_lag::kMetresPerWu > 3.0f, "the published angles missed by metres");
    Check(bend > 4.0f && bend < 7.0f, "the bend is the gain's ~5 deg plus the camera offset");

    // With no learned gain the camera alone is still solved from: the identity model's line hits P.
    in.gain = 0.0f;
    Check(seat_upstream::RecomposeSeat(in, yaw, pitch, bend) == seat_upstream::SeatRecompose::Recomposed,
          "unlearned: still recomposed from the camera");
    seat_upstream::DirFromAngles(yaw, pitch, rec);
    Check(LineMissWu(cam, rec, in.point) * shot_lag::kMmPerWu < 1.0f, "unlearned: the record's own line hits P");

    // Fail closed, each reason, and nothing written.
    const auto refuses = [](seat_upstream::SeatRecomposeInput i, seat_upstream::SeatRecompose why, const char* what) {
        float y = 7.0f, p = 7.0f, b = 7.0f;
        Check(seat_upstream::RecomposeSeat(i, y, p, b) == why && y == 7.0f && p == 7.0f && b == 7.0f, what);
    };
    auto i = HornetInput();
    i.enabled = false;
    refuses(i, seat_upstream::SeatRecompose::Off, "off: kept");
    i = HornetInput();
    i.seated = false;
    refuses(i, seat_upstream::SeatRecompose::NotSeated, "not seated: kept");
    i = HornetInput();
    i.gunSlaved = true;
    refuses(i, seat_upstream::SeatRecompose::GunSlaved, "a gun-slaved seat: kept (the gun's line is not modelled)");
    i = HornetInput();
    i.intentRead = false;
    refuses(i, seat_upstream::SeatRecompose::NoIntent, "no intent: kept");
    i = HornetInput();
    i.setYaw = 0.01f;
    refuses(i, seat_upstream::SeatRecompose::OtherSolve, "a setpoint from another author: kept");
    i = HornetInput();
    i.intent.stampUs = 800'000;
    refuses(i, seat_upstream::SeatRecompose::PointUnusable, "a stale point: kept");
    i = HornetInput();
    i.intent.seated = false;
    refuses(i, seat_upstream::SeatRecompose::PointUnusable, "an on-foot point: kept");
    i = HornetInput();
    i.intent.railHeld = true;
    refuses(i, seat_upstream::SeatRecompose::PointUnusable, "held at the pitch rail: kept");
    i = HornetInput();
    i.intent.generation = 4;
    refuses(i, seat_upstream::SeatRecompose::PointUnusable, "another generation: kept");
    i = HornetInput();
    i.cameraRead = false;
    refuses(i, seat_upstream::SeatRecompose::NoCamera, "no camera this pass: kept");
    i = HornetInput();
    i.point[0] = i.camera[0] + 0.1f * kWuPerM;
    i.point[1] = i.camera[1] + i.cameraVel[1] * i.tickS;
    refuses(i, seat_upstream::SeatRecompose::PointTooClose, "P within 30 cm: kept");
    i = HornetInput();
    i.point[0] = -10.0f * kWuPerM;
    refuses(i, seat_upstream::SeatRecompose::PointBehind, "P behind: kept");
    i = HornetInput();
    i.cameraVel[1] = 300.0f * kWuPerM;
    i.point[1] = 0.0f;
    refuses(i, seat_upstream::SeatRecompose::NoSolution, "an unreachable query: kept");

    // No magnitude limit: a 25 deg bend (a camera far off the solve's eye) is recomposed like any other.
    i = HornetInput();
    i.gain = 0.0f;
    i.camera[1] = -20.0f * kWuPerM;
    i.camera[0] = 10.0f * kWuPerM;
    Check(seat_upstream::RecomposeSeat(i, yaw, pitch, bend) == seat_upstream::SeatRecompose::Recomposed && bend > 20.0f,
          "no size limit on the bend");
}

void TestStockLineWindow()
{
    const float start[3]{0, 0, 0}, vector[3]{1, 0, 0};
    const float point[3]{50.0f * kWuPerM, 1.0f * kWuPerM, 0.0f};
    const float vel[3]{0.0f, -10.0f * kWuPerM, 0.0f};
    const auto s = seat_upstream::MeasureStockLine(start, vector, point, vel);
    Check(s.valid && std::fabs(s.missMm - 1000.0f) < 1.0f, "the miss of P off the stock line");
    Check(std::fabs(s.angleDeg - std::atan(1.0f / 50.0f) * shot_lag::kRadToDeg) < 0.01f, "its angle at the start");
    Check(s.travel && std::fabs(s.alongMm - 1000.0f) < 1.0f,
          "split along the travel (+ = the line runs ahead of P along the travel, shot_lag's convention)");
    seat_upstream::StockLineWindow w;
    w.tag = 1;
    w.Add(s);
    w.Add(s);
    char line[512]{};
    Check(w.Format("Halo 3", "hornet driver", line, sizeof(line)) > 0 && std::strstr(line, "mean 1000 mm") &&
              std::strstr(line, "SHOT UPSTREAM Halo 3 seat hornet driver"),
          "the dev line reads the stock line's miss");
    seat_upstream::StockLineWindow empty;
    Check(empty.Format("Halo 3", "x", line, sizeof(line)) == 0, "an empty window prints nothing");
}
} // namespace

int main()
{
    TestModelAndInverse();
    TestGainFit();
    TestWriteRing();
    TestRecomposeSeat();
    TestStockLineWindow();
    std::printf("%u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
