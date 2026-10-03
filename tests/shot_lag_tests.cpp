// Shot lag (src/common/shot_lag_logic.h): the intent published as a world point and recomposed against each
// shot's own origin, the fail-closed rules, the eye's track and extrapolation, the frame-coherent pair, and the
// dev witness's geometry.
#include "shot_lag_logic.h"

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
bool Near(float a, float b, float tolerance = 1.0e-4f) { return std::fabs(a - b) <= tolerance; }
constexpr float kWuPerM = shot_lag::kWuPerMetre;

// Distance (wu) from `point` to the line (origin, direction).
float LineMiss(const float origin[3], const float direction[3], const float point[3])
{
    const float l = shot_lag::Length(direction);
    const float d[3]{direction[0] / l, direction[1] / l, direction[2] / l};
    const float w[3]{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    const float t = shot_lag::Dot(w, d);
    const float m[3]{w[0] - d[0] * t, w[1] - d[1] * t, w[2] - d[2] * t};
    return shot_lag::Length(m);
}
} // namespace

int main()
{
    using namespace shot_lag;

    // ---- re-aim through the point ----
    {
        const float start[3]{0.0f, 0.0f, 0.0f}, vector[3]{40.0f, 0.0f, 0.0f}, point[3]{5.0f, 0.5f, -0.25f};
        float out[3]{};
        float bend = -1.0f;
        const Reaim r = ReaimThroughPoint(start, vector, point, ReaimRails{}, out, &bend);
        Check(r == Reaim::Applied, "a point ahead and near the line is applied");
        Check(Near(Length(out), 40.0f, 1.0e-3f), "the query keeps its length");
        Check(LineMiss(start, out, point) < 1.0e-5f, "the new query passes through the point");
        Check(bend > 0.0f && bend < 7.0f, "the bend is reported");
    }
    {
        // THE LAG ITSELF: angles solved from the render-time eye E0 toward P, fired from the tick's eye E1 = E0 + v dt.
        // The angles leave a parallel line off by v dt across the shot; re-aiming from E1 through P hits P exactly.
        const float e0[3]{0.0f, 0.0f, 0.0f}, p[3]{10.0f * kWuPerM, 0.0f, 0.0f};
        const float v[3]{0.0f, 4.0f * kWuPerM, 0.0f};   // a 4 m/s strafe, across the shot
        const float dt = 0.035f;
        const float e1[3]{e0[0] + v[0] * dt, e0[1] + v[1] * dt, e0[2] + v[2] * dt};
        const float solved[3]{p[0] - e0[0], p[1] - e0[1], p[2] - e0[2]};
        Check(Near(LineMiss(e1, solved, p) * kMetresPerWu * 1000.0f, 140.0f, 0.5f),
              "the stale angles miss by v*dt = 140 mm at a 4 m/s strafe over 35 ms");
        float out[3]{};
        Check(ReaimThroughPoint(e1, solved, p, ReaimRails{}, out) == Reaim::Applied &&
                  LineMiss(e1, out, p) * kMetresPerWu * 1000.0f < 0.01f,
              "the re-aim from the shot's own origin through P misses by nothing");
    }
    {
        const float start[3]{0, 0, 0}, vector[3]{10, 0, 0};
        float out[3]{7, 7, 7};
        const float behind[3]{-3.0f, 0.1f, 0.0f};
        Check(ReaimThroughPoint(start, vector, behind, ReaimRails{}, out) == Reaim::PointBehind,
              "a point behind the start is refused");
        Check(out[0] == 7 && out[1] == 7 && out[2] == 7, "a refusal leaves the output untouched");
        const float beside[3]{0.0f, 1.0f, 0.0f};
        Check(ReaimThroughPoint(start, vector, beside, ReaimRails{}, out) == Reaim::PointBehind,
              "a point exactly beside the start is not ahead");
        const float close[3]{0.05f, 0.0f, 0.0f};   // 15 cm
        Check(ReaimThroughPoint(start, vector, close, ReaimRails{}, out) == Reaim::PointTooClose,
              "a point within 30 cm defines no direction");
        const float far[3]{5.0f, 2.0f, 0.0f};      // 6 m off the line
        Check(ReaimThroughPoint(start, vector, far, ReaimRails{}, out) == Reaim::BendTooLarge,
              "a point metres off the line is not this lag: refused");
        const float steep[3]{1.0f, 0.8f, 0.0f};    // 0.8 wu = 2.4 m off at 38.7 deg
        Check(ReaimThroughPoint(start, vector, steep, ReaimRails{}, out) == Reaim::BendTooLarge,
              "a bend over 30 deg is refused even within the offset rail");
        const float onLine[3]{5.0f, 0.002f, 0.0f};   // 6 mm off the line
        Check(ReaimThroughPoint(start, vector, onLine, ReaimRails{}, out) == Reaim::AlreadyThrough,
              "a query already through P (within 10 mm) is left exactly as the game built it");
        const float justOff[3]{5.0f, 0.004f, 0.0f};  // 12 mm off the line
        float moved[3]{};
        Check(ReaimThroughPoint(start, vector, justOff, ReaimRails{}, moved) == Reaim::Applied,
              "12 mm off is corrected");
        const float zero[3]{0, 0, 0};
        const float p[3]{5, 0, 0};
        Check(ReaimThroughPoint(start, zero, p, ReaimRails{}, out) == Reaim::Degenerate, "a zero query is refused");
        const float nan3[3]{std::nanf(""), 0, 0};
        Check(ReaimThroughPoint(start, vector, nan3, ReaimRails{}, out) == Reaim::NonFinite &&
                  ReaimThroughPoint(nan3, vector, p, ReaimRails{}, out) == Reaim::NonFinite,
              "non-finite input is refused");
        Check(out[0] == 7 && out[1] == 7 && out[2] == 7, "no refusal wrote the output");
    }

    {
        // FAR POINTS: "already through" must hold at range. distance^2 - ahead^2 in float cancels to noise beyond
        // ~20 m (48 mm at 100 m, ~0.5 m at 1 km), so a query exactly through P read as off it and was bent. Oblique
        // directions from a start at map-sized coordinates, a few of them so no lucky rounding hides the noise.
        const float start[3]{123.456f, -78.9f, 12.3f};
        const float dirs[4][3]{{0.6f, 0.7f, 0.3f}, {-0.31f, 0.88f, -0.12f}, {0.577f, -0.577f, 0.577f},
                               {0.99f, 0.05f, -0.13f}};
        const float ranges[3]{100.0f, 300.0f, 1000.0f};
        for (const float metres : ranges)
        {
            const float rangeWu = metres * kWuPerM;
            bool through = true, near = true, applied = true, hits = true;
            for (const auto& raw : dirs)
            {
                const float l = Length(raw);
                const float d[3]{raw[0] / l, raw[1] / l, raw[2] / l};
                // A unit vector across the line.
                float e[3]{d[1], -d[0], 0.0f};
                const float el = Length(e);
                e[0] /= el; e[1] /= el;
                const float vector[3]{d[0] * 40.0f, d[1] * 40.0f, d[2] * 40.0f};   // the query's own length
                const auto pointOff = [&](float offMm, float out[3]) {
                    const float off = offMm / kMmPerWu;
                    for (int axis = 0; axis < 3; ++axis)
                        out[axis] = start[axis] + d[axis] * rangeWu + e[axis] * off;
                };
                float p[3]{}, out[3]{};
                pointOff(0.0f, p);
                through = through && ReaimThroughPoint(start, vector, p, ReaimRails{}, out) == Reaim::AlreadyThrough;
                pointOff(5.0f, p);
                near = near && ReaimThroughPoint(start, vector, p, ReaimRails{}, out) == Reaim::AlreadyThrough;
                pointOff(15.0f, p);
                float bend = -1.0f;
                const bool ok = ReaimThroughPoint(start, vector, p, ReaimRails{}, out, &bend) == Reaim::Applied;
                applied = applied && ok;
                hits = hits && ok && LineMiss(start, out, p) * kMmPerWu < 2.0f &&
                       Near(bend, std::atan2(15.0f, metres * 1000.0f) * kRadToDeg, 2.0e-4f);
            }
            char what[160];
            std::snprintf(what, sizeof(what), "at %.0f m a query exactly through P is already through", double(metres));
            Check(through, what);
            std::snprintf(what, sizeof(what), "at %.0f m a query 5 mm off P is already through", double(metres));
            Check(near, what);
            std::snprintf(what, sizeof(what), "at %.0f m a query 15 mm off P is re-aimed", double(metres));
            Check(applied, what);
            std::snprintf(what, sizeof(what), "at %.0f m the re-aimed query passes P and reports its true bend",
                          double(metres));
            Check(hits, what);
        }
    }

    // ---- the published intent: when the shot site may use it ----
    {
        IntentPoint p{};
        p.generation = 3;
        p.stampUs = 1000000;
        p.rangeWu = 5.0f;
        p.point[0] = 5.0f;
        p.dir[0] = 1.0f;
        Check(ShouldUseIntent(true, p, 3, 1050000, 200000, true, false) == IntentUse::Use, "a fresh point is used");
        Check(ShouldUseIntent(false, p, 3, 1050000, 200000, true, false) == IntentUse::Missing,
              "nothing published");
        Check(ShouldUseIntent(true, p, 4, 1050000, 200000, true, false) == IntentUse::Missing,
              "another generation's point");
        Check(ShouldUseIntent(true, p, 3, 1300001, 200000, true, false) == IntentUse::Stale, "a stale point");
        Check(ShouldUseIntent(true, p, 3, 999999, 200000, true, false) == IntentUse::Stale, "a point from the future");
        Check(ShouldUseIntent(true, p, 3, 1050000, 200000, false, false) == IntentUse::NotConverged,
              "not converged");
        Check(ShouldUseIntent(true, p, 3, 1050000, 200000, true, true) == IntentUse::NaturalHeld,
              "a natural hold (seat stop, turret slew) leaves the query alone");
        IntentPoint held = p;
        held.railHeld = true;
        Check(ShouldUseIntent(true, held, 3, 1050000, 200000, true, false) == IntentUse::RailHeld,
              "a rail-held solve does not pass through P");
        IntentPoint bad = p;
        bad.point[1] = std::nanf("");
        Check(ShouldUseIntent(true, bad, 3, 1050000, 200000, true, false) == IntentUse::Bad, "a non-finite point");
        bad = p;
        bad.rangeWu = 0.0f;
        Check(ShouldUseIntent(true, bad, 3, 1050000, 200000, true, false) == IntentUse::Bad, "no range");
    }

    // ---- the crosshair query's gate ----
    {
        Check(GateCrosshairQuery(true, kFireLocal, true, false) == QueryGate::Try,
              "the local player's query at the site, in his fire, is tried");
        Check(GateCrosshairQuery(false, kFireLocal, true, false) == QueryGate::NotOurs, "a query elsewhere is never ours");
        Check(GateCrosshairQuery(true, kFireLocal, false, false) == QueryGate::NotOurs,
              "another unit's query is never ours");
        Check(GateCrosshairQuery(true, kFireLocal, true, true) == QueryGate::OwnShotLine,
              "an off-hand or barrel shot's own line is left alone (never the aim hand's point)");
        Check(GateCrosshairQuery(true, kFireLocal, false, true) == QueryGate::NotOurs,
              "another unit's query is not counted as an own-line skip");
        Check(std::strcmp(IntentUseName(IntentUse::OwnShotLine), "own shot line") == 0, "the own-line skip is named");
        // THE LOCAL PLAYER'S FIRE: a query that passes every check of its own is still left alone in another
        // player's fire (co-op, the next seat), or outside any fire we opened.
        Check(GateCrosshairQuery(true, kFireOther, true, false) == QueryGate::NotOurs,
              "a co-op player's fire is never re-aimed, whatever its query looks like");
        Check(GateCrosshairQuery(true, kFireNone, true, false) == QueryGate::NotOurs,
              "a query outside any native fire is never re-aimed");
        Check(GateCrosshairQuery(true, kFireOther, true, true) == QueryGate::NotOurs,
              "another player's barrel shot is not counted as ours");
    }
    {
        // The tag's lifetime: a fire opens Other; its shot line names the shooter; outside a fire nothing marks.
        Check(FireMark(kFireNone, true) == kFireNone, "a shot line outside a fire never tags (no fire was opened)");
        Check(FireMark(kFireOther, true) == kFireLocal, "the local unit's shot line tags the fire local");
        Check(FireMark(kFireOther, false) == kFireOther, "another unit's shot line leaves it another's");
        Check(FireMark(kFireLocal, false) == kFireOther, "a later projectile by another shooter is not ours");
        Check(FireMark(kFireLocal, true) == kFireLocal, "a second projectile of our own stays ours");
    }
    {
        // The query's own evidence (on foot: the unit; seated: the vehicle, and a start at our eye).
        IntentPoint intent{};
        intent.eye[0] = 10.0f;
        intent.eye[1] = 20.0f;
        intent.eye[2] = 1.0f;
        const float atEye[3]{10.1f, 20.0f, 1.0f};
        const float beside[3]{10.0f, 20.0f + 2.0f * kWuPerM, 1.0f};   // 2 m away: the next seat's camera
        const float maxWu = 1.5f * kWuPerM;
        const uint64_t now = 5000000;
        Check(QueryIsLocal(false, 42, 42, -1, -1, intent, atEye, now, maxWu), "on foot: our unit is ours");
        Check(!QueryIsLocal(false, 43, 42, -1, -1, intent, atEye, now, maxWu), "on foot: another unit is not");
        Check(!QueryIsLocal(false, -1, -1, -1, -1, intent, atEye, now, maxWu), "no unit on either side is nobody's");
        Check(QueryIsLocal(true, 900, -1, 800, 900, intent, atEye, now, maxWu), "seated: the vehicle's root");
        Check(QueryIsLocal(true, 800, -1, 800, 900, intent, atEye, now, maxWu), "seated: the seat's parent");
        Check(QueryIsLocal(true, 800, -1, 800, -1, intent, atEye, now, maxWu), "seated, one level: the parent");
        Check(!QueryIsLocal(true, 700, -1, 800, 900, intent, atEye, now, maxWu), "seated: another vehicle is not");
        Check(!QueryIsLocal(true, 900, -1, 800, 900, intent, beside, now, maxWu),
              "seated: a query starting 2 m from our eye is not ours");
        // The eye carried forward: a hull at 20 m/s, 50 ms after the eye was sampled, is a metre on.
        IntentPoint moving = intent;
        moving.eyeVel[1] = 20.0f * kWuPerM;
        moving.eyeVelValid = true;
        moving.eyeStampUs = now - 50000;
        const float ahead[3]{10.0f, 20.0f + 1.0f * kWuPerM, 1.0f};
        Check(QueryIsLocal(true, 900, -1, 800, 900, moving, ahead, now, 0.2f * kWuPerM),
              "seated: our eye is carried forward by its velocity to now");
        Check(!QueryIsLocal(true, 900, -1, 800, 900, intent, ahead, now, 0.2f * kWuPerM),
              "...without a velocity the same start is a metre off");
        const float nan3[3]{std::nanf(""), 0.0f, 0.0f};
        Check(!QueryIsLocal(true, 900, -1, 800, 900, intent, nan3, now, maxWu), "a non-finite start is nobody's");
    }

    // ---- the eye's track ----
    {
        EyeTrack track;
        const float v[3]{2.0f * kWuPerM, -1.0f * kWuPerM, 0.0f};
        for (int i = 0; i <= 10; ++i)
        {
            const float t = float(i) * 0.011f;
            const float e[3]{1.0f + v[0] * t, 2.0f + v[1] * t, 3.0f};
            track.Push(1000000 + uint64_t(i) * 11000, e);
        }
        float out[3]{};
        Check(track.Velocity(100000, 30000, out) && Near(out[0], v[0], 1.0e-3f) && Near(out[1], v[1], 1.0e-3f) &&
                  Near(out[2], 0.0f, 1.0e-3f),
              "a constant pace reads back");
        float none[3]{};
        EyeTrack empty;
        Check(!empty.Velocity(100000, 30000, none), "no track, no velocity");
        EyeTrack two;
        const float a[3]{0, 0, 0}, b[3]{0.001f, 0, 0};
        two.Push(1000, a);
        two.Push(11000, b);
        Check(!two.Velocity(100000, 30000, none), "a 10 ms span is too short to trust");
    }
    {
        // A player who stops: the eye repeats; repeats inside 40 ms are not news, after it they read as standing.
        EyeTrack track;
        const float moving[3]{0.0f, 0.0f, 0.0f}, moved[3]{0.1f, 0.0f, 0.0f};
        track.Push(1000000, moving);
        track.Push(1050000, moved);
        track.Push(1060000, moved);   // a repeat 10 ms later: ignored
        Check(track.count == 2, "a repeat of the last render's eye inside 40 ms is no sample");
        track.Push(1100000, moved);   // 50 ms of standing still
        track.Push(1150000, moved);
        track.Push(1200000, moved);
        float out[3]{};
        Check(track.Velocity(100000, 30000, out) && Near(out[0], 0.0f, 1.0e-6f),
              "after 100 ms standing still the eye reads 0, not the last pace");
    }
    {
        EyeTrack track;
        const float a[3]{0, 0, 0}, b[3]{0.01f, 0, 0}, c[3]{500.0f, 0, 0}, d[3]{500.01f, 0, 0};
        track.Push(1000000, a);
        track.Push(1011000, b);
        track.Push(1022000, c);   // 500 wu in 11 ms: a teleport
        Check(track.count == 1, "a jump faster than any vehicle starts afresh");
        track.Push(1400000, d);   // a 378 ms gap
        Check(track.count == 1, "a gap starts afresh");
        track.Push(1300000, a);   // the clock ran backwards
        Check(track.count == 1, "a clock running backwards starts afresh");
        const float nan3[3]{std::nanf(""), 0, 0};
        track.Push(1310000, nan3);
        Check(track.count == 1, "a non-finite eye is never a sample");
    }

    // ---- the game-eye proof ----
    {
        using S = EyeProof::State;
        const auto feed = [](EyeProof& p, int n, uint32_t gen, int32_t unit, bool eligible, bool rest, float mm) {
            for (int i = 0; i < n; ++i)
                p.Note(gen, unit, eligible, rest, mm);
        };
        EyeProof p;
        feed(p, int(EyeProof::kAgreeToProve) - 1, 1, 7, true, true, 10.0f);
        Check(p.state == S::Unproven, "29 agreeing samples do not prove it");
        p.Note(1, 7, true, true, 10.0f);
        Check(p.state == S::Proven, "30 agreeing at-rest samples on foot prove it");

        // THE FINDING: seated / third person / turret / death camera samples are no evidence at all.
        EyeProof seated;
        feed(seated, 200, 1, 7, false, true, 2000.0f);
        Check(seated.state == S::Unproven && seated.disagree == 0, "ineligible samples never disprove");
        feed(p, 200, 1, 7, false, true, 2000.0f);
        Check(p.state == S::Proven, "nor do they unprove a proven eye");

        // A single at-rest outlier (a respawn frame, a teleport) does not latch a disproof.
        EyeProof q;
        q.Note(1, 7, true, true, 900.0f);
        Check(q.state == S::Unproven, "one disagreeing sample proves nothing");
        feed(q, int(EyeProof::kDisagreeToDisprove) - 2, 1, 7, true, true, 900.0f);
        q.Note(1, 7, true, true, 100.0f);   // a sample between the bands breaks the streak
        feed(q, int(EyeProof::kDisagreeToDisprove) - 1, 1, 7, true, true, 900.0f);
        Check(q.state == S::Unproven, "a broken streak does not disprove");
        q.Note(1, 7, true, false, 900.0f);  // moving: not judged, the streak stands
        q.Note(1, 7, true, true, 900.0f);
        Check(q.state == S::Disproven, "15 consecutive at-rest disagreeing samples disprove it");
        feed(q, 40, 1, 7, true, true, 10.0f);
        Check(q.state == S::Disproven, "a disproof holds for the stint");

        // ...and only for the stint: a seat in between, or a new unit, re-opens it.
        q.Note(1, 7, false, true, 0.0f);
        q.Note(1, 7, true, true, 10.0f);
        Check(q.state == S::Unproven, "the next on-foot stint re-opens the proof");
        feed(q, int(EyeProof::kAgreeToProve), 1, 7, true, true, 10.0f);
        Check(q.state == S::Proven, "and it can be proven again");
        feed(q, int(EyeProof::kDisagreeToDisprove), 1, 7, true, true, 900.0f);
        Check(q.state == S::Disproven, "a proven eye can still be disproven by a streak");
        q.Note(1, 8, true, true, 10.0f);
        Check(q.state == S::Unproven && q.agree == 1, "a new unit (respawn) re-opens the proof");
        q.Note(2, 8, true, true, 10.0f);
        Check(q.state == S::Unproven && q.agree == 1 && q.generation == 2, "a new generation starts over");
        EyeProof nanProof;
        feed(nanProof, 40, 1, 7, true, true, std::nanf(""));
        Check(nanProof.state == S::Unproven, "a non-finite offset is no evidence");
    }

    // ---- extrapolation, angles, identity ----
    {
        const float e[3]{1, 2, 3}, v[3]{10, 0, -5};
        float out[3]{};
        Check(ExtrapolateEye(e, v, 0.02f, 0.1f, out) && Near(out[0], 1.2f) && Near(out[2], 2.9f), "E + v dt");
        Check(ExtrapolateEye(e, v, 5.0f, 0.1f, out) && Near(out[0], 2.0f), "dt clamps at its maximum");
        Check(ExtrapolateEye(e, v, -1.0f, 0.1f, out) && Near(out[0], 1.0f), "a negative dt is 0");
        const float nan3[3]{std::nanf(""), 0, 0};
        Check(!ExtrapolateEye(e, nan3, 0.02f, 0.1f, out), "no velocity, no extrapolation");
    }
    {
        const float o[3]{0, 0, 0}, p[3]{1, 1, std::sqrt(2.0f)};
        float yaw = 9.0f, pitch = 9.0f;
        Check(AnglesTowardPoint(o, p, 0.01f, 1.45f, yaw, pitch) && Near(yaw, 0.785398f) && Near(pitch, 0.785398f),
              "the angles of P - O in the game basis");
        const float up[3]{0.0f, 0.0f, 5.0f};
        Check(AnglesTowardPoint(o, up, 0.01f, 1.45f, yaw, pitch) && Near(pitch, 1.45f), "pitch clamps at the rail");
        const float back[3]{-1.0f, -0.0f, 0.0f};
        Check(AnglesTowardPoint(o, back, 0.01f, 1.45f, yaw, pitch) && Near(std::fabs(yaw), 3.141593f),
              "yaw is atan2's");
        const float near0[3]{0.001f, 0, 0};
        yaw = 9.0f;
        Check(!AnglesTowardPoint(o, near0, 0.01f, 1.45f, yaw, pitch) && yaw == 9.0f, "too close: refused, untouched");
        Check(SameAngles(1.0f, 0.1f, 1.0f + 6.2831853f, 0.1f, 1.0e-4f), "identity is wrap-insensitive (a full turn)");
        Check(SameAngles(3.14155f, 0.0f, -3.14155f, 0.0f, 1.0e-3f), "and across the +-pi seam");
        Check(!SameAngles(0.0f, 0.0f, 0.001f, 0.0f, 1.0e-4f), "a different solve is not the same");
        Check(!SameAngles(std::nanf(""), 0.0f, 0.0f, 0.0f), "non-finite angles never match");
    }

    // ---- the frame-coherent pair ----
    {
        EyeFrame eyes[3]{};
        HandFrame hands[3]{};
        const float k1[3]{0.1f, 1.6f, 0.2f}, k2[3]{0.11f, 1.6f, 0.2f}, k3[3]{0.12f, 1.6f, 0.2f};
        eyes[0] = {true, {k1[0], k1[1], k1[2]}, {}, {}, 1000, 1};
        eyes[1] = {true, {k2[0], k2[1], k2[2]}, {}, {}, 12000, 2};
        hands[0] = {true, {k1[0], k1[1], k1[2]}, {}, 2000, 1};
        hands[1] = {true, {k2[0], k2[1], k2[2]}, {}, 13000, 2};
        hands[2] = {true, {k3[0], k3[1], k3[2]}, {}, 24000, 3};   // its frame has not rendered yet
        size_t e = 9, h = 9;
        Check(SelectCoherentPair(eyes, 3, hands, 3, 100000, e, h) && e == 1 && h == 1,
              "the newest hand whose frame has rendered pairs with that render");
        Check(!SelectCoherentPair(eyes, 3, hands + 2, 1, 100000, e, h), "no rendered frame for the hand: no pair");
        Check(!SelectCoherentPair(eyes, 3, hands, 3, 500, e, h), "a key matched across a long skew is refused");
        // A frozen head: the same key on two renders -- the newest render.
        eyes[2] = {true, {k2[0], k2[1], k2[2]}, {}, {}, 13500, 2};
        Check(SelectCoherentPair(eyes, 3, hands, 3, 100000, e, h) && e == 2 && h == 1,
              "several renders under one head sample: the newest");
        EyeFrame invalid[1]{};
        invalid[0] = eyes[0];
        invalid[0].valid = false;
        Check(!SelectCoherentPair(invalid, 1, hands, 1, 100000, e, h), "an invalid slot never pairs");
        // A RENDER STALL: the newest hand whose frame rendered is old by now. Bounded by the hand's age at `now`.
        Check(SelectCoherentPair(eyes, 3, hands, 3, 100000, e, h, 13000 + 40000, 50000) && h == 1,
              "a hand 40 ms old pairs");
        Check(!SelectCoherentPair(eyes, 3, hands, 3, 100000, e, h, 13000 + 60000, 50000),
              "a hand 60 ms old is the aim of that moment, not of now: no pair (the poll's own pair stands)");
        Check(SelectCoherentPair(eyes, 3, hands, 3, 100000, e, h, 2000 + 40000, 50000) && h == 1,
              "...while it is the newest within the bound, the newest still wins");
        Check(!SelectCoherentPair(eyes, 3, hands, 1, 100000, e, h, 1000, 50000),
              "a hand stamped after `now` is never paired");
        Check(SelectCoherentPair(eyes, 3, hands, 3, 100000, e, h, 13000 + 60000, 0) && h == 1,
              "a zero bound is unbounded (the old behaviour)");
        HandFrame staleNewest[2]{hands[0], hands[1]};
        staleNewest[0].stampUs = 30000;   // hand k1 re-stamped newer than hand k2: only it is within the bound
        Check(SelectCoherentPair(eyes, 2, staleNewest, 2, 100000, e, h, 13000 + 60000, 50000) && h == 0,
              "the bound picks the newest hand within it, never an older one past it");
    }

    // ---- the witness ----
    {
        // Strafing right (+y) at 4 m/s while firing along +x at a point 10 m out, from an eye 140 mm further along
        // the strafe than the solve's: a parallel line ahead of P along the travel by 140 mm, nothing across.
        const float p[3]{10.0f * kWuPerM, 0.0f, 0.0f};
        const float e0[3]{0.0f, 0.0f, 0.0f};
        const float o[3]{0.0f, 0.14f * kWuPerM, 0.0f};
        const float d[3]{1.0f, 0.0f, 0.0f};
        const float v[3]{0.0f, 4.0f * kWuPerM, 0.0f};
        const ShotLagSample s = MeasureShotLag(o, d, p, e0, v);
        Check(s.valid && Near(s.missMm, 140.0f, 0.5f), "the miss is the parallel offset");
        Check(Near(s.alongMm, 140.0f, 0.5f) && Near(s.acrossMm, 0.0f, 0.5f), "...all of it along the travel, ahead");
        Check(s.stale && Near(s.staleAlongMm, 140.0f, 0.5f) && Near(s.impliedDtMs, 35.0f, 0.2f),
              "the stale eye is 140 mm along the travel: 35 ms at 4 m/s");
        Check(s.bucket == 2 && Near(s.speedMps, 4.0f, 1.0e-3f), "4 m/s is the 4-10 bucket");
        const ShotLagSample back = MeasureShotLag(o, d, p, e0, nullptr);
        Check(back.valid && !back.velocityKnown && back.bucket == -1 && Near(back.acrossMm, 140.0f, 0.5f) &&
                  !back.stale,
              "without a velocity the miss is all 'across' and nothing is signed");
        // The fixed shot: from the moved origin through P.
        const float fixed[3]{p[0] - o[0], p[1] - o[1], p[2] - o[2]};
        const ShotLagSample good = MeasureShotLag(o, fixed, p, e0, v);
        Check(good.valid && good.missMm < 0.01f && good.stale && Near(good.impliedDtMs, 35.0f, 0.2f),
              "a shot through P misses by nothing, whatever the stale eye says");
        // Moving along the shot: no travel axis across it.
        const float vAlong[3]{5.0f * kWuPerM, 0.0f, 0.0f};
        const ShotLagSample ahead = MeasureShotLag(o, d, p, e0, vAlong);
        Check(ahead.valid && !ahead.travel && !ahead.stale && ahead.bucket == 2,
              "running straight at the point has no across-shot travel");
        Check(ShotLagBucket(0.5f) == 0 && ShotLagBucket(1.0f) == 1 && ShotLagBucket(3.99f) == 1 &&
                  ShotLagBucket(4.0f) == 2 && ShotLagBucket(10.0f) == 3 && ShotLagBucket(-1.0f) == -1,
              "the speed buckets");
        const float zero[3]{0, 0, 0};
        Check(!MeasureShotLag(o, zero, p, e0, v).valid, "a zero shot direction is unmeasured");

        ShotLagWindow window;
        window.Add(s, false, -1);
        window.Add(good, false, -1);
        window.Add(back, true, 7);
        window.Add(ShotLagSample{}, true, 7);
        char line[1024]{};
        const size_t n = window.Format("Halo 3", line, sizeof(line));
        Check(n > 0 && std::strstr(line, "SHOT LAG Halo 3 (dev): 3 shots (+1 unmeasured") != nullptr,
              "the line counts measured and unmeasured shots");
        Check(std::strstr(line, "4-10: 2, miss 70 mm (along +70, across 0)") != nullptr,
              "per-bucket mean miss, along and across");
        Check(std::strstr(line, "speed unknown: 1, miss 140 mm") != nullptr, "shots without a velocity apart");
        Check(std::strstr(line, "mean lag axis along travel +70 mm") != nullptr, "the mean along-travel bias");
        Check(std::strstr(line, "stale eye along travel +140 mm (implied dt 35 ms)") != nullptr,
              "the stale-eye term and its dt");
        Check(std::strstr(line, "seated=1 seat=7") != nullptr, "the last shot's seat");
        ShotLagWindow emptyWindow;
        Check(emptyWindow.Format("Reach", line, sizeof(line)) == 0, "an empty window logs nothing");
        char tiny[16]{};
        Check(window.Format("Halo 3", tiny, sizeof(tiny)) < sizeof(tiny) && tiny[sizeof(tiny) - 1] == 0,
              "a short buffer truncates safely");
    }

    std::printf("shot_lag_tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
