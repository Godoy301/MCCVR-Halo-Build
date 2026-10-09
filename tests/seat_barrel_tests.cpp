// Aiming a seat's gun through the point (src/common/seat_barrel_logic.h): the barrel's geometry learned from the
// lines of the seat's own fires, the aim for which the barrel's line runs through a point, and the push that
// takes up a barrel settling a little off the aim it is given. The fixture is a
// synthetic vehicle: a seat's eye fixed in the hull, a turret pivot elsewhere in the hull, a barrel beside and
// above its pivot, turning with the aim.
#include "seat_barrel_logic.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

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

using namespace seat_barrel;

struct Vehicle
{
    Frame hull{};
    float eye[3]{};
    double pivot[3]{};         // from the eye, in the hull's frame (forward, left, up)
    double beside = 0, above = 0;   // the barrel from its pivot, in the aim's frame (right, up)

    // The barrel's line when the gun points along `d` (unit, world): a point of it and the direction.
    void Barrel(const double d[3], float origin[3], float direction[3], double along = 1.7) const
    {
        double right[3]{}, up[3]{};
        AimAxes(hull, d, right, up);
        for (int i = 0; i < 3; ++i)
        {
            origin[i] = static_cast<float>(eye[i] + hull.f[i] * pivot[0] + hull.l[i] * pivot[1] +
                                           hull.u[i] * pivot[2] + right[i] * beside + up[i] * above + d[i] * along);
            direction[i] = static_cast<float>(d[i]);
        }
    }
    // The gun fires along `d`.
    bool Fire(Model& model, const double d[3]) const
    {
        float origin[3]{}, direction[3]{};
        Barrel(d, origin, direction);
        return model.Learn(hull, eye, origin, direction);
    }
    // How far the barrel's line passes from `point` when the gun follows the aim the model solves for it; -1 when
    // the model declines.
    double MissAt(const Model& model, const float point[3]) const
    {
        float aim[3]{};
        if (!model.Aim(hull, eye, point, aim))
            return -1.0;
        const double d[3]{aim[0], aim[1], aim[2]};
        float origin[3]{}, direction[3]{};
        Barrel(d, origin, direction);
        const double o[3]{origin[0], origin[1], origin[2]}, p[3]{point[0], point[1], point[2]};
        return LineMiss(o, d, p);
    }
    // The same for the eye's line (what a seat aimed from its camera does): the barrel parallel to it.
    double EyeLineMissAt(const float point[3]) const
    {
        double d[3]{double(point[0]) - eye[0], double(point[1]) - eye[1], double(point[2]) - eye[2]};
        Normalize(d);
        float origin[3]{}, direction[3]{};
        Barrel(d, origin, direction);
        const double o[3]{origin[0], origin[1], origin[2]}, p[3]{point[0], point[1], point[2]};
        return LineMiss(o, d, p);
    }
};

void Heading(const Frame& hull, double yawDeg, double pitchDeg, double d[3])
{
    const double yaw = yawDeg * 0.017453292519943295, pitch = pitchDeg * 0.017453292519943295;
    const double f = std::cos(pitch) * std::cos(yaw), l = std::cos(pitch) * std::sin(yaw), u = std::sin(pitch);
    for (int i = 0; i < 3; ++i)
        d[i] = hull.f[i] * f + hull.l[i] * l + hull.u[i] * u;
}

void PointAlong(const Vehicle& v, double yawDeg, double pitchDeg, double range, float point[3])
{
    double d[3]{};
    Heading(v.hull, yawDeg, pitchDeg, d);
    for (int i = 0; i < 3; ++i)
        point[i] = static_cast<float>(v.eye[i] + d[i] * range);
}

Vehicle Tank()
{
    Vehicle v{};
    // A hull pitched and rolled on a slope, heading somewhere that is no axis.
    const float forward[3]{0.80f, 0.58f, 0.15f}, up[3]{-0.10f, -0.12f, 0.98f};
    MakeFrame(forward, up, v.hull);
    v.eye[0] = 31.5f;
    v.eye[1] = -12.25f;
    v.eye[2] = 4.0f;
    v.pivot[0] = -0.90;   // the turret's pivot 0.9 wu behind the driver's eye, a little left and above
    v.pivot[1] = 0.12;
    v.pivot[2] = 0.35;
    v.beside = 0.05;
    v.above = 0.02;
    return v;
}

void TestFrame()
{
    Frame h{};
    const float forward[3]{2.0f, 0.0f, 0.2f}, up[3]{0.0f, 0.0f, 3.0f};
    Check(MakeFrame(forward, up, h), "a frame from a forward and an up that are neither unit nor square");
    Check(std::fabs(Dot(h.f, h.l)) < 1e-12 && std::fabs(Dot(h.f, h.u)) < 1e-12 && std::fabs(Dot(h.l, h.u)) < 1e-12,
          "its axes are orthogonal");
    Check(std::fabs(Dot(h.f, h.f) - 1.0) < 1e-12 && std::fabs(Dot(h.u, h.u) - 1.0) < 1e-12, "and unit");
    Check(h.l[1] > 0.99, "left is up x forward");
    const float bad[3]{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}, zero[3]{};
    Check(!MakeFrame(bad, up, h) && !MakeFrame(forward, zero, h) && !MakeFrame(forward, forward, h),
          "not a number, no up, or forward along up: no frame");
    double right[3]{}, upAxis[3]{};
    Check(!AimAxes(h, h.u, right, upAxis), "an aim along the vehicle's up has no right");
}

void TestNoFire()
{
    const Vehicle v = Tank();
    Model model;
    float point[3]{};
    PointAlong(v, 10.0, 2.0, 20.0, point);
    float aim[3]{9.0f, 9.0f, 9.0f};
    Check(model.fires() == 0 && !model.Aim(v.hull, v.eye, point, aim) && aim[0] == 9.0f,
          "before the first fire the model declines and leaves the aim alone");
    Check(v.EyeLineMissAt(point) > 0.2, "fixture: the eye's line leaves this barrel well off the point");
}

void TestOneFire()
{
    const Vehicle v = Tank();
    Model model;
    double d[3]{};
    Heading(v.hull, 25.0, 3.0, d);
    Check(v.Fire(model, d) && model.fires() == 1, "one fire is learned");
    // A point the fired barrel pointed at, at any range: exact from the first fire (the gun turns back to that
    // heading), where the eye's line leaves the barrel beside it.
    float fired[3]{}, firedDirection[3]{};
    v.Barrel(d, fired, firedDirection);
    for (double range : {8.0, 20.0, 60.0})
    {
        const float point[3]{fired[0] + float(d[0] * range), fired[1] + float(d[1] * range),
                             fired[2] + float(d[2] * range)};
        const double miss = v.MissAt(model, point);
        Check(miss >= 0.0 && miss < 0.01, "after one fire the barrel's line runs through a point on the fired line");
        Check(v.EyeLineMissAt(point) > 30.0 * (miss + 1e-4), "and the eye's line would have missed it by far more");
    }
    // A heading 20 degrees away: better than the eye's line, not yet exact.
    float point[3]{};
    PointAlong(v, 45.0, 3.0, 20.0, point);
    const double miss = v.MissAt(model, point);
    Check(miss >= 0.0 && miss < v.EyeLineMissAt(point), "another heading already improves on the eye's line");
}

void TestManyFires()
{
    const Vehicle v = Tank();
    Model model;
    const double yaws[]{-70.0, -30.0, 0.0, 20.0, 55.0, 90.0}, pitches[]{-4.0, 6.0, 1.0, 9.0, -2.0, 4.0};
    for (int i = 0; i < 6; ++i)
    {
        double d[3]{};
        Heading(v.hull, yaws[i], pitches[i], d);
        Check(v.Fire(model, d), "a fire at another heading is learned");
    }
    // Headings that were never fired on, both sides, near and far.
    double worst = 0.0, worstEye = 0.0;
    for (double yaw : {-85.0, -50.0, -10.0, 35.0, 75.0, 110.0})
        for (double pitch : {-5.0, 2.0, 8.0})
            for (double range : {6.0, 25.0, 90.0})
            {
                float point[3]{};
                PointAlong(v, yaw, pitch, range, point);
                const double miss = v.MissAt(model, point);
                Check(miss >= 0.0, "the model answers for a heading it never fired on");
                if (miss > worst)
                    worst = miss;
                const double eyeMiss = v.EyeLineMissAt(point);
                if (eyeMiss > worstEye)
                    worstEye = eyeMiss;
            }
    Check(worst < 0.02, "after fires around the hull the barrel's line is within 0.02 wu of every point");
    Check(worstEye > 0.8, "fixture: the eye's line misses by most of the pivot's distance somewhere");
}

void TestChildTurret()
{
    // A gun whose seat's frame turns WITH the gun (a child turret): the eye is the gun's own camera, the barrel a
    // fixed step beside it. One fire fits it for every heading, because the frame carries the heading.
    Model model;
    double worst = 0.0;
    for (int pass = 0; pass < 2; ++pass)
        for (double yaw : {0.0, 40.0, 130.0, -95.0})
        {
            Vehicle v{};
            const double yawRad = yaw * 0.017453292519943295;
            const float forward[3]{float(std::cos(yawRad)), float(std::sin(yawRad)), 0.0f}, up[3]{0.0f, 0.0f, 1.0f};
            MakeFrame(forward, up, v.hull);
            v.eye[0] = 5.0f;
            v.eye[1] = 6.0f;
            v.eye[2] = 1.0f;
            v.pivot[1] = -0.14;   // the barrel 0.14 wu to the right of the camera, in the turret's own frame
            v.pivot[2] = -0.05;
            double d[3]{v.hull.f[0], v.hull.f[1], v.hull.f[2]};
            if (pass == 0 && yaw == 0.0)
                Check(v.Fire(model, d), "the child turret's first fire is learned");
            float point[3]{};
            PointAlong(v, 0.0, 0.0, 9.0, point);
            const double miss = v.MissAt(model, point);
            Check(miss >= 0.0, "the model answers in the turret's frame");
            if (miss > worst)
                worst = miss;
        }
    Check(worst < 0.01, "one fire fits a gun whose frame turns with it");
}

void TestRefusals()
{
    const Vehicle v = Tank();
    Model model;
    double d[3]{};
    Heading(v.hull, 0.0, 0.0, d);
    float origin[3]{}, direction[3]{};
    v.Barrel(d, origin, direction);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    float badOrigin[3]{origin[0], nan, origin[2]}, zeroDir[3]{}, longDir[3]{direction[0] * 5, direction[1] * 5, direction[2] * 5};
    float farOrigin[3]{origin[0] + 40.0f, origin[1] + 40.0f, origin[2]};
    float alongUp[3]{float(v.hull.u[0]), float(v.hull.u[1]), float(v.hull.u[2])};
    Check(!model.Learn(v.hull, v.eye, badOrigin, direction), "a line that is not a number is not learned");
    Check(!model.Learn(v.hull, v.eye, origin, zeroDir) && !model.Learn(v.hull, v.eye, origin, longDir),
          "a direction that is not a direction is not learned");
    Check(!model.Learn(v.hull, v.eye, farOrigin, direction), "a line further away than a gun can be is not learned");
    Check(!model.Learn(v.hull, v.eye, origin, alongUp), "a line along the vehicle's up is not learned");
    Check(model.fires() == 0, "none of them counted");

    Check(v.Fire(model, d), "a good fire");
    float near[3]{}, bad[3]{nan, 0.0f, 0.0f}, aim[3]{7.0f, 7.0f, 7.0f};
    PointAlong(v, 0.0, 0.0, 0.3, near);
    Check(!model.Aim(v.hull, v.eye, near, aim) && aim[0] == 7.0f, "a point nearer than the minimum range is declined");
    PointAlong(v, 0.0, 0.0, 1.0, near);   // the barrel sits about 0.4 wu off the eye across this heading
    Check(!model.Aim(v.hull, v.eye, near, aim), "a point within a few barrel offsets is declined");
    Check(!model.Aim(v.hull, v.eye, bad, aim) && !model.Aim(v.hull, bad, near, aim), "a point or an eye that is not a number");
    Check(!model.Aim(v.hull, v.eye, v.eye, aim), "the eye itself is no point to aim at");

    model.Reset();
    float point[3]{};
    PointAlong(v, 0.0, 0.0, 30.0, point);
    Check(model.fires() == 0 && !model.Aim(v.hull, v.eye, point, aim), "a reset forgets the gun");
}

void TestBadFireDoesNotLock()
{
    // One wrong line (a fire while the barrel was elsewhere) is outvoted by the fires after it.
    const Vehicle v = Tank();
    Model model;
    double d[3]{};
    Heading(v.hull, 10.0, 0.0, d);
    float origin[3]{}, direction[3]{};
    v.Barrel(d, origin, direction);
    float wrong[3]{origin[0] + 0.6f, origin[1] - 0.4f, origin[2] + 0.3f};
    Check(model.Learn(v.hull, v.eye, wrong, direction), "a plausible but wrong line is learned");
    for (int round = 0; round < 6; ++round)
        for (double yaw : {-60.0, -20.0, 10.0, 50.0})
        {
            Heading(v.hull, yaw, 2.0, d);
            v.Fire(model, d);
        }
    float point[3]{};
    PointAlong(v, 30.0, 2.0, 30.0, point);
    const double miss = v.MissAt(model, point);
    Check(miss >= 0.0 && miss < 0.03, "and the fit recovers after the good fires that follow");
}

// A turret whose barrel settles a fixed step off the aim it is given (right and up, radians), and an engine that
// turns the shot toward the point inside a cone about the barrel (0: the shot is the barrel).
struct Turret
{
    Vehicle v{};
    double errorRight = 0.0, errorUp = 0.0, cone = 0.0;

    // The fire for a written aim: the barrel's own direction, then the engine's turn toward the point.
    void Fire(const float aim[3], const float point[3], float origin[3], float shot[3]) const
    {
        double d[3]{aim[0], aim[1], aim[2]}, right[3]{}, up[3]{};
        Normalize(d);
        AimAxes(v.hull, d, right, up);
        double barrel[3]{d[0] + errorRight * right[0] + errorUp * up[0], d[1] + errorRight * right[1] + errorUp * up[1],
                         d[2] + errorRight * right[2] + errorUp * up[2]};
        Normalize(barrel);
        float direction[3]{};
        v.Barrel(barrel, origin, direction);
        double wanted[3]{double(point[0]) - origin[0], double(point[1]) - origin[1], double(point[2]) - origin[2]};
        Normalize(wanted);
        const double angle = std::acos(std::fmin(1.0, Dot(barrel, wanted)));
        double out[3]{barrel[0], barrel[1], barrel[2]};
        if (angle > 1e-12)
        {
            const double t = std::fmin(1.0, cone / angle);
            for (int i = 0; i < 3; ++i)
                out[i] = barrel[i] + (wanted[i] - barrel[i]) * t;
            Normalize(out);
        }
        for (int i = 0; i < 3; ++i)
            shot[i] = static_cast<float>(out[i]);
    }
    // Fires `rounds` at the point, learning from each, and returns the last shot's miss at the point.
    double Shoot(Model& model, Pointing& pointing, const float point[3], int rounds) const
    {
        double miss = -1.0;
        for (int round = 0; round < rounds; ++round)
        {
            float aim[3]{float(point[0] - v.eye[0]), float(point[1] - v.eye[1]), float(point[2] - v.eye[2])};
            const bool hadModel = model.fires() != 0;
            if (hadModel && model.Aim(v.hull, v.eye, point, aim))
                pointing.Apply(v.hull, aim);
            float origin[3]{}, shot[3]{};
            Fire(aim, point, origin, shot);
            const double o[3]{origin[0], origin[1], origin[2]}, s[3]{shot[0], shot[1], shot[2]},
                p[3]{point[0], point[1], point[2]};
            miss = LineMiss(o, s, p);
            model.Learn(v.hull, v.eye, origin, shot);
            if (hadModel)
                pointing.Learn(v.hull, origin, shot, point);
        }
        return miss;
    }
};

void TestPointing()
{
    // The barrel settles 0.9 degrees right and 0.3 up of the aim; the engine does not turn the shot at all.
    Turret t{};
    t.v = Tank();
    t.errorRight = 0.9 * kDegToRad;
    t.errorUp = 0.3 * kDegToRad;
    float point[3]{};
    PointAlong(t.v, 15.0, 4.0, 60.0, point);
    {
        Model model;
        Pointing pointing;
        const double first = t.Shoot(model, pointing, point, 1);
        Check(first > 0.5, "fixture: the first shot misses a point 60 wu away by more than half a world unit");
        const double second = t.Shoot(model, pointing, point, 1);
        Check(second > 0.5, "the second shot has the barrel's place and still misses: the barrel points off the aim");
        const double third = t.Shoot(model, pointing, point, 1);
        Check(third < 0.3 * second, "the third shot is pushed by the second's residual and lands much nearer");
        const double later = t.Shoot(model, pointing, point, 4);
        Check(later < 0.02, "a few shots on, the shot's line runs through the point");
        Check(pointing.fires >= 5, "each of those fires was learned");
        // Another range on the same heading needs no new learning: the push is an angle.
        float far[3]{};
        PointAlong(t.v, 15.0, 4.0, 240.0, far);
        Check(t.Shoot(model, pointing, far, 1) < 0.08, "the same push serves another range on that heading");
    }
    {
        // The engine turns the shot toward the point inside half a degree: the push stops growing once the barrel
        // is inside the cone, and the shot is then exact.
        t.cone = 0.5 * kDegToRad;
        Model model;
        Pointing pointing;
        const double later = t.Shoot(model, pointing, point, 6);
        Check(later < 0.01, "with a cone the engine closes the last of it and the shot is on the point");
        const double pushDeg = std::sqrt(pointing.right * pointing.right + pointing.up * pointing.up) / kDegToRad;
        Check(pushDeg > 0.3 && pushDeg < 1.2, "and the push is no more than the barrel's own error");
        t.cone = 0.0;
    }
    {
        // A fire far off the aim (the turret was still turning) teaches nothing.
        Pointing pointing;
        float origin[3]{}, shot[3]{};
        float aim[3]{float(point[0] - t.v.eye[0]), float(point[1] - t.v.eye[1]), float(point[2] - t.v.eye[2])};
        Turret lagging = t;
        lagging.errorRight = 6.0 * kDegToRad;
        lagging.Fire(aim, point, origin, shot);
        Check(!pointing.Learn(t.v.hull, origin, shot, point) && pointing.fires == 0,
              "a residual past the limit is the turret not being on the aim: not learned");
        float direction[3]{aim[0], aim[1], aim[2]};
        Check(pointing.Apply(t.v.hull, direction) && direction[0] == aim[0], "and an unlearned push changes nothing");
    }
    {
        // The push is capped.
        Pointing pointing;
        Turret wild = t;
        wild.errorRight = 2.5 * kDegToRad;
        Model model;
        for (int round = 0; round < 12; ++round)
        {
            float aim[3]{float(point[0] - t.v.eye[0]), float(point[1] - t.v.eye[1]), float(point[2] - t.v.eye[2])};
            float origin[3]{}, shot[3]{};
            wild.Fire(aim, point, origin, shot);   // the aim is never pushed here: the residual keeps coming
            pointing.Learn(t.v.hull, origin, shot, point);
        }
        const double pushDeg = std::sqrt(pointing.right * pointing.right + pointing.up * pointing.up) / kDegToRad;
        Check(pushDeg <= kMaxPointingDeg + 1e-9 && pushDeg > kMaxPointingDeg - 0.01, "the push stops at its cap");
        const float nan = std::numeric_limits<float>::quiet_NaN();
        float bad[3]{nan, 0.0f, 0.0f}, origin[3]{}, shot[3]{1.0f, 0.0f, 0.0f};
        double r = 0.0, u = 0.0;
        Check(!Pointing::Residual(t.v.hull, origin, shot, bad, r, u) && !Pointing::Residual(t.v.hull, origin, bad, point, r, u),
              "a residual of values that are not numbers is none");
    }
}
}   // namespace

int main()
{
    TestFrame();
    TestNoFire();
    TestOneFire();
    TestManyFires();
    TestChildTurret();
    TestRefusals();
    TestBadFireDoesNotLock();
    TestPointing();
    std::printf("seat_barrel_tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
