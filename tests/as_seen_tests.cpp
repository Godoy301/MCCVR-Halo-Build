// As seen (src/common/as_seen_logic.h): an impact against the crosshair drawn in the first frame that shows it --
// the axes across a line, the offset at the impact's depth, the bins, the window's line, the camera census and the
// flat art size. The same header serves the VR dev witness and the flat probe, so these numbers are both hosts'.
#include "as_seen_logic.h"

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
bool Near(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; }
constexpr float kWuPerM = shot_lag::kWuPerMetre;
} // namespace

int main()
{
    using namespace as_seen;

    // ---- bins -----------------------------------------------------------------------------------------------------
    Check(RangeBin(0.0f) == 0 && RangeBin(4.99f) == 0 && RangeBin(5.0f) == 1 && RangeBin(12.0f) == 2 &&
              RangeBin(25.0f) == 3 && RangeBin(40.0f) == 4 && RangeBin(500.0f) == 4,
          "bins: range edges 5/10/20/40 m");
    Check(RangeBin(NAN) == kRangeBins - 1, "bins: a non-finite range is the far bin");
    Check(SpeedBin(0.0f) == 0 && SpeedBin(0.99f) == 0 && SpeedBin(1.0f) == 1 && SpeedBin(3.9f) == 1 &&
              SpeedBin(4.0f) == 2 && SpeedBin(NAN) == 0,
          "bins: speed edges 1/4 m/s");

    // ---- the axes across a line -----------------------------------------------------------------------------------
    {
        const float dir[3]{2, 0, 0};
        const float vel[3]{0, 5 * kWuPerM, 0};   // strafing +Y at 5 m/s
        const Across a = AxesAcross(dir, vel);
        Check(a.valid && a.travel && Near(a.speedMps, 5.0f, 1e-3f), "axes: travel across the line, its speed");
        Check(Near(a.t[1], 1.0f, 1e-6f) && Near(a.n[2], 1.0f, 1e-6f), "axes: travel +Y, up +Z");
        const float velDown[3]{3 * kWuPerM, 0, 0};   // moving along the line only
        const Across b = AxesAcross(dir, velDown);
        Check(b.valid && !b.travel && Near(b.t[1], -1.0f, 1e-6f) && Near(b.n[2], 1.0f, 1e-6f),
              "axes: no travel -> the horizontal right of the line, up stays up");
        const float zero[3]{0, 0, 0};
        Check(!AxesAcross(zero, vel).valid, "axes: a zero line is refused");
        const float vertical[3]{0, 0, 1};
        Check(AxesAcross(vertical, nullptr).valid, "axes: a vertical line still has axes");
    }

    // ---- the offset at the impact's depth -------------------------------------------------------------------------
    {
        // The eye at the origin looks +X; strafing +Y at 6 m/s; the impact 10 m out, 0.30 m BEHIND (-Y), 0.05 m up.
        const float origin[3]{0, 0, 0};
        const float dir[3]{1, 0, 0};
        const float vel[3]{0, 6 * kWuPerM, 0};
        const float impact[3]{10 * kWuPerM, -0.30f * kWuPerM, 0.05f * kWuPerM};
        const Offset o = Measure(impact, origin, dir, vel);
        Check(o.valid && o.travel, "offset: valid, with travel");
        Check(Near(o.alongMm, -300.0f, 0.5f) && Near(o.upMm, 50.0f, 0.5f), "offset: along (behind) and up, in mm");
        Check(Near(o.depthM, 10.0f, 1e-3f) && Near(o.rangeM, std::sqrt(100.0f + 0.09f + 0.0025f), 1e-3f),
              "offset: depth along the line and range from the eye");
        Check(Near(o.alongDeg, -std::atan2(0.30f, 10.0f) * 57.29578f, 1e-3f) &&
                  Near(o.upDeg, std::atan2(0.05f, 10.0f) * 57.29578f, 1e-3f),
              "offset: the components as angles from the eye");
        Check(Near(o.angleDeg, std::atan2(std::sqrt(0.09f + 0.0025f), 10.0f) * 57.29578f, 1e-3f),
              "offset: the total angle off the line");
        Check(Near(o.speedMps, 6.0f, 1e-3f), "offset: the strafe speed across the line");
        // Strafing the other way flips the sign: the same point is now AHEAD.
        const float back[3]{0, -6 * kWuPerM, 0};
        Check(Near(Measure(impact, origin, dir, back).alongMm, 300.0f, 0.5f), "offset: along is signed by travel");
        // A point behind the eye is refused (a crosshair line never shows it).
        const float behindEye[3]{-1 * kWuPerM, 0, 0};
        Check(!Measure(behindEye, origin, dir, vel).valid, "offset: a point behind the eye is refused");
        // Without a velocity: no travel, along = horizontal right of the line (-Y for +X), still measured.
        const Offset still = Measure(impact, origin, dir, nullptr);
        Check(still.valid && !still.travel && Near(still.alongMm, 300.0f, 0.5f), "offset: no velocity, right axis");
    }

    // ---- the eye's own move along the travel ----------------------------------------------------------------------
    {
        const float dir[3]{1, 0, 0};
        const float vel[3]{0, 6 * kWuPerM, 0};
        const float delta[3]{0.2f * kWuPerM, 0.25f * kWuPerM, 0};
        bool known = false;
        Check(Near(AlongTravelMm(delta, dir, vel, &known), 250.0f, 0.5f) && known, "eye moved: along travel, mm");
        AlongTravelMm(delta, dir, nullptr, &known);
        Check(!known, "eye moved: unknown without a travel axis");
    }

    // ---- the window ---------------------------------------------------------------------------------------------
    {
        Window w{};
        char line[4096]{};
        Check(!w.Format("Halo 3 VR", line, sizeof(line)), "window: empty says nothing");
        // Two impacts at ~10 m while strafing 6 m/s, each 300 mm behind the reticle the frame drew.
        const float origin[3]{0, 0, 0}, dir[3]{1, 0, 0}, vel[3]{0, 6 * kWuPerM, 0};
        const float impact[3]{12 * kWuPerM, -0.30f * kWuPerM, 0};
        Sample s{};
        s.seen = Measure(impact, origin, dir, vel);
        s.stockValid = true;
        s.stock = s.seen;
        s.atFireValid = true;
        s.atFire = Measure(impact, origin, dir, vel);
        s.aimCamUs = 1000000;
        s.fireUs = 1015700;
        s.impactUs = 1050000;
        s.frameCamUs = 1055000;
        s.presentUs = 1060000;
        s.ticksFireToImpact = 2;
        s.ticksAimToFrame = 3;
        s.eyeMovedKnown = true;
        s.eyeMovedAlongMm = 300.0f;
        w.Add(s, 5.05f);
        w.Add(s, 5.05f);
        const Bin& b = w.bin[SpeedBin(6.0f)][RangeBin(12.0f)];
        Check(w.impacts == 2 && b.n == 2, "window: binned by strafe speed and range");
        Check(Near(static_cast<float>(b.impliedMs / b.nImplied), 50.0f, 0.1f),
              "window: 300 mm behind at 6 m/s is 50 ms of strafe");
        Check(Near(static_cast<float>(b.eyeMovedMs / b.nEye), 50.0f, 0.1f), "window: the eye's move as time");
        Check(Near(static_cast<float>(b.aimToFireMs / b.nAimFire), 15.7f, 0.01f) &&
                  Near(static_cast<float>(b.fireToImpactMs / b.nFireImpact), 34.3f, 0.01f) &&
                  Near(static_cast<float>(b.impactToFrameMs / b.nImpactFrame), 5.0f, 0.01f) &&
                  Near(static_cast<float>(b.frameToPresentMs / b.nFramePresent), 5.0f, 0.01f),
              "window: the clock intervals");
        Check(Near(static_cast<float>(b.seenAlongRadii / b.nRadii), s.seen.alongDeg / 5.05f, 1e-4f),
              "window: radii are the along angle over the reticle radius");
        Sample bad{};
        w.Add(bad, 5.05f);
        Check(w.refused == 1 && w.impacts == 2, "window: an invalid sample is refused, not binned");
        w.unframed = 3;
        Check(w.Format("Halo 3 VR", line, sizeof(line)), "window: formats");
        Check(std::strstr(line, "AS SEEN Halo 3 VR (dev): 2 impacts shown") != nullptr &&
                  std::strstr(line, "[4+ m/s, 10-20 m] 2 at 12.0 m") != nullptr &&
                  std::strstr(line, "SEEN along -300 mm") != nullptr &&
                  std::strstr(line, "= 50 ms of strafe") != nullptr && std::strstr(line, "(2.0 ticks)") != nullptr &&
                  std::strstr(line, "3 never reached a frame") != nullptr &&
                  std::strstr(line, "ticks aim cam->frame 3.0") != nullptr,
              "window: the line names the bin, the offset, its time and the clocks");
        // A tiny buffer truncates without overrunning.
        char tiny[32];
        std::memset(tiny, 'x', sizeof(tiny));
        w.Format("Halo 3 VR", tiny, sizeof(tiny));
        Check(std::memchr(tiny, '\0', sizeof(tiny)) != nullptr, "window: truncation keeps a terminator");
    }

    // ---- the camera census --------------------------------------------------------------------------------------
    {
        // Interpolated: moves 6 m/s every 11.1 ms frame.
        CameraCensus smooth{};
        float p[3]{0, 0, 0};
        uint64_t t = 1000000;
        for (int i = 0; i < 90; ++i)
        {
            smooth.Add(p, t);
            p[1] += 6.0f * kWuPerM * 0.0111f;
            t += 11100;
        }
        Check(smooth.moving > 80 && smooth.repeated == 0 && smooth.uneven == 0, "census: interpolated never repeats");
        Check(Near(static_cast<float>(smooth.stepRatioSum / smooth.stepN), 1.0f, 0.02f),
              "census: interpolated steps one frame's worth");
        // Tick-stepped: the eye advances 16.7 ms of travel at each 60 Hz tick, sampled by 90 Hz frames.
        CameraCensus stepped{};
        float q[3]{0, 0, 0};
        for (int i = 0; i < 180; ++i)
        {
            const uint64_t frameUs = 1000000 + static_cast<uint64_t>(i) * 11111;
            const uint64_t ticks = (frameUs - 1000000) / 16667;
            q[1] = static_cast<float>(ticks) * 6.0f * kWuPerM * 0.016667f;
            stepped.Add(q, frameUs);
        }
        const float repeatShare = static_cast<float>(stepped.repeated) / static_cast<float>(stepped.moving);
        Check(stepped.moving > 150 && Near(repeatShare, 1.0f / 3.0f, 0.05f),
              "census: a 60 Hz camera at 90 Hz repeats on a third of the frames");
        Check(stepped.uneven > stepped.moving / 2, "census: and its steps are uneven");
        char clause[512]{};
        Check(stepped.Format(clause, sizeof(clause)) && std::strstr(clause, "REPEATED") != nullptr,
              "census: formats");
        stepped.ResetCounts();
        Check(!stepped.Format(clause, sizeof(clause)), "census: counts reset, history kept");
        // At rest nothing counts as moving.
        CameraCensus rest{};
        const float still[3]{1, 2, 3};
        for (int i = 0; i < 30; ++i)
            rest.Add(still, 1000000 + static_cast<uint64_t>(i) * 11111);
        Check(rest.frames == 30 && rest.moving == 0, "census: at rest nothing moves");
    }

    // ---- the flat art size ----------------------------------------------------------------------------------------
    {
        // A 5242-wide VR HUD and a flat 78 deg horizontal FOV (tan 39 deg).
        const float half = FlatArtHalfAngleDeg(5242.0f, std::tan(39.0f / 57.29578f));
        Check(Near(half, std::atan(128.0f / 5242.0f * std::tan(39.0f / 57.29578f)) * 57.29578f, 1e-4f) &&
                  half > 1.0f && half < 1.3f,
              "flat art: the captured region is ~1.1 deg of a 78 deg flat view");
        Check(std::isnan(FlatArtHalfAngleDeg(0.0f, 0.8f)) && std::isnan(FlatArtHalfAngleDeg(5242.0f, -1.0f)),
              "flat art: refuses nonsense");
    }

    std::printf("%u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
