#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

inline uint32_t ClassifyAuthoredReticleColorOrdering(const float channel[4])
{
    if (!channel)
        return 0;
    for (unsigned int i = 0; i < 4; ++i)
    {
        const float value = channel[i];
        if (!std::isfinite(value) || value < -0.01f || value > 4.0f)
            return 0;
    }

    // Encode pairwise ordering instead of naming channels. Halo 3's placement
    // output is known to contain four colour/alpha floats, but its alpha slot
    // need not be assumed here. Uniform opacity scaling preserves ordering and
    // therefore cannot create recurring swapchain uploads.
    uint32_t ordering = 1;
    unsigned int shift = 1;
    for (unsigned int i = 0; i < 4; ++i)
    {
        for (unsigned int j = i + 1; j < 4; ++j)
        {
            const float tolerance = 0.05f *
                std::fmax(std::fabs(channel[i]), std::fabs(channel[j]));
            if (channel[i] > channel[j] + tolerance)
                ordering |= 1u << shift;
            else if (channel[j] > channel[i] + tolerance)
                ordering |= 2u << shift;
            shift += 2;
        }
    }
    return ordering;
}

enum class AuthoredReticleRefreshPolicy : uint8_t
{
    IdentityAndColorState,
    IdentityImmediate,
    BoundedAnimation,
};

// Floor on how often the blocking OpenXR acquire/wait/copy/release may run.
// Measured at ~4-5 ms of the render window, which is the whole margin between
// a 120 Hz budget (8.33 ms) and missing it, so it must never run every frame.
inline constexpr uint64_t kMinimumUploadGapFrames = 6;

struct AuthoredReticleRefreshState
{
    uint64_t ownerEpoch = 0;
    uint64_t lastPublishedKey = 0;
    uint64_t lastUploadFrame = 0;
    uint64_t settlingKey = 0;
    uint64_t settlingSinceFrame = 0;
    uint32_t lastPublishedColorState = 0;
    uint32_t lastPublishedDraws = 0;
};

inline void ResetAuthoredReticleRefreshState(
    AuthoredReticleRefreshState& state, uint64_t ownerEpoch)
{
    state = {};
    state.ownerEpoch = ownerEpoch;
}

// A captured art identity must persist before it is allowed to replace art
// already on the quad. Two independent reasons, one mechanism:
//
// - A new identity can precede visible pixels during a level or weapon
//   transition, so publishing it on sight replaces good art with a blank
//   capture.
// - Reach emits a SHORT-LIVED alternate class-2 widget set during combat. The
//   preserved accepted Reach run
//   `out/test-runs/74e1477-reach-outer-camera-commit-pass-20260729-112718`
//   records the published key flipping to one and the same weapon-independent
//   value and back within ~50 ms, repeatedly (11:24:12.173/.224,
//   13.491/.542, 16.136/.187, 16.237/.288, 28.548/.599, 36.782/.834,
//   50.010/.061, 54.432/.484, 11:25:06.629/.680, 07.446/.496). The same
//   transient value appears against two different steady weapon keys, so it
//   is not the held weapon changing. Across every one of those flips the quad
//   heartbeat still reads `SUBMITTED ... chain=1 heldArt=1`, which proves the
//   quad and the swapchain were never the problem - the ART was replaced.
//
// Official HREK CHUD tag exports independently rule out the engine hiding the
// crosshair on damage: no collection whose scripting class is `crosshair`
// carries any damage-driven state, and the `<player> taking damage` condition
// is defined in the globals enum but used by no widget in any of the 63
// exported definitions.
inline constexpr uint64_t kAuthoredIdentitySettleFrames = 24;

inline bool AuthoredReticleIdentitySettled(
    uint64_t capturedKey, uint64_t frameSerial,
    AuthoredReticleRefreshState& state)
{
    if (capturedKey != state.settlingKey ||
        frameSerial < state.settlingSinceFrame)
    {
        state.settlingKey = capturedKey;
        state.settlingSinceFrame = frameSerial;
        return false;
    }
    return frameSerial - state.settlingSinceFrame >=
        kAuthoredIdentitySettleFrames;
}

inline bool ShouldUploadAuthoredReticle(
    AuthoredReticleRefreshPolicy policy,
    bool capturedThisFrame,
    bool uploadAdmitted,
    uint64_t capturedKey,
    uint32_t capturedColorState,
    uint32_t capturedDraws,
    uint64_t frameSerial,
    uint64_t ownerEpoch,
    AuthoredReticleRefreshState& state)
{
    if (state.ownerEpoch != ownerEpoch)
        ResetAuthoredReticleRefreshState(state, ownerEpoch);

    if (!capturedThisFrame || !uploadAdmitted || capturedKey == 0)
        return false;

    if (state.lastUploadFrame != 0 && frameSerial >= state.lastUploadFrame &&
        frameSerial - state.lastUploadFrame < kMinimumUploadGapFrames)
        return false;

    if (policy == AuthoredReticleRefreshPolicy::BoundedAnimation)
    {
        // A capture that painted FEWER widget pieces than the art already on
        // the quad is the crosshair on its way OUT, not a new crosshair.
        // Reach's class-2 draws thin out before they stop (preserved c2d9149
        // log: a window uploading 19 where the steady rate was 30), so a
        // frame can capture a fragment of the reticle, and publishing that
        // fragment is what the player sees as the crosshair disappearing.
        // A count of 0 means the title supplies no count, which leaves this
        // inert - that is how Halo 3 and ODST keep their accepted behavior.
        const bool thinnerThanPublished =
            capturedDraws != 0 && state.lastPublishedDraws != 0 &&
            capturedDraws < state.lastPublishedDraws;
        // Republishing the identity ALREADY on the quad is the animation
        // itself - the crosshair kicks on fire and tints on a target without
        // changing which widgets drew - so it keeps the unconditional bounded
        // cadence and this change costs it nothing.
        if (capturedKey == state.lastPublishedKey && !thinnerThanPublished)
            return true;
        // Nothing published yet (first arm, or an owner reset): the crosshair
        // must appear immediately rather than waiting out a settle window.
        if (state.lastPublishedKey == 0)
            return true;
        // A DIFFERENT or THINNER capture must prove it is not a momentary
        // flicker before it replaces good held art. This is the same
        // protection Halo 3's identity policy below already had, which the
        // bounded-animation path was returning above. A crosshair that really
        // has changed - or really has fewer pieces from now on - still
        // publishes once the window passes, so nothing is permanently stuck.
        return AuthoredReticleIdentitySettled(capturedKey, frameSerial, state);
    }
    if (policy == AuthoredReticleRefreshPolicy::IdentityImmediate)
        return capturedKey != state.lastPublishedKey;

    // Once Halo 3 has published a settled widget, a categorical CHUD colour
    // transition is safe to publish immediately. It costs one upload per real
    // blue/green/red edge and no work while the state is unchanged.
    if (capturedKey == state.lastPublishedKey && capturedColorState != 0 &&
        capturedColorState != state.lastPublishedColorState)
        return true;

    if (capturedKey == state.lastPublishedKey)
        return false;

    // A new widget identity can precede visible pixels during level/weapon
    // transitions. Settle that identity before its first publish so a blank
    // capture cannot replace good held art.
    return AuthoredReticleIdentitySettled(capturedKey, frameSerial, state);
}

inline void MarkAuthoredReticleUploaded(
    AuthoredReticleRefreshState& state,
    uint64_t capturedKey,
    uint32_t capturedColorState,
    uint32_t capturedDraws,
    uint64_t frameSerial)
{
    state.lastPublishedKey = capturedKey;
    state.lastPublishedColorState = capturedColorState;
    state.lastPublishedDraws = capturedDraws;
    state.lastUploadFrame = frameSerial;
    state.settlingKey = 0;
    state.settlingSinceFrame = 0;
}

inline bool AuthoredReticleLayerHasContent(
    bool titleCapturesAuthoredArt,
    bool authoredArtHeld)
{
    return !titleCapturesAuthoredArt || authoredArtHeld;
}

// A title renderer can rebind scene colour after a private authored capture
// starts. That exact bind must stay inside the capture instead of returning to
// the normal eye target.
inline bool AuthoredReticleCaptureOwnsSceneBind(
    bool captureActive, bool sceneTargetMatches, bool privateTargetReady)
{
    return captureActive && sceneTargetMatches && privateTargetReady;
}

// Offscreen CHUD capture cadence.
//
// Capturing is not free: every class-2 widget piece redirects the render
// target, saves and restores viewport/scissor/RTV state on the immediate
// context, and draws itself into the private reticle texture. A title that
// already holds valid released art only has to re-sample the game's widget
// often enough to see it change - between samples the compositor keeps
// showing the last released image at no cost. This is the mechanism ODST
// already ships (`60f3929`); the gap is the only per-title difference.
//
// Every widget drawn during a selected frame must be admitted, so a compound
// reticle is captured whole rather than in pieces from different frames.
inline bool ShouldSampleAuthoredCapture(
    uint64_t gapFrames, uint64_t lastSampleSerial, uint64_t frameSerial)
{
    if (gapFrames == 0 || lastSampleSerial == 0)
        return true;
    if (frameSerial == lastSampleSerial)
        return true;
    // A serial restart (title change, session restart) re-samples immediately
    // rather than waiting out a gap measured against a stale frame number.
    if (frameSerial < lastSampleSerial)
        return true;
    return frameSerial - lastSampleSerial >= gapFrames;
}

// Halo 3's authored crosshair animates - it kicks when the weapon fires and
// turns red on a hostile / green on a friendly target - and none of that
// changes which widgets drew, so an identity-keyed publish freezes one
// snapshot. Publishing on a bounded cadence is what makes it live again, and
// the same cadence throttles the capture that feeds it, so the animation is
// paid for out of work the title was already doing every frame.
//
// 0 disables the animation entirely and holds one captured image, which is
// the cheapest possible behavior. Anything else is clamped to at least
// `kMinimumUploadGapFrames`, because a shorter gap cannot produce extra
// publishes anyway - it would only spend capture work that the upload floor
// then throws away.
inline uint64_t ResolveAuthoredAnimationGapFrames(int configuredFrames)
{
    constexpr int kMaximumAnimationGapFrames = 60;
    if (configuredFrames <= 0)
        return 0;
    if (configuredFrames < static_cast<int>(kMinimumUploadGapFrames))
        return kMinimumUploadGapFrames;
    if (configuredFrames > kMaximumAnimationGapFrames)
        return static_cast<uint64_t>(kMaximumAnimationGapFrames);
    return static_cast<uint64_t>(configuredFrames);
}

// WHERE THE ART SITS IN ITS TEXTURE. The reticle quad is centred on the aim
// point, so the art's own centre has to be the texture's centre, or the player
// aims with one edge of it. Halo 4's capture framing is fitted per HUD layout,
// and on the 2026-09-30 layout (baseY 647.256) it left the art wholly above the
// centre: the magnum's bottom edge sat on the aim point, about 0.18 of the
// texture high (1.3 deg at the 6 m cap). The centre is measured instead: the
// ink-weighted centroid of a downsampled copy of the capture. Reticles are
// symmetric about their aim point, so the centroid is that point.
struct AuthoredReticleArtCentre
{
    float u = 0.5f;  // 0..1 across the texture, left to right
    float v = 0.5f;  // 0..1 down the texture, top to bottom
    uint64_t ink = 0;
    bool valid = false;
};

// `rows`: `size` x `size` RGBA8 texels, `rowPitch` bytes apart. `colourInk`
// weighs each texel by r+g+b (Halo 4's CUI target writes colour with a zero
// alpha channel), otherwise by alpha. Less than `minimumInk` in total is no art.
inline AuthoredReticleArtCentre MeasureAuthoredReticleArtCentre(
    const uint8_t* rows, size_t rowPitch, uint32_t size, bool colourInk,
    uint64_t minimumInk)
{
    AuthoredReticleArtCentre out{};
    if (!rows || size == 0 || rowPitch < static_cast<size_t>(size) * 4)
        return out;
    double sumX = 0.0, sumY = 0.0;
    uint64_t ink = 0;
    for (uint32_t y = 0; y < size; ++y)
    {
        const uint8_t* texel = rows + static_cast<size_t>(y) * rowPitch;
        for (uint32_t x = 0; x < size; ++x, texel += 4)
        {
            const uint32_t weight = colourInk
                ? static_cast<uint32_t>(texel[0]) + texel[1] + texel[2]
                : texel[3];
            if (!weight)
                continue;
            sumX += (x + 0.5) * weight;
            sumY += (y + 0.5) * weight;
            ink += weight;
        }
    }
    out.ink = ink;
    if (ink == 0 || ink < minimumInk)
        return out;
    out.u = static_cast<float>(sumX / static_cast<double>(ink) / size);
    out.v = static_cast<float>(sumY / static_cast<double>(ink) / size);
    out.valid = std::isfinite(out.u) && std::isfinite(out.v);
    return out;
}

// Bloom and anti-aliasing move a capture's centroid by a texel or so from one
// sample to the next. The centre in use only follows a measurement that moves
// it by more than `deadband` (a fraction of the texture), so the quad does not
// shimmer with them; a new weapon or layout moves it at once. A sample that
// could not be measured keeps the centre in use.
inline AuthoredReticleArtCentre SettleAuthoredReticleArtCentre(
    const AuthoredReticleArtCentre& applied,
    const AuthoredReticleArtCentre& measured, float deadband)
{
    if (!measured.valid)
        return applied;
    if (!applied.valid || std::fabs(measured.u - applied.u) > deadband ||
        std::fabs(measured.v - applied.v) > deadband)
        return measured;
    return applied;
}

// The shift that puts the art's centre, not the texture's, on the quad's
// intended centre: metres along the quad's own right and up axes, for a square
// quad `width` wide whose texture runs top to bottom down it. Art above the
// texture centre (v < 0.5) moves the quad down. Clamped to a third of the
// quad, so a stray measurement cannot carry the reticle far off its ray.
struct AuthoredReticleQuadShift
{
    float right = 0.0f;
    float up = 0.0f;
};

inline AuthoredReticleQuadShift AuthoredReticleArtCentreShift(
    const AuthoredReticleArtCentre& centre, float width)
{
    constexpr float kMaximumOffset = 1.0f / 3.0f;
    AuthoredReticleQuadShift shift{};
    if (!centre.valid || !std::isfinite(width) || width <= 0.0f ||
        !std::isfinite(centre.u) || !std::isfinite(centre.v))
        return shift;
    const auto clampOffset = [](float value) {
        return value < -kMaximumOffset ? -kMaximumOffset
            : (value > kMaximumOffset ? kMaximumOffset : value);
    };
    shift.right = -clampOffset(centre.u - 0.5f) * width;
    shift.up = clampOffset(centre.v - 0.5f) * width;
    return shift;
}

// WHAT A CAPTURED CROSSHAIR TEXEL HOLDS, AND WHAT THE COMPOSITOR MUST BE GIVEN.
//
// The capture is cleared to transparent black and the game then draws its
// widget into it with its OWN blend state. Whatever that state is, a widget
// that looks translucent over the flat scene ADDS some colour C to it, and
// over a zero target that added colour is exactly what lands in RGB: the
// capture's colour is PREMULTIPLIED (colour already scaled by the widget's
// opacity). Its alpha channel is whatever the game's alpha blend factors left
// there (the coverage itself for the usual premultiplied/over encodings).
//
// The aim quad is submitted with XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT,
// so the runtime multiplies colour by alpha AGAIN. Copying the capture
// straight in therefore applied the game's fade twice (C x A instead of C),
// and the sRGB decode of the already-darkened bytes dimmed it further: Reach's
// DMR, whose crosshair idles well below full opacity, read as far fainter in
// the headset than on the monitor.
//
// The fix converts to straight alpha before upload, in the shared blit shader's
// `fix()` (src/common/vr_blit_shader.h), which this function mirrors exactly:
// alpha = max(A, max(R,G,B)), colour = C / alpha. Straight-alpha compositing
// then adds alpha x C/alpha = C - precisely the colour the flat game adds,
// whatever encoding the alpha channel used - and covers the background by the
// capture's own coverage. Taking the max also keeps the result a valid colour
// (<= 1) when the alpha channel under-reports coverage.
struct AuthoredReticleStraightTexel
{
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 0.0f;
};

inline AuthoredReticleStraightTexel AuthoredReticleStraightAlpha(
    float r, float g, float b, float a)
{
    const float alpha = std::fmax(a, std::fmax(r, std::fmax(g, b)));
    AuthoredReticleStraightTexel out{r, g, b, alpha};
    if (alpha > 0.0f)
    {
        out.r = r / alpha;
        out.g = g / alpha;
        out.b = b / alpha;
    }
    return out;
}

// One channel of a quad layer composited over `background`, as OpenXR defines
// the two encodings (all values 0..1, same colour space on both sides).
inline float CompositeUnpremultipliedChannel(
    float colour, float alpha, float background)
{
    return colour * alpha + background * (1.0f - alpha);
}

inline float CompositePremultipliedChannel(
    float colour, float alpha, float background)
{
    return colour + background * (1.0f - alpha);
}

// Development evidence for the fix above: what the capture held, what the
// flat game adds, what the headset was shown before the straight-alpha
// conversion and what it is shown now. Values are 0..255 and come from the
// coverage probe's small mip level, so they are box-filtered MEANS of the art
// (a 1-texel line reads fainter there than it is); compare them with each
// other, not with a screenshot.
struct AuthoredReticleAlphaSummary
{
    uint32_t inkTexels = 0;
    // Texels whose colour exceeds their alpha: the alpha channel
    // under-reports coverage, so the game's alpha blend factors are not the
    // premultiplied/over pair. Zero means the classic premultiplied encoding.
    uint32_t alphaBelowColourTexels = 0;
    uint32_t capturedAlphaPeak = 0;
    uint32_t capturedAlphaMean = 0;
    // Brightest colour channel = the colour the flat game ADDS to the scene.
    uint32_t addedColourPeak = 0;
    // What the headset added before the fix: colour x alpha a second time.
    uint32_t legacyAddedColourPeak = 0;
    // Alpha the compositor is given now (straight), and the colour it adds.
    uint32_t drawnAlphaPeak = 0;
    uint32_t drawnAlphaMean = 0;
    uint32_t drawnAddedColourPeak = 0;
};

inline AuthoredReticleAlphaSummary SummarizeAuthoredReticleAlpha(
    const uint8_t* rows, size_t rowPitch, uint32_t size)
{
    AuthoredReticleAlphaSummary out{};
    if (!rows || size == 0 || rowPitch < static_cast<size_t>(size) * 4)
        return out;
    uint64_t capturedSum = 0;
    uint64_t drawnSum = 0;
    for (uint32_t y = 0; y < size; ++y)
    {
        const uint8_t* texel = rows + static_cast<size_t>(y) * rowPitch;
        for (uint32_t x = 0; x < size; ++x, texel += 4)
        {
            const uint32_t colour = texel[0] > texel[1]
                ? (texel[0] > texel[2] ? texel[0] : texel[2])
                : (texel[1] > texel[2] ? texel[1] : texel[2]);
            const uint32_t alpha = texel[3];
            if (!colour && !alpha)
                continue;
            const uint32_t drawn = alpha > colour ? alpha : colour;
            const AuthoredReticleStraightTexel straight =
                AuthoredReticleStraightAlpha(
                    colour / 255.0f, 0.0f, 0.0f, alpha / 255.0f);
            const uint32_t drawnAdded = static_cast<uint32_t>(
                straight.r * straight.a * 255.0f + 0.5f);
            const uint32_t legacy = (colour * alpha + 127) / 255;
            ++out.inkTexels;
            if (colour > alpha)
                ++out.alphaBelowColourTexels;
            capturedSum += alpha;
            drawnSum += drawn;
            if (alpha > out.capturedAlphaPeak) out.capturedAlphaPeak = alpha;
            if (colour > out.addedColourPeak) out.addedColourPeak = colour;
            if (legacy > out.legacyAddedColourPeak)
                out.legacyAddedColourPeak = legacy;
            if (drawn > out.drawnAlphaPeak) out.drawnAlphaPeak = drawn;
            if (drawnAdded > out.drawnAddedColourPeak)
                out.drawnAddedColourPeak = drawnAdded;
        }
    }
    if (out.inkTexels)
    {
        out.capturedAlphaMean =
            static_cast<uint32_t>(capturedSum / out.inkTexels);
        out.drawnAlphaMean = static_cast<uint32_t>(drawnSum / out.inkTexels);
    }
    return out;
}
