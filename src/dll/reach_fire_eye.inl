// Included in game.cpp (anonymous namespace) after reach_sight_trace.inl.
//
// FLAT PARITY, Halo Reach on foot (sight_layer.h SightFireEyeFn; shot_lag_logic.h 4f; 2026-10-03). Reach's on-foot
// shot sites aim through the provider's actuated WORLD point P: the unit-adjust re-aim and the muzzle shot from their
// own origins, the firing assist's crosshair query from its own start. Stock flat Halo fires along the crosshair
// direction from the eye AFTER the fire tick's move, so a site that aims through a fixed P trails a strafe by
// E_fire - E_P -- unless Reach fires from the eye the render showed. So each such shot aims through
// P' = P + (E_fire - E_P), with E_fire MEASURED or not at all:
//
//   E_P     the engine eye the provider solved from (the local shot ray's `base`, stamped at its publish);
//   E_fire  Reach's own unit camera-position evaluator (HREK 0xD76E60, retail 0x48A1C0: the trampoline, never our
//           hook), asked once inside the native fire for OUR unit, after the tick's move. It is the camera the
//           unit-adjust builds the native origin on (R-V12: origin := camera + forward * dot(origin - camera,
//           forward)); asked at the same instant, it answers the same eye.
//
// Its answer counts only once PROVEN (shot_lag::EvaluatorEyeProof): at rest, where the two are the same point by
// construction, it must agree with E_P within 5 cm for a streak of 30 samples. The samples come from the trace tick
// (ReachSightTraceTick: the per-unit update the aim trace already runs in, ~30 ms, our unit, on foot), so no shot is
// needed to prove it. Until proven -- or if disproven, or the evaluator is not installed on this build -- the shot
// keeps P (SightPointSample::measuredOnly): never carried a tick, never extrapolated (the user's call: leave Reach
// unshifted rather than carry blindly). If Reach does fire from the render's eye, the measured shift reads ~0 on the
// dev SHOT LAG transport lines, and the transport is a no-op.
//
// The direct drive's record is a DIRECTION solved from E_P (desired_angles); applied by Reach at its own eye at the
// fire it already passes through P', so it is not shifted. Seats are not covered (Sight_TransportWanted).

// Reach's evaluator answers one camera on foot (seated it reads the seat's camera marker; a seat is never eligible), so
// its proof latches this single path; identity (our unit, on foot, alive in gameplay, no cinematic) is the guard.
inline constexpr int32_t kReachFireEyePath = 0;

struct ReachFireEyeState
{
    SRWLOCK lock = SRWLOCK_INIT;
    shot_lag::EvaluatorEyeProof proof{};
    // Every build, per report window: asks inside a fire, and asks that could not reach the evaluator.
    std::atomic<uint32_t> asks{0}, unanswered{0};
#if HALOMCCVR_DEV_DIAGNOSTICS
    // The fire eye against the unit-adjust's native shot line (mm): ~0 says it is the camera that line was built on.
    std::atomic<uint32_t> lineSamples{0}, lineSumMm{0}, lineMaxMm{0};
    // Proof samples at rest, and their offset from E_P (mm).
    std::atomic<uint32_t> restSamples{0}, restSumMm{0}, restMaxMm{0};
#endif
} g_reachFireEye;

// The stock evaluator for `unit` (the trampoline), guarded. Callers hold a lane's callback count that keeps the core
// trampolines alive: g_reachCamera.activeCallbacks (the unit-adjust hook, the per-unit update) or the muzzle lane's
// (retired before the core). Kept free of C++ objects so __try is legal.
bool ReachCallUnitCameraEvaluator(int32_t unit, float* out)
{
    const ReachUnitCameraPositionFn evaluator = g_origReachUnitCameraPosition;
    if (!evaluator || !out || unit == -1)
        return false;
    __try
    {
        evaluator(unit, out);
        return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Inside a native fire, on the fire thread: this fire's eye for `unit`, asked once (a later site of the same fire
// reuses it) and only for our own unit. `fresh`: no enclosing fire scope exists (the fire detour is absent), so the
// scope cannot be trusted to belong to this fire -- ask anew.
void ReachMeasureFireEye(int32_t unit, bool fresh)
{
    if (unit == -1 || !Sight_TransportWanted(GameTitle::HaloReach, false) ||
        unit != LegacyCollisionIgnoredObject(GameTitle::HaloReach))
        return;
    shot_lag::FireEyeScope& scope = t_reachFireEye;
    if (!fresh && scope.asked && scope.unit == unit)
        return;
    float eye[3]{};
    const bool called = ReachCallUnitCameraEvaluator(unit, eye);
    scope.Store(unit, called, called ? eye : nullptr);
    g_reachFireEye.asks.fetch_add(1, std::memory_order_relaxed);
    if (!called)
        g_reachFireEye.unanswered.fetch_add(1, std::memory_order_relaxed);
}

// One proof sample from the trace tick (sim thread, inside the per-unit update, our unit only, ~30 ms): E_P from the
// freshest intent (its camera sample under 100 ms old), at rest only; the evaluator is asked only then. Every build.
void ReachNoteFireEyeProof(int32_t localUnit, bool onFoot, uint32_t generation)
{
    if (!g_origReachUnitCameraPosition || !Sight_TransportWanted(GameTitle::HaloReach, false))
        return;
    SightPointSample sample{};
    uint64_t atUs = 0;
    const uint64_t nowUs = Sight_NowUs();
    const bool read = onFoot && !g_reachCinematicLocked.load(std::memory_order_relaxed) &&
        ReachSightReadIntentSample(100000ull, sample, atUs) && sample.camera.valid &&
        nowUs >= sample.camera.stampUs && nowUs - sample.camera.stampUs <= 100000ull;
    const bool atRest = read && shot_lag::EyeAtRest(sample.camera.velValid, sample.camera.vel);
    float offsetMm = std::numeric_limits<float>::quiet_NaN();
    bool eligible = read;
    if (atRest)
    {
        float eye[3]{};
        eligible = ReachCallUnitCameraEvaluator(localUnit, eye);
        if (eligible)
        {
            const float d[3]{eye[0] - sample.camera.eye[0], eye[1] - sample.camera.eye[1],
                             eye[2] - sample.camera.eye[2]};
            offsetMm = shot_lag::Length(d) * shot_lag::kMmPerWu;
        }
    }
    auto& state = g_reachFireEye;
    AcquireSRWLockExclusive(&state.lock);
    state.proof.Note(generation, localUnit, eligible, atRest, offsetMm, kReachFireEyePath);
    ReleaseSRWLockExclusive(&state.lock);
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (eligible && atRest && std::isfinite(offsetMm))
    {
        const uint32_t mm = static_cast<uint32_t>(std::min(offsetMm, 4.0e6f));
        state.restSamples.fetch_add(1, std::memory_order_relaxed);
        state.restSumMm.fetch_add(mm, std::memory_order_relaxed);
        uint32_t prior = state.restMaxMm.load(std::memory_order_relaxed);
        while (mm > prior && !state.restMaxMm.compare_exchange_weak(prior, mm, std::memory_order_relaxed)) {}
    }
#endif
}

// Dev witness (the unit-adjust, after the native call built origin + direction): how far this fire's eye lies off the
// native shot line. Reach builds its origin on the evaluator's camera line, so the answer that is this fire's camera
// sits ~0 off it (a barrel offset aside); the solve's eye E_P sits off it by the eye's travel since the render.
void ReachNoteFireEyeNativeLine(int32_t unit, const float* origin, const float* direction)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    float eye[3]{};
    if (!origin || !direction || t_reachFireEye.For(unit, eye) != shot_lag::FireEye::Measured)
        return;
    const float offWu = shot_lag::PointLineDistance(eye, origin, direction);
    if (!std::isfinite(offWu))
        return;
    auto& state = g_reachFireEye;
    const uint32_t mm = static_cast<uint32_t>(std::min(offWu * shot_lag::kMmPerWu, 4.0e6f));
    state.lineSamples.fetch_add(1, std::memory_order_relaxed);
    state.lineSumMm.fetch_add(mm, std::memory_order_relaxed);
    uint32_t prior = state.lineMaxMm.load(std::memory_order_relaxed);
    while (mm > prior && !state.lineMaxMm.compare_exchange_weak(prior, mm, std::memory_order_relaxed)) {}
#else
    (void)unit; (void)origin; (void)direction;
#endif
}

// THE FIRE EYE (SightFireEyeFn), on the fire thread: this fire's evaluator answer for our unit, Measured only while
// proven on this generation; anything else is returned as the reason P stays (counted on the transport line).
shot_lag::FireEye ReachFireEyeForShot(const float* siteEye, bool askEvaluator, const SightPointSample& sample,
                                      float eye[3], float& ageMs)
{
    (void)siteEye; (void)askEvaluator; (void)sample;   // the fire's own ask (ReachMeasureFireEye) is the only source
    ageMs = -1.0f;
    if (TitleAdapter_GetActiveTitle() != GameTitle::HaloReach || !g_origReachUnitCameraPosition)
        return shot_lag::FireEye::NoSource;
    float answer[3]{};
    shot_lag::FireEye asked = t_reachFireEye.For(LegacyCollisionIgnoredObject(GameTitle::HaloReach), answer);
    if (asked == shot_lag::FireEye::Measured)
    {
        const uint32_t generation = g_reachCamera.generation.load(std::memory_order_acquire);
        auto& state = g_reachFireEye;
        AcquireSRWLockShared(&state.lock);
        asked = state.proof.proof.generation != generation ? shot_lag::FireEye::Unproven
                                                           : state.proof.Verdict(kReachFireEyePath, true);
        ReleaseSRWLockShared(&state.lock);
    }
    const shot_lag::FireEye verdict = shot_lag::ChooseMeasuredFireEye(asked, answer, eye);
    if (verdict == shot_lag::FireEye::Measured)
        ageMs = 0.0f;   // asked inside this fire
    return verdict;
}
const bool g_reachFireEyeRegistered = (Sight_SetFireEyeSource(GameTitle::HaloReach, &ReachFireEyeForShot), true);

// Cold worker (ReportReachSightTrace): the proof's state when it changes (every build: rare and load-bearing), a loud
// line when this build has no evaluator to measure with, and the dev witness.
void ReportReachFireEye()
{
    auto& state = g_reachFireEye;
    const bool wanted = Sight_TransportWanted(GameTitle::HaloReach, false);
    static int s_lastAvailable = -1;
    const int available = g_origReachUnitCameraPosition ? 1 : 0;
    if (wanted && available != s_lastAvailable)
    {
        if (!available)
            LOG("Reach fire eye (flat parity): the unit camera-position evaluator is NOT installed on this build, so "
                "no fire eye can be measured; Reach's on-foot shots keep aiming through the intent point P unmoved "
                "(no carried or extrapolated eye is used instead)");
        else if (s_lastAvailable == 0)
            LOG("Reach fire eye (flat parity): the unit camera-position evaluator is installed again");
        s_lastAvailable = available;
    }
    AcquireSRWLockShared(&state.lock);
    const shot_lag::EyeProof::State proofState = state.proof.proof.state;
    const bool pathKnown = state.proof.pathKnown;
    ReleaseSRWLockShared(&state.lock);
    static shot_lag::EyeProof::State s_reported = shot_lag::EyeProof::State::Unproven;
    static bool s_reportedPath = false;
    if (proofState != s_reported || pathKnown != s_reportedPath)
    {
        LOG("Reach fire eye (flat parity): the unit camera-position evaluator is %s; an on-foot shot's eye at the "
            "fire is %s",
            proofState == shot_lag::EyeProof::State::Proven ? "PROVEN to be the engine eye at rest"
            : proofState == shot_lag::EyeProof::State::Disproven ? "DISPROVEN (it disagrees with the engine eye at "
                                                                    "rest)"
                                                                  : "unproven",
            proofState == shot_lag::EyeProof::State::Proven ? "measured (that evaluator, asked in the fire)"
                                                            : "not measured: the shot keeps the intent point P");
        s_reported = proofState;
        s_reportedPath = pathKnown;
    }
    const uint32_t asks = state.asks.exchange(0, std::memory_order_relaxed);
    const uint32_t unanswered = state.unanswered.exchange(0, std::memory_order_relaxed);
#if HALOMCCVR_DEV_DIAGNOSTICS
    const uint32_t lines = state.lineSamples.exchange(0, std::memory_order_relaxed);
    const uint32_t lineSum = state.lineSumMm.exchange(0, std::memory_order_relaxed);
    const uint32_t lineMax = state.lineMaxMm.exchange(0, std::memory_order_relaxed);
    const uint32_t rests = state.restSamples.exchange(0, std::memory_order_relaxed);
    const uint32_t restSum = state.restSumMm.exchange(0, std::memory_order_relaxed);
    const uint32_t restMax = state.restMaxMm.exchange(0, std::memory_order_relaxed);
    if (asks || rests)
        LOG("REACH FIRE EYE (dev): proof %s; %u asked in a fire (%u unanswered); off the native shot line avg %u / max "
            "%u mm (%u unit-adjust shots: ~0 = the camera the native origin is built on); at rest vs the solve's eye "
            "avg %u / max %u mm (%u proof samples)",
            proofState == shot_lag::EyeProof::State::Proven ? "PROVEN"
            : proofState == shot_lag::EyeProof::State::Disproven ? "DISPROVEN" : "unproven",
            asks, unanswered, lines ? lineSum / lines : 0u, lineMax, lines, rests ? restSum / rests : 0u, restMax,
            rests);
#else
    (void)asks; (void)unanswered;
#endif
}
