// FLAT PARITY, Halo 2 on foot (sight_layer.h SightFireEyeFn; shot_lag_logic.h 4e; 2026-10-03). A muzzle shot flies
// through P' = P + (E_fire - E_P): E_P the engine eye of the observer publication P was built from, E_fire the eye
// the round leaves from in flat -- the eye AFTER this tick's move, one tick of travel (or more) ahead of the newest
// render. Its source here is the firing assist's own camera evaluator, 0x6D4730 (H2EK 489E10, "with native
// perspective return"; DUAL-NATIVE-TARGETING-2026-09-18.md), which the assist calls at 0x7597B8 inside the fire
// 0x8E4940 (Halo2IndependentCameraDetour below). Asked for our unit inside the same fire, it answers the eye the
// assist traces from.
//
// Its answer counts as E_fire only once PROVEN (shot_lag::EvaluatorEyeProof): at rest it must agree with the
// publication's engine eye within 5 cm for a streak of samples, and only on the perspective it answered while
// agreeing. Samples come from every firing-assist call for our unit (the game's own call: no extra native work) and
// from the muzzle shot's own ask. Until then -- and on any other perspective -- E_fire is E_P carried one measured
// tick (shot_lag::ChooseSampleFireEye), the flat probe's model. Every verdict is counted on the dev SHOT LAG transport
// line. Fire thread (the simulation thread) only.
void* Halo2IndependentTargetStorage(uint32_t unit);

struct Halo2FireEyeState
{
    SRWLOCK lock = SRWLOCK_INIT;
    shot_lag::EvaluatorEyeProof proof{};
    // What the log last said (a transition is logged once, every build: rare and load-bearing).
    shot_lag::EyeProof::State reportedState = shot_lag::EyeProof::State::Unproven;
    bool reportedPath = false;
} g_halo2FireEye;

// Identity: our owned on-foot biped (unseated, no controlling parent: the same unit the muzzle shot admits) while
// the controller owns Halo 2's aim.
bool Halo2FireEyeEligible(uint32_t unit)
{
    if (unit == UINT32_MAX || unit != Halo2OwnedUnit() || !Game_Halo2ControllerAimActive() ||
        !Halo2Observer6Dof_DirectWeaponAimArmed())
        return false;
    __try
    {
        return Halo2IndependentTargetStorage(unit) != nullptr;   // reads the unit: guarded, as every caller is not
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// One sample for the proof: the evaluator's answer `position` (null: non-finite) on perspective `path`, for `unit`,
// against the newest publication's engine eye. Only our unit's samples reach it (another unit's call would end our
// stint), so the callers check the unit first.
void Halo2NoteFireEyeProof(uint32_t unit, const float* position, int32_t path)
{
    Halo2ObserverPosePublication publication{};
    const uint32_t generation = g_generation.load(std::memory_order_acquire);
    const uint64_t nowUs = Sight_NowUs();
    const bool read = generation && Halo2Observer6Dof_ReadPublishedPose(publication) &&
        publication.generation == generation && !publication.vehicleReferenceValid && publication.publishedUs &&
        nowUs >= publication.publishedUs && nowUs - publication.publishedUs <= 100000;
    float eye[3]{};
    Halo2EngineEyePosition(publication, eye);
    const bool atRest = read && publication.engineEyeVelValid &&
        shot_lag::Length(publication.engineEyeVel) * shot_lag::kMetresPerWu < 0.25f;
    float offsetMm = std::numeric_limits<float>::quiet_NaN();
    if (position && shot_lag::Finite3(position) && shot_lag::Finite3(eye))
    {
        const float d[3]{position[0] - eye[0], position[1] - eye[1], position[2] - eye[2]};
        offsetMm = shot_lag::Length(d) * shot_lag::kMmPerWu;
    }
    const bool eligible = read && Halo2FireEyeEligible(unit);
    auto& state = g_halo2FireEye;
    AcquireSRWLockExclusive(&state.lock);
    state.proof.Note(generation, static_cast<int32_t>(unit), eligible, atRest, offsetMm, path);
    const shot_lag::EyeProof::State now = state.proof.proof.state;
    const bool pathKnown = state.proof.pathKnown;
    const int32_t latched = state.proof.path;
    const bool changed = now != state.reportedState || pathKnown != state.reportedPath;
    state.reportedState = now;
    state.reportedPath = pathKnown;
    ReleaseSRWLockExclusive(&state.lock);
    if (changed)
        LOG("Halo 2 fire eye (flat parity): the firing assist's camera evaluator is %s%s (perspective %s%d); a "
            "muzzle shot's eye at the fire is %s", now == shot_lag::EyeProof::State::Proven ? "PROVEN"
            : now == shot_lag::EyeProof::State::Disproven ? "DISPROVEN" : "unproven",
            now == shot_lag::EyeProof::State::Proven ? " to be the engine eye at rest" : "",
            pathKnown ? "" : "not yet latched, last ", pathKnown ? latched : path,
            now == shot_lag::EyeProof::State::Proven ? "measured (that evaluator asked at the fire)"
                                                     : "the newest publication's eye carried one tick");
}

// The evaluator asked for `unit` now, on the fire thread, under SEH with the lane's callback count held (the
// trampoline cannot be retired under it: RemoveHalo2DualAim's quiescence). Returns false when it could not be asked.
bool Halo2CallFireEyeEvaluator(uint32_t unit, float position[3], int32_t& path)
{
    auto& feature = g_halo2Dual;
    feature.callbacks.fetch_add(1, std::memory_order_acq_rel);
    bool called = false;
    __try
    {
        const Halo2DualCameraFn evaluator = feature.cameraOriginal;
        if (evaluator && feature.enabled.load(std::memory_order_acquire) &&
            !feature.faulted.load(std::memory_order_acquire) &&
            feature.generation == g_generation.load(std::memory_order_acquire))
        {
            float direction[3]{};
            path = evaluator(unit, position, direction);
            called = true;
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        called = false;
    }
    feature.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    return called;
}

shot_lag::FireEye Halo2AskFireEye(float eye[3])
{
    const uint32_t unit = Halo2OwnedUnit();
    if (!Halo2FireEyeEligible(unit))
        return shot_lag::FireEye::OtherUnit;
    float position[3]{};
    int32_t path = 0;
    if (!Halo2CallFireEyeEvaluator(unit, position, path))
        return shot_lag::FireEye::Faulted;
    const bool finite = shot_lag::Finite3(position);
    Halo2NoteFireEyeProof(unit, finite ? position : nullptr, path);   // our own ask is a sample too
    auto& state = g_halo2FireEye;
    AcquireSRWLockShared(&state.lock);
    const shot_lag::FireEye verdict = state.proof.Verdict(path, finite);
    ReleaseSRWLockShared(&state.lock);
    if (verdict == shot_lag::FireEye::Measured)
        std::memcpy(eye, position, sizeof(position));
    return verdict;
}

shot_lag::FireEye Halo2FireEyeForShot(const float* siteEye, bool askEvaluator, const SightPointSample& sample,
                                      float eye[3], float& ageMs)
{
    (void)siteEye;   // Halo 2's muzzle shot holds no eye of its own
    ageMs = -1.0f;
    if (TitleAdapter_GetActiveTitle() != GameTitle::Halo2)
        return shot_lag::FireEye::NoSource;
    float measured[3]{};
    const shot_lag::FireEye asked = askEvaluator ? Halo2AskFireEye(measured) : shot_lag::FireEye::Count;
    // 60 Hz while unmeasured (MCC runs Halo 2's simulation at 60 Hz: HALO2-SIGNATURE-EVIDENCE.md E-H2-28).
    const float tickS = float(Sight_TickPeriodUs(GameTitle::Halo2, 16667)) * 1.0e-6f;
    const uint64_t nowUs = Sight_NowUs();
    const shot_lag::FireEye verdict = shot_lag::ChooseSampleFireEye(
        asked, measured, sample.camera.valid, sample.camera.eye, sample.camera.velValid ? sample.camera.vel : nullptr,
        shot_lag::SampleCarrySeconds(sample.camera.stampUs, nowUs, tickS, kSightTransportMaxAgeUs), eye);
    if (verdict == shot_lag::FireEye::Measured)
        ageMs = 0.0f;
    else if (verdict == shot_lag::FireEye::CarriedTick && sample.camera.stampUs && nowUs >= sample.camera.stampUs)
        ageMs = float(nowUs - sample.camera.stampUs) / 1000.0f;
    return verdict;
}
const bool g_halo2FireEyeRegistered = (Sight_SetFireEyeSource(GameTitle::Halo2, &Halo2FireEyeForShot), true);
