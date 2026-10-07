// REACH NAVIGATION MARKERS: THE MARKER PROJECTION ITSELF -- a hook on the game's one world-point-to-HUD function
// (HREK reach_tag_test.exe +0x92D980, chud_draw.cpp; retail +0x2E1430). The sites, their proof and the pure logic
// are in src/common/reach_nav_marker_logic.h; the write-up is docs/REACH-NAV-MARKERS-2026-10-07.md.
//
// THE WITNESS (dev builds, and only with nav_marker_witness = 1 at the level's load). It READS: after the game
// projected a point, it compares the pixel the game put the marker on with the pixel the eye's own picture shows
// that point at, by the projection matrix the eye was rasterised with. That is the measurement behind "the marker
// sits on its target": the residual in pixels of the raster, per eye, with the switch on and off. It writes
// nothing of the game's.
#if HALOMCCVR_DEV_DIAGNOSTICS

// void project(record*, int user, bool ring, bool behind, bool worldPitch). tools/audit_caller_area.py
// haloreach.dll 2E1430:9:v is clean: no caller slot past the fifth argument, eleven native callers, none keeps a
// volatile register across the call or reads a return value.
using ReachNavProjectionFn = void (__fastcall*)(void*, int, uint8_t, uint8_t, uint8_t);
constexpr size_t kReachNavWitnessCount = static_cast<size_t>(reach_nav_markers::Witness::Count);
struct ReachNavProjectionRuntime
{
    uintptr_t base = 0;
    uint32_t generation = 0;
    void* target = nullptr;
    void* original = nullptr;                 // ReachNavProjectionFn
    reach_nav_markers::ProjectionSites sites;  // decoded from the game's own instructions at install
    std::atomic<bool> enabled{false};
    std::atomic<uint32_t> callbacks{0};
    // The window the report drains. Written by the render thread only.
    std::atomic<uint32_t> seen[kReachNavWitnessCount]{};
    std::atomic<uint32_t> compared[2]{};                 // per eye
    std::atomic<float> errorSum[2]{}, errorMax[2]{};     // pixels of the raster
    // The last compared projection of each eye.
    std::atomic<float> lastOffAxisDeg[2]{}, lastDistance[2]{};
    std::atomic<float> lastGame[2][2]{}, lastOwn[2][2]{};
    std::atomic<float> lastRing[2]{};                    // the last "how far outside the ring", per eye
    std::atomic<float> rasterWidth{0.0f}, rasterHeight{0.0f};
    std::atomic<uint32_t> ringCalls{0}, plainCalls{0};   // the caller asked for the ring, or not
} g_reachNavProjection;

// The eye's raster as the stereo transaction committed it for the eye being rendered now.
bool ReachNavEyeRaster(const ReachFpCameraEyeScope& scope, reach_nav_markers::EyeRaster& eye) noexcept
{
    memcpy(eye.camera.position, scope.compact + 0x00, sizeof(eye.camera.position));
    memcpy(eye.camera.forward, scope.compact + kReachCompactCameraForwardOffset, sizeof(eye.camera.forward));
    memcpy(eye.camera.up, scope.compact + 0x18, sizeof(eye.camera.up));
    memcpy(&eye.camera.verticalFov, scope.compact + reach_nav_markers::kCameraVerticalFovOffset,
           sizeof(eye.camera.verticalFov));
    // The derived block's projection matrix: [0] and [5] are its two scales (the same pair the stereo transaction
    // decodes to tell OpenXR what the raster covers).
    const float* projection = reinterpret_cast<const float*>(scope.derived + kReachDerivedProjectionOffset);
    eye.xScale = fabsf(projection[0]);
    eye.yScale = fabsf(projection[5]);
    // The compact camera's window: y0, x0, y1, x1 as 16-bit pixels (retail +0x287C5C writes them in that order).
    int16_t window[4]{};
    memcpy(window, scope.compact + 0x38, sizeof(window));
    eye.width = static_cast<float>(window[3] - window[1]);
    eye.height = static_cast<float>(window[2] - window[0]);
    return eye.width > 0.0f && eye.height > 0.0f && isfinite(eye.xScale) && isfinite(eye.yScale) &&
           eye.xScale > 0.0f && eye.yScale > 0.0f;
}

// The render thread, right after the game projected `record`. Reads only.
__declspec(noinline) void ReachNavWitnessProjection(const void* record, uint8_t ring) noexcept
{
    using namespace reach_nav_markers;
    auto& feature = g_reachNavProjection;
    Witness what = Witness::Fault;
    __try
    {
        const ReachFpCameraEyeScope& scope = g_reachFpCameraEyeScope;
        const auto* bytes = static_cast<const uint8_t*>(record);
        if (!ReachOwnsHudStereoTransaction() || scope.generation != feature.generation || scope.eye > 1)
            what = Witness::NoEyeScope;
        else if (!bytes[0])
            what = Witness::NotValid;
        else
        {
            const ProjectionSites& sites = feature.sites;
            const uint32_t eyeIndex = scope.eye;
            float point[3], pixel[2], offscreen = 0.0f, own[2];
            memcpy(point, bytes + sites.recordPoint, sizeof(point));
            memcpy(pixel, bytes + sites.recordPixel, sizeof(pixel));
            memcpy(&offscreen, bytes + sites.recordOffscreen, sizeof(offscreen));
            (ring ? feature.ringCalls : feature.plainCalls).fetch_add(1, std::memory_order_relaxed);
            EyeRaster eye;
            if (!ReachNavEyeRaster(scope, eye))
                what = Witness::NoRaster;
            else if (offscreen > 0.0f)
            {
                feature.lastRing[eyeIndex].store(offscreen, std::memory_order_relaxed);
                what = Witness::OnRing;
            }
            else if (!EyeRasterPixel(eye, point, own))
                what = Witness::NotInFront;
            else
            {
                const float error = hypotf(pixel[0] - own[0], pixel[1] - own[1]);
                feature.compared[eyeIndex].fetch_add(1, std::memory_order_relaxed);
                feature.errorSum[eyeIndex].store(
                    feature.errorSum[eyeIndex].load(std::memory_order_relaxed) + error, std::memory_order_relaxed);
                if (error > feature.errorMax[eyeIndex].load(std::memory_order_relaxed))
                    feature.errorMax[eyeIndex].store(error, std::memory_order_relaxed);
                const float d[3]{point[0] - eye.camera.position[0], point[1] - eye.camera.position[1],
                                 point[2] - eye.camera.position[2]};
                feature.lastOffAxisDeg[eyeIndex].store(OffAxis(eye.camera, point) * kRadToDeg,
                                                       std::memory_order_relaxed);
                feature.lastDistance[eyeIndex].store(sqrtf(Dot(d, d)), std::memory_order_relaxed);
                for (int i = 0; i < 2; ++i)
                {
                    feature.lastGame[eyeIndex][i].store(pixel[i], std::memory_order_relaxed);
                    feature.lastOwn[eyeIndex][i].store(own[i], std::memory_order_relaxed);
                }
                feature.rasterWidth.store(eye.width, std::memory_order_relaxed);
                feature.rasterHeight.store(eye.height, std::memory_order_relaxed);
                what = Witness::Compared;
            }
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        what = Witness::Fault;
    }
    feature.seen[static_cast<size_t>(what)].fetch_add(1, std::memory_order_relaxed);
}
__declspec(noinline) void __fastcall ReachNavProjectionDetour(void* record, int user, uint8_t ring, uint8_t behind,
                                                              uint8_t worldPitch)
{
    auto& feature = g_reachNavProjection;
    feature.callbacks.fetch_add(1, std::memory_order_acq_rel);
    __try
    {
        if (const auto original = reinterpret_cast<ReachNavProjectionFn>(feature.original))
            CallNativeWithSpareSlots(original, record, user, ring, behind, worldPitch);
        if (record && feature.enabled.load(std::memory_order_acquire))
            ReachNavWitnessProjection(record, ring);
    }
    __finally
    {
        feature.callbacks.fetch_sub(1, std::memory_order_release);
    }
}
// Disables, waits for the detour to be quiet, removes. True when nothing is left installed.
bool RemoveReachNavProjection()
{
    auto& feature = g_reachNavProjection;
    feature.enabled.store(false, std::memory_order_release);
    if (!feature.target)
        return true;
    const auto disabled = MCCVR_DisableHookForRetirement(feature.target);
    if (disabled != MH_OK && disabled != MH_ERROR_DISABLED && disabled != MH_ERROR_NOT_CREATED)
    {
        LOG("NAV MARKERS Reach (dev witness): CleanupRequired: disable failed (%d)", static_cast<int>(disabled));
        return false;
    }
    const void* functions[]{reinterpret_cast<const void*>(&ReachNavProjectionDetour)};
    const void* trampolines[]{feature.original};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 1, feature.callbacks))
    {
        LOG("NAV MARKERS Reach (dev witness): CleanupRequired: the detour is busy");
        return false;
    }
    const auto removed = MH_RemoveHook(feature.target);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
    {
        LOG("NAV MARKERS Reach (dev witness): CleanupRequired: removal failed (%d)", static_cast<int>(removed));
        return false;
    }
    feature.target = nullptr;
    feature.original = nullptr;
    return true;
}
// Cold, on the title's install thread, after the observer-camera hook's fate is known. Binds nothing unless every
// link proves itself on this image.
bool InstallReachNavProjection(uintptr_t base, size_t size, uint32_t generation)
{
    using namespace reach_nav_markers;
    auto& feature = g_reachNavProjection;
    if (!g_config.nav_marker_witness)
        return false;
    if (feature.target)
    {
        LOG("NAV MARKERS Reach (dev witness): StockFallback: the previous install was not retired");
        return false;
    }
    const uintptr_t head = ReachHeadRelativeUnique(base, size, kProjectionHeadPattern);
    const uintptr_t camera = ReachHeadRelativeUnique(base, size, kProjectionCameraPattern);
    const uintptr_t ring = ReachHeadRelativeUnique(base, size, kProjectionRingPattern);
    const uintptr_t offscreen = ReachHeadRelativeUnique(base, size, kProjectionOffscreenPattern);
    const uintptr_t pixel = ReachHeadRelativeUnique(base, size, kProjectionPixelPattern);
    uint8_t headBytes[kProjectionHeadBytes]{}, cameraBytes[kProjectionCameraBytes]{};
    uint8_t ringBytes[kProjectionRingBytes]{}, offscreenBytes[kProjectionOffscreenBytes]{};
    uint8_t pixelBytes[kProjectionPixelBytes]{};
    const bool headRead = ReachHeadRelativeRead(base, size, head, headBytes, kProjectionHeadBytes);
    const bool cameraRead = ReachHeadRelativeRead(base, size, camera, cameraBytes, kProjectionCameraBytes);
    const bool ringRead = ReachHeadRelativeRead(base, size, ring, ringBytes, kProjectionRingBytes);
    const bool offscreenRead = ReachHeadRelativeRead(base, size, offscreen, offscreenBytes, kProjectionOffscreenBytes);
    const bool pixelRead = ReachHeadRelativeRead(base, size, pixel, pixelBytes, kProjectionPixelBytes);
    const int64_t headRva = static_cast<int64_t>(head - base);
    ProjectionSites sites;
    ProjectionBind bind = DecodeProjectionSites(
        headRead ? headBytes : nullptr, headRva, headRead && ReachHeadRelativeFunctionRva(base, head) == headRva,
        cameraRead ? cameraBytes : nullptr, static_cast<int64_t>(camera - base),
        cameraRead ? ReachHeadRelativeFunctionRva(base, camera) : -1, ringRead ? ringBytes : nullptr,
        static_cast<int64_t>(ring - base), ringRead ? ReachHeadRelativeFunctionRva(base, ring) : -1,
        offscreenRead ? offscreenBytes : nullptr, static_cast<int64_t>(offscreen - base),
        offscreenRead ? ReachHeadRelativeFunctionRva(base, offscreen) : -1, pixelRead ? pixelBytes : nullptr,
        static_cast<int64_t>(pixel - base), pixelRead ? ReachHeadRelativeFunctionRva(base, pixel) : -1, sites);
    if (bind == ProjectionBind::Bound)
    {
        // The camera this function builds is the one the observer-camera hook corrects at its CHUD site.
        const uintptr_t observerHook = reinterpret_cast<uintptr_t>(g_reachCamera.observerCameraTarget);
        bind = CheckProjectionConverter(
            sites, observerHook > base ? static_cast<int64_t>(observerHook - base) : 0,
            static_cast<int64_t>(kReachObserverCameraReturnRvas[kReachObserverCameraChudSite]));
    }
    if (bind == ProjectionBind::Bound)
    {
        uint8_t fraction[sizeof(float)]{}, pi[sizeof(float)]{}, floor[sizeof(float)]{};
        const bool constantsRead =
            ReachHeadRelativeRead(base, size, base + static_cast<uintptr_t>(sites.fractionRva), fraction,
                                  sizeof(fraction)) &&
            ReachHeadRelativeRead(base, size, base + static_cast<uintptr_t>(sites.piRva), pi, sizeof(pi)) &&
            ReachHeadRelativeRead(base, size, base + static_cast<uintptr_t>(sites.floorRva), floor, sizeof(floor));
        bind = CheckProjectionConstants(constantsRead ? fraction : nullptr, pi, floor);
    }
    if (bind != ProjectionBind::Bound)
    {
        LOG("NAV MARKERS Reach (dev witness): StockFallback: %s (head %s, camera %s, ring %s, off-screen %s, pixel "
            "%s); nothing is measured",
            ProjectionBindName(bind), head ? "matched" : "unmatched", camera ? "matched" : "unmatched",
            ring ? "matched" : "unmatched", offscreen ? "matched" : "unmatched", pixel ? "matched" : "unmatched");
        return false;
    }
    feature.base = base;
    feature.generation = generation;
    feature.sites = sites;
    for (auto& count : feature.seen)
        count.store(0, std::memory_order_relaxed);
    for (int eye = 0; eye < 2; ++eye)
    {
        feature.compared[eye].store(0, std::memory_order_relaxed);
        feature.errorSum[eye].store(0.0f, std::memory_order_relaxed);
        feature.errorMax[eye].store(0.0f, std::memory_order_relaxed);
    }
    feature.ringCalls.store(0, std::memory_order_relaxed);
    feature.plainCalls.store(0, std::memory_order_relaxed);
    const auto created = MH_CreateHook(reinterpret_cast<void*>(head),
                                       reinterpret_cast<void*>(&ReachNavProjectionDetour), &feature.original);
    if (created != MH_OK)
    {
        feature.original = nullptr;
        LOG("NAV MARKERS Reach (dev witness): StockFallback: hook creation failed (%d)", static_cast<int>(created));
        return false;
    }
    feature.target = reinterpret_cast<void*>(head);
    const auto enabled = MH_EnableHook(feature.target);
    if (enabled != MH_OK)
    {
        LOG("NAV MARKERS Reach (dev witness): StockFallback: hook enable failed (%d)", static_cast<int>(enabled));
        (void)RemoveReachNavProjection();
        return false;
    }
    feature.enabled.store(true, std::memory_order_release);
    LOG("NAV MARKERS Reach (dev witness): Installed: the marker projection haloreach.dll+0x%llX is hooked (read "
        "only); its camera call returns to +0x%llX, the observer-camera hook's CHUD site, and lands on that hook's "
        "function +0x%llX; the ring reads the camera's vertical field of view at +0x%X; the record's point is at "
        "+0x%X, its pixel at +0x%X, its off-screen value at +0x%X",
        static_cast<unsigned long long>(sites.headRva), static_cast<unsigned long long>(sites.converterReturnRva),
        static_cast<unsigned long long>(sites.converterRva), sites.cameraFovOffset, sites.recordPoint,
        sites.recordPixel, sites.recordOffscreen);
    return true;
}

const char* ReachNavWitnessName(size_t what)
{
    return reach_nav_markers::WitnessName(static_cast<reach_nav_markers::Witness>(what));
}
// The worker's two-second report: one line per window in which the game projected anything.
void ReportReachNavProjection()
{
    using reach_nav_markers::Witness;
    auto& feature = g_reachNavProjection;
    if (!feature.target)
        return;
    uint32_t seen[kReachNavWitnessCount]{}, total = 0;
    for (size_t i = 0; i < kReachNavWitnessCount; ++i)
    {
        seen[i] = feature.seen[i].exchange(0, std::memory_order_relaxed);
        total += seen[i];
    }
    const uint32_t ringCalls = feature.ringCalls.exchange(0, std::memory_order_relaxed);
    const uint32_t plainCalls = feature.plainCalls.exchange(0, std::memory_order_relaxed);
    if (!total)
        return;
    char reasons[256];
    const int length = ReachHeadRelativeReasons(reasons, sizeof(reasons), seen, kReachNavWitnessCount,
                                                static_cast<size_t>(Witness::Compared), &ReachNavWitnessName);
    for (int eye = 0; eye < 2; ++eye)
    {
        const uint32_t compared = feature.compared[eye].exchange(0, std::memory_order_relaxed);
        const float sum = feature.errorSum[eye].exchange(0.0f, std::memory_order_relaxed);
        const float maximum = feature.errorMax[eye].exchange(0.0f, std::memory_order_relaxed);
        if (!compared)
            continue;
        const float distance = feature.lastDistance[eye].load(std::memory_order_relaxed);
        LOG("NAV MARKERS Reach (dev witness): eye %d: %u markers drawn on their point; the game's pixel against the "
            "eye's own picture: mean %.2f px, max %.2f px of a %.0f x %.0f raster; last: the point %.1f deg off the "
            "eye's axis, %.2f wu (%.1f m) away, drawn at (%.1f, %.1f), seen at (%.1f, %.1f); switch=%d",
            eye, compared, sum / static_cast<float>(compared), maximum,
            feature.rasterWidth.load(std::memory_order_relaxed), feature.rasterHeight.load(std::memory_order_relaxed),
            feature.lastOffAxisDeg[eye].load(std::memory_order_relaxed), distance, distance / kReachWorldUnitsPerMeter,
            feature.lastGame[eye][0].load(std::memory_order_relaxed),
            feature.lastGame[eye][1].load(std::memory_order_relaxed),
            feature.lastOwn[eye][0].load(std::memory_order_relaxed),
            feature.lastOwn[eye][1].load(std::memory_order_relaxed), g_config.nav_markers_on_target ? 1 : 0);
    }
    LOG("NAV MARKERS Reach (dev witness): %u projections (%u asked for the off-screen ring, %u did not); not "
        "compared: %s; last off-ring values %.4f / %.4f",
        total, ringCalls, plainCalls, length ? reasons : "none",
        feature.lastRing[0].load(std::memory_order_relaxed), feature.lastRing[1].load(std::memory_order_relaxed));
}
#else
bool InstallReachNavProjection(uintptr_t, size_t, uint32_t) { return false; }
bool RemoveReachNavProjection() { return true; }
void ReportReachNavProjection() {}
#endif
