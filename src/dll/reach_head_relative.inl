// REACH: THE MOTION TRACKER FOLLOWS THE HEAD -- the runtime half. The evidence, the sites and the pure logic are
// in src/common/reach_head_relative_logic.h; the write-up is docs/REACH-HEAD-RELATIVE-2026-10-06.md.
//
// In short: Reach's tracker takes its heading from player control's desired yaw -- the record this mod's direct
// drive writes the HAND's direction into -- so its contacts swing with the gun. The game stores that heading in
// the tracker's own object and then calls three functions; the first and the third read it. This lane hooks the
// first. Its detour runs after the store and before any read, and rewrites that one float with the head's yaw.
// It reads and writes nothing else: not the aim record, not a camera, nothing a shot or the network sees.

// ---- THE HEAD, as the frame that rendered with it published it (the two-clocks rule) ---------------------------
// Written by the render thread (ReachMainRenderViewBody, at the head camera's commit), read by whoever places
// something against where the player looks. A sequence lock: an odd sequence is a write in progress, and a read
// that saw the sequence move is retried. Every field is an atomic, so a torn read is impossible, only a mixed
// one, and the sequence refuses those.
struct ReachHeadPublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<float> forward[3]{}, up[3]{};
    std::atomic<uint64_t> stampUs{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint32_t> flags{0};  // bit 0: the mod's head view was live; bit 1: a cinematic held the camera
} g_reachHeadPublication;
constexpr uint32_t kReachHeadLive = 1u, kReachHeadCinematic = 2u;

// The render thread, once per committed head camera. `headCenter` is the committed compact camera (forward at
// +0x0C, up at +0x18, in the game's world); `headLookApplied` is false in the cutscene theater, where the
// camera is the authored one and the head did not steer it.
void ReachPublishHeadSample(const unsigned char* headCenter, bool headLookApplied, uint32_t generation) noexcept
{
    auto& published = g_reachHeadPublication;
    const float* forward = reinterpret_cast<const float*>(headCenter + 0x0C);
    const float* up = reinterpret_cast<const float*>(headCenter + 0x18);
    const bool live = headLookApplied && generation && head_relative::Finite3(forward) && head_relative::Finite3(up);
    const uint32_t flags = (live ? kReachHeadLive : 0u) |
                           (g_reachCinematicLocked.load(std::memory_order_relaxed) ? kReachHeadCinematic : 0u);
    const uint64_t nowUs = Sight_NowUs();
    const uint32_t sequence = published.sequence.load();
    published.sequence.store(sequence + 1);
    for (int i = 0; i < 3; ++i)
    {
        published.forward[i].store(forward[i]);
        published.up[i].store(up[i]);
    }
    published.stampUs.store(nowUs);
    published.generation.store(generation);
    published.flags.store(flags);
    published.sequence.store(sequence + 2);
}
// Any thread. False = nothing coherent could be read (the writer was mid-write four times running).
bool ReachReadHeadSample(head_relative::HeadSample& sample, bool& cinematic) noexcept
{
    auto& published = g_reachHeadPublication;
    for (int attempt = 0; attempt < 4; ++attempt)
    {
        const uint32_t before = published.sequence.load();
        if (before & 1u)
            continue;
        for (int i = 0; i < 3; ++i)
        {
            sample.forward[i] = published.forward[i].load();
            sample.up[i] = published.up[i].load();
        }
        sample.stampUs = published.stampUs.load();
        sample.generation = published.generation.load();
        const uint32_t flags = published.flags.load();
        if (published.sequence.load() != before)
            continue;
        sample.valid = (flags & kReachHeadLive) != 0;
        cinematic = (flags & kReachHeadCinematic) != 0;
        return true;
    }
    return false;
}

// ---- THE TRACKER -----------------------------------------------------------------------------------------------
// HREK reach_tag_test.exe+0x954FD0 / retail +0x2EC164: void(tracker). tools/audit_caller_area.py 2EC164:8:v is
// clean -- it touches no caller slot past its home area and its one native caller keeps no volatile register
// across the call -- and it reads no register but rcx on entry.
using ReachTrackerReaderFn = void (__fastcall*)(void*);
constexpr size_t kReachTrackerUseCount = static_cast<size_t>(head_relative::Use::Count);
struct ReachTrackerHeadRuntime
{
    uintptr_t base = 0;
    uint32_t generation = 0;
    void* target = nullptr;
    ReachTrackerReaderFn original = nullptr;
    uint32_t headingOffset = 0, userOffset = 0;  // decoded from the game's own instructions at install
    std::atomic<bool> enabled{false};
    std::atomic<uint32_t> callbacks{0};
    // The window the report drains: every call lands in exactly one of these.
    std::atomic<uint32_t> uses[kReachTrackerUseCount]{};
    std::atomic<uint32_t> faults{0}, mixedReads{0};
    std::atomic<float> lastNativeDeg{0.0f}, lastHeadDeg{0.0f};
    head_relative::HeadingHold hold;  // the tracker update's thread only
    // The report's thread only.
    int reportedSwitch = -1;
    bool reportedFirstUse = false, reportedFirstRefusal = false;
} g_reachTrackerHead;

// The tracker update's thread, between the game's store of the heading and its first read. No allocation, no
// logging, no lock; every refusal leaves the game's own heading and is counted by reason.
__declspec(noinline) void ReachTrackerApplyHead(void* tracker) noexcept
{
    auto& feature = g_reachTrackerHead;
    head_relative::Use use = head_relative::Use::NoSample;
    __try
    {
        reach_head_relative::TrackerRequest request;
        request.switchOn = g_config.tracker_follows_head;
        request.sign = g_config.tracker_head_sign;
        request.offsetDeg = g_config.tracker_head_offset_deg;
        request.nowUs = Sight_NowUs();
        // This hook belongs to one load of the title; a sample from another is not this tracker's head.
        const uint32_t generation = g_reachCamera.generation.load(std::memory_order_acquire);
        request.generation = generation == feature.generation ? generation : 0;
        head_relative::HeadSample sample;
        bool cinematic = false;
        if (!ReachReadHeadSample(sample, cinematic))
        {
            sample = head_relative::HeadSample{};
            feature.mixedReads.fetch_add(1, std::memory_order_relaxed);
        }
        request.gameplay = !cinematic;
        auto* bytes = static_cast<uint8_t*>(tracker);
        request.outputUser = *reinterpret_cast<const int32_t*>(bytes + feature.userOffset);
        float* field = reinterpret_cast<float*>(bytes + feature.headingOffset);
        const float native = *field;
        float replaced = native, headYaw = 0.0f;
        use = reach_head_relative::TrackerField(request, sample, feature.hold, native, replaced, &headYaw);
        if (use == head_relative::Use::Ok)
        {
            *field = replaced;
            feature.lastNativeDeg.store(reach_head_relative::YawFromField(native) * head_relative::kRadToDeg,
                                        std::memory_order_relaxed);
            feature.lastHeadDeg.store(headYaw * head_relative::kRadToDeg, std::memory_order_relaxed);
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        feature.faults.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    feature.uses[static_cast<size_t>(use)].fetch_add(1, std::memory_order_relaxed);
}
__declspec(noinline) void __fastcall ReachTrackerHeadDetour(void* tracker)
{
    auto& feature = g_reachTrackerHead;
    feature.callbacks.fetch_add(1, std::memory_order_acq_rel);
    __try
    {
        if (tracker && feature.enabled.load(std::memory_order_acquire))
            ReachTrackerApplyHead(tracker);
        if (feature.original)
            CallNativeWithSpareSlots(feature.original, tracker);
    }
    __finally
    {
        feature.callbacks.fetch_sub(1, std::memory_order_release);
    }
}

bool RemoveReachHeadRelative()
{
    auto& feature = g_reachTrackerHead;
    feature.enabled.store(false, std::memory_order_release);
    if (!feature.target)
        return true;
    const auto disabled = MCCVR_DisableHookForRetirement(feature.target);
    if (disabled != MH_OK && disabled != MH_ERROR_DISABLED && disabled != MH_ERROR_NOT_CREATED)
    {
        LOG("TRACKER HEAD Reach: CleanupRequired: disable failed (%d)", static_cast<int>(disabled));
        return false;
    }
    const void* functions[]{reinterpret_cast<const void*>(&ReachTrackerHeadDetour)};
    const void* trampolines[]{reinterpret_cast<const void*>(feature.original)};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 1, feature.callbacks))
    {
        LOG("TRACKER HEAD Reach: CleanupRequired: the detour is busy");
        return false;
    }
    const auto removed = MH_RemoveHook(feature.target);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
    {
        LOG("TRACKER HEAD Reach: CleanupRequired: removal failed (%d)", static_cast<int>(removed));
        return false;
    }
    feature.target = nullptr;
    feature.original = nullptr;
    return true;
}

// The game's bytes at a fixed place, as they stood before any hook of ours. False = unreadable.
__declspec(noinline) bool ReachHeadRelativeCopy(uintptr_t address, void* out, size_t size) noexcept
{
    __try
    {
        (void)sig::CopyUnhooked(address, out, size);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
// A pattern's one match in the image, or 0 when it is missing or matches twice.
uintptr_t ReachHeadRelativeUnique(uintptr_t base, size_t size, const char* pattern)
{
    const uintptr_t match = sig::Find(base, size, pattern);
    if (!match || match + 1 >= base + size)
        return 0;
    return sig::Find(match + 1, base + size - match - 1, pattern) ? 0 : match;
}
// The beginning of the function `address` lies in, from the image's own function table, as an offset from
// `base`; -1 when the table has no entry for it or it belongs to another image.
int64_t ReachHeadRelativeFunctionRva(uintptr_t base, uintptr_t address, int64_t* endRva = nullptr)
{
    DWORD64 imageBase = 0;
    const auto* function = RtlLookupFunctionEntry(address, &imageBase, nullptr);
    if (!function || imageBase != base)
        return -1;
    if (endRva)
        *endRva = static_cast<int64_t>(function->EndAddress);
    return static_cast<int64_t>(function->BeginAddress);
}

// Cold, on the title's install thread, after the camera core. Binds nothing unless every link proves itself on
// this image; a refusal is the game's own tracker, said once.
bool InstallReachHeadRelative(uintptr_t base, size_t size, uint32_t generation)
{
    using namespace reach_head_relative;
    auto& feature = g_reachTrackerHead;
    if (feature.target)
    {
        LOG("TRACKER HEAD Reach: StockFallback: the previous install was not retired");
        return false;
    }
    if (!base || !generation || size < 0x1000)
    {
        LOG("TRACKER HEAD Reach: StockFallback: no title image or generation");
        return false;
    }
    const uintptr_t site = ReachHeadRelativeUnique(base, size, kTrackerHeadingSitePattern);
    const uintptr_t updateHead = ReachHeadRelativeUnique(base, size, kTrackerUpdateHeadPattern);
    const uintptr_t firstHead = ReachHeadRelativeUnique(base, size, kTrackerFirstReaderHeadPattern);
    const auto inside = [&](uintptr_t address, size_t bytes) {
        return address >= base && bytes <= size && address - base <= size - bytes;
    };
    uint8_t siteBytes[kSiteBytes]{}, headBytes[kUpdateHeadBytes]{}, firstBytes[kFirstReaderBytes]{};
    uint8_t thirdBytes[kThirdReaderBytes]{}, halfPiBytes[sizeof(float)]{};
    const bool siteRead = site && inside(site, kSiteBytes) && ReachHeadRelativeCopy(site, siteBytes, kSiteBytes);
    const bool headRead = updateHead && inside(updateHead, kUpdateHeadBytes) &&
                          ReachHeadRelativeCopy(updateHead, headBytes, kUpdateHeadBytes);
    const bool firstRead = firstHead && inside(firstHead, kFirstReaderBytes) &&
                           ReachHeadRelativeCopy(firstHead, firstBytes, kFirstReaderBytes);
    int64_t firstEndRva = 0;
    const bool firstIsFunction =
        firstRead && ReachHeadRelativeFunctionRva(base, firstHead, &firstEndRva) == static_cast<int64_t>(firstHead - base) &&
        firstEndRva >= static_cast<int64_t>(firstHead - base + kFirstReaderBytes);
    TrackerSites sites;
    TrackerBind bind = DecodeTrackerSites(siteRead ? siteBytes : nullptr, static_cast<int64_t>(site - base),
                                          headRead ? headBytes : nullptr, static_cast<int64_t>(updateHead - base),
                                          siteRead ? ReachHeadRelativeFunctionRva(base, site) : -1,
                                          firstIsFunction ? firstBytes : nullptr,
                                          static_cast<int64_t>(firstHead - base), sites);
    if (bind == TrackerBind::Bound)
    {
        const uintptr_t third = base + static_cast<uintptr_t>(sites.thirdReaderRva);
        const uintptr_t halfPi = base + static_cast<uintptr_t>(sites.halfPiRva);
        const bool readersRead =
            inside(third, kThirdReaderBytes) && inside(halfPi, sizeof(float)) &&
            ReachHeadRelativeFunctionRva(base, third) == sites.thirdReaderRva &&
            ReachHeadRelativeCopy(third, thirdBytes, kThirdReaderBytes) &&
            ReachHeadRelativeCopy(halfPi, halfPiBytes, sizeof(float));
        bind = CheckTrackerReaders(firstBytes, readersRead ? thirdBytes : nullptr, halfPiBytes, sites);
    }
    if (bind != TrackerBind::Bound)
    {
        LOG("TRACKER HEAD Reach: StockFallback: %s (store %s, update %s, first reader %s); the game's own tracker",
            TrackerBindName(bind), site ? "matched" : "unmatched", updateHead ? "matched" : "unmatched",
            firstHead ? "matched" : "unmatched");
        return false;
    }
    feature.base = base;
    feature.generation = generation;
    feature.headingOffset = sites.headingOffset;
    feature.userOffset = sites.userOffset;
    feature.hold = head_relative::HeadingHold{};
    feature.reportedSwitch = -1;
    feature.reportedFirstUse = feature.reportedFirstRefusal = false;
    for (auto& count : feature.uses)
        count.store(0, std::memory_order_relaxed);
    feature.faults.store(0, std::memory_order_relaxed);
    feature.mixedReads.store(0, std::memory_order_relaxed);
    void* target = reinterpret_cast<void*>(firstHead);
    const auto created = MH_CreateHook(target, reinterpret_cast<void*>(&ReachTrackerHeadDetour),
                                       reinterpret_cast<void**>(&feature.original));
    if (created != MH_OK)
    {
        feature.original = nullptr;
        LOG("TRACKER HEAD Reach: StockFallback: hook creation failed (%d)", static_cast<int>(created));
        return false;
    }
    feature.target = target;
    const auto enabled = MH_EnableHook(target);
    if (enabled != MH_OK)
    {
        LOG("TRACKER HEAD Reach: StockFallback: hook enable failed (%d)", static_cast<int>(enabled));
        (void)RemoveReachHeadRelative();
        return false;
    }
    feature.enabled.store(true, std::memory_order_release);
    LOG("TRACKER HEAD Reach: Installed: the tracker's first heading reader haloreach.dll+0x%llX is hooked; the "
        "heading is tracker+0x%X (the game stores player control's desired yaw + pi/2 there at +0x%llX, and the "
        "other reader +0x%llX reads the same field), the tracker's user is +0x%X; follows the head for output "
        "user 0 in gameplay, from a head sample no older than %u ms; switch=%d sign=%d offset=%.1f deg",
        static_cast<unsigned long long>(sites.firstReaderRva), sites.headingOffset,
        static_cast<unsigned long long>(site - base + kSiteHeadingDisp - 4),
        static_cast<unsigned long long>(sites.thirdReaderRva), sites.userOffset,
        static_cast<unsigned>(head_relative::kMaxSampleAgeUs / 1000), g_config.tracker_follows_head ? 1 : 0,
        g_config.tracker_head_sign < 0 ? -1 : 1, g_config.tracker_head_offset_deg);
    return true;
}

// The worker's two-second report. Every build: one line when the player's switch changes, and one the first
// time the head is used (or first refused) after an install. Dev builds: the window's counts.
void ReportReachHeadRelative()
{
    auto& feature = g_reachTrackerHead;
    if (!feature.target)
        return;
    using head_relative::Use;
    const int switchNow = g_config.tracker_follows_head ? 1 : 0;
    if (feature.reportedSwitch != switchNow)
    {
        if (feature.reportedSwitch >= 0)
            LOG("TRACKER HEAD Reach: switch %s: the tracker now turns with %s", switchNow ? "on" : "off",
                switchNow ? "your head" : "the game's own heading (the aim)");
        feature.reportedSwitch = switchNow;
    }
    uint32_t uses[kReachTrackerUseCount]{}, refused = 0;
    for (size_t i = 0; i < kReachTrackerUseCount; ++i)
    {
        uses[i] = feature.uses[i].exchange(0, std::memory_order_relaxed);
        if (i != static_cast<size_t>(Use::Ok))
            refused += uses[i];
    }
    const uint32_t used = uses[static_cast<size_t>(Use::Ok)];
    const uint32_t faults = feature.faults.exchange(0, std::memory_order_relaxed);
    const uint32_t mixed = feature.mixedReads.exchange(0, std::memory_order_relaxed);
    if (!used && !refused && !faults)
        return;
    char reasons[256];
    int length = 0;
    reasons[0] = 0;
    for (size_t i = 0; i < kReachTrackerUseCount; ++i)
    {
        if (i == static_cast<size_t>(Use::Ok) || !uses[i] || length >= static_cast<int>(sizeof(reasons)) - 48)
            continue;
        const int wrote = _snprintf_s(reasons + length, sizeof(reasons) - length, _TRUNCATE, "%s%s=%u",
                                      length ? ", " : "", head_relative::UseName(static_cast<Use>(i)), uses[i]);
        if (wrote > 0)
            length += wrote;
    }
    if (used && !feature.reportedFirstUse)
    {
        feature.reportedFirstUse = true;
        LOG("TRACKER HEAD Reach: following the head: first contacts placed by the head's heading (the game's "
            "heading was %.1f deg, the head's %.1f deg)",
            feature.lastNativeDeg.load(std::memory_order_relaxed),
            feature.lastHeadDeg.load(std::memory_order_relaxed));
    }
    if (!used && (refused || faults) && !feature.reportedFirstRefusal)
    {
        feature.reportedFirstRefusal = true;
        LOG("TRACKER HEAD Reach: the game's own tracker for now: %s%s", length ? reasons : "a fault",
            faults && length ? ", and faults" : "");
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    LOG("TRACKER HEAD Reach (dev): substituted %u; the game's heading %.1f deg, the head's %.1f deg (last); "
        "refused %u: %s; faults %u, mixed sample reads %u; switch=%d sign=%d offset=%.1f deg",
        used, feature.lastNativeDeg.load(std::memory_order_relaxed),
        feature.lastHeadDeg.load(std::memory_order_relaxed), refused, length ? reasons : "none", faults, mixed,
        switchNow, g_config.tracker_head_sign < 0 ? -1 : 1, g_config.tracker_head_offset_deg);
#else
    (void)mixed;
#endif
}
