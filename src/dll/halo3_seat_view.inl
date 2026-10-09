// THE HALO 3 SEAT VIEW: FIRST PERSON FOR THE CAMERA DIRECTOR ONLY. Included by game.cpp, after
// seat_aim_world_frame.inl. The why, the addresses and the relay are in src/common/seat_view_logic.h.
//
// WHAT THIS DOES TO THE GAME: three `call rel32` instructions in the camera director (halo3+0x212074, +0x2127EF,
// +0x212DCB) are pointed at a relay in memory of the mod's own, next to the module. The relay calls the function
// those calls went to, unchanged, and for one unit -- the local player, seated, while the mod's own seat camera
// owns the view -- answers "first person" where the game answered a settled "third person". No tag, no object
// field and no instruction on any path the simulation runs is written; the function itself is not hooked, so a
// host that hooks it for its own camera modes keeps its hook and the relay calls through it.
//
// THREADS. Install, sync and remove run on the WaitThread and own g_halo3SeatView. A site's five bytes are
// swapped with every other thread suspended and none of them inside the bytes (the freeze seat_aim_world_frame.inl
// uses). The relay runs on whichever thread the director runs on; it reads one atomic (the armed unit), which the
// camera thread writes from the seat sampler (halo3_native_seat_patch.inl).
//
// LIFETIME. The relay holds the module's address, so it is built per module generation and retired with it: the
// sites are restored first; the relay's memory is released only when its own count of threads inside it is zero
// and no suspended thread's instruction pointer lies in it. Until then removal reports "not yet" and is retried,
// like the other Halo 3 teardown steps.

namespace
{
struct Halo3SeatViewSite
{
    uint8_t* call = nullptr;
    uint8_t stock[seat_view::kCallBytes]{};
    uint8_t redirect[seat_view::kCallBytes]{};
    bool patched = false;
};
struct Halo3SeatViewState
{
    uintptr_t base = 0;
    uint32_t generation = 0;
    uint8_t* relay = nullptr;            // one allocation: the relay, then its unwind data
    RUNTIME_FUNCTION* unwind = nullptr;  // inside that allocation
    bool unwindRegistered = false;
    Halo3SeatViewSite sites[seat_view::kDirectorSiteCount];
    bool standDown = false;              // a site turned out not to be ours: nothing more is redirected here
    bool pendingLogged = false;
    uint32_t removeFailures = 0;
};
constexpr size_t kHalo3SeatViewRelayAllocation = 0x1000;
constexpr size_t kHalo3SeatViewUnwindInfoOffset = 0x100;
constexpr size_t kHalo3SeatViewUnwindEntryOffset = 0x140;

Halo3SeatViewState g_halo3SeatView;                     // WaitThread only
std::atomic<bool> g_halo3SeatViewRedirected{false};     // every director site calls the relay
std::atomic<int32_t> g_halo3SeatViewUnit{-1};           // the occupant the director is told is first person
volatile LONG g_halo3SeatViewInside = 0;                // threads inside the relay or below it (the relay counts)
std::atomic<uint32_t> g_halo3SeatViewAsked{0}, g_halo3SeatViewAnswered{0};

// The relay's second call, on the director's thread. Reads one atomic and the director's own local.
int Halo3SeatViewAdjust(int unit, const int* state, int result) noexcept
{
    g_halo3SeatViewAsked.fetch_add(1, std::memory_order_relaxed);
    if (result != seat_view::kThirdPerson || !state)
        return result;
    const int answer =
        seat_view::Decide(result, *state, unit, g_halo3SeatViewUnit.load(std::memory_order_acquire));
    if (answer != result)
        g_halo3SeatViewAnswered.fetch_add(1, std::memory_order_relaxed);
    return answer;
}

bool Halo3SeatViewRead(const uint8_t* at, uint8_t out[seat_view::kCallBytes])
{
    __try
    {
        memcpy(out, at, seat_view::kCallBytes);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
bool Halo3SeatViewWrite(uint8_t* at, const uint8_t* bytes)
{
    __try
    {
        memcpy(at, bytes, seat_view::kCallBytes);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Memory within a rel32's reach of `target`: the first free region found walking up from 0x70000000 below it.
uint8_t* Halo3SeatViewAllocateNear(uintptr_t target, size_t bytes)
{
    SYSTEM_INFO system{};
    GetSystemInfo(&system);
    const uintptr_t granularity = system.dwAllocationGranularity;
    if (!granularity || (granularity & (granularity - 1)) != 0)
        return nullptr;
    constexpr uintptr_t reach = 0x70000000u;
    const uintptr_t minimum = target > reach + granularity ? target - reach : granularity;
    const uintptr_t maximum = target <= UINTPTR_MAX - reach ? target + reach : UINTPTR_MAX;
    uintptr_t cursor = (minimum + granularity - 1) & ~(granularity - 1);
    for (uint32_t query = 0; cursor < maximum && query < 0x20000; ++query)
    {
        MEMORY_BASIC_INFORMATION info{};
        if (VirtualQuery(reinterpret_cast<const void*>(cursor), &info, sizeof(info)) != sizeof(info))
            break;
        const uintptr_t regionBase = reinterpret_cast<uintptr_t>(info.BaseAddress);
        const uintptr_t regionEnd = regionBase <= UINTPTR_MAX - info.RegionSize ? regionBase + info.RegionSize
                                                                                : UINTPTR_MAX;
        if (info.State == MEM_FREE)
        {
            const uintptr_t candidate = ((regionBase < cursor ? cursor : regionBase) + granularity - 1) &
                ~(granularity - 1);
            if (candidate < maximum && bytes <= maximum - candidate && candidate < regionEnd &&
                bytes <= regionEnd - candidate)
            {
                void* const allocation = VirtualAlloc(reinterpret_cast<void*>(candidate), bytes,
                                                      MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
                if (allocation)
                    return static_cast<uint8_t*>(allocation);
            }
        }
        if (regionEnd <= cursor)
            break;
        cursor = regionEnd;
    }
    return nullptr;
}

enum class Halo3SeatViewSwap
{
    Done,
    Busy,      // no thread freeze, or a thread inside the bytes: try again on the next poll
    Foreign,   // the bytes are neither the expected ones nor the replacement: not ours any more
};

// Replace `expected` with `replacement` at a call site while every other thread is frozen and outside the bytes.
Halo3SeatViewSwap Halo3SeatViewSwapBytes(uint8_t* at, const uint8_t* expected, const uint8_t* replacement)
{
    constexpr size_t n = seat_view::kCallBytes;
    uint8_t now[n];
    if (!SeatAimFrameExecutable(at, n) || !Halo3SeatViewRead(at, now))
        return Halo3SeatViewSwap::Foreign;
    if (!memcmp(now, replacement, n))
        return Halo3SeatViewSwap::Done;
    if (memcmp(now, expected, n))
        return Halo3SeatViewSwap::Foreign;
    DWORD previous = 0;
    if (!VirtualProtect(at, n, PAGE_EXECUTE_READWRITE, &previous))
        return Halo3SeatViewSwap::Busy;
    Halo3SeatViewSwap result = Halo3SeatViewSwap::Busy;
    {
        ReachThreadFreeze frozen;
        if (frozen.Capture())
        {
            bool safe = true;
            const auto site = reinterpret_cast<DWORD64>(at);
            for (const ReachFrozenThread& thread : frozen.Threads())
            {
                if (!thread.suspended)
                    continue;
                CONTEXT context{};
                context.ContextFlags = CONTEXT_CONTROL;
                if (!GetThreadContext(thread.handle, &context))
                {
                    if (WaitForSingleObject(thread.handle, 0) != WAIT_OBJECT_0)
                        safe = false;
                    continue;
                }
                if (context.Rip > site && context.Rip < site + n)
                    safe = false;
            }
            if (safe)
            {
                uint8_t check[n];
                if (!Halo3SeatViewRead(at, check) || memcmp(check, expected, n))
                    result = Halo3SeatViewSwap::Foreign;
                else if (Halo3SeatViewWrite(at, replacement) && Halo3SeatViewRead(at, check) &&
                         !memcmp(check, replacement, n))
                {
                    FlushInstructionCache(GetCurrentProcess(), at, n);
                    result = Halo3SeatViewSwap::Done;
                }
            }
            (void)frozen.Release();
        }
    }
    DWORD ignored = 0;
    VirtualProtect(at, n, previous, &ignored);
    return result;
}

// No thread is inside the relay, and none can return into it: its own count is zero and, with every other thread
// suspended, no instruction pointer lies in it. Only meaningful once every site is back to stock.
bool Halo3SeatViewRelayQuiet(const Halo3SeatViewState& s)
{
    if (!s.relay)
        return true;
    if (InterlockedCompareExchange(&g_halo3SeatViewInside, 0, 0) != 0)
        return false;
    bool quiet = false;
    ReachThreadFreeze frozen;
    if (frozen.Capture())
    {
        quiet = InterlockedCompareExchange(&g_halo3SeatViewInside, 0, 0) == 0;
        const auto begin = reinterpret_cast<DWORD64>(s.relay);
        for (const ReachFrozenThread& thread : frozen.Threads())
        {
            if (!thread.suspended)
                continue;
            CONTEXT context{};
            context.ContextFlags = CONTEXT_CONTROL;
            if (!GetThreadContext(thread.handle, &context))
            {
                if (WaitForSingleObject(thread.handle, 0) != WAIT_OBJECT_0)
                    quiet = false;
                continue;
            }
            if (context.Rip >= begin && context.Rip < begin + seat_view::kRelayBytes)
                quiet = false;
        }
        (void)frozen.Release();
    }
    return quiet;
}

void Halo3SeatViewReleaseRelay(Halo3SeatViewState& s)
{
    if (s.unwindRegistered && s.unwind)
        RtlDeleteFunctionTable(s.unwind);
    if (s.relay)
        VirtualFree(s.relay, 0, MEM_RELEASE);
    s.relay = nullptr;
    s.unwind = nullptr;
    s.unwindRegistered = false;
}

// The director is answered only where the tag is left alone (the default). With the tag written (the dev A/B
// key), the game answers first person by itself and the sites stay stock.
bool Halo3SeatViewWanted()
{
    return g_config.vehicle_first_person && g_config.vehicle_hide_body && !Halo3SeatViewWritesTag();
}

// WaitThread: bring the three sites in line with the setting. Cheap when nothing changes.
void Halo3SeatViewApply(Halo3SeatViewState& s)
{
    if (!s.relay)
        return;
    if (TitleAdapter_GetGeneration(GameTitle::Halo3) != s.generation)
    {
        // The module these sites were resolved in is gone or replaced: its bytes are not ours to touch. The relay
        // is released only if nothing can still be inside it; otherwise it is left (4 KiB) rather than risked.
        g_halo3SeatViewRedirected.store(false, std::memory_order_release);
        const bool quiet = Halo3SeatViewRelayQuiet(s);
        LOG("Halo 3 seat view: the title generation changed; forgetting the director's call sites without "
            "writing%s", quiet ? "" : " (the relay is left in place: a thread is inside it)");
        if (quiet)
            Halo3SeatViewReleaseRelay(s);
        s = {};
        return;
    }
    const bool want = !s.standDown && Halo3SeatViewWanted();
    bool all = true, any = false;
    for (const Halo3SeatViewSite& site : s.sites)
    {
        all = all && site.patched;
        any = any || site.patched;
    }
    if (want ? all : !any)
        return;
    if (!want)
        g_halo3SeatViewRedirected.store(false, std::memory_order_release);
    bool busy = false;
    for (size_t i = 0; i < seat_view::kDirectorSiteCount; ++i)
    {
        Halo3SeatViewSite& site = s.sites[i];
        if (site.patched == want)
            continue;
        const Halo3SeatViewSwap swap = want ? Halo3SeatViewSwapBytes(site.call, site.stock, site.redirect)
                                            : Halo3SeatViewSwapBytes(site.call, site.redirect, site.stock);
        if (swap == Halo3SeatViewSwap::Busy)
        {
            busy = true;
            continue;
        }
        if (swap == Halo3SeatViewSwap::Foreign)
        {
            // Someone else's bytes now: this site is not ours to write either way. Stand down for this module;
            // the next polls put back the sites that are still ours. The relay stays: a site may still call it.
            LOG("Halo 3 seat view: the call at +0x%X (%s) is no longer the one verified at install; the director "
                "keeps the game's own answer and seats keep the game's third-person view",
                seat_view::kHalo3DirectorSites[i].anchorRva + seat_view::kHalo3DirectorSites[i].callOffset,
                seat_view::kHalo3DirectorSites[i].what);
            g_halo3SeatViewRedirected.store(false, std::memory_order_release);
            site.patched = false;
            s.standDown = true;
            return;
        }
        site.patched = want;
    }
    if (busy)
    {
        if (!s.pendingLogged)
            LOG("Halo 3 seat view: redirecting the director's calls is pending (thread snapshot unavailable or a "
                "thread inside the bytes); retrying");
        s.pendingLogged = true;
        return;
    }
    s.pendingLogged = false;
    if (want)
    {
        g_halo3SeatViewRedirected.store(true, std::memory_order_release);
        LOG("Halo 3 seat view: the camera director is answered first person for the seat the mod's camera owns "
            "(its three calls at +0x212074, +0x2127EF and +0x212DCB go through a relay at %p). The vehicle's data, "
            "the game's own function and the simulation's call at +0x%X are untouched: shots, AI and every other "
            "player's game see a stock seat",
            static_cast<void*>(s.relay), seat_view::kHalo3EvaluatorCallRva);
    }
    else
        LOG("Halo 3 seat view: the director's calls are back to the game's own (%s)",
            !g_config.vehicle_first_person || !g_config.vehicle_hide_body
                ? "vehicle_first_person / vehicle_hide_body off"
                : "seat_view_tag_write = 1: the seat's tag flag is written instead (dev A/B)");
}
} // namespace

// The seat sampler's side (halo3_native_seat_patch.inl), camera thread.
bool Halo3SeatViewAvailable()
{
    return g_halo3SeatViewRedirected.load(std::memory_order_acquire);
}
void Halo3SeatViewArm(int32_t unit)
{
    g_halo3SeatViewUnit.store(unit, std::memory_order_release);
}
void Halo3SeatViewDisarm()
{
    g_halo3SeatViewUnit.store(-1, std::memory_order_release);
}
// DEV BUILDS ONLY: seat_view_tag_write = 1 puts back the write to the seat's tag flag, for an A/B in the headset.
// A player build never writes it.
bool Halo3SeatViewWritesTag()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    return g_config.seat_view_tag_write != 0;
#else
    return false;
#endif
}

// WaitThread, at title install: verify the three director sites and build the relay. False = the seat keeps the
// game's third-person view (and its drawn character); nothing was written.
bool Halo3SeatViewInstall(uintptr_t base, size_t size, uint32_t generation)
{
    Halo3SeatViewState& s = g_halo3SeatView;
    if (s.relay && s.base == base && s.generation == generation)
    {
        Halo3SeatViewApply(s);
        return true;
    }
    if (s.relay)
        Halo3SeatViewApply(s);   // another generation's: forgotten there
    if (s.relay)
        return false;            // still held by a thread inside it; tried again on the next install
    s = {};
    const char* why = nullptr;
    const seat_view::DirectorSite* failed = nullptr;
    const uintptr_t function = base + seat_view::kHalo3ThirdPersonRva;
    if (!base || !generation || seat_view::kHalo3ThirdPersonRva >= size)
        why = "the module is not loaded";
    for (size_t i = 0; !why && i < seat_view::kDirectorSiteCount; ++i)
    {
        const seat_view::DirectorSite& site = seat_view::kHalo3DirectorSites[i];
        uint8_t* const call = reinterpret_cast<uint8_t*>(base + site.anchorRva + site.callOffset);
        uintptr_t target = 0;
        failed = &site;
        if (site.anchorRva + site.callOffset + seat_view::kCallBytes > size)
            why = "a call site lies outside the module";
        else if (!SeatAimFrameUniqueAt(base, size, site.anchorRva, site.pattern))
            why = "the code around a call site is missing, ambiguous or moved";
        else if (!SeatAimFrameExecutable(call, seat_view::kCallBytes) || !Halo3SeatViewRead(call, s.sites[i].stock))
            why = "a call site is not in executable memory";
        else if (!seat_view::ReadCall(s.sites[i].stock, reinterpret_cast<uintptr_t>(call), target) ||
                 target != function)
            why = "a call site no longer calls the seat's third-person test";
        else
            s.sites[i].call = call;
    }
    if (!why)
    {
        failed = nullptr;
        s.relay = Halo3SeatViewAllocateNear(base + seat_view::kHalo3DirectorSites[0].anchorRva,
                                            kHalo3SeatViewRelayAllocation);
        if (!s.relay)
            why = "no memory within a call's reach of the module";
    }
    if (!why)
    {
        const size_t code = seat_view::BuildRelay(s.relay, kHalo3SeatViewUnwindInfoOffset, function,
            reinterpret_cast<uintptr_t>(&Halo3SeatViewAdjust), reinterpret_cast<uintptr_t>(&g_halo3SeatViewInside));
        const size_t unwind = seat_view::BuildRelayUnwind(s.relay + kHalo3SeatViewUnwindInfoOffset,
                                                          seat_view::kRelayUnwindBytes);
        bool built = code == seat_view::kRelayBytes && unwind == seat_view::kRelayUnwindBytes;
        for (size_t i = 0; built && i < seat_view::kDirectorSiteCount; ++i)
            built = seat_view::BuildCall(reinterpret_cast<uintptr_t>(s.sites[i].call),
                                         reinterpret_cast<uintptr_t>(s.relay), s.sites[i].redirect);
        if (built)
        {
            // The relay's function-table entry, in the same allocation and written while it is still writable.
            s.unwind = reinterpret_cast<RUNTIME_FUNCTION*>(s.relay + kHalo3SeatViewUnwindEntryOffset);
            s.unwind->BeginAddress = 0;
            s.unwind->EndAddress = static_cast<DWORD>(seat_view::kRelayBytes);
            s.unwind->UnwindData = static_cast<DWORD>(kHalo3SeatViewUnwindInfoOffset);
        }
        DWORD previous = 0;
        if (!built)
            why = "the relay could not be built within reach";
        else if (!VirtualProtect(s.relay, kHalo3SeatViewRelayAllocation, PAGE_EXECUTE_READ, &previous))
            why = "the relay's memory could not be made executable";
        else
        {
            FlushInstructionCache(GetCurrentProcess(), s.relay, kHalo3SeatViewRelayAllocation);
            // Registered so a stack walk or an exception passing through the relay unwinds it exactly.
            s.unwindRegistered = RtlAddFunctionTable(s.unwind, 1, reinterpret_cast<DWORD64>(s.relay)) != 0;
        }
        if (why)
            Halo3SeatViewReleaseRelay(s);
    }
    if (why)
    {
        if (failed)
            LOG("Halo 3 seat view: the director is NOT answered (%s: %s, +0x%X); a seat keeps the game's "
                "third-person view and its drawn character. Nothing was written",
                why, failed->what, failed->anchorRva + failed->callOffset);
        else
            LOG("Halo 3 seat view: the director is NOT answered (%s); a seat keeps the game's third-person view "
                "and its drawn character. Nothing was written", why);
        s = {};
        return false;
    }
    s.base = base;
    s.generation = generation;
    if (!Halo3SeatViewWanted())
        LOG("Halo 3 seat view: the director's three call sites are verified but not redirected (%s)",
            !g_config.vehicle_first_person || !g_config.vehicle_hide_body
                ? "vehicle_first_person / vehicle_hide_body off"
                : "seat_view_tag_write = 1: the seat's tag flag is written instead (dev A/B)");
    Halo3SeatViewApply(s);
    return true;
}

// WaitThread, at title teardown: the sites go back to the game's own calls (only over our own bytes), then the
// relay is released once nothing is inside it. False = try again.
bool Halo3SeatViewRemove()
{
    Halo3SeatViewState& s = g_halo3SeatView;
    Halo3SeatViewDisarm();
    g_halo3SeatViewRedirected.store(false, std::memory_order_release);
    if (!s.relay)
    {
        s = {};
        return true;
    }
    bool busy = false;
    for (size_t i = 0; i < seat_view::kDirectorSiteCount; ++i)
    {
        Halo3SeatViewSite& site = s.sites[i];
        if (!site.patched || !site.call)
            continue;
        const Halo3SeatViewSwap swap = Halo3SeatViewSwapBytes(site.call, site.redirect, site.stock);
        if (swap == Halo3SeatViewSwap::Busy)
            busy = true;
        else
            site.patched = false;   // restored, or no longer ours
    }
    const bool quiet = !busy && Halo3SeatViewRelayQuiet(s);
    if (!quiet && ++s.removeFailures < 40)
        return false;
    if (!quiet)
    {
        // Never release memory a thread may return into: the relay (4 KiB) outlives the module instead.
        LOG("Halo 3 seat view: the relay could not be retired at teardown (%s); left in place",
            busy ? "a call site could not be restored" : "a thread is still inside it");
        s = {};
        return true;
    }
    Halo3SeatViewReleaseRelay(s);
    LOG("Halo 3 seat view: the director's calls are restored and the relay released at teardown");
    s = {};
    return true;
}

// WaitThread poll: follow F1 / config changes, and (dev) say what the director was told.
void Halo3SeatViewSync()
{
    Halo3SeatViewApply(g_halo3SeatView);
#if HALOMCCVR_DEV_DIAGNOSTICS
    static uint64_t s_lastMs = 0;
    static uint32_t s_asked = 0, s_answered = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_lastMs < 5000)
        return;
    s_lastMs = now;
    const uint32_t asked = g_halo3SeatViewAsked.load(std::memory_order_relaxed);
    const uint32_t answered = g_halo3SeatViewAnswered.load(std::memory_order_relaxed);
    if (answered != s_answered)
        LOG("SEAT VIEW Halo 3 (dev): the director asked %u times in 5 s, %u answered first person for unit 0x%08X",
            asked - s_asked, answered - s_answered,
            static_cast<uint32_t>(g_halo3SeatViewUnit.load(std::memory_order_relaxed)));
    s_asked = asked;
    s_answered = answered;
#endif
}
