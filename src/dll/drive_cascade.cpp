// DRIVE CASCADE, the DLL's side (drive_cascade.h).
#include <windows.h>

#include <cstdio>
#include <cstring>

#include "drive_cascade.h"
#include "sigscan.h"
#include "../common/log.h"
#if HALOMCCVR_DEV_DIAGNOSTICS
#include "../common/config.h"   // drive_fault: dev builds only
#endif

namespace
{
bool Readable(DWORD protect)
{
    if (protect & (PAGE_GUARD | PAGE_NOACCESS))
        return false;
    const DWORD kind = protect & 0xFFu;
    return kind == PAGE_READONLY || kind == PAGE_READWRITE || kind == PAGE_WRITECOPY || kind == PAGE_EXECUTE_READ ||
           kind == PAGE_EXECUTE_READWRITE || kind == PAGE_EXECUTE_WRITECOPY;
}

// The committed, readable runs of [begin, end), region by region, appended to `runs` as RVAs of `base`. A title's
// mapped image may hold pages that cannot be read; a search must step over them, not fault on them. False when
// there are more runs than `capacity`.
bool ReadableRuns(uintptr_t base, uintptr_t begin, uintptr_t end, drive_resolve::Range* runs, uint32_t& count,
                  uint32_t capacity)
{
    uintptr_t runBegin = 0;
    const auto close = [&](uintptr_t at) {
        if (!runBegin)
            return true;
        if (count >= capacity)
            return false;
        runs[count++] = drive_resolve::Range{static_cast<uint32_t>(runBegin - base), static_cast<uint32_t>(at - runBegin)};
        runBegin = 0;
        return true;
    };
    for (uintptr_t at = begin; at < end;)
    {
        MEMORY_BASIC_INFORMATION info{};
        if (!VirtualQuery(reinterpret_cast<const void*>(at), &info, sizeof(info)) || !info.RegionSize)
            return close(at);
        const uintptr_t regionEnd = reinterpret_cast<uintptr_t>(info.BaseAddress) + info.RegionSize;
        if (regionEnd <= at)
            return close(at);
        if (info.State == MEM_COMMIT && Readable(info.Protect))
        {
            if (!runBegin)
                runBegin = at;
        }
        else if (!close(at))
        {
            return false;
        }
        at = regionEnd < end ? regionEnd : end;
    }
    return close(end);
}

bool WhollyReadable(uintptr_t begin, size_t size)
{
    for (uintptr_t at = begin, end = begin + size; at < end;)
    {
        MEMORY_BASIC_INFORMATION info{};
        if (!VirtualQuery(reinterpret_cast<const void*>(at), &info, sizeof(info)) || !info.RegionSize ||
            info.State != MEM_COMMIT || !Readable(info.Protect))
            return false;
        at = reinterpret_cast<uintptr_t>(info.BaseAddress) + info.RegionSize;
    }
    return true;
}

// The module as the resolution reads it. The caller guards this (the headers are read as they stand).
bool Open(uintptr_t base, size_t size, drive_resolve::Image& image, const char*& why)
{
    if (!base || !WhollyReadable(base, 0x1000))
    {
        why = "the module's headers are not readable";
        return false;
    }
    if (!drive_resolve::ReadImage(reinterpret_cast<const uint8_t*>(base), size, image, why))
        return false;
    // Search only what can be read of each executable section.
    drive_resolve::Range runs[drive_resolve::kMaxExecutableRanges]{};
    uint32_t count = 0;
    for (uint32_t i = 0; i < image.executableCount; ++i)
    {
        const uintptr_t begin = base + image.executable[i].rva;
        if (!ReadableRuns(base, begin, begin + image.executable[i].size, runs, count,
                          drive_resolve::kMaxExecutableRanges))
        {
            why = "the module's code is split into more readable pieces than a search holds";
            return false;
        }
    }
    if (!count)
    {
        why = "none of the module's code is readable";
        return false;
    }
    for (uint32_t i = 0; i < drive_resolve::kMaxExecutableRanges; ++i)
        image.executable[i] = i < count ? runs[i] : drive_resolve::Range{};
    image.executableCount = count;
    // Without a readable .pdata nothing that needs a function's start or extent resolves, and those drives are
    // not hooked.
    if (image.functions &&
        !WhollyReadable(reinterpret_cast<uintptr_t>(image.functions),
                        static_cast<size_t>(image.functionCount) * sizeof(drive_resolve::RuntimeFunction)))
    {
        image.functions = nullptr;
        image.functionCount = 0;
    }
    // The module's own TLS index, from its PE TLS directory.
    uint32_t index = 0;
    uintptr_t rva = 0;
    if (addrcascade::tls_index(reinterpret_cast<void*>(base), &index, &rva) && rva && rva < size &&
        sizeof(uint32_t) <= size - rva)
        image.tlsIndexRva = static_cast<uint32_t>(rva);
    // Every compare and every count is of the game's own bytes, whatever this mod has hooked by now.
    image.original = &sig::CopyUnhooked;
    image.correct = &sig::CorrectLiveCountForHooks;
    return true;
}

// The faults forced on a resolution. DEV BUILDS ONLY (config key drive_fault; drive_resolve.h: the kFault bits):
// each forces what only a game build the tables were not measured on does, so the rung below it, or the refusal,
// can be watched working in the running game. A player build has no key and none of the forcing code:
// drive_resolve::Fault and addrcascade::fault are constant false there.
uint32_t Faults()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    addrcascade::set_fault_mask(g_config.drive_fault);
    return static_cast<uint32_t>(addrcascade::fault_mask());
#else
    return 0;
#endif
}

// One entry of the announced list: what a line that is not "as recorded" says.
int DescribeLine(const drive_resolve::Line& line, char* out, size_t size)
{
    using drive_resolve::Verdict;
    switch (line.verdict)
    {
    case Verdict::Moved:
        return snprintf(out, size, "%s MOVED from +0x%X to +0x%X", line.name, line.recorded, line.found);
    case Verdict::RecordAmongSeveral:
        return snprintf(out, size, "%s taken at the recorded +0x%X among %u%s matches", line.name, line.found,
                        line.matches, line.matches >= drive_resolve::kCountCap ? " or more" : "");
    case Verdict::Ambiguous:
        return snprintf(out, size, "%s AMBIGUOUS (%u%s matches, none at the recorded +0x%X)", line.name, line.matches,
                        line.matches >= drive_resolve::kCountCap ? " or more" : "", line.recorded);
    case Verdict::Refused:
        return snprintf(out, size, "%s REFUSED (%s)", line.name, line.note ? line.note : "a proof failed");
    default:
        return snprintf(out, size, "%s %s", line.name, drive_resolve::VerdictText(line.verdict));
    }
}
} // namespace

namespace drive_cascade
{
bool RunPlan(uintptr_t base, size_t size, PlanThunk plan, void* context, drive_resolve::Report& report,
             const char*& why)
{
    static bool s_sinkSet = false;
    if (!s_sinkSet)
    {
        s_sinkSet = true;
        addrcascade::set_logger([](const char* message) { LOG("%s", message); });
    }
    report = drive_resolve::Report{};
    why = nullptr;
    __try
    {
        drive_resolve::Image image;
        if (!Open(base, size, image, why))
            return false;
        drive_resolve::Resolver resolver(image, Faults());
        plan(resolver, context);
        report = resolver.report();
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        report = drive_resolve::Report{};
        why = "a read of the module faulted";
        return false;
    }
}

void FirstFailure(const drive_resolve::Report& report, char* out, size_t size)
{
    if (!out || !size)
        return;
    for (uint32_t i = 0; i < report.count; ++i)
        if (!drive_resolve::Found(report.lines[i].verdict))
        {
            (void)DescribeLine(report.lines[i], out, size);
            return;
        }
    snprintf(out, size, "nothing");
}

void Announce(const char* drive, const drive_resolve::Report& report, addrcascade::TierReporter& reporter)
{
    using drive_resolve::Verdict;
    const uint32_t recorded = report.Count(Verdict::AsRecorded), moved = report.Count(Verdict::Moved),
                   several = report.Count(Verdict::RecordAmongSeveral), lost = report.NotFound();
    char tier[32];
    snprintf(tier, sizeof(tier), "%u/%u/%u/%u", recorded, moved, several, lost);
    if (!reporter.changed(static_cast<uintptr_t>(report.Fingerprint()), tier))
        return;
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (const int forced = addrcascade::fault_mask())
        LOG("%s: DEV fault injection (drive_fault 0x%X) is forcing what the next line reports; it says nothing about "
            "this game build", drive, static_cast<unsigned>(forced));
#endif
    if (!moved && !several && !lost)
    {
        LOG("%s: all %u of its addresses were found by search where the tables record them", drive, recorded);
        return;
    }
    // Name the ones that are not as recorded, as many as fit.
    char list[720] = "";
    size_t used = 0;
    uint32_t named = 0, unnamed = 0;
    for (uint32_t i = 0; i < report.count; ++i)
    {
        const drive_resolve::Line& line = report.lines[i];
        if (line.verdict == Verdict::AsRecorded)
            continue;
        char one[200];
        const int length = DescribeLine(line, one, sizeof(one));
        if (length <= 0 || used + static_cast<size_t>(length) + 3 >= sizeof(list) - 24)
        {
            ++unnamed;
            continue;
        }
        used += static_cast<size_t>(snprintf(list + used, sizeof(list) - used, "%s%s", named ? "; " : "", one));
        ++named;
    }
    if (unnamed)
        snprintf(list + used, sizeof(list) - used, "; and %u more", unnamed);
    LOG("%s: of its %u addresses, found by search, %u are where the tables record them, %u MOVED, %u were taken at the "
        "recorded address among several matches, %u were NOT found: %s",
        drive, report.count, recorded, moved, several, lost, list);
}

void HookWatch::Restart()
{
    dog_.reset();
    periodStartMs_ = 0;
    periodPossible_ = false;
    everCalled_ = false;
}

bool HookWatch::Pass(const void* target, bool possible, bool called, uint64_t nowMs)
{
    if (target != target_)
    {
        Restart();
        target_ = target;
    }
    if (!target)
        return false;
    everCalled_ = everCalled_ || called;
    if (!periodStartMs_)
    {
        periodStartMs_ = nowMs;
        periodPossible_ = true;
    }
    // A period counts only if every pass in it found the call possible. However long a pass was in coming, the
    // time between two passes is one period at most: a stalled worker saw nothing in between.
    periodPossible_ = periodPossible_ && possible;
    if (nowMs - periodStartMs_ < kPeriodMs)
        return false;
    const bool wholePeriod = periodPossible_;
    periodStartMs_ = nowMs;
    periodPossible_ = true;
    // DEV fault injection (drive_fault 0x10; constant false in a player build): the watchdog is told the hook
    // never ran, so its one line can be watched appearing after the possible time it waits for.
    const bool silenced = addrcascade::fault(static_cast<int>(drive_resolve::kFaultHookSilent));
    return dog_.tick(wholePeriod, everCalled_ && !silenced);
}

void SayNeverCalled(const char* drive, const char* what, uint32_t rva, const char* state)
{
    LOG("%s: INSTALLED BUT NEVER CALLED. The hook on %s at +0x%X has stood through %u s in which %s, and the "
        "game has not called it once. Either that address is not the code this game build runs for it, or the "
        "game was not in that state after all. Nothing is written through a hook that does not run: the aim "
        "stays on the stick servo",
        drive, what, rva, HookWatch::kSeconds, state);
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (addrcascade::fault(static_cast<int>(drive_resolve::kFaultHookSilent)))
        LOG("%s: DEV fault injection (drive_fault 0x%X) told that watchdog its hook never ran; the line above says "
            "nothing about this game build", drive, static_cast<unsigned>(addrcascade::fault_mask()));
#endif
}
} // namespace drive_cascade
