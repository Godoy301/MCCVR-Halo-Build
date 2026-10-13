// DRIVE CASCADE, the DLL's side (drive_cascade.h).
#include <windows.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <cwchar>

#include "drive_cascade.h"
#include "sigscan.h"
#include "../common/config.h"
#include "../common/log.h"
#include "../common/title_build_identity.h"

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

// The module's own TLS index, from its PE TLS directory. Guarded on its own: a module whose directory cannot be
// read loses the drives that need the index, not the one that does not (Halo 2's).
void ReadTlsIndex(uintptr_t base, size_t size, drive_resolve::Image& image)
{
    __try
    {
        uint32_t index = 0;
        uintptr_t rva = 0;
        if (addrcascade::tls_index(reinterpret_cast<void*>(base), &index, &rva) && rva && rva < size &&
            sizeof(uint32_t) <= size - rva)
            image.tlsIndexRva = static_cast<uint32_t>(rva);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        image.tlsIndexRva = 0;
    }
}

// The module as the resolution reads it. The caller guards this (the headers are read as they stand).
bool Open(uintptr_t base, size_t size, drive_resolve::Image& image, const char*& why)
{
    if (!base || !WhollyReadable(base, 0x1000))
    {
        why = "the module's headers are not readable";
        return false;
    }
    // Outside the readable runs found below, every read asks the system first (WhollyReadable): a page that
    // cannot be read costs the line that needed it, and the rest of the drive's resolution stands. The function
    // table is asked about before any of it is read: one that cannot be read is no table, so nothing that needs
    // a function's start or extent resolves, and the drives that need one are not hooked.
    if (!drive_resolve::ReadImage(reinterpret_cast<const uint8_t*>(base), size, image, why, &WhollyReadable))
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
    ReadTlsIndex(base, size, image);
    // Every compare and every count is of the game's own bytes, whatever this mod has hooked by now.
    image.original = &sig::CopyUnhooked;
    image.correct = &sig::CorrectLiveCountForHooks;
    return true;
}

bool GuardedOpen(uintptr_t base, size_t size, drive_resolve::Image& image, const char*& why)
{
    __try
    {
        return Open(base, size, image, why);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        why = "a read of the module faulted";
        return false;
    }
}

bool GuardedRun(const drive_resolve::Image& image, uint32_t faults, bool recordedBuild, const drive_known::Entry* saved,
                drive_known::PlanFn plan, drive_known::KeepFn keep, void* context, drive_resolve::Report& report,
                drive_known::Outcome& outcome, const char*& why)
{
    __try
    {
        outcome = drive_known::Run(image, faults, recordedBuild, saved, plan, keep, context, report);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        why = "a read of the module faulted";
        return false;
    }
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

// ------------------------------------------------------------------------------------------ the cache's file
// Read and written where the drives are installed, on the cold worker. The lock is for a title whose install
// runs on a worker of its own; the text buffer and the parsed file belong to whoever holds it.
wchar_t g_cachePath[MAX_PATH] = L"";
SRWLOCK g_cacheLock = SRWLOCK_INIT;
char g_cacheText[drive_known::kMaxFileText];
drive_known::File g_cacheFile;

// Reads the file into g_cacheFile. What is missing or cannot be read holds nothing: a cold start.
void ReadCacheLocked()
{
    g_cacheFile = drive_known::File{};
    const HANDLE handle = CreateFileW(g_cachePath, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_DELETE, nullptr,
                                      OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
        return;
    DWORD read = 0;
    const BOOL ok = ReadFile(handle, g_cacheText, sizeof(g_cacheText), &read, nullptr);
    CloseHandle(handle);
    // A file that fills the buffer is larger than any this mod writes: it is not this mod's.
    if (ok && read < sizeof(g_cacheText))
        drive_known::Parse(g_cacheText, read, g_cacheFile);
}

// Writes g_cacheFile. NEVER IN PLACE: the text goes to a temporary file beside the cache, which then takes its
// place in one step. Until that step the old file is whole, and a write that fails leaves it so.
bool WriteCacheLocked()
{
    const size_t length = drive_known::Format(g_cacheFile, g_cacheText, sizeof(g_cacheText));
    wchar_t temporary[MAX_PATH + 8];
    if (!length || _snwprintf_s(temporary, _TRUNCATE, L"%ls.tmp", g_cachePath) < 0)
        return false;
    const HANDLE handle =
        CreateFileW(temporary, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
        return false;
    DWORD written = 0;
    const bool whole = WriteFile(handle, g_cacheText, static_cast<DWORD>(length), &written, nullptr) &&
                       written == length && FlushFileBuffers(handle);
    CloseHandle(handle);
    if (whole && MoveFileExW(temporary, g_cachePath, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        return true;
    DeleteFileW(temporary);
    return false;
}

// This drive's saved addresses for exactly this build of its module, found under the rules this mod searches by.
// `refused`: some were held, and are of another build or of other rules.
bool LookUp(const char* module, const char* id, uint64_t stamp, drive_known::Entry& entry, const char*& refused)
{
    AcquireSRWLockExclusive(&g_cacheLock);
    ReadCacheLocked();
    const drive_known::Entry* const usable = drive_known::Usable(g_cacheFile, module, id, stamp, refused);
    if (usable)
        entry = *usable;
    ReleaseSRWLockExclusive(&g_cacheLock);
    return usable != nullptr;
}

// Puts one drive's addresses into the file. The file is read again first, so every other drive's and every other
// title's entry is kept (a file that is full lets its oldest dead entry give way: drive_known::Put). False when
// the file could not be written.
bool Store(const drive_known::Entry& entry)
{
    AcquireSRWLockExclusive(&g_cacheLock);
    ReadCacheLocked();
    drive_known::Put(g_cacheFile, entry);
    const bool stored = WriteCacheLocked();
    ReleaseSRWLockExclusive(&g_cacheLock);
    return stored;
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
void SetCacheFile(const wchar_t* path)
{
    AcquireSRWLockExclusive(&g_cacheLock);
    if (!path || _snwprintf_s(g_cachePath, _TRUNCATE, L"%ls", path) < 0)
        g_cachePath[0] = 0;
    ReleaseSRWLockExclusive(&g_cacheLock);
}

void SetCacheFolder(const wchar_t* folder)
{
    wchar_t path[MAX_PATH];
    if (!folder || _snwprintf_s(path, _TRUNCATE, L"%lsHaloMCCVR-drive-addresses.txt", folder) < 0)
        path[0] = 0;
    SetCacheFile(path);
}

bool RunPlan(uintptr_t base, size_t size, const Drive& drive, drive_known::PlanFn plan, drive_known::KeepFn keep,
             void* context, drive_resolve::Report& report, Rungs& rungs, const char*& why)
{
    // Installs run on more than one worker: the library's log sink is set by exactly one of them.
    static std::atomic<bool> s_sinkSet{false};
    if (!s_sinkSet.exchange(true))
        addrcascade::set_logger([](const char* message) { LOG("%s", message); });
    report = drive_resolve::Report{};
    rungs = Rungs{};
    why = nullptr;
    drive_resolve::Image image;
    if (!GuardedOpen(base, size, image, why))
        return false;
    const uint32_t faults = Faults();
    // While ANY bit of the dev key is set the player's file is neither read nor written. What a forced fault
    // makes of a drive is not what the game DLL holds and must never be found there by a later launch; and a
    // bit that forces nothing (the self-test, the silenced watchdog) still marks a launch whose result nobody
    // meant to keep.
    const bool cacheOn = !faults && g_config.drive_address_cache != 0 && g_cachePath[0] != 0;
    const char* const module = drive_known::ModuleName(drive.title);
    const uint64_t stamp = drive_known::Stamp(image);
    rungs.recordedBuild =
        title_build::Compare(drive.title, image.timeDateStamp, image.sizeOfImage) == title_build::Match::Known;
    drive_known::Entry entry;
    const char* notUsable = nullptr;
    const bool have = cacheOn && drive.id && LookUp(module, drive.id, stamp, entry, notUsable);
    if (!GuardedRun(image, faults, rungs.recordedBuild, have ? &entry : nullptr, plan, keep, context, report,
                    rungs.outcome, why))
    {
        report = drive_resolve::Report{};
        rungs = Rungs{};
        return false;
    }
    rungs.savedRefused = rungs.outcome.savedRefused ? rungs.outcome.savedRefused : notUsable;
    // LEARN AND SAVE: what the search found on a build the tables do not describe is the next launch's rung 1.
    if (rungs.outcome.learn)
    {
#if HALOMCCVR_DEV_DIAGNOSTICS
        if (faults)
            rungs.notSaved = "drive_fault is set: a launch with the dev key in it neither reads nor writes the file";
        else
#endif
        if (!g_config.drive_address_cache)
            rungs.notSaved = "drive_address_cache = 0";
        else if (!g_cachePath[0])
            rungs.notSaved = "the mod's folder is not known";
        else if (!drive_known::MakeEntry(entry, module, drive.id, stamp, report))
            rungs.notSaved = "this drive has no name to save them under";
        else if (!(rungs.saved = Store(entry)))
            rungs.notSaved = "the file could not be written";
    }
    return true;
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

void Announce(const char* drive, const drive_resolve::Report& report, const Rungs& rungs,
              addrcascade::TierReporter& reporter)
{
    using drive_known::Rung;
    using drive_resolve::Verdict;
    const drive_known::Outcome& outcome = rungs.outcome;
    const uint32_t recorded = report.Count(Verdict::AsRecorded), moved = report.Count(Verdict::Moved),
                   several = report.Count(Verdict::RecordAmongSeveral), lost = report.NotFound();
    char tier[32];
    snprintf(tier, sizeof(tier), "%u %u/%u/%u/%u %u%u%u%u", static_cast<unsigned>(outcome.rung), recorded, moved, several,
             lost, rungs.saved ? 1u : 0u, outcome.searchedToo ? (outcome.searchAgrees ? 1u : 2u) : 0u,
             outcome.recordedTried ? 1u : 0u, rungs.savedRefused ? 1u : 0u);
    if (!reporter.changed(static_cast<uintptr_t>(report.Fingerprint()), tier))
        return;
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (const int forced = addrcascade::fault_mask() & static_cast<int>(drive_resolve::kFaultsThatForce))
        LOG("%s: DEV fault injection (drive_fault 0x%X) is forcing what the next line reports; it says nothing about "
            "this game build", drive, static_cast<unsigned>(forced));
#endif
    if (outcome.rung == Rung::Recorded)
    {
        LOG("%s: verified at the recorded addresses, all %u of them, with no search (this game DLL is the build the "
            "tables were measured on)", drive, report.count);
    }
    else if (outcome.rung == Rung::Saved)
    {
        // A line saved as the single match in its function has had that function searched again: the rule it
        // was placed by, applied whole (drive_resolve.h).
        if (report.scans)
            LOG("%s: verified at the addresses saved from an earlier launch of this game build, all %u of them, with "
                "no search of the module (%u of them are not where the tables record them; %u of its functions were "
                "searched again, each for a line saved as the single match in it)",
                drive, report.count, moved, report.scans);
        else
            LOG("%s: verified at the addresses saved from an earlier launch of this game build, all %u of them, with "
                "no search (%u of them are not where the tables record them)", drive, report.count, moved);
    }
    else
    {
        // What the known addresses came to first.
        char before[240] = "";
        size_t length = 0;
        if (outcome.recordedTried)
            length += static_cast<size_t>(snprintf(before, sizeof(before), "the recorded addresses did not all verify "
                                                                          "in this game DLL; "));
        if (rungs.savedRefused && length < sizeof(before))
            snprintf(before + length, sizeof(before) - length, "the addresses saved earlier were not used (%s); ",
                     rungs.savedRefused);
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
            const int text = DescribeLine(line, one, sizeof(one));
            if (text <= 0 || used + static_cast<size_t>(text) + 3 >= sizeof(list) - 24)
            {
                ++unnamed;
                continue;
            }
            used += static_cast<size_t>(snprintf(list + used, sizeof(list) - used, "%s%s", named ? "; " : "", one));
            ++named;
        }
        if (unnamed)
            snprintf(list + used, sizeof(list) - used, "; and %u more", unnamed);
        if (lost)
        {
            LOG("%s: %ssearched: of its %u addresses %u are where the tables record them, %u MOVED, %u were taken at "
                "the recorded address among several matches, %u were NOT found: %s",
                drive, before, report.count, recorded, moved, several, lost, list);
        }
        else
        {
            char saving[160];
            if (rungs.saved)
                snprintf(saving, sizeof(saving), "saved for the next launch");
            else if (rungs.notSaved)
                snprintf(saving, sizeof(saving), "NOT saved for the next launch (%s)", rungs.notSaved);
            else
                snprintf(saving, sizeof(saving), "nothing to save");
            LOG("%s: %sfound by search: of its %u addresses %u are where the tables record them, %u MOVED, %u were "
                "taken at the recorded address among several matches%s%s; %s",
                drive, before, report.count, recorded, moved, several, list[0] ? ": " : "", list, saving);
        }
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (outcome.searchedToo)
        LOG("%s: DEV self-test (drive_fault 0x%X): the search was run as well, and it %s", drive,
            static_cast<unsigned>(drive_resolve::kFaultSearchToo),
            outcome.searchAgrees ? "chose the same address for every line" : "DISAGREES with the verified addresses");
#endif
}

void HookWatch::Restart()
{
    dog_.reset();
    periodStartMs_ = 0;
    periodPossible_ = false;
    everCalled_ = false;
}

bool HookWatch::Pass(uint64_t install, bool possible, bool called, uint64_t nowMs)
{
    if (install != install_)
    {
        Restart();
        install_ = install;
    }
    if (!install)
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
