#pragma once
// DRIVE CASCADE, the DLL's side: opens a loaded game module for the direct drives' address resolution
// (src/common/drive_resolve.h, drive_known.h, drive_sites.h), runs a drive's resolution on it by the rungs under a
// structured-exception guard, keeps the address cache's file, says once which rung won, and watches a hook that
// is installed for calls that never come.
//
// A drive's install does this, and nothing else decides where it hooks or reads:
//
//     drive_cascade::Resolution<drive_sites::ReachSabre> found;
//     if (!drive_cascade::Resolve(base, size, {GameTitle::HaloReach, "pilot-controller"}, found,
//                                 [](drive_resolve::Resolver& r) { return ResolveReachSabre(r); }))
//         ... not hooked: found.why, or the report's first line that was not found ...
//     drive_cascade::Announce("Reach Sabre direct drive", found, reporter);
//
// THE RUNGS (drive_known.h): the known addresses first, each verified where it stands with no search of the
// module -- the tables' recorded RVAs when the module is the build they were measured on, otherwise the RVAs a
// search of this same build saved -- and only when one of them fails, the search. What the search finds on a
// build the tables do not describe is saved, so the next launch starts from it. Every address the install then
// uses is one that was verified or found in THIS module.
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "../common/addrcascade/AddressCascade.hpp"
#include "../common/drive_known.h"
#include "../common/drive_resolve.h"
#include "../common/runtime_types.h"

namespace drive_cascade
{
// Which drive: the title whose game DLL it is in, and its name in the address cache (one word).
struct Drive
{
    GameTitle title;
    const char* id;
};

// What a resolution went through, beyond its report.
struct Rungs
{
    drive_known::Outcome outcome{};
    bool recordedBuild = false;          // the module is the build the drive's tables were measured on
    const char* savedRefused = nullptr;  // saved addresses were there for this drive and were not used: why
    bool saved = false;                  // what the search found was written to the cache
    const char* notSaved = nullptr;      // ... or why it was not, when it was worth saving
};

// THE ADDRESS CACHE'S FILE: HaloMCCVR-drive-addresses.txt in the mod's own folder, beside its log and its
// settings. `folder` ends with a separator. Until this is called there is no file and nothing is read or saved.
// It is a cache and not a setting: written by the mod, verified before every use, safe to delete. The setting
// drive_address_cache = 0 turns both the reading and the writing off.
void SetCacheFolder(const wchar_t* folder);
// The same for a file named outright (tests).
void SetCacheFile(const wchar_t* path);

// Opens the module at `base` (its headers, the readable runs of its executable sections, its .pdata, the TLS
// index its PE TLS directory names; every byte read as the game has it, through this mod's own enabled hooks)
// and runs `plan` on it by the rungs. `keep` is called for the run whose result is the one in use. False when
// the module could not be opened or a read faulted: `why` says which, the report is empty, and the drive must
// not be hooked. Reads and writes the cache's file: call it only where a file may be read (the installs' own
// cold worker thread), never from a detour.
bool RunPlan(uintptr_t base, size_t size, const Drive& drive, drive_known::PlanFn plan, drive_known::KeepFn keep,
             void* context, drive_resolve::Report& report, Rungs& rungs, const char*& why);

// What one drive's resolution produced.
template <typename Found>
struct Resolution
{
    Found found{};
    drive_resolve::Report report{};
    Rungs rungs{};
    const char* why = nullptr;   // the module did not open, or a read faulted
};

// Runs `plan` (Found (drive_resolve::Resolver&)) on the module. False: nothing was resolved (out.why).
template <typename Found, typename Plan>
bool Resolve(uintptr_t base, size_t size, const Drive& drive, Resolution<Found>& out, Plan&& plan)
{
    struct Context
    {
        std::remove_reference_t<Plan>* plan;
        Found aside;
        Found* found;
    } context{&plan, Found{}, &out.found};
    return RunPlan(
        base, size, drive,
        [](drive_resolve::Resolver& resolver, void* raw) {
            auto* c = static_cast<Context*>(raw);
            c->aside = (*c->plan)(resolver);
        },
        [](void* raw) {
            auto* c = static_cast<Context*>(raw);
            *c->found = c->aside;
        },
        &context, out.report, out.rungs, out.why);
}

// The first line of a report that was not found, as "<name>: <verdict>[: <the proof that failed>]" -- what a
// drive names when it is not hooked. "nothing" when every line was found.
void FirstFailure(const drive_resolve::Report& report, char* out, size_t size);

// WHICH RUNG WON. One line per drive: verified at the recorded addresses with no search; verified at the
// addresses saved from an earlier launch of this build; found by search (how many moved, and whether they were
// saved for the next launch); or which addresses were not found. It is said again only when that answer changes
// (another build of the game DLL, another rung): a line that always appears carries no information. `drive`: the
// drive's name as its own log lines begin.
void Announce(const char* drive, const drive_resolve::Report& report, const Rungs& rungs,
              addrcascade::TierReporter& reporter);

template <typename Found>
void Announce(const char* drive, const Resolution<Found>& resolution, addrcascade::TierReporter& reporter)
{
    Announce(drive, resolution.report, resolution.rungs, reporter);
}

// "INSTALLED" IS NOT "RUNNING". MinHook installs on any address and reports success; the only proof that an
// address was the right one is the hook being called. One HookWatch per hook, passed over by the cold worker on
// every one of its loops, from the drive's own report. It counts only periods in which the call was POSSIBLE at
// every pass -- the drive's own word, from something its hook does not produce, that the game is in the state the
// hooked code serves -- and starts again from nothing whenever it was not, so a menu, a load or a cutscene never
// counts against a healthy address. Log-only: a drive whose hook does not run already leaves the aim to the stick
// servo through its heartbeat.
class HookWatch
{
public:
    // The possible, uncalled time after which the hook is called dead: kPeriods periods of kPeriodMs in a row.
    static constexpr uint32_t kPeriods = 5;
    static constexpr uint32_t kPeriodMs = 2000;
    static constexpr uint32_t kSeconds = kPeriods * kPeriodMs / 1000;

    // One pass of the cold worker. `target`: where the hook is installed (null: it is not; another address than
    // at the last pass: a new install, and the watch starts again). `possible`: at this pass the game is in the
    // state in which it calls the hooked code. `called`: the hook's own counters show a call. True exactly once
    // per install: the hook has stood through kPeriods periods in a row, every pass of which found its call
    // possible, and it has never been called.
    bool Pass(const void* target, bool possible, bool called, uint64_t nowMs);

    bool everCalled() const { return everCalled_; }

private:
    void Restart();

    addrcascade::HookWatchdog dog_{kPeriods};
    const void* target_ = nullptr;
    uint64_t periodStartMs_ = 0;
    bool periodPossible_ = false;
    bool everCalled_ = false;
};

// The one line a watch writes when it fires. `drive`: the drive's name as its own log lines begin (it names the
// title). `what`: the hooked code. `rva`: where in the title's module the hook is. `state`: the state the game
// was seen in all that time, in which it calls that code.
void SayNeverCalled(const char* drive, const char* what, uint32_t rva, const char* state);
} // namespace drive_cascade
