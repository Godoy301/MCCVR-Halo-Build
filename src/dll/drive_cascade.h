#pragma once
// DRIVE CASCADE, the DLL's side: opens a loaded game module for the direct drives' address resolution
// (src/common/drive_resolve.h, src/common/drive_sites.h), runs a drive's resolution on it under a structured-exception
// guard, says once how the addresses were found, and watches a hook that is installed for calls that never come.
//
// A drive's install does this, and nothing else decides where it hooks or reads:
//
//     drive_cascade::Resolution<drive_sites::ReachSabre> found;
//     if (!drive_cascade::Resolve(base, size, found, [](drive_resolve::Resolver& r) { return ResolveReachSabre(r); }))
//         ... not hooked: found.why, or the report's first line that was not found ...
//     drive_cascade::Announce("Reach Sabre direct drive", found.report, reporter);
//
// Every address the install then uses is one the resolution FOUND in this module; the recorded RVAs of the tables
// are only what it is compared with.
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "../common/addrcascade/AddressCascade.hpp"
#include "../common/drive_resolve.h"

namespace drive_cascade
{
using PlanThunk = void (*)(drive_resolve::Resolver& resolver, void* context);

// Opens the module at `base` (its headers, the readable runs of its executable sections, its .pdata, the TLS
// index its PE TLS directory names; every byte read as the game has it, through this mod's own enabled hooks)
// and runs `plan` on it. False when the module could not be opened or a read faulted: `why` says which, the
// report is empty, and the drive must not be hooked.
bool RunPlan(uintptr_t base, size_t size, PlanThunk plan, void* context, drive_resolve::Report& report,
             const char*& why);

// What one drive's resolution produced.
template <typename Found>
struct Resolution
{
    Found found{};
    drive_resolve::Report report{};
    const char* why = nullptr;   // the module did not open, or a read faulted
};

// Runs `plan` (Found (drive_resolve::Resolver&)) on the module. False: nothing was resolved (out.why).
template <typename Found, typename Plan>
bool Resolve(uintptr_t base, size_t size, Resolution<Found>& out, Plan&& plan)
{
    struct Context
    {
        std::remove_reference_t<Plan>* plan;
        Found* found;
    } context{&plan, &out.found};
    return RunPlan(base, size,
                   [](drive_resolve::Resolver& resolver, void* raw) {
                       auto* c = static_cast<Context*>(raw);
                       *c->found = (*c->plan)(resolver);
                   },
                   &context, out.report, out.why);
}

// The first line of a report that was not found, as "<name>: <verdict>[: <the proof that failed>]" -- what a
// drive names when it is not hooked. "nothing" when every line was found.
void FirstFailure(const drive_resolve::Report& report, char* out, size_t size);

// WHICH RUNG WON. One line per drive saying how its addresses were found -- all where the tables record them, or
// which moved (from where to where), which were taken at the recorded address among several matches, which were
// not found -- and it is said again only when that answer changes (another build of the game DLL, another rung):
// a line that always appears carries no information. `drive`: the drive's name as its own log lines begin.
void Announce(const char* drive, const drive_resolve::Report& report, addrcascade::TierReporter& reporter);

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
