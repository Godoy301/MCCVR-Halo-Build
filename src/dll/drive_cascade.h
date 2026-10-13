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
        Plan* plan;
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
// address was the right one is the hook being called. One HookWatch per hook, stepped by the cold worker from the
// drive's own two-second report. It counts only periods in which the call was POSSIBLE -- the drive's own word
// that the game is in the state its hook serves -- and starts again from nothing whenever it was not, so a menu,
// a load or a cutscene never counts against a healthy address. Log-only: a drive whose hook does not run already
// leaves the aim to the stick servo through its heartbeat.
class HookWatch
{
public:
    // The periods of possible, uncalled time after which the hook is called dead: ~10 s at the two seconds per
    // step the drives' reports run at.
    static constexpr uint32_t kPeriods = 5;
    static constexpr uint32_t kPeriodSeconds = 2;

    // A new install (the hook is on other code now): the watch starts again.
    void Restart();

    // `installed`: the hook exists. `possible`: the game was, through this whole period, in the state in which it
    // calls the hooked function. `calls`: how many calls the hook counted in the period. True exactly once: the
    // hook has been installed through kPeriods possible periods in a row and has never been called.
    bool Step(bool installed, bool possible, uint32_t calls);

    uint32_t SecondsCounted() const { return dog_.counted() * kPeriodSeconds; }
    bool everCalled() const { return everCalled_; }

private:
    addrcascade::HookWatchdog dog_{kPeriods};
    bool everCalled_ = false;
};
} // namespace drive_cascade
