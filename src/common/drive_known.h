#pragma once
// DRIVE ADDRESSES, THE KNOWN ONES FIRST -- the rungs of one direct drive's resolution, in order, and the small
// file that remembers what a search found.
//
//   RUNG 1, THE KNOWN ADDRESSES, VERIFIED IN PLACE. No search of the module at all.
//     (a) The tables' recorded RVAs, when the module IS the build they were measured on (its PE timestamp and
//         image size; title_build_identity.h).
//     (b) Otherwise the RVAs an earlier search found in exactly this build, read back from the address cache.
//     Every signature's bytes must stand at its known address, and the drive's own plan makes every proof on
//     those addresses that it makes on searched ones (drive_resolve.h has what that is, and why leaving out the
//     count of matches is sound). ONE line that fails, and the whole drive goes to rung 2: the two are never
//     mixed.
//   RUNG 2, THE SEARCH (drive_resolve.h), as if nothing were known.
//   LEARN. When the search resolved the whole drive and the answer is not simply "the recorded addresses, on the
//     recorded build", the addresses are worth remembering: the caller saves them, and the next launch on this
//     build starts at rung 1 (b).
//
// THE ADDRESS CACHE is a hint, never a setting and never a shortcut past validation. An entry is one drive's
// addresses for one module build (the stamp: PE TimeDateStamp and SizeOfImage, the pair a debugger matches a PDB
// by) and one version of that drive's tables (the plan hash: every name, recorded RVA and pattern the drive asks
// for). A patched game has another stamp and an updated mod another plan, so either makes an entry dead by
// construction; and an entry that is used is still verified byte for byte, so a wrong one costs a few comparisons
// and falls through to the search. Deleting the file is a cold start, nothing more.
//
// This file is the engine-free part: the rungs, and the cache's text. It reads and writes no file (the DLL's side
// does, src/dll/drive_cascade.cpp; the tests and the offline check use the same text functions).
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "drive_resolve.h"
#include "runtime_types.h"

namespace drive_known
{
using drive_resolve::Image;
using drive_resolve::Report;
using drive_resolve::Resolver;

// ------------------------------------------------------------------------------------------------- the cache
inline constexpr uint32_t kSchema = 1;
inline constexpr uint32_t kMaxModule = 24;
inline constexpr uint32_t kMaxId = 32;
inline constexpr uint32_t kMaxEntries = 16;   // one per drive of a module; there are nine drives

// One drive's addresses, as a search found them in one build of its module.
struct Entry
{
    char module[kMaxModule]{};   // "halo3.dll"
    char id[kMaxId]{};           // the drive, without spaces: "control-update"
    uint64_t stamp = 0;          // the module build they are of
    uint64_t plan = 0;           // the version of the drive's tables they answer (Report::plan)
    uint32_t count = 0;
    uint32_t rvas[drive_resolve::kMaxLines]{};   // every line's address, in the plan's order
};

struct File
{
    Entry entries[kMaxEntries]{};
    uint32_t count = 0;
};

// WHICH BUILD: the PE TimeDateStamp and SizeOfImage as one number.
inline uint64_t Stamp(const Image& image)
{
    return (static_cast<uint64_t>(image.timeDateStamp) << 32) | image.sizeOfImage;
}

inline const char* ModuleName(GameTitle title)
{
    switch (title)
    {
    case GameTitle::HaloCE: return "halo1.dll";
    case GameTitle::Halo2: return "halo2.dll";
    case GameTitle::Halo3: return "halo3.dll";
    case GameTitle::Halo3ODST: return "halo3odst.dll";
    case GameTitle::HaloReach: return "haloreach.dll";
    case GameTitle::Halo4: return "halo4.dll";
    default: return "";
    }
}

// A module or drive name as the file holds it: one word.
inline bool Word(const char* text, size_t capacity)
{
    if (!text || !*text)
        return false;
    size_t length = 0;
    for (const char* p = text; *p; ++p, ++length)
        if (*p <= ' ' || *p > '~' || *p == ',')
            return false;
    return length < capacity;
}

// A whole resolution as an entry. False when it is not one that may be saved (a line missing, a name that is not
// a word).
inline bool MakeEntry(Entry& entry, const char* module, const char* id, uint64_t stamp, const Report& report)
{
    entry = Entry{};
    if (!report.Complete() || !stamp || !Word(module, kMaxModule) || !Word(id, kMaxId))
        return false;
    std::snprintf(entry.module, sizeof(entry.module), "%s", module);
    std::snprintf(entry.id, sizeof(entry.id), "%s", id);
    entry.stamp = stamp;
    entry.plan = report.plan;
    entry.count = report.count;
    for (uint32_t i = 0; i < report.count; ++i)
        entry.rvas[i] = report.lines[i].found;
    return true;
}

// The entry held for a drive of a module, whatever build it is of, or null.
inline const Entry* Find(const File& file, const char* module, const char* id)
{
    for (uint32_t i = 0; i < file.count; ++i)
        if (!std::strcmp(file.entries[i].module, module) && !std::strcmp(file.entries[i].id, id))
            return &file.entries[i];
    return nullptr;
}

// Puts a drive's addresses in, in place of whatever was held for that drive of that module (an older build's are
// of no further use). Every other entry stays. False when the file is full.
inline bool Put(File& file, const Entry& entry)
{
    for (uint32_t i = 0; i < file.count; ++i)
        if (!std::strcmp(file.entries[i].module, entry.module) && !std::strcmp(file.entries[i].id, entry.id))
        {
            file.entries[i] = entry;
            return true;
        }
    if (file.count >= kMaxEntries)
        return false;
    file.entries[file.count++] = entry;
    return true;
}

// The longest line the file holds: an entry with every line, eight hex digits and a comma each.
inline constexpr size_t kMaxLineText = 160 + static_cast<size_t>(drive_resolve::kMaxLines) * 9;
// Room for the head and a full file.
inline constexpr size_t kMaxFileText = 1024 + static_cast<size_t>(kMaxEntries) * kMaxLineText;

// The file's text. The length written, or 0 when `capacity` is too small (nothing useful is in `out` then).
inline size_t Format(const File& file, char* out, size_t capacity)
{
    size_t used = 0;
    bool fits = true;
    const auto put = [&](const char* format, auto... arguments) {
        if (!fits)
            return;
        const int n = std::snprintf(out + used, capacity - used, format, arguments...);
        if (n < 0 || static_cast<size_t>(n) >= capacity - used)
            fits = false;
        else
            used += static_cast<size_t>(n);
    };
    if (!out || capacity < 2)
        return 0;
    put("%s", "# Written by HaloMCCVR. This is a cache, NOT a setting: do not edit it.\r\n"
              "# It holds where the direct drives' code was found in a game DLL that is not the build the mod's own\r\n"
              "# tables were measured on, so that the next launch can check those addresses instead of searching the\r\n"
              "# whole DLL for them again. Every address is checked again before it is used.\r\n"
              "# Safe to DELETE at any time: the next launch searches once and writes it again.\r\n");
    put("schema=%u\r\n", kSchema);
    for (uint32_t i = 0; i < file.count; ++i)
    {
        const Entry& e = file.entries[i];
        put("entry %s %016llX %s %016llX %u ", e.module, static_cast<unsigned long long>(e.stamp), e.id,
            static_cast<unsigned long long>(e.plan), e.count);
        for (uint32_t k = 0; k < e.count; ++k)
            put(k ? ",%X" : "%X", e.rvas[k]);
        put("%s", "\r\n");
    }
    return fits ? used : 0;
}

// Reads a file's text. Only whole, well-formed entries of this schema are taken: a line that is cut short, has
// another count of addresses than it says, or follows a schema line of another number is skipped, and a file with
// no schema line holds nothing. Never fails: what cannot be read is a cold start.
inline void Parse(const char* text, size_t size, File& out)
{
    out = File{};
    if (!text)
        return;
    uint32_t schema = 0;
    char line[kMaxLineText];
    for (size_t at = 0; at < size;)
    {
        size_t end = at;
        while (end < size && text[end] != '\n')
            ++end;
        const bool whole = end < size;   // a last line with no newline was cut short by something
        size_t length = end - at;
        const char* const begin = text + at;
        at = end + 1;
        if (length && begin[length - 1] == '\r')
            --length;
        if (!whole || !length || length >= sizeof(line) || begin[0] == '#')
            continue;
        std::memcpy(line, begin, length);
        line[length] = 0;
        if (!std::strncmp(line, "schema=", 7))
        {
            schema = static_cast<uint32_t>(std::strtoul(line + 7, nullptr, 10));
            continue;
        }
        if (schema != kSchema || std::strncmp(line, "entry ", 6))
            continue;
        // entry <module> <stamp> <drive> <plan> <count> <rva,rva,...>
        Entry e;
        char* cursor = line + 6;
        const auto word = [&cursor](char* into, size_t capacity) {
            size_t n = 0;
            while (*cursor && *cursor != ' ')
            {
                if (n + 1 < capacity)
                    into[n] = *cursor;
                ++n;
                ++cursor;
            }
            if (n >= capacity)
                return false;
            into[n] = 0;
            if (*cursor == ' ')
                ++cursor;
            return n != 0;
        };
        const auto number = [&cursor](int base, uint64_t& value) {
            char* after = nullptr;
            if (*cursor == '-' || *cursor == '+' || *cursor == ' ')
                return false;
            value = std::strtoull(cursor, &after, base);
            if (after == cursor)
                return false;
            cursor = after;
            return true;
        };
        uint64_t count = 0;
        if (!word(e.module, sizeof(e.module)) || !number(16, e.stamp) || *cursor++ != ' ' ||
            !word(e.id, sizeof(e.id)) || !number(16, e.plan) || *cursor++ != ' ' || !number(10, count) ||
            *cursor++ != ' ' || !e.stamp || !count || count > drive_resolve::kMaxLines ||
            !Word(e.module, kMaxModule) || !Word(e.id, kMaxId))
            continue;
        e.count = static_cast<uint32_t>(count);
        bool good = true;
        for (uint32_t k = 0; good && k < e.count; ++k)
        {
            uint64_t rva = 0;
            good = number(16, rva) && rva != 0 && rva <= 0xFFFFFFFFull && (k + 1 == e.count ? *cursor == 0 : *cursor++ == ',');
            e.rvas[k] = static_cast<uint32_t>(rva);
        }
        if (good)
            (void)Put(out, e);
    }
}

// -------------------------------------------------------------------------------------------------- the rungs
enum class Rung : uint8_t
{
    None,
    Recorded,   // rung 1 (a): verified at the tables' recorded addresses
    Saved,      // rung 1 (b): verified at the addresses saved from an earlier search of this build
    Search,     // rung 2
};

// What one drive's resolution went through.
struct Outcome
{
    Rung rung = Rung::None;             // the rung whose result is the one in use
    bool recordedTried = false;         // rung 1 (a) was tried (and failed, when `rung` is not Recorded)
    bool savedTried = false;            // rung 1 (b) was tried
    const char* savedRefused = nullptr; // ... and why its addresses were not used
    bool learn = false;                 // the search resolved the whole drive, at addresses worth saving
    uint32_t scans = 0;                 // searches run, over every rung that ran
    bool searchedToo = false;           // the self-test ran the search beside a rung 1 success
    bool searchAgrees = false;          // ... and it chose the same address for every line
};

using PlanFn = void (*)(Resolver& resolver, void* context);   // runs the drive's plan; its result is kept aside
using KeepFn = void (*)(void* context);                       // that result is the one in use

inline constexpr char kSavedStale[] = "this game DLL does not have that code at them";
inline constexpr char kSavedOtherPlan[] = "they were saved for another version of this drive's tables";

// Whether two resolutions chose the same address for every line.
inline bool SameAddresses(const Report& a, const Report& b)
{
    if (a.count != b.count)
        return false;
    for (uint32_t i = 0; i < a.count; ++i)
        if (a.lines[i].found != b.lines[i].found)
            return false;
    return true;
}

// One drive, by the rungs. `recordedBuild`: the module is the build the drive's tables were measured on. `saved`:
// the cache's entry for this drive in exactly this build of the module, or null. `report` is the report of the
// rung in use.
inline Outcome Run(const Image& image, uint32_t faults, bool recordedBuild, const Entry* saved, PlanFn plan, KeepFn keep,
                   void* context, Report& report)
{
    using drive_resolve::Known;
    using drive_resolve::Source;
    Outcome out;
    report = Report{};
    // A forced stale save stands for a build that is not the recorded one and whose saved addresses are wrong.
    const bool stale = drive_resolve::Fault(faults, drive_resolve::kFaultSavedStale);
    if (recordedBuild && !stale)
    {
        Resolver resolver(image, faults, Known{Source::Recorded, nullptr, 0});
        plan(resolver, context);
        out.recordedTried = true;
        out.scans += resolver.report().scans;
        if (resolver.report().Complete())
        {
            keep(context);
            report = resolver.report();
            out.rung = Rung::Recorded;
        }
    }
    if (out.rung == Rung::None && (saved || stale))
    {
        Resolver resolver(image, faults, Known{Source::Saved, saved ? saved->rvas : nullptr, saved ? saved->count : 0});
        plan(resolver, context);
        out.savedTried = true;
        out.scans += resolver.report().scans;
        const Report& got = resolver.report();
        if (!got.Complete())
        {
            out.savedRefused = kSavedStale;
        }
        else if (!saved || got.plan != saved->plan || got.count != saved->count)
        {
            out.savedRefused = kSavedOtherPlan;
        }
        else
        {
            // What follows from the verified addresses must be what followed from them when they were saved.
            for (uint32_t i = 0; i < got.count && !out.savedRefused; ++i)
                if (got.lines[i].found != saved->rvas[i])
                    out.savedRefused = kSavedStale;
        }
        if (!out.savedRefused)
        {
            keep(context);
            report = got;
            out.rung = Rung::Saved;
        }
    }
    if (out.rung == Rung::None)
    {
        Resolver resolver(image, faults);
        plan(resolver, context);
        keep(context);
        report = resolver.report();
        out.scans += report.scans;
        out.rung = Rung::Search;
        // Worth remembering unless it is simply the recorded addresses on the recorded build, which rung 1 (a)
        // has without a file.
        bool atRecorded = true;
        for (uint32_t i = 0; i < report.count; ++i)
            atRecorded = atRecorded && report.lines[i].found == report.lines[i].recorded;
        out.learn = report.Complete() && !(recordedBuild && atRecorded);
    }
    else if (drive_resolve::Fault(faults, drive_resolve::kFaultSearchToo))
    {
        // THE SELF-TEST: the search as well, beside the rung 1 result that stays in use.
        Resolver resolver(image, faults);
        plan(resolver, context);
        out.scans += resolver.report().scans;
        out.searchedToo = true;
        out.searchAgrees = SameAddresses(report, resolver.report());
    }
    return out;
}

// The same for a plan that is a callable: Found (Resolver&).
template <typename Found, typename Plan>
inline Outcome RunTyped(const Image& image, uint32_t faults, bool recordedBuild, const Entry* saved, Plan& plan,
                        Found& found, Report& report)
{
    struct Context
    {
        Plan* plan;
        Found aside;
        Found* found;
    } context{&plan, Found{}, &found};
    return Run(
        image, faults, recordedBuild, saved,
        [](Resolver& resolver, void* raw) {
            auto* c = static_cast<Context*>(raw);
            c->aside = (*c->plan)(resolver);
        },
        [](void* raw) {
            auto* c = static_cast<Context*>(raw);
            *c->found = c->aside;
        },
        &context, report);
}
} // namespace drive_known
