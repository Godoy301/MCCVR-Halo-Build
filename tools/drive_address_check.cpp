// THE OFFLINE CHECK OF THE DIRECT DRIVES' ADDRESSES, against the game's own DLL files. No game, no headset.
//
//   halomccvr_drive_address_check --mcc "<the MCC install folder>" [--faults] [--moved] [--title <dll name>] [--brief]
//                                 [--loaded <a scratch folder>]
//
// For each game DLL it lays the file out as an image (headers and sections at their RVAs, base relocations
// applied, as the loader would), and runs every direct drive's own resolution on it: the same tables and the same
// code the DLL runs at install (src/common/drive_sites.h, src/common/drive_resolve.h). Per site it prints the
// recorded RVA, the RVA found, how many places the signature matched, how it was found and the verdict.
//
// ACCEPTANCE. On the build the tables were measured on, every site must resolve "as recorded": the resolution
// then hooks and reads exactly what the recorded addresses named. Any other verdict fails the check (exit 1).
// On another build the same table says what moved and what was lost; --any-build prints it without failing.
//
// --faults repeats the whole resolution with each fault forced (drive_resolve.h, kFault...), and checks what each
// must do: a signature that finds nothing leaves the drive unhooked; an ambiguous one is settled by the recorded
// address; a wrong record is overruled by the search (the same addresses, reported as moved); ambiguous with a
// wrong record is refused; a TLS directory that disagrees stops every drive that reads the TLS index; an anchor
// denied its offset is found as the one match in its function, or not at all where it has two.
//
// --moved builds a second image of each DLL in which every section sits 0x40 bytes higher -- the same code after
// something was inserted ahead of it, so that no recorded RVA is right -- and checks that every drive follows it:
// every site found 0x40 above where it was, reported moved, and the drive hookable there.
//
// --loaded <folder> repeats the healthy resolution on each DLL AS THE WINDOWS LOADER MAPS IT, through the DLL's own
// side of the resolution (src/dll/drive_cascade.cpp: the module opened by VirtualQuery, its .pdata where the
// loader put it, its TLS directory as relocated). A copy of the file is made in that folder and mapped with
// DONT_RESOLVE_DLL_REFERENCES: sections placed and relocations applied, no import resolved, none of its code run.
// Every drive must resolve exactly as it did from the file, and at every site the function-table entry the
// resolution read must be the one the operating system's own lookup answers with. The copy is deleted afterwards.
//
// The MCC folder comes from --mcc or the HALOMCCVR_MCC_DIR environment variable. With neither, the check is
// SKIPPED (exit 77), so a machine without the game does not fail its tests. The DLL files are only read.
#include <Windows.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "addrcascade/AddressCascade.hpp"
#include "drive_sites.h"
#include "title_build_identity.h"
#include "../src/dll/drive_cascade.h"

// The DLL's side of the resolution logs through this (--loaded).
void Logf(const char* format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    std::printf("   | ");
    std::vprintf(format, arguments);
    std::printf("\n");
    va_end(arguments);
}

namespace
{
using drive_resolve::Report;
using drive_resolve::Resolver;
using drive_resolve::Verdict;

constexpr int kExitSkipped = 77;

struct Loaded
{
    uint8_t* base = nullptr;
    size_t size = 0;
    uint32_t timestamp = 0;
    drive_resolve::Image image{};
};

// Lays a PE32+ DLL file out as the loader maps it: headers, each section at its RVA, base relocations applied for
// where the copy lives. Imports are not bound (nothing here reads them).
bool LoadAsImage(const std::wstring& path, Loaded& out, std::string& why)
{
    const HANDLE file = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                                    nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE)
    {
        why = "the file could not be opened";
        return false;
    }
    LARGE_INTEGER fileSize{};
    std::vector<uint8_t> bytes;
    bool read = GetFileSizeEx(file, &fileSize) && fileSize.QuadPart > 0x400 && fileSize.QuadPart < 0x40000000;
    if (read)
    {
        bytes.resize(static_cast<size_t>(fileSize.QuadPart));
        size_t done = 0;
        while (read && done < bytes.size())
        {
            DWORD got = 0;
            const DWORD want = static_cast<DWORD>((std::min)(bytes.size() - done, static_cast<size_t>(1u << 24)));
            read = ReadFile(file, bytes.data() + done, want, &got, nullptr) && got != 0;
            done += got;
        }
    }
    CloseHandle(file);
    if (!read)
    {
        why = "the file could not be read";
        return false;
    }
    const auto u16 = [&bytes](size_t at) { uint16_t v = 0; std::memcpy(&v, bytes.data() + at, 2); return v; };
    const auto u32 = [&bytes](size_t at) { uint32_t v = 0; std::memcpy(&v, bytes.data() + at, 4); return v; };
    const auto u64 = [&bytes](size_t at) { uint64_t v = 0; std::memcpy(&v, bytes.data() + at, 8); return v; };
    const size_t pe = u32(0x3C);
    if (u16(0) != 0x5A4D || pe + 0x108 > bytes.size() || u32(pe) != 0x00004550u || u16(pe + 4) != 0x8664 ||
        u16(pe + 24) != 0x20B)
    {
        why = "not an x64 PE32+ file";
        return false;
    }
    const size_t optional = pe + 24;
    const uint32_t sections = u16(pe + 6);
    const size_t table = optional + u16(pe + 20);
    const uint64_t preferredBase = u64(optional + 24);
    const uint32_t sizeOfImage = u32(optional + 56), sizeOfHeaders = u32(optional + 60);
    if (!sizeOfImage || sizeOfHeaders > bytes.size() || sizeOfHeaders > sizeOfImage ||
        table + static_cast<size_t>(sections) * 40 > bytes.size())
    {
        why = "its headers do not fit the file";
        return false;
    }
    auto* image = static_cast<uint8_t*>(VirtualAlloc(nullptr, sizeOfImage, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!image)
    {
        why = "no memory for the image";
        return false;
    }
    std::memcpy(image, bytes.data(), sizeOfHeaders);
    for (uint32_t i = 0; i < sections; ++i)
    {
        const size_t at = table + static_cast<size_t>(i) * 40;
        const uint32_t virtualSize = u32(at + 8), rva = u32(at + 12), rawSize = u32(at + 16), raw = u32(at + 20);
        const uint32_t copy = virtualSize ? (std::min)(virtualSize, rawSize) : rawSize;
        if (!copy)
            continue;
        if (rva > sizeOfImage || copy > sizeOfImage - rva || raw > bytes.size() || copy > bytes.size() - raw)
        {
            VirtualFree(image, 0, MEM_RELEASE);
            why = "a section does not fit the file or the image";
            return false;
        }
        std::memcpy(image + rva, bytes.data() + raw, copy);
    }
    // Base relocations (IMAGE_DIRECTORY_ENTRY_BASERELOC), type DIR64 only: what the loader does for an x64 image.
    const uint32_t relocRva = u32(optional + 112 + 5 * 8), relocSize = u32(optional + 112 + 5 * 8 + 4);
    const uint64_t delta = reinterpret_cast<uint64_t>(image) - preferredBase;
    if (relocRva && relocSize && relocRva < sizeOfImage && relocSize <= sizeOfImage - relocRva)
    {
        for (uint32_t at = 0; at + 8 <= relocSize;)
        {
            uint32_t page = 0, block = 0;
            std::memcpy(&page, image + relocRva + at, 4);
            std::memcpy(&block, image + relocRva + at + 4, 4);
            if (block < 8 || block > relocSize - at)
                break;
            for (uint32_t e = 8; e + 2 <= block; e += 2)
            {
                uint16_t entry = 0;
                std::memcpy(&entry, image + relocRva + at + e, 2);
                const uint32_t target = page + (entry & 0xFFFu);
                if ((entry >> 12) == 10 && target <= sizeOfImage - 8)
                {
                    uint64_t value = 0;
                    std::memcpy(&value, image + target, 8);
                    value += delta;
                    std::memcpy(image + target, &value, 8);
                }
            }
            at += block;
        }
    }
    out.base = image;
    out.size = sizeOfImage;
    out.timestamp = u32(pe + 8);
    return true;
}

// Reads a laid-out image the way the DLL reads a loaded module: its executable sections and .pdata from its own
// headers, and its TLS index from its PE TLS directory.
bool OpenImage(Loaded& module, std::string& why)
{
    const char* refused = nullptr;
    if (!drive_resolve::ReadImage(module.base, module.size, module.image, refused))
    {
        why = refused ? refused : "its image did not read";
        return false;
    }
    uint32_t index = 0;
    uintptr_t rva = 0;
    if (addrcascade::tls_index(module.base, &index, &rva) && rva && rva + sizeof(uint32_t) <= module.size)
        module.image.tlsIndexRva = static_cast<uint32_t>(rva);
    return true;
}

// THE SAME BUILD WITH EVERYTHING MOVED: a copy of the image in which every section sits `shift` bytes higher, as
// after code was inserted ahead of everything. The code's own bytes are unchanged, so every call and every
// rip-relative operand still lands where it did, `shift` higher; what holds RVAs or addresses and is read here is
// rewritten to match: the section table, the data directories, every .pdata entry (and the parent entry inside a
// chained fragment's unwind info) and the TLS directory. No recorded RVA is right for this image.
bool MoveImage(const Loaded& module, uint32_t shift, Loaded& out, std::string& why)
{
    const uint8_t* const src = module.base;
    const auto s32 = [src](size_t at) { uint32_t v = 0; std::memcpy(&v, src + at, 4); return v; };
    const size_t pe = s32(0x3C), optional = pe + 24;
    uint16_t sections = 0, optionalSize = 0;
    std::memcpy(&sections, src + pe + 6, 2);
    std::memcpy(&optionalSize, src + pe + 20, 2);
    const size_t table = optional + optionalSize;
    uint32_t first = static_cast<uint32_t>(module.size);
    for (uint32_t i = 0; i < sections; ++i)
        first = (std::min)(first, s32(table + static_cast<size_t>(i) * 40 + 12));
    if (first < table + static_cast<size_t>(sections) * 40 || first >= module.size)
    {
        why = "its sections do not follow its headers";
        return false;
    }
    const size_t size = module.size + shift;
    auto* image = static_cast<uint8_t*>(VirtualAlloc(nullptr, size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!image)
    {
        why = "no memory for the moved image";
        return false;
    }
    std::memcpy(image, src, first);
    std::memcpy(image + first + shift, src + first, module.size - first);
    const auto add32 = [image, shift](size_t at) {
        uint32_t v = 0;
        std::memcpy(&v, image + at, 4);
        if (v)
        {
            v += shift;
            std::memcpy(image + at, &v, 4);
        }
    };
    add32(optional + 56);   // SizeOfImage
    for (uint32_t i = 0; i < sections; ++i)
        add32(table + static_cast<size_t>(i) * 40 + 12);
    for (uint32_t d = 0; d < 16; ++d)
        add32(optional + 112 + static_cast<size_t>(d) * 8);
    // .pdata: each entry's begin, end and unwind address; a chained fragment's unwind info also holds its parent.
    const uint32_t pdata = s32(optional + 112 + 3 * 8) + shift, pdataSize = s32(optional + 112 + 3 * 8 + 4);
    std::vector<uint32_t> chained;
    for (uint32_t at = 0; at + 12 <= pdataSize; at += 12)
    {
        uint32_t begin = 0, unwind = 0;
        std::memcpy(&begin, image + pdata + at, 4);
        if (!begin)
            continue;
        add32(pdata + at);
        add32(pdata + at + 4);
        add32(pdata + at + 8);
        std::memcpy(&unwind, image + pdata + at + 8, 4);
        if (!(unwind & 1u) && unwind + 4 <= size && ((image[unwind] >> 3) & 4u))
            chained.push_back(unwind + 4 + ((static_cast<uint32_t>(image[unwind + 2]) + 1u) & ~1u) * 2u);
    }
    std::sort(chained.begin(), chained.end());
    chained.erase(std::unique(chained.begin(), chained.end()), chained.end());
    for (const uint32_t parent : chained)
        if (parent + 12 <= size)
        {
            add32(parent);
            add32(parent + 4);
            add32(parent + 8);
        }
    // The TLS directory holds addresses (already relocated to where the first copy lives): each moves too.
    const uint32_t tls = s32(optional + 112 + 9 * 8);
    if (tls)
        for (uint32_t field = 0; field < 4; ++field)
        {
            uint64_t v = 0;
            std::memcpy(&v, image + tls + shift + field * 8, 8);
            if (v)
            {
                v = v - reinterpret_cast<uint64_t>(src) + reinterpret_cast<uint64_t>(image) + shift;
                std::memcpy(image + tls + shift + field * 8, &v, 8);
            }
        }
    out = Loaded{};
    out.base = image;
    out.size = size;
    out.timestamp = module.timestamp;
    if (!OpenImage(out, why))
    {
        VirtualFree(image, 0, MEM_RELEASE);
        out = Loaded{};
        return false;
    }
    return true;
}

// One drive's resolution: its report, where it would hook, and whether the hook may exist.
struct Outcome
{
    bool ok = false;
    uint32_t hook = 0;
    char detail[240]{};
    Report report{};
};

using RunFn = void (*)(Resolver&, Outcome&);

void RunCallers(Resolver& r, const drive_sites::CallerAnchors& anchors, uint32_t entry, char* text, size_t size)
{
    const drive_sites::Callers c = drive_sites::ResolveCallers(r, anchors, entry);
    const bool dispatcher = anchors.accumulatorCall.pattern != nullptr;
    int proven = 0, sites = 0;
    bool stick = false;
    for (int i = 0; i < drive_sites::kMaxCallers; ++i)
    {
        if (!anchors.sites[i].pattern)
            continue;
        ++sites;
        proven += c.call[i] ? 1 : 0;
        stick = stick || (c.call[i] && anchors.sites[i].role == aim_caller::Caller::Stick);
    }
    if (dispatcher)
        std::snprintf(text, size, "caller map %s (dispatcher +0x%X, frame 0x%X, %d of %d callers%s%s)",
                      c.frame && stick ? "proven" : "NOT proven", c.dispatcher, c.frame, proven, sites,
                      c.why ? "; " : "", c.why ? c.why : "");
    else
        std::snprintf(text, size, "caller map %s (by return address, %d of %d callers)", stick ? "proven" : "NOT proven",
                      proven, sites);
}

void RunReachControl(Resolver& r, Outcome& out)
{
    const drive_sites::ReachControl f = drive_sites::ResolveReachControl(r);
    out.ok = f.ok;
    out.hook = f.entry;
    RunCallers(r, drive_sites::kReachCallerAnchors, f.entry, out.detail, sizeof(out.detail));
    (void)drive_sites::ResolveTick(r, drive_sites::kReachTickAnchors);
}

void RunLegacy(Resolver& r, Outcome& out, int slot)
{
    const drive_sites::LegacyControl f = drive_sites::ResolveLegacyControl(r, slot);
    out.ok = f.ok;
    out.hook = f.entry;
    char callers[160];
    RunCallers(r, drive_sites::kLegacyCallerAnchors[slot], f.entry, callers, sizeof(callers));
    if (slot == 2)
        std::snprintf(out.detail, sizeof(out.detail), "%s; record unit load %s", callers,
                      f.recordUnitLoad ? "proven (seats may be written)" : "NOT proven (seats keep the stick servo)");
    else
        std::snprintf(out.detail, sizeof(out.detail), "%s; pitch clamp %s", callers,
                      f.pitchLimit ? "found" : "NOT found (a look-steered driver's reticle is not clamped in pitch)");
    (void)drive_sites::ResolveTick(r, drive_sites::kLegacyTickAnchors[slot]);
}
void RunHalo3(Resolver& r, Outcome& out) { RunLegacy(r, out, 0); }
void RunOdst(Resolver& r, Outcome& out) { RunLegacy(r, out, 1); }
void RunHalo4(Resolver& r, Outcome& out) { RunLegacy(r, out, 2); }

void DescribeFlight(const drive_sites::Flight& f, Outcome& out)
{
    out.ok = f.ok;
    out.hook = f.entry;
    std::snprintf(out.detail, sizeof(out.detail),
                  "rows missing %u; controller's call +0x%X; dispatcher +0x%X, frame 0x%X%s%s; callers +0x%X +0x%X "
                  "+0x%X +0x%X (stick update +0x%X)",
                  f.missing, f.call, f.dispatcher, f.frame, f.frameWhy ? ": " : "", f.frameWhy ? f.frameWhy : "",
                  f.callerCall[0], f.callerCall[1], f.callerCall[2], f.callerCall[3], f.stickCall);
}

void RunReachSabre(Resolver& r, Outcome& out)
{
    DescribeFlight(drive_sites::ResolveReachSabre(r).flight, out);
}

void RunHalo4Flight(Resolver& r, Outcome& out)
{
    DescribeFlight(drive_sites::ResolveHalo4Flight(r).flight, out);
}

void RunHalo2(Resolver& r, Outcome& out)
{
    const drive_sites::Halo2Control f = drive_sites::ResolveHalo2Control(r);
    out.ok = f.ok;
    out.hook = f.entry;
    std::snprintf(out.detail, sizeof(out.detail), "control update's call +0x%X; player control globals slot +0x%X",
                  f.call, f.slot);
}

template <size_t N>
void RunT13(Resolver& r, Outcome& out, const drive_sites::Anchor (&rows)[N], const drive_sites::FlightSites& flight,
            const drive_sites::T13Sites& sites)
{
    uint32_t found[N]{};
    const drive_sites::T13 f = drive_sites::ResolveT13(r, rows, found, flight, sites);
    out.ok = f.ok;
    out.hook = 0;
    std::snprintf(out.detail, sizeof(out.detail),
                  "rows missing %u; TLS index +0x%X, tag pages +0x%X, tag instances +0x%X (each agreed by its sites); "
                  "dispatcher frame 0x%X",
                  f.flight.missing, f.tlsIndex, f.pages, f.instances, f.flight.frame);
}
void RunT13Reach(Resolver& r, Outcome& out)
{
    RunT13(r, out, drive_sites::kT13ReachAnchors, drive_sites::kT13ReachFlight, drive_sites::kT13ReachSites);
}
void RunT13Halo4(Resolver& r, Outcome& out)
{
    RunT13(r, out, drive_sites::kT13Halo4Anchors, drive_sites::kT13Halo4Flight, drive_sites::kT13Halo4Sites);
}

struct Drive
{
    const char* dll;
    const char* name;
    RunFn run;
    bool readsTlsIndex;        // its resolution needs the engine TLS index
    bool survivesOffsetFault;  // with every in-function anchor denied its offset it can still be hooked
    const char* lostToOffsetFault[3];   // the sites that fault must lose (two matches in their function)
};

constexpr Drive kDrives[] = {
    {"halo2", "Halo 2 control update (on foot and seats)", &RunHalo2, false, true, {}},
    {"halo3", "Halo 3 control update (on foot and seats)", &RunHalo3, true, true, {}},
    {"halo3odst", "ODST control update (on foot and seats)", &RunOdst, true, true, {}},
    {"haloreach", "Reach control update (on foot and seats)", &RunReachControl, true, true, {}},
    {"haloreach", "Reach Sabre / Seraph pilot controller", &RunReachSabre, true, true, {}},
    {"haloreach", "Reach pilot controller, dev observer (dev builds only)", &RunT13Reach, true, true, {}},
    // Halo 4's record unit load stands three times in the control update: without its offset it is not found,
    // and only the seats lose direct drive.
    {"halo4", "Halo 4 control update (on foot and seats)", &RunHalo4, true, true, {"record unit load"}},
    {"halo4", "Halo 4 Broadsword pilot controller", &RunHalo4Flight, true, true, {}},
    // The observer's IDLE and RECUNIT rows stand more than once in their functions: the observer stays off.
    {"halo4", "Halo 4 pilot controller, dev observer (dev builds only)", &RunT13Halo4, true, false, {"IDLE", "RECUNIT"}},
};

struct TitleDll
{
    const char* dll;
    GameTitle title;
};
constexpr TitleDll kTitles[] = {{"halo1", GameTitle::HaloCE},        {"halo2", GameTitle::Halo2},
                                {"halo3", GameTitle::Halo3},         {"halo3odst", GameTitle::Halo3ODST},
                                {"haloreach", GameTitle::HaloReach}, {"halo4", GameTitle::Halo4}};

Outcome Run(const Loaded& module, const Drive& drive, uint32_t faults)
{
    Outcome out;
    Resolver r(module.image, faults);
    drive.run(r, out);
    out.report = r.report();
    return out;
}

void PrintReport(const Report& report)
{
    std::printf("     %-30s %11s %11s %7s  %-26s %s\n", "site", "recorded", "found", "matches", "how", "verdict");
    for (uint32_t i = 0; i < report.count; ++i)
    {
        const drive_resolve::Line& line = report.lines[i];
        char found[16] = "-", matches[8] = "-";
        if (line.found)
            std::snprintf(found, sizeof(found), "+0x%07X", line.found);
        if (line.how == drive_resolve::How::Signature || line.how == drive_resolve::How::FunctionExtent ||
            line.how == drive_resolve::How::FunctionStart)
            std::snprintf(matches, sizeof(matches), "%u%s", line.matches,
                          line.matches >= drive_resolve::kCountCap ? "+" : "");
        std::printf("     %-30s  +0x%07X %11s %7s  %-26s %s%s%s\n", line.name, line.recorded, found, matches,
                    drive_resolve::HowText(line.how), drive_resolve::VerdictText(line.verdict), line.note ? ": " : "",
                    line.note ? line.note : "");
    }
    if (report.dropped)
        std::printf("     (%u more lines than the report holds)\n", report.dropped);
}

bool Lost(const Drive& drive, const char* name)
{
    for (const char* lost : drive.lostToOffsetFault)
        if (lost && !std::strcmp(lost, name))
            return true;
    return false;
}

struct FaultCase
{
    uint32_t faults;
    const char* what;
    const char* expect;
};

constexpr FaultCase kFaultCases[] = {
    {drive_resolve::kFaultSignatureMissing, "a signature finds nothing", "no drive is hooked"},
    {drive_resolve::kFaultSignatureAmbiguous, "every signature is ambiguous",
     "every drive is hooked at the same addresses, each taken at its recorded address among several"},
    {drive_resolve::kFaultRecordWrong, "every recorded RVA is wrong",
     "every drive is hooked at the same addresses, found by search and reported MOVED"},
    {drive_resolve::kFaultSignatureAmbiguous | drive_resolve::kFaultRecordWrong,
     "ambiguous signatures and wrong records together", "no drive is hooked"},
    {drive_resolve::kFaultTlsDirectory, "the PE TLS directory disagrees",
     "no drive that reads the engine TLS index is hooked; the rest are unchanged"},
    {drive_resolve::kFaultOffsetMoved, "no in-function anchor is at its recorded offset",
     "each is found as the one match in its function; one with two matches there is not found"},
    {drive_resolve::kFaultHookSilent, "a drive's watchdog is told its hook never ran",
     "nothing about the addresses changes (the watchdog is the DLL's)"},
};

// What a forced fault must do to one drive, judged against its healthy resolution. Returns the failure, or null.
const char* JudgeFault(const Drive& drive, const FaultCase& test, const Outcome& healthy, const Outcome& faulted)
{
    const uint32_t f = test.faults;
    const bool missing = f == drive_resolve::kFaultSignatureMissing;
    const bool refused = f == (drive_resolve::kFaultSignatureAmbiguous | drive_resolve::kFaultRecordWrong);
    if (missing || refused)
    {
        if (faulted.ok)
            return "the drive would still be hooked";
        for (uint32_t i = 0; i < faulted.report.count; ++i)
            if (faulted.report.lines[i].how == drive_resolve::How::Signature && faulted.report.lines[i].found)
                return "a signature search still found something";
        return nullptr;
    }
    if (f == drive_resolve::kFaultTlsDirectory)
    {
        if (drive.readsTlsIndex ? faulted.ok : !faulted.ok)
            return drive.readsTlsIndex ? "the drive would still be hooked" : "a drive that reads no TLS index was lost";
        return nullptr;
    }
    const bool expectOk = healthy.ok && (f != drive_resolve::kFaultOffsetMoved || drive.survivesOffsetFault);
    if (faulted.ok != expectOk)
        return expectOk ? "the drive was lost" : "the drive would still be hooked";
    if (faulted.report.count != healthy.report.count && f != drive_resolve::kFaultOffsetMoved)
        return "the resolution took another path";
    for (uint32_t i = 0; i < faulted.report.count && i < healthy.report.count; ++i)
    {
        const drive_resolve::Line& was = healthy.report.lines[i];
        const drive_resolve::Line& now = faulted.report.lines[i];
        if (std::strcmp(was.name, now.name))
            return f == drive_resolve::kFaultOffsetMoved ? nullptr : "the resolution took another path";
        if (f == drive_resolve::kFaultOffsetMoved && Lost(drive, now.name))
        {
            if (now.found)
                return "an anchor with two matches in its function was still taken";
            continue;
        }
        if (f == drive_resolve::kFaultOffsetMoved && !now.found && !was.found)
            continue;
        if (now.found != was.found)
            return "an address changed";
        if (f == drive_resolve::kFaultSignatureAmbiguous && now.how == drive_resolve::How::Signature &&
            now.verdict != Verdict::RecordAmongSeveral)
            return "an ambiguous signature was not settled by the recorded address";
        if (f == drive_resolve::kFaultRecordWrong && now.found && now.verdict != Verdict::Moved)
            return "an address found against a wrong record was not reported moved";
        if (f == drive_resolve::kFaultOffsetMoved && was.how == drive_resolve::How::FunctionOffset && now.found &&
            now.how != drive_resolve::How::FunctionExtent)
            return "an in-function anchor was still taken at its offset";
    }
    return nullptr;
}

std::wstring Widen(const char* text)
{
    const int n = MultiByteToWideChar(CP_UTF8, 0, text, -1, nullptr, 0);
    std::wstring wide(n > 0 ? static_cast<size_t>(n - 1) : 0, L'\0');
    if (n > 1)
        MultiByteToWideChar(CP_UTF8, 0, text, -1, wide.data(), n);
    return wide;
}

// --loaded: a copy of the DLL file, mapped as the Windows loader maps an image. DONT_RESOLVE_DLL_REFERENCES places
// the sections and applies the relocations, and stops there: no import is resolved and nothing of the DLL runs.
HMODULE MapCopy(const std::wstring& source, const std::wstring& folder, const char* title, std::wstring& copy,
                std::string& why)
{
    CreateDirectoryW(folder.c_str(), nullptr);
    copy = folder + L"\\" + Widen(title) + L".drivecheck.dll";
    if (!CopyFileW(source.c_str(), copy.c_str(), FALSE))
    {
        why = "the copy could not be made";
        return nullptr;
    }
    const HMODULE module = LoadLibraryExW(copy.c_str(), nullptr, DONT_RESOLVE_DLL_REFERENCES);
    if (!module)
        why = "the copy could not be mapped";
    return module;
}

// One drive's resolution on the mapped copy, through drive_cascade::RunPlan.
struct LoadedRun
{
    const Drive* drive = nullptr;
    uintptr_t base = 0;
    Outcome out{};
    unsigned tableChecked = 0, tableAgreed = 0;
};

void LoadedThunk(Resolver& r, void* raw)
{
    auto* run = static_cast<LoadedRun*>(raw);
    run->drive->run(r, run->out);
    // At every site found: the .pdata entry the resolution reads is the operating system's own answer.
    const Report& report = r.report();
    for (uint32_t i = 0; i < report.count; ++i)
    {
        const uint32_t at = report.lines[i].found;
        if (!at)
            continue;
        DWORD64 imageBase = 0;
        const PRUNTIME_FUNCTION os = RtlLookupFunctionEntry(run->base + at, &imageBase, nullptr);
        const drive_resolve::RuntimeFunction* ours = drive_resolve::EntryAt(r.image(), at);
        ++run->tableChecked;
        if (os && imageBase == run->base ? (ours && ours->begin == os->BeginAddress && ours->end == os->EndAddress)
                                         : ours == nullptr)
            ++run->tableAgreed;
    }
}

// The drive on the mapped copy against the same drive on the file: null when they are the same, else what differs.
const char* CompareLoaded(const Outcome& file, const LoadedRun& run, const Report& report, bool opened)
{
    if (!opened)
        return "the mapped module did not open";
    if (run.out.ok != file.ok || run.out.hook != file.hook || report.count != file.report.count)
        return "the drive did not resolve the same way";
    for (uint32_t i = 0; i < report.count; ++i)
        if (report.lines[i].found != file.report.lines[i].found || report.lines[i].verdict != file.report.lines[i].verdict)
            return report.lines[i].name;
    if (run.tableAgreed != run.tableChecked)
        return "the operating system's function table disagrees with the .pdata that was read";
    return nullptr;
}
} // namespace

int main(int argc, char** argv)
{
    std::string mcc, loadedFolder;
    const char* only = nullptr;
    bool faults = false, brief = false, anyBuild = false, moved = false;
    for (int i = 1; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--mcc") && i + 1 < argc)
            mcc = argv[++i];
        else if (!std::strcmp(argv[i], "--title") && i + 1 < argc)
            only = argv[++i];
        else if (!std::strcmp(argv[i], "--faults"))
            faults = true;
        else if (!std::strcmp(argv[i], "--moved"))
            moved = true;
        else if (!std::strcmp(argv[i], "--brief"))
            brief = true;
        else if (!std::strcmp(argv[i], "--any-build"))
            anyBuild = true;
        else if (!std::strcmp(argv[i], "--loaded") && i + 1 < argc)
            loadedFolder = argv[++i];
        else
        {
            std::printf("usage: %s --mcc <MCC install folder> [--faults] [--moved] [--title <dll name>] [--brief] "
                        "[--any-build] [--loaded <scratch folder>]\n", argv[0]);
            return 2;
        }
    }
    if (mcc.empty())
    {
        char env[1024];
        const DWORD n = GetEnvironmentVariableA("HALOMCCVR_MCC_DIR", env, sizeof(env));
        if (n && n < sizeof(env))
            mcc = env;
    }
    if (mcc.empty())
    {
        std::printf("drive address check: SKIPPED (no MCC folder: pass --mcc <folder> or set HALOMCCVR_MCC_DIR)\n");
        return kExitSkipped;
    }
    addrcascade::set_logger([](const char* message) { std::printf("%s\n", message); });

    unsigned drives = 0, sites = 0, notAsRecorded = 0, unhooked = 0, faultChecks = 0, faultFailures = 0, modules = 0;
    unsigned movedSites = 0, movedFollowed = 0, movedFailures = 0;
    unsigned loadedDrives = 0, loadedSites = 0, loadedFailures = 0;
    constexpr uint32_t kMovedShift = 0x40;
    for (const TitleDll& entry : kTitles)
    {
        const char* const title = entry.dll;
        if (only && std::strcmp(only, title))
            continue;
        const std::string path = mcc + "\\" + title + "\\" + title + ".dll";
        Loaded module;
        std::string why;
        if (!LoadAsImage(Widen(path.c_str()), module, why) || !OpenImage(module, why))
        {
            // A title that is not installed is not a failure; a DLL that is there and does not read is.
            const bool absent = GetFileAttributesW(Widen(path.c_str()).c_str()) == INVALID_FILE_ATTRIBUTES;
            std::printf("== %s.dll: NOT CHECKED (%s: %s)\n", title, absent ? "not installed" : why.c_str(), path.c_str());
            notAsRecorded += absent ? 0u : 1u;
            continue;
        }
        ++modules;
        const title_build::Match match =
            title_build::Compare(entry.title, module.timestamp, static_cast<uint32_t>(module.size));
        std::printf("== %s.dll: PE timestamp 0x%08X, image 0x%08zX: %s; TLS index at +0x%X (PE TLS directory)\n", title,
                    module.timestamp, module.size,
                    match == title_build::Match::Known ? "the build the tables were measured on (MCC 2025.08.16.178512.1)"
                                                       : "NOT the build the tables were measured on",
                    module.image.tlsIndexRva);
        Loaded shifted;
        if (moved && !MoveImage(module, kMovedShift, shifted, why))
        {
            std::printf("   the moved image could not be built (%s)\n", why.c_str());
            ++movedFailures;
        }
        bool any = false;
        HMODULE mapped = nullptr;
        std::wstring mappedCopy;
        bool mapTried = false;
        for (const Drive& drive : kDrives)
        {
            if (std::strcmp(drive.dll, title))
                continue;
            any = true;
            ++drives;
            const Outcome healthy = Run(module, drive, 0);
            unsigned off = 0;
            for (uint32_t i = 0; i < healthy.report.count; ++i)
                off += healthy.report.lines[i].verdict == Verdict::AsRecorded ? 0u : 1u;
            sites += healthy.report.count;
            notAsRecorded += off;
            unhooked += healthy.ok ? 0u : 1u;
            std::printf("-- %s\n", drive.name);
            if (!brief || off || !healthy.ok)
                PrintReport(healthy.report);
            if (healthy.hook)
                std::printf("   %s +0x%X; %s\n", healthy.ok ? "HOOK at" : "NO HOOK, though its entry was found at",
                            healthy.hook, healthy.detail);
            else
                std::printf("   %s; %s\n", healthy.ok ? "READY" : "NOT READY", healthy.detail);
            std::printf("   %u sites, %u as recorded, %u moved, %u taken at the recorded address among several, %u not "
                        "found\n",
                        healthy.report.count, healthy.report.Count(Verdict::AsRecorded),
                        healthy.report.Count(Verdict::Moved), healthy.report.Count(Verdict::RecordAmongSeveral),
                        healthy.report.NotFound());
            if (moved && shifted.base)
            {
                // The same drive on the image with every section kMovedShift higher: it must follow, site by site.
                const Outcome there = Run(shifted, drive, 0);
                unsigned followed = 0, total = 0;
                const char* failure = there.ok == healthy.ok && there.report.count == healthy.report.count
                    ? nullptr : "the drive did not resolve the same way";
                for (uint32_t i = 0; !failure && i < there.report.count; ++i)
                {
                    const drive_resolve::Line& was = healthy.report.lines[i];
                    const drive_resolve::Line& now = there.report.lines[i];
                    if (!was.found)
                        continue;
                    ++total;
                    if (now.found == was.found + kMovedShift && now.verdict == Verdict::Moved)
                        ++followed;
                    else
                        failure = now.name;
                }
                movedSites += total;
                movedFollowed += followed;
                movedFailures += failure ? 1u : 0u;
                char where[48];
                if (there.hook)
                    std::snprintf(where, sizeof(where), "%s +0x%X", there.ok ? "HOOK at" : "NO HOOK at", there.hook);
                else
                    std::snprintf(where, sizeof(where), "%s", there.ok ? "READY" : "NOT READY");
                std::printf("   moved build (every section 0x%X higher): %s; %u of %u sites followed and reported "
                            "MOVED%s%s\n",
                            kMovedShift, where, followed, total, failure ? " -- NOT FOLLOWED: " : "",
                            failure ? failure : "");
            }
            if (!loadedFolder.empty())
            {
                // The same drive on a copy of the DLL as the Windows loader maps it, through the DLL's own side.
                if (!mapTried)
                {
                    mapTried = true;
                    mapped = MapCopy(Widen(path.c_str()), Widen(loadedFolder.c_str()), title, mappedCopy, why);
                    if (!mapped)
                        std::printf("   as the loader maps it: NOT CHECKED (%s)\n", why.c_str());
                }
                ++loadedDrives;
                if (!mapped)
                {
                    ++loadedFailures;
                }
                else
                {
                    const uintptr_t base = reinterpret_cast<uintptr_t>(mapped);
                    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
                    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
                    LoadedRun run;
                    run.drive = &drive;
                    run.base = base;
                    Report report{};
                    const char* notOpened = nullptr;
                    const bool opened = drive_cascade::RunPlan(base, nt->OptionalHeader.SizeOfImage, &LoadedThunk, &run,
                                                               report, notOpened);
                    const char* failure = CompareLoaded(healthy, run, report, opened);
                    loadedSites += report.count;
                    loadedFailures += failure ? 1u : 0u;
                    std::printf("   as the loader maps it (base %p, through drive_cascade): %s; %u of %u sites as from "
                                "the file; the system's function table agrees at %u of %u%s%s%s%s\n",
                                reinterpret_cast<void*>(base),
                                run.out.hook ? (run.out.ok ? "HOOK at the same address" : "NO HOOK")
                                             : (run.out.ok ? "READY" : "NOT READY"),
                                failure ? 0u : report.count, report.count, run.tableAgreed, run.tableChecked,
                                failure ? " -- DIFFERS: " : "", failure ? failure : "",
                                notOpened ? ": " : "", notOpened ? notOpened : "");
                }
            }
            if (!faults)
                continue;
            for (const FaultCase& test : kFaultCases)
            {
                const Outcome faulted = Run(module, drive, test.faults);
                const char* failure = JudgeFault(drive, test, healthy, faulted);
                ++faultChecks;
                faultFailures += failure ? 1u : 0u;
                // What the fault made of the drive, in one line: its first site that is not as recorded.
                const drive_resolve::Line* first = nullptr;
                for (uint32_t i = 0; i < faulted.report.count && !first; ++i)
                    if (faulted.report.lines[i].verdict != Verdict::AsRecorded)
                        first = &faulted.report.lines[i];
                char example[200] = "every site as recorded";
                if (first)
                    std::snprintf(example, sizeof(example), "e.g. %s: recorded +0x%X, found +0x%X, %u match(es), %s%s%s",
                                  first->name, first->recorded, first->found, first->matches,
                                  drive_resolve::VerdictText(first->verdict), first->note ? ": " : "",
                                  first->note ? first->note : "");
                std::printf("   fault 0x%02X (%s): %s; %u as recorded, %u moved, %u at the record among several, %u not "
                            "found; %s -- %s%s\n",
                            test.faults, test.what, faulted.ok ? "HOOKED" : "NOT hooked",
                            faulted.report.Count(Verdict::AsRecorded), faulted.report.Count(Verdict::Moved),
                            faulted.report.Count(Verdict::RecordAmongSeveral), faulted.report.NotFound(), example,
                            failure ? "UNEXPECTED: " : "as it must", failure ? failure : "");
            }
        }
        if (mapped)
            FreeLibrary(mapped);
        if (!mappedCopy.empty())
            DeleteFileW(mappedCopy.c_str());
        if (!any)
            std::printf("-- no direct drive: nothing in this title's game DLL is hooked or resolved for one\n");
        if (shifted.base)
            VirtualFree(shifted.base, 0, MEM_RELEASE);
        VirtualFree(module.base, 0, MEM_RELEASE);
    }
    if (faults)
    {
        std::printf("\nFAULTS (each forced on every drive):\n");
        for (const FaultCase& test : kFaultCases)
            std::printf("   0x%02X %s -> %s\n", test.faults, test.what, test.expect);
        std::printf("   %u checks, %u unexpected\n", faultChecks, faultFailures);
    }
    if (moved)
        std::printf("\nMOVED BUILD (every section 0x%X higher, so no recorded RVA is right): %u of %u sites followed; %u "
                    "drives did not follow\n", kMovedShift, movedFollowed, movedSites, movedFailures);
    if (!loadedFolder.empty())
        std::printf("\nAS THE LOADER MAPS THEM (a copy of each DLL mapped with its code not run, resolved through the "
                    "DLL's own side): %u drives, %u sites; %u drives differed from the file\n",
                    loadedDrives, loadedSites, loadedFailures);
    std::printf("\n%u game DLLs, %u drives, %u sites: %u not as recorded, %u drives that could not be hooked\n", modules,
                drives, sites, notAsRecorded, unhooked);
    if (!modules)
        return 2;
    if (faultFailures || movedFailures || loadedFailures)
        return 1;
    if (anyBuild)
        return 0;
    std::printf("%s\n", notAsRecorded || unhooked
                            ? "ACCEPTANCE FAILED: on the recorded build every site must resolve as recorded"
                            : "ACCEPTANCE: every site of every drive resolves where the tables record it");
    return notAsRecorded || unhooked ? 1 : 0;
}
