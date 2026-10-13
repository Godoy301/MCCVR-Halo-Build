// The DLL's side of the direct drives' address resolution (src/dll/drive_cascade.cpp), as shipped, run on a module
// that is really loaded: this test's own executable. What runs here that the synthetic-image tests cannot reach:
// the module is opened through VirtualQuery and its own headers, its .pdata is read where the loader mapped it
// and compared with what the operating system answers for the same addresses, its TLS index is taken from its PE
// TLS directory, a pattern is found through one of "this mod's hooks" (a patch reported by the hook-patch
// source), the address cache's file is really written, read back, replaced, refused and found damaged on disk, the one line
// that says which rung won is really formatted, the proofs an install makes after its plan are run on a module
// whose unwind info cannot be read (they must fail, not fault), and the hook watchdog is stepped as the cold
// worker steps it. It maps no game code and opens no game file.
//
// Built twice by CMake: as a player build compiles the file (no fault injection: the dev key must do nothing) and
// as a dev build does (HALOMCCVR_DEV_DIAGNOSTICS=1, ADDRCASCADE_FAULTS=1: each forced fault must show).
#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/common/config.h"
#include "../src/dll/drive_cascade.h"
#include "../src/dll/drive_install.h"
#include "../src/dll/sigscan.h"

#ifndef HALOMCCVR_DEV_DIAGNOSTICS
#define HALOMCCVR_DEV_DIAGNOSTICS 0
#endif

namespace
{
unsigned checks{}, failures{};
void Check(bool ok, const char* what)
{
    ++checks;
    if (!ok)
    {
        ++failures;
        std::printf("FAIL: %s\n", what);
    }
}

std::vector<std::string> lines;
bool printLines{};
size_t Lines(const char* text)
{
    size_t n = 0;
    for (const std::string& line : lines)
        n += line.find(text) != std::string::npos ? 1u : 0u;
    return n;
}
} // namespace

void Logf(const char* format, ...)
{
    char text[2048];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    lines.emplace_back(text);
    if (printLines)
        std::printf("    | %s\n", text);
}

// Forty bytes in a section of their own that the image marks executable, so the search covers them. No
// instruction stream reads like them, and nothing else in the image holds them.
#pragma section(".drvmk", read, execute)
__declspec(allocate(".drvmk")) extern const uint8_t g_marker[40] = {
    0x4D, 0xC3, 0x56, 0x52, 0xD1, 0x7E, 0x0B, 0xA5, 0x93, 0x1C, 0xE8, 0x47, 0x2F, 0x60, 0xBD, 0x19,
    0x84, 0xF2, 0x3A, 0x75, 0xC9, 0x0E, 0x51, 0xAC, 0x66, 0xDB, 0x12, 0x9F, 0x38, 0xE1, 0x7A, 0x05,
    0xB4, 0x2D, 0x91, 0x6C, 0xF7, 0x40, 0x8B, 0x23};

// A thread-local the test reads, so this image has a PE TLS directory (and the loader's _tls_index is its index).
thread_local int t_touched = 0;
extern "C" unsigned long _tls_index;

namespace
{
using drive_known::Rung;

uintptr_t g_base{};
size_t g_size{};
uint32_t Rva(const void* p) { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p) - g_base); }

// The drive every resolution here is made for. This executable is not a build any title's tables were measured
// on, so nothing is ever verified at recorded addresses: each resolution is a search, or saved addresses.
constexpr drive_cascade::Drive kHalo3{GameTitle::Halo3, "control-update"};
constexpr drive_cascade::Drive kHalo4{GameTitle::Halo4, "pilot-controller"};
constexpr drive_cascade::Drive kReach{GameTitle::HaloReach, "control-update"};

// One of "this mod's hooks", as the hook-patch source reports it: the five bytes at the marker's start are
// overwritten in the loaded image, and what stood there is what a scan must read.
sig::HookPatch g_patch{};
bool g_patched{};
uint32_t PatchSource(uintptr_t base, size_t size, sig::HookPatch* out, uint32_t capacity)
{
    if (!g_patched || g_patch.address + g_patch.size <= base || g_patch.address >= base + size)
        return 0;
    if (capacity)
        out[0] = g_patch;
    return 1;
}

bool WriteCode(void* at, const void* bytes, size_t size)
{
    DWORD old = 0;
    if (!VirtualProtect(at, size, PAGE_EXECUTE_READWRITE, &old))
        return false;
    std::memcpy(at, bytes, size);
    DWORD ignored = 0;
    VirtualProtect(at, size, old, &ignored);
    FlushInstructionCache(GetCurrentProcess(), at, size);
    return true;
}

std::string MarkerPattern()
{
    std::string text;
    char byte[4];
    for (size_t i = 0; i < sizeof(g_marker); ++i)
    {
        std::snprintf(byte, sizeof(byte), "%02X", static_cast<const volatile uint8_t*>(g_marker)[i]);
        text += i ? " " : "";
        text += byte;
    }
    return text;
}

// A function with a frame, for the function-table checks.
__declspec(noinline) int Framed(int depth)
{
    char text[64];
    std::snprintf(text, sizeof(text), "%d", depth);
    return depth > 0 ? Framed(depth - 1) + static_cast<int>(std::strlen(text)) : 0;
}

struct Found
{
    uint32_t marker = 0;
    uint32_t tls = 0;
    uint32_t tlsWrong = 0;
    bool framedIsStart = false, framedInsideIsStart = false;
    uint32_t framedStart = 0;
    uint32_t tableChecked = 0, tableAgreed = 0;
    uint32_t tlsDirectory = 0;
    uint32_t executableRanges = 0;
    drive_resolve::Verdict verdict = drive_resolve::Verdict::None;
};

// The plan a drive's install would run: one signature, the TLS index against an operand, and the function table.
Found Plan(drive_resolve::Resolver& r, uint32_t markerRecorded, const std::string& pattern, uintptr_t framed)
{
    Found f;
    f.marker = r.Unique("MARKER", markerRecorded, pattern.c_str());
    f.verdict = r.report().count ? r.report().lines[r.report().count - 1].verdict : drive_resolve::Verdict::None;
    f.tlsDirectory = r.image().tlsIndexRva;
    f.executableRanges = r.image().executableCount;
    f.tls = r.TlsIndex("TLS index", Rva(&_tls_index), Rva(&_tls_index));
    f.tlsWrong = r.TlsIndex("TLS index (another operand)", Rva(&_tls_index), Rva(&_tls_index) + 8);
    const uint32_t framedRva = static_cast<uint32_t>(framed - g_base);
    f.framedStart = drive_resolve::FunctionStartOf(r.image(), framedRva + 2);
    f.framedIsStart = r.IsFunctionStart(framedRva);
    f.framedInsideIsStart = r.IsFunctionStart(framedRva + 2);
    // THE FUNCTION TABLE IS THE OPERATING SYSTEM'S: at addresses spread over the code, the entry the resolution
    // reads from .pdata is the one RtlLookupFunctionEntry answers with.
    for (uint32_t i = 0; i < r.image().executableCount; ++i)
    {
        const drive_resolve::Range range = r.image().executable[i];
        for (uint32_t at = range.rva; at < range.rva + range.size; at += 0x397)
        {
            DWORD64 imageBase = 0;
            const PRUNTIME_FUNCTION os = RtlLookupFunctionEntry(g_base + at, &imageBase, nullptr);
            const drive_resolve::RuntimeFunction* ours = drive_resolve::EntryAt(r.image(), at);
            ++f.tableChecked;
            if (os && imageBase == g_base ? (ours && ours->begin == os->BeginAddress && ours->end == os->EndAddress)
                                          : ours == nullptr)
                ++f.tableAgreed;
        }
    }
    return f;
}

bool Run(drive_cascade::Resolution<Found>& out, uint32_t markerRecorded, const std::string& pattern, uintptr_t framed)
{
    return drive_cascade::Resolve(g_base, g_size, kHalo3, out, [&](drive_resolve::Resolver& r) {
        return Plan(r, markerRecorded, pattern, framed);
    });
}

void TestOpenAndResolve()
{
    drive_cascade::SetCacheFile(nullptr);   // no file: every resolution here is a search
    const std::string pattern = MarkerPattern();
    const uint32_t markerRva = Rva(g_marker);
    // The function's own start, as the operating system has it (an incremental-link thunk is not the function).
    DWORD64 imageBase = 0;
    const PRUNTIME_FUNCTION entry = RtlLookupFunctionEntry(reinterpret_cast<DWORD64>(&Framed), &imageBase, nullptr);
    const uintptr_t framed = entry ? g_base + entry->BeginAddress : 0;
    Check(entry && imageBase == g_base && framed, "the test's framed function has an unwind entry in this image");
    Check(Framed(3) == 3, "the framed function runs");

    drive_cascade::Resolution<Found> found;
    Check(Run(found, markerRva, pattern, framed) && !found.why, "this executable opens as a module");
    Check(!found.rungs.recordedBuild && found.rungs.outcome.rung == Rung::Search && !found.rungs.outcome.recordedTried,
          "it is no title's recorded build, so nothing is taken from recorded addresses: it is searched");
    Check(found.found.marker == markerRva && found.found.verdict == drive_resolve::Verdict::AsRecorded,
          "a signature is found in the loaded image, where it is recorded");
    Check(found.found.executableRanges >= 2, "the code section and the marker's own section are both searched");
    Check(found.found.tlsDirectory == Rva(&_tls_index), "the TLS index is read from the image's PE TLS directory");
    Check(found.found.tls == Rva(&_tls_index), "an operand that names the directory's index is the TLS index");
    Check(found.found.tlsWrong == 0, "an operand that names another address is refused");
    Check(found.found.framedIsStart && !found.found.framedInsideIsStart &&
              found.found.framedStart == static_cast<uint32_t>(framed - g_base),
          "a function's start is read from the loaded image's .pdata");
    Check(found.found.tableChecked > 16 && found.found.tableAgreed == found.found.tableChecked,
          "at every address tried, the .pdata entry is the one the operating system's lookup answers with");

    // Recorded somewhere else: the search wins, and says the code moved.
    drive_cascade::Resolution<Found> moved;
    Check(Run(moved, markerRva + 0x10, pattern, framed) && moved.found.marker == markerRva &&
              moved.found.verdict == drive_resolve::Verdict::Moved,
          "a wrong recorded RVA: the signature is still found where it is, and reported MOVED");

    // A signature that is not in the image.
    std::string absent = pattern;
    absent[0] = absent[0] == '0' ? '1' : '0';
    drive_cascade::Resolution<Found> missing;
    Check(Run(missing, markerRva, absent, framed) && missing.found.marker == 0 &&
              missing.found.verdict == drive_resolve::Verdict::Missing,
          "a signature that is nowhere in the image is not found, and nothing stands in for it");
    char first[200];
    drive_cascade::FirstFailure(missing.report, first, sizeof(first));
    Check(std::strstr(first, "MARKER") != nullptr, "the first failure names the anchor");

    // THROUGH ONE OF THIS MOD'S HOOKS. The marker's first five bytes are overwritten in the loaded image, as an
    // enabled hook overwrites the function it is on; the hook-patch source reports what stood there.
    uint8_t original[5];
    std::memcpy(original, g_marker, sizeof(original));
    const uint8_t jump[5] = {0xE9, 0x11, 0x22, 0x33, 0x44};
    g_patch.address = reinterpret_cast<uintptr_t>(g_marker);
    g_patch.size = 5;
    std::memcpy(g_patch.original, original, sizeof(original));
    void* const markerAt = const_cast<void*>(static_cast<const void*>(g_marker));
    Check(WriteCode(markerAt, jump, sizeof(jump)), "the marker's start is overwritten as a hook would");
    drive_cascade::Resolution<Found> blind;
    Check(Run(blind, markerRva, pattern, framed) && blind.found.marker == 0,
          "with no word of the hook, the overwritten bytes are not the signature");
    g_patched = true;
    sig::SetHookPatchSource(&PatchSource);
    drive_cascade::Resolution<Found> through;
    Check(Run(through, markerRva, pattern, framed) && through.found.marker == markerRva &&
              through.found.verdict == drive_resolve::Verdict::AsRecorded,
          "under one of this mod's enabled hooks the game's own bytes are what is matched");
    g_patched = false;
    sig::SetHookPatchSource(nullptr);
    Check(WriteCode(markerAt, original, sizeof(original)), "the marker is put back");

    // What cannot be opened is refused with a reason, and nothing is resolved.
    drive_cascade::Resolution<Found> none;
    Check(!drive_cascade::Resolve(uintptr_t{0}, g_size, kHalo3, none, [&](drive_resolve::Resolver& r) {
        return Plan(r, markerRva, pattern, framed);
    }) && none.why && none.report.count == 0, "no module: not opened, with the reason");
    static uint8_t notAnImage[0x2000];
    drive_cascade::Resolution<Found> junk;
    Check(!drive_cascade::Resolve(reinterpret_cast<uintptr_t>(notAnImage), sizeof(notAnImage), kHalo3, junk,
                                  [&](drive_resolve::Resolver& r) { return Plan(r, markerRva, pattern, framed); }) &&
              junk.why && junk.report.count == 0,
          "memory that is not a module: not opened, with the reason");
}

// The two-line drive the rest of the tests resolve: a signature, and the TLS index. `recorded`: what its table
// records for the signature (another value is another version of the table).
struct Simple
{
    uint32_t marker = 0, tls = 0;
};
using Resolved = drive_cascade::Resolution<Simple>;

bool Resolve(Resolved& out, const drive_cascade::Drive& drive, uint32_t recorded)
{
    static const std::string pattern = MarkerPattern();
    return drive_cascade::Resolve(g_base, g_size, drive, out, [&](drive_resolve::Resolver& r) {
        Simple f;
        f.marker = r.Unique("MARKER", recorded, pattern.c_str());
        f.tls = r.TlsIndex("TLS index", Rva(&_tls_index), Rva(&_tls_index));
        return f;
    });
}

bool Whole(const Resolved& r) { return r.found.marker == Rva(g_marker) && r.found.tls == Rva(&_tls_index); }

void TestAnnounceAndFaults()
{
    drive_cascade::SetCacheFile(nullptr);
    const uint32_t markerRva = Rva(g_marker);
    addrcascade::TierReporter reporter;
    Resolved healthy;
    lines.clear();
    Check(Resolve(healthy, kHalo3, markerRva) && Whole(healthy) && healthy.rungs.outcome.learn && !healthy.rungs.saved,
          "a healthy resolution, with no cache file to save to");
    drive_cascade::Announce("TEST DRIVE", healthy, reporter);
    Check(Lines("TEST DRIVE: found by search: of its 2 addresses 2 are where the tables record them, 0 MOVED, 0 were "
                "taken at the recorded address among several matches; NOT saved for the next launch (the mod's folder "
                "is not known)") == 1,
          "which rung won is said: found by search, and why it was not saved");
    drive_cascade::Announce("TEST DRIVE", healthy, reporter);
    Check(lines.size() == 1, "and not said again while nothing changed");

    // The dev key, in each flavour.
    struct Case
    {
        int mask;
        const char* what;
    };
    const Case cases[] = {{0x01, "a signature finds nothing"},
                          {0x02, "every signature is ambiguous"},
                          {0x04, "every recorded RVA is wrong"},
                          {0x06, "ambiguous, with a wrong record"},
                          {0x08, "the PE TLS directory disagrees"},
                          {0x40, "the saved addresses are stale"}};
    for (const Case& c : cases)
    {
        g_config.drive_fault = c.mask;
        Resolved forced;
        lines.clear();
        const bool opened = Resolve(forced, kHalo3, markerRva);
        drive_cascade::Announce("TEST DRIVE", forced, reporter);
        const drive_resolve::Verdict verdict = forced.report.lines[0].verdict;
        char what[160];
#if HALOMCCVR_DEV_DIAGNOSTICS
        using drive_resolve::Verdict;
        bool as = false;
        switch (c.mask)
        {
        case 0x01: as = forced.found.marker == 0 && verdict == Verdict::Missing; break;
        case 0x02: as = forced.found.marker == markerRva && verdict == Verdict::RecordAmongSeveral; break;
        case 0x04: as = forced.found.marker == markerRva && verdict == Verdict::Moved; break;
        case 0x06: as = forced.found.marker == 0 && verdict == Verdict::Ambiguous; break;
        case 0x08: as = forced.found.marker == markerRva && forced.found.tls == 0; break;
        case 0x40:
            as = Whole(forced) && forced.rungs.outcome.savedTried &&
                 forced.rungs.savedRefused == drive_known::kSavedStale && forced.rungs.outcome.rung == Rung::Search &&
                 Lines("the addresses saved earlier were not used (this game DLL does not have that code at them)") == 1;
            break;
        }
        std::snprintf(what, sizeof(what), "dev build, drive_fault 0x%02X (%s): the resolution shows it", c.mask, c.what);
        Check(opened && as, what);
        std::snprintf(what, sizeof(what), "dev build, drive_fault 0x%02X: the line says the result is forced", c.mask);
        Check(Lines("DEV fault injection (drive_fault") == 1 && lines.size() == 2, what);
        std::snprintf(what, sizeof(what), "dev build, drive_fault 0x%02X: nothing is saved under a forced fault", c.mask);
        Check(!forced.rungs.saved, what);
#else
        std::snprintf(what, sizeof(what), "player build, drive_fault 0x%02X (%s): the key does nothing", c.mask, c.what);
        Check(opened && Whole(forced) && verdict == drive_resolve::Verdict::AsRecorded && lines.empty(), what);
#endif
    }
    g_config.drive_fault = 0;
    Resolved again;
    lines.clear();
    Check(Resolve(again, kHalo3, markerRva) && Whole(again), "healthy again");
    drive_cascade::Announce("TEST DRIVE", again, reporter);
#if HALOMCCVR_DEV_DIAGNOSTICS
    Check(Lines("TEST DRIVE: found by search: of its 2 addresses 2 are where the tables record them") == 1 &&
              Lines("DEV fault injection") == 0,
          "back to healthy is a change, and is said, without the forced-result line");
#else
    Check(lines.empty(), "player build: nothing changed, nothing said");
#endif
}

// ---- the address cache's file, on disk ---------------------------------------------------------------------------
std::string ReadAll(const std::wstring& path)
{
    std::string text;
    const HANDLE handle = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
        return text;
    char buffer[4096];
    DWORD read = 0;
    while (ReadFile(handle, buffer, sizeof(buffer), &read, nullptr) && read)
        text.append(buffer, read);
    CloseHandle(handle);
    return text;
}

bool WriteAll(const std::wstring& path, const std::string& text)
{
    const HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, 0, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
        return false;
    DWORD written = 0;
    const bool ok = WriteFile(handle, text.data(), static_cast<DWORD>(text.size()), &written, nullptr) &&
                    written == text.size();
    CloseHandle(handle);
    return ok;
}

bool Exists(const std::wstring& path) { return GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES; }

// The cache file's text with one drive's entry changed and the file written out again as the mod writes it, so
// that the entry's checksum is its own: what a file left by another build of the game, or by another version of
// this mod, holds.
std::string Rewritten(const std::string& text, const char* module, const char* id, void (*change)(drive_known::Entry&))
{
    static drive_known::File file;
    static char out[drive_known::kMaxFileText];
    drive_known::Parse(text.data(), text.size(), file);
    for (uint32_t i = 0; i < file.count; ++i)
        if (!std::strcmp(file.entries[i].module, module) && !std::strcmp(file.entries[i].id, id))
            change(file.entries[i]);
    return std::string(out, drive_known::Format(file, out, sizeof(out)));
}

void TestCacheFile()
{
    const uint32_t markerRva = Rva(g_marker);
    wchar_t temp[MAX_PATH];
    GetTempPathW(MAX_PATH, temp);
    const std::wstring folder = std::wstring(temp) + L"halomccvr_drive_cascade_tests_" +
                                std::to_wstring(GetCurrentProcessId()) + L"\\";
    CreateDirectoryW(folder.c_str(), nullptr);
    const std::wstring cache = folder + L"HaloMCCVR-drive-addresses.txt";
    const std::wstring temporary = cache + L".tmp";
    DeleteFileW(cache.c_str());
    drive_cascade::SetCacheFolder(folder.c_str());
    g_config.drive_address_cache = 1;
    g_config.drive_fault = 0;

    // FIRST LAUNCH on a build no table describes: searched, and saved.
    Resolved first;
    lines.clear();
    addrcascade::TierReporter reporter;
    Check(Resolve(first, kHalo3, markerRva) && Whole(first) && first.rungs.outcome.rung == Rung::Search &&
              first.rungs.outcome.scans == 1 && first.rungs.outcome.learn && first.rungs.saved && !first.rungs.notSaved,
          "first launch: found by search (one scan of the module), and saved");
    drive_cascade::Announce("TEST DRIVE", first, reporter);
    Check(Lines("TEST DRIVE: found by search: of its 2 addresses 2 are where the tables record them, 0 MOVED, 0 were "
                "taken at the recorded address among several matches; saved for the next launch") == 1,
          "and says so");
    const std::string saved = ReadAll(cache);
    Check(Exists(cache) && !Exists(temporary) && saved.find("NOT a setting") != std::string::npos &&
              saved.find("Safe to DELETE") != std::string::npos && saved.find("schema=2\r\n") != std::string::npos &&
              saved.find("entry halo3.dll ") != std::string::npos && saved.find(" control-update ") != std::string::npos,
          "the file is beside the mod's log, says what it is, and holds the drive's entry; no temporary file is left");

    // SECOND LAUNCH: verified at the saved addresses, the module not searched at all.
    Resolved second;
    lines.clear();
    Check(Resolve(second, kHalo3, markerRva) && Whole(second) && second.rungs.outcome.rung == Rung::Saved &&
              second.rungs.outcome.scans == 0 && second.report.scans == 0 && !second.rungs.saved &&
              second.report.lines[0].how == drive_resolve::How::Known,
          "second launch: verified at the saved addresses with zero scans of the module");
    drive_cascade::Announce("TEST DRIVE", second, reporter);
    Check(Lines("TEST DRIVE: verified at the addresses saved from an earlier launch of this game build, all 2 of them, "
                "with no search (0 of them are not where the tables record them)") == 1,
          "and says so");
    Check(ReadAll(cache) == saved, "a launch that only verifies leaves the file as it was");

    // ANOTHER TITLE'S DRIVE: its entry is added, and the first title's is kept.
    Resolved other;
    Check(Resolve(other, kHalo4, markerRva) && other.rungs.outcome.rung == Rung::Search && other.rungs.saved,
          "another title's drive is searched and saved");
    const std::string both = ReadAll(cache);
    Check(both.find("entry halo3.dll ") != std::string::npos && both.find("entry halo4.dll ") != std::string::npos &&
              both.find(" pilot-controller ") != std::string::npos,
          "two titles' entries coexist in the file");
    Resolved again3, again4;
    Check(Resolve(again3, kHalo3, markerRva) && again3.rungs.outcome.rung == Rung::Saved && Whole(again3) &&
              Resolve(again4, kHalo4, markerRva) && again4.rungs.outcome.rung == Rung::Saved && Whole(again4) &&
              again3.rungs.outcome.scans + again4.rungs.outcome.scans == 0,
          "and each is verified from it at its next launch, with no search");

    // ANOTHER BUILD'S ENTRY (the stamp differs): not tried, said, searched, and replaced.
    {
        const std::string stale = Rewritten(both, "halo3.dll", "control-update",
                                            [](drive_known::Entry& e) { e.stamp ^= 1ull << 32; });
        Check(stale != both && stale.size() == both.size() && WriteAll(cache, stale),
              "the file is given an entry of another build");
        Resolved r;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(r, kHalo3, markerRva) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  !r.rungs.outcome.savedTried && r.rungs.savedRefused && r.rungs.saved,
              "an entry of another build of the DLL is not tried: the drive is searched, and saved again");
        drive_cascade::Announce("TEST DRIVE", r, fresh);
        Check(Lines("the addresses saved earlier were not used (they are of another build of this game DLL)") == 1,
              "one line says the saved addresses were of another build");
        Check(ReadAll(cache) == both, "the file again holds this build's entry, and the other title's untouched");
    }

    // FOUND UNDER OTHER RULES OF THE SEARCH: not tried, said, searched, and replaced.
    {
        const std::string aged =
            Rewritten(both, "halo3.dll", "control-update", [](drive_known::Entry& e) { ++e.rules; });
        Check(aged != both && WriteAll(cache, aged), "the file is given an entry found under other rules of the search");
        Resolved r;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(r, kHalo3, markerRva) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  !r.rungs.outcome.savedTried && r.rungs.savedRefused == drive_known::kOtherRules && r.rungs.saved,
              "an entry found under other rules is not tried, though every address in it would verify: the drive is "
              "searched, and saved again");
        drive_cascade::Announce("TEST DRIVE", r, fresh);
        Check(Lines("the addresses saved earlier were not used (they were found under other rules of this mod's "
                    "search)") == 1,
              "one line says the saved addresses were found under other rules");
        Check(ReadAll(cache) == both, "the file again holds an entry under this mod's rules, and the other title's untouched");
    }

    // A DAMAGED ENTRY (one digit of it changed, so its checksum is no longer its own): as if it were not there.
    {
        const size_t at = both.find("entry halo3.dll ") + 16;
        std::string damaged = both;
        damaged[at] = damaged[at] == '1' ? '2' : '1';   // the first digit of the stamp
        Check(WriteAll(cache, damaged), "one digit of an entry in the file is changed");
        Resolved r;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(r, kHalo3, markerRva) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  !r.rungs.outcome.savedTried && !r.rungs.savedRefused && r.rungs.saved,
              "an entry whose checksum is not its own is not read at all: the drive is searched as on a first launch");
        drive_cascade::Announce("TEST DRIVE", r, fresh);
        Check(Lines("the addresses saved earlier were not used") == 0 && Lines("saved for the next launch") == 1,
              "nothing is said of it: there was no entry");
        Resolved halo3Again, halo4Again;
        Check(Resolve(halo3Again, kHalo3, markerRva) && halo3Again.rungs.outcome.rung == Rung::Saved &&
                  Resolve(halo4Again, kHalo4, markerRva) && halo4Again.rungs.outcome.rung == Rung::Saved,
              "the drive's entry is whole again, and the other title's was kept through it");
    }

    // ANOTHER VERSION OF THE DRIVE'S TABLES (the plan differs): verified bytes are not enough; searched, replaced.
    {
        Resolved r;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(r, kHalo3, markerRva + 0x10) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  r.rungs.outcome.savedTried && r.rungs.savedRefused == drive_known::kSavedOtherPlan && r.rungs.saved,
              "an entry saved for another version of the tables is not used: the drive is searched, and saved again");
        drive_cascade::Announce("TEST DRIVE", r, fresh);
        Check(Lines("the addresses saved earlier were not used (they were saved for another version of this drive's "
                    "tables)") == 1 && Lines("MARKER MOVED from") == 1,
              "one line says so, and names what the search found moved");
        Resolved next;
        Check(Resolve(next, kHalo3, markerRva + 0x10) && next.rungs.outcome.rung == Rung::Saved && Whole(next) &&
                  next.report.lines[0].verdict == drive_resolve::Verdict::Moved,
              "the next launch with those tables verifies the new entry, still reported as moved");
        Check(ReadAll(cache).find("entry halo4.dll ") != std::string::npos, "the other title's entry is still there");
    }

    // A FILE THAT IS NOT ONE: a cold start, and a whole file again afterwards.
    for (const std::string& broken : {std::string("\x01\x02 not a cache \xFF\r\n"), std::string(), both.substr(0, both.size() / 2),
                                      std::string("schema=7\r\n") + both.substr(both.find("entry halo3.dll "))})
    {
        Check(WriteAll(cache, broken), "the file is replaced by something that is not a cache");
        Resolved r;
        Check(Resolve(r, kHalo3, markerRva) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  !r.rungs.outcome.savedTried && r.rungs.saved,
              "a corrupted, empty, truncated or foreign-schema file is a cold start: searched, and saved");
        const std::string after = ReadAll(cache);
        Check(after.find("schema=2\r\n") != std::string::npos && after.find("entry halo3.dll ") != std::string::npos &&
                  after.back() == '\n',
              "and the file is whole again");
    }
    {
        // Cut short inside its LAST entry: the whole entries before it are still used.
        Resolved seed;
        (void)Resolve(seed, kHalo4, markerRva);   // halo3's entry, then halo4's
        const std::string two = ReadAll(cache);
        Check(two.find("entry halo3.dll ") < two.find("entry halo4.dll "), "two entries, halo4's last");
        Check(WriteAll(cache, two.substr(0, two.size() - 4)), "the file loses its last four bytes");
        Resolved kept, cut;
        Check(Resolve(kept, kHalo3, markerRva) && kept.rungs.outcome.rung == Rung::Saved,
              "a whole entry in a file that was cut short is still verified and used");
        Check(Resolve(cut, kHalo4, markerRva) && cut.rungs.outcome.rung == Rung::Search && cut.rungs.saved,
              "the entry that was cut is not: that drive is searched and saved again");
    }

    // THE SETTING OFF: the file is neither read nor written.
    {
        const std::string before = ReadAll(cache);
        g_config.drive_address_cache = 0;
        Resolved r;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(r, kHalo3, markerRva) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  !r.rungs.outcome.savedTried && !r.rungs.savedRefused && !r.rungs.saved && r.rungs.notSaved,
              "drive_address_cache = 0: a good entry is not read, and the drive is searched");
        drive_cascade::Announce("TEST DRIVE", r, fresh);
        Check(Lines("NOT saved for the next launch (drive_address_cache = 0)") == 1, "the line says why nothing was saved");
        Check(ReadAll(cache) == before, "and the file is not written");
        DeleteFileW(cache.c_str());
        Resolved none;
        Check(Resolve(none, kHalo3, markerRva) && Whole(none) && !Exists(cache) && !Exists(temporary),
              "with no file and the setting off, none is created");
        g_config.drive_address_cache = 1;
    }

    // A WRITE THAT FAILS LEAVES THE OLD FILE WHOLE.
    {
        Resolved seed;
        Check(Resolve(seed, kHalo3, markerRva) && seed.rungs.saved, "a good file");
        const std::string before = ReadAll(cache);
        Check(CreateDirectoryW(temporary.c_str(), nullptr), "the temporary file's name is taken by a folder");
        Resolved r;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(r, kHalo3, markerRva + 0x20) && Whole(r) && r.rungs.outcome.rung == Rung::Search &&
                  r.rungs.outcome.learn && !r.rungs.saved && r.rungs.notSaved,
              "the drive is resolved all the same; only the saving failed");
        drive_cascade::Announce("TEST DRIVE", r, fresh);
        Check(Lines("NOT saved for the next launch (the file could not be written)") == 1, "the line says so");
        Check(ReadAll(cache) == before, "the old file is whole, byte for byte");
        RemoveDirectoryW(temporary.c_str());
        Resolved retry;
        Check(Resolve(retry, kHalo3, markerRva + 0x20) && retry.rungs.saved && !Exists(temporary),
              "and the next launch saves it");
    }

    // THE DEV KEY AND THE FILE.
    {
        Resolved seed;
        Check(Resolve(seed, kHalo3, markerRva) && seed.rungs.saved, "the file holds this drive's entry for these tables");
        const std::string before = ReadAll(cache);
        g_config.drive_fault = 0x04;
        Resolved forced;
        Check(Resolve(forced, kHalo3, markerRva) && Whole(forced), "a resolution with drive_fault 0x04");
#if HALOMCCVR_DEV_DIAGNOSTICS
        Check(forced.rungs.outcome.rung == Rung::Search && !forced.rungs.outcome.savedTried && !forced.rungs.saved &&
                  forced.rungs.notSaved && forced.report.lines[0].verdict == drive_resolve::Verdict::Moved,
              "dev build: under a forced fault the file is not read, and the forced result is not saved");
        Check(ReadAll(cache) == before, "dev build: a forced fault never writes the file");
        g_config.drive_fault = 0x80;
        Resolved tested;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(Resolve(tested, kHalo3, markerRva) && Whole(tested) && tested.rungs.outcome.rung == Rung::Saved &&
                  tested.rungs.outcome.searchedToo && tested.rungs.outcome.searchAgrees &&
                  tested.rungs.outcome.scans == 1,
              "dev build, self-test 0x80: the saved addresses are verified and used, and the search is run beside them");
        drive_cascade::Announce("TEST DRIVE", tested, fresh);
        Check(Lines("DEV self-test (drive_fault 0x80): the search was run as well, and it chose the same address for "
                    "every line") == 1 && Lines("is forcing what the next line reports") == 0,
              "dev build: and the log says the two agree");
        Check(ReadAll(cache) == before, "dev build: the self-test writes nothing");
#else
        Check(forced.rungs.outcome.rung == Rung::Saved && forced.rungs.outcome.scans == 0 &&
                  forced.report.lines[0].verdict == drive_resolve::Verdict::AsRecorded,
              "player build: the dev key does nothing, and the saved addresses are verified as ever");
        Check(ReadAll(cache) == before, "player build: nothing written");
#endif
        g_config.drive_fault = 0;
    }

    // A LINE SAVED AS THE SINGLE MATCH IN ITS FUNCTION: the next launch searches that one function again, and
    // says so.
    {
        DWORD64 imageBase = 0;
        const PRUNTIME_FUNCTION entry = RtlLookupFunctionEntry(reinterpret_cast<DWORD64>(&Framed), &imageBase, nullptr);
        const uint32_t start = entry ? entry->BeginAddress : 0, end = entry ? entry->EndAddress : 0;
        // Twelve bytes of the framed function's own body that stand in it once. The plan records them four bytes
        // further on, where they are not: the search finds them off their offset, as the function's single match.
        const auto* const code = reinterpret_cast<const uint8_t*>(g_base);
        uint32_t at = 0;
        for (uint32_t candidate = start + 4; start && candidate + 16 <= end && !at; ++candidate)
        {
            uint32_t matches = 0;
            for (uint32_t other = start; other + 12 <= end; ++other)
                matches += !std::memcmp(code + other, code + candidate, 12) ? 1u : 0u;
            if (matches == 1)
                at = candidate;
        }
        Check(at != 0, "the framed function holds twelve bytes that stand in it once");
        std::string row;
        char byte[4];
        for (uint32_t i = 0; at && i < 12; ++i)
        {
            std::snprintf(byte, sizeof(byte), "%02X", code[at + i]);
            row += i ? " " : "";
            row += byte;
        }
        struct WithRow
        {
            uint32_t marker = 0, row = 0;
        };
        const auto resolve = [&](drive_cascade::Resolution<WithRow>& out) {
            static const std::string marker = MarkerPattern();
            return drive_cascade::Resolve(g_base, g_size, kReach, out, [&](drive_resolve::Resolver& r) {
                WithRow f;
                f.marker = r.Unique("MARKER", markerRva, marker.c_str());
                f.row = r.InFunction("ROW", at + 4, start, start, row.c_str());
                return f;
            });
        };
        drive_cascade::Resolution<WithRow> first, next;
        lines.clear();
        addrcascade::TierReporter fresh;
        Check(at && resolve(first) && first.found.row == at && first.rungs.outcome.rung == Rung::Search &&
                  first.report.lines[1].how == drive_resolve::How::FunctionExtent && first.rungs.saved,
              "a row that is not at its recorded offset is found as the single match in its function, and saved");
        Check(at && resolve(next) && next.found.row == at && next.found.marker == markerRva &&
                  next.rungs.outcome.rung == Rung::Saved && next.rungs.outcome.scans == 1 && next.report.scans == 1,
              "the next launch verifies it with that one function searched again, and the module not searched");
        drive_cascade::Announce("TEST DRIVE", next, fresh);
        Check(Lines("TEST DRIVE: verified at the addresses saved from an earlier launch of this game build, all 2 of "
                    "them, with no search of the module (1 of them are not where the tables record them; 1 of its "
                    "functions were searched again, each for a line saved as the single match in it)") == 1,
              "and its line says which search was made again");
    }

    drive_cascade::SetCacheFile(nullptr);
    DeleteFileW(cache.c_str());
    RemoveDirectoryW(temporary.c_str());
    RemoveDirectoryW(folder.c_str());
}

// ---- the installs' proofs after the plan (src/dll/drive_install.h) never fault -----------------------------------
// A plan reads the module through checks: a page that cannot be read costs it one line, and it returns with that
// line refused and everything else still set. What an install does next runs with no guard around it. Here the
// dispatcher's unwind info lies on a page that cannot be read.
void TestInstallProofsNeverFault()
{
    // A three-page "module" with a function table of its own: a dispatcher at +0x100 (two pushes and
    // `sub rsp, 0xC0`: a 0xD0 frame) that calls +0x180 from +0x140; its unwind info alone on the third page.
    auto* const image = static_cast<uint8_t*>(VirtualAlloc(nullptr, 0x3000, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    Check(image != nullptr, "three pages for a module of the test's own");
    if (!image)
        return;
    const uintptr_t base = reinterpret_cast<uintptr_t>(image);
    std::memset(image, 0xCC, 0x1000);
    const uint8_t prologue[] = {0x48, 0x8B, 0xC4, 0x55, 0x53, 0x48, 0x81, 0xEC, 0xC0, 0x00, 0x00, 0x00};
    std::memcpy(image + 0x100, prologue, sizeof(prologue));
    image[0x140] = 0xE8;
    const int32_t rel = 0x180 - 0x145;
    std::memcpy(image + 0x141, &rel, sizeof(rel));
    const uint8_t unwind[12] = {0x01, 12, 3, 0, 12, 0x11, 0xD0, 0, 0, 0, 0, 0};   // one ALLOC_LARGE of 0xD0
    std::memcpy(image + 0x2000, unwind, sizeof(unwind));
    static RUNTIME_FUNCTION table[1];
    table[0].BeginAddress = 0x100;
    table[0].EndAddress = 0x170;
    table[0].UnwindData = 0x2000;
    Check(RtlAddFunctionTable(table, 1, base) != 0, "the module's function table is the system's");

    uint32_t frame = 0;
    const char* why = nullptr;
    Check(drive_install::ProveDispatcher(base, 0x3000, 0x100, 0x140, frame, why) && frame == 0xD8,
          "with everything readable the dispatcher's frame is proven from the system's table");

    DWORD old = 0;
    Check(VirtualProtect(image + 0x2000, 0x1000, PAGE_NOACCESS, &old) != 0, "the unwind info's page can no longer be read");
    // THE READ ITSELF IS GUARDED: without the guard this faults, on a thread that has none.
    frame = 1;
    why = nullptr;
    Check(!drive_install::ProveDispatcher(base, 0x3000, 0x100, 0x140, frame, why) && frame == 0 && why != nullptr,
          "unwind info that cannot be read is a failed proof, not a fault");

    // AND NOTHING THE PLAN REFUSED IS READ AGAIN. The plan's result for such a module: the control update and
    // the dispatcher found, the frame refused.
    drive_sites::Callers callers;
    callers.accumulatorCall = 0x140;
    callers.dispatcher = 0x100;
    callers.frame = 0;
    aim_caller::CallerMap map;
    drive_install::CallerMapProof proof =
        drive_install::BuildCallerMap(base, 0x3000, 0x180, callers, drive_sites::kReachCallerAnchors, map);
    Check(!proof.proven && proof.why && std::strstr(proof.why, "the plan did not prove") && !map.stick_proven(),
          "a caller map whose frame the plan refused is not proven, and says that the plan did not prove it");
    callers.frame = 0xD8;   // a plan that did prove it, on a module whose page has since gone
    proof = drive_install::BuildCallerMap(base, 0x3000, 0x180, callers, drive_sites::kReachCallerAnchors, map);
    Check(!proof.proven && proof.why != nullptr, "a frame that can no longer be read is not proven either, with no fault");
    // Roles the plan refused: the map says so, not that the stick update's call failed.
    callers.why = drive_sites::kRolesNotDistinct;
    DWORD ignored = 0;
    VirtualProtect(image + 0x2000, 0x1000, PAGE_READWRITE, &ignored);
    proof = drive_install::BuildCallerMap(base, 0x3000, 0x180, callers, drive_sites::kReachCallerAnchors, map);
    Check(!proof.proven && proof.why == drive_sites::kRolesNotDistinct,
          "a map with no role left says what the plan refused: the roles, not the stick update's call");
    VirtualProtect(image + 0x2000, 0x1000, PAGE_NOACCESS, &ignored);

    // A pilot's drive.
    const drive_sites::Anchor rows[1] = {{"ROW", 0x180, drive_sites::kAnywhere, "CC"}};
    const uint32_t rowsFound[1] = {0x180};
    drive_sites::Flight flight;
    flight.entry = 0x180;
    flight.call = 0x140;
    flight.dispatcher = 0x100;
    flight.tlsIndex = 0x1800;
    flight.frame = 0;
    drive_install::FlightDecision decision =
        drive_install::DecideFlight(base, 0x3000, true, rows, rowsFound, flight, base + 0x1800, false, 1, 0);
    Check(decision.rowsOk && decision.tlsOk && decision.callOk && !decision.frameOk && !decision.hook,
          "a pilot's drive whose frame the plan refused is not hooked, and its unwind info is not read");
    flight.frame = 0xD8;
    decision = drive_install::DecideFlight(base, 0x3000, true, rows, rowsFound, flight, base + 0x1800, false, 1, 0);
    Check(!decision.frameOk && !decision.hook && decision.frameWhy != nullptr,
          "nor is one whose frame can no longer be read: a failed proof, no fault");
    // A call on a page that cannot be read.
    VirtualProtect(image, 0x1000, PAGE_NOACCESS, &ignored);
    Check(drive_install::CallReturn(base, 0x3000, 0x140, 0x180) == 0, "a call that cannot be read lands nowhere");
    VirtualProtect(image, 0x3000, PAGE_READWRITE, &ignored);
    Check(drive_install::CallReturn(base, 0x3000, 0x140, 0x180) == base + 0x145, "and read again when it can be");

    RtlDeleteFunctionTable(table);
    VirtualFree(image, 0, MEM_RELEASE);
}

// The cold worker passes over a watch about every 50 ms.
struct Worker
{
    drive_cascade::HookWatch watch;
    uint64_t now = 1000000;
    // Passes for `ms`, returns how many times the watch fired. `install`: 0 for no hook.
    int Run(uint64_t install, bool possible, bool called, uint32_t ms)
    {
        int fired = 0;
        for (uint32_t t = 0; t < ms; t += 50)
        {
            now += 50;
            fired += watch.Pass(install, possible, called, now) ? 1 : 0;
        }
        return fired;
    }
};

void TestHookWatch()
{
    const int somewhere = 0;
    const uint64_t hookA = drive_cascade::HookWatch::Install(&somewhere, 1);
    // The same address, installed again: another install.
    const uint64_t hookB = drive_cascade::HookWatch::Install(&somewhere, 2);
    Check(hookA && hookB && hookA != hookB && !drive_cascade::HookWatch::Install(nullptr, 3),
          "an install is its address and its number; no hook is none");
    const uint32_t limitMs = drive_cascade::HookWatch::kPeriods * drive_cascade::HookWatch::kPeriodMs;
    {
        Worker w;
        Check(w.Run(0, true, false, 60000) == 0, "no hook: nothing to say");
    }
    {
        Worker w;
        Check(w.Run(hookA, true, false, limitMs - 200) == 0, "installed, possible, uncalled: silent before the limit");
        Check(w.Run(hookA, true, false, 400) == 1, "and said exactly once when the limit is reached");
        Check(w.Run(hookA, true, false, 60000) == 0, "and never again for that install");
        Check(w.Run(hookB, true, false, limitMs + 200) == 1,
              "the same hook installed again is a new install, and is watched afresh");
    }
    {
        Worker w;
        Check(w.Run(hookA, false, false, 600000) == 0, "ten minutes of a menu, a load or a cutscene count for nothing");
        Check(w.Run(hookA, true, false, limitMs - 200) == 0, "and leave nothing behind for the play that follows");
        Check(w.Run(hookA, true, false, 400) == 1, "which is counted from its own beginning");
    }
    {
        Worker w;
        int fired = 0;
        // Play broken by a cutscene every eight seconds: never ten seconds of possible time in a row.
        for (int i = 0; i < 20; ++i)
        {
            fired += w.Run(hookA, true, false, 8000);
            fired += w.Run(hookA, false, false, 2100);
        }
        Check(fired == 0, "possible time is counted only in an unbroken run");
        // One pass in a period that found the call impossible spoils that period.
        Worker one;
        fired = one.Run(hookA, true, false, 7900);
        fired += one.watch.Pass(hookA, false, false, one.now += 50) ? 1 : 0;
        fired += one.Run(hookA, true, false, 6000);
        Check(fired == 0, "a single pass that found the call impossible restarts the count");
    }
    {
        Worker w;
        Check(w.Run(hookA, true, false, 4000) == 0 && w.Run(hookA, true, true, 50) == 0 && w.watch.everCalled(),
              "one call is seen");
        Check(w.Run(hookA, true, false, 600000) == 0, "a hook that was called once is never called dead");
        // Installed again a moment later: the call belonged to the hook before it. (The drive clears its counters
        // when it installs, so `called` is false for the new one.)
        Check(w.Run(hookB, true, false, limitMs + 200) == 1,
              "an install that follows a called one is watched, and is said when it is never called");
    }
    {
        // A stalled worker: two passes a minute apart are one period, not thirty.
        Worker w;
        int fired = w.watch.Pass(hookA, true, false, w.now) ? 1 : 0;
        fired += w.watch.Pass(hookA, true, false, w.now += 60000) ? 1 : 0;
        Check(fired == 0, "a worker that stalled for a minute counted one period");
    }
    {
        // The dev key's bit 4: the watchdog is told the hook never ran.
        addrcascade::set_fault_mask(0x10);
        Worker w;
        const int fired = w.Run(hookA, true, true, limitMs + 200);
#if HALOMCCVR_DEV_DIAGNOSTICS
        Check(fired == 1, "dev build, drive_fault 0x10: a hook that IS called is called dead, once");
        lines.clear();
        drive_cascade::SayNeverCalled("TEST DRIVE", "the control update", 0x1234, "the player was in control");
        Check(Lines("TEST DRIVE: INSTALLED BUT NEVER CALLED. The hook on the control update at +0x1234 has stood "
                    "through 10 s in which the player was in control") == 1 &&
                  Lines("DEV fault injection (drive_fault 0x10) told that watchdog") == 1,
              "and its line is followed by one saying the fault was forced");
#else
        Check(fired == 0, "player build: the fault mask does nothing to a watchdog");
        lines.clear();
        drive_cascade::SayNeverCalled("TEST DRIVE", "the control update", 0x1234, "the player was in control");
        Check(lines.size() == 1 &&
                  Lines("TEST DRIVE: INSTALLED BUT NEVER CALLED. The hook on the control update at +0x1234 has stood "
                        "through 10 s in which the player was in control") == 1,
              "the watchdog's one line, as formatted");
#endif
        addrcascade::set_fault_mask(0);
    }
}
} // namespace

int main(int argc, char** argv)
{
    printLines = argc > 1 && !std::strcmp(argv[1], "--print");
    t_touched = 1;
    g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + dos->e_lfanew);
    g_size = nt->OptionalHeader.SizeOfImage;
    Check(t_touched == 1 && nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_TLS].VirtualAddress != 0,
          "this image has a PE TLS directory");

    TestOpenAndResolve();
    TestAnnounceAndFaults();
    TestCacheFile();
    TestInstallProofsNeverFault();
    TestHookWatch();

    std::printf("drive_cascade_tests (%s build): %u checks, %u failures\n",
                HALOMCCVR_DEV_DIAGNOSTICS ? "dev" : "player", checks, failures);
    return failures ? 1 : 0;
}
