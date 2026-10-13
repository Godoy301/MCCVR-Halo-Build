// The DLL's side of the direct drives' address resolution (src/dll/drive_cascade.cpp), as shipped, run on a module
// that is really loaded: this test's own executable. What runs here that the synthetic-image tests cannot reach:
// the module is opened through VirtualQuery and its own headers, its .pdata is read where the loader mapped it
// and compared with what the operating system answers for the same addresses, its TLS index is taken from its PE
// TLS directory, a pattern is found through one of "this mod's hooks" (a patch reported by the hook-patch
// source), the one line that says which rung won is really formatted, and the hook watchdog is stepped as the
// cold worker steps it. It maps no game code and opens no game file.
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
uintptr_t g_base{};
size_t g_size{};
uint32_t Rva(const void* p) { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p) - g_base); }

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
    return drive_cascade::Resolve(g_base, g_size, out, [&](drive_resolve::Resolver& r) {
        return Plan(r, markerRecorded, pattern, framed);
    });
}

void TestOpenAndResolve()
{
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
    Check(!drive_cascade::Resolve(uintptr_t{0}, g_size, none, [&](drive_resolve::Resolver& r) {
        return Plan(r, markerRva, pattern, framed);
    }) && none.why && none.report.count == 0, "no module: not opened, with the reason");
    static uint8_t notAnImage[0x2000];
    drive_cascade::Resolution<Found> junk;
    Check(!drive_cascade::Resolve(reinterpret_cast<uintptr_t>(notAnImage), sizeof(notAnImage), junk,
                                  [&](drive_resolve::Resolver& r) { return Plan(r, markerRva, pattern, framed); }) &&
              junk.why && junk.report.count == 0,
          "memory that is not a module: not opened, with the reason");
}

void TestAnnounceAndFaults()
{
    const std::string pattern = MarkerPattern();
    const uint32_t markerRva = Rva(g_marker);
    const auto plan = [&](drive_resolve::Resolver& r) {
        Found f;
        f.marker = r.Unique("MARKER", markerRva, pattern.c_str());
        f.tls = r.TlsIndex("TLS index", Rva(&_tls_index), Rva(&_tls_index));
        f.verdict = r.report().lines[0].verdict;
        return f;
    };
    addrcascade::TierReporter reporter;
    drive_cascade::Resolution<Found> healthy;
    lines.clear();
    Check(drive_cascade::Resolve(g_base, g_size, healthy, plan) && healthy.found.marker == markerRva &&
              healthy.found.tls == Rva(&_tls_index),
          "a healthy resolution");
    drive_cascade::Announce("TEST DRIVE", healthy.report, reporter);
    Check(Lines("TEST DRIVE: all 2 of its addresses were found by search where the tables record them") == 1,
          "which rung won is said: all as recorded");
    drive_cascade::Announce("TEST DRIVE", healthy.report, reporter);
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
                          {0x08, "the PE TLS directory disagrees"}};
    for (const Case& c : cases)
    {
        g_config.drive_fault = c.mask;
        drive_cascade::Resolution<Found> forced;
        lines.clear();
        const bool opened = drive_cascade::Resolve(g_base, g_size, forced, plan);
        drive_cascade::Announce("TEST DRIVE", forced.report, reporter);
        char what[160];
#if HALOMCCVR_DEV_DIAGNOSTICS
        using drive_resolve::Verdict;
        bool as = false;
        switch (c.mask)
        {
        case 0x01: as = forced.found.marker == 0 && forced.found.verdict == Verdict::Missing; break;
        case 0x02: as = forced.found.marker == markerRva && forced.found.verdict == Verdict::RecordAmongSeveral; break;
        case 0x04: as = forced.found.marker == markerRva && forced.found.verdict == Verdict::Moved; break;
        case 0x06: as = forced.found.marker == 0 && forced.found.verdict == Verdict::Ambiguous; break;
        case 0x08: as = forced.found.marker == markerRva && forced.found.tls == 0; break;
        }
        std::snprintf(what, sizeof(what), "dev build, drive_fault 0x%02X (%s): the resolution shows it", c.mask, c.what);
        Check(opened && as, what);
        std::snprintf(what, sizeof(what), "dev build, drive_fault 0x%02X: the line says the result is forced", c.mask);
        Check(Lines("DEV fault injection (drive_fault") == 1 && lines.size() == 2, what);
#else
        std::snprintf(what, sizeof(what), "player build, drive_fault 0x%02X (%s): the key does nothing", c.mask, c.what);
        Check(opened && forced.found.marker == markerRva && forced.found.verdict == drive_resolve::Verdict::AsRecorded &&
                  forced.found.tls == Rva(&_tls_index) && lines.empty(),
              what);
#endif
    }
    g_config.drive_fault = 0;
    drive_cascade::Resolution<Found> again;
    lines.clear();
    Check(drive_cascade::Resolve(g_base, g_size, again, plan) && again.found.marker == markerRva, "healthy again");
    drive_cascade::Announce("TEST DRIVE", again.report, reporter);
#if HALOMCCVR_DEV_DIAGNOSTICS
    Check(Lines("all 2 of its addresses were found by search where the tables record them") == 1 &&
              Lines("DEV fault injection") == 0,
          "back to healthy is a change, and is said, without the forced-result line");
#else
    Check(lines.empty(), "player build: nothing changed, nothing said");
#endif
}

// The cold worker passes over a watch about every 50 ms.
struct Worker
{
    drive_cascade::HookWatch watch;
    uint64_t now = 1000000;
    // Passes for `ms`, returns how many times the watch fired.
    int Run(const void* target, bool possible, bool called, uint32_t ms)
    {
        int fired = 0;
        for (uint32_t t = 0; t < ms; t += 50)
        {
            now += 50;
            fired += watch.Pass(target, possible, called, now) ? 1 : 0;
        }
        return fired;
    }
};

void TestHookWatch()
{
    const int hookA = 0, hookB = 0;
    const uint32_t limitMs = drive_cascade::HookWatch::kPeriods * drive_cascade::HookWatch::kPeriodMs;
    {
        Worker w;
        Check(w.Run(nullptr, true, false, 60000) == 0, "no hook: nothing to say");
    }
    {
        Worker w;
        Check(w.Run(&hookA, true, false, limitMs - 200) == 0, "installed, possible, uncalled: silent before the limit");
        Check(w.Run(&hookA, true, false, 400) == 1, "and said exactly once when the limit is reached");
        Check(w.Run(&hookA, true, false, 60000) == 0, "and never again for that install");
        Check(w.Run(&hookB, true, false, limitMs + 200) == 1, "a new install is watched afresh");
    }
    {
        Worker w;
        Check(w.Run(&hookA, false, false, 600000) == 0, "ten minutes of a menu, a load or a cutscene count for nothing");
        Check(w.Run(&hookA, true, false, limitMs - 200) == 0, "and leave nothing behind for the play that follows");
        Check(w.Run(&hookA, true, false, 400) == 1, "which is counted from its own beginning");
    }
    {
        Worker w;
        int fired = 0;
        // Play broken by a cutscene every eight seconds: never ten seconds of possible time in a row.
        for (int i = 0; i < 20; ++i)
        {
            fired += w.Run(&hookA, true, false, 8000);
            fired += w.Run(&hookA, false, false, 2100);
        }
        Check(fired == 0, "possible time is counted only in an unbroken run");
        // One pass in a period that found the call impossible spoils that period.
        Worker one;
        fired = one.Run(&hookA, true, false, 7900);
        fired += one.watch.Pass(&hookA, false, false, one.now += 50) ? 1 : 0;
        fired += one.Run(&hookA, true, false, 6000);
        Check(fired == 0, "a single pass that found the call impossible restarts the count");
    }
    {
        Worker w;
        Check(w.Run(&hookA, true, false, 4000) == 0 && w.Run(&hookA, true, true, 50) == 0 && w.watch.everCalled(),
              "one call is seen");
        Check(w.Run(&hookA, true, false, 600000) == 0, "a hook that was called once is never called dead");
    }
    {
        // A stalled worker: two passes a minute apart are one period, not thirty.
        Worker w;
        int fired = w.watch.Pass(&hookA, true, false, w.now) ? 1 : 0;
        fired += w.watch.Pass(&hookA, true, false, w.now += 60000) ? 1 : 0;
        Check(fired == 0, "a worker that stalled for a minute counted one period");
    }
    {
        // The dev key's bit 4: the watchdog is told the hook never ran.
        addrcascade::set_fault_mask(0x10);
        Worker w;
        const int fired = w.Run(&hookA, true, true, limitMs + 200);
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
    TestHookWatch();

    std::printf("drive_cascade_tests (%s build): %u checks, %u failures\n",
                HALOMCCVR_DEV_DIAGNOSTICS ? "dev" : "player", checks, failures);
    return failures ? 1 : 0;
}
