// Drive address resolution (src/common/drive_resolve.h) over synthetic images: an anchor found where the table
// says, followed when the code before it grows, refused when its signature is ambiguous or gone, taken at its
// recorded address when that is what tells several matches apart; an anchor that is resolved inside its function;
// function starts and extents from .pdata, chained fragments included; the call, rip-operand and TLS proofs; the
// reads through this mod's own hooks; and every forced fault.
#define DRIVE_RESOLVE_FAULTS 1
#include "drive_resolve.h"

#include <cstdio>
#include <cstring>
#include <vector>

namespace
{
using namespace drive_resolve;

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

// A synthetic module: `code` bytes of executable section at RVA 0x1000, then a data area (unwind info, a TLS
// index), and a .pdata table held beside it. Filler is 0xCC, which no test pattern contains.
struct Module
{
    static constexpr uint32_t kCode = 0x1000;
    std::vector<uint8_t> bytes;
    std::vector<RuntimeFunction> table;
    uint32_t codeSize = 0;
    uint32_t dataAt = 0;
    Image view{};

    explicit Module(uint32_t code = 0x4000) : codeSize(code)
    {
        bytes.assign(kCode + code + 0x1000, 0xCC);
        dataAt = kCode + code;
        std::memset(bytes.data() + dataAt, 0, 0x1000);
    }

    // Writes "48 8B ..." at `rva` ("??" writes 0xA5) and returns the RVA after it.
    uint32_t Put(uint32_t rva, const char* text)
    {
        Pattern p;
        if (!Compile(text, p))
        {
            Check(false, "a test pattern compiles");
            return rva;
        }
        for (uint32_t i = 0; i < p.length; ++i)
            bytes[rva + i] = p.wild[i] ? 0xA5 : p.bytes[i];
        return rva + p.length;
    }

    void Rel32(uint32_t at, uint32_t nextInstruction, uint32_t target)
    {
        const int32_t rel = static_cast<int32_t>(target) - static_cast<int32_t>(nextInstruction);
        std::memcpy(bytes.data() + at, &rel, sizeof(rel));
    }

    // A .pdata entry with plain unwind info, or one chained to `parent` (an index into `table`).
    uint32_t Function(uint32_t begin, uint32_t end, int parent = -1)
    {
        const uint32_t unwind = dataAt;
        if (parent < 0)
        {
            const uint8_t info[4] = {0x01, 0x04, 0x00, 0x00};   // version 1, no flags, no codes
            std::memcpy(bytes.data() + unwind, info, sizeof(info));
            dataAt += 4;
        }
        else
        {
            const uint8_t info[4] = {0x01 | (4u << 3), 0x00, 0x00, 0x00};   // UNW_FLAG_CHAININFO, no codes
            std::memcpy(bytes.data() + unwind, info, sizeof(info));
            std::memcpy(bytes.data() + unwind + 4, &table[static_cast<size_t>(parent)], sizeof(RuntimeFunction));
            dataAt += 16;
        }
        table.push_back(RuntimeFunction{begin, end, unwind});
        return static_cast<uint32_t>(table.size() - 1);
    }

    const Image& Finish()
    {
        view = Image{};
        view.base = bytes.data();
        view.size = static_cast<uint32_t>(bytes.size());
        view.executable[0] = Range{kCode, codeSize};
        view.executableCount = 1;
        view.functions = table.data();
        view.functionCount = static_cast<uint32_t>(table.size());
        return view;
    }
};

const Line& Last(const Resolver& r)
{
    return r.report().lines[r.report().count - 1];
}

// Distinct byte runs the tests place as "code".
constexpr char kEntry[] = "48 8B C4 55 53 56 57 41 54 48 8D A8 18 FF FF FF 48 81 EC A8 01 00 00";
constexpr char kOther[] = "40 53 48 83 EC 20 8B D9 E8 ?? ?? ?? ?? 48 8B C8";
constexpr char kField[] = "4C 8D 7B 14";                                // short: the instruction that reads a field
constexpr char kStamp[] = "F2 41 0F 10 85 C0 0A 00 00 41 8B 85 C8 0A 00 00";

void TestCompile()
{
    Pattern p;
    Check(Compile("48 8B ?? C0", p) && p.length == 4 && p.bytes[0] == 0x48 && p.wild[2] == 1 && p.bytes[3] == 0xC0,
          "a pattern with a wildcard compiles");
    Check(Compile("48 ? c0", p) && p.length == 3 && p.wild[1] == 1 && p.bytes[2] == 0xC0,
          "a single '?' is a wildcard, and hex is case-blind");
    Check(!Compile("", p) && !Compile(nullptr, p) && !Compile("4G", p) && !Compile("4", p),
          "empty, null and malformed text is refused");
    Check(!Compile("?? ?? ??", p), "a pattern with no byte to match on is refused");
    std::vector<char> big;
    for (uint32_t i = 0; i < kMaxPattern + 1; ++i)
    {
        big.push_back('9');
        big.push_back('0');
        big.push_back(' ');
    }
    big.push_back(0);
    Check(!Compile(big.data(), p), "a pattern longer than the limit is refused");
}

void TestFoundAsRecorded()
{
    Module m;
    m.Put(0x1200, kEntry);
    m.Put(0x1400, kOther);
    Resolver r(m.Finish(), 0);
    Check(r.Unique("entry", 0x1200, kEntry) == 0x1200, "a unique anchor at its recorded RVA is found there");
    Check(Last(r).verdict == Verdict::AsRecorded && Last(r).matches == 1 && Last(r).how == How::Signature,
          "it is reported as recorded, one match, by signature");
    Check(r.report().NotFound() == 0 && r.report().Count(Verdict::AsRecorded) == 1, "the report counts it");
}

void TestMovedAndFollowed()
{
    // The same code after 0x30 bytes were inserted ahead of the function.
    Module m;
    m.Put(0x1230, kEntry);
    Resolver r(m.Finish(), 0);
    Check(r.Unique("entry", 0x1200, kEntry) == 0x1230, "a unique anchor that moved is found where it is now");
    Check(Last(r).verdict == Verdict::Moved && Last(r).recorded == 0x1200 && Last(r).found == 0x1230,
          "it is reported as moved, from the recorded RVA to the found one");
}

void TestAmbiguousRefused()
{
    Module m;
    m.Put(0x1300, kEntry);
    m.Put(0x1900, kEntry);
    Resolver r(m.Finish(), 0);
    Check(r.Unique("entry", 0x1200, kEntry) == 0, "two matches, neither at the recorded RVA: not found");
    Check(Last(r).verdict == Verdict::Ambiguous && Last(r).matches == 2 && Last(r).found == 0,
          "it is reported ambiguous with both matches counted, and the first is not taken");
    Check(r.report().NotFound() == 1, "the report counts the refusal");
}

void TestAmbiguousDisambiguatedByRecord()
{
    Module m;
    m.Put(0x1200, kEntry);
    m.Put(0x1900, kEntry);
    m.Put(0x2100, kEntry);
    Resolver r(m.Finish(), 0);
    Check(r.Unique("entry", 0x1900, kEntry) == 0x1900,
          "several matches, one at the recorded RVA: that one is taken (not the first)");
    Check(Last(r).verdict == Verdict::RecordAmongSeveral && Last(r).matches == 3,
          "it is reported as taken at the recorded address among three");
}

void TestMissingRefused()
{
    Module m;
    m.Put(0x1200, kOther);
    Resolver r(m.Finish(), 0);
    Check(r.Unique("entry", 0x1200, kEntry) == 0 && Last(r).verdict == Verdict::Missing && Last(r).matches == 0,
          "no match: not found, reported missing");
    // A match in data is not code: only the executable sections are searched.
    Module d;
    d.Put(d.dataAt + 0x100, kEntry);
    Resolver rd(d.Finish(), 0);
    Check(rd.Unique("entry", 0x1200, kEntry) == 0, "bytes outside the executable sections never match");
    // A recorded RVA outside the image is only a failed cross-check.
    Module e;
    e.Put(0x1200, kEntry);
    Resolver re(e.Finish(), 0);
    Check(re.Unique("entry", 0xFFFFFF00u, kEntry) == 0x1200 && Last(re).verdict == Verdict::Moved,
          "a recorded RVA outside the image does not stop the search");
    Check(re.Unique("bad", 0x1200, "zz") == 0 && Last(re).verdict == Verdict::Refused,
          "a pattern that does not compile is refused");
}

void TestCountEdges()
{
    Module m(0x100);
    const uint32_t end = Module::kCode + 0x100;
    m.Put(end - 4, kField);            // ends exactly at the end of the executable range
    m.Put(Module::kCode, kField);      // starts exactly at its beginning
    Pattern p;
    Compile(kField, p);
    const Image& im = m.Finish();
    Check(CountExecutable(im, p).count == 2 && CountExecutable(im, p).first == Module::kCode,
          "matches at both edges of a range are counted, the lowest first");
    Check(CountIn(im, p, Module::kCode + 1, end - 1).count == 0, "a match that straddles a range edge is not in it");
    Check(CountIn(im, p, Module::kCode, Module::kCode + 3).count == 0, "a range shorter than the pattern holds no match");
    Check(CountIn(im, p, end, Module::kCode).count == 0 && CountIn(im, p, 0, 0xFFFFFFF0u).count == 0,
          "an empty or out-of-image range holds no match");
    // The count stops at the cap: enough to say "several".
    Module many(0x400);
    for (uint32_t i = 0; i < 40; ++i)
        many.Put(Module::kCode + i * 16, kField);
    Check(CountExecutable(many.Finish(), p).count == kCountCap, "the count is capped");
    // Overlapping matches each count.
    Module over(0x100);
    over.Put(Module::kCode + 8, "AA AA AA");
    Pattern aa;
    Compile("AA AA", aa);
    Check(CountExecutable(over.Finish(), aa).count == 2, "overlapping matches are both counted");
    // A wildcard first byte does not narrow the search.
    Pattern lead;
    Compile("?? 8D 7B 14", lead);
    Check(CountExecutable(im, lead).count == 2, "a leading wildcard matches any byte");
}

// Against a count that compares every offset in full.
void TestCountAgainstBruteForce()
{
    Module m(0x2000);
    uint32_t seed = 0x1234567u;
    for (uint32_t i = 0; i < 0x2000; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        m.bytes[Module::kCode + i] = static_cast<uint8_t>((seed >> 24) & 0x3u);   // four byte values: many matches
    }
    const Image& im = m.Finish();
    const char* patterns[] = {"00 01", "03 ?? 03", "?? 02 01 00", "01 01 01 01", "02", "00 ?? ?? 03 03"};
    bool agree = true;
    for (const char* text : patterns)
    {
        Pattern p;
        Compile(text, p);
        uint32_t brute = 0, first = 0;
        for (uint32_t at = Module::kCode; at + p.length <= Module::kCode + 0x2000; ++at)
        {
            bool match = true;
            for (uint32_t i = 0; i < p.length && match; ++i)
                match = p.wild[i] || m.bytes[at + i] == p.bytes[i];
            if (match && !brute++)
                first = at;
        }
        const Hits hits = CountExecutable(im, p);
        agree = agree && hits.count == (brute > kCountCap ? kCountCap : brute) && (!brute || hits.first == first);
    }
    Check(agree, "the anchored search counts what a full compare at every offset counts");
}

void TestFunctionsFromPdata()
{
    Module m;
    m.Function(0x1200, 0x1300);
    const uint32_t head = m.Function(0x1400, 0x1420);
    m.Function(0x1420, 0x1500, static_cast<int>(head));   // a chained fragment of the function at 0x1400
    m.Function(0x1500, 0x1540, static_cast<int>(head));   // and another, contiguous
    m.Function(0x1540, 0x1600);                           // the next function
    m.Function(0x1700, 0x1720, static_cast<int>(head));   // a fragment that is not contiguous with its function
    const Image& im = m.Finish();
    Check(EntryAt(im, 0x1200) && EntryAt(im, 0x12FF) && !EntryAt(im, 0x1300) && !EntryAt(im, 0x11FF),
          "a .pdata entry holds [begin, end)");
    Check(FunctionStartOf(im, 0x1250) == 0x1200 && FunctionEndOf(im, 0x1200) == 0x1300,
          "a plain function starts and ends with its entry");
    Check(FunctionStartOf(im, 0x1450) == 0x1400 && FunctionStartOf(im, 0x1530) == 0x1400,
          "a chained fragment belongs to the function its unwind info names");
    Check(FunctionEndOf(im, 0x1400) == 0x1540, "a function's extent runs over its contiguous chained fragments");
    Check(FunctionStartOf(im, 0x1560) == 0x1540 && FunctionEndOf(im, 0x1540) == 0x1600,
          "and stops at the next function");
    Check(FunctionStartOf(im, 0x1710) == 0x1400, "a detached fragment still names its function");
    Check(FunctionStartOf(im, 0x1650) == 0 && FunctionEndOf(im, 0x1450) == 0,
          "an address in no entry has no function, and a fragment is not a start");
    Resolver r(im, 0);
    Check(r.IsFunctionStart(0x1400) && !r.IsFunctionStart(0x1420) && !r.IsFunctionStart(0x1401) && !r.IsFunctionStart(0),
          "only the first byte of a function is where it starts");
    Check(r.SameFunction(0x1530, 0x1400) && r.SameFunction(0x1710, 0x1400) && !r.SameFunction(0x1560, 0x1400) &&
              !r.SameFunction(0x1650, 0x1400) && !r.SameFunction(0x1450, 0),
          "an address is in a function when .pdata says so");
    // A chain that points outside the image, or loops, resolves to nothing.
    Module bad;
    bad.table.push_back(RuntimeFunction{0x1200, 0x1300, 0xFFFFFF00u});
    Check(FunctionStartOf(bad.Finish(), 0x1250) == 0, "unwind info outside the image is not followed");
    Module loop;
    const uint8_t chain[4] = {0x01 | (4u << 3), 0, 0, 0};
    std::memcpy(loop.bytes.data() + loop.dataAt, chain, 4);
    const RuntimeFunction self{0x1200, 0x1300, loop.dataAt};
    std::memcpy(loop.bytes.data() + loop.dataAt + 4, &self, sizeof(self));
    loop.table.push_back(self);
    Check(FunctionStartOf(loop.Finish(), 0x1250) == 0, "a chain that never ends is given up");
    // An entry that points at another entry instead of unwind info.
    Module indirect;
    const uint32_t real = indirect.Function(0x1200, 0x1300);
    std::memcpy(indirect.bytes.data() + indirect.dataAt, &indirect.table[real], sizeof(RuntimeFunction));
    indirect.table.push_back(RuntimeFunction{0x1300, 0x1340, indirect.dataAt | 1u});
    Check(FunctionStartOf(indirect.Finish(), 0x1320) == 0x1200, "an indirect entry is followed to the one it names");
}

// A function at `start` whose body carries the short field anchor at +0x60 and the stamp anchor at +0x90.
void PutFunction(Module& m, uint32_t start, uint32_t fieldAt = 0x60)
{
    m.Put(start, kEntry);
    m.Put(start + fieldAt, kField);
    m.Put(start + 0x90, kStamp);
    m.Function(start, start + 0x100);
}

void TestInFunction()
{
    // As recorded.
    {
        Module m;
        PutFunction(m, 0x1200);
        m.Put(0x2000, kField);   // the same short bytes in other code: the module-wide count is two
        Resolver r(m.Finish(), 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x1260 && Last(r).verdict == Verdict::AsRecorded &&
                  Last(r).how == How::FunctionOffset,
              "an in-function anchor is found at its recorded offset, whatever the rest of the module holds");
    }
    // The function moved (code inserted ahead of it): the anchor follows it at the same offset.
    {
        Module m;
        PutFunction(m, 0x1750);
        m.Put(0x1260, kField);   // the recorded RVA now holds the same bytes in someone else's code
        Resolver r(m.Finish(), 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(entry == 0x1750, "the function is followed");
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x17B0 && Last(r).verdict == Verdict::Moved &&
                  Last(r).how == How::FunctionOffset,
              "its anchor is found at the same offset from the function's new start, not at the stale RVA");
        Check(r.InFunction("stamp", 0x1290, 0x1200, entry, kStamp) == 0x17E0, "and so is every other anchor in it");
    }
    // The function changed ahead of the anchor: it is no longer at its offset, and is the one match in the function.
    {
        Module m;
        PutFunction(m, 0x1200, 0x68);
        Resolver r(m.Finish(), 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x1268 && Last(r).verdict == Verdict::Moved &&
                  Last(r).how == How::FunctionExtent && Last(r).matches == 1,
              "an anchor that moved inside its function is found as the one match in its extent");
    }
    // Not at its offset and twice in the function: nothing tells them apart.
    {
        Module m;
        PutFunction(m, 0x1200, 0x68);
        m.Put(0x12C0, kField);
        Resolver r(m.Finish(), 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0 && Last(r).verdict == Verdict::Ambiguous &&
                  Last(r).matches == 2,
              "two matches inside the function and none at the offset: refused");
    }
    // At its offset AND again later in the function: the offset decides.
    {
        Module m;
        PutFunction(m, 0x1200);
        m.Put(0x12C0, kField);
        Resolver r(m.Finish(), 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x1260 && Last(r).verdict == Verdict::AsRecorded,
              "an anchor at its recorded offset is taken there even where its bytes recur in the function");
    }
    // Gone from the function (the field's offset changed: its displacement bytes are exact).
    {
        Module m;
        m.Put(0x1200, kEntry);
        m.Put(0x1260, "4C 8D 7B 18");
        m.Function(0x1200, 0x1300);
        m.Put(0x2000, kField);   // present elsewhere in the module, which must not count
        Resolver r(m.Finish(), 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0 && Last(r).verdict == Verdict::Missing,
              "a field offset that changed is not found: the layout check fails closed");
    }
    // No function: the anchor that locates it was not found, or has no .pdata entry.
    {
        Module m;
        m.Put(0x1200, kEntry);
        m.Put(0x1260, kField);
        Resolver r(m.Finish(), 0);
        Check(r.InFunction("field", 0x1260, 0x1200, 0, kField) == 0 && Last(r).verdict == Verdict::NoFunction,
              "an anchor whose function was not found is not found");
        Check(r.InFunction("field", 0x1260, 0x1200, 0x1200, kField) == 0 && Last(r).verdict == Verdict::NoFunction,
              "nor one whose function has no .pdata entry");
        Check(r.InFunction("field", 0x1100, 0x1200, 0x1200, kField) == 0 && Last(r).verdict == Verdict::NoFunction,
              "nor one recorded ahead of its own function");
    }
    // Located through an anchor in the middle of the function (not its entry), across a chained fragment.
    {
        Module m;
        const uint32_t head = m.Function(0x1800, 0x1820);
        m.Function(0x1820, 0x1900, static_cast<int>(head));
        m.Put(0x1830, kStamp);
        m.Put(0x18A0, kField);
        Resolver r(m.Finish(), 0);
        const uint32_t via = r.Unique("stamp", 0x1230, kStamp);
        Check(via == 0x1830, "the locating anchor is followed");
        Check(r.InFunction("field", 0x12A0, 0x1200, via, kField) == 0x18A0 && Last(r).how == How::FunctionOffset,
              "an anchor in a chained fragment is found at its offset from the function's start");
        Resolver s(m.Finish(), kFaultOffsetMoved);
        const uint32_t via2 = s.Unique("stamp", 0x1230, kStamp);
        Check(s.InFunction("field", 0x12A0, 0x1200, via2, kField) == 0x18A0 && Last(s).how == How::FunctionExtent,
              "and the extent searched for it covers the fragments");
    }
}

void TestFunctionStart()
{
    // Three functions begin with the same bytes; the one wanted is the one that holds the unique stamp.
    Module m;
    for (uint32_t start : {0x1200u, 0x1600u, 0x1A00u})
    {
        m.Put(start, kEntry);
        m.Function(start, start + 0x100);
    }
    m.Put(0x1680, kStamp);
    {
        Resolver r(m.Finish(), 0);
        const uint32_t stamp = r.Unique("stamp", 0x1680, kStamp);
        Check(r.FunctionStart("entry", 0x1600, stamp, kEntry) == 0x1600 && Last(r).verdict == Verdict::AsRecorded &&
                  Last(r).matches == 3 && Last(r).how == How::FunctionStart,
              "a function whose first bytes are shared is found as the start of the function that holds its anchor");
        Check(r.Unique("entry by bytes", 0x1600, kEntry) == 0x1600 && Last(r).verdict == Verdict::RecordAmongSeveral,
              "by its bytes alone it is only told apart by the recorded address");
        Check(r.FunctionStart("entry", 0x1700, stamp, kEntry) == 0x1600 && Last(r).verdict == Verdict::Moved,
              "and with a stale record it is still found, and reported moved");
        Check(r.FunctionStart("entry", 0x1600, stamp, kOther) == 0 && Last(r).verdict == Verdict::Refused,
              "a start that does not begin with the recorded bytes is refused");
        Check(r.FunctionStart("entry", 0x1600, 0, kEntry) == 0 && Last(r).verdict == Verdict::NoFunction,
              "no anchor, no function");
    }
}

void TestProofs()
{
    Module m;
    m.Put(0x1200, kEntry);
    m.Function(0x1200, 0x1300);
    m.bytes[0x1500] = 0xE8;
    m.Rel32(0x1501, 0x1505, 0x1200);
    m.bytes[0x1510] = 0xE9;   // a jump, not a call
    m.Rel32(0x1511, 0x1515, 0x1200);
    m.bytes[0x1520] = 0xE8;
    m.Rel32(0x1521, 0x1525, 0x7FFFFFF0u);   // leaves the image
    // mov ecx, [rip + disp32] -> the "TLS index" in the data area
    const uint32_t index = m.dataAt + 0x800;
    m.Put(0x1600, "8B 0D 00 00 00 00");
    m.Rel32(0x1602, 0x1606, index);
    Image im = m.Finish();
    im.tlsIndexRva = index;
    Resolver r(im, 0);
    Check(r.CallTarget(0x1500) == 0x1200, "a call's target is read from its rel32");
    Check(r.CallTarget(0x1510) == 0 && r.CallTarget(0x1520) == 0 && r.CallTarget(0) == 0 &&
              r.CallTarget(m.dataAt) == 0,
          "what is not a call, leaves the image, or is not code has no target");
    Check(r.RipTarget(0x1600, 2, 6) == index, "a rip-relative operand's target is read from its displacement");
    Check(r.RipTarget(0x1600, 4, 6) == 0 && r.RipTarget(0, 2, 6) == 0 && r.RipTarget(0x1600, 2, 64) == 0,
          "a displacement that does not fit its instruction is refused");
    Check(r.Derived("call", 0x1500, 0x1500) == 0x1500 && Last(r).verdict == Verdict::AsRecorded &&
              Last(r).how == How::Derived,
          "a derived address that agrees with the table is reported as recorded");
    Check(r.Derived("call", 0x1400, 0x1500) == 0x1500 && Last(r).verdict == Verdict::Moved,
          "one that disagrees is reported moved");
    Check(r.Derived("call", 0x1500, 0) == 0 && Last(r).verdict == Verdict::Missing, "one that was not derived is missing");
    Check(r.Refuse("its target is not the entry") == 0 && Last(r).verdict == Verdict::Refused && Last(r).found == 0 &&
              std::strcmp(Last(r).note, "its target is not the entry") == 0,
          "a line that fails a proof is refused, with the proof named");

    // The engine TLS index: the PE TLS directory and the anchored instruction must name one address.
    Check(r.TlsIndex("TLS index", index, r.RipTarget(0x1600, 2, 6)) == index && Last(r).verdict == Verdict::AsRecorded &&
              Last(r).how == How::TlsDirectory,
          "the TLS index is the directory's when the instruction loads it from there");
    Check(r.TlsIndex("TLS index", index + 0x40, index) == index && Last(r).verdict == Verdict::Moved,
          "a stale record of it is reported, and the directory's address is used");
    Check(r.TlsIndex("TLS index", index, index + 8) == 0 && Last(r).verdict == Verdict::Refused,
          "an instruction that loads another address is refused");
    Check(r.TlsIndex("TLS index", index, 0) == 0 && Last(r).verdict == Verdict::Refused,
          "an instruction that was not found is refused");
    Image none = m.Finish();
    Resolver rn(none, 0);
    Check(rn.TlsIndex("TLS index", index, index) == 0 && Last(rn).verdict == Verdict::Refused,
          "a module whose TLS directory did not read has no TLS index");
    Image outside = m.Finish();
    outside.tlsIndexRva = outside.size - 2;
    Resolver ro(outside, 0);
    Check(ro.TlsIndex("TLS index", index, outside.size - 2) == 0, "a TLS index that does not fit the image is refused");
}

void TestFaults()
{
    Module m;
    PutFunction(m, 0x1200);
    const uint32_t index = m.dataAt + 0x800;
    Image im = m.Finish();
    im.tlsIndexRva = index;
    {
        Resolver r(im, kFaultSignatureMissing);
        Check(r.Unique("entry", 0x1200, kEntry) == 0 && Last(r).verdict == Verdict::Missing,
              "fault: a signature that finds nothing leaves the anchor not found");
    }
    {
        Resolver r(im, kFaultSignatureAmbiguous);
        Check(r.Unique("entry", 0x1200, kEntry) == 0x1200 && Last(r).verdict == Verdict::RecordAmongSeveral &&
                  Last(r).matches == 2,
              "fault: an ambiguous signature is settled by the recorded address");
    }
    {
        Resolver r(im, kFaultRecordWrong);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(entry == 0x1200 && Last(r).verdict == Verdict::Moved && Last(r).recorded == 0x1200 + kFaultRecordShift,
              "fault: with a wrong record the search wins and the anchor is reported moved");
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x1260 && Last(r).verdict == Verdict::Moved &&
                  Last(r).how == How::FunctionOffset,
              "fault: an in-function anchor follows its function all the same");
        Check(r.Recorded(0x1200) == 0x1200 + kFaultRecordShift && r.Recorded(0) == 0, "fault: every record reads shifted");
    }
    {
        Resolver r(im, kFaultSignatureAmbiguous | kFaultRecordWrong);
        Check(r.Unique("entry", 0x1200, kEntry) == 0 && Last(r).verdict == Verdict::Ambiguous,
              "fault: ambiguous with a wrong record is refused");
    }
    {
        Resolver r(im, kFaultOffsetMoved);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x1260 && Last(r).how == How::FunctionExtent,
              "fault: an anchor denied its offset is found as the one match in its function");
    }
    {
        Resolver r(im, kFaultTlsDirectory);
        Check(r.TlsIndex("TLS index", index, index) == 0 && Last(r).verdict == Verdict::Refused,
              "fault: a TLS directory that disagrees is refused");
    }
    {
        Resolver r(im, kFaultHookSilent);
        Check(r.Unique("entry", 0x1200, kEntry) == 0x1200 && Last(r).verdict == Verdict::AsRecorded,
              "the watchdog's fault bit changes nothing about where things are");
    }
    Check(Fault(kFaultSignatureMissing, kFaultSignatureMissing) && !Fault(0, kFaultSignatureMissing),
          "a fault is forced only by its own bit");
}

// THIS MOD'S OWN HOOKS. A hooked function's first bytes are a jump; proofs read what stood there before.
uint32_t g_patchAt = 0;
uint8_t g_patchOriginal[5]{};
const uint8_t* g_patchBase = nullptr;

uint32_t FakeOriginal(uintptr_t address, void* out, size_t size)
{
    std::memcpy(out, reinterpret_cast<const void*>(address), size);
    const uintptr_t patch = reinterpret_cast<uintptr_t>(g_patchBase) + g_patchAt;
    uint32_t undone = 0;
    for (size_t i = 0; i < 5; ++i)
        if (patch + i >= address && patch + i < address + size)
        {
            static_cast<uint8_t*>(out)[patch + i - address] = g_patchOriginal[i];
            undone = 1;
        }
    return undone;
}

void FakeCorrect(uintptr_t begin, size_t size, const uint8_t* bytes, const uint8_t* wild, size_t n, uintptr_t& first,
                 uint32_t& count)
{
    // The count over the range with the patch undone, by a full compare at every offset.
    std::vector<uint8_t> copy(size);
    (void)FakeOriginal(begin, copy.data(), size);
    first = 0;
    count = 0;
    for (size_t at = 0; at + n <= size; ++at)
    {
        bool match = true;
        for (size_t i = 0; i < n && match; ++i)
            match = wild[i] || copy[at + i] == bytes[i];
        if (match && !count++)
            first = begin + at;
    }
}

void TestThroughOwnHooks()
{
    Module m;
    PutFunction(m, 0x1200);
    m.bytes[0x1500] = 0xE8;
    m.Rel32(0x1501, 0x1505, 0x1200);
    // Hook the function: E9 rel32 over its first five bytes.
    g_patchAt = 0x1200;
    std::memcpy(g_patchOriginal, m.bytes.data() + 0x1200, 5);
    m.bytes[0x1200] = 0xE9;
    m.Rel32(0x1201, 0x1205, 0x3000);
    Image im = m.Finish();
    g_patchBase = im.base;
    {
        Resolver r(im, 0);
        Check(r.Unique("entry", 0x1200, kEntry) == 0 && Last(r).verdict == Verdict::Missing,
              "control: read as it stands, a hooked function's signature is gone");
    }
    im.original = &FakeOriginal;
    im.correct = &FakeCorrect;
    {
        Resolver r(im, 0);
        const uint32_t entry = r.Unique("entry", 0x1200, kEntry);
        Check(entry == 0x1200 && Last(r).verdict == Verdict::AsRecorded && Last(r).matches == 1,
              "read through the hook, the function is found by the game's own bytes");
        Check(r.InFunction("field", 0x1260, 0x1200, entry, kField) == 0x1260, "and its anchors with it");
        Pattern p;
        Compile(kEntry, p);
        Check(MatchesAt(im, p, 0x1200), "a compare at a fixed place reads the game's bytes too");
    }
    // A hook on the call site itself: the call proof reads the original instruction.
    g_patchAt = 0x1500;
    std::memcpy(g_patchOriginal, m.bytes.data() + 0x1500, 5);
    m.bytes[0x1500] = 0xE9;
    {
        Resolver r(im, 0);
        Check(r.CallTarget(0x1500) == 0x1200, "a hooked call site still proves its original target");
    }
    g_patchBase = nullptr;
}

// A minimal PE32+ header over a buffer, to read the executable sections and .pdata the way the DLL does.
void TestReadImage()
{
    std::vector<uint8_t> pe(0x6000, 0);
    const auto put16 = [&pe](size_t at, uint16_t v) { std::memcpy(pe.data() + at, &v, sizeof(v)); };
    const auto put32 = [&pe](size_t at, uint32_t v) { std::memcpy(pe.data() + at, &v, sizeof(v)); };
    const size_t nt = 0x80, optional = nt + 24, table = optional + 240;
    put16(0, 0x5A4D);
    put32(0x3C, static_cast<uint32_t>(nt));
    put32(nt, 0x00004550u);
    put16(nt + 4, 0x8664);
    put16(nt + 6, 3);        // sections
    put16(nt + 20, 240);     // optional header size
    put16(optional, 0x20B);
    put32(optional + 108, 16);
    // .text (executable), .rdata, .pdata
    put32(table + 8, 0x2000);
    put32(table + 12, 0x1000);
    put32(table + 36, 0x60000020u);
    put32(table + 40 + 8, 0x1000);
    put32(table + 40 + 12, 0x3000);
    put32(table + 40 + 36, 0x40000040u);
    put32(table + 80 + 8, 0x1000);
    put32(table + 80 + 12, 0x4000);
    put32(table + 80 + 36, 0x40000040u);
    // The exception directory: two entries and a zeroed third.
    put32(optional + 112 + 3 * 8, 0x4000);
    put32(optional + 112 + 3 * 8 + 4, 36);
    const RuntimeFunction entries[2] = {{0x1100, 0x1200, 0x3000}, {0x1200, 0x1280, 0x3004}};
    std::memcpy(pe.data() + 0x4000, entries, sizeof(entries));
    pe[0x3000] = 0x01;
    pe[0x3004] = 0x01;

    Image im;
    const char* why = nullptr;
    Check(ReadImage(pe.data(), pe.size(), im, why) && why == nullptr, "a PE32+ image's headers are read");
    Check(im.executableCount == 1 && im.executable[0].rva == 0x1000 && im.executable[0].size == 0x2000,
          "its executable sections are the ones marked executable");
    Check(im.functionCount == 2 && FunctionStartOf(im, 0x1250) == 0x1200, "its .pdata is read, without the zeroed tail");
    Check(im.tlsIndexRva == 0, "the TLS index is left to the image's owner");

    std::vector<uint8_t> bad = pe;
    bad[0] = 0;
    Check(!ReadImage(bad.data(), bad.size(), im, why) && why, "no DOS header is refused, with the reason");
    bad = pe;
    put32(nt, 0);
    std::memcpy(bad.data(), pe.data(), pe.size());
    Check(!ReadImage(bad.data(), bad.size(), im, why), "no PE header is refused");
    put32(nt, 0x00004550u);
    bad = pe;
    bad[optional] = 0x0B;
    bad[optional + 1] = 0x01;   // PE32, not PE32+
    Check(!ReadImage(bad.data(), bad.size(), im, why), "a 32-bit image is refused");
    bad = pe;
    std::memcpy(bad.data() + table + 12, "\x00\x00\x10\x00", 4);   // .text at an RVA outside the image
    Check(!ReadImage(bad.data(), bad.size(), im, why), "a section outside the image is refused");
    bad = pe;
    std::memcpy(bad.data() + optional + 112 + 3 * 8, "\x00\x00\x20\x00", 4);
    Check(!ReadImage(bad.data(), bad.size(), im, why), "an exception directory outside the image is refused");
    bad = pe;
    std::memcpy(bad.data() + table + 36, "\x40\x00\x00\x40", 4);   // no executable section left
    Check(!ReadImage(bad.data(), bad.size(), im, why), "an image with no executable section is refused");
    Check(!ReadImage(nullptr, 0x6000, im, why) && !ReadImage(pe.data(), 0x100, im, why),
          "a null or tiny image is refused");
}

void TestReport()
{
    Module m;
    PutFunction(m, 0x1200);
    Resolver a(m.Finish(), 0), b(m.Finish(), 0), c(m.Finish(), kFaultRecordWrong);
    a.Unique("entry", 0x1200, kEntry);
    b.Unique("entry", 0x1200, kEntry);
    c.Unique("entry", 0x1200, kEntry);
    Check(a.report().Fingerprint() == b.report().Fingerprint(), "the same resolution has the same fingerprint");
    Check(a.report().Fingerprint() != c.report().Fingerprint(),
          "the same address found another way has another fingerprint");
    Check(std::strcmp(VerdictText(Verdict::Moved), "MOVED") == 0 && std::strcmp(HowText(How::Signature), "signature") == 0 &&
              Found(Verdict::RecordAmongSeveral) && !Found(Verdict::Ambiguous) && !Found(Verdict::Refused),
          "verdicts read as text, and only three of them are a find");
    // More lines than the report holds: the resolution goes on, the overflow is counted.
    Resolver many(m.Finish(), 0);
    uint32_t found = 0;
    for (uint32_t i = 0; i < kMaxLines + 5; ++i)
        found += many.Unique("entry", 0x1200, kEntry) == 0x1200 ? 1u : 0u;
    Check(found == kMaxLines + 5 && many.report().count == kMaxLines && many.report().dropped == 5,
          "lines beyond the report's size are counted, not lost, and the answers stay right");
    Check(many.Refuse("x") == 0 && many.report().lines[kMaxLines - 1].verdict == Verdict::AsRecorded,
          "refusing a line that was not stored touches no other line");
}
} // namespace

int main()
{
    TestCompile();
    TestFoundAsRecorded();
    TestMovedAndFollowed();
    TestAmbiguousRefused();
    TestAmbiguousDisambiguatedByRecord();
    TestMissingRefused();
    TestCountEdges();
    TestCountAgainstBruteForce();
    TestFunctionsFromPdata();
    TestInFunction();
    TestFunctionStart();
    TestProofs();
    TestFaults();
    TestThroughOwnHooks();
    TestReadImage();
    TestReport();
    std::printf("drive resolve tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
