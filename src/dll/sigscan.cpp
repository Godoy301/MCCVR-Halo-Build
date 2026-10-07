#include <windows.h>
#include <psapi.h>
#include <atomic>
#include <vector>
#include <cstring>
#include "sigscan.h"

namespace
{
    // Parse "48 8B ?? C0" into parallel arrays: byte values, and a mask that
    // says which positions are wildcards.
    // The mask is uint8_t, NOT vector<bool>. vector<bool> is a bitset whose
    // operator[] does a shift-and-mask per access, and this mask is read once
    // per byte compared - tens of millions of times per module scan.
    bool ParsePattern(const char* pattern, std::vector<uint8_t>& bytes, std::vector<uint8_t>& wild)
    {
        for (const char* p = pattern; *p;)
        {
            if (*p == ' ')
            {
                p++;
                continue;
            }
            if (p[0] == '?')
            {
                bytes.push_back(0);
                wild.push_back(1);
                p++;
                if (*p == '?')
                    p++;
            }
            else
            {
                auto hex = [](char c) -> int {
                    if (c >= '0' && c <= '9') return c - '0';
                    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                    return -1;
                };
                const int hi = hex(p[0]);
                const int lo = hex(p[1]);
                if (hi < 0 || lo < 0)
                    return false;
                bytes.push_back((uint8_t)((hi << 4) | lo));
                wild.push_back(0);
                p += 2;
            }
        }
        return !bytes.empty();
    }
}

namespace sig
{
    // Matching semantics are EXACTLY those of the original byte-by-byte loop:
    // the first offset in [base, base+size-n] where every non-wildcard byte
    // equals the pattern. Only the search order changed - instead of testing
    // every offset, memchr skips directly to offsets where the pattern's first
    // concrete byte can sit. memchr is SIMD-optimised, while the old loop paid
    // a bounds-checked bitset read per byte compared, and a whole-module scan
    // is ~74 MB. Nothing about verification is relaxed: every candidate is
    // still compared in full, and the ambiguity re-scans callers run to prove
    // a signature is unique are sped up identically. Proven equivalent against
    // a brute-force reference over randomised data in tests/core_tests.cpp.
    //
    // With a hook patch source set, "the image" is the image with this mod's
    // enabled hooks undone (sigscan.h). The live scan is unchanged; what is
    // added is bounded by the hooks, not by the image: a live match that reads
    // a hook must also match the bytes the hook replaced, and the few offsets
    // whose window reads a hook are compared against those bytes.
    namespace
    {
        std::atomic<HookPatchSource> g_hookPatchSource{nullptr};
        std::atomic<ProofThroughHookObserver> g_proofThroughHookObserver{nullptr};

        // The first live match at an offset in [from, last], or last + 1 when
        // there is none.
        size_t FindLive(const uint8_t* data, size_t from, size_t last,
                        const std::vector<uint8_t>& bytes,
                        const std::vector<uint8_t>& wild, size_t anchor)
        {
            const size_t n = bytes.size();
            const uint8_t anchorByte = bytes[anchor];
            size_t i = from;
            while (i <= last)
            {
                // Candidate offsets are exactly those whose anchor byte matches.
                const uint8_t* hit = static_cast<const uint8_t*>(
                    memchr(data + i + anchor, anchorByte, last - i + 1));
                if (!hit)
                    return last + 1;
                i = static_cast<size_t>(hit - data) - anchor;
                size_t j = 0;
                for (; j < n; j++)
                    if (!wild[j] && data[i + j] != bytes[j])
                        break;
                if (j == n)
                    return i;
                ++i;
            }
            return last + 1;
        }

        // This mod's enabled hooks inside [base, base + size). Empty when no
        // source is set.
        void EnabledPatches(uintptr_t base, size_t size, std::vector<HookPatch>& patches)
        {
            patches.clear();
            const HookPatchSource source = g_hookPatchSource.load(std::memory_order_acquire);
            if (!source)
                return;
            HookPatch local[64];
            uint32_t total = source(base, size, local, 64);
            if (total <= 64)
            {
                patches.assign(local, local + total);
                return;
            }
            // More than fit. Hooks may change between the two calls: keep what
            // the second one copied.
            patches.resize(static_cast<size_t>(total) + 16);
            total = source(base, size, patches.data(), static_cast<uint32_t>(patches.size()));
            if (total < patches.size())
                patches.resize(total);
        }
    }

    void SetHookPatchSource(HookPatchSource source)
    {
        g_hookPatchSource.store(source, std::memory_order_release);
    }

    void SetProofThroughHookObserver(ProofThroughHookObserver observer)
    {
        g_proofThroughHookObserver.store(observer, std::memory_order_release);
    }

    uint32_t CopyUnhooked(uintptr_t address, void* out, size_t size)
    {
        if (!address || !out || !size)
            return 0;
        memcpy(out, reinterpret_cast<const void*>(address), size);
        std::vector<HookPatch> patches;
        EnabledPatches(address, size, patches);
        uint32_t undone = 0;
        auto* bytes = static_cast<uint8_t*>(out);
        for (const HookPatch& patch : patches)
        {
            bool changed = false;
            for (size_t k = 0; k < patch.size && k < sizeof(patch.original); ++k)
            {
                const uintptr_t at = patch.address + k;
                if (at < address || at - address >= size)
                    continue;
                changed = changed || bytes[at - address] != patch.original[k];
                bytes[at - address] = patch.original[k];
            }
            undone += changed ? 1u : 0u;
        }
        return undone;
    }

    void CorrectLiveCountForHooks(uintptr_t base, size_t size,
                                  const uint8_t* bytes, const uint8_t* wild,
                                  size_t n, uintptr_t& first, uint32_t& count)
    {
        if (!base || !bytes || !wild || !n || size < n || count >= 2)
            return;
        const HookPatchSource source = g_hookPatchSource.load(std::memory_order_acquire);
        if (!source)
            return;
        // Fixed storage: the caller's scan is SEH-guarded and must not leak.
        constexpr uint32_t kCapacity = 256;
        HookPatch patches[kCapacity];
        const uint32_t total = source(base, size, patches, kCapacity);
        const uint32_t patchCount = total < kCapacity ? total : kCapacity;
        if (!patchCount)
            return;
        const uint8_t* data = reinterpret_cast<const uint8_t*>(base);
        const size_t last = size - n;
        const auto unhooked = [&](size_t at) -> uint8_t {
            const uintptr_t address = base + at;
            for (uint32_t p = 0; p < patchCount; ++p)
                if (address >= patches[p].address && address - patches[p].address < patches[p].size &&
                    address - patches[p].address < sizeof(patches[p].original))
                    return patches[p].original[address - patches[p].address];
            return data[at];
        };
        const auto matchesUnhooked = [&](size_t at) {
            for (size_t j = 0; j < n; ++j)
                if (!wild[j] && unhooked(at + j) != bytes[j])
                    return false;
            return true;
        };
        const auto matchesLive = [&](size_t at) {
            for (size_t j = 0; j < n; ++j)
                if (!wild[j] && data[at + j] != bytes[j])
                    return false;
            return true;
        };
        // The offsets whose window reads patch p: [firstTouching(p), lastTouching(p)].
        const auto firstTouching = [&](uint32_t p) -> size_t {
            return patches[p].address > base + (n - 1)
                ? static_cast<size_t>(patches[p].address - base) - (n - 1) : 0;
        };
        const auto lastTouching = [&](uint32_t p) -> size_t {
            return static_cast<size_t>(patches[p].address + patches[p].size - 1 - base);
        };

        uintptr_t newFirst = 0, throughPatch = 0;
        uint32_t newCount = 0;
        // The live match stands unless it exists only because of our jump.
        if (count == 1 && first >= base && first - base <= last && matchesUnhooked(first - base))
        {
            newFirst = first;
            newCount = 1;
        }
        // Matches the live scan could not see: windows that read a hook.
        for (uint32_t p = 0; p < patchCount; ++p)
        {
            if (patches[p].address + patches[p].size <= base)
                continue;
            for (size_t at = firstTouching(p); at <= lastTouching(p) && at <= last; ++at)
            {
                bool seen = false; // two hooks closer than a pattern share offsets
                for (uint32_t q = 0; q < p && !seen; ++q)
                    seen = patches[q].address + patches[q].size > base &&
                           at >= firstTouching(q) && at <= lastTouching(q);
                if (seen || matchesLive(at) || !matchesUnhooked(at))
                    continue;
                ++newCount;
                if (!newFirst || base + at < newFirst)
                {
                    newFirst = base + at;
                    throughPatch = patches[p].address;
                }
            }
        }
        first = newFirst;
        count = newCount;
        if (newFirst && throughPatch)
            if (const ProofThroughHookObserver observer =
                    g_proofThroughHookObserver.load(std::memory_order_acquire))
                observer(newFirst, n, throughPatch);
    }

    uintptr_t Find(uintptr_t base, size_t size, const char* pattern)
    {
        std::vector<uint8_t> bytes;
        std::vector<uint8_t> wild;
        if (!ParsePattern(pattern, bytes, wild))
            return 0;
        const size_t n = bytes.size();
        if (!n || size < n)
            return 0;

        const uint8_t* data = reinterpret_cast<const uint8_t*>(base);
        const size_t last = size - n;

        // First concrete byte to anchor the skip on.
        size_t anchor = 0;
        while (anchor < n && wild[anchor])
            ++anchor;
        // An all-wildcard pattern matches at the first offset, as before.
        if (anchor == n)
            return base;

        std::vector<HookPatch> patches;
        EnabledPatches(base, size, patches);
        if (patches.empty())
        {
            const size_t i = FindLive(data, 0, last, bytes, wild, anchor);
            return i <= last ? base + i : 0;
        }

        // The byte at image offset `at` as it stood before our hooks.
        const auto unhooked = [&](size_t at) -> uint8_t {
            const uintptr_t address = base + at;
            for (const HookPatch& patch : patches)
                if (address >= patch.address && address - patch.address < patch.size &&
                    address - patch.address < sizeof(patch.original))
                    return patch.original[address - patch.address];
            return data[at];
        };
        const auto matchesUnhooked = [&](size_t at) {
            for (size_t j = 0; j < n; ++j)
                if (!wild[j] && unhooked(at + j) != bytes[j])
                    return false;
            return true;
        };
        const auto matchesLive = [&](size_t at) {
            for (size_t j = 0; j < n; ++j)
                if (!wild[j] && data[at + j] != bytes[j])
                    return false;
            return true;
        };
        // The hook whose rewritten bytes the window at `at` reads, if any.
        const auto touched = [&](size_t at) -> const HookPatch* {
            const uintptr_t lo = base + at, hi = lo + n;
            for (const HookPatch& patch : patches)
                if (patch.address < hi && lo < patch.address + patch.size)
                    return &patch;
            return nullptr;
        };

        // 1. The first live match that is also a match of the game's own
        //    bytes. A live match across one of our jumps that the saved bytes
        //    do not confirm is our own artefact: skip it.
        size_t best = last + 1;
        for (size_t from = 0; from <= last;)
        {
            const size_t i = FindLive(data, from, last, bytes, wild, anchor);
            if (i > last)
                break;
            if (!touched(i) || matchesUnhooked(i))
            {
                best = i;
                break;
            }
            from = i + 1;
        }
        // 2. Matches the live scan cannot see: windows that read a hook. Only
        //    offsets below `best` can change the answer.
        for (const HookPatch& patch : patches)
        {
            const uintptr_t patchEnd = patch.address + patch.size; // exclusive
            if (patchEnd <= base)
                continue;
            const size_t lastTouching = static_cast<size_t>(patchEnd - 1 - base);
            const size_t firstTouching = patch.address > base + (n - 1)
                ? static_cast<size_t>(patch.address - base) - (n - 1) : 0;
            for (size_t at = firstTouching; at <= lastTouching && at <= last && at < best; ++at)
                if (matchesUnhooked(at))
                {
                    best = at;
                    break;
                }
        }
        if (best > last)
            return 0;
        if (const HookPatch* patch = touched(best); patch && !matchesLive(best))
            if (const ProofThroughHookObserver observer =
                    g_proofThroughHookObserver.load(std::memory_order_acquire))
                observer(base + best, n, patch->address);
        return base + best;
    }

    bool ModuleRange(const wchar_t* moduleName, uintptr_t& base, size_t& size)
    {
        HMODULE mod = GetModuleHandleW(moduleName);
        if (!mod)
            return false;
        MODULEINFO mi{};
        if (!GetModuleInformation(GetCurrentProcess(), mod, &mi, sizeof(mi)))
            return false;
        base = reinterpret_cast<uintptr_t>(mi.lpBaseOfDll);
        size = mi.SizeOfImage;
        return true;
    }

    bool SelfModuleRange(uintptr_t& base, size_t& size)
    {
        HMODULE mod = nullptr;
        if (!GetModuleHandleExW(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                    GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                reinterpret_cast<LPCWSTR>(&SelfModuleRange), &mod))
            return false;
        MODULEINFO mi{};
        if (!GetModuleInformation(GetCurrentProcess(), mod, &mi, sizeof(mi)))
            return false;
        base = reinterpret_cast<uintptr_t>(mi.lpBaseOfDll);
        size = mi.SizeOfImage;
        return true;
    }

    uintptr_t FindInModule(const wchar_t* moduleName, const char* pattern)
    {
        uintptr_t base = 0;
        size_t size = 0;
        if (!ModuleRange(moduleName, base, size))
            return 0;
        return Find(base, size, pattern);
    }

    uintptr_t RipTarget(uintptr_t dispAddr, uintptr_t nextInstr)
    {
        const int32_t disp = *reinterpret_cast<const int32_t*>(dispAddr);
        return nextInstr + disp;
    }
}
