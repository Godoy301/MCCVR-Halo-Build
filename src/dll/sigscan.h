#pragma once
#include <cstdint>
#include <cstddef>

// AOB (array-of-bytes) signature scanning. We locate game code/data by the
// bytes of the surrounding instructions rather than hardcoded addresses, so
// the mod keeps working when an MCC update shifts everything around. When a
// pattern stops matching after a patch, we detect it and warn instead of
// crashing (that's the whole reason we scan).

namespace sig
{
    // Pattern is space-separated hex bytes with "??" as a wildcard, e.g.
    //   "48 8B 05 ?? ?? ?? ?? 48 85 C0"
    // Returns the address of the first match inside [base, base+size), or 0.
    //
    // THE IMAGE THAT IS MATCHED IS THE GAME'S, NOT THE ONE WE HAVE REWRITTEN.
    // Where this mod has an enabled hook (SetHookPatchSource) the bytes compared
    // are the ones that stood there before the hook, so a proof does not depend
    // on whether it runs before or after another lane hooked the same function.
    // Two muzzle lanes were lost to that order: Halo 4's (+0x3B1B4C, the avatar
    // lane's producer hook) and Reach's (+0x484F24, the unit-adjust hook), each
    // logging "missing/ambiguous binding" on a healthy game, for every player.
    // A match that had to read through one of our hooks is reported once
    // (SetProofThroughHookObserver): it still wants moving ahead of the hook,
    // because a direct byte compare at the same address has no such net.
    uintptr_t Find(uintptr_t base, size_t size, const char* pattern);

    // One enabled hook of this mod: the bytes it overwrote, and what stood
    // there before.
    struct HookPatch
    {
        uintptr_t address = 0;
        uint8_t size = 0;
        uint8_t original[8]{};
    };
    // Copies up to `capacity` enabled patches that intersect [base, base+size)
    // and returns how many there are in all.
    using HookPatchSource = uint32_t (*)(uintptr_t base, size_t size,
                                         HookPatch* out, uint32_t capacity);
    // `match` is where a pattern of `length` bytes matched only because the
    // hook at `patch` was read as the bytes it replaced.
    using ProofThroughHookObserver = void (*)(uintptr_t match, size_t length,
                                              uintptr_t patch);
    // Set once at startup, before any title is scanned. Null (the default)
    // scans the live bytes exactly as before.
    void SetHookPatchSource(HookPatchSource source);
    void SetProofThroughHookObserver(ProofThroughHookObserver observer);

    // For a scanner that walks an image itself and counts matches (Halo 2's
    // guarded region walker, CountPatternMatches). Given the result of its LIVE
    // scan of [base, base+size) - the first match, and the count, counted at
    // least up to 2 - makes it the result of the same scan over the image with
    // this mod's enabled hooks undone. Exact while the live count is below 2; a
    // live count of 2 or more stays ambiguous. `wild[i]` != 0 is a wildcard.
    // Reads only bytes within n-1 of an enabled hook, inside [base, base+size);
    // allocates nothing. Reports a match read through a hook like Find does.
    void CorrectLiveCountForHooks(uintptr_t base, size_t size,
                                  const uint8_t* bytes, const uint8_t* wild,
                                  size_t n, uintptr_t& first, uint32_t& count);

    // The bytes at [address, address+size) with this mod's enabled hooks
    // undone, for a proof that compares bytes at a fixed place instead of
    // scanning. Returns how many hooks it read through. The range must be
    // readable.
    uint32_t CopyUnhooked(uintptr_t address, void* out, size_t size);

    // Same, but over a loaded module's image (by name, e.g. L"halo3.dll").
    // Returns 0 if the module isn't loaded or the pattern isn't found.
    uintptr_t FindInModule(const wchar_t* moduleName, const char* pattern);

    // Resolve a 32-bit RIP-relative reference (the addressing mode x64 uses for
    // things like `mov rax,[rip+disp32]`). dispAddr points at the 4-byte
    // displacement; nextInstr is the address of the instruction that follows
    // (RIP's value when the displacement is applied). Returns the target.
    uintptr_t RipTarget(uintptr_t dispAddr, uintptr_t nextInstr);

    // Base address and size of a loaded module's image, for bounding a scan.
    bool ModuleRange(const wchar_t* moduleName, uintptr_t& base, size_t& size);

    // Base address and size of the module containing this code, resolved by
    // address rather than file name, so it survives the DLL being renamed.
    bool SelfModuleRange(uintptr_t& base, size_t& size);
}
