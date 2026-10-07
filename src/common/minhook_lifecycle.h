#pragma once
#include <MinHook.h>

#ifdef __cplusplus
extern "C" {
#endif
// Worker only. Disables an exact live MinHook patch normally, or marks an
// absent mapping / already restored entry disabled WITHOUT writing it.
// This never frees a trampoline. Callers must prove detour quiescence before
// MH_RemoveHook, including when this returns MH_ERROR_DISABLED.
MH_STATUS WINAPI MCCVR_DisableHookForRetirement(LPVOID target);

// One of this mod's ENABLED MinHook patches: the bytes it overwrote (`address`, `size` of them) and what stood
// there before (`original`). A normal hook rewrites five bytes at the target (E9 rel32); a hot-patch hook rewrites
// the five bytes above it and two at it.
typedef struct MCCVR_HOOK_PATCH
{
    const unsigned char* address;
    unsigned char size;
    unsigned char original[8];
} MCCVR_HOOK_PATCH;
// Any thread. Copies up to `capacity` enabled patches that intersect [base, base + size) and returns how many
// there are in all (a return above `capacity` means the copy is incomplete). This is how a byte-pattern proof
// reads the GAME's code where this mod has hooked it: a signature is a statement about the image as it shipped,
// and after MH_EnableHook the head of a hooked function reads our jump instead (sigscan.h, sig::Find).
UINT WINAPI MCCVR_CopyEnabledHookPatches(LPCVOID base, SIZE_T size, MCCVR_HOOK_PATCH* out, UINT capacity);
#ifdef __cplusplus
}
#endif
