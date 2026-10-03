// HaloMCCFlatProbe.dll -- THE FLAT CONTROL ARM for the AS SEEN measurement (dev tooling, 2026-10-03; never packaged).
//
// The user strafes with the assault rifle in Halo 3: "in flat I see the impact marks happen at the reticle; in VR
// they are offset by practically half the reticle behind". The VR dev build measures what the headset shows
// (halo3_impact_witness.inl, AS SEEN). This DLL measures the SAME thing in stock flat Halo 3, with the same code
// (src/common/as_seen_logic.h), so the two numbers can be compared bin for bin.
//
// OBSERVE ONLY. It is loaded without the VR mod (HaloMCCVRLauncher.exe --flat-probe, dev builds) and touches nothing:
// no OpenXR, no stereo, no swapchain or presentation change, no aim or record write, no input hook, no camera or seat
// patch. Its hooks forward every argument and the return value exactly and only READ:
//   * the camera copy (the VR mod's kCamCopySig, halo3.dll; the same prototype as its audited CamCopyHook): after the
//     native copy, the render camera's position, forward, up and projection tangents;
//   * object_new (the impact witness's anchors): the fire routine's projectile creation, by its return address --
//     the placement's position and forward;
//   * the collision scheduler (the VR mod's kHalo3CollisionVectorSignature, its 8-argument prototype): after the
//     native query, a tracked round's own query and, when it hits, the hit point;
//   * IDXGISwapChain::Present / Present1: the frame boundary and its time.
// Every signature must match exactly once or that hook is not installed (and the probe says so); every read of game
// memory is under SEH. ADDR-HYGIENE: dev-only (a measurement tool; it decides nothing and writes nothing).
//
// A round is the local player's when it spawns within 1.5 m of the render camera and its forward is within 30 deg of
// the camera's (campaign: an ally standing that close firing that way is rare; counted rounds are logged). Its
// collision queries are matched by geometry: they start on its spawn line and run along its forward (no object read is
// needed). Its first presented frame is the first Present whose frame's camera was copied after the hit.
//
// Lifetime: a worker thread waits for halo3.dll, installs, and retires the hooks (no-write retirement,
// minhook_lifecycle) if MCC unloads that module, installing again when it comes back. Log: HaloMCCFlatProbe.log next to
// this DLL, one AS SEEN line per ~2 s window with impacts, plus the camera census.

#include <windows.h>
#include <d3d11.h>
#include <dxgi1_2.h>
#include <intrin.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cwchar>

#include <MinHook.h>

#include "../common/as_seen_logic.h"
#include "../common/log.h"
#include "../common/minhook_lifecycle.h"
#include "../dll/sigscan.h"

namespace
{
// ---- the anchors (copied from the VR mod, which proves and uses each; each must match exactly once here) ---------
// game.cpp kCamCopySig: mov [rsp+8],rbx; push rdi; sub rsp,0x30; movaps [rsp+0x20],xmm6; mov rdi,rdx; mov rbx,rcx; ...
constexpr char kCamCopySig[] =
    "48 89 5C 24 08 57 48 83 EC 30 0F 29 74 24 20 48 8B FA 48 8B D9 48 85 D2 74 ?? F3 0F 10 1D ?? ?? ?? ??";
// halo3_impact_witness.inl: object_new's prologue, its placement copy, the fire routine's call to it.
constexpr char kObjectNewSig[] =
    "48 8B C4 53 55 56 57 41 54 41 55 41 56 41 57 48 83 EC 78 83 CD FF 45 33 E4 F6 41 18 10 48 8B F1 8B FD 44 88 "
    "60 10";
constexpr char kPlacementCopySig[] =
    "F2 0F 10 46 ?? F2 0F 11 43 ?? 8B 46 ?? 89 43 ?? F2 0F 10 46 ?? F2 0F 11 43 ?? 8B 46 ?? 89 43 ?? F2 0F 10 46 ?? "
    "F2 0F 11 43 ?? 8B 46 ?? 89 43 ?? F2 0F 10 46 ?? F2 0F 11 43 ?? 8B 46 ?? 89 43 ??";
constexpr char kFireCallSig[] =
    "48 8D 8D ?? ?? 00 00 E8 ?? ?? ?? ?? F3 44 0F 10 4C 24 ?? 41 83 C9 FF 89 44 24 ?? 41 3B C1 0F 84";
// game.cpp kHalo3CollisionVectorSignature: the active collision scheduler (the VR mod's Halo3CollisionResolveDetour).
constexpr char kCollisionSig[] =
    "48 8B C4 4C 89 40 18 88 50 10 48 89 48 08 55 53 56 57 41 54 "
    "41 55 41 56 41 57 48 8D A8 18 BB FF FF B8 A8 45 00 00 E8 ?? ?? ?? ?? 48 2B E0";
// The camera copy's source record (game.cpp kSrcPos / kSrcFwd / kSrcUp / kSrcProjX / kSrcProjY).
// ADDR-HYGIENE: dev-only (the VR mod reads the same fields every frame; here they are only read).
constexpr size_t kSrcPos = 0x00, kSrcFwd = 0x28, kSrcUp = 0x34, kSrcProjX = 0x68, kSrcProjY = 0x6C;

using CamCopyFn = void*(__fastcall*)(void*, void*);
using ObjectNewFn = int32_t(__fastcall*)(void*);
using CollisionFn = uint8_t(__fastcall*)(uint64_t, int32_t, const float*, const float*, int32_t, int32_t, int32_t,
                                          void*);
using PresentFn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain*, UINT, UINT);
using Present1Fn = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain1*, UINT, UINT, const DXGI_PRESENT_PARAMETERS*);

uint64_t NowUs()
{
    static LARGE_INTEGER frequency{};
    if (!frequency.QuadPart)
        QueryPerformanceFrequency(&frequency);
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    return static_cast<uint64_t>(now.QuadPart / frequency.QuadPart * 1000000 +
                                 now.QuadPart % frequency.QuadPart * 1000000 / frequency.QuadPart);
}

// ---- state ---------------------------------------------------------------------------------------------------------
struct Camera
{
    bool valid = false;
    uint64_t us = 0;
    float pos[3]{}, fwd[3]{}, up[3]{};
    float tanX = 0.0f, tanY = 0.0f;
};

struct Round
{
    bool used = false;
    uint64_t fireUs = 0;
    float spawn[3]{}, fwd[3]{};
    Camera aim{};             // the newest camera copied before the fire: the crosshair the player fired through
    bool velValid = false;
    float vel[3]{};           // the camera eye's velocity at the fire (wu/s)
    uint32_t queries = 0;
};

struct Pending
{
    bool used = false;
    float impact[3]{};
    uint64_t impactUs = 0;
    Round round{};
};

struct Probe
{
    SRWLOCK lock = SRWLOCK_INIT;
    // Hooks.
    void* camTarget = nullptr;
    void* newTarget = nullptr;
    void* colTarget = nullptr;
    CamCopyFn camOriginal = nullptr;
    ObjectNewFn newOriginal = nullptr;
    CollisionFn colOriginal = nullptr;
    std::atomic<uint32_t> callbacks{0};
    std::atomic<bool> ready{false};
    HMODULE module = nullptr;
    uintptr_t fireReturn = 0;
    uint32_t placePos = 0, placeFwd = 0;
    // Present (dxgi.dll: installed once, never retired).
    PresentFn presentOriginal = nullptr;
    Present1Fn present1Original = nullptr;
    // Under `lock`.
    Camera latest{};                  // the newest camera copy
    Camera frame{};                   // the first camera copied since the last Present: this frame's
    uint32_t framePasses = 0;         // camera copies in this frame
    float frameSpreadMm = 0.0f;       // the largest distance of another pass's camera from the frame's
    static constexpr int kRounds = 16, kPending = 16;
    Round rounds[kRounds]{};
    Pending pending[kPending]{};
    as_seen::Window window{};
    as_seen::CameraCensus census{};   // per presented frame (also the source of the eye's velocity)
    // Window counters.
    uint32_t presents = 0, cameraCopies = 0, spawnsAtCall = 0, spawnsLocal = 0, spawnsForeign = 0, queriesMatched = 0,
             queriesAmbiguous = 0, roundsLost = 0, roundsEvicted = 0, faults = 0;
    double passesSum = 0.0, spreadMaxMm = 0.0, tanXSum = 0.0;
    uint32_t tanXN = 0;
    uint64_t firstPresentUs = 0, lastPresentUs = 0;
};
Probe g_probe;
float g_vrHudWidth = 5242.0f;   // the VR HUD raster width the art fraction refers to (HaloMCCFlatProbe.cfg)

bool Read3(const void* at, float out[3])
{
    __try
    {
        std::memcpy(out, at, sizeof(float) * 3);
        return as_seen::Finite3(out);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
bool Read1(const void* at, float& out)
{
    __try
    {
        std::memcpy(&out, at, sizeof(float));
        return std::isfinite(out);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// The camera eye's velocity from the census history (40-200 ms back), under the lock.
bool EyeVelocity(const float pos[3], uint64_t us, float out[3])
{
    const auto& c = g_probe.census;
    for (int k = 1; k <= c.count; ++k)
    {
        const int i = (c.head + as_seen::CameraCensus::kHistory - k) % as_seen::CameraCensus::kHistory;
        if (c.us[i] >= us)
            continue;
        const uint64_t dt = us - c.us[i];
        if (dt < 40000)
            continue;
        if (dt > 200000)
            return false;
        for (int axis = 0; axis < 3; ++axis)
            out[axis] = (pos[axis] - c.pos[i][axis]) / (float(dt) * 1.0e-6f);
        return as_seen::Finite3(out);
    }
    return false;
}

// A presented frame (under the lock): the census, and every impact this frame is the first to show.
void FinishFrame(uint64_t presentUs)
{
    auto& p = g_probe;
    ++p.presents;
    if (!p.firstPresentUs)
        p.firstPresentUs = presentUs;
    p.lastPresentUs = presentUs;
    if (!p.frame.valid)
        return;
    p.passesSum += p.framePasses;
    p.spreadMaxMm = p.frameSpreadMm > p.spreadMaxMm ? p.frameSpreadMm : p.spreadMaxMm;
    float vel[3]{};
    const bool velKnown = EyeVelocity(p.frame.pos, p.frame.us, vel);
    p.census.Add(p.frame.pos, p.frame.us);
    for (auto& q : p.pending)
    {
        if (!q.used || q.impactUs > p.frame.us)
            continue;
        const float* v = velKnown ? vel : (q.round.velValid ? q.round.vel : nullptr);
        as_seen::Sample s{};
        s.seen = as_seen::Measure(q.impact, p.frame.pos, p.frame.fwd, v);
        s.stockValid = true;   // in flat the crosshair on screen IS the stock one
        s.stock = s.seen;
        s.atFireValid = q.round.aim.valid;
        if (s.atFireValid)
            s.atFire = as_seen::Measure(q.impact, q.round.aim.pos, q.round.aim.fwd, v);
        s.aimCamUs = q.round.aim.us;
        s.fireUs = q.round.fireUs;
        s.impactUs = q.impactUs;
        s.frameCamUs = p.frame.us;
        s.presentUs = presentUs;
        s.ticksFireToImpact = q.round.queries ? static_cast<int32_t>(q.round.queries - 1) : -1;
        if (q.round.aim.valid)
        {
            const float moved[3]{p.frame.pos[0] - q.round.aim.pos[0], p.frame.pos[1] - q.round.aim.pos[1],
                                 p.frame.pos[2] - q.round.aim.pos[2]};
            s.eyeMovedAlongMm = as_seen::AlongTravelMm(moved, p.frame.fwd, v, &s.eyeMovedKnown);
        }
        const float radius = as_seen::FlatArtHalfAngleDeg(g_vrHudWidth, p.frame.tanX);
        p.window.Add(s, std::isfinite(radius) ? radius : 0.0f);
        q = Pending{};
    }
    p.frame = Camera{};
    p.framePasses = 0;
    p.frameSpreadMm = 0.0f;
}

// ---- the detours (forward exactly; read only) -------------------------------------------------------------------
__declspec(noinline) void* __fastcall CamCopyDetour(void* dst, void* src)
{
    auto& p = g_probe;
    p.callbacks.fetch_add(1, std::memory_order_acq_rel);
    void* result = nullptr;
    __try
    {
        result = p.camOriginal ? p.camOriginal(dst, src) : nullptr;
        if (src && p.ready.load(std::memory_order_acquire))
        {
            const auto* bytes = static_cast<const uint8_t*>(src);
            Camera c{};
            c.us = NowUs();
            c.valid = Read3(bytes + kSrcPos, c.pos) && Read3(bytes + kSrcFwd, c.fwd) && Read3(bytes + kSrcUp, c.up) &&
                      Read1(bytes + kSrcProjX, c.tanX) && Read1(bytes + kSrcProjY, c.tanY);
            AcquireSRWLockExclusive(&p.lock);
            ++p.cameraCopies;
            if (c.valid)
            {
                p.latest = c;
                if (!p.frame.valid)
                {
                    p.frame = c;
                    p.framePasses = 1;
                    if (c.tanX > 0.2f && c.tanX < 3.0f)
                    {
                        p.tanXSum += c.tanX;
                        ++p.tanXN;
                    }
                }
                else
                {
                    ++p.framePasses;
                    const float d[3]{c.pos[0] - p.frame.pos[0], c.pos[1] - p.frame.pos[1], c.pos[2] - p.frame.pos[2]};
                    const float mm = as_seen::Length(d) * as_seen::kMmPerWu;
                    p.frameSpreadMm = mm > p.frameSpreadMm ? mm : p.frameSpreadMm;
                }
            }
            else
                ++p.faults;
            ReleaseSRWLockExclusive(&p.lock);
        }
    }
    __finally
    {
        p.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
    return result;
}

__declspec(noinline) int32_t __fastcall ObjectNewDetour(void* placement)
{
    auto& p = g_probe;
    p.callbacks.fetch_add(1, std::memory_order_acq_rel);
    int32_t result = -1;
    __try
    {
        const bool ours = reinterpret_cast<uintptr_t>(_ReturnAddress()) == p.fireReturn && placement &&
                          p.ready.load(std::memory_order_acquire);
        float spawn[3]{}, fwd[3]{};
        const bool placed = ours && Read3(static_cast<const uint8_t*>(placement) + p.placePos, spawn) &&
                            Read3(static_cast<const uint8_t*>(placement) + p.placeFwd, fwd);
        if (p.newOriginal)
            result = p.newOriginal(placement);
        if (placed && result != -1)
        {
            const uint64_t nowUs = NowUs();
            AcquireSRWLockExclusive(&p.lock);
            ++p.spawnsAtCall;
            const Camera cam = p.latest;
            bool local = false;
            if (cam.valid && nowUs - cam.us < 100000)
            {
                const float d[3]{spawn[0] - cam.pos[0], spawn[1] - cam.pos[1], spawn[2] - cam.pos[2]};
                const float deg = shot_lag::Length(d) * as_seen::kMetresPerWu < 1.5f
                    ? std::acos(std::fmax(-1.0f, std::fmin(1.0f, as_seen::Dot(fwd, cam.fwd) /
                                                              (as_seen::Length(fwd) * as_seen::Length(cam.fwd) +
                                                               1.0e-9f)))) * as_seen::kRadToDeg
                    : 180.0f;
                local = deg < 30.0f;
            }
            if (!local)
                ++p.spawnsForeign;
            else
            {
                ++p.spawnsLocal;
                Round* slot = nullptr;
                for (auto& r : p.rounds)
                    if (!r.used)
                    {
                        slot = &r;
                        break;
                    }
                if (!slot)
                {
                    slot = &p.rounds[0];
                    for (auto& r : p.rounds)
                        if (r.fireUs < slot->fireUs)
                            slot = &r;
                    ++p.roundsEvicted;
                }
                *slot = Round{};
                slot->used = true;
                slot->fireUs = nowUs;
                std::memcpy(slot->spawn, spawn, sizeof(spawn));
                std::memcpy(slot->fwd, fwd, sizeof(fwd));
                slot->aim = cam;
                slot->velValid = EyeVelocity(cam.pos, cam.us, slot->vel);
            }
            ReleaseSRWLockExclusive(&p.lock);
        }
    }
    __finally
    {
        p.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
    return result;
}

// A query that starts on a tracked round's spawn line (within 15 cm, not behind its spawn) and runs along its forward
// (within 5 deg) is that round's. The closest line wins; two within 1 cm of each other go to the older round.
void NoteQuery(const float* start, const float* vector, const void* result, uint8_t hit)
{
    auto& p = g_probe;
    float s[3]{}, v[3]{};
    if (!Read3(start, s) || !Read3(vector, v) || !(as_seen::Length(v) > 1.0e-6f))
        return;
    float point[3]{}, fraction = NAN;
    bool landed = false;
    if (hit && result)
    {
        landed = Read1(static_cast<const uint8_t*>(result) + 4, fraction) &&
                 Read3(static_cast<const uint8_t*>(result) + 8, point) && fraction >= 0.0f && fraction <= 1.0f;
    }
    const uint64_t nowUs = NowUs();
    AcquireSRWLockExclusive(&p.lock);
    Round* best = nullptr;
    float bestMiss = 0.0f;
    bool ambiguous = false;
    for (auto& r : p.rounds)
    {
        if (!r.used)
            continue;
        if (nowUs - r.fireUs > 3000000)
        {
            ++p.roundsLost;
            r = Round{};
            continue;
        }
        const float fl = as_seen::Length(r.fwd);
        if (!(fl > 1.0e-6f))
            continue;
        const float f[3]{r.fwd[0] / fl, r.fwd[1] / fl, r.fwd[2] / fl};
        const float cosine = as_seen::Dot(v, f) / as_seen::Length(v);
        if (!(cosine >= std::cos(5.0f / as_seen::kRadToDeg)))
            continue;
        const float w[3]{s[0] - r.spawn[0], s[1] - r.spawn[1], s[2] - r.spawn[2]};
        const float along = as_seen::Dot(w, f);
        if (along < -0.02f * as_seen::kWuPerMetre)
            continue;
        const float m[3]{w[0] - f[0] * along, w[1] - f[1] * along, w[2] - f[2] * along};
        const float miss = as_seen::Length(m) * as_seen::kMetresPerWu;
        if (!(miss <= 0.15f))
            continue;
        if (best && std::fabs(miss - bestMiss) < 0.01f)
        {
            ambiguous = true;
            if (r.fireUs < best->fireUs)
            {
                best = &r;
                bestMiss = miss;
            }
        }
        else if (!best || miss < bestMiss)
        {
            best = &r;
            bestMiss = miss;
        }
    }
    if (best)
    {
        ++p.queriesMatched;
        if (ambiguous)
            ++p.queriesAmbiguous;
        ++best->queries;
        if (landed)
        {
            Pending* slot = nullptr;
            for (auto& q : p.pending)
                if (!q.used)
                {
                    slot = &q;
                    break;
                }
            if (!slot)
            {
                slot = &p.pending[0];
                for (auto& q : p.pending)
                    if (q.impactUs < slot->impactUs)
                        slot = &q;
                ++p.window.unframed;
            }
            *slot = Pending{};
            slot->used = true;
            std::memcpy(slot->impact, point, sizeof(point));
            slot->impactUs = nowUs;
            slot->round = *best;
            *best = Round{};
        }
    }
    ReleaseSRWLockExclusive(&p.lock);
}

__declspec(noinline) uint8_t __fastcall CollisionDetour(uint64_t flags, int32_t mode, const float* start,
                                                        const float* desired, int32_t ignoredA, int32_t ignoredB,
                                                        int32_t ignoredC, void* result)
{
    auto& p = g_probe;
    p.callbacks.fetch_add(1, std::memory_order_acq_rel);
    uint8_t native = 0;
    __try
    {
        if (p.colOriginal)
            native = p.colOriginal(flags, mode, start, desired, ignoredA, ignoredB, ignoredC, result);
        if (p.ready.load(std::memory_order_acquire) && start && desired)
            NoteQuery(start, desired, result, native);
    }
    __finally
    {
        p.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
    return native;
}

void NotePresent()
{
    const uint64_t nowUs = NowUs();
    AcquireSRWLockExclusive(&g_probe.lock);
    FinishFrame(nowUs);
    ReleaseSRWLockExclusive(&g_probe.lock);
}

HRESULT STDMETHODCALLTYPE PresentDetour(IDXGISwapChain* chain, UINT sync, UINT flags)
{
    if (!(flags & DXGI_PRESENT_TEST))
        NotePresent();
    return g_probe.presentOriginal(chain, sync, flags);
}

HRESULT STDMETHODCALLTYPE Present1Detour(IDXGISwapChain1* chain, UINT sync, UINT flags,
                                         const DXGI_PRESENT_PARAMETERS* parameters)
{
    if (!(flags & DXGI_PRESENT_TEST))
        NotePresent();
    return g_probe.present1Original(chain, sync, flags, parameters);
}

// ---- install / retire --------------------------------------------------------------------------------------------
uintptr_t Unique(uintptr_t base, size_t size, const char* pattern)
{
    const uintptr_t hit = sig::Find(base, size, pattern);
    if (!hit || sig::Find(hit + 1, base + size - hit - 1, pattern))
        return 0;
    return hit;
}

uintptr_t FunctionOf(uintptr_t address)
{
    DWORD64 imageBase = 0;
    const auto* entry = RtlLookupFunctionEntry(static_cast<DWORD64>(address), &imageBase, nullptr);
    return entry ? static_cast<uintptr_t>(imageBase) + entry->BeginAddress : 0;
}

bool Hook(void* target, void* detour, void** original, const char* name)
{
    if (MH_CreateHook(target, detour, original) != MH_OK)
    {
        LOG("probe: %s: the hook was not created", name);
        *original = nullptr;
        return false;
    }
    if (MH_EnableHook(target) != MH_OK)
    {
        LOG("probe: %s: the hook did not enable", name);
        MH_RemoveHook(target);
        *original = nullptr;
        return false;
    }
    return true;
}

void Retire(const char* why)
{
    auto& p = g_probe;
    p.ready.store(false, std::memory_order_release);
    void* const targets[]{p.camTarget, p.newTarget, p.colTarget};
    for (void* target : targets)
        if (target)
            MCCVR_DisableHookForRetirement(target);
    // Quiescence: nothing is inside a detour any more (they are reached only through the patches just disabled).
    for (int i = 0; i < 200 && p.callbacks.load(std::memory_order_acquire); ++i)
        Sleep(10);
    if (p.callbacks.load(std::memory_order_acquire))
    {
        LOG("probe: retiring (%s): a detour is still busy; the hooks stay disabled and are never removed", why);
        p.camTarget = p.newTarget = p.colTarget = nullptr;
        return;
    }
    for (void* target : targets)
        if (target)
            MH_RemoveHook(target);
    p.camTarget = p.newTarget = p.colTarget = nullptr;
    p.camOriginal = nullptr;
    p.newOriginal = nullptr;
    p.colOriginal = nullptr;
    p.module = nullptr;
    AcquireSRWLockExclusive(&p.lock);
    for (auto& r : p.rounds)
        r = Round{};
    for (auto& q : p.pending)
        q = Pending{};
    p.latest = p.frame = Camera{};
    ReleaseSRWLockExclusive(&p.lock);
    LOG("probe: hooks retired (%s)", why);
}

bool InstallHalo3(HMODULE module)
{
    auto& p = g_probe;
    uintptr_t base = 0;
    size_t size = 0;
    if (!sig::ModuleRange(L"halo3.dll", base, size) || base != reinterpret_cast<uintptr_t>(module))
        return false;
    const uintptr_t cam = Unique(base, size, kCamCopySig);
    const uintptr_t entry = Unique(base, size, kObjectNewSig);
    const uintptr_t copy = Unique(base, size, kPlacementCopySig);
    const uintptr_t call = Unique(base, size, kFireCallSig);
    const uintptr_t col = Unique(base, size, kCollisionSig);
    LOG("probe: halo3.dll at %p (%zu bytes): camera copy %s, object_new %s, its placement copy %s, the fire's call "
        "%s, collision scheduler %s", module, size, cam ? "unique" : "MISSING/AMBIGUOUS",
        entry ? "unique" : "MISSING/AMBIGUOUS", copy ? "unique" : "MISSING/AMBIGUOUS",
        call ? "unique" : "MISSING/AMBIGUOUS", col ? "unique" : "MISSING/AMBIGUOUS");
    if (!cam || !entry || !copy || !call || !col)
    {
        LOG("probe: an anchor did not prove; nothing is hooked in this halo3.dll (stock game untouched)");
        return false;
    }
    int32_t rel = 0;
    std::memcpy(&rel, reinterpret_cast<const void*>(call + 8), sizeof(rel));
    const auto* c = reinterpret_cast<const uint8_t*>(copy);
    const uint32_t placePos = c[4], objPos = c[9], placePosZ = c[0x0C], objPosZ = c[0x0F];
    const uint32_t placeFwd = c[0x14], objFwd = c[0x19];
    const bool coherent = placePosZ == placePos + 8 && objPosZ == objPos + 8 && placeFwd == placePos + 0xC &&
                          objFwd == objPos + 0xC;
    const bool inObjectNew = FunctionOf(copy) == entry;
    const bool callsObjectNew = call + 12 + static_cast<intptr_t>(rel) == entry;
    if (!coherent || !inObjectNew || !callsObjectNew)
    {
        LOG("probe: the decoded layout did not prove (placement copy coherent %d, inside object_new %d, the fire's "
            "call targets it %d); nothing is hooked", coherent ? 1 : 0, inObjectNew ? 1 : 0, callsObjectNew ? 1 : 0);
        return false;
    }
    p.fireReturn = call + 12;
    p.placePos = placePos;
    p.placeFwd = placeFwd;
    p.module = module;
    if (!Hook(reinterpret_cast<void*>(cam), reinterpret_cast<void*>(&CamCopyDetour),
              reinterpret_cast<void**>(&p.camOriginal), "camera copy"))
        return false;
    p.camTarget = reinterpret_cast<void*>(cam);
    if (!Hook(reinterpret_cast<void*>(entry), reinterpret_cast<void*>(&ObjectNewDetour),
              reinterpret_cast<void**>(&p.newOriginal), "object_new"))
    {
        Retire("object_new failed");
        return false;
    }
    p.newTarget = reinterpret_cast<void*>(entry);
    if (!Hook(reinterpret_cast<void*>(col), reinterpret_cast<void*>(&CollisionDetour),
              reinterpret_cast<void**>(&p.colOriginal), "collision scheduler"))
    {
        Retire("collision failed");
        return false;
    }
    p.colTarget = reinterpret_cast<void*>(col);
    p.ready.store(true, std::memory_order_release);
    LOG("probe: OBSERVING Halo 3 -- camera copy +0x%llX, object_new +0x%llX (fire call return +0x%llX, placement "
        "position +0x%X forward +0x%X), collision scheduler +0x%llX",
        static_cast<unsigned long long>(cam - base), static_cast<unsigned long long>(entry - base),
        static_cast<unsigned long long>(p.fireReturn - base), placePos, placeFwd,
        static_cast<unsigned long long>(col - base));
    return true;
}

// Present / Present1 of a throwaway swapchain's vtable (dxgi.dll's own code, shared by MCC's chain). Installed once.
bool InstallPresent()
{
    WNDCLASSEXW wc{sizeof(wc)};
    wc.lpfnWndProc = DefWindowProcW;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.lpszClassName = L"halomcc_flat_probe_dummy";
    RegisterClassExW(&wc);
    HWND window = CreateWindowExW(0, wc.lpszClassName, L"", WS_OVERLAPPED, 0, 0, 64, 64, nullptr, nullptr,
                                  wc.hInstance, nullptr);
    if (!window)
        return false;
    DXGI_SWAP_CHAIN_DESC desc{};
    desc.BufferCount = 1;
    desc.BufferDesc.Width = 64;
    desc.BufferDesc.Height = 64;
    desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    desc.OutputWindow = window;
    desc.SampleDesc.Count = 1;
    desc.Windowed = TRUE;
    desc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    IDXGISwapChain* chain = nullptr;
    ID3D11Device* device = nullptr;
    ID3D11DeviceContext* context = nullptr;
    D3D_FEATURE_LEVEL level{};
    const HRESULT hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0,
                                                     D3D11_SDK_VERSION, &desc, &chain, &device, &level, &context);
    bool ok = false;
    if (SUCCEEDED(hr) && chain)
    {
        void** table = *reinterpret_cast<void***>(chain);
        ok = Hook(table[8], reinterpret_cast<void*>(&PresentDetour),
                  reinterpret_cast<void**>(&g_probe.presentOriginal), "Present");
        IDXGISwapChain1* chain1 = nullptr;
        if (SUCCEEDED(chain->QueryInterface(__uuidof(IDXGISwapChain1), reinterpret_cast<void**>(&chain1))) && chain1)
        {
            void** table1 = *reinterpret_cast<void***>(chain1);
            if (!Hook(table1[22], reinterpret_cast<void*>(&Present1Detour),
                      reinterpret_cast<void**>(&g_probe.present1Original), "Present1"))
                LOG("probe: Present1 not hooked (Present alone marks frames)");
            chain1->Release();
        }
    }
    else
        LOG("probe: the throwaway D3D11 device was not created (hr 0x%08X)", static_cast<unsigned>(hr));
    if (context)
        context->Release();
    if (device)
        device->Release();
    if (chain)
        chain->Release();
    DestroyWindow(window);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return ok;
}

void Report()
{
    auto& p = g_probe;
    AcquireSRWLockExclusive(&p.lock);
    // An impact no frame showed within half a second is dropped (counted).
    const uint64_t nowUs = NowUs();
    for (auto& q : p.pending)
        if (q.used && nowUs > q.impactUs + 500000)
        {
            ++p.window.unframed;
            q = Pending{};
        }
    const as_seen::Window window = p.window;
    p.window = as_seen::Window{};
    char census[512]{};
    const bool censusAny = p.census.Format(census, sizeof(census));
    p.census.ResetCounts();
    const uint32_t presents = p.presents, copies = p.cameraCopies, atCall = p.spawnsAtCall, local = p.spawnsLocal,
                   foreign = p.spawnsForeign, matched = p.queriesMatched, ambiguous = p.queriesAmbiguous,
                   lost = p.roundsLost, evicted = p.roundsEvicted, faults = p.faults, tanXN = p.tanXN;
    const double passes = presents ? p.passesSum / presents : 0.0, spread = p.spreadMaxMm;
    const double tanX = tanXN ? p.tanXSum / tanXN : 0.0;
    const double seconds = p.lastPresentUs > p.firstPresentUs ? double(p.lastPresentUs - p.firstPresentUs) * 1e-6 : 0;
    p.presents = p.cameraCopies = p.spawnsAtCall = p.spawnsLocal = p.spawnsForeign = p.queriesMatched =
        p.queriesAmbiguous = p.roundsLost = p.roundsEvicted = p.faults = p.tanXN = 0;
    p.passesSum = p.spreadMaxMm = p.tanXSum = 0.0;
    p.firstPresentUs = p.lastPresentUs = 0;
    ReleaseSRWLockExclusive(&p.lock);
    if (!p.ready.load(std::memory_order_acquire) && !presents)
        return;
    static char line[4096];
    const float radius = as_seen::FlatArtHalfAngleDeg(g_vrHudWidth, static_cast<float>(tanX));
    char label[256];
    std::snprintf(label, sizeof(label),
                  "Halo 3 FLAT [vs the stock crosshair: the camera of the first presented frame after the impact, "
                  "along its forward; radii = the VR reticle's art in this view, %.2f deg]", radius);
    if (window.Format(label, line, sizeof(line)))
        LOG("%s", line);
    if (censusAny)
        LOG("AS SEEN Halo 3 FLAT camera (dev; per presented frame): %s", census);
    if (atCall || presents)
        LOG("AS SEEN Halo 3 FLAT probe (dev): %u presents (%.0f fps), %.1f camera copies a frame (other passes up to "
            "%.0f mm off the frame's), horizontal projection tangent %.3f (%.1f deg FOV); rounds from the fire "
            "routine %u: ours %u, not ours %u; their queries matched %u (%u ambiguous), lost %u, evicted %u; read "
            "faults %u", presents, seconds > 0 ? (presents - 1) / seconds : 0.0, passes, spread, tanX,
            tanX > 0 ? 2.0 * std::atan(tanX) * as_seen::kRadToDeg : 0.0, atCall, local, foreign, matched, ambiguous,
            lost, evicted, faults);
    (void)copies;
}

void ReadConfig(const wchar_t* directory)
{
    wchar_t path[MAX_PATH]{};
    std::swprintf(path, MAX_PATH, L"%lsHaloMCCFlatProbe.cfg", directory);
    FILE* file = nullptr;
    if (_wfopen_s(&file, path, L"rt") || !file)
        return;
    char text[256];
    while (std::fgets(text, sizeof(text), file))
    {
        float value = 0.0f;
        if (std::sscanf(text, " vr_hud_width = %f", &value) == 1 && std::isfinite(value) && value > 100.0f)
            g_vrHudWidth = value;
    }
    std::fclose(file);
}

DWORD WINAPI Worker(void*)
{
    wchar_t self[MAX_PATH]{};
    HMODULE me = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&Worker), &me);
    GetModuleFileNameW(me, self, MAX_PATH);
    wchar_t* slash = std::wcsrchr(self, L'\\');
    if (slash)
        slash[1] = L'\0';
    wchar_t logPath[MAX_PATH]{};
    std::swprintf(logPath, MAX_PATH, L"%lsHaloMCCFlatProbe.log", self);
    LogInit(logPath);
    ReadConfig(self);
    LOG("HaloMCCFlatProbe loaded into pid %lu (source %s, compiled %s %s): OBSERVE ONLY -- no VR, no writes; the AS "
        "SEEN control arm for stock flat Halo 3. vr_hud_width %.0f (HaloMCCFlatProbe.cfg: the VR HUD raster width "
        "the reticle radius refers to)", GetCurrentProcessId(), HALOMCCVR_BUILD_COMMIT, __DATE__, __TIME__,
        static_cast<double>(g_vrHudWidth));
    if (MH_Initialize() != MH_OK)
    {
        LOG("probe: MinHook did not initialise; nothing is hooked");
        return 0;
    }
    if (!InstallPresent())
        LOG("probe: Present is not hooked; impacts cannot be placed in frames (nothing else is hooked either)");
    else
        LOG("probe: Present hooked (frame boundaries); waiting for halo3.dll");
    HMODULE failedFor = nullptr;
    uint64_t nextReportUs = NowUs() + 2000000;
    for (;;)
    {
        Sleep(250);
        if (!g_probe.presentOriginal)
            continue;
        const HMODULE module = GetModuleHandleW(L"halo3.dll");
        if (g_probe.module && module != g_probe.module)
            Retire(module ? "halo3.dll was reloaded" : "halo3.dll was unloaded");
        if (!g_probe.module && module && module != failedFor)
        {
            if (!InstallHalo3(module))
                failedFor = module;
        }
        if (!module)
            failedFor = nullptr;
        const uint64_t nowUs = NowUs();
        if (nowUs >= nextReportUs)
        {
            nextReportUs = nowUs + 2000000;
            Report();
        }
    }
}
} // namespace

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(module);
        // Pin: hooks point into this module for the life of the process.
        HMODULE pinned = nullptr;
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                           reinterpret_cast<LPCWSTR>(&DllMain), &pinned);
        if (HANDLE thread = CreateThread(nullptr, 0, &Worker, nullptr, 0, nullptr))
            CloseHandle(thread);
    }
    return TRUE;
}
