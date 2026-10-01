// Included in game.cpp at file scope, right after legacy_direct_drive.inl, in DEV BUILDS ONLY (the #include
// sits inside #if HALOMCCVR_DEV_DIAGNOSTICS: a player build contains none of this).
//
// THE TYPE-13 PILOT CONTROLLERS, LOG-ONLY (docs/TYPE13-FLIGHT-CONTROLLERS-2026-10-01.md, section 4 Phase B).
// Reach's Sabre/Seraph and Halo 4's Broadsword are physics type 13. Each title's per-user control dispatcher
// (Reach 0x6350C, Halo 4 0xA8828) calls a dedicated attitude controller for a type-13 pilot -- Reach
// haloreach+0x64828, Halo 4 halo4+0xAA524 -- and jumps past the desired-angles accumulator the direct-drive
// hook sits on, so the pilot is never driven and stays on the stick servo. Before anything is written
// there, the running game has to answer what static analysis could not: whether the record the controller
// writes is what the hull/physics steers by (the LINK), how fast the stick can turn the craft (the RATE, for
// a slew cap), the forced-steer/redirect windows, the roll mode, the time arguments, the tag limits, Halo
// 4's biped physics mode and runtime type override, and which dispatcher callers run in a tick.
//
// Nothing is hooked unless type13_log = 1 (read at startup): a 0 run carries no detour, so it is a true control.
// The detours write NOTHING to the game. They run the native unchanged (through CallNativeWithSpareSlots:
// seven arguments, three of them on the stack; tools/audit_caller_area.py 64828:11:v and aa524:11:v clean,
// 6 slots a violation at entry+0x38), read under SEH before and after it, and push POD events into a
// lock-free single-producer ring (src/common/type13_flight_log.h); the cold worker formats every line. Only
// one owner thread observes a lane (claimed by compare_exchange, taken over after 500 ms of silence), and a
// busy flag keeps the producer single-threaded even across a takeover. They never stamp g_reachPcLiveMs or
// g_reachLastCamCopyMs, so the Reach heartbeat question (Phase A3) stays measurable.
//
// ADDR-HYGIENE: dev-only, guarded. Every RVA and offset below is a measurement of the retail Steam builds
// (haloreach.dll SizeOfImage 0x4EDA000, halo4.dll 0x4A3F000; the Store modules carry the same code at the
// same RVAs, docs/MCC-EDITIONS-EVIDENCE.md). The install proves each one before a hook exists: every anchor
// in the tables matches at its RVA (the ones that locate the entry, the call site or a shared global are
// also unique in the whole module), the dispatcher's type-13 call (rel32) lands on the entry, the TLS index,
// tag pages and tag instances globals agree across independent sites, and the caller-attribution frame is
// derived from the dispatcher prologue the table pins. Any miss: no hook, one log line naming what failed,
// the pilot stays exactly as today. The anchors were generated from the retail DLLs, and
// tools/verify_type13_anchors.py re-proves every one of them (and the install's other checks) offline, reading
// the tables straight out of this file: run it after any edit here and on any new game build.

#include "../common/type13_flight_log.h"

namespace
{
constexpr size_t kT13RingSize = 256;
constexpr uint64_t kT13TakeoverMs = 500;

struct T13Anchor
{
    const char* name;
    uint32_t rva;
    bool unique;           // also unique in the whole module (it locates code or supplies an address)
    const char* pattern;   // the rest match exactly at their RVA (pure field offsets)
};

// One title's observer. Install/remove fields belong to the worker; the producer, the ring's producer
// side and the claim to the sim thread that owns the lane; the counters are read by the worker.
struct T13Lane
{
    explicit T13Lane(uint8_t laneTitle) : title(laneTitle), producer(laneTitle) {}
    const uint8_t title;                     // 0 Reach, 1 Halo 4 (the events' title)
    // Install (worker). Read by the hook only while enabled (released after they are set).
    void* target = nullptr;
    void* original = nullptr;                // Halo 4's trampoline (Reach's is g_reachT13Orig)
    uintptr_t base = 0;
    std::atomic<uint32_t> generation{0};
    uintptr_t ctrlRet = 0;                   // the controller's return address in the dispatcher
    uint32_t dispSlot = 0;                   // the dispatcher's own return address: entry rsp + slot * 8
    t13::CallerTable<4> callers{};
    uintptr_t pages = 0, instances = 0;      // the tag globals (pages: u64[16]; instances: u64 pointer)
    const uint32_t* tlsIndex = nullptr;
    std::atomic<bool> enabled{false};
    std::atomic<uint32_t> callbacks{0};      // Halo 4's quiescence counter (Reach holds g_reachCamera's)
    // Sim thread.
    t13::OwnerClaim owner;
    std::atomic<bool> busy{false};
    std::atomic<bool> resetPending{false};      // a new owner's reset, applied once it holds the busy flag
    std::atomic<uint32_t> previousOwner{0};
    t13::Producer<kT13RingSize> producer;
    t13::SpscRing<t13::Event, kT13RingSize> ring;
    // Counters (sim thread -> worker, every 2 s).
    std::atomic<uint32_t> calls{0}, ticks{0}, dups{0}, byCaller[5]{}, foreignRet{0}, otherThreads{0},
        otherUsers{0}, busyHits{0}, faults{0}, badTime{0};
    std::atomic<uint32_t> lastVehicle{0}, lastParent{0xFFFFFFFFu};
    std::atomic<int32_t> lastSeat{-1}, master{-1};
    // The latest facing and craft forward (Halo 4 CAM line): a seqlock, one writer (the owner).
    std::atomic<uint32_t> snapSeq{0};
    std::atomic<float> snapFacingYaw{0.0f}, snapFacingPitch{0.0f}, snapCraftYaw{0.0f}, snapCraftPitch{0.0f};
    std::atomic<uint64_t> snapMs{0};
};
T13Lane g_t13Reach{0}, g_t13Halo4{1};

// One controller call's working state (POD: it lives inside the detours' __try).
struct T13Call
{
    t13::Sample s;
    uintptr_t record = 0;          // Reach: player_control (PC); Halo 4: the controller's record (pc+0x120)
    const uint8_t* tls = nullptr;
    const uint8_t* vehicle = nullptr;
    uintptr_t dispRet = 0;
    uint32_t thread = 0;
};

using T13ControllerFn = void (*)(int, uint32_t, float, float, const void*, float, float);

bool T13Executable(uintptr_t address)
{
    MEMORY_BASIC_INFORMATION info{};
    if (!address || !VirtualQuery(reinterpret_cast<const void*>(address), &info, sizeof(info)) ||
        info.State != MEM_COMMIT || (info.Protect & PAGE_GUARD))
        return false;
    const DWORD protection = info.Protect & 0xFFu;
    return protection == PAGE_EXECUTE || protection == PAGE_EXECUTE_READ || protection == PAGE_EXECUTE_READWRITE ||
           protection == PAGE_EXECUTE_WRITECOPY;
}

bool T13AnchorMatches(uintptr_t base, size_t size, const T13Anchor& a)
{
    if (!T13Executable(base + a.rva))
        return false;
    return a.unique ? LegacyDriveUniqueAt(base, size, a.rva, a.pattern) : LegacyDriveBytesAt(base, size, a.rva, a.pattern);
}

// Checks every anchor; the names of the ones that failed go into `failed` (comma-separated).
template <size_t N>
bool T13AnchorsMatch(uintptr_t base, size_t size, const T13Anchor (&anchors)[N], char* failed, size_t failedSize)
{
    bool all = true;
    size_t used = 0;
    failed[0] = 0;
    for (const T13Anchor& a : anchors)
    {
        if (T13AnchorMatches(base, size, a))
            continue;
        all = false;
        const int n = snprintf(failed + used, failedSize - used, "%s%s", used ? "," : "", a.name);
        if (n > 0 && used + static_cast<size_t>(n) < failedSize)
            used += static_cast<size_t>(n);
    }
    return all;
}

// The target of a rip-relative operand: the instruction at insnRva, its disp32 at +dispOffset, `length` bytes
// long. 0 unless it lands inside the module. Read only after the anchor covering the instruction matched.
uintptr_t T13Rip(uintptr_t base, size_t size, uint32_t insnRva, uint32_t dispOffset, uint32_t length)
{
    if (static_cast<size_t>(insnRva) + length > size)
        return 0;
    int32_t disp = 0;
    memcpy(&disp, reinterpret_cast<const void*>(base + insnRva + dispOffset), sizeof(disp));
    const uintptr_t target = base + insnRva + length + static_cast<intptr_t>(disp);
    return target >= base && target < base + size ? target : 0;
}

// The target of an `E8 rel32` at callRva; 0 when it is not one or lands outside the module.
uintptr_t T13CallTarget(uintptr_t base, size_t size, uint32_t callRva)
{
    if (static_cast<size_t>(callRva) + 5 > size)
        return 0;
    const auto* p = reinterpret_cast<const uint8_t*>(base + callRva);
    if (p[0] != 0xE8)
        return 0;
    int32_t rel = 0;
    memcpy(&rel, p + 1, sizeof(rel));
    const uintptr_t target = base + callRva + 5 + static_cast<intptr_t>(rel);
    return target >= base && target < base + size ? target : 0;
}

// The caller-attribution slot: the call site's own function (.pdata) must begin at the dispatcher whose
// prologue the anchors pinned, and the slot is derived from that prologue (t13::DispatcherReturnSlot).
bool T13DispatcherSlot(uintptr_t base, uint32_t dispatcherRva, size_t prologueBytes, uint32_t callRva, uint32_t& slot)
{
    slot = 0;
    DWORD64 imageBase = 0;
    const PRUNTIME_FUNCTION function = RtlLookupFunctionEntry(base + callRva, &imageBase, nullptr);
    if (!function || imageBase != base || function->BeginAddress != dispatcherRva)
        return false;
    return t13::DispatcherReturnSlot(reinterpret_cast<const uint8_t*>(base + dispatcherRva), prologueBytes, slot);
}

// Dispatcher callers -> CallerTable (labels only, never a gate): each must be an E8 to the dispatcher.
template <size_t N>
void T13Callers(uintptr_t base, size_t size, uint32_t dispatcherRva, const uint32_t (&callRvas)[N], T13Lane& lane,
                bool (&ok)[N])
{
    static_assert(N <= 4);
    for (size_t i = 0; i < 4; ++i)
        lane.callers.ret[i] = 0;
    for (size_t i = 0; i < N; ++i)
    {
        ok[i] = T13CallTarget(base, size, callRvas[i]) == base + dispatcherRva;
        lane.callers.ret[i] = ok[i] ? base + callRvas[i] + 5 : 0;
    }
}

void T13Push(T13Lane& lane, const t13::Event& e) { lane.ring.Push(e); }

// The common front of both detours, after the title's own admission: user, return address, owner, busy.
// True = this call is observed, and the busy flag is held until T13Release.
bool T13Claim(T13Lane& lane, int user, const uintptr_t* entry, T13Call& call)
{
    lane.calls.fetch_add(1, std::memory_order_relaxed);
    if (user != 0)
    {
        lane.otherUsers.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (entry[0] != lane.ctrlRet)   // our own entry stack: the native's return address
    {
        lane.foreignRet.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    call.s.nowMs = GetTickCount64();
    call.thread = GetCurrentThreadId();
    uint32_t previous = 0;
    const t13::Claim claim = lane.owner.Step(call.thread, call.s.nowMs, kT13TakeoverMs, &previous);
    if (claim == t13::Claim::Other)
    {
        lane.otherThreads.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (claim == t13::Claim::NewOwner)
    {
        // Applied below once this thread holds the busy flag: the old owner may still be mid-observation.
        lane.previousOwner.store(previous, std::memory_order_relaxed);
        lane.resetPending.store(true, std::memory_order_release);
    }
    if (lane.busy.exchange(true, std::memory_order_acquire))
    {
        lane.busyHits.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (lane.resetPending.exchange(false, std::memory_order_acq_rel))
    {
        // A new owner starts from nothing: the old owner's stint ends here.
        lane.producer.EndStint(t13::StintEndReason::Owner, lane.ring);
        lane.producer.Reset();
        const uint32_t from = lane.previousOwner.load(std::memory_order_relaxed);
        if (from)
        {
            t13::Event e{};
            e.title = lane.title;
            e.kind = t13::Kind::Owner;
            e.u.owner = {from, call.thread};
            T13Push(lane, e);
        }
    }
    if (lane.producer.SkipFaulted(call.s.vehicle, call.s.nowMs))
    {
        lane.busy.store(false, std::memory_order_release);
        return false;
    }
    return true;
}

void T13Release(T13Lane& lane) { lane.busy.store(false, std::memory_order_release); }

// A read faulted: count it, end the stint (it observes nothing more until another one), release.
void T13Fault(T13Lane& lane, const T13Call& call, uint32_t where)
{
    lane.faults.fetch_add(1, std::memory_order_relaxed);
    lane.producer.MarkFaulted(call.s.vehicle, call.s.nowMs, where, lane.ring);
    T13Release(lane);
}

// After both reads: the producer's tick and stint bookkeeping, the stint's one tag read, the events.
template <class ReadTag>
void T13Observe(T13Lane& lane, T13Call& call, ReadTag readTag)
{
    const t13::CallStep step = lane.producer.Begin(call.s, call.thread, lane.ring);
    (step.newTick ? lane.ticks : lane.dups).fetch_add(1, std::memory_order_relaxed);
    lane.byCaller[lane.callers.Index(call.dispRet)].fetch_add(1, std::memory_order_relaxed);
    if (step.needTag)
    {
        t13::Event e{};
        e.title = lane.title;
        e.kind = t13::Kind::Tag;
        e.user = call.s.user;
        e.vehicle = call.s.vehicle;
        e.tick = call.s.tick;
        t13::TagFacts facts;
        bool faulted = false;
        const bool ok = call.vehicle && readTag(call.vehicle, e.u.tag, facts, faulted);
        if (faulted)
        {
            T13Fault(lane, call, 2);
            return;
        }
        e.u.tag.valid = ok ? 1 : 0;
        T13Push(lane, e);
        lane.producer.SetTag(facts);   // asked once per stint, read or not
    }
    lane.producer.Finish(call.s, step, lane.ring);
    lane.lastVehicle.store(call.s.vehicle, std::memory_order_relaxed);
    lane.lastParent.store(call.s.parent, std::memory_order_relaxed);
    lane.lastSeat.store(call.s.seat, std::memory_order_relaxed);
    if (step.newTick)
    {
        lane.snapSeq.fetch_add(1, std::memory_order_acq_rel);
        lane.snapFacingYaw.store(lane.producer.facingYaw, std::memory_order_relaxed);
        lane.snapFacingPitch.store(lane.producer.facingPitch, std::memory_order_relaxed);
        lane.snapCraftYaw.store(lane.producer.craftYaw, std::memory_order_relaxed);
        lane.snapCraftPitch.store(lane.producer.craftPitch, std::memory_order_relaxed);
        lane.snapMs.store(call.s.nowMs, std::memory_order_relaxed);
        lane.snapSeq.fetch_add(1, std::memory_order_release);
    }
    T13Release(lane);
}

t13::Vec3 T13Vec(const uint8_t* p) { return {reinterpret_cast<const float*>(p)[0], reinterpret_cast<const float*>(p)[1], reinterpret_cast<const float*>(p)[2]}; }
template <class T>
T T13Read(const uint8_t* p, size_t offset)
{
    T value{};
    memcpy(&value, p + offset, sizeof(value));
    return value;
}

// The tag chain both controllers compile: instance = instances[index]; def = pages[enc >> 28] + enc * 4.
const uint8_t* T13TagAddress(const T13Lane& lane, uint32_t enc)
{
    if (!enc)
        return nullptr;
    const auto* page = T13Read<const uint8_t*>(reinterpret_cast<const uint8_t*>(lane.pages), (enc >> 28) * 8u);
    return page ? page + static_cast<uint64_t>(enc) * 4u : nullptr;
}
const uint8_t* T13TagDefinition(const T13Lane& lane, uint16_t index)
{
    const auto* instances = T13Read<const uint8_t*>(reinterpret_cast<const uint8_t*>(lane.instances), 0);
    return instances ? T13TagAddress(lane, T13Read<uint32_t>(instances, static_cast<size_t>(index) * 8u + 4u)) : nullptr;
}
// The type accessors' own walk (Reach 0x4AC1E4, Halo 4 0x62DAD0): the first of 15 twelve-byte blocks with a
// nonzero count; 16 when none.
int32_t T13TagType(const uint8_t* def, size_t blocks)
{
    for (int32_t i = 0; i < 15; ++i)
        if (T13Read<int32_t>(def, blocks + static_cast<size_t>(i) * 12u) != 0)
            return i;
    return 16;
}
} // namespace

#if HALOMCCVR_EXPERIMENTAL_REACH_RENDER_CANDIDATE
// ============================================================================================================
// REACH: haloreach+0x64828, the Sabre/Seraph pilot attitude controller.
// ============================================================================================================
namespace
{
// ADDR-HYGIENE: dev-only, guarded. The whole Reach image is also hash-pinned before InstallReachCameraCore
// reaches this install (the preflight token), so every one of these is pinned twice.
constexpr uint32_t kT13ReachEntryRva = 0x64828;
constexpr uint32_t kT13ReachCallRva = 0x63607;       // in CALL: E8 -> the entry, then jmp past 0x6365E
constexpr uint32_t kT13ReachDispatcherRva = 0x6350C;
constexpr const T13Anchor kT13ReachAnchors[] = {
    // The controller's entry: 151 bytes (the 37-byte prologue alone also matches 0x12B7F0 and 0x15280C).
    // TLS index load at +0x648A8, tag pages lea at +0x648A1.
    {"ENTRY", 0x64828, true,
     "48 8B C4 55 53 56 57 41 54 41 55 41 56 41 57 48 8D A8 18 FF FF FF 48 81 EC A8 01 00 00 0F 29 70 A8 0F 29 78 98 "
     "44 0F 29 40 88 44 0F 29 88 78 FF FF FF 44 0F 29 90 68 FF FF FF 44 0F 29 98 58 FF FF FF 44 0F 29 A0 48 FF FF FF "
     "44 0F 29 A8 38 FF FF FF 44 0F 29 B0 28 FF FF FF 44 0F 29 B8 18 FF FF FF 48 8B 05 ?? ?? ?? ?? 48 33 C4 48 89 45 F0 "
     "65 48 8B 04 25 58 00 00 00 48 8D 35 ?? ?? ?? ?? 44 8B 05 ?? ?? ?? ?? 45 0F 57 C9 41 B9 10 00 00 00 41 BD D8 00 00 00"},
    // The dispatcher: physics type == 13 -> call the controller (+0x63607) -> jmp past the accumulator call.
    {"CALL", 0x635D5, true,
     "8B 08 E8 ?? ?? ?? ?? 83 F8 0D 75 ?? F3 0F 10 45 7F 0F 28 DF F3 0F 10 4D 77 0F 28 D6 8B 55 C7 8B CF "
     "F3 0F 11 44 24 30 F3 0F 11 4C 24 28 4C 89 6C 24 20 E8 ?? ?? ?? ?? EB ??"},
    // The dispatcher's earlier tests: the camera parent is a vehicle (kind 1) and the seat has flag bit 2.
    {"DRIVER", 0x63592, false,
     "E8 ?? ?? ?? ?? 83 7D C7 FF 74 ?? 0F B7 45 C7 B9 10 00 00 00 4A 8B 0C 31 4C 8D 34 40 4C 8B 79 50 43 80 7C F7 04 01 "
     "75 ?? 0F B7 55 CB 41 B8 02 00 00 00 8B 4D C7 E8 ?? ?? ?? ?? 84 C0 74 ??"},
    // The dispatcher's prologue: 7 pushes + sub 0xC0, the caller-attribution frame (DispatcherReturnSlot).
    {"DISP", 0x6350C, false, "48 8B C4 55 53 57 41 54 41 55 41 56 41 57 48 8D 68 B1 48 81 EC C0 00 00 00"},
    // The record: PC = *(TLS+0xD8) + user*0x180 (lea r12,[rax+rax*2]; shl r12,7); the controller's rbx = PC+0x80.
    {"REC", 0x648B3, false,
     "41 B9 10 00 00 00 41 BD D8 00 00 00 F3 0F 11 54 24 48 4E 8B 04 C0 48 63 C1 0F B7 CA F3 0F 11 5C 24 50 4F 8B 2C 28 "
     "4C 8D 24 40 4C 89 45 A8 4B 8B 04 01 48 8D 14 49 49 C1 E4 07 48 89 54 24 68 49 8D 9D 80 00 00 00 49 03 DC"},
    {"PC88", 0x64900, false, "44 8A 7B 08 48 8B 44 D1 10 48 89 45 A0 41 C0 EF 02"},   // PC+0x88 bit 2: roll mode
    // The tag chain: def index u16 at veh+0; instances (+0x64914); the type-13 block's address at def+0x570.
    {"TAGDEF", 0x64911, true,
     "0F B7 08 48 8B 05 ?? ?? ?? ?? 8B 54 C8 04 B9 01 00 00 00 8B C2 48 C1 E8 1C 48 8B 04 C6 8B BC 90 70 05 00 00 "
     "8B C7 48 C1 E8 1C 48 8B 34 C6"},
    {"ROLLIN", 0x64944, false, "48 8B 85 10 01 00 00 F3 0F 10 78 04"},          // input (entry+0x28) +4: roll axis
    {"HOLD", 0x6496D, false, "F3 0F 10 83 C8 00 00 00"},                        // PC+0x148 look hold
    {"LATCH", 0x649E8, false, "43 88 84 2C 4C 01 00 00"},                       // PC+0x14C latch byte
    {"IDLE", 0x64A34, false, "89 8B C0 00 00 00 F3 0F 10 43 18"},               // PC+0x140 idle; PC+0x98 pitch
    {"RRATE", 0x64AD2, false, "F3 0F 10 64 BE 5C 4C 8D B3 C4 00 00 00"},        // sf+0x5C; PC+0x144 roll rate
    {"SF60", 0x64AF1, false, "F3 0F 59 5C BE 60"},
    {"SF64", 0x64B0C, false, "F3 0F 59 5C BE 64"},
    {"SF68", 0x64C2A, false, "F3 0F 59 5C BE 68"},
    {"SF6C", 0x64A7B, false, "F3 0F 5D 44 BE 6C"},
    {"SF70", 0x64A51, false, "0F 2F 4C BE 70"},
    {"SF74", 0x64B89, false, "F3 0F 5C 4C BE 74"},
    {"SF78", 0x64BAB, false, "F3 0F 10 44 BE 78"},
    {"SF7C", 0x64A1C, false, "F3 0F 59 54 BE 7C"},
    {"SF80", 0x64B6B, false, "F3 0F 10 84 BE 80 00 00 00"},
    {"SF84", 0x64BB7, false, "F3 0F 10 84 BE 84 00 00 00"},
    {"ANGPTR", 0x64C88, false, "4C 8D 7B 14"},                                  // r15 = PC+0x94, the Euler output
    {"ANG", 0x64D19, false,
     "4C 8D 44 24 58 48 8D 55 C0 48 8D 4D B0 E8 ?? ?? ?? ?? 41 0F 28 D1 48 8D 4C 24 58 49 8B D7 E8 ?? ?? ?? ??"},
    // The forced steer's stamp: direction veh+0xAC0/+0xAC8, tick veh+0xABC.
    {"STAMP", 0x64D6A, false, "F2 41 0F 10 85 C0 0A 00 00 41 8B 85 C8 0A 00 00 45 8B B5 BC 0A 00 00"},
    {"SFA8", 0x64D9C, false, "84 94 BE A8 00 00 00"},                           // sf+0xA8 bit 0
    {"AIM", 0x64DA9, false, "F3 41 0F 10 9D 14 02 00 00"},                      // veh+0x214 aiming vector
    // The window: sf+0xAC; game time TLS+0xA0 -> tick +0xC, seconds per tick +8 (comiss/jbe).
    {"WIN", 0x64E58, false,
     "F3 0F 10 84 BE AC 00 00 00 B9 A0 00 00 00 4A 8B 0C 21 8B 41 0C 41 2B C6 66 0F 6E C8 0F 5B C9 F3 0F 59 49 08 "
     "0F 2F C1 0F 86 ?? ?? ?? ??"},
    {"WRAPP", 0x651E4, false, "F3 0F 11 43 18"},                                // PC+0x98 pitch wrap
    {"WRAPR", 0x65200, false, "F3 0F 11 43 1C"},                                // PC+0x9C roll wrap
    {"EXIT", 0x65205, false,
     "F3 41 0F 10 07 41 0F 28 CE F3 41 0F 58 C6 E8 ?? ?? ?? ?? F3 41 0F 11 07 48 8B 4D F0 48 33 CC"},   // yaw wrap
    // The accumulator 0x6375C adds the stick into the same desired yaw PC+0x94 (direct drive's own anchor).
    {"ACC", 0x63AF9, false,
     "0F 28 D0 F3 0F 58 94 1F 94 00 00 00 41 83 CB FF B8 10 00 00 00 F3 0F 11 94 1F 94 00 00 00"},
    // The vehicle-type accessor: 15 twelve-byte type blocks from def+0x4D0 (lea r8,[rdx+0x134]).
    {"TYPEACC", 0x4AC1E4, false,
     "48 8B 05 ?? ?? ?? ?? 0F B7 C9 8B 54 C8 04 48 8D 0D ?? ?? ?? ?? 8B C2 48 C1 E8 1C 4C 8D 82 34 01 00 00 48 8B 0C C1 "
     "33 C0 45 33 C9 44 8D 50 10 4E 8D 04 81 41 83 38 00 75 11 FF C0 49 FF C1 49 83 C0 0C 49 83 F9 0F 7C EB EB 03 "
     "44 8B D0 41 8B C2 C3"},
    // The hull 0x51DE3C (the type-13 case of 0x4AD958): pages (+0x51DEB5) and the TLS index (+0x51DEC2) ...
    {"HPAGES", 0x51DEAC, true, "65 48 8B 04 25 58 00 00 00 48 8D 3D ?? ?? ?? ?? 4C 8B F1 44 8B E2 8B 0D ?? ?? ?? ??"},
    {"HDEF", 0x51DEF3, true, "48 8B 05 ?? ?? ?? ?? 0F B7 0E 0F B7 96 D8 09 00 00 44 8B 6C C8 04"},   // ... instances
    {"HSF", 0x51DF22, false, "42 8B 9C A8 70 05 00 00"},                        // ... the same block, def+0x570
    {"HFWD", 0x51DF73, false,                                                     // veh+0x50/0x54/0x58 (atan2)
     "F3 0F 10 4E 54 F3 0F 10 46 50 F3 0F 59 C0 F3 0F 59 C9 F3 0F 58 C1 E8 ?? ?? ?? ?? 0F 28 C8 F3 0F 10 46 58"},
    // The desired facing the hull steers by, veh+0x1E4/0x1E8/0x1EC, and its roll base veh+0x250.
    {"HX", 0x51E551, false,
     "F3 0F 10 B6 E8 01 00 00 F3 0F 10 BE E4 01 00 00 0F 28 C6 0F 28 CF E8 ?? ?? ?? ?? F3 0F 59 FF F3 0F 11 45 C0 "
     "F3 0F 59 F6 F3 0F 58 FE 0F 28 C7 E8 ?? ?? ?? ?? 0F 28 C8 F3 0F 10 86 EC 01 00 00 E8 ?? ?? ?? ?? F3 0F 10 B6 50 02 00 00"},
    {"H10", 0x51E526, false, "F3 44 0F 10 54 9F 10"},                           // hull sf+0x10
    {"H14", 0x51E04D, false, "F3 44 0F 59 54 9F 14"},                           // hull sf+0x14
    {"H38", 0x51E7CE, false, "F3 0F 10 64 9F 38 F3 0F 59 A6 A0 03 00 00"},      // hull sf+0x38 x veh+0x3A0
    {"H88", 0x51E5F7, false, "F3 0F 59 84 9F 88 00 00 00"},
    {"H8C", 0x51E5DC, false, "F3 0F 10 9C 9F 8C 00 00 00"},
    {"H90", 0x51E62A, false, "F3 0F 10 84 9F 90 00 00 00"},
    // Hull steering is skipped while veh + i16[veh+0x1AE] holds 0x1B.
    {"STATE", 0x51FDE0, false, "48 0F BF 81 AE 01 00 00 48 03 C1 74 07 83 38 1B 41 0F 94 C1"},
};
// Dispatcher callers (labels): the action/kit consumer 0x5DF80, then 0x61A38's two calls ([I] the second is
// the no-unit path, the analogue of Halo 4's 0xA7A01).
constexpr uint32_t kT13ReachCallerRvas[3] = {0x5EE43, 0x62467, 0x628A3};
constexpr const char* kT13ReachCallerNames[4] = {"alt+5EE48", "main+6246C", "nounit?+628A8", "-"};

bool T13ReachAdmitted()
{
    const auto& lane = g_t13Reach;
    return g_reachT13Enabled.load(std::memory_order_acquire) && g_config.type13_log &&
           !g_reachCamera.teardownRequested.load(std::memory_order_acquire) &&
           g_reachCamera.generation.load(std::memory_order_acquire) == lane.generation.load(std::memory_order_acquire);
}

// Before the native: the record the controller is about to update, its flags, the roll axis, the tick.
bool T13ReachReadPre(int user, const void* input, const uintptr_t* entry, T13Call& call, bool& faulted)
{
    faulted = false;
    __try
    {
        const auto& lane = g_t13Reach;
        auto& s = call.s;
        call.dispRet = lane.dispSlot ? entry[lane.dispSlot] : 0;
        auto** slots = reinterpret_cast<const uint8_t**>(__readgsqword(0x58));
        const uint32_t index = *lane.tlsIndex;
        const uint8_t* tls = (slots && index < 0x200) ? slots[index] : nullptr;
        if (!tls)
            return false;
        const uintptr_t globals = T13Read<uintptr_t>(tls, 0xD8);   // REC
        if (!globals)
            return false;
        call.tls = tls;
        call.record = globals + static_cast<uintptr_t>(user) * 0x180;
        const auto* pc = reinterpret_cast<const uint8_t*>(call.record);
        for (int i = 0; i < 3; ++i)
            s.pre[i] = T13Read<float>(pc, 0x94 + 4u * i);          // ANGPTR / EXIT / WRAPP / WRAPR
        s.rollMode = ((T13Read<uint8_t>(pc, 0x88) >> 2) & 1u) != 0; // PC88
        s.moveX = input ? T13Read<float>(static_cast<const uint8_t*>(input), 4) : 0.0f;   // ROLLIN
        const auto* time = T13Read<const uint8_t*>(tls, 0xA0);      // WIN
        if (!time)
            return false;
        s.tick = T13Read<int32_t>(time, 0xC);
        s.spt = T13Read<float>(time, 8);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        faulted = true;
        return false;
    }
}

// After the native: what it wrote, its timers, and the vehicle fields the hull, the camera and the forced
// steer read.
bool T13ReachReadPost(T13Call& call, bool& faulted)
{
    faulted = false;
    __try
    {
        auto& s = call.s;
        const auto* pc = reinterpret_cast<const uint8_t*>(call.record);
        for (int i = 0; i < 3; ++i)
            s.post[i] = T13Read<float>(pc, 0x94 + 4u * i);
        s.idle = T13Read<float>(pc, 0x140);       // IDLE
        s.rollRate = T13Read<float>(pc, 0x144);   // RRATE
        s.hold = T13Read<float>(pc, 0x148);       // HOLD
        s.latch = T13Read<uint8_t>(pc, 0x14C);    // LATCH
        uint8_t kind = 0;
        const unsigned char* veh = ReachVehicleObjectData(static_cast<int32_t>(s.vehicle), kind);
        if (veh && kind == kReachObjectKindVehicle)
        {
            call.vehicle = veh;
            s.haveVehicle = true;
            s.facing = T13Vec(veh + 0x1E4);              // HX
            s.rollField = T13Read<float>(veh, 0x250);   // HX
            s.craftFwd = T13Vec(veh + 0x50);             // HFWD ([I] the nose)
            s.aim = T13Vec(veh + 0x214);                 // AIM
            s.stamp = T13Read<int32_t>(veh, 0xABC);     // STAMP
            s.stampDir = T13Vec(veh + 0xAC0);            // STAMP
            const int16_t off = T13Read<int16_t>(veh, 0x1AE);   // STATE
            s.stateOff = off;
            s.state = (off > 0 && off < 0x2000) ? T13Read<int32_t>(veh, static_cast<size_t>(off)) : t13::kUnread;
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        faulted = true;
        return false;
    }
}

// The stint's tag: the type-13 block (sf) the controller and the hull read, and the vehicle's hull scale.
bool T13ReachReadTag(const uint8_t* veh, t13::TagData& d, t13::TagFacts& f, bool& faulted)
{
    faulted = false;
    __try
    {
        const auto& lane = g_t13Reach;
        d.def = T13Read<uint16_t>(veh, 0);                 // TAGDEF
        const uint8_t* def = T13TagDefinition(lane, d.def);
        if (!def)
            return false;
        d.tagType = T13TagType(def, 0x4D0);                // TYPEACC
        d.n13 = T13Read<int32_t>(def, 0x4D0 + 13 * 12);    // the type-13 block's count, def+0x56C
        const uint8_t* sf = d.n13 >= 1 ? T13TagAddress(lane, T13Read<uint32_t>(def, 0x570)) : nullptr;   // SF
        if (!sf)
            return false;
        static constexpr uint32_t kControl[11]{0x5C, 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78, 0x7C, 0x80, 0x84};
        for (int i = 0; i < 11; ++i)
            d.ctl[i] = T13Read<float>(sf, kControl[i]);
        d.projWord = T13Read<uint32_t>(sf, 0xA8);         // SFA8
        d.window = T13Read<float>(sf, 0xAC);              // WIN
        static constexpr uint32_t kHull[6]{0x10, 0x14, 0x38, 0x88, 0x8C, 0x90};
        for (int i = 0; i < 6; ++i)
            d.phys[i] = T13Read<float>(sf, kHull[i]);
        d.vehScale = T13Read<float>(veh, 0x3A0);          // H38
        f.valid = true;
        f.window = d.window;
        f.projBit = static_cast<int32_t>(d.projWord & 1u);
        f.tagType = d.tagType;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        faulted = true;
        return false;
    }
}

// The pre-call half. True = observe; the busy flag is then held.
bool T13ReachBefore(int user, uint32_t vehicle, float yawD, float pitchD, const void* input, float dtA, float dtB,
                    const uintptr_t* entry, T13Call& call)
{
    auto& lane = g_t13Reach;
    if (!T13ReachAdmitted())
        return false;
    call.s.vehicle = vehicle;
    call.s.user = static_cast<uint16_t>(user);
    if (!T13Claim(lane, user, entry, call))
        return false;
    call.s.yawD = yawD;
    call.s.pitchD = pitchD;
    call.s.dtA = dtA;
    call.s.dtB = dtB;
    bool faulted = false;
    if (!T13ReachReadPre(user, input, entry, call, faulted))
    {
        if (faulted)
            T13Fault(lane, call, 0);
        else
            T13Release(lane);
        return false;
    }
    if (!(call.s.spt > 0.001f && call.s.spt < 0.2f))
    {
        lane.badTime.fetch_add(1, std::memory_order_relaxed);
        T13Release(lane);
        return false;
    }
    lane.master.store(call.record == g_reachPcExpected.load(std::memory_order_acquire) ? 1 : 0,
                      std::memory_order_relaxed);
    return true;
}

void T13ReachAfter(T13Call& call)
{
    auto& lane = g_t13Reach;
    bool faulted = false;
    if (!T13ReachReadPost(call, faulted))
    {
        if (faulted)
            T13Fault(lane, call, 1);
        else
            T13Release(lane);
        return;
    }
    T13Observe(lane, call, T13ReachReadTag);
}
} // namespace

// MinHook's relay jumps here, so _AddressOfReturnAddress() is the native's own entry rsp. The callback count
// spans the whole call, the native included (as ReachUnitCameraPositionHook): the teardown's quiescence
// proof and ScanForReachDetourIngress cover it. The native call is inside __try/__finally only; each read
// has its own __except.
__declspec(noinline) void ReachType13Detour(int user, uint32_t vehicle, float yawD, float pitchD, const void* input,
                                            float dtA, float dtB)
{
    const auto* entry = static_cast<const uintptr_t*>(_AddressOfReturnAddress());
    volatile bool observing = false;
    g_reachCamera.activeCallbacks.fetch_add(1, std::memory_order_acq_rel);
    __try
    {
        const auto native = reinterpret_cast<T13ControllerFn>(g_reachT13Orig);
        if (!native)
            return;
        T13Call call{};
        observing = T13ReachBefore(user, vehicle, yawD, pitchD, input, dtA, dtB, entry, call);
        CallNativeWithSpareSlots(native, user, vehicle, yawD, pitchD, input, dtA, dtB);
        if (observing)
        {
            observing = false;
            T13ReachAfter(call);   // releases the busy flag
        }
    }
    __finally
    {
        if (observing)
            T13Release(g_t13Reach);
        g_reachCamera.activeCallbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
}

// Worker, inside InstallReachCameraCore (whole-image-hashed module, g_reachCamera.base/generation set).
bool ReachType13LogInstall(uintptr_t base, size_t size, uint32_t generation)
{
    auto& lane = g_t13Reach;
    if (!g_config.type13_log)
    {
        // Read at startup only, so a 0 run never carries the detour: it is the true control arm.
        LOG("REACH SF: type13_log 0: type-13 pilot controller +0x%X not hooked (no detour)", kT13ReachEntryRva);
        return false;
    }
    if (g_reachT13Target)
    {
        if (lane.base == base)
        {
            lane.generation.store(generation, std::memory_order_release);
            return true;
        }
        LOG("REACH SF: the type-13 log hook is still installed on another module; not reinstalled");
        return false;
    }
    const uint64_t started = GetTickCount64();
    char failed[512];
    const bool anchorsOk = T13AnchorsMatch(base, size, kT13ReachAnchors, failed, sizeof(failed));
    const bool callOk = anchorsOk && T13CallTarget(base, size, kT13ReachCallRva) == base + kT13ReachEntryRva;
    // The shared globals, each from independent sites: TLS index (ENTRY, HPAGES, and the build's pinned RVA),
    // tag pages (ENTRY, HPAGES, TYPEACC), tag instances (TAGDEF, HDEF, TYPEACC).
    const uintptr_t tls = anchorsOk ? T13Rip(base, size, 0x648A8, 3, 7) : 0;
    const bool tlsOk = tls && tls == T13Rip(base, size, 0x51DEC2, 2, 6) && tls == base + kReachEngineTlsIndexRva;
    const uintptr_t pages = anchorsOk ? T13Rip(base, size, 0x648A1, 3, 7) : 0;
    const bool pagesOk = pages && pages == T13Rip(base, size, 0x51DEB5, 3, 7) && pages == T13Rip(base, size, 0x4AC1F2, 3, 7);
    const uintptr_t instances = anchorsOk ? T13Rip(base, size, 0x64914, 3, 7) : 0;
    const bool instOk = instances && instances == T13Rip(base, size, 0x51DEF3, 3, 7) &&
                        instances == T13Rip(base, size, 0x4AC1E4, 3, 7);
    if (!anchorsOk || !callOk || !tlsOk || !pagesOk || !instOk)
    {
        LOG("REACH SF: type-13 pilot controller +0x%X NOT hooked (anchors failed: %s; dispatcher call=%d tls=%d "
            "pages=%d instances=%d); the log-only lane is off, the Sabre stays exactly as before",
            kT13ReachEntryRva, failed[0] ? failed : "none", callOk, tlsOk, pagesOk, instOk);
        return false;
    }
    bool callerOk[3]{};
    T13Callers(base, size, kT13ReachDispatcherRva, kT13ReachCallerRvas, lane, callerOk);
    uint32_t slot = 0;
    const bool frameOk = T13DispatcherSlot(base, kT13ReachDispatcherRva, 25, kT13ReachCallRva, slot);
    void* target = reinterpret_cast<void*>(base + kT13ReachEntryRva);
    const MH_STATUS created = MH_CreateHook(target, reinterpret_cast<void*>(&ReachType13Detour), &g_reachT13Orig);
    if (created != MH_OK)
    {
        g_reachT13Orig = nullptr;
        LOG("REACH SF: type-13 log hook FAILED (create=%d); the lane is off", static_cast<int>(created));
        return false;
    }
    // Everything the hook reads is set before it can run; a fresh owner, producer and counters.
    lane.base = base;
    lane.ctrlRet = base + kT13ReachCallRva + 5;
    lane.dispSlot = frameOk ? slot : 0;
    lane.pages = pages;
    lane.instances = instances;
    lane.tlsIndex = reinterpret_cast<const uint32_t*>(tls);
    lane.generation.store(generation, std::memory_order_release);
    lane.owner.Reset();
    lane.busy.store(false, std::memory_order_release);
    lane.producer.Reset();
    if (MH_EnableHook(target) != MH_OK)
    {
        MH_RemoveHook(target);
        g_reachT13Orig = nullptr;
        LOG("REACH SF: type-13 log hook FAILED to enable; removed, the lane is off");
        return false;
    }
    g_reachT13Target = target;
    g_reachT13Enabled.store(true, std::memory_order_release);
    LOG("REACH SF: type-13 pilot controller +0x%X hooked LOG-ONLY in %llu ms (writes nothing; type13_log %d); "
        "dispatcher return at entry rsp+0x%X %s; caller labels alt %d main %d nounit? %d",
        kT13ReachEntryRva, static_cast<unsigned long long>(GetTickCount64() - started), g_config.type13_log,
        lane.dispSlot * 8u, frameOk ? "(derived from the dispatcher prologue)" : "UNAVAILABLE (caller attribution off)",
        callerOk[0], callerOk[1], callerOk[2]);
    return true;
}
#endif // HALOMCCVR_EXPERIMENTAL_REACH_RENDER_CANDIDATE

#if HALOMCCVR_EXPERIMENTAL_HALO4_CAMERA
// ============================================================================================================
// HALO 4: halo4+0xAA524, the Broadsword pilot attitude controller.
// ============================================================================================================
namespace
{
// ADDR-HYGIENE: dev-only, guarded (the anchors below; Halo 4's module is identified by PE timestamp, size and
// the camera anchors, not by a whole-image hash, so every offset read here has its own anchor).
constexpr uint32_t kT13Halo4EntryRva = 0xAA524;
constexpr uint32_t kT13Halo4CallRva = 0xA892E;       // in CALL: E8 -> the entry, then jmp 0xA898D
constexpr uint32_t kT13Halo4DispatcherRva = 0xA8828;
constexpr const T13Anchor kT13Halo4Anchors[] = {
    // The controller's entry: 137 bytes (the short prologue alone also matches 0x7BE124). Pages lea +0xAA59D.
    {"ENTRY", 0xAA524, true,
     "48 8B C4 55 53 56 57 41 54 41 55 41 56 41 57 48 8D A8 38 FF FF FF 48 81 EC 88 01 00 00 0F 29 70 A8 0F 29 78 98 "
     "44 0F 29 40 88 44 0F 29 88 78 FF FF FF 44 0F 29 90 68 FF FF FF 44 0F 29 98 58 FF FF FF 44 0F 29 A0 48 FF FF FF "
     "44 0F 29 A8 38 FF FF FF 44 0F 29 B0 28 FF FF FF 44 0F 29 B8 18 FF FF FF 48 8B 05 ?? ?? ?? ?? 48 33 C4 48 89 45 D8 "
     "65 48 8B 04 25 58 00 00 00 48 8D 35 ?? ?? ?? ?? 44 8B CA 41 BC 01 00 00 00"},
    // The dispatcher: runtime type (0x63EC30) == 13 -> call the controller (+0xA892E) -> jmp 0xA898D.
    {"CALL", 0xA88FA, true,
     "8B 4D C7 E8 ?? ?? ?? ?? 83 F8 0D 75 2E F3 0F 10 45 7F 41 0F 28 D8 F3 0F 10 75 77 0F 28 D7 8B 55 C7 8B CE "
     "F3 0F 11 44 24 30 F3 0F 11 74 24 28 4C 89 64 24 20 E8 ?? ?? ?? ?? EB 58"},
    // The dispatcher's object table (TLS+0x18, kind 1 = vehicle) and seat flag bit 2 tests, and its post-block.
    {"VEH", 0xA88C2, false,
     "83 7D C7 FF 74 6D 0F B7 45 C7 BA 18 00 00 00 48 8D 0C 40 4A 8B 14 3A 48 8B 42 50 80 7C C8 04 01 75 51"},
    {"DRIVER", 0xA88E4, false, "0F B7 55 CB 41 B8 02 00 00 00 8B 4D C7 E8 ?? ?? ?? ?? 84 C0 74 3B"},
    {"JOIN", 0xA898D, false,
     "F2 0F 10 43 18 0F 57 ED F3 0F 10 25 ?? ?? ?? ?? 8B 43 20 F3 44 0F 10 05 ?? ?? ?? ?? F2 0F 11 45 A7"},
    // The dispatcher's prologue: 5 pushes + sub 0xD0 (mov [rax+20h],rbx moves nothing).
    {"DISP", 0xA8828, false, "48 8B C4 48 89 58 20 55 56 41 54 41 56 41 57 48 8D 68 B1 48 81 EC D0 00 00 00"},
    // The TLS index load (must be the one the camera core resolved); player_control at TLS+0xB0.
    {"TLSIDX", 0xAA5AD, true,
     "8B 15 ?? ?? ?? ?? 45 0F 57 C9 41 B8 B0 00 00 00 F3 0F 11 54 24 58 F3 0F 11 5C 24 40 48 8B 14 D0"},
    // The record: *(TLS+0xB0) + 0x120 + user*0x1F0 (the legacy-drive record + 0x120; its angles at +0x18).
    {"RECSETUP", 0xAA5C9, false, "48 8B 14 D0 48 63 C1 48 69 D8 F0 01 00 00 49 8B 04 10 48 05 20 01 00 00"},
    {"PC08", 0xAA5F6, false, "44 8A 7B 08 4D 8B 04 10 48 8D 14 49 41 C0 EF 02"},   // rec+0x08 bit 2: roll mode
    // Instances; def index u16 at veh+8; the type-13 block (T) at def+0x670.
    {"TAGDEF", 0xAA60F, true,
     "48 8B 05 ?? ?? ?? ?? 41 0F B7 4D 08 8B 54 C8 04 8B C2 48 C1 E8 1C 48 8B 04 C6 8B BC 90 70 06 00 00 8B C7 48 C1 "
     "E8 1C 48 8B 34 C6"},
    {"ROLLIN", 0xAA63F, false, "48 8B 85 F0 00 00 00 F3 0F 10 70 04"},          // input (entry+0x28) +4
    {"HOLD", 0xAA668, false, "F3 0F 10 83 20 01 00 00"},                        // rec+0x120 look hold
    {"LATCH", 0xAA6E5, false, "88 83 24 01 00 00"},                             // rec+0x124 latch byte
    {"IDLE", 0xAA731, false, "89 8B 18 01 00 00"},                              // rec+0x118 idle timer
    {"RRATE", 0xAA7D1, false, "F3 0F 10 64 BE 60 4C 8D B3 1C 01 00 00"},        // T+0x60; rec+0x11C roll rate
    {"T64", 0xAA7F1, false, "F3 0F 59 5C BE 64"},
    {"T68", 0xAA80D, false, "F3 0F 59 5C BE 68"},
    {"T6C", 0xAA925, false, "F3 0F 59 5C BE 6C"},
    {"T70", 0xAA77B, false, "F3 0F 5D 44 BE 70"},
    {"T74", 0xAA750, false, "0F 2F 4C BE 74"},
    {"T78", 0xAA88A, false, "F3 0F 5C 4C BE 78"},
    {"T7C", 0xAA8AC, false, "F3 0F 10 44 BE 7C"},
    {"T80", 0xAA717, false, "F3 0F 59 94 BE 80 00 00 00"},
    {"T84", 0xAA86C, false, "F3 0F 10 84 BE 84 00 00 00"},
    {"T88", 0xAA8B8, false, "F3 0F 10 84 BE 88 00 00 00"},
    {"ANGPTR", 0xAA984, false, "4C 8D 7B 18"},                                  // r15 = rec+0x18, the Euler output
    {"COMPOSE", 0xAAA03, false,
     "48 8D 54 24 48 48 8D 4D A8 E8 ?? ?? ?? ?? 4C 8D 44 24 48 48 8D 55 A8 48 8D 4D 98 E8 ?? ?? ?? ?? 41 0F 28 D1 "
     "48 8D 4C 24 48 49 8B D7 E8 ?? ?? ?? ??"},
    // The redirect: direction veh+0x1050/+0x1058, stamp veh+0x104C; T+0xC8 bit 0.
    {"REDIR", 0xAAA3B, false,
     "E8 ?? ?? ?? ?? 83 F8 0D 75 23 F2 41 0F 10 85 50 10 00 00 41 8B 85 58 10 00 00 45 8B B5 4C 10 00 00"},
    {"TC8", 0xAAA72, false, "F6 84 BE C8 00 00 00 01"},
    // The window: T+0xCC; game time TLS+0x90 -> tick +0xC, seconds per tick +8.
    {"WIN", 0xAAB35, false,
     "F3 0F 10 84 BE CC 00 00 00 B8 90 00 00 00 48 8B 0C 08 8B 41 0C 41 2B C6 66 0F 6E C8 0F 5B C9 F3 0F 59 49 08 "
     "0F 2F C1 0F 86 ?? ?? ?? ??"},
    {"YAWWRAP", 0xAAEF3, false,
     "F3 41 0F 10 07 41 0F 28 CD F3 41 0F 58 C5 E8 ?? ?? ?? ?? F3 41 0F 11 07 48 8B 4D D8 48 33 CC"},
    // The redirect's writer 0x630EE8: veh+0x104C = tick, +0x1050/+0x1058 = direction, byte +0x105C = 1.
    {"REDIRW", 0x631014, false,
     "83 F8 0D 75 41 B8 90 00 00 00 48 8B 0C 30 8B 41 0C 41 89 86 4C 10 00 00 48 8B 44 24 30 F2 0F 10 00 "
     "F2 41 0F 11 86 50 10 00 00 8B 40 08 41 89 86 58 10 00 00 41 C6 86 5C 10 00 00 01"},
    // The physics 0x6CACD8 (case 13 of 0x6425BC): pages (+0x6CAD51), the TLS index (+0x6CAD5E), instances
    // (+0x6CAD8A), the same block def+0x670, veh+0x70/0x74 (atan2), the facing it steers by and the roll.
    {"PPAGES", 0x6CAD48, true,
     "65 48 8B 04 25 58 00 00 00 48 8D 3D ?? ?? ?? ?? 44 8B E2 48 8B F1 8B 15 ?? ?? ?? ?? 4D 8B E8 41 0F B7 CC "
     "41 B8 18 00 00 00 48 8B 04 D0 48 89 45 E8 48 8D 14 49 4D 8B 04 00 49 8B 40 50 4C 8B 74 D0 10 48 8B 05 ?? ?? ?? ?? "
     "41 0F B7 4E 08 44 8B 7C C8 04"},
    {"PT", 0x6CADAE, false, "42 8B 9C B8 70 06 00 00"},
    {"PFWD", 0x6CADEF, false, "F3 41 0F 10 4E 74 F3 41 0F 10 46 70"},
    {"PX", 0x6CB43E, false,
     "F3 41 0F 10 B6 C0 04 00 00 F3 41 0F 10 BE BC 04 00 00 0F 28 C6 0F 28 CF E8 ?? ?? ?? ?? F3 0F 59 FF 44 0F 28 C0 "
     "F3 0F 59 F6 F3 0F 58 FE 0F 28 C7 E8 ?? ?? ?? ?? 0F 28 C8 F3 41 0F 10 86 C4 04 00 00 E8 ?? ?? ?? ?? "
     "F3 41 0F 10 B6 28 05 00 00"},
    {"P10", 0x6CB426, false, "F3 44 0F 10 64 9F 10"},                           // physics T+0x10
    {"P14", 0x6CAEDA, false, "F3 44 0F 59 5C 9F 14"},                           // physics T+0x14
    {"P3C", 0x6CB864, false, "F3 0F 10 64 9F 3C F3 41 0F 59 A6 A4 06 00 00"},   // T+0x3C x veh+0x6A4
    {"PA4", 0x6CB4B7, false, "F6 84 9F A4 00 00 00 01"},                        // T+0xA4 bit 0
    {"PA8", 0x6CB52B, false, "F3 0F 59 B4 9F A8 00 00 00"},                     // T+0xA8
    // The biped physics mode *(i32*)(unit + u16[unit+0x47A]) (7/8 rebase the record's angles).
    {"PHYSMODE", 0x80C91, false, "0F B7 81 7A 04 00 00 8B 04 08 83 C0 F9 83 F8 01 0F 87"},
    // The runtime type: veh+0xF58 set -> (i8) veh+0xF59, else the tag accessor 0x62DAD0.
    {"TYPEOVR", 0x63EC48, false, "80 B8 58 0F 00 00 00 74 09 0F BE 98 59 0F 00 00 EB 0A 8B 48 08 E8"},
    {"TAGTYPE", 0x62DAD0, true,
     "48 8B 05 ?? ?? ?? ?? 0F B7 C9 8B 54 C8 04 48 8D 0D ?? ?? ?? ?? 8B C2 48 C1 E8 1C 4C 8D 82 74 01 00 00 48 8B 0C C1 "
     "33 C0 45 33 C9 44 8D 50 10 4E 8D 04 81 41 83 38 00 75 11 FF C0 49 FF C1 49 83 C0 0C 49 83 F9 0F 7C EB EB 03 "
     "44 8B D0 41 8B C2 C3"},
    // The driver -> vehicle facing copy 0x5EE590: skipped while veh+0x16C bit 2; reads the driver unit's
    // +0x4C4/+0x4BC and writes the vehicle's.
    {"GATE16C", 0x5EEAC1, false, "83 F8 FF 0F 84 ?? ?? ?? ?? 8B 8E 6C 01 00 00 C1 E9 02 80 E1 01 0F 85"},
    {"UNIT4BC", 0x5EEB76, false, "45 8B 84 24 C4 04 00 00 F2 45 0F 10 94 24 BC 04 00 00"},
    {"COPY4BC", 0x5EEF09, false, "F2 44 0F 11 96 BC 04 00 00 44 89 86 C4 04 00 00 41 8B 84 24 28 05 00 00"},
    // The record's unit at player_control +0x120 (= the controller record +0; direct drive's own anchor).
    {"RECUNIT", 0xA8C15, false, "8B 8C 1F 20 01 00 00"},
};
// Dispatcher callers (labels): the action/kit consumer 0xA2670; 0xA6BA8's aim-SWAY call (0xA5B90 computes
// sinf of tag-driven phases; not recoil: verifier 2026-10-01), its main stick update and its no-unit path.
constexpr uint32_t kT13Halo4CallerRvas[4] = {0xA349E, 0xA7751, 0xA7774, 0xA7A01};
constexpr const char* kT13Halo4CallerNames[4] = {"alt+A34A3", "sway+A7756", "main+A7779", "nounit+A7A06"};

bool T13Halo4Admitted()
{
    const auto& lane = g_t13Halo4;
    return lane.enabled.load(std::memory_order_acquire) && g_config.type13_log &&
           !g_halo4Camera.teardownRequested.load(std::memory_order_acquire) &&
           g_halo4Camera.generation.load(std::memory_order_acquire) == lane.generation.load(std::memory_order_acquire) &&
           TitleAdapter_GetActiveTitle() == GameTitle::Halo4;
}

bool T13Halo4ReadPre(int user, const void* input, const uintptr_t* entry, T13Call& call, bool& faulted)
{
    faulted = false;
    __try
    {
        const auto& lane = g_t13Halo4;
        auto& s = call.s;
        call.dispRet = lane.dispSlot ? entry[lane.dispSlot] : 0;
        auto** slots = reinterpret_cast<const uint8_t**>(__readgsqword(0x58));
        const uint32_t index = *lane.tlsIndex;
        const uint8_t* tls = (slots && index < 0x200) ? slots[index] : nullptr;
        if (!tls)
            return false;
        const uintptr_t globals = T13Read<uintptr_t>(tls, 0xB0);   // TLSIDX / RECSETUP
        if (!globals)
            return false;
        call.tls = tls;
        call.record = globals + 0x120 + static_cast<uintptr_t>(user) * 0x1F0;
        const auto* rec = reinterpret_cast<const uint8_t*>(call.record);
        for (int i = 0; i < 3; ++i)
            s.pre[i] = T13Read<float>(rec, 0x18 + 4u * i);          // ANGPTR / YAWWRAP
        s.rollMode = ((T13Read<uint8_t>(rec, 0x08) >> 2) & 1u) != 0; // PC08
        s.moveX = input ? T13Read<float>(static_cast<const uint8_t*>(input), 4) : 0.0f;   // ROLLIN
        const auto* time = T13Read<const uint8_t*>(tls, 0x90);       // WIN
        if (!time)
            return false;
        s.tick = T13Read<int32_t>(time, 0xC);
        s.spt = T13Read<float>(time, 8);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        faulted = true;
        return false;
    }
}

bool T13Halo4ReadPost(T13Call& call, bool& faulted)
{
    faulted = false;
    __try
    {
        auto& s = call.s;
        const auto* rec = reinterpret_cast<const uint8_t*>(call.record);
        for (int i = 0; i < 3; ++i)
            s.post[i] = T13Read<float>(rec, 0x18 + 4u * i);
        s.idle = T13Read<float>(rec, 0x118);       // IDLE
        s.rollRate = T13Read<float>(rec, 0x11C);   // RRATE
        s.hold = T13Read<float>(rec, 0x120);       // HOLD
        s.latch = T13Read<uint8_t>(rec, 0x124);    // LATCH
        const auto* table = T13Read<const uint8_t*>(call.tls, 0x18);   // VEH
        const uint8_t* veh = Halo4VehicleObject(table, s.vehicle, 0x2);
        if (veh)
        {
            call.vehicle = veh;
            s.haveVehicle = true;
            s.facing = T13Vec(veh + 0x4BC);                 // PX
            s.rollField = T13Read<float>(veh, 0x528);      // PX
            s.craftFwd = T13Vec(veh + 0x70);                // PFWD ([I] the nose)
            s.stamp = T13Read<int32_t>(veh, 0x104C);       // REDIR / REDIRW
            s.stampDir = T13Vec(veh + 0x1050);              // REDIR
            s.redirectFlag = T13Read<uint8_t>(veh, 0x105C);   // REDIRW
            s.gateBit = static_cast<int32_t>((T13Read<uint32_t>(veh, 0x16C) >> 2) & 1u);   // GATE16C
            s.overrideSet = T13Read<uint8_t>(veh, 0xF58);  // TYPEOVR
            s.overrideType = T13Read<int8_t>(veh, 0xF59);  // TYPEOVR
        }
        s.unit = T13Read<uint32_t>(rec, 0);                 // RECUNIT
        const uint8_t* unit = Halo4VehicleObject(table, s.unit, 0x1);
        if (unit)
        {
            s.parent = T13Read<uint32_t>(unit, 0x24);       // Halo4ReadVehicleInputMemory's own reads
            s.seat = T13Read<int16_t>(unit, 0x2C);
            s.haveUnitFacing = true;
            s.unitFacing = T13Vec(unit + 0x4BC);            // UNIT4BC
            const uint16_t off = T13Read<uint16_t>(unit, 0x47A);   // PHYSMODE
            s.physOff = off;
            s.physMode = (off >= 0x10 && off < 0x2000) ? T13Read<int32_t>(unit, off) : t13::kUnread;
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        faulted = true;
        return false;
    }
}

bool T13Halo4ReadTag(const uint8_t* veh, t13::TagData& d, t13::TagFacts& f, bool& faulted)
{
    faulted = false;
    __try
    {
        const auto& lane = g_t13Halo4;
        d.def = T13Read<uint16_t>(veh, 8);                 // TAGDEF
        const uint8_t* def = T13TagDefinition(lane, d.def);
        if (!def)
            return false;
        d.tagType = T13TagType(def, 0x5D0);                // TAGTYPE (lea r8,[rdx+0x174])
        d.n13 = T13Read<int32_t>(def, 0x5D0 + 13 * 12);    // the type-13 block's count, def+0x66C
        const uint8_t* t = d.n13 >= 1 ? T13TagAddress(lane, T13Read<uint32_t>(def, 0x670)) : nullptr;   // T
        if (!t)
            return false;
        static constexpr uint32_t kControl[11]{0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78, 0x7C, 0x80, 0x84, 0x88};
        for (int i = 0; i < 11; ++i)
            d.ctl[i] = T13Read<float>(t, kControl[i]);
        d.projWord = T13Read<uint32_t>(t, 0xC8);          // TC8
        d.window = T13Read<float>(t, 0xCC);               // WIN
        static constexpr uint32_t kPhysics[4]{0x10, 0x14, 0x3C, 0xA8};
        for (int i = 0; i < 4; ++i)
            d.phys[i] = T13Read<float>(t, kPhysics[i]);
        d.phys[4] = d.phys[5] = std::numeric_limits<float>::quiet_NaN();
        d.physWord = T13Read<uint32_t>(t, 0xA4);          // PA4
        d.vehScale = T13Read<float>(veh, 0x6A4);          // P3C
        f.valid = true;
        f.window = d.window;
        f.projBit = static_cast<int32_t>(d.projWord & 1u);
        f.tagType = d.tagType;
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        faulted = true;
        return false;
    }
}

bool T13Halo4Before(int user, uint32_t vehicle, float yawD, float pitchD, const void* input, float dtA, float dtB,
                    const uintptr_t* entry, T13Call& call)
{
    auto& lane = g_t13Halo4;
    if (!T13Halo4Admitted())
        return false;
    call.s.vehicle = vehicle;
    call.s.user = static_cast<uint16_t>(user);
    if (!T13Claim(lane, user, entry, call))
        return false;
    call.s.yawD = yawD;
    call.s.pitchD = pitchD;
    call.s.dtA = dtA;
    call.s.dtB = dtB;
    bool faulted = false;
    if (!T13Halo4ReadPre(user, input, entry, call, faulted))
    {
        if (faulted)
            T13Fault(lane, call, 0);
        else
            T13Release(lane);
        return false;
    }
    if (!(call.s.spt > 0.001f && call.s.spt < 0.2f))
    {
        lane.badTime.fetch_add(1, std::memory_order_relaxed);
        T13Release(lane);
        return false;
    }
    // The legacy-drive record is this record - 0x120 (its angles at +0x138 are this record's +0x18).
    lane.master.store(call.record - 0x120 == g_legacyDrive[2].expected.load(std::memory_order_acquire) ? 1 : 0,
                      std::memory_order_relaxed);
    return true;
}

void T13Halo4After(T13Call& call)
{
    auto& lane = g_t13Halo4;
    bool faulted = false;
    if (!T13Halo4ReadPost(call, faulted))
    {
        if (faulted)
            T13Fault(lane, call, 1);
        else
            T13Release(lane);
        return;
    }
    T13Observe(lane, call, T13Halo4ReadTag);
}

// Same shape as ReachType13Detour, on the lane's own quiescence counter (as legacy drive's hook).
__declspec(noinline) void Halo4Type13Detour(int user, uint32_t vehicle, float yawD, float pitchD, const void* input,
                                            float dtA, float dtB)
{
    const auto* entry = static_cast<const uintptr_t*>(_AddressOfReturnAddress());
    auto& lane = g_t13Halo4;
    volatile bool observing = false;
    lane.callbacks.fetch_add(1, std::memory_order_acq_rel);
    __try
    {
        const auto native = reinterpret_cast<T13ControllerFn>(lane.original);
        if (!native)
            return;
        T13Call call{};
        observing = T13Halo4Before(user, vehicle, yawD, pitchD, input, dtA, dtB, entry, call);
        CallNativeWithSpareSlots(native, user, vehicle, yawD, pitchD, input, dtA, dtB);
        if (observing)
        {
            observing = false;
            T13Halo4After(call);
        }
    }
    __finally
    {
        if (observing)
            T13Release(lane);
        lane.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
}
} // namespace

// Worker, inside InstallHalo4CameraCore after InstallHalo4VehicleInput (the module size, cold observation and
// g_halo4EngineTlsIndex are established).
bool Halo4Type13LogInstall(uintptr_t base, size_t size, uint32_t generation)
{
    auto& lane = g_t13Halo4;
    if (!g_config.type13_log)
    {
        // As Reach's: a 0 run never carries the detour (Halo4Type13LogRemove has nothing to remove).
        LOG("H4 T13: type13_log 0: type-13 pilot controller +0x%X not hooked (no detour)", kT13Halo4EntryRva);
        return false;
    }
    if (lane.target)
    {
        if (lane.base == base)
        {
            lane.generation.store(generation, std::memory_order_release);
            return true;
        }
        LOG("H4 T13: the type-13 log hook is still installed on another module; not reinstalled");
        return false;
    }
    if (!g_halo4EngineTlsIndex)
    {
        LOG("H4 T13: type-13 pilot controller +0x%X NOT hooked (the camera core's TLS index is unresolved); the "
            "log-only lane is off", kT13Halo4EntryRva);
        return false;
    }
    const uint64_t started = GetTickCount64();
    char failed[512];
    const bool anchorsOk = T13AnchorsMatch(base, size, kT13Halo4Anchors, failed, sizeof(failed));
    const bool callOk = anchorsOk && T13CallTarget(base, size, kT13Halo4CallRva) == base + kT13Halo4EntryRva;
    // TLS index (TLSIDX, PPAGES, the camera core's resolution, the pinned RVA), pages (ENTRY, TAGTYPE, PPAGES),
    // instances (TAGDEF, TAGTYPE, PPAGES).
    const uintptr_t tls = anchorsOk ? T13Rip(base, size, 0xAA5AD, 2, 6) : 0;
    const bool tlsOk = tls && tls == T13Rip(base, size, 0x6CAD5E, 2, 6) && tls == base + kHalo4EngineTlsIndexRva &&
                       tls == reinterpret_cast<uintptr_t>(g_halo4EngineTlsIndex);
    const uintptr_t pages = anchorsOk ? T13Rip(base, size, 0xAA59D, 3, 7) : 0;
    const bool pagesOk = pages && pages == T13Rip(base, size, 0x62DADE, 3, 7) && pages == T13Rip(base, size, 0x6CAD51, 3, 7);
    const uintptr_t instances = anchorsOk ? T13Rip(base, size, 0xAA60F, 3, 7) : 0;
    const bool instOk = instances && instances == T13Rip(base, size, 0x62DAD0, 3, 7) &&
                        instances == T13Rip(base, size, 0x6CAD8A, 3, 7);
    if (!anchorsOk || !callOk || !tlsOk || !pagesOk || !instOk)
    {
        LOG("H4 T13: type-13 pilot controller +0x%X NOT hooked (anchors failed: %s; dispatcher call=%d tls=%d "
            "pages=%d instances=%d); the log-only lane is off, the Broadsword stays exactly as before",
            kT13Halo4EntryRva, failed[0] ? failed : "none", callOk, tlsOk, pagesOk, instOk);
        return false;
    }
    bool callerOk[4]{};
    T13Callers(base, size, kT13Halo4DispatcherRva, kT13Halo4CallerRvas, lane, callerOk);
    uint32_t slot = 0;
    const bool frameOk = T13DispatcherSlot(base, kT13Halo4DispatcherRva, 26, kT13Halo4CallRva, slot);
    void* target = reinterpret_cast<void*>(base + kT13Halo4EntryRva);
    const MH_STATUS created = MH_CreateHook(target, reinterpret_cast<void*>(&Halo4Type13Detour), &lane.original);
    if (created != MH_OK)
    {
        lane.original = nullptr;
        LOG("H4 T13: type-13 log hook FAILED (create=%d); the lane is off", static_cast<int>(created));
        return false;
    }
    lane.base = base;
    lane.ctrlRet = base + kT13Halo4CallRva + 5;
    lane.dispSlot = frameOk ? slot : 0;
    lane.pages = pages;
    lane.instances = instances;
    lane.tlsIndex = g_halo4EngineTlsIndex;
    lane.generation.store(generation, std::memory_order_release);
    lane.owner.Reset();
    lane.busy.store(false, std::memory_order_release);
    lane.producer.Reset();
    if (MH_EnableHook(target) != MH_OK)
    {
        MH_RemoveHook(target);
        lane.original = nullptr;
        LOG("H4 T13: type-13 log hook FAILED to enable; removed, the lane is off");
        return false;
    }
    lane.target = target;
    lane.enabled.store(true, std::memory_order_release);
    LOG("H4 T13: type-13 pilot controller +0x%X hooked LOG-ONLY in %llu ms (writes nothing; type13_log %d); "
        "dispatcher return at entry rsp+0x%X %s; caller labels alt %d sway %d main %d nounit %d",
        kT13Halo4EntryRva, static_cast<unsigned long long>(GetTickCount64() - started), g_config.type13_log,
        lane.dispSlot * 8u, frameOk ? "(derived from the dispatcher prologue)" : "UNAVAILABLE (caller attribution off)",
        callerOk[0], callerOk[1], callerOk[2], callerOk[3]);
    return true;
}

// Worker, inside RemoveHalo4CameraCore before RemoveHalo4VehicleInput: admission off, disable without
// touching foreign bytes, prove quiescence (callback count + frozen-thread ingress scan of the detour and the
// trampoline), then remove; the trampoline is forgotten only after MH_RemoveHook.
bool Halo4Type13LogRemove()
{
    auto& lane = g_t13Halo4;
    lane.enabled.store(false, std::memory_order_release);
    if (!lane.target)
        return true;
    const MH_STATUS disabled = MCCVR_DisableHookForRetirement(lane.target);
    if (disabled != MH_OK && disabled != MH_ERROR_DISABLED && disabled != MH_ERROR_NOT_CREATED)
        return false;
    const void* functions[]{reinterpret_cast<const void*>(&Halo4Type13Detour)};
    const void* trampolines[]{lane.original};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 1, lane.callbacks))
        return false;
    const MH_STATUS removed = MH_RemoveHook(lane.target);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
        return false;
    lane.target = nullptr;
    lane.original = nullptr;
    return true;
}
#endif // HALOMCCVR_EXPERIMENTAL_HALO4_CAMERA

// ============================================================================================================
// THE COLD WORKER: drains both rings every loop (~50 ms) and formats every line; counters every 2 s.
// ============================================================================================================
namespace
{
const char* T13Prefix(uint8_t title) { return title == 0 ? "REACH SF" : "H4 T13"; }
float T13Deg(float rad) { return rad * t13::kRadToDeg; }
const char* T13EndReason(uint8_t reason)
{
    switch (static_cast<t13::StintEndReason>(reason))
    {
    case t13::StintEndReason::Vehicle: return "another vehicle";
    case t13::StintEndReason::Discontinuity: return "the tick went back or jumped: a load or revert";
    case t13::StintEndReason::Gap: return "no call for more than 500 ms";
    case t13::StintEndReason::Owner: return "another thread took the lane over";
    case t13::StintEndReason::Fault: return "a read faulted";
    }
    return "?";
}

void T13Format(const t13::Event& e)
{
    const char* p = T13Prefix(e.title);
    const bool reach = e.title == 0;
    switch (e.kind)
    {
    case t13::Kind::StintBegin:
        LOG("%s stint BEGIN veh 0x%08X user %u tick %d thread %u", p, e.vehicle, e.user, e.tick, e.u.stint.thread);
        break;
    case t13::Kind::StintEnd:
        LOG("%s stint END veh 0x%08X: %u ticks, %u calls (%s)", p, e.vehicle, e.u.stint.ticks, e.u.stint.calls,
            T13EndReason(e.u.stint.reason));
        break;
    case t13::Kind::Link:
    {
        const auto& d = e.u.link;
        if (reach)
            LOG("REACH SF LINK t=%d pc(y %.4f p %.4f r %.4f) facing(+0x1E4)(y %.4f p %.4f) err %.3f deg errPrev %.3f "
                "deg veh250 %.4f dRoll %.3f deg; hullFwd(+0x50)-facing %.2f deg",
                e.tick, d.post[0], d.post[1], d.post[2], d.facingYaw, d.facingPitch, T13Deg(d.err), T13Deg(d.errPrev),
                d.rollField, T13Deg(d.dRoll), T13Deg(d.craftErr));
        else
            LOG("H4 T13 LINK t=%d rec(y %.4f p %.4f r %.4f) facing(+0x4BC)(y %.4f p %.4f) err %.3f deg errPrev %.3f "
                "deg roll528 %.4f dRoll %.3f deg unit4BC err %.3f deg gate16Cbit2 %d; craftFwd(+0x70)-facing %.2f deg",
                e.tick, d.post[0], d.post[1], d.post[2], d.facingYaw, d.facingPitch, T13Deg(d.err), T13Deg(d.errPrev),
                d.rollField, T13Deg(d.dRoll), T13Deg(d.unitErr), d.gateBit, T13Deg(d.craftErr));
        break;
    }
    case t13::Kind::Window:
    {
        const auto& d = e.u.window;
        if (reach)
            LOG("REACH SF LINK 2s n %u err max %.3f mean %.3f errPrev max %.3f mean %.3f deg |dRoll| max %.3f deg; "
                "hullFwd-facing max %.2f mean %.2f deg",
                d.linkN, T13Deg(d.errMax), T13Deg(d.errMean), T13Deg(d.errPrevMax), T13Deg(d.errPrevMean),
                T13Deg(d.dRollMax), T13Deg(d.craftMax), T13Deg(d.craftMean));
        else
            LOG("H4 T13 LINK 2s n %u err max %.3f mean %.3f errPrev max %.3f mean %.3f deg |dRoll| max %.3f deg "
                "unit4BC err max %.3f deg; craftFwd-facing max %.2f mean %.2f deg",
                d.linkN, T13Deg(d.errMax), T13Deg(d.errMean), T13Deg(d.errPrevMax), T13Deg(d.errPrevMean),
                T13Deg(d.dRollMax), T13Deg(d.unitErrMax), T13Deg(d.craftMax), T13Deg(d.craftMean));
        LOG("%s RATE %.1fs maxTurn %.1f maxYaw %.1f maxPitch %.1f deg/s; stick args maxYaw %.1f maxPitch %.1f deg/s "
            "(|delta|/dtA); ticks %u excluded %u (stint start, %s window ticks %u); change lines held back %u",
            p, d.seconds, T13Deg(d.maxTurn), T13Deg(d.maxYaw), T13Deg(d.maxPitch), T13Deg(d.maxArgYaw),
            T13Deg(d.maxArgPitch), d.rateTicks, d.rateExcluded, reach ? "forced-steer" : "redirect", d.windowTicks,
            d.heldBack);
        break;
    }
    case t13::Kind::Tag:
    {
        const auto& d = e.u.tag;
        if (!d.valid)
        {
            LOG("%s TAG veh 0x%08X: the tag chain did not resolve (def 0x%04X n13 %d); window unknown, so forced-steer "
                "or redirect windows are not marked this stint", p, e.vehicle, d.def, d.n13);
            break;
        }
        if (reach)
            LOG("REACH SF TAG def 0x%04X n13 %d tagType %d | 5C %g 60 %g 64 %g 68 %g 6C %g 70 %g 74 %g 78 %g 7C %g "
                "80 %g 84 %g | A8 0x%08X (bit0 %u) AC %g | hull 10 %g 14 %g 38 %g 88 %g 8C %g 90 %g | veh3A0 %g",
                d.def, d.n13, d.tagType, d.ctl[0], d.ctl[1], d.ctl[2], d.ctl[3], d.ctl[4], d.ctl[5], d.ctl[6], d.ctl[7],
                d.ctl[8], d.ctl[9], d.ctl[10], d.projWord, d.projWord & 1u, d.window, d.phys[0], d.phys[1], d.phys[2],
                d.phys[3], d.phys[4], d.phys[5], d.vehScale);
        else
            LOG("H4 T13 TAG def 0x%04X n13 %d tagType %d | 60 %g 64 %g 68 %g 6C %g 70 %g 74 %g 78 %g 7C %g 80 %g "
                "84 %g 88 %g | C8 0x%08X (bit0 %u) CC %g | phys 10 %g 14 %g 3C %g A8 %g A4 0x%08X (bit0 %u) | "
                "veh6A4 %g",
                d.def, d.n13, d.tagType, d.ctl[0], d.ctl[1], d.ctl[2], d.ctl[3], d.ctl[4], d.ctl[5], d.ctl[6], d.ctl[7],
                d.ctl[8], d.ctl[9], d.ctl[10], d.projWord, d.projWord & 1u, d.window, d.phys[0], d.phys[1], d.phys[2],
                d.phys[3], d.physWord, d.physWord & 1u, d.vehScale);
        break;
    }
    case t13::Kind::Stamp:
    {
        const auto& d = e.u.stampEv;
        char was[16];
        if (d.was == t13::kUnread)
            snprintf(was, sizeof(was), "stint start");
        else
            snprintf(was, sizeof(was), "%d", d.was);
        if (reach)
            LOG("REACH SF FORCED t=%d stamp(+0xABC) %d (was %s) age %.3f s window(sf+0xAC) %.3f open %u "
                "dir(+0xAC0)(%.3f %.3f %.3f) projBit(sf+0xA8) %d aim214(%.3f %.3f %.3f) nativeJump %.2f deg",
                e.tick, d.stamp, was, d.age, d.window, d.open, d.dir.x, d.dir.y, d.dir.z, d.projBit, d.aim.x, d.aim.y,
                d.aim.z, T13Deg(d.nativeJump));
        else
            LOG("H4 T13 REDIRECT t=%d stamp(+0x104C) %d (was %s) age %.3f s window(T+0xCC) %.3f open %u "
                "dir(+0x1050)(%.3f %.3f %.3f) flag105C %d projBit(T+0xC8) %d nativeJump %.2f deg",
                e.tick, d.stamp, was, d.age, d.window, d.open, d.dir.x, d.dir.y, d.dir.z, d.flag, d.projBit,
                T13Deg(d.nativeJump));
        break;
    }
    case t13::Kind::Roll:
    {
        const auto& d = e.u.roll;
        if (reach)
            LOG("REACH SF ROLL t=%d pc88bit2 %u moveX %.3f rate(+0x144) %.4f idle(+0x140) %.3f hold(+0x148) %.3f "
                "latch(+0x14C) %u", e.tick, d.bit2, d.moveX, d.rate, d.idle, d.hold, d.latch);
        else
            LOG("H4 T13 ROLL t=%d rec08bit2 %u moveX %.3f rate(+0x11C) %.4f idle(+0x118) %.3f hold(+0x120) %.3f "
                "latch(+0x124) %u", e.tick, d.bit2, d.moveX, d.rate, d.idle, d.hold, d.latch);
        break;
    }
    case t13::Kind::Dt:
        LOG("%s DT t=%d a6(dtA) %.6f a7(dtB) %.6f spt %.6f", p, e.tick, e.u.dt.dtA, e.u.dt.dtB, e.u.dt.spt);
        break;
    case t13::Kind::State:
    {
        const auto& d = e.u.state;
        if (d.state == t13::kUnread)
            LOG("REACH SF STATE t=%d obj[+0x1AE] offset %d out of the read range: unread", e.tick, d.off);
        else if (d.was == t13::kUnread)
            LOG("REACH SF STATE t=%d obj[+0x1AE=0x%X] %d at the stint start (0x1B = hull steering skipped)", e.tick,
                d.off, d.state);
        else
            LOG("REACH SF STATE t=%d obj[+0x1AE=0x%X] -> %d (was %d; 0x1B = hull steering skipped)", e.tick, d.off,
                d.state, d.was);
        break;
    }
    case t13::Kind::Phys:
    {
        const auto& d = e.u.phys;
        char was[16];
        if (d.was == t13::kUnread)
            snprintf(was, sizeof(was), "stint start");
        else
            snprintf(was, sizeof(was), "%d", d.was);
        if (d.mode == t13::kUnread)
            LOG("H4 T13 BIPED t=%d unit 0x%08X physOff 0x%X out of the read range: unread", e.tick, d.unit, d.off);
        else
            LOG("H4 T13 BIPED t=%d unit 0x%08X physOff 0x%X mode %d (was %s; 7/8 = the record's angles are rebased, "
                "NOT world)", e.tick, d.unit, d.off, d.mode, was);
        break;
    }
    case t13::Kind::Type:
        LOG("H4 T13 TYPE t=%d tagType %d override F58 %d F59 %d (the hook firing = runtime type 13)", e.tick,
            e.u.type.tagType, e.u.type.f58, e.u.type.f59);
        break;
    case t13::Kind::Owner:
        LOG("%s owner thread %u -> %u (the previous owner was silent for more than %llu ms)", p, e.u.owner.from,
            e.u.owner.to, static_cast<unsigned long long>(kT13TakeoverMs));
        break;
    case t13::Kind::Fault:
        LOG("%s FAULT reading %s (veh 0x%08X); this stint observes nothing more", p,
            e.u.fault.where == 0 ? "before the native" : e.u.fault.where == 1 ? "after the native" : "the tag",
            e.vehicle);
        break;
    }
}

void T13Drain(T13Lane& lane)
{
    t13::Event e;
    for (unsigned n = 0; n < kT13RingSize && lane.ring.Pop(e); ++n)
        T13Format(e);
}

struct T13Counts
{
    uint32_t calls, ticks, dups, byCaller[5], foreignRet, otherThreads, otherUsers, busy, faults, badTime, drops;
};
T13Counts T13Take(T13Lane& lane)
{
    T13Counts c{};
    c.calls = lane.calls.exchange(0, std::memory_order_relaxed);
    c.ticks = lane.ticks.exchange(0, std::memory_order_relaxed);
    c.dups = lane.dups.exchange(0, std::memory_order_relaxed);
    for (int i = 0; i < 5; ++i)
        c.byCaller[i] = lane.byCaller[i].exchange(0, std::memory_order_relaxed);
    c.foreignRet = lane.foreignRet.exchange(0, std::memory_order_relaxed);
    c.otherThreads = lane.otherThreads.exchange(0, std::memory_order_relaxed);
    c.otherUsers = lane.otherUsers.exchange(0, std::memory_order_relaxed);
    c.busy = lane.busyHits.exchange(0, std::memory_order_relaxed);
    c.faults = lane.faults.exchange(0, std::memory_order_relaxed);
    c.badTime = lane.badTime.exchange(0, std::memory_order_relaxed);
    c.drops = lane.ring.TakeDropped();
    return c;
}
} // namespace

// Cold worker, every loop (after LegacyDriveReport).
void Type13LogReport()
{
    T13Drain(g_t13Reach);
    T13Drain(g_t13Halo4);
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 2000)
        return;
    s_last = now;
#if HALOMCCVR_EXPERIMENTAL_REACH_RENDER_CANDIDATE
    {
        const T13Counts c = T13Take(g_t13Reach);
        if (c.calls)
        {
            const uint64_t beat = g_reachLastCamCopyMs.load(std::memory_order_relaxed);
            LOG("REACH SF calls %u ticks %u dup %u /2s; callers %s %u %s %u %s %u unknown %u; foreignRet %u "
                "otherThreads %u otherUsers %u busy %u badTime %u; veh 0x%08X pcMaster %d armed %d; faults %u "
                "ringDrops %u; heartbeatAge %llu ms (read only, never stamped)",
                c.calls, c.ticks, c.dups, kT13ReachCallerNames[0], c.byCaller[0], kT13ReachCallerNames[1], c.byCaller[1],
                kT13ReachCallerNames[2], c.byCaller[2], c.byCaller[4] + c.byCaller[3], c.foreignRet, c.otherThreads,
                c.otherUsers, c.busy, c.badTime, g_t13Reach.lastVehicle.load(std::memory_order_relaxed),
                g_t13Reach.master.load(std::memory_order_relaxed), g_reachPcArmed.load(std::memory_order_acquire) ? 1 : 0,
                c.faults, c.drops, static_cast<unsigned long long>(beat && now >= beat ? now - beat : 0));
        }
    }
#endif
#if HALOMCCVR_EXPERIMENTAL_HALO4_CAMERA
    {
        auto& lane = g_t13Halo4;
        const T13Counts c = T13Take(lane);
        const uint32_t parent = lane.lastParent.load(std::memory_order_relaxed);
        const int32_t seat = lane.lastSeat.load(std::memory_order_relaxed);
        const bool viewOwned = NativeVehicleFirstPerson_SeatViewOwned(GameTitle::Halo4, parent, seat);
        if (c.calls)
            LOG("H4 T13 calls %u ticks %u dup %u /2s; callers %s %u %s %u %s %u %s %u unknown %u; foreignRet %u "
                "otherThreads %u otherUsers %u busy %u badTime %u; veh 0x%08X parent 0x%08X seat %d viewOwned %d "
                "recMaster %d; faults %u ringDrops %u",
                c.calls, c.ticks, c.dups, kT13Halo4CallerNames[0], c.byCaller[0], kT13Halo4CallerNames[1], c.byCaller[1],
                kT13Halo4CallerNames[2], c.byCaller[2], kT13Halo4CallerNames[3], c.byCaller[3], c.byCaller[4],
                c.foreignRet, c.otherThreads, c.otherUsers, c.busy, c.badTime,
                lane.lastVehicle.load(std::memory_order_relaxed), parent, seat, viewOwned ? 1 : 0,
                lane.master.load(std::memory_order_relaxed), c.faults, c.drops);
        // H4 CAM: the game's own aim (the camera forward the servo closes on, published with direct drive)
        // against the facing the physics steers by and the craft forward, while a stint is live.
        float facing[2]{}, craft[2]{};
        uint64_t snapMs = 0;
        bool snapOk = false;
        for (int attempt = 0; attempt < 2 && !snapOk; ++attempt)
        {
            const uint32_t before = lane.snapSeq.load(std::memory_order_acquire);
            if (before & 1u)
                continue;
            facing[0] = lane.snapFacingYaw.load(std::memory_order_relaxed);
            facing[1] = lane.snapFacingPitch.load(std::memory_order_relaxed);
            craft[0] = lane.snapCraftYaw.load(std::memory_order_relaxed);
            craft[1] = lane.snapCraftPitch.load(std::memory_order_relaxed);
            snapMs = lane.snapMs.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            snapOk = lane.snapSeq.load(std::memory_order_relaxed) == before;
        }
        if (snapOk && snapMs && now >= snapMs && now - snapMs <= 2500)
        {
            const auto& drive = g_legacyDrive[2];
            float cam[2]{};
            uint64_t setMs = 0;
            bool camOk = false;
            for (int attempt = 0; attempt < 2 && !camOk; ++attempt)
            {
                const uint32_t before = drive.seq.load(std::memory_order_acquire);
                if (before & 1u)
                    continue;
                cam[0] = drive.aimYaw.load(std::memory_order_relaxed);
                cam[1] = drive.aimPitch.load(std::memory_order_relaxed);
                setMs = drive.setMs.load(std::memory_order_relaxed);
                std::atomic_thread_fence(std::memory_order_acquire);
                camOk = drive.seq.load(std::memory_order_relaxed) == before;
            }
            const uint64_t camAge = setMs && now >= setMs ? now - setMs : ~0ull;
            if (!camOk || camAge > 500)
                LOG("H4 T13 CAM camFwd: no publication (the game's aim is published with aim_direct_drive=1) "
                    "facing(y %.3f p %.3f) craftFwd(y %.3f p %.3f) viewOwned %d",
                    facing[0], facing[1], craft[0], craft[1], viewOwned ? 1 : 0);
            else
            {
                const t13::Vec3 fwd = t13::ForwardFromYawPitch(cam[0], cam[1]);
                LOG("H4 T13 CAM camFwd(y %.3f p %.3f) age %llu ms vs facing %.2f deg vs craftFwd %.2f deg viewOwned %d",
                    cam[0], cam[1], static_cast<unsigned long long>(camAge),
                    T13Deg(t13::AngleBetween(fwd, t13::ForwardFromYawPitch(facing[0], facing[1]))),
                    T13Deg(t13::AngleBetween(fwd, t13::ForwardFromYawPitch(craft[0], craft[1]))), viewOwned ? 1 : 0);
            }
        }
    }
#endif
}
