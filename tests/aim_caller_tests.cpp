// Aim caller split (src/common/aim_caller_logic.h): which caller ran the aim accumulator, what that call may
// do, the dispatcher-frame proofs, the put-back slot, and the dev census -- plus pass simulations of the
// seat-limit latch and the seat stint check, with and without the split.
#include "aim_caller_logic.h"
#include "aim_write_logic.h"
#include "seat_drive_policy.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

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

using aim_caller::Action;
using aim_caller::ActionFor;
using aim_caller::Caller;
using aim_caller::CallerMap;
using aim_caller::Context;
using aim_caller::PutBack;
using aim_caller::Refusal;

constexpr float kDeg = 0.01745329252f;

// The measured UNWIND_INFO of the two dispatchers (retail Steam modules).
const uint8_t kH4DispatcherUnwind[] = {0x19, 0x35, 0x0F, 0x00, 0x27, 0x88, 0x0A, 0x00, 0x22, 0x78, 0x0B, 0x00,
                                       0x1E, 0x68, 0x0C, 0x00, 0x1A, 0x34, 0x23, 0x00, 0x1A, 0x01, 0x1A, 0x00,
                                       0x0F, 0xF0, 0x0D, 0xE0, 0x0B, 0xC0, 0x09, 0x60, 0x08, 0x50, 0x00, 0x00};
const uint8_t kReachDispatcherUnwind[] = {0x19, 0x2F, 0x0D, 0x00, 0x21, 0x78, 0x0A, 0x00, 0x1D, 0x68, 0x0B,
                                          0x00, 0x19, 0x01, 0x18, 0x00, 0x0E, 0xF0, 0x0C, 0xE0, 0x0A, 0xD0,
                                          0x08, 0xC0, 0x06, 0x70, 0x05, 0x30, 0x04, 0x50, 0x00, 0x00};
// Their prologues, up to and including the stack allocation.
const uint8_t kH4DispatcherPrologue[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x20, 0x55, 0x56, 0x41, 0x54, 0x41,
                                         0x56, 0x41, 0x57, 0x48, 0x8D, 0x68, 0xB1, 0x48, 0x81, 0xEC, 0xD0, 0x00,
                                         0x00, 0x00, 0x0F, 0x29, 0x70, 0xC8};
const uint8_t kReachDispatcherPrologue[] = {0x48, 0x8B, 0xC4, 0x55, 0x53, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41,
                                            0x56, 0x41, 0x57, 0x48, 0x8D, 0x68, 0xB1, 0x48, 0x81, 0xEC, 0xC0,
                                            0x00, 0x00, 0x00, 0x0F, 0x29, 0x70, 0xB8};

CallerMap DirectMap(Action noUnit)
{
    CallerMap m;
    m.add(0x1000, Caller::Stick);
    m.add(0x2000, Caller::Kick);
    m.add(0x3000, Caller::NoUnit);
    m.no_unit_action = noUnit;
    return m;
}

CallerMap DispatcherMap()
{
    CallerMap m;
    m.acc_ret = 0xA898D;
    m.dispatcher_frame = 0x100;
    m.add(0xA7779, Caller::Stick);
    m.add(0xA34A3, Caller::Impulse);
    m.add(0xA7756, Caller::Sway);
    m.add(0xA7A06, Caller::NoUnit);
    return m;
}

void TestClassifyDirect()
{
    const CallerMap m = DirectMap(Action::RestoreOnly);
    Check(m.stick_proven(), "direct: a stick site proves rung 1");
    Check(m.classify(0x1000, 0) == Caller::Stick, "direct: the stick return is the stick");
    Check(m.classify(0x2000, 0) == Caller::Kick, "direct: the kick return is the kick");
    Check(m.classify(0x4000, 0) == Caller::Unknown, "direct: any other return is unknown");
    Check(m.classify(0x1000, 0x2000) == Caller::Stick, "direct: a dispatcher return is ignored");
    Check(ActionFor(m, Caller::Stick) == Action::StepAndWrite, "direct: the stick steps and writes");
    Check(ActionFor(m, Caller::Kick) == Action::RestoreOnly, "direct: a kick only puts back");
    Check(ActionFor(m, Caller::Unknown) == Action::RestoreOnly, "direct: an unknown caller only puts back (as before)");
    Check(ActionFor(m, Caller::NoUnit) == Action::RestoreOnly,
          "Halo 3 / ODST: the no-unit call keeps the put-back it always had");
    const CallerMap ends = DirectMap(Action::EndUpdate);
    Check(ActionFor(ends, Caller::NoUnit) == Action::EndUpdate, "a title may end the update on the no-unit call");
    uint8_t stack[0x110]{};
    Check(aim_caller::DispatcherReturn(m, stack, 0x1000) == 0, "direct: no dispatcher return is read");
}

void TestClassifyDispatcher()
{
    const CallerMap m = DispatcherMap();
    Check(m.stick_proven(), "dispatcher: the stick site and a sane frame prove rung 1");
    // The hook's stack at accumulator entry: its own return at [0], the dispatcher's at [0x100].
    alignas(8) uint8_t stack[0x120]{};
    const uintptr_t accRet = 0xA898D;
    const auto put = [&stack](size_t at, uintptr_t v) { std::memcpy(stack + at, &v, sizeof(v)); };
    put(0, accRet);
    const struct { uintptr_t dispatcherRet; Caller want; Action act; } rows[] = {
        {0xA7779, Caller::Stick, Action::StepAndWrite},
        {0xA34A3, Caller::Impulse, Action::RestoreOnly},
        {0xA7756, Caller::Sway, Action::RestoreOnly},
        {0xA7A06, Caller::NoUnit, Action::EndUpdate},
        {0xDEAD0, Caller::Unknown, Action::RestoreOnly},
    };
    for (const auto& r : rows)
    {
        put(0x100, r.dispatcherRet);
        const uintptr_t read = aim_caller::DispatcherReturn(m, stack, accRet);
        Check(read == r.dispatcherRet, "dispatcher: the return is read at entry rsp + 0x100");
        const Caller c = m.classify(accRet, read);
        Check(c == r.want, "dispatcher: the dispatcher's return names the caller");
        Check(ActionFor(m, c) == r.act, "dispatcher: each role gets its action");
    }
    // The hook entered from somewhere other than the accumulator's one call site.
    put(0x100, 0xA7779);
    Check(aim_caller::DispatcherReturn(m, stack, 0x12345) == 0, "dispatcher: a foreign hook return reads nothing");
    Check(m.classify(0x12345, 0xA7779) == Caller::Unknown, "dispatcher: a foreign hook return is unknown");
    Check(ActionFor(m, Caller::Unknown) == Action::RestoreOnly,
          "dispatcher: once the stick is proven an unknown caller is put back, never the update");
}

void TestLadder()
{
    // No stick site: every call is the update (the behaviour before the split), whatever its role.
    CallerMap m;
    m.add(0xA34A3, Caller::Impulse);
    Check(!m.stick_proven(), "no stick site: rung 1 not proven");
    for (int c = 0; c < aim_caller::kCallerCount; ++c)
        Check(ActionFor(m, static_cast<Caller>(c)) == Action::StepAndWrite, "unproven: every call steps and writes");
    // Dispatcher mode with a frame that was not proven.
    m = DispatcherMap();
    m.dispatcher_frame = 0;
    Check(!m.stick_proven(), "dispatcher: no frame, no proof");
    uint8_t stack[0x120]{};
    Check(aim_caller::DispatcherReturn(m, stack, 0xA898D) == 0, "dispatcher: an unproven frame is never read");
    m.dispatcher_frame = 0x104;
    Check(!m.stick_proven(), "dispatcher: a misaligned frame is refused");
    m.dispatcher_frame = 0x2000;
    Check(!m.stick_proven(), "dispatcher: an oversized frame is refused");
    // A missing extra site is only an unknown caller: still put back once the stick is proven.
    m = CallerMap{};
    m.acc_ret = 0xA898D;
    m.dispatcher_frame = 0x100;
    m.add(0xA7779, Caller::Stick);
    Check(m.stick_proven(), "the stick site alone proves rung 1");
    Check(m.classify(0xA898D, 0xA34A3) == Caller::Unknown && ActionFor(m, Caller::Unknown) == Action::RestoreOnly,
          "an unmapped impulse site reads unknown and is put back");
    // The table itself.
    CallerMap t;
    Check(t.add(0x10, Caller::Stick) && t.add(0x10, Caller::Stick), "the same site twice is accepted");
    Check(!t.add(0x10, Caller::Kick), "one return with two roles is refused");
    Check(!t.add(0x20, Caller::Unknown) && !t.add(0, Caller::Kick), "unknown roles and null returns are refused");
    for (uintptr_t r = 0x30; r < 0x30 + 16; ++r)
        t.add(r, Caller::Kick);
    Check(t.count == aim_caller::kMaxSites, "the table holds at most kMaxSites");
    t.clear();
    Check(t.count == 0 && !t.stick_proven(), "clear empties the map");
}

void TestUnwind()
{
    uint32_t frame = 0;
    uint8_t prolog = 0;
    Check(aim_caller::FrameFromUnwindInfo(kH4DispatcherUnwind, sizeof(kH4DispatcherUnwind), frame, &prolog) &&
              frame == 0xF8 && prolog == 0x35,
          "Halo 4 dispatcher unwind: 5 pushes + ALLOC_LARGE 0xD0 = 0xF8 (SAVE_NONVOL and SAVE_XMM128 skipped)");
    Check(aim_caller::FrameFromUnwindInfo(kReachDispatcherUnwind, sizeof(kReachDispatcherUnwind), frame, &prolog) &&
              frame == 0xF8 && prolog == 0x2F,
          "Reach dispatcher unwind: 7 pushes + ALLOC_LARGE 0xC0 = 0xF8");

    std::vector<uint8_t> u(kH4DispatcherUnwind, kH4DispatcherUnwind + sizeof(kH4DispatcherUnwind));
    u[3] = 0x05;   // frame register rbp
    Check(!aim_caller::FrameFromUnwindInfo(u.data(), u.size(), frame), "a frame register is refused");
    u.assign(kH4DispatcherUnwind, kH4DispatcherUnwind + sizeof(kH4DispatcherUnwind));
    u[0] = 0x21;   // version 1, CHAININFO
    Check(!aim_caller::FrameFromUnwindInfo(u.data(), u.size(), frame), "a chained entry is refused");
    u[0] = 0x1A;   // version 2
    Check(!aim_caller::FrameFromUnwindInfo(u.data(), u.size(), frame), "an unknown version is refused");
    Check(!aim_caller::FrameFromUnwindInfo(kH4DispatcherUnwind, 20, frame), "codes past the readable length are refused");
    Check(!aim_caller::FrameFromUnwindInfo(kH4DispatcherUnwind, 3, frame), "a truncated header is refused");
    {
        // ALLOC_LARGE whose operand slot lies past CountOfCodes.
        const uint8_t bad[] = {0x01, 0x04, 0x01, 0x00, 0x04, 0x01, 0x1A, 0x00};
        Check(!aim_caller::FrameFromUnwindInfo(bad, sizeof(bad), frame), "a multi-slot op cut short by the count is refused");
    }
    {
        const uint8_t fpreg[] = {0x01, 0x08, 0x02, 0x00, 0x08, 0x03, 0x04, 0x50};
        Check(!aim_caller::FrameFromUnwindInfo(fpreg, sizeof(fpreg), frame), "SET_FPREG is refused");
        const uint8_t mach[] = {0x01, 0x04, 0x02, 0x00, 0x04, 0x0A, 0x04, 0x50};
        Check(!aim_caller::FrameFromUnwindInfo(mach, sizeof(mach), frame), "PUSH_MACHFRAME is refused");
    }
    {
        // ALLOC_LARGE info 1 (32-bit size), ALLOC_SMALL, SAVE_NONVOL_FAR (3 slots), then a push: widths exact.
        const uint8_t ok[] = {0x01, 0x20, 0x08, 0x00,
                              0x20, 0x35, 0x00, 0x00, 0x10, 0x00,   // SAVE_NONVOL_FAR rbx (3 slots)
                              0x18, 0x32,                           // ALLOC_SMALL 3 -> 0x20
                              0x10, 0x11, 0x45, 0x23, 0x01, 0x00,   // ALLOC_LARGE info 1 -> 0x12345
                              0x04, 0x50, 0x00, 0x00};              // push rbp; padding
        Check(aim_caller::FrameFromUnwindInfo(ok, sizeof(ok), frame) && frame == 0x8 + 0x20 + 0x12345,
              "FAR saves are skipped by their exact width; ALLOC_LARGE info 1 reads 32 bits");
        const uint8_t save[] = {0x01, 0x10, 0x04, 0x00,
                                0x10, 0x99, 0x02, 0x00,              // SAVE_XMM128_FAR xmm9 (3 slots)
                                0x00, 0x00,
                                0x04, 0x50, 0x00, 0x00};
        uint8_t local[sizeof(save)];
        std::memcpy(local, save, sizeof(save));
        Check(aim_caller::FrameFromUnwindInfo(local, sizeof(local), frame) && frame == 8,
              "SAVE_XMM128_FAR moves no rsp");
    }
    {
        const uint8_t empty[] = {0x01, 0x00, 0x00, 0x00};
        Check(!aim_caller::FrameFromUnwindInfo(empty, sizeof(empty), frame), "a function with no frame is refused");
    }
}

void TestPrologue()
{
    uint32_t frame = 0;
    Check(aim_caller::FrameFromPrologue(kH4DispatcherPrologue, sizeof(kH4DispatcherPrologue), frame) && frame == 0xF8,
          "Halo 4 dispatcher prologue: 5 pushes + sub rsp,0xD0 = 0xF8");
    Check(aim_caller::FrameFromPrologue(kReachDispatcherPrologue, sizeof(kReachDispatcherPrologue), frame) &&
              frame == 0xF8,
          "Reach dispatcher prologue: 7 pushes + sub rsp,0xC0 = 0xF8");
    // Halo 4's accumulator prologue: lea rbp,[rax-0xF8] (disp32 form), 5 pushes, sub rsp,0x1D0.
    const uint8_t acc[] = {0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x10, 0x48, 0x89, 0x70, 0x18, 0x48, 0x89, 0x78,
                           0x20, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xA8, 0x08,
                           0xFF, 0xFF, 0xFF, 0x48, 0x81, 0xEC, 0xD0, 0x01, 0x00, 0x00};
    Check(aim_caller::FrameFromPrologue(acc, sizeof(acc), frame) && frame == 0x28 + 0x1D0,
          "a disp32 lea and five pushes before a 0x1D0 allocation");
    const uint8_t small[] = {0x48, 0x8B, 0xC4, 0x53, 0x48, 0x83, 0xEC, 0x20};
    Check(aim_caller::FrameFromPrologue(small, sizeof(small), frame) && frame == 0x28, "sub rsp, imm8");
    const uint8_t movss[] = {0x48, 0x8B, 0xC4, 0xF3, 0x0F, 0x11, 0x58, 0x20, 0x55, 0x48, 0x83, 0xEC, 0x20};
    Check(!aim_caller::FrameFromPrologue(movss, sizeof(movss), frame), "an instruction it does not know is refused");
    Check(!aim_caller::FrameFromPrologue(kH4DispatcherPrologue, 20, frame), "a prologue cut before its allocation is refused");
    const uint8_t noAlloc[] = {0x55, 0x56, 0x57};
    Check(!aim_caller::FrameFromPrologue(noAlloc, sizeof(noAlloc), frame), "pushes with no allocation are refused");
}

void TestProveDispatcherFrame()
{
    std::vector<uint8_t> image(0x1000, 0xCC);
    const uint32_t disp = 0x100, unwind = 0x800, begin = 0x100, end = 0x400, call = 0x260;
    std::memcpy(image.data() + disp, kH4DispatcherPrologue, sizeof(kH4DispatcherPrologue));
    std::memcpy(image.data() + unwind, kH4DispatcherUnwind, sizeof(kH4DispatcherUnwind));
    uint32_t frame = 0;
    const char* why = nullptr;
    Check(aim_caller::ProveDispatcherFrame(image.data(), image.size(), disp, begin, end, unwind, call, frame, why) &&
              frame == 0x100 && !why,
          "the Halo 4 dispatcher proves an entry frame of 0x100");
    Check(!aim_caller::ProveDispatcherFrame(image.data(), image.size(), disp, begin + 0x10, end, unwind, call, frame, why) &&
              frame == 0 && why,
          "a .pdata entry that does not begin at the dispatcher is refused");
    Check(!aim_caller::ProveDispatcherFrame(image.data(), image.size(), disp, begin, end, unwind, end - 2, frame, why),
          "a call that does not lie whole inside the dispatcher is refused");
    Check(!aim_caller::ProveDispatcherFrame(image.data(), image.size(), disp, begin, end, unwind, begin + 0x10, frame, why),
          "a call inside the prolog is refused");
    Check(!aim_caller::ProveDispatcherFrame(image.data(), image.size(), disp, begin, end, 0xFFF, call, frame, why),
          "unreadable unwind info is refused");
    std::vector<uint8_t> other = image;
    other[disp + 22] = 0xD8;   // sub rsp, 0xD8: the prologue no longer agrees with the unwind data
    Check(!aim_caller::ProveDispatcherFrame(other.data(), other.size(), disp, begin, end, unwind, call, frame, why) &&
              why && std::strstr(why, "disagrees"),
          "a prologue that disagrees with the unwind info is refused");
    other = image;
    other[unwind + 3] = 0x05;
    Check(!aim_caller::ProveDispatcherFrame(other.data(), other.size(), disp, begin, end, unwind, call, frame, why),
          "a frame register in the unwind info is refused");
    // Reach's measured pair proves the same frame.
    std::memcpy(image.data() + disp, kReachDispatcherPrologue, sizeof(kReachDispatcherPrologue));
    std::memcpy(image.data() + unwind, kReachDispatcherUnwind, sizeof(kReachDispatcherUnwind));
    Check(aim_caller::ProveDispatcherFrame(image.data(), image.size(), disp, begin, end, unwind, call, frame, why) &&
              frame == 0x100,
          "the Reach dispatcher proves an entry frame of 0x100");
}

void TestPutBack()
{
    PutBack s;
    s.max_age = 500;
    const Context foot{};
    Context seat{true, 0x1234u, 1u, true};
    const bool both[2]{true, true}, yawOnly[2]{true, false};
    const float w[2]{1.0f, 0.2f};
    const float native[2]{1.0f + 2.0f * kDeg, 0.2f - 1.0f * kDeg};
    Check(s.check(0x500, foot, 1, 10, native) == Refusal::Invalid, "nothing written: nothing to put back");
    s.wrote(0x500, both, w, foot, 7, 100);
    Check(s.check(0x500, foot, 7, 150, native) == Refusal::None, "the last stick write is put back over an impulse");
    Check(s.check(0x600, foot, 7, 150, native) == Refusal::Record, "another record is never written");
    Check(s.check(0x500, foot, 8, 150, native) == Refusal::StickSince, "a stick call since the write ends it");
    Check(s.check(0x500, seat, 7, 150, native) == Refusal::Context, "a seat entry since the write ends it");
    Check(s.check(0x500, foot, 7, 601, native) == Refusal::Age, "the safety cap ends it");
    Check(s.check(0x500, foot, 7, 99, native) == Refusal::Age, "a clock that went back ends it");
    const float far[2]{1.0f + 12.0f * kDeg, 0.2f};
    Check(s.check(0x500, foot, 7, 150, far) == Refusal::Implausible, "a record moved past an impulse is never reverted");
    const float nan[2]{std::nanf(""), 0.2f};
    Check(s.check(0x500, foot, 7, 150, nan) == Refusal::Implausible, "a non-finite record is never reverted");
    // Yaw wraps.
    const float wrapW[2]{3.13f, 0.0f}, wrapN[2]{-3.13f, 0.0f};
    s.wrote(0x500, both, wrapW, foot, 7, 100);
    Check(s.check(0x500, foot, 7, 150, wrapN) == Refusal::None, "yaw differences wrap across +-pi");
    // An axis not written is not judged (a Halo 4 seat writes the heading only).
    s.wrote(0x500, yawOnly, w, seat, 9, 100);
    const float pitchFar[2]{1.0f, 1.2f};
    Check(s.check(0x500, seat, 9, 150, pitchFar) == Refusal::None, "an unwritten axis is not judged");
    seat.axes = 3u;
    Check(s.check(0x500, seat, 9, 150, pitchFar) == Refusal::Context, "an axis-ownership change ends it");
    seat.axes = 1u;
    seat.seat_key = 0x9999u;
    Check(s.check(0x500, seat, 9, 150, native) == Refusal::Context, "another seat ends it");
    seat.seat_key = 0x1234u;
    seat.pitch_latch = false;
    Check(s.check(0x500, seat, 9, 150, native) == Refusal::Context, "a latch-policy change ends it");
    s.invalidate();
    Check(s.check(0x500, foot, 9, 150, native) == Refusal::Invalid, "invalidated: nothing to put back");
    const bool none[2]{false, false};
    s.wrote(0x500, none, w, foot, 9, 100);
    Check(!s.valid, "a write of no axis records nothing");
    const float bad[2]{std::nanf(""), 0.0f};
    s.wrote(0x500, both, bad, foot, 9, 100);
    Check(!s.valid, "a non-finite write records nothing");
    PutBack noCap;
    noCap.wrote(0x500, both, w, foot, 1, 100);
    Check(noCap.check(0x500, foot, 1, 100000, w) == Refusal::None, "max_age 0 is no cap");
}

// One seated control pass, Halo 4 / Reach shape: [impulse] -> stick. The record is the yaw axis only (the
// pitch is not written: a Halo 4 seat). `split`: the caller split; otherwise every call steps the latch and
// writes, as before. Returns whether the stick call's write was held at a "limit".
struct SeatSim
{
    aim_write::LimitLatch latch;
    PutBack slot;
    float record = 0.0f;
    uint32_t seq = 0;
    uint64_t now = 0;
    bool limitedOnStick = false;
    unsigned limitedFrames = 0;

    SeatSim()
    {
        slot.max_age = 500;
        const float w[2]{0.0f, 0.0f};
        const bool rec[2]{true, false};
        latch.wrote(w, rec);   // primed: last frame wrote 0
        const bool axes[2]{true, false};
        slot.wrote(0x500, axes, w, Context{true, 1u, 1u, true}, seq, now);
    }

    bool step(float target)
    {
        const bool enabled[2]{true, false};
        const float native[2]{record, 0.0f}, tgt[2]{target, 0.0f};
        float out[2]{};
        const bool limited = latch.apply(native, tgt, out, enabled);
        record = out[0];
        const float written[2]{record, 0.0f};
        const bool rec[2]{true, false};
        latch.wrote(written, rec);
        return limited;
    }

    void pass(float extraDelta, float target, bool split)
    {
        now += 8;
        if (extraDelta != 0.0f)
        {
            record += extraDelta;   // the extra call's native accumulation
            if (split)
            {
                const float native[2]{record, 0.0f};
                if (slot.check(0x500, Context{true, 1u, 1u, true}, seq, now, native) == Refusal::None)
                    record = slot.value[0];
            }
            else
            {
                step(target);       // before the split: the extra call was an update too
            }
        }
        ++seq;                      // the stick call
        limitedOnStick = step(target);
        if (limitedOnStick)
            ++limitedFrames;
        const bool axes[2]{true, false};
        const float w[2]{record, 0.0f};
        slot.wrote(0x500, axes, w, Context{true, 1u, 1u, true}, seq, now);
    }
};

void TestLimitLatchPass()
{
    // A 2 deg impulse ahead of the stick call, the hand still (target = last write).
    SeatSim before;
    before.pass(2.0f * kDeg, 0.0f, false);
    Check(before.limitedOnStick, "before the split: an impulse ahead of the stick call latches a false seat limit");
    Check(std::fabs(before.record - 0.5f * kDeg) < 0.02f * kDeg,
          "before the split: that frame's stick write lands 0.5 deg short of the hand");
    SeatSim after;
    after.pass(2.0f * kDeg, 0.0f, true);
    Check(!after.limitedOnStick && after.record == 0.0f,
          "with the split: the impulse is put back and the stick write lands on the hand, unheld");

    // A weapon sway on every pass (Halo 4 0xA7751 in a moving vehicle), the hand sweeping slowly.
    SeatSim swayBefore, swayAfter;
    for (int f = 0; f < 120; ++f)
    {
        const float sway = 2.5f * kDeg * std::sin(f * 0.7f);
        const float hand = 10.0f * kDeg * std::sin(f * 0.02f);
        swayBefore.pass(sway, hand, false);
        swayAfter.pass(sway, hand, true);
    }
    Check(swayBefore.limitedFrames > 10, "before the split: a sway every pass keeps the seat 'held at a limit'");
    Check(swayAfter.limitedFrames == 0, "with the split: a sway every pass never reads as a seat limit");
    Check(std::fabs(swayAfter.record - 10.0f * kDeg * std::sin(119 * 0.02f)) < 1e-5f,
          "with the split: the record ends on the hand");
}

void TestStintPass()
{
    // The seat stint check (item 11 realism). While a stint is CHECKING nothing is written, so the put-back
    // slot is invalid and an impulse legitimately stays in the record; the game's published aim follows the
    // record a frame later. Only the stick call steps the check, once per pass.
    const auto run = [](float bigImpulseDeg, int impulsePass, int& passedAt) {
        seat_drive::StintCheck stint;
        float record = 0.3f, publishedAim = 0.3f;
        passedAt = -1;
        for (int pass = 1; pass <= 80 && passedAt < 0; ++pass)
        {
            float delta = (pass % 5 == 0) ? 1.0f * kDeg : 0.0f;   // small impulses under fire
            if (pass == impulsePass)
                delta = bigImpulseDeg * kDeg;
            record += delta;                                       // the extra call; nothing to put back
            stint.step(true, record, 0.0f, publishedAim, 0.0f, 1000u + pass * 8u, true);   // the stick call
            if (stint.yaw_ok)
                passedAt = pass;
            publishedAim = record;                                 // the camera catches up next frame
        }
    };
    int passedAt = 0;
    run(0.0f, 0, passedAt);
    Check(passedAt == 30, "impulses under the 1.7 deg tolerance: yaw passes on the 30th stick call");
    run(2.5f, 10, passedAt);
    Check(passedAt == 40,
          "a 2.5 deg impulse while checking resets yaw (the record leads the published aim): passes 30 later");

    // Before the split, a pass with an extra call stepped twice: the extra call consumed the fresh publication
    // (new_sample) and the stick call was never judged. Same verdict while checking, one judgement per pass.
    seat_drive::StintCheck stint;
    float record = 0.3f, aim = 0.3f;
    int judged = 0;
    for (int pass = 1; pass <= 29; ++pass)
    {
        const bool impulse = pass % 2 == 0;
        if (impulse)
        {
            record += 0.5f * kDeg;
            stint.step(true, record, 0.0f, aim, 0.0f, 1000u + pass * 8u, true);   // the impulse call
            ++judged;
        }
        stint.step(true, record, 0.0f, aim, 0.0f, 1000u + pass * 8u + 1u, !impulse);   // the stick call
        judged += impulse ? 0 : 1;
        aim = record;
    }
    Check(stint.yaw_count == 29 && judged == 29, "before the split: still one judgement per pass, on the first call");
}

void TestCensus()
{
    aim_caller::TickCensus census;
    // 120 fps over a 60 Hz sim: two passes per tick, an impulse ahead of the stick on frame 11.
    for (uint32_t frame = 0; frame < 200; ++frame)
    {
        if (frame == 11)
            census.note(true, 1000 + frame / 2, Caller::Impulse, false);
        census.note(true, 1000 + frame / 2, Caller::Stick, frame >= 100);
    }
    auto t = census.take();
    Check(t.ticks == 99 && t.sticks == 198, "120 fps over 60 Hz: 99 folded ticks, 198 stick calls");
    Check(std::fabs(static_cast<double>(t.sticks) / t.ticks - 2.0) < 1e-9, "two stick calls per tick");
    Check(t.extra_before_stick == 1 && t.extra_passes == 1, "the impulse ahead of the stick call is counted once");
    Check(t.calls[0][static_cast<int>(Caller::Stick)] == 100 && t.calls[1][static_cast<int>(Caller::Stick)] == 100,
          "calls split on foot / seated");
    Check(t.calls[1][static_cast<int>(Caller::Impulse)] == 0 && t.calls[0][static_cast<int>(Caller::Impulse)] == 1,
          "the impulse is counted on foot");
    Check(t.max_calls == 3 && t.no_stick == 0 && t.stalled == 0 && t.went_back == 0, "max 3 calls in a tick");

    // A pause: the tick freezes while the control pass keeps running (40 calls), then resumes.
    for (int i = 0; i < 40; ++i)
        census.note(true, 1100, Caller::Stick, false);
    census.note(true, 1101, Caller::Stick, false);
    t = census.take();
    Check(t.stalled == 1 && t.ticks == 1, "a 40-call tick is a stall, kept out of the mean");
    // A map change or a revert sends the tick back.
    census.note(true, 1101, Caller::Stick, false);
    census.note(true, 5, Caller::Stick, false);
    census.note(true, 6, Caller::Stick, false);
    t = census.take();
    Check(t.went_back == 1 && t.ticks == 2, "a tick that went back is counted as a reset");
    // No unit (dead): impulse then no-unit, no stick.
    census.note(true, 7, Caller::Impulse, false);
    census.note(true, 7, Caller::NoUnit, false);
    census.note(true, 8, Caller::NoUnit, false);
    census.note(true, 9, Caller::Stick, false);
    t = census.take();
    Check(t.no_stick == 2 && t.extra_passes == 0, "ticks with no stick call; an impulse before no-unit is no stick pass");
    census.note(false, 0, Caller::Stick, false);
    t = census.take();
    Check(t.unreadable == 1 && t.calls[0][0] == 1, "an unreadable tick still counts the call");
    census.restart();
    census.note(true, 50, Caller::Stick, false);
    census.note(true, 51, Caller::Stick, false);
    t = census.take();
    Check(t.ticks == 1 && t.went_back == 0, "restart forgets the previous tick");
}

void TestAtomicMax()
{
    std::atomic<uint32_t> m{0};
    aim_caller::AtomicMaxFloat(m, 0.5f);
    aim_caller::AtomicMaxFloat(m, 0.25f);
    aim_caller::AtomicMaxFloat(m, -3.0f);
    aim_caller::AtomicMaxFloat(m, std::nanf(""));
    Check(aim_caller::BitsToFloat(m.load()) == 0.5f, "the float maximum keeps the largest non-negative value");
    aim_caller::AtomicMaxFloat(m, 2.0f);
    Check(aim_caller::BitsToFloat(m.exchange(0)) == 2.0f && m.load() == 0, "and is taken by exchange");
    std::atomic<uint32_t> u{3};
    aim_caller::AtomicMaxU32(u, 2);
    aim_caller::AtomicMaxU32(u, 9);
    Check(u.load() == 9, "the integer maximum");
}

void TestNames()
{
    Check(std::strcmp(aim_caller::CallerName(Caller::Sway), "sway") == 0 &&
              std::strcmp(aim_caller::CallerName(Caller::NoUnit), "no-unit") == 0,
          "caller names");
    Check(std::strcmp(aim_caller::RefusalName(Refusal::StickSince), "stick since") == 0, "refusal names");
}
} // namespace

int main()
{
    TestClassifyDirect();
    TestClassifyDispatcher();
    TestLadder();
    TestUnwind();
    TestPrologue();
    TestProveDispatcherFrame();
    TestPutBack();
    TestLimitLatchPass();
    TestStintPass();
    TestCensus();
    TestAtomicMax();
    TestNames();
    std::printf("aim caller tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
