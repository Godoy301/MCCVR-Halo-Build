// Unit tests for the engine-free aim-solve core + provider orchestrator (src/common/aim_solve_logic.h,
// src/common/aim_provider.h). No game, no engine -- a mock provider drives solve_and_actuate. Plain
// Check() harness; non-zero exit = failure, so CTest reports it.

#include "../src/common/aim_provider.h"
#include "../src/common/aim_write_logic.h"

#include <cmath>
#include <cstdio>

using namespace aim_solve;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { ++g_fail; \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static bool near(float a, float b, float eps = 0.05f) { return std::fabs(a - b) <= eps; }

// A provider whose every answer is a settable field, so a test states the game's shape as data.
struct MockProvider : IAimProvider {
    AimCaps c{};
    Ray   intent{}; bool intent_ok = true;
    bool  trace_ok = false; float trace_cm = 0.0f;
    bool  delta_ok = false; Vec3 delta{};

    // observations
    bool actuated = false; Actuation last_how = Actuation::None; Angles last_angles{};
    bool engine_conv = false; Converge last_conv = Converge::DirectionOnly;
    int  trace_calls = 0;

    const char* title() const override { return "mock"; }
    AimCaps caps() const override { return c; }
    bool intent_ray(Ray* o) override { if (!intent_ok) return false; *o = intent; return true; }
    bool trace_impact_cm(const Ray&, float* o) override { ++trace_calls; if (!trace_ok) return false; *o = trace_cm; return true; }
    bool origin_delta(Vec3* o) override { if (!delta_ok) return false; *o = delta; return true; }
    ActuateResult actuate(Actuation how, const Angles& a) override {
        actuated = true; last_how = how; last_angles = a;
        return { true, how == Actuation::WriteState };
    }
    void apply_engine_convergence(Converge cv, const Ray&) override { engine_conv = true; last_conv = cv; }
};

static void test_selectors() {
    AimCaps c{};
    CHECK(select_actuation(c) == Actuation::None);           // nothing available -> fail closed
    c.has_stick_loop = true;
    CHECK(select_actuation(c) == Actuation::StickLoop);
    c.can_write_state = true;
    CHECK(select_actuation(c) == Actuation::WriteState);      // state write beats stick

    AimCaps v{};
    CHECK(select_converge(v) == Converge::DirectionOnly);     // floor
    v.has_fixed_range_reaim = true;
    CHECK(select_converge(v) == Converge::FixedRange);
    v.has_origin_relocation = true;
    CHECK(select_converge(v) == Converge::OriginRelocation);
    v.has_sightline_trace = true;
    CHECK(select_converge(v) == Converge::TraceReaim);        // trace beats all
}

static void test_angles_from_dir() {
    float y = 9, p = 9;
    CHECK(angles_from_dir(Vec3{1, 0, 0}, y, p) && near(y, 0) && near(p, 0));
    CHECK(angles_from_dir(Vec3{0, 1, 0}, y, p) && near(y, 90) && near(p, 0));
    CHECK(angles_from_dir(Vec3{0, 0, 1}, y, p) && near(p, 90));
    CHECK(!angles_from_dir(Vec3{0, 0, 0}, y, p));            // degenerate -> false
}

static void test_converge() {
    ConvergeRails r{};
    // Leashed: a sub-threshold delta must be an exact no-op (angles untouched, returns false).
    {
        float y = 3.0f, p = -2.0f;
        CHECK(!converge_trace_reaim(y, p, Vec3{1, 0, 0}, 1000.0f, r) && near(y, 3.0f) && near(p, -2.0f));
    }
    // Below the range floor: decline.
    {
        float y = 0, p = 0;
        CHECK(!converge_trace_reaim(y, p, Vec3{0, 10, 0}, 10.0f, r) && near(y, 0) && near(p, 0));
    }
    // A known correction: aim +x, eye 10 cm off in +y, target 1000 cm out -> ~0.573 deg of +yaw.
    {
        float y = 0, p = 0;
        CHECK(converge_trace_reaim(y, p, Vec3{0, 10, 0}, 1000.0f, r));
        CHECK(near(y, std::atan2(10.0f, 1000.0f) * 57.29578f) && near(p, 0));
    }
    // Sanity rail: an implausibly large correction is declined, not applied.
    {
        float y = 0, p = 0;
        CHECK(!converge_trace_reaim(y, p, Vec3{0, 10000, 0}, 1000.0f, r) && near(y, 0));
    }
}

static void test_orchestrator() {
    // No foundation -> fail closed, never actuates.
    {
        MockProvider m; m.intent = Ray{ Vec3{}, Vec3{1, 0, 0} };
        SolveState st{};
        ActuateResult res = solve_and_actuate(m, st, ConvergeRails{}, 0.011f, 120.0f);
        CHECK(!res.applied && !m.actuated);
    }
    // Stick loop + no convergence caps -> DirectionOnly engine conv, actuate raw intent on the stick.
    {
        MockProvider m; m.c.has_stick_loop = true; m.intent = Ray{ Vec3{}, Vec3{0, 1, 0} };
        SolveState st{};
        ActuateResult res = solve_and_actuate(m, st, ConvergeRails{}, 0.011f, 120.0f);
        CHECK(res.applied && m.actuated && m.last_how == Actuation::StickLoop);
        CHECK(m.engine_conv && m.last_conv == Converge::DirectionOnly);
        CHECK(m.trace_calls == 0);                            // no trace when not TraceReaim
        CHECK(near(m.last_angles.yaw_deg, 90) && near(m.last_angles.pitch_deg, 0));
    }
    // Origin relocation -> engine convergence, angles stay the intent, trace never consulted.
    {
        MockProvider m; m.c.can_write_state = true; m.c.has_origin_relocation = true;
        m.intent = Ray{ Vec3{}, Vec3{1, 0, 0} };
        SolveState st{};
        ActuateResult res = solve_and_actuate(m, st, ConvergeRails{}, 0.011f, 120.0f);
        CHECK(res.applied && res.wrote_replicated_state && m.last_how == Actuation::WriteState);
        CHECK(m.engine_conv && m.last_conv == Converge::OriginRelocation && m.trace_calls == 0);
        CHECK(near(m.last_angles.yaw_deg, 0));
    }
    // Trace-and-reaim -> core bends the setpoint from the traced range + delta.
    {
        MockProvider m; m.c.can_write_state = true; m.c.has_sightline_trace = true;
        m.intent = Ray{ Vec3{}, Vec3{1, 0, 0} };
        m.trace_ok = true; m.trace_cm = 1000.0f; m.delta_ok = true; m.delta = Vec3{0, 10, 0};
        SolveState st{};
        ActuateResult res = solve_and_actuate(m, st, ConvergeRails{}, 1.0f, 1.0f); // alpha~1: range=trace
        CHECK(res.applied && m.trace_calls == 1 && !m.engine_conv);
        CHECK(near(m.last_angles.yaw_deg, std::atan2(10.0f, 1000.0f) * 57.29578f));
    }
    // No intent this tick -> fail closed.
    {
        MockProvider m; m.c.has_stick_loop = true; m.intent_ok = false;
        SolveState st{};
        ActuateResult res = solve_and_actuate(m, st, ConvergeRails{}, 0.011f, 120.0f);
        CHECK(!res.applied && !m.actuated);
    }
}

// ---- aim_write_logic.h: the state-write rung's guards --------------------------------------------
static void test_unwrap_to_branch() {
    using aim_write::unwrap_to_branch;
    using aim_write::kTwoPi;
    CHECK(near(unwrap_to_branch(1.0f, 1.2f), 1.0f, 1e-5f));                   // same branch: unchanged
    // Game holds yaw on [0,2pi); the aim computes -0.1 for the same direction as 6.183. Keep 6.183.
    CHECK(near(unwrap_to_branch(-0.1f, 6.2f), kTwoPi - 0.1f, 1e-4f));
    CHECK(near(unwrap_to_branch(6.2f, -0.1f), 6.2f - kTwoPi, 1e-4f));         // and the other way
    // Accumulated game yaw far past 2*pi (never normalised): still the nearest branch.
    CHECK(near(unwrap_to_branch(0.5f, 5.0f * kTwoPi + 0.4f), 5.0f * kTwoPi + 0.5f, 1e-3f));
    CHECK(near(unwrap_to_branch(0.3f, NAN), 0.3f, 1e-6f));                    // no current value
    // Never more than half a turn from the current value.
    for (float t = -10.0f; t <= 10.0f; t += 0.37f)
        CHECK(std::fabs(unwrap_to_branch(t, 2.0f) - 2.0f) <= 3.1416f);
}

static void test_clamp_pitch() {
    using aim_write::clamp_pitch;
    CHECK(near(clamp_pitch(0.2f, 1.45f), 0.2f, 1e-6f));
    CHECK(near(clamp_pitch(1.6f, 1.45f), 1.45f, 1e-6f));
    CHECK(near(clamp_pitch(-2.0f, 1.45f), -1.45f, 1e-6f));
}

static void test_arm_gate() {
    using aim_write::ArmGate;
    using Ev = ArmGate::Event;
    constexpr uint64_t A = 0x1000, B = 0x2000;   // two candidate record addresses
    {   // arms only after 3 CONSECUTIVE agreements, and reports what it proved
        ArmGate g;
        CHECK(g.step(true, A, 0, 0) == Ev::None && g.step(true, A, 0, 10) == Ev::None);
        CHECK(g.step(false, A, 0, 20) == Ev::None && !g.armed);               // streak broken
        CHECK(g.step(true, A, 0, 30) == Ev::None && g.step(true, A, 0, 40) == Ev::None);
        CHECK(g.step(true, A, 0, 50) == Ev::Armed && g.armed && g.record == A);
        CHECK(g.step(false, 0, 0, 60) == Ev::None && g.armed);                // agreement not consulted while armed
        CHECK(!g.wants_evidence(70));                                        // ...so the caller need not gather it
    }
    {   // a streak must be about ONE record: a changed candidate restarts the count on the new one
        ArmGate g;
        g.step(true, A, 0, 0); g.step(true, A, 0, 1);
        CHECK(g.step(true, B, 0, 2) == Ev::None && g.streak == 1 && g.record == B);
        CHECK(g.step(true, B, 0, 3) == Ev::None);
        CHECK(g.step(true, B, 0, 4) == Ev::Armed && g.record == B);
    }
    {   // "agreement" about no record at all never counts
        ArmGate g;
        for (int i = 0; i < 5; ++i) CHECK(g.step(true, 0, 0, i) == Ev::None);
        CHECK(!g.armed);
    }
    {   // a writer mismatch disarms at once, then backs off before re-validating
        ArmGate g;
        for (int i = 0; i < 3; ++i) g.step(true, A, 0, i);
        CHECK(g.armed);
        CHECK(g.step(true, 0, 1, 100) == Ev::Disarmed && !g.armed && g.disarms == 1);
        CHECK(!g.wants_evidence(200) && g.wants_evidence(5100));
        for (int i = 0; i < 5; ++i) CHECK(g.step(true, A, 1, 200 + i) == Ev::None);  // inside the backoff
        CHECK(!g.armed);
        g.step(true, A, 1, 5200); g.step(true, A, 1, 5210);
        CHECK(g.step(true, A, 1, 5220) == Ev::Armed);                         // re-proven after backoff
    }
    {   // mismatches counted while DISARMED are not the armed record's fault
        ArmGate g;
        g.step(true, A, 7, 0); g.step(true, A, 8, 1);
        CHECK(g.step(true, A, 9, 2) == Ev::Armed);
        CHECK(g.step(true, 0, 9, 3) == Ev::None && g.armed);
    }
    {   // three disarms -> give up for the session, and stay given up
        ArmGate g;
        uint64_t t = 0; unsigned mm = 0;
        for (int round = 0; round < 3; ++round) {
            for (int i = 0; i < 3; ++i) g.step(true, A, mm, t++);
            CHECK(g.armed);
            const Ev e = g.step(true, 0, ++mm, t);
            CHECK(e == (round < 2 ? Ev::Disarmed : Ev::GaveUp));
            t += 6000;
        }
        CHECK(g.given_up && !g.armed && !g.wants_evidence(t));
        for (int i = 0; i < 5; ++i) CHECK(g.step(true, A, mm, t + i) == Ev::None && !g.armed);
    }
    {   // release() drops the arm without counting a failure; re-arming re-proves from scratch
        ArmGate g;
        for (int i = 0; i < 3; ++i) g.step(true, A, 0, i);
        g.release();
        CHECK(!g.armed && g.disarms == 0 && g.record == 0);
        CHECK(g.step(true, A, 0, 10) == Ev::None && g.step(true, A, 0, 11) == Ev::None);
        CHECK(g.step(true, A, 0, 12) == Ev::Armed);
    }
}

int main() {
    test_selectors();
    test_angles_from_dir();
    test_converge();
    test_orchestrator();
    test_unwrap_to_branch();
    test_clamp_pitch();
    test_arm_gate();
    if (g_fail == 0) std::printf("aim_solve_tests: OK\n");
    return g_fail == 0 ? 0 : 1;
}
