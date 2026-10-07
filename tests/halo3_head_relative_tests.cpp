// Head-relative motion tracker, Halo 3 / ODST (src/common/halo3_head_relative_logic.h): the game's own placement
// arithmetic as read from halo3.dll, the substitution of the head's heading into it, every refusal, and the
// player's two headset checks as geometry -- head still and the gun swept: contacts stay put; gun still and the
// head turned: contacts turn with the head.
#include "halo3_head_relative_logic.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <string>
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
bool Near(float a, float b, float tol = 2.0e-4f) { return std::fabs(a - b) <= tol; }
float Wrap(float a)
{
    while (a > head_relative::kPi)
        a -= 2.0f * head_relative::kPi;
    while (a <= -head_relative::kPi)
        a += 2.0f * head_relative::kPi;
    return a;
}

using namespace halo3_head_relative;
using head_relative::kDegToRad;
using head_relative::kRadToDeg;
using head_relative::Use;

// A forward vector at `yawDeg` (counter-clockwise from +X, seen from above) and `pitchDeg` above the horizon.
void Forward(float yawDeg, float pitchDeg, float out[3])
{
    const float yaw = yawDeg * kDegToRad, pitch = pitchDeg * kDegToRad;
    out[0] = std::cos(pitch) * std::cos(yaw);
    out[1] = std::cos(pitch) * std::sin(yaw);
    out[2] = std::sin(pitch);
}

// Where a blip sits on the tracker as the player reads it: degrees counter-clockwise from the tracker's "up"
// (a contact to the LEFT is positive). Blips are x right, y DOWN.
float TrackerDeg(const float blip[2]) { return std::atan2(-blip[0], -blip[1]) * kRadToDeg; }

// A fresh, valid head sample looking along `yawDeg` / `pitchDeg` in generation 7 at clock 1'000'000.
constexpr uint32_t kGeneration = 7;
constexpr uint64_t kNowUs = 1000000;
Request Asked(float headYawDeg, float headPitchDeg = 0.0f)
{
    Request in{};
    in.switchOn = true;
    in.user = 0;
    in.sampleRead = true;
    in.gameplay = true;
    Forward(headYawDeg, headPitchDeg, in.sample.forward);
    in.sample.up[2] = 1.0f;
    in.sample.stampUs = kNowUs - 5000;
    in.sample.generation = kGeneration;
    in.sample.valid = true;
    in.generation = kGeneration;
    in.nowUs = kNowUs;
    in.sign = 1;
    in.offsetRad = 0.0f;
    return in;
}

// The tracker angle of a contact `contactYawDeg` around the unit (1 world unit away, range 2), for the game's aim
// at `aimYawDeg` and the request `in`. `use` gets the verdict.
float PlacedDeg(float aimYawDeg, float contactYawDeg, const Request& in, Hold& hold, Use* use = nullptr)
{
    float s = 0.0f, c = 0.0f;
    NativePair(aimYawDeg * kDegToRad, s, c);
    const Use verdict = Substitute(in, hold, s, c);
    if (use)
        *use = verdict;
    float blip[2];
    Place(std::cos(contactYawDeg * kDegToRad), std::sin(contactYawDeg * kDegToRad), s, c, 0.5f, blip);
    return TrackerDeg(blip);
}

// ---- the game's arithmetic, as disassembled ----
void TestNativeModel()
{
    float s = 0.0f, c = 0.0f, blip[2], f[3];
    // Aim along +X: a contact ahead (+X) is straight up; one to the right (-Y) is on the right; left on the left.
    NativePair(0.0f, s, c);
    Check(Near(s, -1.0f) && Near(c, 0.0f), "yaw 0: S = -cos(yaw) = -1, C = -sin(yaw) = 0");
    Place(3.0f, 0.0f, s, c, 0.25f, blip);
    Check(Near(blip[0], 0.0f) && Near(blip[1], -0.75f), "a contact ahead: x 0, y -distance/range (up)");
    Place(0.0f, -3.0f, s, c, 0.25f, blip);
    Check(Near(blip[0], 0.75f) && Near(blip[1], 0.0f), "a contact to the right (-Y of a +X heading): +x");
    Place(0.0f, 3.0f, s, c, 0.25f, blip);
    Check(Near(blip[0], -0.75f) && Near(blip[1], 0.0f), "a contact to the left: -x");
    Place(-3.0f, 0.0f, s, c, 0.25f, blip);
    Check(Near(blip[0], 0.0f) && Near(blip[1], 0.75f), "a contact behind: +y (down)");
    for (float yaw : {-170.0f, -90.0f, -33.0f, 0.0f, 12.5f, 90.0f, 135.0f, 180.0f})
    {
        NativePair(yaw * kDegToRad, s, c);
        Check(PairForward(s, c, f) && Near(f[0], std::cos(yaw * kDegToRad)) && Near(f[1], std::sin(yaw * kDegToRad)) &&
                  f[2] == 0.0f,
              "the pair's heading is (cos yaw, sin yaw): the control yaw's forward");
        float s2 = 0.0f, c2 = 0.0f;
        Check(ForwardPair(f, s2, c2) && Near(s2, s) && Near(c2, c), "heading -> pair is the inverse");
        // The native tracker: a contact's angle on it is its bearing from the aim.
        for (float contact : {-120.0f, -45.0f, 0.0f, 60.0f, 179.0f})
        {
            Place(std::cos(contact * kDegToRad), std::sin(contact * kDegToRad), s, c, 1.0f, blip);
            Check(Near(Wrap((TrackerDeg(blip) - (contact - yaw)) * kDegToRad), 0.0f, 1.0e-3f),
                  "stock: a contact's tracker angle is its bearing from the control yaw");
        }
    }
    Check(!PairForward(0.5f, 0.5f, f), "a pair that is not sin and cos of one angle is refused");
    Check(!PairForward(std::numeric_limits<float>::quiet_NaN(), 0.0f, f), "a NaN pair is refused");
    const float vertical[3]{0.0f, 0.0f, 1.0f};
    Check(!ForwardPair(vertical, s, c), "a vertical vector has no pair");
}

// ---- acceptance 1: head still, sweep the gun: contacts stay put ----
void TestGunSweepMovesNothing()
{
    for (float head : {-150.0f, -20.0f, 0.0f, 75.0f})
        for (float contact : {-90.0f, 10.0f, 140.0f})
        {
            Hold hold;
            const Request in = Asked(head);
            Use use = Use::Count;
            const float reference = PlacedDeg(head, contact, in, hold, &use);
            Check(use == Use::Ok, "a fresh head sample for the local user is used");
            bool still = true, agrees = true;
            for (float aim = -180.0f; aim <= 180.0f; aim += 15.0f)
            {
                const float placed = PlacedDeg(aim, contact, in, hold, &use);
                still = still && use == Use::Ok && Near(Wrap((placed - reference) * kDegToRad), 0.0f, 1.0e-3f);
                float offset[3]{std::cos(contact * kDegToRad), std::sin(contact * kDegToRad), 0.0f}, heading[3];
                float bearing = 0.0f;
                Forward(head, 0.0f, heading);
                agrees = agrees && head_relative::Bearing(offset, heading, kWorldUp, bearing) &&
                         Near(Wrap(placed * kDegToRad - bearing), 0.0f, 1.0e-3f);
            }
            Check(still, "head still, the aim swept all the way round: the contact does not move");
            Check(agrees, "and it sits at its bearing from the HEAD (the shared model of a tracker)");
        }
    // A contact straight ahead of the head is at the tracker's up, whatever the aim is.
    for (float aim : {-135.0f, -1.0f, 44.0f, 90.0f, 180.0f})
    {
        Hold hold;
        Check(Near(PlacedDeg(aim, 30.0f, Asked(30.0f), hold), 0.0f, 1.0e-2f),
              "a contact straight ahead of the head lands at up, whatever the aim");
    }
}

// ---- acceptance 2: gun still, turn the head: contacts turn with the head ----
void TestHeadTurnTurnsContacts()
{
    const float aim = 20.0f, contact = 20.0f;
    Hold hold;
    Check(Near(PlacedDeg(aim, contact, Asked(20.0f), hold), 0.0f, 1.0e-2f), "head on the contact: up");
    Check(Near(PlacedDeg(aim, contact, Asked(110.0f), hold), -90.0f, 1.0e-2f),
          "the head turned +90 deg (left): the fixed contact moves -90 deg (to the right side)");
    Check(Near(PlacedDeg(aim, contact, Asked(-70.0f), hold), 90.0f, 1.0e-2f),
          "the head turned -90 deg (right): the contact moves +90 deg (to the left side)");
    Check(Near(std::fabs(PlacedDeg(aim, contact, Asked(200.0f), hold)), 180.0f, 1.0e-2f),
          "the head turned round: the contact is behind");
    // Stock (switch off) for the same poses: the contact stays at its bearing from the AIM.
    Request off = Asked(110.0f);
    off.switchOn = false;
    Use use = Use::Count;
    Check(Near(PlacedDeg(aim, contact, off, hold, &use), 0.0f, 1.0e-2f) && use == Use::SwitchOff,
          "switch off: the game's own heading (the aim), head turned or not");
    // Head pitch and roll do not count: only the heading.
    Check(Near(PlacedDeg(aim, contact, Asked(110.0f, 60.0f), hold), -90.0f, 1.0e-2f), "looking 60 deg up: same heading");
    Check(Near(PlacedDeg(aim, contact, Asked(110.0f, -75.0f), hold), -90.0f, 1.0e-2f), "looking 75 deg down: same heading");
    Request rolled = Asked(110.0f);
    rolled.sample.up[0] = 0.6f;
    rolled.sample.up[2] = 0.8f;
    Check(Near(PlacedDeg(aim, contact, rolled, hold), -90.0f, 1.0e-2f), "a rolled head: same heading");
}

// ---- the safety values ----
void TestSignAndOffset()
{
    const float aim = 20.0f, contact = 20.0f;
    Hold hold;
    Request mirrored = Asked(110.0f);
    mirrored.sign = -1;
    Check(Near(PlacedDeg(aim, contact, mirrored, hold), 90.0f, 1.0e-2f),
          "sign -1 mirrors the head's turn: +90 deg of head moves the contact +90 deg");
    mirrored = Asked(20.0f);
    mirrored.sign = -1;
    Check(Near(PlacedDeg(aim, contact, mirrored, hold), 0.0f, 1.0e-2f), "sign -1 with the head on the aim: unchanged");
    Request shifted = Asked(20.0f);
    shifted.offsetRad = 30.0f * kDegToRad;
    Check(Near(PlacedDeg(aim, contact, shifted, hold), -30.0f, 1.0e-2f),
          "offset +30 deg: the tracker's up is 30 deg left of the head, so the contact reads 30 deg right");
    shifted.offsetRad = std::numeric_limits<float>::quiet_NaN();
    Check(Near(PlacedDeg(aim, contact, shifted, hold), 0.0f, 1.0e-2f), "a NaN offset counts as none");
}

// ---- every refusal leaves the game's own pair, bit for bit ----
void TestRefusals()
{
    const auto refused = [](Request in, Use expected, const char* what, float s0 = 0.0f, float c0 = 0.0f,
                            bool nativePair = true) {
        Hold hold;
        float s = s0, c = c0;
        if (nativePair)
            NativePair(33.0f * kDegToRad, s, c);
        float before[2]{s, c};
        float nativeForward[3]{9, 9, 9}, usedForward[3]{9, 9, 9};
        const Use use = Substitute(in, hold, s, c, nativeForward, usedForward);
        Check(use == expected, what);
        Check(std::memcmp(before, &s, sizeof(float)) == 0 && std::memcmp(before + 1, &c, sizeof(float)) == 0,
              "a refusal leaves the pair exactly as the game made it");
        Check(nativeForward[0] == 9 && usedForward[0] == 9, "and writes no heading");
    };
    Request in = Asked(120.0f);
    in.switchOn = false;
    refused(in, Use::SwitchOff, "the player's switch off");
    in = Asked(120.0f);
    in.user = 1;
    refused(in, Use::NotLocalUser, "output user 1");
    in.user = -1;
    refused(in, Use::NotLocalUser, "no user");
    in = Asked(120.0f);
    in.sampleRead = false;
    refused(in, Use::NoSample, "no coherent sample read");
    in = Asked(120.0f);
    in.sample.valid = false;
    refused(in, Use::NoSample, "the head view was not live");
    in = Asked(120.0f);
    in.sample.stampUs = kNowUs - head_relative::kMaxSampleAgeUs - 1;
    refused(in, Use::Stale, "a head sample older than 100 ms");
    in.sample.stampUs = kNowUs - head_relative::kMaxSampleAgeUs;
    {
        Hold hold;
        float s = 0.0f, c = 0.0f;
        NativePair(0.0f, s, c);
        Check(Substitute(in, hold, s, c) == Use::Ok, "exactly 100 ms old is still this frame's head");
    }
    in = Asked(120.0f);
    in.sample.stampUs = kNowUs + 1;
    refused(in, Use::FromTheFuture, "a sample stamped after now");
    in = Asked(120.0f);
    in.sample.generation = kGeneration + 1;
    refused(in, Use::OtherGeneration, "a sample from another load of the title");
    in = Asked(120.0f);
    in.generation = 0;
    refused(in, Use::OtherGeneration, "no title generation");
    in = Asked(120.0f);
    in.gameplay = false;
    refused(in, Use::NotGameplay, "a cinematic frame");
    in = Asked(120.0f);
    in.sample.forward[1] = std::numeric_limits<float>::infinity();
    refused(in, Use::NoHeading, "a non-finite head forward");
    in = Asked(120.0f);
    refused(in, Use::NoHeading, "a pair that is not a unit pair", 0.2f, 0.3f, false);
    refused(in, Use::NoHeading, "a NaN pair", std::numeric_limits<float>::quiet_NaN(), 1.0f, false);
    // Looking straight up before any heading was ever defined: nothing to hold.
    in = Asked(120.0f, 89.0f);
    refused(in, Use::NoHeading, "straight up with no heading held yet");
}

// ---- near straight up / down keep the last good heading; a new load forgets it ----
void TestHold()
{
    const float aim = -40.0f, contact = 10.0f;
    Hold hold;
    Use use = Use::Count;
    const float level = PlacedDeg(aim, contact, Asked(100.0f), hold, &use);
    Check(use == Use::Ok && Near(level, -90.0f, 1.0e-2f), "level head at 100 deg: the contact is 90 deg to the right");
    // Straight up: the sample's horizontal part is noise (here it points somewhere else entirely).
    const float up = PlacedDeg(aim, contact, Asked(-20.0f, 88.0f), hold, &use);
    Check(use == Use::Ok && Near(up, level, 1.0e-2f), "looking straight up: the tracker holds the last heading");
    const float down = PlacedDeg(aim, contact, Asked(170.0f, -85.0f), hold, &use);
    Check(use == Use::Ok && Near(down, level, 1.0e-2f), "looking straight down: still held");
    const float back = PlacedDeg(aim, contact, Asked(10.0f, 70.0f), hold, &use);
    Check(use == Use::Ok && Near(back, 0.0f, 1.0e-2f), "70 deg up has a heading again: it is used");
    // Another generation: the held heading belongs to the old load.
    Request next = Asked(-20.0f, 88.0f);
    next.generation = kGeneration + 1;
    next.sample.generation = kGeneration + 1;
    float s = 0.0f, c = 0.0f;
    NativePair(aim * kDegToRad, s, c);
    Check(Substitute(next, hold, s, c) == Use::NoHeading, "a new load starts with nothing held");
    Check(hold.generation == kGeneration + 1 && !hold.heading.have, "and the hold now belongs to it");
}

// ---- the head on the aim changes nothing: the substitution is the game's own quantity ----
void TestIdentityWhenHeadIsAim()
{
    for (float yaw : {-179.0f, -91.0f, -12.0f, 0.0f, 3.0f, 88.0f, 157.0f})
    {
        Hold hold;
        float s = 0.0f, c = 0.0f, s0 = 0.0f, c0 = 0.0f, nativeForward[3], usedForward[3];
        NativePair(yaw * kDegToRad, s, c);
        s0 = s;
        c0 = c;
        Check(Substitute(Asked(yaw, 25.0f), hold, s, c, nativeForward, usedForward) == Use::Ok && Near(s, s0) &&
                  Near(c, c0),
              "head heading = aim heading: the pair handed back is the game's own");
        Check(Near(nativeForward[0], usedForward[0]) && Near(nativeForward[1], usedForward[1]) &&
                  Near(usedForward[0], std::cos(yaw * kDegToRad)) && Near(usedForward[1], std::sin(yaw * kDegToRad)),
              "and both logged headings are that yaw");
        Check(Near(s * s + c * c, 1.0f), "the pair handed to the builder is a unit pair");
    }
}

// ---- the signatures: each decode offset lands where the comment says ----
std::vector<std::string> Tokens(const char* pattern)
{
    std::vector<std::string> out;
    std::istringstream in(pattern);
    std::string token;
    while (in >> token)
        out.push_back(token);
    return out;
}
void TestPatterns()
{
    const auto wild4 = [](const std::vector<std::string>& t, int at) {
        return at >= 0 && static_cast<size_t>(at) + 4 <= t.size() && t[at] == "??" && t[at + 1] == "??" &&
               t[at + 2] == "??" && t[at + 3] == "??";
    };
    const auto call = [&](const std::vector<std::string>& t, int at) {
        return at >= 0 && static_cast<size_t>(at) + 5 <= t.size() && t[at] == "E8" && wild4(t, at + 1);
    };
    const auto update = Tokens(kUpdateEntryPattern);
    Check(update.size() == 60 && update[kUpdateUserDisp - 2] == "8B" && update[kUpdateUserDisp - 1] == "89" &&
              update[kUpdateUserDisp] == "??" && update[kUpdateUserDisp + 1] == "??" &&
              update[kUpdateUserDisp + 2] == "00" && update[kUpdateUserDisp + 3] == "00",
          "the update's entry: mov ecx,[rcx+USER] with USER at the decode offset");
    Check(call(update, kUpdateUnitCall), "the update's entry: the unit call at its offset");
    const auto trig = Tokens(kTrigPattern);
    Check(trig.size() == 54 && trig[0] == "F3" && trig[3] == "35" && wild4(trig, kTrigHalfPi) &&
              trig[kTrigMask - 3] == "0F" && trig[kTrigMask - 2] == "57" && wild4(trig, kTrigMask) &&
              call(trig, kTrigCallA) && call(trig, kTrigCallB),
          "the trig site: pi/2, the sign mask and both calls at their offsets");
    Check(trig[kTrigCallA + 5] == "0F" && trig[kTrigCallA + 6] == "28" && trig[kTrigCallA + 7] == "F8",
          "the trig site: call A's result goes to xmm7");
    Check(trig[51] == "0F" && trig[52] == "28" && trig[53] == "F0", "the trig site: call B's result goes to xmm6");
    const auto point = Tokens(kPointCallPattern);
    Check(point.size() == 31 && call(point, kPointCall) && point[5] == "28" && point[15] == "20",
          "the point call: xmm6 to [rsp+0x28], xmm7 to [rsp+0x20], the call at its offset");
    const auto objectA = Tokens(kObjectCallAPattern), objectB = Tokens(kObjectCallBPattern);
    Check(objectA.size() == 31 && call(objectA, kObjectCallA) && objectA[5] == "30" && objectA[14] == "28",
          "object call A: xmm6 to [rsp+0x30], xmm7 to [rsp+0x28], the call at its offset");
    Check(objectB.size() == 33 && call(objectB, kObjectCallB) && objectB[5] == "30" && objectB[14] == "28",
          "object call B: the same slots, the call at its offset");
    for (const char* pattern : {kPointEntryPattern, kPointPlacePattern, kObjectEntryPattern, kObjectPlacePattern})
    {
        const auto t = Tokens(pattern);
        bool exact = t.size() >= 40;
        for (const auto& token : t)
            exact = exact && token != "??";
        Check(exact, "the builders' entry and placement patterns are exact bytes, 40 or more");
    }
}
} // namespace

int main()
{
    TestNativeModel();
    TestGunSweepMovesNothing();
    TestHeadTurnTurnsContacts();
    TestSignAndOffset();
    TestRefusals();
    TestHold();
    TestIdentityWhenHeadIsAim();
    TestPatterns();
    std::printf("halo3_head_relative_tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
