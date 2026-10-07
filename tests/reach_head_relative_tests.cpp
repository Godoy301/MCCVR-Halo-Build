// Reach: the motion tracker follows the head (src/common/reach_head_relative_logic.h). Two halves: the decode of
// the game's own sites (the heading's offset, the hook, the readers' agreement, the pi/2 constant), and the
// player's two headset checks as geometry -- on the game's OWN placement arithmetic, not on a model of it.
#include "reach_head_relative_logic.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
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
bool Near(float a, float b, float tol = 1.0e-4f) { return std::fabs(a - b) <= tol; }
float Wrap(float a)
{
    while (a > head_relative::kPi)
        a -= 2.0f * head_relative::kPi;
    while (a <= -head_relative::kPi)
        a += 2.0f * head_relative::kPi;
    return a;
}

using namespace reach_head_relative;
using head_relative::HeadingHold;
using head_relative::HeadSample;
using head_relative::kDegToRad;
using head_relative::kPi;
using head_relative::Use;

// ---- the sites ----------------------------------------------------------------------------------------------------
// A pattern's bytes, wildcards as zero.
std::vector<uint8_t> PatternBytes(const char* pattern)
{
    std::vector<uint8_t> bytes;
    for (const char* at = pattern; *at;)
    {
        while (*at == ' ')
            ++at;
        if (!*at)
            break;
        if (at[0] == '?')
        {
            bytes.push_back(0);
            while (*at == '?')
                ++at;
            continue;
        }
        char* end = nullptr;
        bytes.push_back(static_cast<uint8_t>(std::strtoul(at, &end, 16)));
        at = end;
    }
    return bytes;
}
void PutU32(std::vector<uint8_t>& bytes, size_t at, uint32_t value) { std::memcpy(bytes.data() + at, &value, 4); }
void PutRel(std::vector<uint8_t>& bytes, size_t opcode, int64_t fromRva, int64_t toRva)
{
    const int32_t rel = static_cast<int32_t>(toRva - (fromRva + static_cast<int64_t>(opcode) + 5));
    std::memcpy(bytes.data() + opcode + 1, &rel, 4);
}

// The image as retail haloreach.dll lays it out (the offsets from the module base measured 2026-10-06).
constexpr int64_t kUpdateRva = 0x2ED0C8, kSiteRva = 0x2ED187, kFirstRva = 0x2EC164, kSecondRva = 0x2EC404,
                  kThirdRva = 0x2ECEB0, kHalfPiRva = 0xA8B004;
constexpr uint32_t kHeading = 0x13D4, kUser = 0x13B8;

struct Image
{
    std::vector<uint8_t> site, update, first, third;
    uint8_t halfPi[4]{};
};
Image Retail()
{
    Image image;
    image.site = PatternBytes(kTrackerHeadingSitePattern);
    image.site.resize(kSiteBytes, 0);
    const int32_t halfPiRel = static_cast<int32_t>(kHalfPiRva - (kSiteRva + static_cast<int64_t>(kSiteHalfPiNext)));
    std::memcpy(image.site.data() + kSiteHalfPiDisp, &halfPiRel, 4);
    PutU32(image.site, kSiteHeadingDisp, kHeading);
    PutRel(image.site, kSiteCallFirst, kSiteRva, kFirstRva);
    PutRel(image.site, kSiteCallSecond, kSiteRva, kSecondRva);
    PutRel(image.site, kSiteCallThird, kSiteRva, kThirdRva);
    image.update = PatternBytes(kTrackerUpdateHeadPattern);
    PutU32(image.update, kUpdateUserDisp, kUser);
    image.first = PatternBytes(kTrackerFirstReaderHeadPattern);
    image.first.resize(kFirstReaderBytes, 0x90);
    std::memcpy(image.first.data() + kFirstReaderRead, kFirstReaderReadOpcode, sizeof(kFirstReaderReadOpcode));
    PutU32(image.first, kFirstReaderReadDisp, kHeading);
    image.third.assign(kThirdReaderBytes, 0x90);
    std::memcpy(image.third.data() + kThirdReaderRead, kThirdReaderReadOpcode, sizeof(kThirdReaderReadOpcode));
    PutU32(image.third, kThirdReaderReadDisp, kHeading);
    const float value = kHalfPi;
    std::memcpy(image.halfPi, &value, 4);
    return image;
}
TrackerBind Decode(const Image& image, TrackerSites& sites, int64_t siteFunctionRva = kUpdateRva,
                   int64_t firstRva = kFirstRva)
{
    return DecodeTrackerSites(image.site.data(), kSiteRva, image.update.data(), kUpdateRva, siteFunctionRva,
                              image.first.data(), firstRva, sites);
}

void TestSites()
{
    Check(PatternBytes(kTrackerHeadingSitePattern).size() + 4 == kSiteBytes, "the store site's length is its pattern plus the last rel32");
    Check(PatternBytes(kTrackerUpdateHeadPattern).size() == kUpdateHeadBytes, "the update head's length is its pattern");
    Check(PatternBytes(kTrackerFirstReaderHeadPattern).size() == kFirstReaderHeadBytes &&
              kFirstReaderHeadBytes <= kFirstReaderRead,
          "the first reader's head pattern ends before its read of the heading");

    const Image retail = Retail();
    TrackerSites sites;
    Check(Decode(retail, sites) == TrackerBind::Bound, "retail's layout decodes");
    Check(sites.headingOffset == kHeading && sites.userOffset == kUser, "the heading and the user index are read from the instructions");
    Check(sites.firstReaderRva == kFirstRva && sites.thirdReaderRva == kThirdRva && sites.halfPiRva == kHalfPiRva,
          "the hook, the other reader and the constant are where the site points");
    Check(CheckTrackerReaders(retail.first.data(), retail.third.data(), retail.halfPi, sites) == TrackerBind::Bound,
          "both readers read the stored field and the constant is pi/2");

    TrackerSites untouched;
    Check(DecodeTrackerSites(nullptr, kSiteRva, retail.update.data(), kUpdateRva, kUpdateRva, retail.first.data(),
                             kFirstRva, untouched) == TrackerBind::SiteMissing,
          "no unique store site: nothing binds");
    Check(DecodeTrackerSites(retail.site.data(), kSiteRva, nullptr, kUpdateRva, kUpdateRva, retail.first.data(),
                             kFirstRva, untouched) == TrackerBind::UpdateHeadMissing,
          "no unique update head: nothing binds");
    Check(DecodeTrackerSites(retail.site.data(), kSiteRva, retail.update.data(), kUpdateRva, kUpdateRva, nullptr,
                             kFirstRva, untouched) == TrackerBind::FirstHeadMissing,
          "no unique first reader: nothing binds");
    Check(Decode(retail, untouched, kUpdateRva + 0x40) == TrackerBind::SiteOutsideUpdate,
          "a store inside another function than the matched update is refused");
    Check(Decode(retail, untouched, -1) == TrackerBind::SiteOutsideUpdate, "a store in no function is refused");
    Check(Decode(retail, untouched, kUpdateRva, kFirstRva + 0x10) == TrackerBind::CallMismatch,
          "a call that lands elsewhere than the matched head is refused");
    Check(untouched.headingOffset == 0 && untouched.firstReaderRva == 0, "a refusal fills nothing");

    Image odd = retail;
    PutU32(odd.site, kSiteHeadingDisp, 0x13D5);
    Check(Decode(odd, untouched) == TrackerBind::Implausible, "a heading that is not float-aligned is refused");
    odd = retail;
    PutU32(odd.update, kUpdateUserDisp, kHeading + 8);
    Check(Decode(odd, untouched) == TrackerBind::Implausible, "a user index past the heading is refused");

    // The neighbouring field: the likeliest way a patch goes wrong is a reader that moved and a store that did not.
    Image moved = retail;
    PutU32(moved.first, kFirstReaderReadDisp, kHeading + 4);
    Check(CheckTrackerReaders(moved.first.data(), moved.third.data(), moved.halfPi, sites) == TrackerBind::FieldMismatch,
          "the first reader reading the neighbouring field is refused");
    moved = retail;
    PutU32(moved.third, kThirdReaderReadDisp, kHeading - 4);
    Check(CheckTrackerReaders(moved.first.data(), moved.third.data(), moved.halfPi, sites) == TrackerBind::FieldMismatch,
          "the third reader reading the neighbouring field is refused");
    moved = retail;
    moved.first[kFirstReaderRead + 4] = 0x96;  // another base register
    Check(CheckTrackerReaders(moved.first.data(), moved.third.data(), moved.halfPi, sites) == TrackerBind::ReaderMismatch,
          "a first reader that does not read the heading there is refused");
    Check(CheckTrackerReaders(retail.first.data(), nullptr, retail.halfPi, sites) == TrackerBind::ReaderMismatch,
          "an unreadable third reader is refused");
    const float pi = kPi;
    uint8_t wrong[4];
    std::memcpy(wrong, &pi, 4);
    Check(CheckTrackerReaders(retail.first.data(), retail.third.data(), wrong, sites) == TrackerBind::ConstantMismatch,
          "a constant that is not pi/2 is refused");
    bool named = true;
    for (int i = 0; i < static_cast<int>(TrackerBind::Count); ++i)
        named = named && TrackerBindName(static_cast<TrackerBind>(i))[0] != '?';
    Check(named, "every refusal has a name for the log");
}

// ---- the heading --------------------------------------------------------------------------------------------------
void Forward(float yawDeg, float pitchDeg, float out[3])
{
    const float yaw = yawDeg * kDegToRad, pitch = pitchDeg * kDegToRad;
    out[0] = std::cos(pitch) * std::cos(yaw);
    out[1] = std::cos(pitch) * std::sin(yaw);
    out[2] = std::sin(pitch);
}
HeadSample Head(float yawDeg, float pitchDeg = 0.0f, uint64_t stampUs = 5000000, uint32_t generation = 3)
{
    HeadSample sample;
    Forward(yawDeg, pitchDeg, sample.forward);
    sample.up[2] = 1.0f;
    sample.stampUs = stampUs;
    sample.generation = generation;
    sample.valid = true;
    return sample;
}
TrackerRequest Request(int sign = 1, float offsetDeg = 0.0f)
{
    TrackerRequest request;
    request.switchOn = true;
    request.outputUser = 0;
    request.gameplay = true;
    request.generation = 3;
    request.nowUs = 5000000 + 8000;  // the sample is 8 ms old
    request.sign = sign;
    request.offsetDeg = offsetDeg;
    return request;
}
// The game's yaw as its accessor returns it: [0, 2 pi).
float GameYaw(float yawDeg) { return WrapYaw(yawDeg * kDegToRad); }
// Where the game draws `contact` (a world offset from the player) with the aim at aimYawDeg and the head at
// headYawDeg: the bearing from the tracker's top, positive to its left. `substitute` = this lane ran.
float Drawn(const float contact[3], float aimYawDeg, float headYawDeg, bool substitute = true, int sign = 1,
            float offsetDeg = 0.0f)
{
    float field = FieldFromYaw(GameYaw(aimYawDeg));
    if (substitute)
    {
        HeadingHold hold;
        float replaced = field;
        if (TrackerField(Request(sign, offsetDeg), Head(headYawDeg), hold, field, replaced) != Use::Ok)
            return 1.0e9f;
        field = replaced;
    }
    float point[2];
    TrackerPoint(contact, field, point);
    return TrackerPointBearing(point);
}

void TestPlacement()
{
    // The game's own arithmetic: ahead is the tracker's top (0, -r), right is (+r, 0).
    float point[2];
    const float ahead[3]{5.0f, 0.0f, 0.0f}, right[3]{0.0f, -5.0f, 0.0f}, left[3]{0.0f, 5.0f, 0.0f};
    TrackerPoint(ahead, FieldFromYaw(0.0f), point);
    Check(Near(point[0], 0.0f) && Near(point[1], -5.0f), "facing +X, a contact ahead is at the tracker's top");
    TrackerPoint(right, FieldFromYaw(0.0f), point);
    Check(Near(point[0], 5.0f) && Near(point[1], 0.0f), "facing +X, a contact at -Y (the right) is at the tracker's right");
    TrackerPoint(left, FieldFromYaw(0.0f), point);
    Check(Near(TrackerPointBearing(point), 90.0f * kDegToRad), "a contact to the left has a positive bearing");
    float north[3]{0.0f, 7.0f, 0.0f};
    TrackerPoint(north, FieldFromYaw(90.0f * kDegToRad), point);
    Check(Near(point[0], 0.0f) && Near(point[1], -7.0f), "facing +Y, a contact at +Y is at the top");
    Check(Near(WrapYaw(-0.1f), kTwoPi - 0.1f) && Near(WrapYaw(kTwoPi + 0.25f), 0.25f) && WrapYaw(kTwoPi) < kTwoPi &&
              WrapYaw(-kTwoPi) >= 0.0f,
          "a yaw is wrapped into [0, 2 pi)");
}

void TestAcceptance()
{
    // (0) A contact straight ahead of the HEAD is at the tracker's top, whatever the aim is.
    bool ahead = true;
    for (float headYaw : {0.0f, 45.0f, 170.0f, -120.0f, 359.0f})
    {
        float contact[3];
        Forward(headYaw, 0.0f, contact);
        contact[0] *= 12.0f;
        contact[1] *= 12.0f;
        contact[2] = 4.0f;  // above the player: the tracker ignores height for the bearing
        for (float aimYaw : {0.0f, 30.0f, -75.0f, 179.0f, 270.0f})
            ahead = ahead && Near(Drawn(contact, aimYaw, headYaw), 0.0f, 1.0e-3f);
    }
    Check(ahead, "a contact straight ahead of the head is at the tracker's top, whatever the aim");

    // (1) Head still, sweep the gun left and right: the contact stays put.
    const float contact[3]{4.0f, 7.0f, 0.0f};
    const float reference = Drawn(contact, 0.0f, 20.0f);
    bool still = true;
    for (float aimYaw = -175.0f; aimYaw <= 175.0f; aimYaw += 13.0f)
        still = still && Near(Wrap(Drawn(contact, aimYaw, 20.0f) - reference), 0.0f, 1.0e-3f);
    Check(still, "head still, gun swept through 350 deg: the contact does not move");

    // (2) Gun still, turn the head by +90 deg (to the left): a fixed contact turns by -90 deg on the tracker.
    Check(Near(Wrap(Drawn(contact, 35.0f, 110.0f) - Drawn(contact, 35.0f, 20.0f)), -90.0f * kDegToRad, 1.0e-3f),
          "gun still, head +90 deg: the contact turns -90 deg on the tracker");

    // The control arm: without this lane the GUN moves the contact and the head does not.
    Check(Near(Wrap(Drawn(contact, 60.0f, 20.0f, false) - Drawn(contact, 0.0f, 20.0f, false)), -60.0f * kDegToRad,
               1.0e-3f) &&
              Near(Wrap(Drawn(contact, 0.0f, 110.0f, false) - Drawn(contact, 0.0f, 20.0f, false)), 0.0f, 1.0e-3f),
          "control arm: on the game's own tracker a 60 deg gun sweep moves the contact 60 deg and the head moves nothing");

    // Head and gun together: the lane changes nothing (what a flat player sees).
    Check(Near(Wrap(Drawn(contact, 77.0f, 77.0f) - Drawn(contact, 77.0f, 0.0f, false)), 0.0f, 1.0e-3f),
          "head along the aim: exactly the game's own tracker");

    // sign -1 mirrors the head's turn (the live safety value), and the offset is added after it.
    Check(Near(Wrap(Drawn(contact, 0.0f, 90.0f, true, -1) - Drawn(contact, 0.0f, 0.0f, true, -1)), 90.0f * kDegToRad,
               1.0e-3f),
          "sign -1: head +90 deg turns the contact +90 deg (mirrored)");
    Check(Near(Wrap(Drawn(contact, 0.0f, 20.0f, true, 1, 30.0f) - Drawn(contact, 0.0f, 20.0f)), -30.0f * kDegToRad,
               1.0e-3f),
          "an offset of +30 deg turns the tracker's heading 30 deg further left");
}

void TestRefusals()
{
    const float native = FieldFromYaw(GameYaw(40.0f));
    const HeadSample head = Head(130.0f);
    HeadingHold hold;
    float field = 123.0f, headYaw = 0.0f;
    Check(TrackerField(Request(), head, hold, native, field, &headYaw) == Use::Ok &&
              Near(field, FieldFromYaw(130.0f * kDegToRad)) && Near(headYaw, 130.0f * kDegToRad),
          "used: the field is the head's yaw plus pi/2");

    const auto refused = [&](TrackerRequest request, HeadSample sample, Use why, const char* what) {
        HeadingHold fresh;
        float out = 123.0f;
        Check(TrackerField(request, sample, fresh, native, out) == why && out == 123.0f, what);
    };
    TrackerRequest request = Request();
    request.switchOn = false;
    refused(request, head, Use::SwitchOff, "the switch off: the game's own heading");
    request = Request();
    request.outputUser = 1;
    refused(request, head, Use::NotLocalUser, "another output user: the game's own heading");
    request.outputUser = -1;
    refused(request, head, Use::NotLocalUser, "no output user: the game's own heading");
    request = Request();
    request.gameplay = false;
    refused(request, head, Use::NotGameplay, "a cinematic: the game's own heading");
    request = Request();
    request.nowUs = head.stampUs + head_relative::kMaxSampleAgeUs + 1;
    refused(request, head, Use::Stale, "a head sample older than 100 ms: the game's own heading");
    request = Request();
    request.generation = 4;
    refused(request, head, Use::OtherGeneration, "a head sample from another load: the game's own heading");
    request.generation = 0;
    refused(request, head, Use::OtherGeneration, "a hook from another load (generation 0): the game's own heading");
    request = Request();
    request.nowUs = head.stampUs - 1;
    refused(request, head, Use::FromTheFuture, "a head sample stamped after now: the game's own heading");
    refused(Request(), HeadSample{}, Use::NoSample, "nothing published yet: the game's own heading");
    HeadSample notLive = head;
    notLive.valid = false;
    refused(Request(), notLive, Use::NoSample, "the head view was not live in that frame: the game's own heading");
    HeadSample nan = head;
    nan.forward[0] = std::numeric_limits<float>::quiet_NaN();
    refused(Request(), nan, Use::NoHeading, "a non-finite head: the game's own heading");
    refused(Request(), Head(10.0f, 88.0f), Use::NoHeading, "looking straight up before any heading was known: the game's own");
    {
        HeadingHold fresh;
        float out = 123.0f;
        Check(TrackerField(Request(), head, fresh, std::numeric_limits<float>::infinity(), out) == Use::NoHeading &&
                  out == 123.0f,
              "a non-finite native field is left alone");
    }

    // Looking straight up or down keeps the last heading the head had.
    HeadingHold held;
    float out = 0.0f;
    Check(TrackerField(Request(), Head(200.0f, -20.0f), held, native, out) == Use::Ok &&
              Near(out, FieldFromYaw(200.0f * kDegToRad)),
          "a head pitched 20 deg down: its heading");
    Check(TrackerField(Request(), Head(75.0f, -89.0f), held, native, out) == Use::Ok &&
              Near(out, FieldFromYaw(200.0f * kDegToRad)),
          "then straight down: the last heading is held, not the noise of a vertical forward");
    // Head pitch and roll never reach the tracker: the same heading at any pitch below the limit.
    bool flat = true;
    for (float pitch : {-70.0f, -30.0f, 0.0f, 45.0f, 79.0f})
    {
        HeadingHold each;
        float value = 0.0f;
        flat = flat && TrackerField(Request(), Head(300.0f, pitch), each, native, value) == Use::Ok &&
               Near(value, FieldFromYaw(300.0f * kDegToRad), 1.0e-3f);
    }
    Check(flat, "head pitch does not change the tracker's heading");

    // What is written is always in the game's own range for the field, whatever the head and the offset are.
    bool inRange = true;
    for (float headYaw = -720.0f; headYaw <= 720.0f; headYaw += 37.0f)
        for (float offset : {-180.0f, 0.0f, 180.0f})
        {
            HeadingHold each;
            float value = 0.0f;
            inRange = inRange && TrackerField(Request(1, offset), Head(headYaw), each, native, value) == Use::Ok &&
                      value >= kHalfPi && value < kHalfPi + kTwoPi;
        }
    Check(inRange, "the written field stays in [pi/2, 5 pi/2), the range the game's own yaw gives it");
}
} // namespace

int main()
{
    TestSites();
    TestPlacement();
    TestAcceptance();
    TestRefusals();
    std::printf("reach head relative tests: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
