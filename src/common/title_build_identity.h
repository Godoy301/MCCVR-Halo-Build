#pragma once
// WHICH BUILD OF A GAME DLL IS THIS? One line in the log per title, the first thing to read in a report.
//
// Most of this mod's cores install only on the exact build of each game DLL they were made for (a PE timestamp
// and an image size, checked in each title's own gate). That is deliberate: a hook placed by a rule made for
// another build is a wrong write. But when the build is NOT that one, the gates differ in how they say so, and
// two of them said nothing at all (Reach returned without a line when the image size differed; CE printed
// "cold install waiting" every two seconds for ever). A player whose game had been updated then had a log that
// read as healthy and a game with no VR, and someone had to work out from which lines were MISSING what had
// happened.
//
// So the title adapter states it once, where it first sees a title's DLL: this is the build the mod was made
// for, or it is not (and what it found and expected). Nothing is decided here: every gate still makes its own
// decision. A table of this build's values beside those gates' own constants; tests/title_build_identity_tests.cpp
// holds the two together.
//
// The Steam and the Microsoft Store / Game Pass copies of these DLLs are ONE build: on 2026-10-08, MCC
// 2025.08.16.178512.1-Release, all seven were identical in every section (only the Authenticode certificate
// table and the PE checksum differ; tools/compare-store-build.py). The timestamp and the size are the same on
// both, so this check and every gate behave the same on both.
#include "runtime_types.h"
#include <cstdint>

namespace title_build
{
inline constexpr char kKnownMccBuild[] = "2025.08.16.178512.1-Release";
struct Identity
{
    GameTitle title;
    uint32_t peTimestamp;
    uint32_t sizeOfImage;
};
// ADDR-HYGIENE: structural -- identities to COMPARE and print, never addresses; nothing is read or written by them.
inline constexpr Identity kKnown[]{
    {GameTitle::HaloCE, 0x68A0E4CF, 0x03D82000},     // halo1.dll
    {GameTitle::Halo2, 0x68A0F0F2, 0x02A38000},      // halo2.dll
    {GameTitle::Halo3, 0x68A0EDFC, 0x04768000},      // halo3.dll (its core has no build gate; said for the record)
    {GameTitle::Halo3ODST, 0x68A0F232, 0x04797000},  // halo3odst.dll
    {GameTitle::HaloReach, 0x68A0EFE1, 0x04EDA000},  // haloreach.dll
    {GameTitle::Halo4, 0x68A0E7BF, 0x04A3F000},      // halo4.dll
};
inline const Identity* Known(GameTitle title) noexcept
{
    for (const Identity& known : kKnown)
        if (known.title == title)
            return &known;
    return nullptr;
}
enum class Match : uint8_t
{
    Known,      // the build this mod was made for
    Different,  // another build: MCC was updated, or this is a build nobody has looked at
    NoRecord,   // a title with no recorded build
};
inline Match Compare(GameTitle title, uint32_t peTimestamp, uint32_t sizeOfImage) noexcept
{
    const Identity* const known = Known(title);
    if (!known)
        return Match::NoRecord;
    return known->peTimestamp == peTimestamp && known->sizeOfImage == sizeOfImage ? Match::Known : Match::Different;
}
// Whether a title's VR core is gated on this exact build (every one but Halo 3's, which resolves by signature
// alone). Only for the wording of the line.
inline bool CoreIsBuildGated(GameTitle title) noexcept
{
    return title != GameTitle::Halo3 && Known(title) != nullptr;
}
} // namespace title_build
