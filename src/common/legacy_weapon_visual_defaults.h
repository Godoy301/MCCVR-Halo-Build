#pragma once
#include "runtime_types.h"
#include <cstdint>
namespace weapon_alignment {
struct LegacyVisualDefaults { float yaw{},pitch{},roll{}; };
inline LegacyVisualDefaults LegacyWeaponVisualDefaults(GameTitle title,uint64_t model,bool classic=false) noexcept {
    (void)classic;
    if(title!=GameTitle::Halo3&&title!=GameTitle::Halo3ODST)return {};
    LegacyVisualDefaults d{0.f,1.f,0.f};
    switch(model) {
    case 0x15130F16ull: // H3 Spiker
    case 0x11140D13ull: d.roll=10.f;break; // ODST Spiker
    case 0x10190A19ull: d.pitch=8.f;break; // Needler
    case 0x1C080C11ull: d.pitch=-25.f;break; // Energy Sword
    case 0x16120C11ull: // H3 Plasma Rifle / ODST Brute Plasma Rifle
    case 0x191D0A13ull: d.yaw=1.f;break; // ODST Plasma Rifle
    case 0x1A100F1Eull: d.pitch=0.f;break; // Automag
    case 0x150A000Cull: d.yaw=-1.f;break; // Plasma Pistol
    default: break;
    }
    return d;
}
}
