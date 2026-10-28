#pragma once
#include "runtime_types.h"
#include <cstdint>
namespace weapon_alignment {
struct LegacyVisualDefaults { float yaw{},pitch{},roll{}; };
inline LegacyVisualDefaults LegacyWeaponVisualDefaults(GameTitle title,uint64_t model,bool classic=false) noexcept {
    if(title==GameTitle::Halo2) {
        LegacyVisualDefaults d{};
        switch(model) {
        case 0xA2DBA1A2B3208E57ull: if(!classic)d.pitch=1.f;break; // Magnum
        case 0x67EC65EB1DE2A459ull: if(classic)d.yaw=1.f;break; // SMG
        case 0x1B55D3C632C37F00ull: d.yaw=classic?1.f:.5f;break; // Plasma Rifle
        case 0xED4D20677ED1FEE9ull: if(classic)d.yaw=1.f;break; // Brute Plasma Rifle
        case 0x6D99DEAEE0C79FD5ull: d.yaw=classic?1.5f:1.f;break; // Plasma Pistol
        case 0x1295A15AB1C5709Cull: if(classic)d.yaw=1.5f;break; // Battle Rifle
        case 0xACBAC7877B648512ull: d.pitch=classic?10.f:0.f;d.roll=90.f;break; // Brute Shot
        default: break;
        }
        return d;
    }
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
