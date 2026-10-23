#pragma once
#include "odst_modded_fp_logic.h"

// New Covenant's ODST cache and native probe agree: 31 body nodes, followed
// by the held model and one final camera node. Never infer this from count alone.
inline bool ResolveOdstEliteLayout(uint32_t checksum,std::span<const int32_t> map,
    int count,OdstMappedFpLayout& result)
{
    if(checksum!=0x10000E0Bu || map.size()!=31 || count<33 || count>64)return false;
    for(int i=0;i<31;++i)if(map[i]!=i)return false;
    OdstMappedFpLayout candidate{};
    if(!ResolveOdstMappedFpLayout(checksum,map,count,candidate))return false;
    candidate.skeleton.cameraControl=count-1;
    for(int i=31;i<count-1;++i)
        candidate.skeleton.rightHandAndWeaponDescendants|=uint64_t{1}<<i;
    result=candidate;
    return true;
}
