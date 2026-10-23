#pragma once
#include "odst_bringup_logic.h"
#include "fp_source_ownership.h"
#include <array>
#include <span>

// Own-title evidence: ODST-MODDED-WEAPONS-MAP-EVIDENCE.json and the official
// recon/ONI FP exports. Checksums identify body rigs, never weapon names.
struct OdstMappedFpLayout
{
    OdstFpSkeletonLayout skeleton{};
    uint64_t bodySources = 0;
    uint32_t checksum = 0;
    int bodyCount = 0;
    std::array<int32_t,37> remap{};
    bool operator==(const OdstMappedFpLayout&) const = default;
};

inline bool ResolveOdstMappedFpLayout(uint32_t checksum,
    std::span<const int32_t> remap, int sourceCount, OdstMappedFpLayout& out)
{
    const bool human = remap.size()==37 &&
        (checksum==0x1114091Cu || checksum==0x18080211u || checksum==0x1B120B14u);
    const bool elite = remap.size()==31 && checksum==0x10000E0Bu;
    if ((!human && !elite) || sourceCount<=int(remap.size()) || sourceCount>64)
        return false;
    OdstMappedFpLayout result{};
    result.checksum=checksum;
    result.bodyCount=int(remap.size());
    // Both the official ODST graphs and the supplied mod place camera_control
    // last. Do not admit a body mapping that consumes that reserved source.
    result.skeleton.cameraControl=sourceCount-1;
    for (size_t i=0;i<remap.size();++i)
    {
        const int source=remap[i];
        if (source<0 || source>=result.skeleton.cameraControl ||
            (result.bodySources & (uint64_t{1}<<source))) return false;
        result.bodySources|=uint64_t{1}<<source;
        result.remap[i]=source;
    }
    auto& s=result.skeleton;
    s.leftShoulder=remap[1]; s.rightShoulder=remap[2];
    s.leftElbow=remap[3]; s.rightElbow=remap[4];
    s.leftWrist=remap[5]; s.rightWrist=remap[6];
    s.leftHandDescendants=uint64_t{1}<<s.leftWrist;
    s.rightHandAndWeaponDescendants=uint64_t{1}<<s.rightWrist;
    const int digits=human?5:4;
    for (int joint=0;joint<3;++joint)
        for (int digit=0;digit<digits;++digit)
        {
            s.leftHandDescendants|=uint64_t{1}<<remap[7+joint*digits*2+digit];
            s.rightHandAndWeaponDescendants|=uint64_t{1}<<remap[7+joint*digits*2+digits+digit];
        }
    out=result;
    return true;
}

inline bool ResolveOdstHeldSources(const OdstMappedFpLayout& body,
    std::span<const int32_t> remap, int sourceCount, FpHeldSources& out)
{
    if (!body.bodyCount || sourceCount<=0 || sourceCount>64 ||
        body.skeleton.cameraControl!=sourceCount-1 || remap.empty() || remap.size()>64)
        return false;
    FpHeldSources held{};
    held.explicitMap=true;
    for (int source:remap)
    {
        if (source==-1) continue;
        if (source<0 || source>=sourceCount-1) return false;
        const uint64_t bit=uint64_t{1}<<source;
        if ((body.bodySources | held.bits[0]) & bit) return false;
        held.bits[0]|=bit;
    }
    if (!held.bits[0]) return false;
    out=held;
    return true;
}

struct OdstFpLayoutKey
{
    uint32_t generation=0;
    int view=0,id=0,slot=0,count=0;
    bool operator==(const OdstFpLayoutKey&) const = default;
};
struct OdstFpLayoutEntry
{
    bool valid=false;
    OdstFpLayoutKey key{};
    uint16_t bodyTag=0xFFFF;
    OdstMappedFpLayout layout{};
};

// Learn only at the native body palette; freeze the learned layout for the
// next stereo pair. No eye may adopt a layout learned by the other eye.
struct OdstFpLayoutCache
{
    std::array<OdstFpLayoutEntry,4> learned{},frozen{};
    uint64_t serial=0,epoch=0;
    uint32_t generation=0;
    bool Begin(uint64_t nextSerial,uint64_t nextEpoch,uint32_t nextGeneration)
    {
        if (!nextSerial || !nextEpoch || !nextGeneration) { *this={}; return false; }
        if (epoch!=nextEpoch || generation!=nextGeneration || nextSerial<serial)
            *this={};
        if (serial!=nextSerial)
        {
            frozen=learned;
            serial=nextSerial; epoch=nextEpoch; generation=nextGeneration;
        }
        return true;
    }
    OdstFpLayoutEntry Find(const OdstFpLayoutKey& key) const
    {
        for (const auto& e:frozen) if (e.valid && e.key==key) return e;
        return {};
    }
    void Reject(const OdstFpLayoutKey& key)
    {
        for (auto& e:learned) if (e.valid && e.key==key) e={};
    }
    void Learn(const OdstFpLayoutKey& key,uint16_t tag,const OdstMappedFpLayout& layout)
    {
        if (!serial || key.generation!=generation || !layout.bodyCount) return;
        OdstFpLayoutEntry* target=nullptr;
        for (auto& e:learned)
        {
            if (e.valid && e.key==key) {target=&e;break;}
            if (!e.valid && !target) target=&e;
        }
        if (!target) target=&learned[0];
        *target={true,key,tag,layout};
    }
};
