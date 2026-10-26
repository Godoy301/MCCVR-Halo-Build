#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace weapon_animation
{
// Rigid animation poses only. Adapters remove their own native camera/root
// before observation; a world-space pose must never become a reference.
struct Pose {
    float rotation[9]{1,0,0,0,1,0,0,0,1};
    float translation[3]{};
};
inline bool Valid(const Pose& p)
{
    for(float v:p.rotation)if(!std::isfinite(v))return false;
    for(float v:p.translation)if(!std::isfinite(v))return false;
    for(int a=0;a<3;++a)for(int b=a;b<3;++b) {
        float dot=0;for(int r=0;r<3;++r)dot+=p.rotation[a*3+r]*p.rotation[b*3+r];
        if(std::fabs(dot-(a==b?1.f:0.f))>.03f)return false;
    }
    const auto* r=p.rotation;
    return r[0]*(r[4]*r[8]-r[7]*r[5])-r[3]*(r[1]*r[8]-r[7]*r[2])+r[6]*(r[1]*r[5]-r[4]*r[2])>.95f;
}
inline Pose Relative(const Pose& reference,const Pose& current)
{
    Pose out;
    for(int r=0;r<3;++r) {
        out.translation[r]=0;
        for(int k=0;k<3;++k)out.translation[r]+=reference.rotation[r*3+k]*(current.translation[k]-reference.translation[k]);
        for(int c=0;c<3;++c) {
            out.rotation[c*3+r]=0;
            for(int k=0;k<3;++k)out.rotation[c*3+r]+=reference.rotation[r*3+k]*current.rotation[c*3+k];
        }
    }
    return out;
}
inline Pose Compose(const Pose& a,const Pose& b)
{
    Pose out;
    for(int r=0;r<3;++r) {
        out.translation[r]=a.translation[r];
        for(int k=0;k<3;++k)out.translation[r]+=a.rotation[k*3+r]*b.translation[k];
        for(int c=0;c<3;++c) {
            out.rotation[c*3+r]=0;
            for(int k=0;k<3;++k)out.rotation[c*3+r]+=a.rotation[k*3+r]*b.rotation[c*3+k];
        }
    }
    return out;
}
struct Reference
{
    std::array<uint64_t,4> key{};
    Pose anchor{},reference{};
    uint64_t started{},stableSince{},last{};
    bool observed=false,ready=false;
    // A bounded settling heuristic, not a native idle-animation detector.
    // Keep controller alignment while drawing/equipping; freeze the reference
    // only after the initial delay AND a quiet window. Never chase later recoil
    // or reload motion with a moving average.
    bool Update(std::array<uint64_t,4> identity,const Pose& local,uint64_t now,
                float unitsPerMetre,Pose& delta)
    {
        delta={};
        if(!Valid(local)||!std::isfinite(unitsPerMetre)||unitsPerMetre<=0) {
            observed=ready=false;return false;
        }
        if(!observed||key!=identity||now<last||now-last>500) {
            *this={};observed=true;key=identity;anchor=local;
            started=stableSince=last=now;return false;
        }
        last=now;
        if(!ready) {
            const Pose change=Relative(anchor,local);
            const float trace=change.rotation[0]+change.rotation[4]+change.rotation[8];
            if(trace<2.997259f) {
                anchor=local;stableSince=now;
            }
            if(now-started<1500||now-stableSince<500)return false;
            reference=local;ready=true;
        }
        delta=Relative(reference,local);
        // Controllers own wrist position. Retain pitch/yaw/roll animation only;
        // camera/stance translations must neither delay settling nor move hands.
        for(float& value:delta.translation)value=0;
        return Valid(delta);
    }
};
}
