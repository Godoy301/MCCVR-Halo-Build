#pragma once
#include <algorithm>
#include <cmath>

namespace weapon_alignment
{
// Shared gun offsets move both grips; the left-only slider remains additive.
inline float Forward(bool left,float gun,float leftExtra) noexcept
{ return std::clamp(gun,-.3f,.5f)+(left?std::clamp(leftExtra,-.15f,.30f):0.f); }
inline float Right(bool left,float value) noexcept
{ return (left?-1.f:1.f)*std::clamp(value,-.3f,.3f); }
inline float Up(float value) noexcept { return std::clamp(value,-.3f,.3f); }
inline constexpr float kH2ClassicVisualPitch=-9.5f;
inline float H3SecondaryYaw(unsigned slot,unsigned checksum) noexcept
{ return slot==1 && (checksum==0x171C0B10u || checksum==0x16120C11u) ? 30.f : 0.f; }
inline bool RotateYaw(const float frame[9],const float pivot[3],float degrees,
    float basis[9],float position[3]) noexcept
{
    if (!std::isfinite(degrees)) return false;
    for(int i=0;i<9;++i) if(!std::isfinite(frame[i])||!std::isfinite(basis[i]))return false;
    for(int i=0;i<3;++i) if(!std::isfinite(pivot[i])||!std::isfinite(position[i]))return false;
    const float angle=-degrees*.01745329252f,c=std::cos(angle),s=std::sin(angle);
    const auto rotate=[&](const float* v,float* out) {
        float local[3]{};
        for(int col=0;col<3;++col)for(int row=0;row<3;++row)local[col]+=frame[col*3+row]*v[row];
        const float x=c*local[0]-s*local[1],y=s*local[0]+c*local[1];
        for(int row=0;row<3;++row)out[row]=frame[row]*x+frame[3+row]*y+frame[6+row]*local[2];
    };
    float result[9]{},offset[3]{},p[3]{};
    for(int col=0;col<3;++col)rotate(basis+col*3,result+col*3);
    for(int i=0;i<3;++i)offset[i]=position[i]-pivot[i];
    rotate(offset,p);
    for(int i=0;i<9;++i) { if(!std::isfinite(result[i]))return false; }
    for(int i=0;i<3;++i) { p[i]+=pivot[i];if(!std::isfinite(p[i]))return false; }
    for(int i=0;i<9;++i)basis[i]=result[i];
    for(int i=0;i<3;++i)position[i]=p[i];
    return true;
}
}
