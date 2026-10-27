#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
namespace authored_grab {
using Key=std::array<uint32_t,4>; // title, generation, unit, weapon
using Point=std::array<float,3>; // primary-controller local OpenXR metres
inline bool Valid(Point p){float n=0;for(float v:p){if(!std::isfinite(v))return false;n+=v*v;}return n<=1.0f;}
inline float Distance2(Point a,Point b){float n=0;for(int i=0;i<3;++i)n+=(a[i]-b[i])*(a[i]-b[i]);return n;}
// Continuously relearn after motion; never permanently cache an equip/reload pose.
struct Learner {
    Key key{};Point anchor{};uint64_t start{},quiet{},last{};bool active=false;
    bool Observe(Key k,Point p,uint64_t now){
        if(!Valid(p)){active=false;return false;}
        if(!active||key!=k||now<last||now-last>250){key=k;anchor=p;start=quiet=last=now;active=true;return false;}
        last=now;
        if(Distance2(anchor,p)>.0009f){anchor=p;quiet=now;}
        return now-start>=1500&&now-quiet>=500;
    }
};
struct Publication {
    std::atomic<uint64_t> sequence{0},at{0};std::atomic<uint32_t> key[4]{};std::atomic<float> point[3]{};
    void Publish(Key k,Point p,uint64_t now){
        auto seq=sequence.load();if((seq&1)||!sequence.compare_exchange_strong(seq,seq+1))return;
        for(int i=0;i<4;++i)key[i].store(k[i]);
        for(int i=0;i<3;++i)point[i].store(p[i]);
        at.store(now);sequence.store(seq+2);
    }
    bool Read(Key k,uint64_t now,Point& p)const{
        const auto seq=sequence.load();if(!seq||(seq&1))return false;
        const auto time=at.load();bool matches=true;
        for(int i=0;i<4;++i)matches=matches&&key[i].load()==k[i];
        for(int i=0;i<3;++i)p[i]=point[i].load();
        return sequence.load()==seq&&matches&&time&&now>=time&&now-time<=100&&Valid(p);
    }
};
inline Publication publication;
inline std::atomic<uint64_t> hits{0},misses{0},fallbacks{0};
inline bool Hit(Point centre,Point sample){return Valid(centre)&&Valid(sample)&&Distance2(centre,sample)<.09f*.09f;}
}
