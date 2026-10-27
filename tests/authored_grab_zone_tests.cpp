#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../src/common/authored_grab_zone.h"
#include <cassert>
#include <limits>
int main(){
    using namespace authored_grab;
    const Key pistol{1,2,3,4},rifle{1,2,3,5};
    const Point rear{-.025f,0,.01f},front{0,0,-.35f};
    assert(Hit(rear,{0,0,0}));assert(!Hit(front,{0,0,0}));assert(Hit(front,{.02f,0,-.34f}));
    assert(!Hit(rear,{0,0,-.3f}));assert(!Hit(rear,{std::numeric_limits<float>::quiet_NaN(),0,0}));
    Learner learner;assert(!learner.Observe(pistol,rear,100));
    for(uint64_t t=200;t<1600;t+=100)assert(!learner.Observe(pistol,rear,t));
    assert(learner.Observe(pistol,rear,1600));
    assert(!learner.Observe(pistol,front,1700)); // reload movement revokes readiness
    assert(!learner.Observe(pistol,rear,1800));
    for(uint64_t t=1900;t<2300;t+=100)assert(!learner.Observe(pistol,rear,t));
    assert(learner.Observe(pistol,rear,2300)); // never freezes the reload pose
    assert(!learner.Observe(rifle,front,2400));
    Publication pub;Point out{};pub.Publish(pistol,rear,2000);
    assert(pub.Read(pistol,2050,out));assert(out==rear);
    assert(!pub.Read(rifle,2050,out));assert(!pub.Read(pistol,2101,out));assert(!pub.Read(pistol,1999,out));
    for(int i=0;i<20;++i){assert(!Hit(rear,{0,0,-.6f}));assert(pub.Read(pistol,2050,out));}
    assert(Hit(out,{0,0,0})); // an outside attempt never poisons the next attempt
    assert(pub.Owns(pistol)&&!pub.Owns(rifle));
    pub.Publish(pistol,front,2200,false); // attachment/settling heartbeat preserves learned zone
    assert(pub.Read(pistol,2250,out)&&out==rear);
    pub.Publish(rifle,front,2300,false);
    assert(pub.Owns(rifle)&&!pub.Read(rifle,2300,out)); // pending never inherits old gun
    pub.Publish(rifle,front,2400,true);assert(pub.Read(rifle,2400,out)&&out==front);
    assert(Short(rear)&&!Short(front));assert(ShortHit({-.12f,0,0}));
    assert(!ShortHit({0,0,-.3f}));assert(!ShortHit({.3f,0,0}));
    assert(Authority(.10f)==0&&Authority(.16f)==0&&Authority(.30f)==1);
    assert(Authority(.23f)>.49f&&Authority(.23f)<.51f);
    pub.Publish(pistol,rear,0);assert(!pub.Read(pistol,2050,out));
}
