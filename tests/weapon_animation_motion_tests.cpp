#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../src/common/weapon_animation_motion.h"
#include "../src/common/halo2_weapon_animation.h"
#include <cassert>
#include <limits>
using weapon_animation::Pose;
static bool near(float a,float b){return std::fabs(a-b)<.00001f;}
static void same(const Pose& a,const Pose& b)
{
    for(int i=0;i<9;++i)assert(near(a.rotation[i],b.rotation[i]));
    for(int i=0;i<3;++i)assert(near(a.translation[i],b.translation[i]));
}
int main()
{
    weapon_animation::Reference reference;
    std::array<uint64_t,4> key{1,2,3,4};Pose equip{},idle{},delta{};
    equip.translation[2]=.4f;equip.rotation[0]=0;equip.rotation[1]=1;
    equip.rotation[3]=-1;equip.rotation[4]=0;
    // A quiet initial equip plateau must not immediately become the baseline.
    for(uint64_t t=0;t<1000;t+=100)assert(!reference.Update(key,equip,t,.33f,delta));
    assert(!reference.Update(key,idle,1000,.33f,delta));
    for(uint64_t t=1100;t<1500;t+=100)assert(!reference.Update(key,idle,t,.33f,delta));
    assert(reference.Update(key,idle,1500,.33f,delta));same(delta,Pose{});
    Pose firing=idle;firing.translation[2]=.01f;
    assert(reference.Update(key,firing,1600,.33f,delta));assert(near(delta.translation[2],.01f));
    // Recoil/reload never replace an established neutral reference.
    assert(reference.Update(key,equip,1700,.33f,delta));
    assert(reference.Update(key,idle,1800,.33f,delta));same(delta,Pose{});
    // Regrab and graphics/title identity changes settle independently.
    for(int change=0;change<4;++change) {
        ++key[change];assert(!reference.Update(key,equip,1900+change*2000,.33f,delta));
        for(uint64_t t=2000+change*2000;t<=uint64_t(3500+change*2000);t+=100)
            reference.Update(key,idle,t,.33f,delta);
        assert(reference.ready);same(delta,Pose{});
    }
    auto invalid=idle;invalid.translation[0]=std::numeric_limits<float>::quiet_NaN();
    assert(!reference.Update(key,invalid,10000,.33f,delta));assert(!reference.ready);
    // Multiple eyes at one timestamp cannot complete the settling window.
    for(int i=0;i<2000;++i)assert(!reference.Update(key,idle,10100,.33f,delta));

    Halo2CameraBasis camera{{2,3,4},{1,0,0},{0,0,1}};
    Pose root{},local{};local.translation[0]=.3f;
    assert(Halo2AnimationCameraPose(camera,root));
    auto world=weapon_animation::Compose(root,local);
    float packet[13]{};packet[0]=1;
    const auto write=[&](){for(int i=0;i<9;++i)packet[i+1]=world.rotation[i];for(int i=0;i<3;++i)packet[i+10]=world.translation[i];};
    write();reference={};
    for(uint64_t t=0;t<=1600;t+=100)Halo2ObserveWeaponAnimation(reference,key,packet,packet,camera,t,.33f,delta);
    assert(reference.ready);same(delta,Pose{});
    // Moving/turning the camera and its native gun together is not animation.
    camera.position[0]+=10;camera.forward[0]=0;camera.forward[1]=1;
    assert(Halo2AnimationCameraPose(camera,root));world=weapon_animation::Compose(root,local);write();
    assert(Halo2ObserveWeaponAnimation(reference,key,packet,packet,camera,1700,.33f,delta));same(delta,Pose{});
    local.translation[2]+=.01f;world=weapon_animation::Compose(root,local);write();
    assert(Halo2ObserveWeaponAnimation(reference,key,packet,packet,camera,1800,.33f,delta));
    assert(near(delta.translation[2],.01f));
    // Gun-only motion is already preserved by packet ownership; it must not
    // also become an extra wrist translation.
    float wrist[13];std::memcpy(wrist,packet,sizeof(wrist));
    packet[10]+=.1f;
    Pose independent{};
    assert(Halo2ObserveWeaponAnimation(reference,key,packet,wrist,camera,1850,.33f,independent));
    same(independent,delta);
    Halo2CameraBasis controller{{4,5,6},{1,0,0},{0,0,1}};
    assert(Halo2ApplyWeaponAnimation(controller,delta,1));assert(near(controller.position[2],6.01f));
}
