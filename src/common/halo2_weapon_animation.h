#pragma once
#include "halo2_render_logic.h"
#include "weapon_animation_motion.h"

inline bool Halo2AnimationCameraPose(const Halo2CameraBasis& camera,weapon_animation::Pose& out)
{
    if(!Halo2BuildControllerHandMountBasis(camera,out.rotation))return false;
    std::memcpy(out.translation,camera.position,sizeof(out.translation));
    return weapon_animation::Valid(out);
}
inline bool Halo2ObserveWeaponAnimation(weapon_animation::Reference& reference,
    std::array<uint64_t,4> key,const float* nativeGun,const float* nativeWrist,const Halo2CameraBasis& camera,
    uint64_t now,float unitsPerMetre,weapon_animation::Pose& delta)
{
    Halo2FirstPersonTransform gun{},wrist{};weapon_animation::Pose root{},world{};
    if(!Halo2ReadFirstPersonTransform(nativeGun,gun)||!Halo2ReadFirstPersonTransform(nativeWrist,wrist)||!Halo2AnimationCameraPose(camera,root)) {
        reference={};return false;
    }
    std::memcpy(world.rotation,gun.rotation,sizeof(world.rotation));
    // Existing packet ownership preserves the gun's translation relative to
    // its wrist. Restore only the removed wrist translation and gun rotation.
    std::memcpy(world.translation,wrist.translation,sizeof(world.translation));
    if(!weapon_animation::Valid(world)){reference={};return false;}
    return reference.Update(key,weapon_animation::Relative(root,world),now,unitsPerMetre,delta);
}
inline bool Halo2ApplyWeaponAnimation(Halo2CameraBasis& carrier,const weapon_animation::Pose& delta,float scale)
{
    weapon_animation::Pose pose{};
    if(!Halo2AnimationCameraPose(carrier,pose)||!weapon_animation::Valid(delta)||!std::isfinite(scale)||scale<=0)return false;
    auto scaled=delta;for(float& v:scaled.translation)v*=scale;
    const auto result=weapon_animation::Compose(pose,scaled);
    if(!weapon_animation::Valid(result))return false;
    std::memcpy(carrier.forward,result.rotation,sizeof(carrier.forward));
    std::memcpy(carrier.up,result.rotation+6,sizeof(carrier.up));
    std::memcpy(carrier.position,result.translation,sizeof(carrier.position));
    return true;
}
