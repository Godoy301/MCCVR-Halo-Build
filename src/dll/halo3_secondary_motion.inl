// The authored secondary wrist carries idle/fire/reload motion. Cancelling
// its current pose against the controller also cancels those animations.
// Keep a per-equipped-weapon reference and apply only its subsequent delta.
struct H3SecondaryMotion
{
    uint32_t generation{},unit=UINT32_MAX,weapon=UINT32_MAX;
    int count{},wrist=-1;
    uint64_t at{};
    BoneMatrix inverseReference{},delta{};
    bool valid=false;
};
thread_local H3SecondaryMotion g_h3SecondaryMotion;
std::atomic<uint64_t> g_h3SecondaryMotionSamples{},g_h3SecondaryMotionResets{},g_h3SecondaryMotionRefused{};
bool H3MotionFinite(const BoneMatrix& value)
{
    if(!std::isfinite(value.scale)||value.scale<=.001f)return false;
    for(float x:value.rotation)if(!std::isfinite(x))return false;
    for(float x:value.translation)if(!std::isfinite(x))return false;
    return true;
}
void CaptureH3SecondaryMotion(const FpInterpolationContext& context,const BoneMatrix* authored)
{
    if(context.slot!=1||TitleAdapter_GetActiveTitle()!=GameTitle::Halo3)return;
    auto& state=g_h3SecondaryMotion;
    const uint64_t now=GetTickCount64();
    if(!context.valid||context.player!=0||context.muzzleWeapon==UINT32_MAX||
        context.muzzleUnit==UINT32_MAX||context.count<=0||context.count>64||!authored||context.lWrist<0||context.lWrist>=context.count) {
        state.valid=false;++g_h3SecondaryMotionRefused;return;
    }
    const auto& current=authored[context.lWrist];
    if(!H3MotionFinite(current)) {state.valid=false;++g_h3SecondaryMotionRefused;return;}
    if(!state.valid||state.generation!=context.generation||state.unit!=context.muzzleUnit||
        state.weapon!=context.muzzleWeapon||state.count!=context.count||state.wrist!=context.lWrist||now-state.at>500) {
        state={};
        if(!InvertBoneMatrix(current,state.inverseReference)){++g_h3SecondaryMotionRefused;return;}
        state.generation=context.generation;state.unit=context.muzzleUnit;state.weapon=context.muzzleWeapon;
        state.count=context.count;state.wrist=context.lWrist;state.valid=true;
        ++g_h3SecondaryMotionResets;
    }
    state.at=now;
    if(!ComposeBoneMatrices(state.inverseReference,current,state.delta)||!H3MotionFinite(state.delta)) {
        state.valid=false;++g_h3SecondaryMotionRefused;return;
    }
    // Scale remains a player setting, rather than an animated wrist scale.
    state.delta.scale=1.f;
    ++g_h3SecondaryMotionSamples;
}
void ApplyH3SecondaryMotion(BoneMatrix& visual,BoneMatrix& neutral)
{
    const auto& state=g_h3SecondaryMotion;
    const auto& context=g_fpInterpolationContexts[1];
    if(TitleAdapter_GetActiveTitle()!=GameTitle::Halo3||!state.valid||!context.valid||
        context.generation!=state.generation||context.muzzleUnit!=state.unit||
        context.muzzleWeapon!=state.weapon||GetTickCount64()-state.at>100)return;
    BoneMatrix moved{},neutralMoved{};
    if(ComposeBoneMatrices(visual,state.delta,moved)&&ComposeBoneMatrices(neutral,state.delta,neutralMoved)&&
        H3MotionFinite(moved)&&H3MotionFinite(neutralMoved)) {visual=moved;neutral=neutralMoved;}
}

// Model-specific visual correction is shared with both native effect paths.
// The mask comes from the actual render-model remap, never a guessed subtree.
struct H3SecondaryEffectBinding
{
    std::atomic<uint32_t> sequence{0},generation{0},unit{UINT32_MAX},weapon{UINT32_MAX},checksum{0};
    std::atomic<uint64_t> mask{0};
};
H3SecondaryEffectBinding g_h3SecondaryEffectBinding;
std::atomic<uint64_t> g_h3SecondaryEffectApplied{},g_h3SecondaryEffectRefused{};
inline constexpr float kH3SecondaryLateralMetres=.06f;
bool H3CorrectSecondaryMatrix(const BoneMatrix& frame,float yaw,BoneMatrix& value)
{
    if(!weapon_alignment::RotateYaw(frame.rotation,frame.translation,yaw,value.rotation,value.translation))return false;
    const float lateral=kH3SecondaryLateralMetres*g_worldScale.load();
    for(int axis=0;axis<3;++axis)value.translation[axis]+=frame.rotation[3+axis]*lateral;
    return H3MotionFinite(value);
}
void ApplyH3SecondaryEffects(BoneMatrix* bones,int count,const BoneMatrix& frame,const BoneMatrix* root,float meshScale)
{
    if(!bones||count<=0||count>64||g_config.left_handed||TitleAdapter_GetActiveTitle()!=GameTitle::Halo3)return;
    const auto& context=g_fpInterpolationContexts[1];auto& binding=g_h3SecondaryEffectBinding;
    const auto sequence=binding.sequence.load(std::memory_order_acquire);
    if(!sequence||(sequence&1)||!context.valid||context.muzzleWeapon==UINT32_MAX)return;
    const auto generation=binding.generation.load(),unit=binding.unit.load(),weapon=binding.weapon.load(),checksum=binding.checksum.load();
    const uint64_t mask=binding.mask.load();
    if(sequence!=binding.sequence.load(std::memory_order_acquire)||generation!=context.generation||
        unit!=context.muzzleUnit||weapon!=context.muzzleWeapon)return;
    const float yaw=weapon_alignment::H3SecondaryYaw(1,checksum);
    if(!yaw||!mask)return;
    if(context.count!=count||context.lWrist<0||context.lWrist>=count||!std::isfinite(meshScale)||meshScale<=0)return;
    BoneMatrix nativeRoot{};
    if(root)nativeRoot=*root;
    else {
        if(!LoadCachedRenderRoot(context.player,nativeRoot))return;
        float camera[9];
        if(!g_camValid.load()||!LoadCameraBasis(camera))return;
        memcpy(nativeRoot.rotation,camera,sizeof(camera));
        nativeRoot.translation[0]=g_camX.load();nativeRoot.translation[1]=g_camY.load();nativeRoot.translation[2]=g_camZ.load();
    }
    BoneMatrix inverse{},anchor{},inverseAnchor{},carry{},candidate[64];
    const BoneMatrix* original=g_fpUnmodifiedInterpolations[1];
    if(!ComposeBoneMatrices(nativeRoot,original[context.lWrist],anchor)||!InvertBoneMatrix(anchor,inverseAnchor)||
        !ComposeBoneMatrices(frame,inverseAnchor,carry)||(root&&!InvertBoneMatrix(nativeRoot,inverse))) {
        ++g_h3SecondaryEffectRefused;return;
    }
    for(int i=0;i<count;++i) {
        candidate[i]=bones[i];if(!(mask&(uint64_t{1}<<i)))continue;
        BoneMatrix authored{},world{};
        if(!ComposeBoneMatrices(nativeRoot,original[i],authored)||!ComposeBoneMatrices(carry,authored,world)) {
            ++g_h3SecondaryEffectRefused;return;
        }
        for(int axis=0;axis<3;++axis)
            world.translation[axis]=frame.translation[axis]+(world.translation[axis]-frame.translation[axis])*meshScale;
        world.scale*=meshScale;
        if(!H3CorrectSecondaryMatrix(frame,yaw,world)||
            (root&&!ComposeBoneMatrices(inverse,world,candidate[i]))) {++g_h3SecondaryEffectRefused;return;}
        if(!root)candidate[i]=world;
    }
    memcpy(bones,candidate,size_t(count)*sizeof(BoneMatrix));++g_h3SecondaryEffectApplied;
}
