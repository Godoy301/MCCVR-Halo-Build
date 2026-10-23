// Keep only one positively observed body layout per slot. No union learning or
// admission gate: unknown identities continue through the original solver.
struct OdstMythicLayoutReceipt {
    OdstFpLayoutKey key{};
    uint64_t epoch=0;
    OdstMappedFpLayout layout{};
    bool valid=false;
};
thread_local OdstMythicLayoutReceipt g_odstMythicReceipts[2];
thread_local OdstFpLayoutKey g_odstMythicInvocation[2];

void OdstPrepareMythicInterpolation(int view,int id,int slot,FpInterpolationContext& context)
{
    auto& receipt=g_odstMythicReceipts[slot];
    const OdstFpLayoutKey key{context.generation,view,id,slot,context.count};
    g_odstMythicInvocation[slot]=key;
    const uint64_t epoch=g_fpStereoSolveScope.armed?
        g_fpStereoSolveScope.anatomicalTracking.referenceEpoch:0;
    if(!epoch || receipt.key!=key || receipt.epoch!=epoch) {
        if(receipt.valid)OdstResetAuthoredBarrel(slot);
        receipt={};
        return;
    }
    if(receipt.valid) {
        OdstApplyMappedLayout(receipt.layout,context);
        context.heldSources={};
    }
}

// Correct only the live, positively identified early-camera Mythic body palette.
// Keep the original solver for all other palettes, including partial dual bodies.
void OdstCorrectMythicBodyPalette(uint16_t tag,const int32_t* map,
    FpInterpolationContext& context)
{
    if(!context.valid || context.slot<0 || context.slot>1 || !map || !context.source ||
        context.generation!=g_odstRuntimeGeneration.load(std::memory_order_acquire))return;
    uint32_t checksum=0;
    if(OdstReadFpModelIdentity(tag,checksum)!=37)return;
    int32_t remap[37]{};
    OdstMappedFpLayout layout{};
    if(!OdstTryReadMappedPaletteBytes(map,remap,sizeof(remap)) ||
        !ResolveOdstMythicEarlyCamera(checksum,std::span<const int32_t>{remap,37},context.count,layout)) {
        g_odstMythicReceipts[context.slot]={};
        return;
    }
    const uint64_t epoch=g_fpStereoSolveScope.armed?
        g_fpStereoSolveScope.anatomicalTracking.referenceEpoch:0;
    if(epoch)g_odstMythicReceipts[context.slot]={g_odstMythicInvocation[context.slot],epoch,layout,true};
    if(context.wrist==layout.skeleton.rightWrist && context.cameraControl==1) {
        // Interpolation already used the corrected wrist. Do not filter the
        // barrel twice or apply the marker transform a second time.
        g_fpInterpolationContexts[context.slot]=context;
        return;
    }
    OdstResetAuthoredBarrel(context.slot);
    // The interpolation hook already saved the original graph. Rebuild marker
    // transforms from that copy, never from its incorrectly indexed result.
    BoneMatrix root{};
    if(!BuildOdstCenterFpRoot(context.slot,root))return;
    OdstApplyMappedLayout(layout,context);
    context.heldSources={};
    auto* source=const_cast<BoneMatrix*>(context.source);
    memcpy(source,g_fpUnmodifiedInterpolations[context.slot],size_t(context.count)*sizeof(BoneMatrix));
    MeasureOdstAuthoredBarrel(context.slot,g_fpUnmodifiedInterpolations[context.slot],context.wrist);
    ApplyControllerToMarkerBonesFromRoot(root,source,context.count,context.wrist,context.lWrist,context.slot==1);
    // The following gun palette uses the same source and corrected role masks.
    g_fpInterpolationContexts[context.slot]=context;
    PublishOdstFpLayoutSelfCheck(context.slot,context.count,true,layout.skeleton,37,checksum);
}
