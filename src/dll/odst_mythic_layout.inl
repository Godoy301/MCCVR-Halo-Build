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
        !ResolveOdstMythicEarlyCamera(checksum,std::span<const int32_t>{remap,37},context.count,layout))return;
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
