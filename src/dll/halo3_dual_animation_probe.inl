// Temporary diagnostic: bounded atomic snapshots in the palette hook; all
// formatting and file output run on the existing diagnostic reporting path.
struct H3DualAnimationSample
{
    std::atomic_flag writing=ATOMIC_FLAG_INIT;
    std::atomic<uint32_t> sequence{0},generation{0},tag{0},time{0};
    std::atomic<int> nodes[4]{};
    std::atomic<float> values[96]{};
};
H3DualAnimationSample g_h3DualAnimation[2];
void CaptureH3DualAnimation(uint16_t tag,const FpInterpolationContext& context,
    const BoneMatrix* authored,const BoneMatrix* solved,const BoneMatrix* root,const BoneMatrix* finalPalette)
{
    if(!context.valid || context.player!=0 || context.slot<0 || context.slot>1 ||
       context.count<=0 || context.count>64 || !authored || !solved || !root || !finalPalette) return;
    auto& sample=g_h3DualAnimation[context.slot];
    if(sample.writing.test_and_set(std::memory_order_acquire)) return;
    const uint32_t now=GetTickCount();
    if(uint32_t(now-sample.time.load(std::memory_order_relaxed))<100) {
        sample.writing.clear(std::memory_order_release);return;
    }
    int held=-1;
    for(int i=0;i<context.count;++i)
        if(FpSourceIndexIsHeldObject(i,context.count,context.heldObjectStart,context.heldSources)) {held=i;break;}
    const int nodes[]{context.wrist,context.lWrist,held,0};
    sample.sequence.fetch_add(1,std::memory_order_acq_rel);
    sample.generation.store(context.generation,std::memory_order_relaxed);
    sample.tag.store(tag,std::memory_order_relaxed);
    sample.time.store(now,std::memory_order_relaxed);
    for(int n=0;n<4;++n) {
        const int node=nodes[n];
        sample.nodes[n].store(node,std::memory_order_relaxed);
        for(int stage=0;stage<2;++stage) {
            const BoneMatrix* matrix=n==3 ? (stage?finalPalette:root) :
                (node>=0 && node<context.count ? (stage?solved:authored)+node : nullptr);
            for(int k=0;k<12;++k) {
                const float value=matrix?(k<3?matrix->translation[k]:matrix->rotation[k-3]):0.f;
                sample.values[n*24+stage*12+k].store(value,std::memory_order_relaxed);
            }
        }
    }
    sample.sequence.fetch_add(1,std::memory_order_release);
    sample.writing.clear(std::memory_order_release);
}
void ReportH3DualAnimation()
{
    static uint32_t reported[2]{};
    for(int slot=0;slot<2;++slot) {
        auto& sample=g_h3DualAnimation[slot];
        const auto sequence=sample.sequence.load(std::memory_order_acquire);
        if(!sequence || (sequence&1) || sequence==reported[slot]) continue;
        const auto generation=sample.generation.load(),tag=sample.tag.load(),time=sample.time.load();
        int nodes[4];float values[96];
        for(int n=0;n<4;++n) nodes[n]=sample.nodes[n].load();
        for(int k=0;k<96;++k) values[k]=sample.values[k].load();
        if(sequence!=sample.sequence.load(std::memory_order_acquire)) continue;
        reported[slot]=sequence;
        for(int n=0;n<4;++n) for(int stage=0;stage<2;++stage) {
            const float* v=values+n*24+stage*12;
            LOG("H3 DUAL ANIM time=%u gen=%u slot=%d tag=%04X role=%d node=%d stage=%s p=%.5f,%.5f,%.5f basis=%.5f,%.5f,%.5f/%.5f,%.5f,%.5f/%.5f,%.5f,%.5f",
                time,generation,slot,tag,n,nodes[n],n==3?(stage?"final-node0":"root"):(stage?"solved":"authored"),
                v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8],v[9],v[10],v[11]);
        }
    }
}
