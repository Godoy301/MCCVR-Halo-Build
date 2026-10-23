#include "../src/common/odst_modded_fp_logic.h"
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
static unsigned checks=0;
static void Check(bool ok,const char* message)
{ ++checks; if(!ok){std::fprintf(stderr,"FAIL: %s\n",message);std::exit(1);} }
#include "odst_modded_fp_assets.inl"
// Engine boundaries are deterministic stubs; the shipping capture, cache,
// palette admission and rollback code below are compiled without rewriting.
struct BoneMatrix { float value=0; };
struct FpInterpolationContext {
    const BoneMatrix* source=nullptr; uint32_t generation=0;
    int count=0,player=-1,slot=-1,wrist=-1,lWrist=-1,elbow=-1,lElbow=-1;
    int shoulder=-1,lShoulder=-1,cameraControl=-1,heldObjectStart=-1;
    uint64_t wristDescendants=0,lWristDescendants=0;
    FpHeldSources heldSources{}; uint32_t muzzleUnit=UINT32_MAX,muzzleWeapon=UINT32_MAX;
    bool valid=false;
};
static FpInterpolationContext g_fpInterpolationContexts[2];
static BoneMatrix g_fpUnmodifiedInterpolations[2][64];
static std::atomic<uint32_t> g_odstRuntimeGeneration{7};
struct Scope { bool armed=true; struct Tracking {uint64_t serial=1,referenceEpoch=1;} anatomicalTracking; };
static Scope g_fpStereoSolveScope;
enum class GameTitle {Halo3ODST};
static const Asset* active=nullptr;
static bool readable=true,markerSuccess=true;
int OdstReadFpModelIdentity(uint16_t tag,uint32_t& checksum)
{ checksum=tag==1?active->checksum:0; return tag==1?active->bodyCount:active->modelCount; }
static bool SafeReadBytes(const void* p,void* q,size_t n)
{ if(!p||!readable)return false;std::memcpy(q,p,n);return true; }
static bool OdstCaptureMuzzleOwner(int,int,uint32_t&,uint32_t&) {return false;}
static void VR_ObserveSecondaryWeaponPresentation(GameTitle,uint32_t) {}
static void MeasureOdstAuthoredBarrel(int,const BoneMatrix*,int) {}
static bool BuildOdstCenterFpRoot(int,BoneMatrix&) {return true;}
static bool ApplyControllerToMarkerBonesFromRoot(BoneMatrix,BoneMatrix* bones,int count,int,int,bool)
{if(!markerSuccess)return false;for(int i=0;i<count;++i)bones[i].value+=100;return true;}
static void PublishOdstFpLayoutSelfCheck(int,int,bool,const OdstFpSkeletonLayout&,int=0,uint32_t=0) {}
#define ODST_MODDED_FP_TESTING 1
#include "../src/dll/odst_modded_fp.inl"
#undef ODST_MODDED_FP_TESTING
static uint64_t Descendants(const Asset& a,int root)
{
    uint64_t mask=0;
    for(int i=0;i<a.bodyCount;++i)
        for(int n=i,depth=0;n>=0&&depth<a.count;++depth)
        { if(n==root){mask|=uint64_t{1}<<i;break;}n=a.parents[n]; }
    return mask;
}
static FpInterpolationContext Capture(BoneMatrix* bones,int count,int slot=0,int id=42)
{
    g_fpInterpolationContexts[slot]={};
    OdstCaptureMappedInterpolation(0,id,slot,true,&bones,&count,false,UINT32_MAX,UINT32_MAX);
    return g_fpInterpolationContexts[slot];
}
int main()
{
    for(const auto& a:assets)
    {
        active=&a;
        OdstMappedFpLayout layout{};
        const auto body=std::span<const int32_t>{a.body.data(),size_t(a.bodyCount)};
        const auto model=std::span<const int32_t>{a.model.data(),size_t(a.modelCount)};
        Check(ResolveOdstMappedFpLayout(a.checksum,body,a.count,layout),"actual ODST body admitted");
        Check(layout.skeleton.leftHandDescendants==Descendants(a,5),"own left fingers");
        Check(layout.skeleton.rightHandAndWeaponDescendants==Descendants(a,6),"own right fingers");
        if(a.bodyCount==31)
        {
            OdstFpSkeletonLayout old{};
            const bool oldAccepted=ComputeOdstFpSkeletonLayout(a.count,old);
            Check(a.count<39?!oldAccepted:oldAccepted,"reproduce old count gate");
            if(oldAccepted)Check(old.leftHandDescendants!=Descendants(a,5),"reproduce old Elite finger corruption");
        }
        FpHeldSources held{};
        Check(ResolveOdstHeldSources(layout,model,a.count,held),"actual gun mapping");
        for(int n=0;n<a.count;++n)
            Check(FpSourceIndexIsHeldObject(n,a.count,-1,held)==
                (std::find(model.begin(),model.end(),n)!=model.end()),"only native gun outputs carried/visible");
        // Permute every source except the separately reserved camera. Neither
        // hand ownership nor gun ownership may depend on prefix positions.
        auto perm=a.body,gun=a.model;
        for(int n=0;n<a.bodyCount;++n)perm[n]=a.count-2-perm[n];
        for(int n=0;n<a.modelCount;++n)gun[n]=a.count-2-gun[n];
        OdstMappedFpLayout reordered{};
        Check(ResolveOdstMappedFpLayout(a.checksum,{perm.data(),size_t(a.bodyCount)},a.count,reordered),"reordered body");
        Check(reordered.skeleton.rightWrist==perm[6],"mapped wrist");
        Check(ResolveOdstHeldSources(reordered,{gun.data(),size_t(a.modelCount)},a.count,held),"reordered gun");
        Check(!ResolveOdstMappedFpLayout(0,body,a.count,reordered),"unknown body rejected");
        perm[1]=perm[0];
        Check(!ResolveOdstMappedFpLayout(a.checksum,{perm.data(),size_t(a.bodyCount)},a.count,reordered),"duplicate body rejected");
        Check(!ResolveOdstHeldSources(layout,body,a.count,held),"body cannot become gun");
        int32_t invalid[]{a.count-1};
        Check(!ResolveOdstHeldSources(layout,invalid,a.count,held),"camera cannot become gun");
        invalid[0]=-2;
        Check(!ResolveOdstHeldSources(layout,invalid,a.count,held),"invalid sentinel rejected");
        invalid[0]=-1;
        Check(!ResolveOdstHeldSources(layout,invalid,a.count,held),"empty selection rejected");
        Check(!ResolveOdstMappedFpLayout(a.checksum,body,65,reordered),"bounded graph");
        for(int slot=0;slot<2;++slot)
        {
            g_odstMappedFpCache={};g_fpStereoSolveScope={};readable=true;
            BoneMatrix bones[64]{};
            auto context=Capture(bones,a.count,slot);
            Check(!OdstResolveMappedPalette(1,a.body.data(),context),"first body learns stock");
            context=Capture(bones,a.count,slot);
            Check(!OdstResolveMappedPalette(2,a.model.data(),context),"same-pair gun remains stock");
            context=Capture(bones,a.count,slot);
            Check(!OdstResolveMappedPalette(1,a.body.data(),context),"second eye cannot adopt newly learned body");
            ++g_fpStereoSolveScope.anatomicalTracking.serial;
            context=Capture(bones,a.count,slot);
            Check(OdstResolveMappedPalette(1,a.body.data(),context),"next-pair body resolves");
            Check(context.wrist==6 && context.lWrist==5,"runtime anchor assignment");
            Check(context.heldSources.explicitMap&&!context.heldSources.bits[0],"body does not own gun outputs");
            std::fill(std::begin(bones),std::end(bones),BoneMatrix{});
            context=Capture(bones,a.count,slot);
            Check(OdstResolveMappedPalette(2,a.model.data(),context),"next-pair gun resolves");
            Check(context.heldSources.bits[0]!=0,"runtime gun selection");
            Check(bones[0].value==100,"marker transformed");
            std::fill(std::begin(bones),std::end(bones),BoneMatrix{});
            context=Capture(bones,a.count,slot);readable=false;
            Check(!OdstResolveMappedPalette(2,a.model.data(),context),"unreadable map rejects");
            Check(bones[0].value==0,"rejection restores native graph");readable=true;
            ++g_fpStereoSolveScope.anatomicalTracking.serial;
            context=Capture(bones,a.count,slot);
            Check(!OdstResolveMappedPalette(2,a.model.data(),context),"rejected layout retired next pair");
            context=Capture(bones,a.count,slot);OdstResolveMappedPalette(1,a.body.data(),context);
            ++g_fpStereoSolveScope.anatomicalTracking.serial;
            context=Capture(bones,a.count,slot,99);
            Check(!OdstResolveMappedPalette(2,a.model.data(),context),"interpolation identity isolated");
            // A different body remap with the same count/id must be observed
            // afresh, and speculative marker writes must be rolled back.
            std::fill(std::begin(bones),std::end(bones),BoneMatrix{});
            context=Capture(bones,a.count,slot);
            auto changed=a.body;std::swap(changed[1],changed[2]);
            Check(!OdstResolveMappedPalette(1,changed.data(),context),"same-count body change rejects");
            Check(bones[0].value==0,"changed body restores markers");
            ++g_fpStereoSolveScope.anatomicalTracking.serial;
            context=Capture(bones,a.count,slot);
            Check(OdstResolveMappedPalette(1,changed.data(),context),"changed body learned next pair");
            Check(context.shoulder==1&&context.lShoulder==2,"changed anchors applied");
            ++g_odstRuntimeGeneration;
            context=Capture(bones,a.count,slot);
            Check(!OdstResolveMappedPalette(2,a.model.data(),context),"generation retires layout");
            ++g_fpStereoSolveScope.anatomicalTracking.referenceEpoch;
            context=Capture(bones,a.count,slot);
            Check(!OdstResolveMappedPalette(2,a.model.data(),context),"recenter retires layout");
        }
    }
    std::printf("ODST modded FP: %u checks across %zu asset/body combinations passed\n",checks,std::size(assets));
}
