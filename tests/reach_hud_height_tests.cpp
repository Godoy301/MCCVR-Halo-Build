#include <windows.h>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "../src/common/reach_nav_marker_logic.h"

namespace
{
enum class GameTitle { Halo3, HaloReach };
GameTitle activeTitle = GameTitle::HaloReach;
uint32_t activeGeneration = 3;
GameTitle TitleAdapter_GetActiveTitle() { return activeTitle; }
uint32_t TitleAdapter_GetGeneration(GameTitle) { return activeGeneration; }
bool stereo = true;
bool VR_IsStereoEnabled() { return stereo; }
std::atomic<bool> g_enabled{true};
struct
{
    std::atomic<uint32_t> generation{3}, activeCallbacks{0};
    std::atomic<bool> installed{true}, armed{true}, teardownRequested{false};
} g_reachCamera;
struct { float hud_vertical_offset = 16.0f; bool nav_markers_on_target = true; } g_config;
constexpr float kHudHeightMin = -300.0f, kHudHeightMax = 300.0f;
#include "../src/dll/reach_hud_height.inl"

unsigned calls = 0, checks = 0;
bool nativeResult = true, throwNative = false, revokeDuringNative = false;
bool nativeFlag = false;
bool recurseParent = false;
int recurseParentAnchor = 1;    // the anchor type the parent chain's own producer call is made with
bool recurseParentFlag = false; // and the sixth output that call leaves
std::array<float, 13> nativeBasis{};
int seenUser = 0, seenAnchor = 0;
void* seenFlags = nullptr;
void* seenDrawData = nullptr;
void* seenBasis = nullptr;
bool* seenNativeFlag = nullptr;

bool __fastcall NativeAnchor(int user, int anchor, void* flags, void* data,
    void* basis, bool* nativeAnchorFlag)
{
    ++calls;
    seenUser=user; seenAnchor=anchor; seenFlags=flags; seenDrawData=data;
    seenBasis=basis; seenNativeFlag=nativeAnchorFlag;
    if (throwNative)
        RaiseException(0xE017DEAD, 0, 0, nullptr);
    if (revokeDuringNative)
        g_reachCamera.teardownRequested.store(true);
    if (recurseParent && anchor == 0)
    {
        bool parentFlag = !recurseParentFlag;
        const bool previousFlag = nativeFlag;
        nativeFlag = recurseParentFlag;
        const bool result = ReachHudAnchorBasisDetour(user,recurseParentAnchor,flags,data,basis,&parentFlag);
        nativeFlag = previousFlag;
        // Native parent composition may scale the child's basis. Height must
        // be applied after that composition, once, in final virtual pixels.
        static_cast<float*>(basis)[11] *= 2.0f;
        return result;
    }
    if (basis && basis != reinterpret_cast<void*>(1))
        memcpy(basis, nativeBasis.data(), sizeof(nativeBasis));
    if (nativeAnchorFlag) *nativeAnchorFlag = nativeFlag;
    return nativeResult;
}

void Check(bool value, const char* name)
{
    ++checks;
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", name); ExitProcess(1); }
}

void Reset()
{
    activeTitle=GameTitle::HaloReach; activeGeneration=3; stereo=true;
    g_enabled=true; g_reachCamera.generation=3; g_reachCamera.installed=true;
    g_reachCamera.armed=true; g_reachCamera.teardownRequested=false;
    g_reachHudHeightEnabled=true; g_reachHudHeightRedirected=false;
    g_reachHudHeightApplied=0; g_reachHudHeightRefused=0; g_reachHudHeightWorldExempt=0;
    g_config.nav_markers_on_target=true; recurseParentAnchor=1; recurseParentFlag=false;
    g_reachOrigHudAnchorBasis=&NativeAnchor; g_config.hud_vertical_offset=16;
    calls=0; nativeResult=true; throwNative=false; revokeDuringNative=false;
    recurseParent=false; nativeFlag=false;
    for (size_t i=0;i<nativeBasis.size();++i) nativeBasis[i]=float(i*17+3);
}

void Run(bool translated, const char* name, int anchor = 0x1234ABCD)
{
    std::array<float,13> output{};
    std::array<uint64_t,8> drawData{};
    const auto pristineDrawData=drawData;
    uint32_t placementFlags=0xFEDCBA98;
    bool nativeAnchorFlag=!nativeFlag;
    const auto beforeCalls=calls;
    const bool returned=ReachHudAnchorBasisDetour(2, anchor,
        &placementFlags, drawData.data(), output.data(), &nativeAnchorFlag);
    Check(calls==beforeCalls+1,"native called exactly once");
    Check(returned==nativeResult,"native success/failure preserved");
    Check(seenUser==2 && seenAnchor==anchor &&
        seenFlags==&placementFlags && seenDrawData==drawData.data() &&
        seenBasis==output.data() && seenNativeFlag==&nativeAnchorFlag,
        "six full arguments forwarded without narrowing");
    Check(drawData==pristineDrawData && placementFlags==0xFEDCBA98 &&
        nativeAnchorFlag==nativeFlag,"argument four and native sixth-argument flag preserved");
    auto expected=nativeBasis;
    if (translated) expected[11]-=g_config.hud_vertical_offset;
    Check(memcmp(expected.data(),output.data(),sizeof(expected))==0,name);
    Check(g_reachCamera.activeCallbacks==0,"callback accounting drained");
}

bool CatchWorldAnchorException()
{
    bool flag = false;
    __try { ReachHudAnchorBasisDetour(0,26,nullptr,nullptr,nullptr,&flag); }
    __except (GetExceptionCode()==0xE017DEAD ?
        EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) { return true; }
    return false;
}

bool CatchNativeException()
{
    __try { ReachHudAnchorBasisDetour(0,1,nullptr,nullptr,nullptr,nullptr); }
    __except (GetExceptionCode()==0xE017DEAD ?
        EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) { return true; }
    return false;
}
}

int main()
{
    for (float height : {-300.0f,-16.0f,0.0f,16.0f,300.0f})
    for (bool flag : {false,true})
    { Reset(); nativeFlag=flag; g_config.hud_vertical_offset=height;
      Run(true,"only virtual pixel Y translated for either native anchor flag"); }
    Reset(); nativeResult=false; Run(false,"failed native anchor unchanged");
    Reset(); g_reachHudHeightRedirected=true; Run(false,"captured/suppressed reticle unchanged");
    Reset(); g_reachHudHeightEnabled=false; Run(false,"feature unavailable stays stock");
    Reset(); activeTitle=GameTitle::Halo3; Run(false,"other title stays stock");
    Reset(); activeGeneration=4; Run(false,"stale generation stays stock");
    Reset(); g_reachCamera.generation=0; Run(false,"zero generation stays stock");
    Reset(); g_reachCamera.installed=false; Run(false,"uninstalled stays stock");
    Reset(); g_reachCamera.armed=false; Run(false,"unarmed stays stock");
    Reset(); g_reachCamera.teardownRequested=true; Run(false,"teardown stays stock");
    Reset(); g_enabled=false; Run(false,"disabled VR stays stock");
    Reset(); stereo=false; Run(false,"flat rendering stays stock");
    Reset(); revokeDuringNative=true; Run(false,"ownership rechecked after original returns");
    for (float height : {-301.0f,301.0f,std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::quiet_NaN()})
    { Reset(); g_config.hud_vertical_offset=height; Run(false,"invalid configuration stays stock");
      Check(g_reachHudHeightRefused==1,"numeric refusal reported off hook"); }
    Reset(); nativeBasis[11]=std::numeric_limits<float>::quiet_NaN();
    Run(false,"nonfinite native Y remains untouched");
    Reset(); Check(ReachHudAnchorBasisDetour(0,1,nullptr,nullptr,nullptr,nullptr),"null basis preserves native result");
    Check(calls==1 && g_reachCamera.activeCallbacks==0,"null basis call accounted");
    Reset(); Check(ReachHudAnchorBasisDetour(0,1,nullptr,nullptr,reinterpret_cast<void*>(1),nullptr),"bad optional basis preserves native result");
    Check(g_reachHudHeightRefused==1 && g_reachHudHeightEnabled &&
        g_reachCamera.armed && g_reachCamera.activeCallbacks==0,"SEH isolated to height and counted");
    Reset(); throwNative=true; Check(CatchNativeException(),"native exception is propagated");
    Check(calls==1 && g_reachCamera.activeCallbacks==0 && g_reachHudAnchorDepth==0,
        "native exception drains callback and recursion accounting");
    Reset(); recurseParent=true;
    std::array<float,13> composed{};
    Check(ReachHudAnchorBasisDetour(0,0,nullptr,nullptr,composed.data(),nullptr),
        "parent anchor preserves nested native result");
    Check(calls==2 && g_reachHudHeightApplied==1 && g_reachHudAnchorDepth==0 &&
        composed[11]==nativeBasis[11]*2.0f-16.0f,
        "parent anchor translates once after complete native composition");
    // NAVIGATION MARKERS: a basis the producer put on a point in the world is left where the game put it.
    // HREK's own anchor types, and the sixth output the producer clears when it projected the point.
    for (int anchor = -2; anchor <= 60; ++anchor)
    {
        const bool world = anchor==15 || anchor==16 || (anchor>=21 && anchor<=26) || (anchor>=40 && anchor<=49);
        Check(reach_nav_markers::AnchorIsWorld(anchor)==world,"the world anchor types are HREK's");
        Reset(); nativeFlag=false; Run(!world,"a world anchor is not moved by the HUD height; every other is",anchor);
        Check(g_reachHudHeightWorldExempt==(world?1u:0u) && g_reachHudHeightApplied==(world?0u:1u),
            "each anchor is counted once, as left alone or as moved");
        // The same anchor type with the sixth output still set is not a projected point (anchor 21 falls back to
        // the crosshair's own place when the weapon has no target): it is HUD furniture and moves.
        Reset(); nativeFlag=true; Run(true,"an anchor the producer did not project still moves",anchor);
        Check(g_reachHudHeightWorldExempt==0,"nothing is counted as left alone when the point was not projected");
        // The player's switch off: every anchor moves, as before this feature.
        Reset(); nativeFlag=false; g_config.nav_markers_on_target=false;
        Run(true,"switch off: a world anchor moves with the HUD height as before",anchor);
        Check(g_reachHudHeightWorldExempt==0 && g_reachHudHeightApplied==1,"switch off counts nothing as left alone");
        Check(!g_reachHudAnchorWorld,"the world mark never outlives its transaction");
    }
    // A world anchor that failed (no such object) produces no basis: nothing moves and nothing is counted.
    Reset(); nativeResult=false; nativeFlag=false; Run(false,"a failed world anchor is untouched",26);
    Check(g_reachHudHeightWorldExempt==0 && g_reachHudHeightApplied==0,"a failed world anchor counts nothing");
    // The reticle redirect and a flat frame never counted as moved; a world anchor under them is not counted as
    // left alone either (the counter says "would have been moved").
    Reset(); nativeFlag=false; g_reachHudHeightRedirected=true; Run(false,"a redirected world anchor is untouched",25);
    Check(g_reachHudHeightWorldExempt==0,"a redirected world anchor is not counted");
    Reset(); nativeFlag=false; stereo=false; Run(false,"a flat world anchor is untouched",25);
    Check(g_reachHudHeightWorldExempt==0,"a flat world anchor is not counted");
    // No sixth output to read: the type alone does not make it a projected point.
    Reset();
    { std::array<float,13> basis{};
      Check(ReachHudAnchorBasisDetour(0,26,nullptr,nullptr,basis.data(),nullptr),"no sixth output: native result kept");
      Check(basis[11]==nativeBasis[11]-16.0f && g_reachHudHeightWorldExempt==0,
          "no sixth output: the anchor moves as before"); }
    // A child widget under a world-anchored parent: the outer call is the parent anchor (0), the world anchor
    // is the nested one. The composed basis rides the marker, so it is not moved either.
    for (int parent : {15,16,21,22,23,24,25,26,40,45,46,47,49})
    {
        Reset(); recurseParent=true; recurseParentAnchor=parent; recurseParentFlag=false;
        std::array<float,13> child{};
        bool childFlag=true;
        Check(ReachHudAnchorBasisDetour(0,0,nullptr,nullptr,child.data(),&childFlag),"a child of a world anchor is produced");
        Check(calls==2 && child[11]==nativeBasis[11]*2.0f && g_reachHudHeightApplied==0 &&
            g_reachHudHeightWorldExempt==1 && g_reachHudAnchorDepth==0 && !g_reachHudAnchorWorld,
            "a child of a world anchor is left on the marker, counted once");
        // The same chain with the switch off, and with a parent the producer did not project.
        Reset(); recurseParent=true; recurseParentAnchor=parent; g_config.nav_markers_on_target=false;
        Check(ReachHudAnchorBasisDetour(0,0,nullptr,nullptr,child.data(),&childFlag) &&
            child[11]==nativeBasis[11]*2.0f-16.0f && g_reachHudHeightWorldExempt==0,
            "switch off: a child of a world anchor moves once, as before");
        Reset(); recurseParent=true; recurseParentAnchor=parent; recurseParentFlag=true;
        Check(ReachHudAnchorBasisDetour(0,0,nullptr,nullptr,child.data(),&childFlag) &&
            child[11]==nativeBasis[11]*2.0f-16.0f && g_reachHudHeightWorldExempt==0,
            "a child of a parent that was not projected moves once");
    }
    // The mark is one transaction's: a world anchor, then a screen anchor, on the same thread.
    Reset(); nativeFlag=false; Run(false,"a world anchor first",26);
    nativeFlag=false; Run(true,"then a screen anchor moves",3);
    Check(g_reachHudHeightWorldExempt==1 && g_reachHudHeightApplied==1,"the mark does not leak into the next anchor");
    // A native exception inside a world anchor drains the mark with the depth.
    Reset(); throwNative=true;
    Check(CatchWorldAnchorException() && g_reachHudAnchorDepth==0 && !g_reachHudAnchorWorld &&
        g_reachCamera.activeCallbacks==0,
        "a native exception in a world anchor drains the mark, the depth and the callback count");
    Reset(); g_reachOrigHudAnchorBasis=nullptr;
    Check(!ReachHudAnchorBasisDetour(0,1,nullptr,nullptr,nullptr,nullptr) &&
        g_reachCamera.activeCallbacks==0,"absent original safely refuses");
    std::printf("Reach production HUD height wrapper: %u checks passed\n",checks);
    return 0;
}
