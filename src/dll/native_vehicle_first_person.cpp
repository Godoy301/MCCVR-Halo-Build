#include "native_vehicle_first_person.h"
#include "title_adapter.h"
#include "game.h"
#include "sigscan.h"
#include "hook_quiescence.h"
#include "sight_layer.h"
#if HALOMCCVR_DEV_DIAGNOSTICS
#include "halo2_observer_6dof.h"
#include "../common/seat_head_trace_logic.h"
#include <cstdio>
#include <string>
#endif
#include "../common/config.h"
#include "../common/log.h"
#include "../common/minhook_lifecycle.h"
#include "../common/seat_camera_ride_logic.h"
#include <windows.h>
#include <intrin.h>
#include <MinHook.h>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <type_traits>

namespace
{
enum Role : unsigned { Selector,Camera,Marker,MarkerProof,RoleCount };
struct Binding { uint32_t rva;const char* pattern; };
struct BindingSet { GameTitle title;uint32_t markerRva;std::array<Binding,RoleCount> entries; };
#include "native_vehicle_first_person_bindings.generated.h"
struct Runtime
{
    HMODULE module{};uintptr_t base{};uint32_t generation{};
    bool attempted{},ready{},enabled[2]{};
    void* target[2]{};void* original[2]{};void* marker{};void* markerInterpolated{};
    std::atomic<uint64_t> trimIdentity{};
    std::atomic_flag trimWriting=ATOMIC_FLAG_INIT;
    std::atomic<uint32_t> trimVersion{},trimGeneration{};
    std::atomic<int> trimSeat{-1};
    std::atomic<bool> requested{},faulted{};
    std::atomic<uint32_t> callbacks{};
    std::atomic<uint64_t> selected{},positioned{},missingMarker{},faults{};
    // The seat this override last placed the camera in, and when: the seat's view is ours (seat direct drive
    // writes a seat's aim only while it is -- a chase camera would swing round the vehicle with each write).
    std::atomic<uint64_t> positionedMs{};
    std::atomic<uint32_t> positionedParent{UINT32_MAX};
    std::atomic<int> positionedSeat{-1};
    // The simulation's in-tick byte, where this title's is known (ResolveTick): the calling thread's engine TLS
    // block holds, at tickSlot, the frame interpolator's state, whose byte at tickByte the tick loop sets before
    // every tick and the post-tick record clears. tlsIndex null: this title's tick cannot be told from its frame.
    const uint32_t* tlsIndex{};
    uint32_t tickSlot{},tickByte{};
    // Selector calls for our own seat that were answered as the game built them, for the simulation's sake.
    std::atomic<uint64_t> simulationCalls{};
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Why the selector was not taken over: the native mode it chose (0-4, other), and Owner()'s first refusal.
    std::atomic<uint32_t> modeSeen[6]{};
    std::atomic<uint32_t> ownerWhy[7]{};
    std::atomic<uint32_t> lastSelectorUnit{},lastOwnerUnit{};
    // The native seat eye's offset from the placed point (NativeVehicleFirstPerson_SeatEyeDelta). Seqlock with
    // a try-lock writer: PositionOwned runs from every caller of the camera evaluator, on threads nobody has
    // proven are one, so a second writer skips rather than tear the record (the trim publication's rule).
    std::atomic_flag eyeWriting=ATOMIC_FLAG_INIT;
    std::atomic<uint32_t> eyeVersion{},eyeGeneration{},eyeParent{UINT32_MAX};
    std::atomic<int> eyeSeat{-1};
    std::atomic<float> eyeDelta[3]{};
    std::atomic<uint64_t> eyeMs{},eyePublished{},eyeSkipped{};
#endif
    uint64_t reportAt{};
};
Runtime runtime[3];
#if HALOMCCVR_DEV_DIAGNOSTICS
// Halo 2's seat camera, frame by frame (seat_head_trace_logic.h): the camera evaluator records, the cold poll
// reports. One short exclusive section per frame.
SRWLOCK g_seatHeadLock=SRWLOCK_INIT;
seat_head_trace::Sample g_seatHead[seat_head_trace::kRing];
unsigned g_seatHeadCount=0;
void NoteSeatHead(const NativeVehicleCameraOwner& owner)
{
    const auto& r=runtime[1];
    if(!r.marker||!r.markerInterpolated) return;
    const auto query=reinterpret_cast<int16_t(__fastcall*)(uint32_t,uint32_t,void*,int16_t,uint8_t,uint8_t)>(
        r.markerInterpolated);
    alignas(16) uint8_t head[128]{},simulated[128]{},drawn[128]{};
    if(reinterpret_cast<int16_t(__fastcall*)(uint32_t,uint32_t,void*,int16_t)>(
        r.marker)(owner.unit,0x4000095,head,1)!=1) return;
    seat_head_trace::Sample s{};
    std::memcpy(s.atTick,head+96,12);std::memcpy(s.placed,head+96,12);
    const uint32_t vehicle=owner.root!=UINT32_MAX?owner.root:owner.parent;
    if(vehicle!=UINT32_MAX&&query(vehicle,0,simulated,1,0,0)==1&&query(vehicle,0,drawn,1,0,1)==1) {
        float a[seat_camera_ride::kFrameFloats]{},b[seat_camera_ride::kFrameFloats]{};
        std::memcpy(a,simulated+56,sizeof(a));std::memcpy(b,drawn+56,sizeof(b));
        s.hullValid=seat_camera_ride::UsableFrame(a)&&seat_camera_ride::UsableFrame(b);
        std::memcpy(s.hull,a+10,12);std::memcpy(s.hullDrawn,b+10,12);
        if(g_config.vehicle_cam_smoothing) s.carried=seat_camera_ride::CarryPoint(a,b,s.placed);
    }
    long long tick=-1;
    Halo2Observer6Dof_DevGameClock(tick,s.factor);
    s.tick=tick;s.us=Sight_NowUs();s.thread=GetCurrentThreadId();
    AcquireSRWLockExclusive(&g_seatHeadLock);
    if(g_seatHeadCount<seat_head_trace::kRing) g_seatHead[g_seatHeadCount++]=s;
    ReleaseSRWLockExclusive(&g_seatHeadLock);
}
// The cold poll's half: one line for the window, and the rows themselves into a file beside the log while a file
// named halomccvr_seat_trace.on sits there.
void ReportSeatHead()
{
    static seat_head_trace::Sample rows[seat_head_trace::kRing];
    AcquireSRWLockExclusive(&g_seatHeadLock);
    const unsigned n=g_seatHeadCount;
    std::memcpy(rows,g_seatHead,sizeof(rows[0])*n);
    g_seatHeadCount=0;
    ReleaseSRWLockExclusive(&g_seatHeadLock);
    if(n<2) return;
    static seat_head_trace::Carry carry{};
    char line[1800]{};
    if(seat_head_trace::Summarise(rows,n,carry,line,sizeof(line)))
        LOG("Vehicle first person title=6 seat camera trace (dev): %s",line);
    std::wstring dir=LogDirectory();
    if(dir.empty()) return;
    if(dir.back()!=L'\\'&&dir.back()!=L'/') dir+=L'\\';
    if(GetFileAttributesW((dir+L"halomccvr_seat_trace.on").c_str())==INVALID_FILE_ATTRIBUTES) return;
    FILE* f=nullptr;
    if(_wfopen_s(&f,(dir+L"halomccvr_seat_trace.csv").c_str(),L"ab")||!f) return;
    for(unsigned k=0;k<n;++k) {
        const auto& s=rows[k];
        std::fprintf(f,"%llu,%lld,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%u\n",
            static_cast<unsigned long long>(s.us),static_cast<long long>(s.tick),double(s.factor),
            double(s.placed[0]),double(s.placed[1]),double(s.placed[2]),double(s.atTick[0]),double(s.atTick[1]),
            double(s.atTick[2]),double(s.hull[0]),double(s.hull[1]),double(s.hull[2]),double(s.hullDrawn[0]),
            double(s.hullDrawn[1]),double(s.hullDrawn[2]),s.hullValid?1:0,s.carried?1:0,s.thread);
    }
    std::fclose(f);
}
// The first-person witness (NativeVehicleFirstPerson_ArmSelectWitness): armed and taken by one caller on its own
// thread around one native call; Halo 4's selector hook records into it only while it is armed.
thread_local NativeVehicleSelectWitness t_selectWitness{};
thread_local bool t_selectWitnessArmed=false;
#endif
using CameraFn=void(__fastcall*)(uint32_t,const void*,void*);
bool Current(unsigned i) noexcept
{
    const auto& r=runtime[i];
    return r.requested.load(std::memory_order_acquire)&&!r.faulted.load(std::memory_order_acquire)&&
        TitleAdapter_GetActiveTitle()==kBindings[i].title&&
        TitleAdapter_GetGeneration(kBindings[i].title)==r.generation;
}
enum OwnerWhy : unsigned { OwnerOk,OwnerNotCurrent,OwnerBadUnit,OwnerRead,OwnerUnitMismatch,OwnerGeneration,
    OwnerNoSeat };
unsigned OwnerReason(unsigned i,uint32_t unit,NativeVehicleCameraOwner& owner) noexcept
{
    if (!Current(i)) return OwnerNotCurrent;
    if (unit==UINT32_MAX||!(unit>>16)) return OwnerBadUnit;
    if (!Game_ReadVehicleCameraOwner(kBindings[i].title,owner)) return OwnerRead;
    if (owner.unit!=unit) return OwnerUnitMismatch;
    if (owner.generation!=runtime[i].generation) return OwnerGeneration;
    if (owner.parent==UINT32_MAX||owner.seat<0) return OwnerNoSeat;
    return OwnerOk;
}
bool Owner(unsigned i,uint32_t unit,NativeVehicleCameraOwner& owner) noexcept
{
    const unsigned why=OwnerReason(i,unit,owner);
#if HALOMCCVR_DEV_DIAGNOSTICS
    runtime[i].ownerWhy[why].fetch_add(1,std::memory_order_relaxed);
    if (why==OwnerUnitMismatch) {
        runtime[i].lastSelectorUnit.store(unit,std::memory_order_relaxed);
        runtime[i].lastOwnerUnit.store(owner.unit,std::memory_order_relaxed);
    }
#endif
    return why==OwnerOk;
}
// Halo 2 draws a vehicle between the last two ticks (MCC's frame interpolator) but answers a marker of its seated
// occupant from the simulation, interpolation asked for or not: the occupant is not interpolated on its own. Left
// there, the camera sits at the newest tick while the hull is drawn up to a tick's travel behind it, by an amount
// that changes every frame. So the point is carried by the vehicle's own interpolation this frame
// (seat_camera_ride_logic.h): the marker query, asked for no marker (0), answers the object's root node, as drawn
// when interpolation is asked for and the interpolator holds the object, else as simulated. A vehicle the
// interpolator does not hold gives the same frame twice and the point stays where it is.
bool RideDrawnVehicle(const NativeVehicleCameraOwner& owner,float position[3])
{
    const auto& r=runtime[1];
    const uint32_t vehicle=owner.root!=UINT32_MAX?owner.root:owner.parent;
    if (!r.markerInterpolated||vehicle==UINT32_MAX) return false;
    const auto query=reinterpret_cast<int16_t(__fastcall*)(uint32_t,uint32_t,void*,int16_t,uint8_t,uint8_t)>(
        r.markerInterpolated);
    alignas(16) uint8_t simulated[128]{},drawn[128]{};
    if (query(vehicle,0,simulated,1,0,0)!=1||query(vehicle,0,drawn,1,0,1)!=1) return false;
    float a[seat_camera_ride::kFrameFloats]{},b[seat_camera_ride::kFrameFloats]{};
    std::memcpy(a,simulated+56,sizeof(a));std::memcpy(b,drawn+56,sizeof(b));
    return seat_camera_ride::CarryPoint(a,b,position);
}
// All three native marker records have their world transform at +56 and
// translation at +96, independently witnessed by their own camera consumers.
// Positive count is essential: the native missing-marker fallback also fills
// a transform, but returns zero. It must not put the camera at the body origin.
// `place`: the point will be used as the camera's (the selector only asks whether the marker is there).
__declspec(noinline) bool Head(unsigned i,const NativeVehicleCameraOwner& owner,float position[3],bool place)
{
    const uint32_t unit=owner.unit;
    alignas(16) uint8_t record[128]{};
    int16_t count=0;
    if (i==0) count=reinterpret_cast<int16_t(__fastcall*)(uint32_t,const char*,void*,int16_t)>(
        runtime[i].marker)(unit,"head",record,1);
    else if (i==1) count=reinterpret_cast<int16_t(__fastcall*)(uint32_t,uint32_t,void*,int16_t)>(
        runtime[i].marker)(unit,0x4000095,record,1);
    else count=reinterpret_cast<int16_t(__fastcall*)(uint32_t,uint32_t,void*,int16_t,uint8_t,uint8_t,uint8_t)>(
        runtime[i].marker)(unit,0x122,record,1,0,0,1);
    if (count!=1) return false;
    std::memcpy(position,record+96,12);
    float forward[3]{},up[3]{};
    std::memcpy(forward,record+60,12);std::memcpy(up,record+84,12);
    float forwardLength=0,upLength=0,dot=0;
    for (unsigned axis=0;axis<3;++axis) {
        if (!std::isfinite(forward[axis])||!std::isfinite(up[axis])) return false;
        forwardLength+=forward[axis]*forward[axis];upLength+=up[axis]*up[axis];dot+=forward[axis]*up[axis];
    }
    if (std::fabs(forwardLength-1)>0.05f||std::fabs(upLength-1)>0.05f||std::fabs(dot)>0.05f) return false;
    const float right[3]{forward[1]*up[2]-forward[2]*up[1],
        forward[2]*up[0]-forward[0]*up[2],forward[0]*up[1]-forward[1]*up[0]};
    const float scale=Game_GetWorldScale();
    const auto title=kBindings[i].title;
    const float f=ConfigVehicleModelCam(g_config,title,owner.vehicleIdentity,owner.seat,0),
        u=ConfigVehicleModelCam(g_config,title,owner.vehicleIdentity,owner.seat,1),
        h=ConfigVehicleModelCam(g_config,title,owner.vehicleIdentity,owner.seat,2);
    if (!std::isfinite(scale)||scale<=0||scale>100||!std::isfinite(f)||!std::isfinite(u)||
        !std::isfinite(h)||std::fabs(f)>5||std::fabs(u)>5||std::fabs(h)>5) return false;
    for (unsigned axis=0;axis<3;++axis)
        position[axis]+=scale*(forward[axis]*f+up[axis]*u+right[axis]*h);
    for (unsigned axis=0;axis<3;++axis)
        if (!std::isfinite(position[axis])||std::fabs(position[axis])>100000.0f) return false;
    if (i==1&&place&&g_config.vehicle_cam_smoothing) RideDrawnVehicle(owner,position);
    return true;
}
__declspec(noinline) void Fault(unsigned i) noexcept
{
    // Keep a real handler call under MSVC /O2. Production-detour SEH fixtures
    // verify that a native marker exception persists this feature-only latch.
    runtime[i].faulted.store(true,std::memory_order_release);
    runtime[i].faults.fetch_add(1,std::memory_order_relaxed);
}
// Inside a simulation tick on the calling thread (1), outside one (0), or not known (-1). Guarded: the block and
// the state are the engine's own.
__declspec(noinline) int InTick(unsigned i) noexcept
{
    const auto& r=runtime[i];
    if (!r.tlsIndex) return -1;
    __try {
        const uint32_t index=*r.tlsIndex;
        const auto* slots=reinterpret_cast<const uintptr_t*>(__readgsqword(0x58));
        if (!slots||index>=0x400||!slots[index]) return -1;
        const auto* state=*reinterpret_cast<const uint8_t* const*>(slots[index]+r.tickSlot);
        return state?(state[r.tickByte]?1:0):-1;
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}
// WHO IS ASKING. The selector and the camera answer the simulation as well as the frame: the player's per-tick
// targeting and a fire's crosshair line start from the seat's camera, and in a shared simulation every machine
// must get the game's own answer there. While one-machine changes are not allowed (sight_layer.h
// Sight_FireTimeChangesAllowed) the override answers only outside a simulation tick -- the frame's cameras are
// built between ticks -- and not at all in a title whose tick cannot be read. Halo 2's one simulation caller
// stands the override down itself (NativeVehicleFirstPerson_SetStockOnThisThread).
template<unsigned I> bool ForTheSimulation() noexcept
{
    if constexpr (I==1) return false;
    else return !Sight_FireTimeChangesAllowed()&&InTick(I)!=0;
}
template<unsigned I> using Mode=std::conditional_t<I==2,int32_t,int16_t>;
template<unsigned I> using SelectorFn=uint32_t(__fastcall*)(uint32_t,Mode<I>*);
// Keep the guarded feature body separate from native forwarding/finally. A
// fault in the optional marker query must be handled here, never swallowed by
// or attributed to the native camera call.
template<unsigned I> bool SelectOwned(uint32_t unit,Mode<I>* mode) noexcept
{
    __try {
        NativeVehicleCameraOwner before{},after{};float point[3]{};
#if HALOMCCVR_DEV_DIAGNOSTICS
        if (mode) runtime[I].modeSeen[(*mode>=0&&*mode<=4)?unsigned(*mode):5u].fetch_add(1,std::memory_order_relaxed);
#endif
        if (!mode||*mode!=2||!Owner(I,unit,before)) return false;
        if (!Head(I,before,point,false)) {runtime[I].missingMarker.fetch_add(1,std::memory_order_relaxed);return false;}
        return Owner(I,unit,after)&&before==after;
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        Fault(I);return false;
    }
}
template<unsigned I> void PositionOwned(uint32_t unit,void* result) noexcept
{
    __try {
        NativeVehicleCameraOwner before{},after{};float point[3]{};Mode<I> mode{};
        if (!result||!Owner(I,unit,before)) return;
        reinterpret_cast<SelectorFn<I>>(runtime[I].original[Selector])(unit,&mode);
        if (mode!=2) return;
        if (!Head(I,before,point,true)) {runtime[I].missingMarker.fetch_add(1,std::memory_order_relaxed);return;}
        if (!Owner(I,unit,after)||before!=after) return;
#if HALOMCCVR_DEV_DIAGNOSTICS
        if constexpr (I==1) NoteSeatHead(before);
        // The native camera point, before the override replaces it (NativeVehicleFirstPerson_SeatEyeDelta).
        [[maybe_unused]] float native[3]{};
        if constexpr (I==2) std::memcpy(native,static_cast<const uint8_t*>(result)+4,12);
#endif
        // After the full native evaluation, including any slave-turret marker.
        std::memcpy(static_cast<uint8_t*>(result)+4,point,12);
        auto& r=runtime[I];
#if HALOMCCVR_DEV_DIAGNOSTICS
        if constexpr (I==2) {
            const float delta[3]{native[0]-point[0],native[1]-point[1],native[2]-point[2]};
            if (std::isfinite(delta[0])&&std::isfinite(delta[1])&&std::isfinite(delta[2])) {
                if (!r.eyeWriting.test_and_set(std::memory_order_acquire)) {
                    r.eyeVersion.fetch_add(1,std::memory_order_acq_rel);   // odd: writing
                    r.eyeGeneration.store(before.generation,std::memory_order_relaxed);
                    r.eyeParent.store(before.parent,std::memory_order_relaxed);
                    r.eyeSeat.store(before.seat,std::memory_order_relaxed);
                    for (unsigned axis=0;axis<3;++axis) r.eyeDelta[axis].store(delta[axis],std::memory_order_relaxed);
                    r.eyeMs.store(GetTickCount64(),std::memory_order_relaxed);
                    r.eyeVersion.fetch_add(1,std::memory_order_release);   // even: published
                    r.eyeWriting.clear(std::memory_order_release);
                    r.eyePublished.fetch_add(1,std::memory_order_relaxed);
                }
                else r.eyeSkipped.fetch_add(1,std::memory_order_relaxed);
            }
        }
#endif
        // Menu edits the last positively identified seat in this generation.
        // Opening F1 pauses native ownership, so a time expiry would otherwise
        // silently rebind a slider to the per-game base during a drag.
        if(!r.trimWriting.test_and_set(std::memory_order_acquire)) {
            r.trimVersion.fetch_add(1,std::memory_order_acq_rel);
            r.trimIdentity.store(before.vehicleIdentity,std::memory_order_relaxed);
            r.trimSeat.store(before.seat,std::memory_order_relaxed);
            r.trimGeneration.store(before.generation,std::memory_order_relaxed);
            r.trimVersion.fetch_add(1,std::memory_order_release);
            r.trimWriting.clear(std::memory_order_release);
        }
        // Which vehicle model and seat this is, once per change: the name a seat's own camera offsets are
        // saved under (config: vehicle_model_<game>_<model>_seat<N>_forward_m / _up_m / _right_m).
        {
            static std::atomic<uint64_t> named{0};
            const uint64_t key=before.vehicleIdentity^(static_cast<uint64_t>(before.seat+1)<<58);
            if (named.exchange(key,std::memory_order_relaxed)!=key)
                LOG("Vehicle first person title=%u: the camera is in seat %d of vehicle model %016llx",
                    unsigned(kBindings[I].title),int(before.seat),
                    static_cast<unsigned long long>(before.vehicleIdentity));
        }
        runtime[I].positioned.fetch_add(1,std::memory_order_relaxed);
        r.positionedParent.store(before.parent,std::memory_order_relaxed);
        r.positionedSeat.store(before.seat,std::memory_order_relaxed);
        r.positionedMs.store(GetTickCount64(),std::memory_order_release);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        Fault(I);
    }
}
// NativeVehicleFirstPerson_SetStockOnThisThread: the hooks below forward and change nothing while it is set.
thread_local bool t_stockOnThisThread=false;
template<unsigned I> uint32_t __fastcall SelectHook(uint32_t unit,Mode<I>* mode)
{
    auto& r=runtime[I];r.callbacks.fetch_add(1,std::memory_order_acq_rel);
    uint32_t result=0;
    __try {
        result=reinterpret_cast<SelectorFn<I>>(r.original[Selector])(unit,mode);
#if HALOMCCVR_DEV_DIAGNOSTICS
        const uint32_t nativeResult=result;
#endif
        bool owned=false;
        if (!t_stockOnThisThread&&SelectOwned<I>(unit,mode)) {
            if (ForTheSimulation<I>()) r.simulationCalls.fetch_add(1,std::memory_order_relaxed);
            else owned=true;
        }
        if (owned) {result=0;r.selected.fetch_add(1,std::memory_order_relaxed);}
#if HALOMCCVR_DEV_DIAGNOSTICS
        if constexpr (I==2) {
            if (t_selectWitnessArmed) {
                auto& w=t_selectWitness;
                ++w.calls;w.unit=unit;w.nativeResult=nativeResult;w.result=result;
                w.mode=mode?int32_t(*mode):-1;w.overridden=owned;
            }
        }
#endif
    } __finally {r.callbacks.fetch_sub(1,std::memory_order_release);}
    return result;
}
template<unsigned I> void __fastcall CameraHook(uint32_t unit,const void* facing,void* result)
{
    auto& r=runtime[I];r.callbacks.fetch_add(1,std::memory_order_acq_rel);
    __try {
        reinterpret_cast<CameraFn>(r.original[Camera])(unit,facing,result);
        if (!t_stockOnThisThread&&!ForTheSimulation<I>()) PositionOwned<I>(unit,result);
    } __finally {r.callbacks.fetch_sub(1,std::memory_order_release);}
}
void* const hooks[3][2]{
    {reinterpret_cast<void*>(&SelectHook<0>),reinterpret_cast<void*>(&CameraHook<0>)},
    {reinterpret_cast<void*>(&SelectHook<1>),reinterpret_cast<void*>(&CameraHook<1>)},
    {reinterpret_cast<void*>(&SelectHook<2>),reinterpret_cast<void*>(&CameraHook<2>)}
};
bool Prove(uintptr_t base,size_t size,const Binding& binding)
{
    if (binding.rva>=size) return false;
    const auto hit=sig::Find(base,size,binding.pattern);
    return hit==base+binding.rva&&!sig::Find(hit+1,base+size-hit-1,binding.pattern);
}
bool Retire(unsigned i)
{
    auto& r=runtime[i];r.requested.store(false,std::memory_order_release);
    for (unsigned j=0;j<2;++j) if (r.enabled[j]) {
        const auto status=MCCVR_DisableHookForRetirement(r.target[j]);
        if (status!=MH_OK&&status!=MH_ERROR_DISABLED) return false;
        r.enabled[j]=false;
    }
    // Only these two entries can enter this feature from the engine. Their
    // callback lease encloses Current/Owner/Head/Fault, including exception
    // cleanup. Requiring unwind metadata for the leaf Fault helper made every
    // retirement fail permanently in Release and retained the old title DLL.
    // The ingress ranges still cover the window before callback acquisition;
    // the count protects every nested helper and original until it returns.
    const void* functions[]{hooks[i][0],hooks[i][1]};
    const void* originals[]{r.original[0],r.original[1]};
    if (!WaitForNativeDetourQuiescence(functions,originals,2,r.callbacks)) return false;
    for (unsigned j=0;j<2;++j) if (r.target[j]) {
        if (MH_RemoveHook(r.target[j])!=MH_OK) return false;
        r.target[j]=r.original[j]=nullptr;
    }
    if (r.module) FreeLibrary(r.module);
    r.module=nullptr;r.base=0;r.generation=0;r.marker=r.markerInterpolated=nullptr;
    r.tlsIndex=nullptr;r.tickSlot=r.tickByte=0;
    r.trimIdentity.store(0);r.trimGeneration.store(0);r.trimSeat.store(-1);
    r.attempted=r.ready=false;r.faulted.store(false,std::memory_order_release);return true;
}
// Halo 4's tick loop where it marks a tick begun: mov r8d, <slot>; inc ecx; mov r8, [r8+rsi]; mov byte ptr
// [r8+<byte>], 1 -- rsi the thread's engine TLS block. The slot and the byte are the match's own operands, and the
// module's TLS index is its PE TLS directory's.
// ADDR-HYGIENE: resolved (one signature that must match exactly once, decoded; the PE TLS directory). Unresolved
// fails closed: while one-machine changes are not allowed the override then answers nothing (ForTheSimulation).
constexpr char kHalo4TickBegun[]="41 B8 ?? ?? ?? ?? FF C1 4D 8B 04 30 41 C6 40 ?? 01";
void ResolveTick(unsigned i,size_t size)
{
    auto& r=runtime[i];r.tlsIndex=nullptr;r.tickSlot=r.tickByte=0;
    if (kBindings[i].title!=GameTitle::Halo4) return;
    const auto hit=sig::Find(r.base,size,kHalo4TickBegun);
    if (!hit||sig::Find(hit+1,r.base+size-hit-1,kHalo4TickBegun)) return;
    uint32_t slot=0;std::memcpy(&slot,reinterpret_cast<const void*>(hit+2),sizeof(slot));
    const uint8_t byte=*reinterpret_cast<const uint8_t*>(hit+15);
    const auto* dos=reinterpret_cast<const IMAGE_DOS_HEADER*>(r.base);
    const auto* nt=reinterpret_cast<const IMAGE_NT_HEADERS64*>(r.base+dos->e_lfanew);
    const auto& directory=nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_TLS];
    if (!slot||slot>0x4000||(slot&7)||byte>0x7F||!directory.VirtualAddress||directory.VirtualAddress>=size||
        size-directory.VirtualAddress<sizeof(IMAGE_TLS_DIRECTORY64)) return;
    const auto* tls=reinterpret_cast<const IMAGE_TLS_DIRECTORY64*>(r.base+directory.VirtualAddress);
    if (tls->AddressOfIndex<r.base||tls->AddressOfIndex-r.base>=size||
        size-(tls->AddressOfIndex-r.base)<sizeof(uint32_t)) return;
    r.tickSlot=slot;r.tickByte=byte;r.tlsIndex=reinterpret_cast<const uint32_t*>(tls->AddressOfIndex);
}
bool Install(unsigned i,size_t size)
{
    auto& r=runtime[i];
    for (const auto& binding:kBindings[i].entries) if (!Prove(r.base,size,binding)) return false;
    ResolveTick(i,size);
    r.marker=reinterpret_cast<void*>(r.base+kBindings[i].markerRva);
    // H2EK object marker argument 6 selects frame-interpolated object nodes.
    // This exact retail entry is already independently cold-proved above.
    if(i==1)r.markerInterpolated=reinterpret_cast<void*>(r.base+kBindings[i].entries[MarkerProof].rva);
    // Both detours are installed before admission. Partial installation stays
    // stock and is retired through the same quiescence path as a title change.
    for (unsigned j=0;j<2;++j) {
        void* target=reinterpret_cast<void*>(r.base+kBindings[i].entries[j].rva);
        if (MH_CreateHook(target,hooks[i][j],&r.original[j])!=MH_OK) return false;
        r.target[j]=target;
        if (MH_EnableHook(target)!=MH_OK) return false;
        r.enabled[j]=true;
    }
    return true;
}
}
void NativeVehicleFirstPerson_SetStockOnThisThread(bool stock) noexcept
{
    t_stockOnThisThread=stock;
}
bool NativeVehicleFirstPerson_SeatViewOwned(GameTitle title,uint32_t parent,int seat) noexcept
{
    if (title!=TitleAdapter_GetActiveTitle()||parent==UINT32_MAX||seat<0) return false;
    for (unsigned i=0;i<3;++i) if (kBindings[i].title==title)
    {
        const auto& r=runtime[i];
        if (!r.requested.load(std::memory_order_acquire)||r.faulted.load(std::memory_order_acquire)) return false;
        const uint64_t at=r.positionedMs.load(std::memory_order_acquire),now=GetTickCount64();
        return at&&now>=at&&now-at<=250&&r.positionedParent.load(std::memory_order_relaxed)==parent&&
            r.positionedSeat.load(std::memory_order_relaxed)==seat;
    }
    return false;
}
bool NativeVehicleFirstPerson_TrimTarget(GameTitle title,uint64_t& identity,int& seat) noexcept
{
    identity=0;seat=-1;
    if(title!=TitleAdapter_GetActiveTitle())return false;
    for(unsigned i=0;i<3;++i)if(kBindings[i].title==title)
    {
        auto& r=runtime[i];const auto before=r.trimVersion.load(std::memory_order_acquire);
        if(before&1u)return false;
        const auto candidate=r.trimIdentity.load(std::memory_order_relaxed);
        const auto candidateSeat=r.trimSeat.load(std::memory_order_relaxed);
        const auto generation=r.trimGeneration.load(std::memory_order_relaxed);
        if(before!=r.trimVersion.load(std::memory_order_acquire)||!candidate||candidateSeat<0||candidateSeat>31||
            !generation||generation!=TitleAdapter_GetGeneration(title))return false;
        identity=candidate;seat=candidateSeat;return true;
    }
    return false;
}
#if HALOMCCVR_DEV_DIAGNOSTICS
bool NativeVehicleFirstPerson_SeatEyeDelta(GameTitle title,NativeVehicleSeatEyeDelta& out) noexcept
{
    constexpr unsigned i=2;   // Halo 4 only
    if (title!=kBindings[i].title||title!=TitleAdapter_GetActiveTitle()) return false;
    const auto& r=runtime[i];
    if (!r.requested.load(std::memory_order_acquire)||r.faulted.load(std::memory_order_acquire)) return false;
    for (int attempt=0;attempt<2;++attempt) {
        const uint32_t before=r.eyeVersion.load(std::memory_order_acquire);
        if (!before||(before&1u)) continue;
        NativeVehicleSeatEyeDelta d{};
        d.generation=r.eyeGeneration.load(std::memory_order_relaxed);
        d.parent=r.eyeParent.load(std::memory_order_relaxed);
        d.seat=static_cast<int16_t>(r.eyeSeat.load(std::memory_order_relaxed));
        for (unsigned axis=0;axis<3;++axis) d.delta[axis]=r.eyeDelta[axis].load(std::memory_order_relaxed);
        d.ms=r.eyeMs.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (r.eyeVersion.load(std::memory_order_relaxed)!=before) continue;
        if (!d.ms||d.generation!=TitleAdapter_GetGeneration(title)||d.parent==UINT32_MAX||d.seat<0) return false;
        out=d;return true;
    }
    return false;
}
void NativeVehicleFirstPerson_ArmSelectWitness() noexcept
{
    t_selectWitness={};t_selectWitnessArmed=true;
}
NativeVehicleSelectWitness NativeVehicleFirstPerson_TakeSelectWitness() noexcept
{
    t_selectWitnessArmed=false;return t_selectWitness;
}
#endif
void NativeVehicleFirstPerson_Poll()
{
    const auto active=TitleAdapter_GetActiveTitle();const uint64_t now=GetTickCount64();
    for (unsigned i=0;i<3;++i) {
        auto& r=runtime[i];const auto title=kBindings[i].title;
        const auto generation=TitleAdapter_GetGeneration(title);
        if (r.module&&(active!=title||generation!=r.generation)) {
            if (!Retire(i)) {
                if (now>=r.reportAt) {r.reportAt=now+2000;LOG("Vehicle first person title=%u: cleanup pending; feature stock",unsigned(title));}
                continue;
            }
        }
        if (active!=title||!generation) continue;
        if (!r.attempted&&g_config.vehicle_first_person) {
            const auto* descriptor=TitleRegistry_Find(title);
            if (!descriptor||!GetModuleHandleExW(0,descriptor->moduleName,&r.module)) continue;
            r.base=reinterpret_cast<uintptr_t>(r.module);r.generation=generation;r.attempted=true;
            const auto* dos=reinterpret_cast<const IMAGE_DOS_HEADER*>(r.base);
            const auto* nt=reinterpret_cast<const IMAGE_NT_HEADERS64*>(r.base+dos->e_lfanew);
            r.ready=Install(i,nt->OptionalHeader.SizeOfImage);
            LOG("Vehicle first person %s: %s; existing toggle, local seated head marker, native entry/exit; headset verification pending",
                descriptor->displayName,r.ready?"installed":"STOCK FALLBACK (binding/hook)");
            if (r.ready&&title!=GameTitle::Halo2)
                LOG("Vehicle first person %s with co-op safe shots on: %s",descriptor->displayName,r.tlsIndex
                    ?"inside a simulation tick the seat's camera is the game's own (every machine's targeting and "
                     "fire start from it); the frame's is the one placed here"
                    :"this game's simulation tick cannot be told from its frame, so seats keep the game's own camera");
        }
        r.requested.store(r.ready&&g_config.vehicle_first_person,std::memory_order_release);
        if (r.module&&now>=r.reportAt) {
            r.reportAt=now+5000;
            LOG("Vehicle first person title=%u enabled=%d selected=%llu positioned=%llu missingHead=%llu faults=%llu "
                "simulation=%llu (co-op safe: selector calls for our seat answered as the game built them)%s",
                unsigned(title),r.requested.load()?1:0,r.selected.load(),r.positioned.load(),
                r.missingMarker.load(),r.faults.load(),r.simulationCalls.load(),
                r.faulted.load()?" STOCK FALLBACK (guarded access fault)":"");
#if HALOMCCVR_DEV_DIAGNOSTICS
            uint32_t modes[6]{},why[7]{};
            for (unsigned k=0;k<6;++k) modes[k]=r.modeSeen[k].exchange(0,std::memory_order_relaxed);
            for (unsigned k=0;k<7;++k) why[k]=r.ownerWhy[k].exchange(0,std::memory_order_relaxed);
            LOG("Vehicle first person title=%u why: native mode 0/1/2/3/4/other %u/%u/%u/%u/%u/%u; owner ok %u, "
                "not current %u, bad unit %u, read refused %u, unit mismatch %u (selector 0x%08X vs local 0x%08X), "
                "generation %u, no seat %u",unsigned(title),modes[0],modes[1],modes[2],modes[3],modes[4],modes[5],
                why[OwnerOk],why[OwnerNotCurrent],why[OwnerBadUnit],why[OwnerRead],why[OwnerUnitMismatch],
                r.lastSelectorUnit.load(),r.lastOwnerUnit.load(),why[OwnerGeneration],why[OwnerNoSeat]);
            if (title==GameTitle::Halo2) ReportSeatHead();
            if (title==GameTitle::Halo4) {
                Game_Halo4CameraOwnerReport();
                const uint64_t published=r.eyePublished.exchange(0,std::memory_order_relaxed);
                const uint64_t skipped=r.eyeSkipped.exchange(0,std::memory_order_relaxed);
                if (published||skipped)
                    LOG("Vehicle first person title=%u: native seat eye offsets published %llu (a concurrent "
                        "writer skipped %llu)",unsigned(title),static_cast<unsigned long long>(published),
                        static_cast<unsigned long long>(skipped));
            }
#endif
        }
    }
}
