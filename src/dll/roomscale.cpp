#include "roomscale.h"
#include "vr.h"
#include "../common/roomscale_logic.h"
#include "../common/config.h"
#include "../common/log.h"
#include "title_adapter.h"
#include <windows.h>
#include <atomic>
#include <bit>
#include <cstdio>

namespace {
// Re-enabled after title admission and nested XInput cancellation fixes.
constexpr bool kEnableRoomscaleBodyFollow = true;
std::atomic<uint64_t> inputAt{0}, commandAt{0}, command{0};
std::atomic<bool> inputAllowed{false}, manualMove{false};
std::atomic<uint32_t> inputEpoch{1}, historyEpoch{1}, commandEpoch{0}, commandGeneration{0};
std::atomic<int> commandTitle{0};
std::atomic<uint32_t> version{0};
std::atomic_flag publishing = ATOMIC_FLAG_INIT;
std::atomic<uint64_t> admitted{0}, refused{0}, consumed{0};
std::atomic<uint64_t> nativeBlocked{0}, trackingBlocked{0}, inputBlocked{0};
std::atomic<uint64_t> manualSamples{0}, demandSamples{0}, travelMm{0};
// Follow quality for the report: how far the head is from where the body will be (the controller's
// error), the largest in the window, and the largest stick command issued.
std::atomic<uint64_t> errorLastMm{0}, errorMaxMm{0}, commandMaxMilli{0};
// The A/B switch (roomscale_follow_law): the law the camera hook last ran, for the report.
std::atomic<int> lawInEffect{-1};
// RoomscaleFollow's guards (see kNoResponseMs): stand-downs, and probe travel it refused to claim.
std::atomic<uint64_t> guardStoodDown{0}, guardNotOurs{0};
#if HALOMCCVR_DEV_DIAGNOSTICS
// Follow trace (dev builds only): one sample per ~100 ms while a follow is in progress (error past the
// deadband, a command, or native travel still settling). Written only by the camera hook under
// `publishing`, dumped by the status tick, so a catch-up that lurches, overshoots or stalls shows its
// shape in the log. The 64-slot ring holds ~6 s, three report periods.
struct FollowSample { uint64_t at; float errorX, errorZ, command, speed; };
constexpr unsigned kFollowTrace = 64;
FollowSample followTrace[kFollowTrace]{};
std::atomic<unsigned> followHead{0};
uint64_t followSampledAt = 0;
#endif
}

void Roomscale_Input(bool allowed,float x,float y) noexcept
{
    const auto now=GetTickCount64();
    const auto previousAt=inputAt.load(std::memory_order_acquire);
    const bool expired=previousAt&&(now<previousAt||now-previousAt>100);
    allowed=kEnableRoomscaleBodyFollow && allowed && g_config.roomscale_movement && VR_RoomscaleTrackingFresh();
    const bool manual=!std::isfinite(x)||!std::isfinite(y)||x*x+y*y>0.02f;
    const bool allowedChanged=inputAllowed.exchange(allowed,std::memory_order_acq_rel)!=allowed;
    const bool manualChanged=manualMove.exchange(manual,std::memory_order_acq_rel)!=manual;
    // Retire the command even when a short stick press/release occurs entirely
    // between camera callbacks. That camera cannot observe the manual interval
    // and must not replay a pre-stick body-follow packet after release.
    if (allowedChanged || manualChanged || expired)
        inputEpoch.fetch_add(1,std::memory_order_acq_rel);
    // Admission loss ends the follow in progress (the camera rebases it, keeping what is owed).
    // Manual movement only retires the packet; the camera can retain the tracked step until native
    // travel stops.
    if (allowedChanged || expired) historyEpoch.fetch_add(1,std::memory_order_acq_rel);
    inputAt.store(now,std::memory_order_release);
}

bool Roomscale_Move(float& x,float& y) noexcept
{
    if (!g_config.roomscale_movement || !VR_RoomscaleTrackingFresh() || !std::isfinite(x) || !std::isfinite(y) ||
        x*x+y*y>0.02f || manualMove.load(std::memory_order_acquire) ||
        !inputAllowed.load(std::memory_order_acquire)) return false;
    const auto now=GetTickCount64();
    const auto inputTime=inputAt.load(std::memory_order_acquire);
    if (!inputTime || now<inputTime || now-inputTime>100) return false;
    const auto before=version.load(std::memory_order_acquire);
    if (before&1u) return false;
    const auto at=commandAt.load(std::memory_order_relaxed);
    const auto title=static_cast<GameTitle>(commandTitle.load(std::memory_order_relaxed));
    const auto generation=commandGeneration.load(std::memory_order_relaxed);
    const auto epoch=commandEpoch.load(std::memory_order_relaxed);
    const uint64_t value=command.load(std::memory_order_relaxed);
    if (version.load(std::memory_order_acquire)!=before || !at || now<at || now-at>100 ||
        title!=TitleAdapter_GetActiveTitle() || !generation ||
        generation!=TitleAdapter_GetGeneration(title) || epoch!=inputEpoch.load()) return false;
    const float cx=std::bit_cast<float>(uint32_t(value));
    const float cy=std::bit_cast<float>(uint32_t(value>>32));
    if (!std::isfinite(cx) || !std::isfinite(cy) || cx*cx+cy*cy<=1e-6f) return false;
    x=cx; y=cy;
    consumed.fetch_add(1,std::memory_order_relaxed);
    return true;
}

void Roomscale_Camera(GameTitle title,bool allowed,const float body[3],
    const float head[3],const float q[4],const float forward[3],
    float reference[3],float scale) noexcept
{
    if (title!=TitleAdapter_GetActiveTitle()) return;
    // Fail immediately on overlapping title callbacks; no locks or waits in a hook.
    if (publishing.test_and_set(std::memory_order_acquire)) return;
    // The published tracking reference belongs to the title, not one engine
    // thread. The nonblocking publishing guard above protects this shared
    // history when native camera callbacks migrate between threads.
    // Two follow laws for the A/B switch (roomscale_follow_law): the refined one (default) and the
    // original Alpha 0.6.0 one, verbatim. Only the selected one runs; a switch starts both afresh, so
    // each begins from where the player stands.
    static RoomscaleFollow state;
    static RoomscaleFollowOriginal original;
    static int priorLaw=-1;
    const int law=g_config.roomscale_follow_law ? 1 : 0;
    static GameTitle prior=GameTitle::None;
    static uint32_t priorInputEpoch=0, priorHistoryEpoch=0;
    const auto epoch=inputEpoch.load(std::memory_order_acquire);
    const auto history=historyEpoch.load(std::memory_order_acquire);
    const auto now=GetTickCount64(),at=inputAt.load(std::memory_order_acquire);
    if (law!=priorLaw)
    { state={}; original={}; priorLaw=law; lawInEffect.store(law,std::memory_order_relaxed); }
    // A different title starts over. An admission change (priorHistoryEpoch) only ends the follow in
    // progress: the distance the player is owed survives it (RoomscaleFollow::Rebase). The original
    // law started over there too, as it shipped.
    if (prior!=title)
    { state={}; original={}; prior=title; priorHistoryEpoch=history; }
    else if (priorHistoryEpoch!=history)
    {
        if (law) state.Rebase(); else original={};
        priorHistoryEpoch=history;
    }
    else if (priorInputEpoch!=epoch)
    {
        if (law && state.seeded) state.SuspendForManual(now);
        else if (!law && original.seeded) original.SuspendForManual(now);
    }
    priorInputEpoch=epoch;
    const auto generation=TitleAdapter_GetGeneration(title);
    const bool tracking = VR_RoomscaleTrackingFresh();
    const bool input = at && now>=at && now-at<=100 && inputAllowed.load(std::memory_order_acquire);
    const bool active=kEnableRoomscaleBodyFollow && allowed && g_config.roomscale_movement &&
        tracking && input && title==TitleAdapter_GetActiveTitle();
    if (g_config.roomscale_movement)
    {
        if (!allowed) nativeBlocked.fetch_add(1, std::memory_order_relaxed);
        if (!tracking) trackingBlocked.fetch_add(1, std::memory_order_relaxed);
        if (!input) inputBlocked.fetch_add(1, std::memory_order_relaxed);
    }
    const float hx=-2*(q[3]*q[1]+q[0]*q[2]);
    const float hz=-(1-2*(q[0]*q[0]+q[1]*q[1]));
    float x=0,y=0;
    const float oldRefX=reference[0], oldRefZ=reference[2];
    const bool manual=manualMove.load(std::memory_order_acquire);
    const bool valid=law
        ? state.Update(generation,now,active,manual,body,head,reference,hx,hz,forward[0],forward[1],scale,x,y)
        : original.Update(generation,now,active,manual,body,head,reference,hx,hz,forward[0],forward[1],scale,x,y);
    const float* followError=law ? state.error : original.error;
    if (g_config.roomscale_movement)
    {
        (valid ? admitted : refused).fetch_add(1,std::memory_order_relaxed);
        // The refined law's guards (the original law has none).
        if (law && state.guard==RoomscaleFollow::Guard::StoodDown) guardStoodDown.fetch_add(1,std::memory_order_relaxed);
        else if (law && state.guard==RoomscaleFollow::Guard::NotOurs) guardNotOurs.fetch_add(1,std::memory_order_relaxed);
        if (valid && manual) manualSamples.fetch_add(1,std::memory_order_relaxed);
        if (valid && (x!=0 || y!=0)) demandSamples.fetch_add(1,std::memory_order_relaxed);
        const float distance=std::hypot(reference[0]-oldRefX,reference[2]-oldRefZ);
        if (valid && std::isfinite(distance) && distance<=0.4f)
            travelMm.fetch_add(uint64_t(distance*1000.0f+0.5f),std::memory_order_relaxed);
        if (valid)
        {
            const float err=std::hypot(followError[0],followError[1]);
            const float cmd=std::hypot(x,y);
            if (std::isfinite(err))
            {
                const uint64_t mm=uint64_t(err*1000.0f+0.5f);
                errorLastMm.store(mm,std::memory_order_relaxed);
                uint64_t prior=errorMaxMm.load(std::memory_order_relaxed);
                while (mm>prior && !errorMaxMm.compare_exchange_weak(prior,mm,std::memory_order_relaxed)) {}
            }
            if (std::isfinite(cmd))
            {
                const uint64_t milli=uint64_t(cmd*1000.0f+0.5f);
                uint64_t prior=commandMaxMilli.load(std::memory_order_relaxed);
                while (milli>prior && !commandMaxMilli.compare_exchange_weak(prior,milli,std::memory_order_relaxed)) {}
            }
#if HALOMCCVR_DEV_DIAGNOSTICS
            // Only while a follow is actually running (or stood down, which is a follow held off): a
            // steady lean inside the start band is the normal idle state and must not log a line every
            // report.
            const float* followVelocity=law ? state.velocity : original.velocity;
            const float speed=std::hypot(followVelocity[0],followVelocity[1]);
            const bool inProgress=law ? (state.following || state.settling || state.stalled) : original.settling;
            if (std::isfinite(err) && std::isfinite(cmd) && std::isfinite(speed) &&
                (inProgress || cmd>0) &&
                (now<followSampledAt || now-followSampledAt>=100))
            {
                const unsigned slot=followHead.load(std::memory_order_relaxed);
                followTrace[slot%kFollowTrace]={now,followError[0],followError[1],cmd,speed};
                followHead.store(slot+1,std::memory_order_release);
                followSampledAt=now;
            }
#endif
        }
    }
    version.fetch_add(1,std::memory_order_acq_rel);
    command.store(uint64_t(std::bit_cast<uint32_t>(x))|
        (uint64_t(std::bit_cast<uint32_t>(y))<<32),std::memory_order_relaxed);
    commandTitle.store(static_cast<int>(title),std::memory_order_relaxed);
    commandGeneration.store(generation,std::memory_order_relaxed);
    commandEpoch.store(epoch,std::memory_order_relaxed);
    commandAt.store(now,std::memory_order_relaxed);
    version.fetch_add(1,std::memory_order_release);
    publishing.clear(std::memory_order_release);
}

// Existing non-hook status tick owns logging. Camera/input hooks only count.
void Roomscale_Report() noexcept
{
    static uint64_t last=0;
    static bool previous=false;
    const auto now=GetTickCount64();
    const bool enabled=g_config.roomscale_movement;
    if (enabled!=previous)
    {
        LOG("Roomscale body movement %s: native walking, head-relative movement; physical steps during VR-stick travel wait for native quiet before catch-up",
            enabled ? "ON" : "OFF");
        previous=enabled;
    }
    static int previousLaw=-2;
    const int law=g_config.roomscale_follow_law ? 1 : 0;
    if (enabled && law!=previousLaw)
    {
        LOG("Roomscale follow: %s (A/B switch roomscale_follow_law = %d)",
            law ? "REFINED (hysteresis 3/1.5 cm, brake by release, owed distance kept)"
                : "ORIGINAL (Alpha 0.6.0: 1.5 cm threshold, reverse brake)", law);
        previousLaw=law;
    }
    if (now-last<2000) return;
    last=now;
    const auto good=admitted.exchange(0),bad=refused.exchange(0),moves=consumed.exchange(0);
    const auto native=nativeBlocked.exchange(0), tracking=trackingBlocked.exchange(0), input=inputBlocked.exchange(0);
    const auto manual=manualSamples.exchange(0), demand=demandSamples.exchange(0), travel=travelMm.exchange(0);
    const auto errLast=errorLastMm.load(std::memory_order_relaxed), errMax=errorMaxMm.exchange(0);
    const auto cmdMax=commandMaxMilli.exchange(0);
    if (enabled)
        LOG("Roomscale: title=%u admitted=%llu unavailable=%llu movement polls=%llu; "
            "blocked native=%llu tracking=%llu input=%llu manual=%llu demand=%llu consumed-mm=%llu; "
            "follow error now=%llu max=%llu mm, stick cmd max=%.2f; law=%s; %s",
            unsigned(TitleAdapter_GetActiveTitle()), (unsigned long long)good,
            (unsigned long long)bad,(unsigned long long)moves,
            (unsigned long long)native,(unsigned long long)tracking,(unsigned long long)input,
            (unsigned long long)manual,(unsigned long long)demand,(unsigned long long)travel,
            (unsigned long long)errLast,(unsigned long long)errMax,double(cmdMax)*0.001,
            lawInEffect.load(std::memory_order_relaxed)==0 ? "original" : "refined",
            good ? "native body follow available" :
            "StockFallback: awaiting fresh on-foot camera, tracking and gameplay input; VR stays active");
    const auto stoodDown=guardStoodDown.exchange(0), notOurs=guardNotOurs.exchange(0);
    if (enabled && (stoodDown || notOurs))
        LOG("Roomscale guard: stood down %llu time(s) (the character ignored the follow's stick for %llu ms); "
            "%llu move(s) while probing it again were the game moving him, not counted as walking",
            (unsigned long long)stoodDown,(unsigned long long)RoomscaleFollow::kNoResponseMs,
            (unsigned long long)notOurs);
#if HALOMCCVR_DEV_DIAGNOSTICS
    static unsigned printed=0;
    const unsigned head=followHead.load(std::memory_order_acquire);
    if (enabled && head!=printed)
    {
        if (head-printed>kFollowTrace) printed=head-kFollowTrace;
        char line[1024];
        // The error is a tracking-space vector (x right, z back), so a catch-up that walks at an
        // angle to it shows up as the other component growing.
        int used=snprintf(line,sizeof line,
            "Roomscale trace (error x,z mm / stick / speed mm/s, ~100 ms apart; | = gap):");
        uint64_t previousAt=0;
        for (unsigned i=printed; i!=head && used>0 && used<int(sizeof line)-48; ++i)
        {
            const FollowSample s=followTrace[i%kFollowTrace];
            if (previousAt && s.at-previousAt>250) used+=snprintf(line+used,sizeof line-used," |");
            used+=snprintf(line+used,sizeof line-used," %.0f,%.0f/%.2f/%.0f",
                double(s.errorX)*1000.0,double(s.errorZ)*1000.0,double(s.command),double(s.speed)*1000.0);
            previousAt=s.at;
        }
        LOG("%s",line);
        printed=head;
    }
#endif
}
