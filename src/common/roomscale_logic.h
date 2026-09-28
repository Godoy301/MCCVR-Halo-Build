#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "runtime_types.h"

// Roomscale has its own positive native on-foot camera proof. The historical
// shared-feature gate is H3-only and is not this feature's admission policy.
inline bool RoomscaleGameplayEligible(GameTitle title, RuntimeMode mode) noexcept
{
    return mode == RuntimeMode::Gameplay &&
        (title == GameTitle::Halo2 || title == GameTitle::Halo3 ||
         title == GameTitle::Halo3ODST || title == GameTitle::HaloReach ||
         title == GameTitle::Halo4);
}

// Native locomotion owns collision, steps, ground support and networking.
// Consume observed follow motion and its bounded native stopping tail:
// body position + remaining tracked lean then represents one physical step.
struct RoomscaleFollow
{
    // Follow hysteresis (Halo: Campaign Evolved's rule): a catch-up starts only once the head is
    // kFollowStart from the body and, once started, runs until it is inside kFollowStop. With a
    // single 1.5 cm threshold, ordinary head sway fired a stream of ~1 cm body steps (measured in
    // Reach: a 2.5 cm lean moved the body 13 mm in one burst), each a walk-cycle blip. The view is
    // head-locked either way; only the collision body lags, by at most kFollowStart.
    static constexpr float kFollowStart=0.03f, kFollowStop=0.015f;
    // How long after the last command the body's own stopping tail may still be consumed as ours (it
    // also ends after 150 ms without motion). Commands stop at the edge of the stop band, so this
    // covers a lagged native walker coasting in from there; it is still a hard cap, so later platform
    // or physics travel is never claimed.
    static constexpr uint64_t kSettleMaxMs=700;
    bool seeded=false, commanded=false, settling=false;
    bool manualPaused=false, following=false;
    uint32_t generation=0;
    uint64_t time=0, lastCommand=0, lastMotion=0;
    uint64_t manualQuietSince=0;
    float body[2]{}, expectedReference[2]{}, initialLean[2]{};
    float headForward[2]{}, worldForward[2]{}, error[2]{}, velocity[2]{};
    // Tracking-space direction of the last nonzero command: the way the stopping tail must keep moving
    // to be claimed as ours.
    float tailDir[2]{};

    void SuspendForManual(uint64_t now) noexcept
    {
        // Manual travel cannot be attributed to a prior physical-follow
        // packet. Retain the tracked goal, but retire that movement receipt.
        // A follow the stick interrupted is over too: once native travel is
        // quiet again, a catch-up must re-cross kFollowStart like any other,
        // or a residual inside the start band would resume walking.
        commanded=settling=following=false;velocity[0]=velocity[1]=0;
        manualPaused=true;manualQuietSince=now;
    }

    bool Update(uint32_t epoch,uint64_t now,bool enabled,bool manualMove,
        const float position[3],const float head[3],float reference[3],
        float hx,float hz,float wx,float wy,float scale,float& moveX,float& moveY) noexcept
    {
        moveX=moveY=0;
        const float values[]{position[0],position[1],head[0],head[2],reference[0],
            reference[2],hx,hz,wx,wy,scale};
        for (float v:values) if (!std::isfinite(v)) { seeded=false; commanded=settling=manualPaused=following=false; return false; }
        const float hl=std::hypot(hx,hz),wl=std::hypot(wx,wy);
        if (!enabled || !epoch || !now || scale<=0 || hl<0.001f || wl<0.001f)
        { seeded=false; commanded=settling=manualPaused=following=false; return false; }
        hx/=hl; hz/=hl; wx/=wl; wy/=wl;
        const bool reset=!seeded || generation!=epoch || now<time || now-time>250 ||
            std::fabs(reference[0]-expectedReference[0])>0.0001f ||
            std::fabs(reference[2]-expectedReference[1])>0.0001f ||
            std::hypot(position[0]-body[0],position[1]-body[1])/scale>0.35f;
        if (reset)
        {
            initialLean[0]=head[0]-reference[0];
            initialLean[1]=head[2]-reference[2];
            commanded=settling=following=false; velocity[0]=velocity[1]=0;
            lastCommand=lastMotion=now; seeded=true;
            manualPaused=false;
        }
        if (manualMove) SuspendForManual(now);
        else if (manualPaused)
        {
            // A released stick may still be braking. Wait for observed native
            // motion to remain quiet before creating a new follow receipt;
            // continuing platform/camera movement extends this bounded wait.
            if (std::hypot(position[0]-body[0],position[1]-body[1])/scale>0.0001f)
                manualQuietSince=now;
            if (now>=manualQuietSince&&now-manualQuietSince>=150)
                manualPaused=false;
        }
        else if (!reset && (commanded || (settling && now>=lastCommand && now-lastCommand<=kSettleMaxMs &&
            now>=lastMotion && now-lastMotion<=150)))
        {
            const float dx=(position[0]-body[0])/scale;
            const float dy=(position[1]-body[1])/scale;
            const float forward=dx*worldForward[0]+dy*worldForward[1];
            const float right=dx*worldForward[1]-dy*worldForward[0];
            const float tx=forward*headForward[0]-right*headForward[1];
            const float tz=forward*headForward[1]+right*headForward[0];
            // Native acceleration and braking can move after the last nonzero
            // stick request, including past the target. Account for that bounded
            // tail in the SAME tracking frame instead of adding it to the view.
            // Epoch/recenter/teleport/manual-input guards above still abandon
            // authority. Never consume a requested distance before it occurred.
            // The tail (no command on the last update) is ours only while it keeps moving the way we
            // walked: travel across or against that direction is a platform, a knockback or other
            // physics, and it ends the tail unclaimed (the view then carries it, as it should).
            if (!commanded && (tx!=0 || tz!=0) && tx*tailDir[0]+tz*tailDir[1]<=0)
            {
                settling=false; velocity[0]=velocity[1]=0;
            }
            else
            {
                reference[0]+=tx; reference[2]+=tz;
                const float elapsed=now>time?float(now-time)*.001f:0;
                if(elapsed>0)
                {
                    const float blend=1-std::exp(-elapsed/.06f);
                    velocity[0]+=(tx/elapsed-velocity[0])*blend;
                    velocity[1]+=(tz/elapsed-velocity[1])*blend;
                }
                if(tx!=0 || tz!=0)lastMotion=now;
            }
        }
        else velocity[0]=velocity[1]=0;
        error[0]=head[0]-reference[0]-initialLean[0];
        error[1]=head[2]-reference[2]-initialLean[1];
        const float distance=std::hypot(error[0],error[1]);
        if (distance>1.2f)
        {
            // A tracking jump must not become a long unattended walk.
            initialLean[0]=head[0]-reference[0];
            initialLean[1]=head[2]-reference[2];
            error[0]=error[1]=0;velocity[0]=velocity[1]=0;settling=following=false;
        }
        else
        {
            following=!manualMove && !manualPaused &&
                (distance>kFollowStart || (following && distance>kFollowStop));
            // Command ONLY while a follow is running. Once it ends inside kFollowStop, the game's own
            // deceleration finishes the step: `settling` below still consumes that tail into the
            // reference, but a command here would be floored by MCC to a >=27% stick -- a fresh walk
            // step inside the band the hysteresis exists to keep quiet. A real overshoot past
            // kFollowStart starts a new follow, which corrects it.
            if (following)
            {
                // Position feedback follows promptly; measured native velocity
                // brakes before overshoot. A short filter tolerates 30 Hz body
                // updates under higher-frequency camera callbacks.
                float cx=error[0]*3.5f-velocity[0]*.18f;
                float cz=error[1]*3.5f-velocity[1]*.18f;
                // Brake by RELEASING, never by reversing. Near the target the velocity term turns
                // the command against the error, and MCC's move-stick floor (any nonzero command
                // is sent as >=27% stick) makes that a real walk the other way: measured in Reach
                // FA, a 0.3 m step braked from 4 mm into an 18 mm reverse residual. Releasing lets
                // the game's own deceleration stop the unit. A genuine overshoot still corrects,
                // because the error itself has flipped and the command again points along it.
                if(cx*error[0]+cz*error[1]<=0){cx=cz=0;}
                const float magnitude=std::hypot(cx,cz);
                if(magnitude>0){tailDir[0]=cx/magnitude;tailDir[1]=cz/magnitude;}
                if(magnitude>.8f){cx*=.8f/magnitude;cz*=.8f/magnitude;}
                moveX=-cx*hz+cz*hx;
                moveY=cx*hx+cz*hz;
            }
        }
        commanded=moveX!=0 || moveY!=0;
        if(commanded)lastCommand=now;
        // Do not claim unrelated later platform/physics travel. Settling ends
        // after a quiet interval and always expires after half a second without
        // a command. Fresh input/tracking admission is still required outside.
        settling=commanded || (settling && now>=lastCommand && now-lastCommand<=kSettleMaxMs &&
            now>=lastMotion && now-lastMotion<=150);
        generation=epoch; time=now;
        body[0]=position[0]; body[1]=position[1];
        expectedReference[0]=reference[0]; expectedReference[1]=reference[2];
        headForward[0]=hx; headForward[1]=hz;
        worldForward[0]=wx; worldForward[1]=wy;
        return true;
    }
};
