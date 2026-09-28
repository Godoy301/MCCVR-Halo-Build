#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Tracking-space height only: native walking, slopes and moving platforms do
// not alter this reference. Recenter/enable samples the player's normal height.
class PhysicalCrouch
{
public:
    // A height more than this far above the calibration sample is not a head at all (a headset lifted
    // off, or set down on a shelf, or a seated player standing up) and is never learned.
    static constexpr float kMaxRise = .30f;
    // What IS learned is capped below the release margin, so the posture the player calibrated in can
    // never read as a crouch afterwards: a seated player who half-rises for a second and sits back down
    // stays standing. (The cap is re-derived from the depth, which recalibrates when it changes.)
    float MaxLearnedRise() const noexcept {return std::min(kMaxRise,threshold_*.70f-.02f);}
    // A rise must be held this steady (metres) for this long (ms) before it counts.
    static constexpr float kRiseSteady = .02f;
    static constexpr uint64_t kRiseHoldMs = 1000;

    bool Update(float height,uint64_t epoch,uint64_t now,bool available,float depth) noexcept
    {
        const float threshold=std::isfinite(depth)?std::clamp(depth,.08f,.65f):.22f;
        if(!available||!std::isfinite(height)||!epoch||!now)
        {active_=false;suspended_=true;last_=0;riseSince_=0;return false;}
        if(!initialized_||epoch_!=epoch||threshold_!=threshold)
        {
            initialized_=true;epoch_=epoch;threshold_=threshold;baseline_=calibrated_=height;riseCapped_=false;
            active_=false;suspended_=false;last_=now;riseSince_=0;return false;
        }
        if(last_&&(now<last_||now-last_>250)) {active_=false;suspended_=true;riseSince_=0;}
        last_=now;
        LearnStanding(height,now);
        const float down=baseline_-height;
        const float release=threshold*.70f;
        // Resume must never replay a crouch that began in a menu or vehicle.
        // Stand up first; toggling/recentering explicitly calibrates anew.
        if(suspended_)
        {if(down<=release)suspended_=false;return false;}
        if(active_)
        {if(down<=release)active_=false;}
        else if(down>=threshold) active_=true;
        return active_;
    }
    void Reset() noexcept {*this=PhysicalCrouch{};}
    // Diagnostics only (the input hook's 1 Hz crouch line).
    float Baseline() const noexcept {return baseline_;}
    float Calibrated() const noexcept {return calibrated_;}
    bool Active() const noexcept {return active_;}
    bool Suspended() const noexcept {return suspended_;}
    bool Initialized() const noexcept {return initialized_;}
    // True once this calibration saw a steady height above what may be learned (the rise was capped):
    // the reference then sits below the player's standing height, and calibrating again while standing
    // tall gives them the full depth back. The menu shows it.
    bool RiseCapped() const noexcept {return riseCapped_;}
private:
    // Learn the standing height UPWARD only -- the rule Halo: Campaign Evolved's physical crouch uses,
    // adapted to tracking space (no floor here). A calibration taken while slouched or half-crouched
    // (enable, recenter, the Calibrate button) set the line too low: every later crouch then read short,
    // or never triggered at all. Raise the baseline to a height held steady for a second, never above
    // the calibration + MaxLearnedRise(), never while crouched, and ignore heights beyond kMaxRise. It
    // never falls: a long crouch cannot drag the line down after it, and only a new calibration (epoch)
    // resets it.
    void LearnStanding(float height,uint64_t now) noexcept
    {
        if(active_||height<=baseline_+kRiseSteady||height>calibrated_+kMaxRise)
        {riseSince_=0;return;}
        if(!riseSince_||std::fabs(height-riseValue_)>kRiseSteady)
        {riseSince_=now;riseValue_=height;return;}
        if(now>=riseSince_&&now-riseSince_>=kRiseHoldMs)
        {
            const float learned=std::min(riseValue_,height),cap=calibrated_+MaxLearnedRise();
            if(learned>cap) riseCapped_=true;
            baseline_=std::max(baseline_,std::min(learned,cap));
            riseSince_=0;
        }
    }

    uint64_t epoch_{},last_{},riseSince_{};
    float baseline_{},calibrated_{},threshold_{},riseValue_{};
    bool initialized_{},active_{},suspended_{},riseCapped_{};
};

// A new calibration or tracking-space epoch re-samples the standing height. A runtime recenter announces
// its new space before poses arrive in it, so sampling on the next poll can take the height from the OLD
// space -- and a baseline sampled too high never heals (it only rises). Hand the gesture a new epoch
// only once the head has held its height (kSteady) for kSettleMs; until then Epoch() keeps the last
// settled one and Settled() is false, and the caller keeps the gesture unavailable (released, then
// re-armed standing).
class CrouchEpochSettle
{
public:
    static constexpr float kSteady=.02f;
    static constexpr uint64_t kSettleMs=300;
    void Update(uint64_t epochNow,bool tracking,float height,uint64_t now) noexcept
    {
        if(epochNow!=seen_) {seen_=epochNow;since_=now;height_=height;}
        else if(!tracking||!std::isfinite(height)||std::fabs(height-height_)>kSteady) {since_=now;height_=height;}
        settled_=given_==epochNow||(tracking&&now>=since_&&now-since_>=kSettleMs);
        if(settled_) given_=epochNow;
    }
    bool Settled() const noexcept {return settled_;}
    uint64_t Epoch() const noexcept {return given_;}
private:
    uint64_t seen_{},given_{},since_{};
    float height_{};
    bool settled_{};
};
