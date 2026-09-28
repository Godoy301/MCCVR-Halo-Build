#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Tracking-space height only: native walking, slopes and moving platforms do
// not alter this reference. Recenter/enable samples the player's normal height.
class PhysicalCrouch
{
public:
    // The learned standing height may rise at most this far above the calibration sample: a headset
    // lifted off the head, or set down on a shelf, must never become "standing".
    static constexpr float kMaxRise = .30f;
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
            initialized_=true;epoch_=epoch;threshold_=threshold;baseline_=calibrated_=height;
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
private:
    // Learn the standing height UPWARD only -- the rule Halo: Campaign Evolved's physical crouch uses,
    // adapted to tracking space (no floor here). A calibration taken while slouched or half-crouched
    // (enable, recenter, the Calibrate button) set the line too low: every later crouch then read short,
    // or never triggered at all. Raise the baseline to a height held steady for a second, never above
    // the calibration + kMaxRise, never while crouched. It never falls: a long crouch cannot drag the
    // line down after it, and only a new calibration (epoch) resets it.
    void LearnStanding(float height,uint64_t now) noexcept
    {
        if(active_||height<=baseline_+kRiseSteady||height>calibrated_+kMaxRise)
        {riseSince_=0;return;}
        if(!riseSince_||std::fabs(height-riseValue_)>kRiseSteady)
        {riseSince_=now;riseValue_=height;return;}
        if(now>=riseSince_&&now-riseSince_>=kRiseHoldMs)
        {baseline_=std::max(baseline_,std::min(riseValue_,height));riseSince_=0;}
    }

    uint64_t epoch_{},last_{},riseSince_{};
    float baseline_{},calibrated_{},threshold_{},riseValue_{};
    bool initialized_{},active_{},suspended_{};
};
