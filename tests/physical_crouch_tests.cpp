#include "../src/common/physical_crouch_logic.h"
#include "../src/common/physical_crouch_native_read.h"
#include "../src/common/standing_height_follow.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
static unsigned checks{};
static void Check(bool value,const char* label)
{++checks;if(!value){std::fprintf(stderr,"FAIL: %s\n",label);std::exit(1);}}
template<class T> static void Put(uint8_t* bytes,unsigned at,T value)
{std::memcpy(bytes+at,&value,sizeof(value));}
int main()
{
    for(bool odst:{false,true})
    {
        uint8_t unit[0x2400]{},definition[0x400]{};
        const unsigned parent=odst?0x10:0x14,scale=odst?0x8C:0xA0,
            fraction=odst?0x398:0x30C,height=odst?0x310:0x218;
        Put(unit,parent,UINT32_MAX);Put(unit,scale,2.f);
        Put(definition,height,1.f);Put(definition,odst?0x318:0x21C,.6f);
        Put(unit,0x15A,int16_t(0x600));
        const auto read=odst?physical_crouch_native::Odst:physical_crouch_native::Halo2;
        for(float blend:{0.f,.25f,.5f,.75f,1.f})
        {
            Put(unit,fraction,blend);float reduction=-1;
            Check(read(unit,definition,reduction)&&std::fabs(reduction-.8f*blend)<.00001f,
                "own-title native crouch height interpolation and scale");
        }
        float reduction=0;
        Put(unit,parent,uint32_t(0x12340001));
        Check(!read(unit,definition,reduction),"seated native camera is never compensated");
        Put(unit,parent,UINT32_MAX);
        Put(unit,fraction,std::numeric_limits<float>::quiet_NaN());
        Check(!read(unit,definition,reduction),"invalid native crouch blend rejected");
        Put(unit,fraction,1.f);
        if(odst)
        {
            Put(unit,0x600,int32_t(7));Check(!read(unit,definition,reduction),"tilted ODST up-vector keeps native camera");
            Put(unit,0x600,int32_t(0));unit[0x516]=6;Put(unit,0x51C,uint32_t(0x10));
            Check(read(unit,definition,reduction)&&reduction==0,"ODST special standing state has no crouch reduction");
            unit[0x516]=0;Put(unit,0x51C,uint32_t(0));
            Put(unit,0x15A,int16_t(0x2310));Put(unit,0x2310,int32_t(0));
            Check(read(unit,definition,reduction)&&reduction>0,"ODST physics block past 8 KB (the Halo 3 live layout) is read");
            Put(unit,0x2310,int32_t(8));Check(!read(unit,definition,reduction),"ODST tilted mode found past 8 KB still refused");
            Put(unit,0x15A,int16_t(-1));Check(!read(unit,definition,reduction),"absent ODST physics block refused");
            Put(unit,0x15A,int16_t(0x400));Check(!read(unit,definition,reduction),"ODST physics block overlapping the unit's fields refused");
            Put(unit,0x15A,int16_t(0x600));
        }
        else
        {
            Put(unit,0x390,uint16_t(0x2000));Check(!read(unit,definition,reduction),"H2 animation special state conservatively refused");
        }
        Check(!read(nullptr,definition,reduction)&&!read(unit,nullptr,reduction),"missing native storage leaves camera unchanged");
    }
    // The gesture, against a reference it is handed (the OpenXR frame thread owns the reference).
    for(float normal:{.9f,1.2f,1.7f,2.0f}) for(float depth:{.08f,.22f,.65f})
    {
        PhysicalCrouch crouch;uint64_t now=1000;
        auto step=[&](float height,bool ready=true,uint64_t reference=7)
        {now+=20;return crouch.Update(height,normal,reference,now,ready,depth);};
        Check(!step(normal),"first sample at the reference starts released");
        Check(!step(normal-depth*.9f),"movement above threshold does not crouch");
        Check(step(normal-depth*1.1f),"lowering beyond selected depth presses crouch");
        Check(step(normal-depth*.8f),"hysteresis prevents boundary chatter");
        Check(!step(normal-depth*.6f),"standing above release threshold releases crouch");
        Check(step(normal-depth*1.1f),"deliberate second crouch can activate");
        Check(!step(normal-depth*1.1f,false),"menus vehicles and lost tracking release the gesture");
        Check(!step(normal-depth*1.1f),"resume cannot replay a crouch held during exclusion");
        Check(!step(normal),"standing rearms without a press");
        Check(step(normal-depth*1.1f),"fresh crouch after resume works");
        Check(!step(normal-depth*1.1f,true,8),"a new reference sample re-arms released");
        Check(!step(normal-depth*1.1f,true,8),"and a head already below release must stand first");
        Check(!step(normal,true,8),"standing under the new reference");
        Check(step(normal-depth*1.1f,true,8),"then crouches against it");
        crouch.Reset();Check(!step(normal),"disable and re-enable starts neutral");
        now+=300;Check(!step(normal-depth*1.1f),"stale sample cannot become a crouch");
        Check(!step(normal),"stale tracking rearms upright");
        Check(step(normal-depth*1.1f),"fresh tracked crouch after gap works");
        Check(!step(std::numeric_limits<float>::quiet_NaN()),"nonfinite tracking releases input");
        Check(!step(normal-depth*1.1f),"nonfinite recovery requires standing again");
        Check(!crouch.Update(normal-depth*1.1f,normal,0,now+20,true,depth),"no reference yet: no crouch");
    }
    {
        PhysicalCrouch state;
        Check(!state.Update(1.7f,1.7f,1,1000,true,.22f),"first sample starts neutral");
        Check(!state.Update(1.7f,1.7f,1,1020,true,.4f),"sensitivity edit cannot create a crouch edge");
        Check(!state.Update(1.7f,1.7f,1,1040,true,std::numeric_limits<float>::infinity()),
            "invalid sensitivity uses the finite default");
        Check(state.Update(1.4f,1.7f,1,1060,true,.22f),"finite fallback still permits physical crouching");
    }

    // A LEARNED reference (the first head, or a break) rises to the median of the last 10 s once 5 s exist.
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float seated=1.2f,standing=1.75f;
        auto step=[&](float h){now+=11;r.Update(h,1,0,now,true,.22f);};
        step(seated);
        Check(r.Valid()&&!r.Calibrated()&&r.Seeds()==1&&std::fabs(r.Baseline()-seated)<.001f,
            "the first tracked head is sampled as a learned reference");
        for(int i=0;i<300;++i) step(standing);
        Check(std::fabs(r.Baseline()-seated)<.001f,"three seconds are not enough to move it");
        for(int i=0;i<300;++i) step(standing);
        Check(std::fabs(r.Baseline()-standing)<.001f,"the window's median lifts a seated sample to standing");
        Check(r.Seeds()==1,"learning is not a new sample");
    }
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float standing=1.7f;
        auto step=[&](float h){now+=11;r.Update(h,1,0,now,true,.22f);};
        step(standing);
        for(int i=0;i<600;++i) step(standing);
        for(int i=0;i<300;++i) step(standing+.6f);
        Check(std::fabs(r.Baseline()-standing)<.02f,"a headset held up for a few seconds does not move it");
        for(int i=0;i<900;++i) step(standing-.5f);
        Check(std::fabs(r.Baseline()-standing)<.02f,"a long crouch never drags it down");
    }
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float standing=1.7f;
        auto step=[&](float h){now+=11;r.Update(h,1,0,now,true,.22f);};
        step(standing);
        for(int i=0;i<1000;++i) step(standing+.5f);
        Check(r.Baseline()>standing+.4f,"(setup) a long hold does lift a learned reference");
        for(int i=0;i<5000;++i) step(standing);
        Check(r.Baseline()>standing+.4f,"under a minute below its release line keeps it");
        for(int i=0;i<1000;++i) step(standing);
        Check(std::fabs(r.Baseline()-standing)<.02f,"a minute below the release line falls back to the median");
    }
    // A reference CONFIRMED by a minute of standing is the player's height: a two-minute kneel behind cover
    // stays a crouch, and it falls back only after kStaleLowConfirmedMs.
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float standing=1.7f,kneel=1.2f;
        auto step=[&](float h){now+=100;r.Update(h,1,0,now,true,.22f);};
        step(standing);
        for(int i=0;i<100;++i) step(standing);
        Check(!r.Confirmed(),"ten seconds of standing do not confirm a learned reference (a recenter lift waits)");
        for(int i=0;i<600;++i) step(standing);
        Check(r.Confirmed(),"a minute of standing near it confirms it");
        for(int i=0;i<1200;++i) step(kneel);
        Check(std::fabs(r.Baseline()-standing)<.02f,"a confirmed reference holds through a two-minute kneel");
        for(int i=0;i<1900;++i) step(kneel);
        Check(std::fabs(r.Baseline()-kneel)<.02f,"...and falls back only after kStaleLowConfirmedMs");
        // A seated player who stood up briefly is unconfirmed again: the minute's fallback applies.
        for(int i=0;i<100;++i) step(standing);
        Check(std::fabs(r.Baseline()-standing)<.02f,"(setup) standing for ten seconds lifts it again");
        Check(!r.Confirmed(),"a reference lifted by a few seconds' standing is not confirmed");
        for(int i=0;i<620;++i) step(kneel);
        Check(std::fabs(r.Baseline()-kneel)<.02f,"a reference lifted by a few seconds' standing falls back in a minute");
        // ...however the stand-up looks. Confirmed seated for over a minute, then an ordinary one-second
        // stand-up: the median climbs a few centimetres a sample, never a release band at once, and the
        // reference must still count as new.
        for(int i=0;i<700;++i) step(kneel);
        for(int i=1;i<=10;++i) step(kneel+(standing-kneel)*float(i)/10.f);
        for(int i=0;i<100;++i) step(standing);
        Check(std::fabs(r.Baseline()-standing)<.02f,"(setup) a gradual stand-up lifts it");
        for(int i=0;i<620;++i) step(kneel);
        Check(std::fabs(r.Baseline()-kneel)<.02f,"a gradual stand-up is a new reference too: it falls back in a minute");
    }
    {
        PhysicalCrouchReference r;uint64_t now=5000;
        r.Update(1.7f,1,0,now+=20,true,.22f);
        r.Update(1.0f,1,1,now+=20,true,.22f);
        Check(r.Seeds()==2&&!r.Calibrated()&&std::fabs(r.Baseline()-1.f)<.001f,
            "a continuity break re-samples a learned reference");
        r.Update(1.0f,2,1,now+=20,true,.22f);
        Check(r.Calibrated()&&r.Seeds()==3,"Calibrate re-samples it as the player's word");
        r.Update(1.6f,2,2,now+=20,true,.22f);
        Check(!r.Calibrated()&&r.Seeds()==4,"a break after Calibrate falls back to learning (the frame moved)");
        r.Update(std::numeric_limits<float>::quiet_NaN(),2,2,now+=20,true,.22f);
        Check(r.Seeds()==4,"a non-finite head samples nothing");
        r.Update(1.0f,3,2,now+=20,false,.22f);
        Check(r.Seeds()==4,"an unavailable head samples nothing, even after Calibrate");
        r.Update(1.0f,3,2,now+=20,true,.22f);
        Check(r.Calibrated()&&r.Seeds()==5,"the pending Calibrate lands on the next available head");
    }

    // A CALIBRATED reference rises only to a height held for a second, by at most 0.7 x depth - 2 cm
    // (13.4 cm here), and ignores a head more than 30 cm above the calibration.
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float normal=1.7f,depth=.22f;uint32_t calibration=1;
        auto step=[&](float h){now+=20;r.Update(h,calibration,0,now,true,depth);};
        step(normal+.3f);
        calibration=2;step(normal-.10f);
        Check(r.Calibrated()&&r.Seeds()==2&&std::fabs(r.Baseline()-(normal-.10f))<.001f,
            "Calibrate samples the current head");
        for(int i=0;i<60;++i) step(normal);
        Check(std::fabs(r.Baseline()-normal)<.001f,"standing tall for a second raises it");
    }
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float normal=1.7f,depth=.22f;uint32_t calibration=1;
        auto step=[&](float h){now+=20;r.Update(h,calibration,0,now,true,depth);};
        const float cap=depth*.70f-.02f;
        step(normal);calibration=2;step(normal-.20f);
        Check(!r.RiseCapped(),"nothing capped yet");
        for(int i=0;i<60;++i) step(normal);
        Check(std::fabs(r.Baseline()-(normal-.20f+cap))<.001f,"a rise beyond the cap is learned only up to the cap");
        Check(r.RiseCapped(),"a capped rise is reported, so the menu can suggest calibrating standing");
    }
    // Seated play: calibrate seated, stand up, half-rise, sit back down -- never a crouch while seated.
    {
        PhysicalCrouchReference r;PhysicalCrouch g;uint64_t now=5000;const float seated=1.2f,depth=.22f;
        uint32_t calibration=1;
        auto step=[&](float h)
        {now+=20;r.Update(h,calibration,0,now,true,depth);return g.Update(h,r.Baseline(),r.Seeds(),now,true,depth);};
        step(seated);calibration=2;step(seated);
        Check(r.Calibrated(),"calibrated seated");
        for(int i=0;i<100;++i) step(seated+.5f);
        Check(std::fabs(r.Baseline()-seated)<.001f,"standing up from a seated calibration is not learned");
        Check(!step(seated),"sitting back down is not a crouch");
        for(int i=0;i<60;++i) step(seated+.25f);
        Check(r.Baseline()>seated+.1f&&r.Baseline()<seated+depth*.70f,"a half-rise is learned, capped below release");
        bool seatedCrouch=false;
        for(int i=0;i<60;++i) seatedCrouch|=step(seated);
        Check(!seatedCrouch,"sitting back down after a half-rise deeper than the depth is still not a crouch");
        Check(step(r.Baseline()-depth*1.1f),"a real crouch below the learned line still works");
    }
    // ...and the cap follows the depth slider: a half-rise learned at 40 cm, then the depth lowered to 22 cm --
    // sitting is still not a crouch (the slider no longer re-samples the reference).
    {
        PhysicalCrouchReference r;PhysicalCrouch g;uint64_t now=5000;const float seated=1.2f;float depth=.40f;
        uint32_t calibration=1;
        auto step=[&](float h)
        {now+=20;r.Update(h,calibration,0,now,true,depth);return g.Update(h,r.Baseline(),r.Seeds(),now,true,depth);};
        step(seated);calibration=2;step(seated);
        for(int i=0;i<60;++i) step(seated+.27f);
        Check(r.Baseline()>seated+.2f,"fixture: a half-rise is learned at the deeper setting");
        depth=.22f;
        bool seatedCrouch=false;
        for(int i=0;i<60;++i) seatedCrouch|=step(seated);
        Check(!seatedCrouch&&r.Baseline()<seated+depth*.70f&&r.Baseline()>=seated,
            "a shallower depth re-caps the learned rise, never below the calibration: sitting is not a crouch");
    }
    {
        PhysicalCrouchReference r;uint64_t now=5000;const float normal=1.7f,depth=.22f;uint32_t calibration=1;
        auto step=[&](float h){now+=20;r.Update(h,calibration,0,now,true,depth);};
        step(normal);calibration=2;step(normal);
        for(int i=0;i<30;++i) step(normal+.10f);
        step(normal);
        Check(std::fabs(r.Baseline()-normal)<.001f,"a brief rise does not move a calibrated reference");
        for(int i=0;i<100;++i) step(normal+.80f);
        Check(std::fabs(r.Baseline()-normal)<.001f,"a rise beyond 30 cm (headset lifted) is ignored");
        for(int i=0;i<100;++i) step(normal-depth*1.5f);
        Check(std::fabs(r.Baseline()-normal)<.001f,"a long crouch never drags it down");
        for(int i=0;i<60;++i) step(normal+.05f);
        Check(std::fabs(r.Baseline()-(normal+.05f))<.001f,"standing taller later still raises it");
    }

    // Height continuity: a runtime recenter moves the LOCAL space under the player; the frame absorbs it.
    const int64_t frame=11'111'111;
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        auto at=[&](float raw){t+=frame;return c.Update(raw,t);};
        Check(std::fabs(at(.5f)-.5f)<1e-6f,"the first sample is its own height");
        for(int i=0;i<10;++i) at(.5f-.002f*float(i));
        Check(std::fabs(at(.48f)-.48f)<1e-6f,"real motion is not absorbed");
        c.NoteSpaceChange(t+2*frame,true,.55f);
        Check(std::fabs(at(.47f)-.47f)<1e-6f,"before the change, old-space heights pass");
        Check(std::fabs(at(.47f-.55f)-.47f)<1e-5f,"the switch frame is absorbed exactly (the runtime's offset)");
        Check(c.Announced()==1&&c.Breaks()==0,"counted as seen, no break");
        Check(std::fabs(at(-.10f)-(-.10f+.55f))<1e-5f,"later heights stay in the continuous frame");
    }
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        auto at=[&](float raw){t+=frame;return c.Update(raw,t);};
        float raw=.5f,cont=0;at(raw);
        c.NoteSpaceChange(t+frame,true,.55f);
        for(int i=0;i<40;++i) {raw-=.02f;if(i==1) raw-=.55f;cont=at(raw);}
        Check(std::fabs(cont-(.5f-.80f))<1e-4f,"a duck straight through a recenter reads as the duck alone");
    }
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        auto at=[&](float raw){t+=frame;return c.Update(raw,t);};
        at(.5f);at(.5f);
        c.NoteSpaceChange(t+frame,false,0.f);
        at(.5f);
        Check(std::fabs(at(-.05f)-.5f)<1e-5f,"a recenter without an offset is absorbed from its step");
        at(-.05f);
        Check(std::fabs(at(-.05f+.12f)-.5f)<1e-5f,"an unannounced tracking step is absorbed");
        Check(c.Unannounced()==1&&c.Announced()==1,"each counted");
        c.NoteSpaceChange(t,true,-.12f);
        Check(c.Late()==1,"a late announcement of an absorbed step is recognised");
        for(int i=0;i<300;++i) at(.07f);
        Check(std::fabs(at(.07f)-.5f)<1e-5f,"and the frame does not move a second time");
    }
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        c.Update(.5f,t);
        c.NoteSpaceChange(t+5*frame,true,.55f);
        t+=500'000'000;
        Check(std::fabs(c.Update(.5f-.55f,t)-.5f)<1e-5f,"a recenter inside a tracking gap takes the runtime's offset");
        Check(c.Applied()==1&&c.Breaks()==0,"applied, no break");
        c.NoteSpaceChange(t+5*frame,false,0.f);
        t+=500'000'000;
        c.Update(.2f,t);
        Check(c.Breaks()==1,"an unmeasured move inside a gap breaks continuity");
    }
    // The usual order here: events are polled before present, the head is sampled after the wait, so a
    // recenter that fell inside a tracking gap can be polled only after the sample that crossed it. No pair
    // across a gap is absorbed (the step went in as motion), so the runtime's offset must still be taken.
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        auto at=[&](float raw){t+=frame;return c.Update(raw,t);};
        at(.5f);at(.5f);
        const int64_t before=t;
        t+=100'000'000;
        c.Update(.5f-.30f,t);
        c.NoteSpaceChange(before+50'000'000,true,.30f);
        Check(std::fabs(at(.5f-.30f)-.5f)<1e-5f&&c.Applied()==1&&c.Breaks()==0&&c.Unseen()==0,
            "a recenter polled after the gap it fell in takes the runtime's offset");
        for(int i=0;i<300;++i) at(.2f);
        Check(std::fabs(at(.2f)-.5f)<1e-5f&&c.Unseen()==0,"and nothing is left pending to expire");
        const int64_t before2=t;
        t+=100'000'000;
        c.Update(-.1f,t);
        c.NoteSpaceChange(before2+50'000'000,false,0.f);
        Check(c.Breaks()==1,"without an offset, a recenter in a crossed gap breaks continuity");
    }
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        auto at=[&](float raw){t+=frame;return c.Update(raw,t);};
        at(.5f);
        c.NoteSpaceChange(t+2*frame,true,.01f);
        at(.5f);
        Check(std::fabs(at(.49f)-.5f)<1e-5f,"a 1 cm recenter is applied at its change time");
        Check(c.Applied()==1,"counted as given");
        Check(std::fabs(c.Update(5.f,t)-(5.f+.01f))<1e-5f&&c.Unannounced()==0,"a repeated display time is not a step");
    }
    {
        CrouchHeightContinuity c;int64_t t=1'000'000'000;
        auto at=[&](float raw){t+=frame;return c.Update(raw,t);};
        at(.5f);
        c.NoteSpaceChange(t+frame,true,.40f);
        for(int i=0;i<200;++i) at(.5f);
        Check(c.Unseen()==1&&std::fabs(at(.5f)-.5f)<1e-5f,"a recenter never seen moves nothing and is counted");
    }
    {
        CrouchHeightContinuity c;
        c.Update(.5f,1'000'000'000);
        c.Break();
        Check(c.Breaks()==1,"a new session breaks continuity");
        c.Update(-.3f,2'000'000'000);
        Check(c.Breaks()==1,"the first sample after a break starts the new frame");
    }
    // End to end, the Halo 4 log of 2026-09-30: sampled bent over, then standing at 0.5 m in the old space;
    // a runtime recenter puts the head at 0.0 and the player ducks straight away. The reference must stay
    // at standing, and the duck must crouch at the activation depth.
    {
        CrouchHeightContinuity c;PhysicalCrouchReference r;PhysicalCrouch g;
        int64_t t=1'000'000'000;uint64_t ms=10'000;const float depth=.22f;bool crouched=false;
        auto at=[&](float raw)
        {
            t+=frame;ms+=11;const float h=c.Update(raw,t);r.Update(h,1,c.Breaks(),ms,true,depth);
            crouched=g.Update(h,r.Baseline(),r.Seeds(),ms,true,depth);
        };
        at(.07f);
        // Stands up over half a second (a step of 43 cm in one frame would be the space moving, not him).
        for(int i=1;i<=45;++i) at(.07f+.43f*float(i)/45.f);
        Check(std::fabs(c.Carry())<1e-6f,"standing up is motion, never absorbed");
        for(int i=0;i<700;++i) at(.5f);
        Check(std::fabs(r.Baseline()-.5f)<.01f,"a low first sample heals to standing");
        c.NoteSpaceChange(t+frame,true,.5f);
        float raw=.5f;
        for(int i=0;i<60;++i) {raw-=.015f;if(i==1) raw-=.5f;at(raw);}
        Check(std::fabs(r.Baseline()-.5f)<.01f&&r.Seeds()==1,"a recenter mid-duck keeps the standing reference");
        Check(crouched,"and the duck crouches at the activation depth");
    }
    {
        // THE VIEW'S NEUTRAL HEIGHT FOLLOWS THE CONFIRMED STANDING HEIGHT UP (standing_height_follow.h).
        using standing_follow::State; using standing_follow::Step;
        uint64_t now=1000;
        // A recenter caught the player 30 cm low: reference 1.40, standing 1.70; the head is at 1.55 now, so
        // the crouch's own frame says it is 15 cm below standing.
        State s{}; float reference=1.40f; bool started=true;
        Check(Step(s,false,1.55f,.15f,reference,now,&started)==0&&!s.active&&!started,
            "no standing height confirmed: the reference stays where the recenter put it");
        now+=11;
        Check(Step(s,true,1.55f,.15f,reference,now,&started)>0&&s.active&&started,
            "confirmed and 30 cm low: the glide starts, and says so once");
        float raised=0; unsigned starts=0; uint64_t began=now;
        for(int i=0;i<400&&s.active;++i)
        {
            now+=11;
            // The player bobs and crouches meanwhile: head and deficit move together, the standing height does not.
            const float dip=.10f*float(i%20)/20.f;
            const float step=Step(s,true,1.55f-dip,.15f+dip,reference,now,&started);
            Check(step>=0&&step<=standing_follow::kRateMps*.011f+1e-6f,"a step is never faster than the glide's rate");
            reference+=step;raised+=step;starts+=started?1u:0u;
        }
        Check(!s.active&&std::fabs(reference-1.70f)<=standing_follow::kDoneM+1e-4f&&reference<=1.70f+1e-4f,
            "the glide ends on the standing height and never above it");
        Check(starts==0,"one glide says so once");
        Check(now-began>900&&now-began<1200,"30 cm takes about a second");
        now+=11;
        Check(Step(s,true,1.73f,0.f,reference,now,&started)==0&&!s.active,
            "3 cm under standing is within the tolerance: nothing moves");
        now+=11;
        Check(Step(s,true,1.80f,-.10f,reference,now,&started)==0&&!s.active,
            "a head above the standing height (a jump, tiptoe) does not raise the reference");
        float high=1.90f; now+=11;
        Check(Step(s,true,1.70f,0.f,high,now,&started)==0&&!s.active,"a reference above standing is never lowered");
        // A break in the calls (a cutscene, a seat) is not one long step.
        State b{}; float low=1.20f; now+=11;
        (void)Step(b,true,1.70f,0.f,low,now); now+=11;
        const float first=Step(b,true,1.70f,0.f,low,now); low+=first;
        Check(first>0&&first<.01f,"a glide moves by the time since the last call");
        now+=5000;
        Check(Step(b,true,1.70f,0.f,low,now)==0&&b.active,"five seconds without a call move nothing on the next");
        now+=11;
        Check(Step(b,true,1.70f,0.f,low,now)>0,"and the glide goes on from there");
        now+=11;
        Check(Step(b,false,1.70f,0.f,low,now)==0&&!b.active,"a seat (or the switch) stops a glide where it is");
        State c{}; float r=1.20f; now+=11;
        Check(Step(c,true,1.70f,1.5f,r,now)==0&&!c.active,"a deficit past a metre is not a standing person's sample");
        Check(Step(c,true,std::numeric_limits<float>::quiet_NaN(),0.f,r,now+11)==0&&!c.active,"a head height that is not a number moves nothing");
    }
    std::printf("Physical crouch: %u checks passed\n",checks);
}
