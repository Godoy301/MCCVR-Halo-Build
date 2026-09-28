#include "../src/common/physical_crouch_logic.h"
#include "../src/common/physical_crouch_native_read.h"
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
        }
        else
        {
            Put(unit,0x390,uint16_t(0x2000));Check(!read(unit,definition,reduction),"H2 animation special state conservatively refused");
        }
        Check(!read(nullptr,definition,reduction)&&!read(unit,nullptr,reduction),"missing native storage leaves camera unchanged");
    }
    for(float normal:{.9f,1.2f,1.7f,2.0f}) for(float depth:{.08f,.22f,.65f})
    {
        PhysicalCrouch crouch;uint64_t now=1000;
        auto step=[&](float height,bool ready=true,uint64_t epoch=7){now+=20;return crouch.Update(height,epoch,now,ready,depth);};
        Check(!step(normal),"first normal standing or seated height calibrates without a press");
        Check(!step(normal-depth*.9f),"movement above threshold does not crouch");
        Check(step(normal-depth*1.1f),"lowering beyond selected depth presses crouch");
        Check(step(normal-depth*.8f),"hysteresis prevents boundary chatter");
        Check(!step(normal-depth*.6f),"standing above release threshold releases crouch");
        Check(step(normal-depth*1.1f),"deliberate second crouch can activate");
        Check(!step(normal-depth*1.1f,false),"menus vehicles and lost tracking release the gesture");
        Check(!step(normal-depth*1.1f),"resume cannot replay a crouch held during exclusion");
        Check(!step(normal),"standing rearms without a press");
        Check(step(normal-depth*1.1f),"fresh crouch after resume works");
        Check(!step(normal-depth*1.1f,true,8),"recenter samples new reference without crouching");
        crouch.Reset();Check(!step(normal),"disable and re-enable starts neutral");
        now+=300;Check(!step(normal-depth*1.1f),"stale sample cannot become a crouch");
        Check(!step(normal),"stale tracking rearms upright");
        Check(step(normal-depth*1.1f),"fresh tracked crouch after gap works");
        Check(!step(std::numeric_limits<float>::quiet_NaN()),"nonfinite tracking releases input");
        Check(!step(normal-depth*1.1f),"nonfinite recovery requires standing again");
    }
    PhysicalCrouch state;
    Check(!state.Update(1.7f,1,1000,true,.22f),"calibration starts neutral");
    Check(!state.Update(1.7f,1,1020,true,.4f),"sensitivity edit cannot create a crouch edge");
    Check(!state.Update(1.7f,1,1040,true,std::numeric_limits<float>::infinity()),"invalid sensitivity uses finite default and recalibrates");
    Check(state.Update(1.4f,1,1060,true,.22f),"finite fallback still permits physical crouching");

    // Learned standing height (only rises). Steps are 20 ms; a rise needs 1 s held steady.
    {
        PhysicalCrouch c;uint64_t now=5000;const float normal=1.7f,depth=.22f;
        auto step=[&](float h){now+=20;return c.Update(h,9,now,true,depth);};
        Check(!step(normal-.10f),"calibration taken slouched");
        for(int i=0;i<60;++i) step(normal);   // stand tall for 1.2 s
        Check(std::fabs(c.Baseline()-normal)<.001f,"standing tall for a second raises the baseline");
        Check(step(normal-depth*1.1f),"a crouch is measured from the learned standing height");
        Check(!step(normal),"and releases on standing");
    }
    // The learned rise is capped below the release margin (0.7 x depth - 2 cm = 13.4 cm here), so the
    // posture the player calibrated in never reads as a crouch afterwards.
    {
        PhysicalCrouch c;uint64_t now=5000;const float normal=1.7f,depth=.22f;
        auto step=[&](float h){now+=20;return c.Update(h,9,now,true,depth);};
        const float cap=depth*.70f-.02f;
        Check(!step(normal-.20f),"calibration taken deeply slouched");
        Check(!c.RiseCapped(),"nothing capped yet");
        for(int i=0;i<60;++i) step(normal);
        Check(std::fabs(c.Baseline()-(normal-.20f+cap))<.001f,"a rise beyond the cap is learned only up to the cap");
        Check(c.RiseCapped(),"a capped rise is reported, so the menu can suggest recalibrating standing");
        Check(!step(normal-.20f),"the calibrated posture still reads standing");
    }
    // Seated play: calibrate seated, stand up, half-rise, sit back down -- never a crouch while seated.
    // The half-rise (25 cm) is deeper than the activation depth (22 cm), so without the cap sitting back
    // down would read as a crouch.
    {
        PhysicalCrouch c;uint64_t now=5000;const float seated=1.2f,depth=.22f;
        auto step=[&](float h){now+=20;return c.Update(h,9,now,true,depth);};
        Check(!step(seated),"calibrated seated");
        for(int i=0;i<100;++i) step(seated+.5f);   // stands up for 2 s: beyond kMaxRise, not learned
        Check(std::fabs(c.Baseline()-seated)<.001f,"standing up from a seated calibration is not learned");
        Check(!step(seated),"sitting back down is not a crouch");
        for(int i=0;i<60;++i) step(seated+.25f);   // a half-rise held for 1.2 s
        Check(c.Baseline()>seated+.1f&&c.Baseline()<seated+depth*.70f,"a half-rise is learned, capped below release");
        bool seatedCrouch=false;
        for(int i=0;i<60;++i) seatedCrouch|=step(seated);
        Check(!seatedCrouch,"sitting back down after a half-rise deeper than the depth is still not a crouch");
        Check(step(c.Baseline()-depth*1.1f),"a real crouch below the learned line still works");
    }
    // A new epoch reaches the gesture only once the head has held its height for 300 ms.
    {
        CrouchEpochSettle s;
        s.Update(0,true,1.7f,1000);
        Check(s.Settled()&&s.Epoch()==0,"the initial epoch (0) is settled as it is");
        s.Update(7,true,1.7f,1020);
        Check(!s.Settled()&&s.Epoch()==0,"a new epoch is held back at first");
        s.Update(7,true,1.71f,1200);
        Check(!s.Settled()&&s.Epoch()==0,"still held within the settle window");
        s.Update(7,true,1.71f,1321);
        Check(s.Settled()&&s.Epoch()==7,"a head steady within 2 cm for 300 ms settles it");
        s.Update(7,false,0.f,1400);
        Check(s.Settled()&&s.Epoch()==7,"a later tracking gap does not unsettle a settled epoch");
        s.Update(8,true,1.7f,1500);
        s.Update(8,true,1.6f,1700);   // still moving: 10 cm since the epoch changed
        s.Update(8,true,1.6f,1950);
        Check(!s.Settled()&&s.Epoch()==7,"a head still moving restarts the window");
        s.Update(8,false,0.f,2000);
        s.Update(8,true,1.6f,2100);
        s.Update(8,true,1.6f,2350);
        Check(!s.Settled()&&s.Epoch()==7,"a tracking gap restarts the window too");
        s.Update(8,true,1.6f,2400);
        Check(s.Settled()&&s.Epoch()==8,"then it settles 300 ms after the head stopped");
    }
    {
        PhysicalCrouch c;uint64_t now=5000;const float normal=1.7f,depth=.22f;
        auto step=[&](float h){now+=20;return c.Update(h,9,now,true,depth);};
        Check(!step(normal),"calibrated standing");
        for(int i=0;i<30;++i) step(normal+.10f);   // 0.6 s: too brief
        step(normal);
        Check(std::fabs(c.Baseline()-normal)<.001f,"a brief rise does not move the baseline");
        for(int i=0;i<100;++i) step(normal+.80f);  // headset lifted off the head, held still for 2 s
        Check(std::fabs(c.Baseline()-normal)<.001f,"a rise beyond the cap (headset lifted) is ignored");
        Check(step(normal-depth*1.1f),"crouch still measured from the true standing height");
    }
    {
        PhysicalCrouch c;uint64_t now=5000;const float normal=1.7f,depth=.22f;
        auto step=[&](float h){now+=20;return c.Update(h,9,now,true,depth);};
        Check(!step(normal),"calibrated standing");
        Check(step(normal-depth*1.5f),"crouched");
        for(int i=0;i<100;++i) step(normal-depth*1.5f);  // a long crouch
        Check(std::fabs(c.Baseline()-normal)<.001f,"a long crouch never drags the line down");
        Check(!step(normal),"stands");
        for(int i=0;i<60;++i) step(normal+.05f);          // standing taller later
        Check(std::fabs(c.Baseline()-(normal+.05f))<.001f,"standing taller later still raises it");
        // Raised line = 1.75 - 0.22 = 1.53; old line was 1.48. A head at 1.50 is past the raised line
        // (crouches) but would not have been past the old one.
        Check(step(normal-depth+.02f),"the raised line triggers higher, as learned");
    }
    std::printf("Physical crouch: %u checks passed\n",checks);
}
