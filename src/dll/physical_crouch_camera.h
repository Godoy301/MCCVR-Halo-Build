#pragma once
#include "../common/physical_crouch_camera_logic.h"

void PhysicalCrouchCamera_Publish(GameTitle title,uint32_t generation,
    uint64_t trackingEpoch,uint64_t nowMs,bool available,bool requested) noexcept;
void PhysicalCrouchCamera_Invalidate() noexcept;
bool PhysicalCrouchCamera_Read(PhysicalCrouchCameraRequest& request) noexcept;

// DEV BUILD ONLY: the game's OWN crouch camera drop for a title (world units), as that title's camera
// correction just read it from the unit, so a headless test judges a physical duck by the game's state
// (the CROUCH line's nativeDrop/unitCrouched). A player build compiles both to nothing.
#if HALOMCCVR_DEV_DIAGNOSTICS
void PhysicalCrouchCamera_NoteNativeDrop(GameTitle title,float dropWu) noexcept;
bool PhysicalCrouchCamera_NativeDrop(GameTitle title,uint64_t maxAgeMs,float& dropWu) noexcept;
#else
inline void PhysicalCrouchCamera_NoteNativeDrop(GameTitle,float) noexcept {}
inline bool PhysicalCrouchCamera_NativeDrop(GameTitle,uint64_t,float& dropWu) noexcept {dropWu=0;return false;}
#endif
