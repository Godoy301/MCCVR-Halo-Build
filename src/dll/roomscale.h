#pragma once
#include "../common/runtime_types.h"
void Roomscale_Input(bool allowed,float moveX,float moveY) noexcept;
// The jump action, as this poll feeds it to the game: the view recenter's takeoff witness reads it.
void Roomscale_JumpInput(bool held) noexcept;
bool Roomscale_Move(float& moveX,float& moveY) noexcept;
bool PhysicalRunning_Move(float& moveX,float& moveY) noexcept;
void Roomscale_Camera(GameTitle title,bool allowed,const float body[3],
    const float head[3],const float orientation[4],const float forward[3],
    float reference[3],float scale) noexcept;

void Roomscale_Report() noexcept;
