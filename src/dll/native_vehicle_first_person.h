#pragma once
#include <cstdint>

enum class GameTitle : uint8_t;
struct NativeVehicleCameraOwner
{
    uint32_t generation{},unit{UINT32_MAX},parent{UINT32_MAX};
    int16_t seat{-1};
    uintptr_t object{};
    uint64_t vehicleIdentity{}; // stable authored identity; never a live object datum
    uint32_t root{UINT32_MAX};  // the vehicle at the top of the seat's parent chain, where the title reads it
    bool operator==(const NativeVehicleCameraOwner&) const = default;
};

// Optional CE/H2/H4 camera feature. Existing H3/ODST/Reach paths are independent.
void NativeVehicleFirstPerson_Poll();
bool NativeVehicleFirstPerson_TrimTarget(GameTitle title,uint64_t& identity,int& seat) noexcept;
// This override placed the camera in exactly this seat within 250 ms: the seat's view is the occupant's own
// head, not the native chase camera (which orbits with the aim).
bool NativeVehicleFirstPerson_SeatViewOwned(GameTitle title,uint32_t parent,int seat) noexcept;
bool Game_ReadVehicleCameraOwner(GameTitle title, NativeVehicleCameraOwner& owner) noexcept;
#if HALOMCCVR_DEV_DIAGNOSTICS
void Game_Halo4CameraOwnerReport() noexcept;   // dev: why Game_ReadVehicleCameraOwner refused Halo 4 seats

// DEV, Halo 4 seated convergence (stage 0; game.cpp halo4_seat_sight.inl).
//
// THE NATIVE SEAT EYE at render time. The override moves the seat camera to the occupant's head plus the
// seat trims, but the game's AIM line in a seat starts at the native camera point 0x5F8A1C (the seat-camera
// marker, the head, or the slaved turret's marker) -- the point 0x125190 left in result+4 before the
// override wrote over it. Published as an OFFSET from the placed point (both from the same call, so it is
// coherent whatever thread made the call), never as a world position: the render transaction adds it to its
// own camera root, in the same frame as the rendered head. Halo 4 only; any thread; nothing while the seat
// view is not ours.
struct NativeVehicleSeatEyeDelta
{
    uint32_t generation{}, parent{UINT32_MAX};
    int16_t seat{-1};
    float delta[3]{};   // native camera point minus the placed point, world units
    uint64_t ms{};      // GetTickCount64 of the publication
};
bool NativeVehicleFirstPerson_SeatEyeDelta(GameTitle title, NativeVehicleSeatEyeDelta& out) noexcept;

// THE FIRST-PERSON WITNESS. 0x2061A8 (the player's aim camera) asks the selector 0x10D268 once, nested on
// the same thread, and builds the first-person eye only when it answers 0. A caller arms this thread's
// witness, makes the native call, and takes what Halo 4's selector hook saw: how often it ran, for which
// unit, the native answer, the answer after our override, and whether the override took it.
struct NativeVehicleSelectWitness
{
    uint32_t calls{}, unit{UINT32_MAX}, nativeResult{}, result{};
    int32_t mode{-1};
    bool overridden{};
};
void NativeVehicleFirstPerson_ArmSelectWitness() noexcept;
NativeVehicleSelectWitness NativeVehicleFirstPerson_TakeSelectWitness() noexcept;
#endif
bool HaloCEControls_ReadVehicleCameraOwner(NativeVehicleCameraOwner& owner) noexcept;
bool Halo2Observer6Dof_ReadVehicleCameraOwner(NativeVehicleCameraOwner& owner) noexcept;
