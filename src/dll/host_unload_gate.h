#pragma once
// MCC'S RELOAD OF A GAME DLL WAITS FOR OUR TEARDOWN (host_unload_gate.cpp has the why and the evidence).
//
// A title core that keeps a loader reference on its game DLL while its hooks are installed registers that
// reference here. When the game ends MCC loads the DLL again for its menus; while the core is tearing down and
// still holds the DLL, that load waits, bounded, until the core has let go, so the old image unloads first and
// the load maps a fresh one, as it does without the mod.
#include <windows.h>
#include <atomic>
#include <cstdint>

// Once, after MH_Initialize. False: the gate is not installed and everything behaves as before.
bool HostUnloadGate_Install();
// Worker: `module` is the reference the core holds. `tearingDown` is the core's own "teardown requested" flag:
// only while it is set does a load of that DLL wait. `lastFrameMs` is the GetTickCount64 of the title's last
// frame (for the report). `title` is a string literal for the log.
void HostUnloadGate_Pin(HMODULE module, const std::atomic<bool>* tearingDown,
                        const std::atomic<uint64_t>* lastFrameMs, const char* title);
// Worker, AFTER the core's own FreeLibrary of that reference: a waiting load may go on.
void HostUnloadGate_Unpin(HMODULE module);
// Worker: the core cannot let go without the title's own thread (a restore that only that thread can make), so a
// load waiting for it stops waiting now instead of at the time-out. The reference stays registered.
void HostUnloadGate_Abandon(HMODULE module);
// Any thread (the game window's procedure, on WM_CLOSE or WM_DESTROY): MCC itself is closing. From here no
// load waits, and one that is waiting goes on at once: a shutdown must not be held back.
void HostUnloadGate_HostClosing() noexcept;
// Cold worker: one line per game end (when the reload came, how long it waited and how the wait ended).
void HostUnloadGate_Report();
