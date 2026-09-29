; Halo 3 / ODST / Halo 4 direct drive (legacy_direct_drive): entry detours for each title's per-user
; control update -- the ACCUMULATOR of player_control.desired_angles, Reach's 0x6375C in each sibling
; engine (located 2026-09-29 by structure: TLS player_control, a per-user stride, desired += stick delta
; then clamps; see LegacyDrive in game.cpp for the RVAs and the anchors that guard them).
; Signature, as Reach's: ecx = user index, xmm1..xmm3 = floats, no stack arguments; the record is reached
; through the calling (sim) thread's own TLS, so the C callee re-derives it from the same thread.
; Each detour runs the native update unchanged, then ALWAYS calls LegacyPcApplyForUser(slot, user), which
; decides (armed / fresh setpoint / validated record / on foot) whether to overwrite desired_angles.
; Naked so the native ABI (rcx + xmm1..xmm3; the natives return nothing meaningful -- each ends by storing
; its clamped pitch) is forwarded byte-exact. PROC FRAME + .allocstack give each an unwind entry, which an
; exception unwinding through it and the teardown's quiescence scan both need.
option casemap:none

EXTERN g_halo3PcOrig:QWORD
EXTERN g_odstPcOrig:QWORD
EXTERN g_halo4PcOrig:QWORD
EXTERN LegacyPcApplyForUser:PROC          ; void LegacyPcApplyForUser(int slot /*ecx*/, int user /*edx*/)

.code
Halo3PcHook PROC FRAME
    sub  rsp, 38h
    .allocstack 38h
    .endprolog
    mov  QWORD PTR [rsp+30h], rcx         ; user index survives the native call
    call QWORD PTR [g_halo3PcOrig]
    mov  QWORD PTR [rsp+28h], rax
    mov  rdx, QWORD PTR [rsp+30h]         ; user -> arg2
    xor  ecx, ecx                         ; slot 0 = Halo 3
    call LegacyPcApplyForUser
    mov  rax, QWORD PTR [rsp+28h]
    add  rsp, 38h
    ret
Halo3PcHook ENDP

OdstPcHook PROC FRAME
    sub  rsp, 38h
    .allocstack 38h
    .endprolog
    mov  QWORD PTR [rsp+30h], rcx
    call QWORD PTR [g_odstPcOrig]
    mov  QWORD PTR [rsp+28h], rax
    mov  rdx, QWORD PTR [rsp+30h]
    mov  ecx, 1                           ; slot 1 = ODST
    call LegacyPcApplyForUser
    mov  rax, QWORD PTR [rsp+28h]
    add  rsp, 38h
    ret
OdstPcHook ENDP

Halo4PcHook PROC FRAME
    sub  rsp, 38h
    .allocstack 38h
    .endprolog
    mov  QWORD PTR [rsp+30h], rcx
    call QWORD PTR [g_halo4PcOrig]
    mov  QWORD PTR [rsp+28h], rax
    mov  rdx, QWORD PTR [rsp+30h]
    mov  ecx, 2                           ; slot 2 = Halo 4
    call LegacyPcApplyForUser
    mov  rax, QWORD PTR [rsp+28h]
    add  rsp, 38h
    ret
Halo4PcHook ENDP
END
