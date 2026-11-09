/* Cortex-M3 size-first memcpy. The caller supplies valid, non-overlapping
 * ranges. Byte accesses support all alignments without touching adjacent bytes.
 * Zero length performs no memory access. r0 (the original destination) is
 * returned for the C ABI; only caller-saved r1/r2/r3/r12 and flags are clobbered.
 * The AEABI aliases share this routine, which also accepts unaligned buffers.
 */
.syntax unified
.thumb
.section .text.memcpy,"ax",%progbits
.p2align 1
.global memcpy
.global __aeabi_memcpy
.global __aeabi_memcpy4
.global __aeabi_memcpy8
.type memcpy,%function
.thumb_func
memcpy:
    mov r12, r0
    cbz r2, 2f
1:
    ldrb r3, [r1], #1
    strb r3, [r12], #1
    subs r2, r2, #1
    bne 1b
2:
    bx lr
.size memcpy, .-memcpy
.thumb_set __aeabi_memcpy, memcpy
.thumb_set __aeabi_memcpy4, memcpy
.thumb_set __aeabi_memcpy8, memcpy
