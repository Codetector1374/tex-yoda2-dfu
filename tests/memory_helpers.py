"""Execute the linked Thumb memory helpers, not host-language surrogates.

Requires a release build, GNU Arm nm, and Python's unicorn package.
Run from the repository root: python3 tests/memory_helpers.py
"""
import pathlib
import subprocess
import unittest

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn import UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE, UC_MEM_WRITE
from unicorn.arm_const import (
    UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_LR,
    UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R7,
    UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11,
    UC_ARM_REG_SP, UC_ARM_REG_PC,
)

ROOT = pathlib.Path(__file__).resolve().parents[1]
ELF = ROOT / "target/thumbv7m-none-eabi/release/bootloader"
IMAGE = ROOT / "target/bootloader.bin"


class Memcpy(unittest.TestCase):
    def test_bounds_alignment_return_value_and_callee_saved_registers(self):
        symbols = subprocess.check_output(["arm-none-eabi-nm", str(ELF)], text=True)
        entry = next(int(row.split()[0], 16) for row in symbols.splitlines()
                     if row.split()[-1] == "memcpy")
        cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        cpu.mem_map(0, 0x4000)
        cpu.mem_write(0, IMAGE.read_bytes())
        cpu.mem_map(0x20000000, 0x4000)
        stop = 0x3000
        registers = [UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R7,
                     UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11,
                     UC_ARM_REG_SP]
        saved = [0x12340000 + i for i in range(8)] + [0x20004000]
        source = bytes((i * 37 + 11) & 255 for i in range(4096))
        guard = b"\xa5" * 4096
        region = [0, 0, 0]

        def access(_cpu, kind, address, size, _value, _context):
            src, dst, length = region
            start = dst if kind == UC_MEM_WRITE else src
            self.assertTrue(start <= address and address + size <= start + length,
                            f"out-of-range memory access: {address:#x}+{size}")

        cpu.hook_add(UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, access)
        lengths = list(range(18)) + [31, 32, 63, 64, 65, 255, 256, 511, 512, 513, 1024]
        cases = 0
        for src_page, dst_page in [(0x20001000, 0x20002000), (0x20002000, 0x20001000)]:
            for src_align in range(8):
                for dst_align in range(8):
                    for length in lengths:
                        with self.subTest(src=src_align, dst=dst_align, length=length,
                                          reverse=src_page > dst_page):
                            src, dst = src_page + src_align, dst_page + dst_align
                            cpu.mem_write(src_page, source)
                            cpu.mem_write(dst_page, guard)
                            region[:] = src, dst, length
                            for reg, value in zip(registers, saved):
                                cpu.reg_write(reg, value)
                            cpu.reg_write(UC_ARM_REG_R0, dst)
                            cpu.reg_write(UC_ARM_REG_R1, src)
                            cpu.reg_write(UC_ARM_REG_R2, length)
                            cpu.reg_write(UC_ARM_REG_LR, stop | 1)
                            cpu.emu_start(entry | 1, stop, count=6 * length + 10)
                            self.assertEqual(cpu.reg_read(UC_ARM_REG_PC), stop)
                            self.assertEqual(cpu.reg_read(UC_ARM_REG_R0), dst)
                            self.assertEqual([cpu.reg_read(r) for r in registers], saved)
                            expected = bytearray(guard)
                            expected[dst_align:dst_align + length] = source[src_align:src_align + length]
                            self.assertEqual(bytes(cpu.mem_read(dst_page, 4096)), expected)
                            self.assertEqual(bytes(cpu.mem_read(src_page, 4096)), source)
                            cases += 1
        print(f"Validated {cases} linked Thumb memcpy cases")


class Memclr(unittest.TestCase):
    def test_zeroing_boundaries_alignment_and_callee_saved_registers(self):
        symbols = subprocess.check_output(["arm-none-eabi-nm", str(ELF)], text=True)
        entry = next(int(row.split()[0], 16) for row in symbols.splitlines()
                     if row.split()[-1] == "__aeabi_memclr4")
        cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        cpu.mem_map(0, 0x4000)
        cpu.mem_write(0, IMAGE.read_bytes())
        page = 0x20001000
        cpu.mem_map(page, 4096)
        stop = 0x3000
        registers = [UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R7,
                     UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11,
                     UC_ARM_REG_SP]
        saved = [0x56780000 + i for i in range(8)] + [0x20002000]
        guard = b"\xa5" * 4096
        region = [0, 0]

        def access(_cpu, kind, address, size, value, _context):
            start, length = region
            self.assertEqual(kind, UC_MEM_WRITE, "zeroing must not read data memory")
            self.assertTrue(start <= address and address + size <= start + length,
                            f"out-of-range zeroing: {address:#x}+{size}")
            self.assertEqual(value, 0)

        cpu.hook_add(UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, access)
        lengths = list(range(18)) + [31, 32, 63, 64, 65, 255, 256, 511, 512, 513, 1024]
        # Exercise tails at every alignment, including ones beyond AEABI's
        # aligned-entry preconditions. The byte loop supports all of them.
        cases = [(page + offset, length) for offset in range(8) for length in lengths]
        cases.append((0xdeadbeef, 0))  # no memory access for zero length
        for destination, length in cases:
            with self.subTest(destination=hex(destination), length=length):
                cpu.mem_write(page, guard)
                region[:] = destination, length
                for reg, value in zip(registers, saved):
                    cpu.reg_write(reg, value)
                cpu.reg_write(UC_ARM_REG_R0, destination)
                cpu.reg_write(UC_ARM_REG_R1, length)
                cpu.reg_write(UC_ARM_REG_LR, stop | 1)
                cpu.emu_start(entry | 1, stop, count=5 * length + 10)
                self.assertEqual(cpu.reg_read(UC_ARM_REG_PC), stop)
                self.assertEqual([cpu.reg_read(r) for r in registers], saved)
                expected = bytearray(guard)
                if length:
                    offset = destination - page
                    expected[offset:offset + length] = bytes(length)
                self.assertEqual(bytes(cpu.mem_read(page, 4096)), expected)
        print(f"Validated {len(cases)} linked Thumb memclr cases")


if __name__ == "__main__":
    unittest.main()
