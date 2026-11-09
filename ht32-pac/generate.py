#!/usr/bin/env python3
"""Generate the complete polling-only PAC: python3 ht32-pac/generate.py.

Input is the unmodified, licensed local SVD. Corrections are applied only to a
throwaway copy: strip redundant peripheral register prefixes; normalize GPIO
registers and cluster USB endpoints following ht32-rs 88c62cdc448ad5028d51cd6a48843305ed29a1a1;
normalize PWRCU base/offsets without changing absolute addresses, and make BAKTEST
read-only as specified in User Manual Rev. 1.01 pp. 71, 75. See NOTICE.

The vendored chiptool has one local addition, --no-interrupts, which omits only
Cortex-M interrupt/runtime metadata. No register block is omitted. No formatter
or external PAC generator is involved. The original SVD and generated output
remain tracked; temporary intermediate XML and device.x are not retained.
"""

from copy import deepcopy
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


CHIPTOOL_REVISION = "e8902440a6eee471a9b14b38ee1cd2e526d2c495"
HERE = Path(__file__).resolve().parent
CHIPTOOL = HERE.parent.parent / "chiptool"


def set_text(element, name, value):
    child = element.find(name)
    if child is None:
        child = ET.SubElement(element, name)
    child.text = value


def prepare_svd(source, output):
    tree = ET.parse(source)
    root = tree.getroot()
    for peripheral in root.findall("peripherals/peripheral"):
        name = peripheral.findtext("name")
        for register in peripheral.findall("registers/register"):
            original = register.findtext("name")
            prefix = name + "_"
            if original.startswith(prefix):
                set_text(register, "name", original[len(prefix):])
                set_text(register, "displayName", original[len(prefix):])
        if name == "PWRCU":
            # The SVD uses 0x4006A100 plus offsets 0x0..0x124. The manual
            # uses 0x4006A000 plus offsets 0x100..0x224: identical addresses.
            set_text(peripheral, "baseAddress", "0x4006A000")
            for register in peripheral.findall("registers/register"):
                offset = int(register.findtext("addressOffset"), 0)
                set_text(register, "addressOffset", hex(offset + 0x100))
                if register.findtext("name") == "BAKTEST":
                    set_text(register, "access", "read-only")
                    for field in register.findall("fields/field"):
                        set_text(field, "access", "read-only")
            block = peripheral.find("addressBlock")
            set_text(block, "offset", "0x100")
            set_text(block, "size", "0x128")
        if name == "USB":
            registers = peripheral.find("registers")
            by_name = {r.findtext("name"): r for r in registers}
            for endpoint, count, cluster_name, description in (
                (0, 1, "EP0", "USB control endpoint"),
                (1, 3, "EP%sS", "Single-buffered endpoints 1..3 (zero-based array index)"),
                (4, 4, "EP%sD", "Double-buffered endpoints 4..7 (zero-based array index)"),
            ):
                cluster = ET.Element("cluster")
                if count > 1:
                    ET.SubElement(cluster, "dim").text = str(count)
                    ET.SubElement(cluster, "dimIncrement").text = "0x14"
                    ET.SubElement(cluster, "dimIndex").text = ",".join(
                        str(i) for i in range(endpoint, endpoint + count)
                    )
                ET.SubElement(cluster, "name").text = cluster_name
                ET.SubElement(cluster, "description").text = description
                base = int(by_name[f"EP{endpoint}CSR"].findtext("addressOffset"), 0)
                ET.SubElement(cluster, "addressOffset").text = hex(base)
                for suffix in ("CSR", "IER", "ISR", "TCR", "CFGR"):
                    register = deepcopy(by_name[f"EP{endpoint}{suffix}"])
                    set_text(register, "name", suffix)
                    set_text(register, "displayName", suffix)
                    offset = int(register.findtext("addressOffset"), 0)
                    set_text(register, "addressOffset", hex(offset - base))
                    cluster.append(register)
                    for i in range(endpoint, endpoint + count):
                        registers.remove(by_name[f"EP{i}{suffix}"])
                registers.append(cluster)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main():
    revision = subprocess.check_output(
        ["git", "-C", str(CHIPTOOL), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != CHIPTOOL_REVISION:
        raise SystemExit(f"chiptool must be pinned to {CHIPTOOL_REVISION}, got {revision}")
    subprocess.run(
        ["cargo", "+stable", "build", "--locked", "--manifest-path", str(CHIPTOOL / "Cargo.toml")],
        check=True,
    )
    metadata = subprocess.check_output(
        ["cargo", "+stable", "metadata", "--no-deps", "--format-version=1", "--manifest-path", str(CHIPTOOL / "Cargo.toml")],
        text=True,
    )
    import json
    generator = Path(json.loads(metadata)["target_directory"]) / "debug" / "chiptool"
    with tempfile.TemporaryDirectory(prefix="ht32-pac-") as temporary:
        temporary = Path(temporary)
        prepared = temporary / "HT32F1653_54.svd"
        prepare_svd(HERE / "svd" / "HT32F1653_54.svd", prepared)
        subprocess.run(
            [str(generator), "generate", "--svd", str(prepared), "--transform", str(HERE / "transforms.yaml"), "--no-defmt", "--no-interrupts", "--output", str(temporary)],
            check=True,
        )
        header = '''//! Full ownership-free peripheral access for HT32F1653/HT32F1654.
//!
//! Generated from the licensed local SVD with `python3 ht32-pac/generate.py`.
//! Global peripheral constants and register handles are Copy; there is no
//! singleton acquisition API, heap, HAL, interrupt table, or runtime dependency.
//! Use e.g. `CKCU.gccr().modify(|r| r.set_hseen(true))` and
//! `FMC.opcr().read().opm()` (logical OPM values, bits 4:1).
//! USB endpoints: `USB.ep0()`, `USB.eps(n)` for endpoints 1..3 (n=0..2),
//! and `USB.epd(n)` for endpoints 4..7 (n=0..3).
//! PWRCU uses the manual's base 0x4006A000: BAKTEST is offset 0x108,
//! BAKREG0 is offset 0x200. No absolute register addresses were changed.
//! Register writes and read/modify/write are not synchronized. Callers must
//! respect hardware access, write-one-to-clear flags and reserved-bit rules.
//! Source provenance, redistribution terms and tracked corrections: see NOTICE.
#![doc = include_str!("../NOTICE")]
'''
        (HERE / "src").mkdir(exist_ok=True)
        (HERE / "src" / "lib.rs").write_text(header + (temporary / "lib.rs").read_text() + "\n")


if __name__ == "__main__":
    main()
