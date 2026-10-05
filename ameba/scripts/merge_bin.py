#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import argparse
import subprocess
import logging
from pathlib import Path
from dataclasses import dataclass

# --- Import Internal Modules ---
THIS_DIR = Path(__file__).resolve().parent
sys.path.append(str(THIS_DIR / 'image_process'))

try:
    from image_process.context import Context
    from image_process.op_base import OperationEmpty
    from image_process.op_cut import Cut as op_cut
    from image_process.op_prepend_header import PrependHeader as op_prepend_header
    from image_process.op_pad import Pad as op_pad
    from image_process.utility import parse_map_file
    from image_process.ameba_layout_addrs import parse_amebasmart_layout_addrs
except ImportError as e:
    print(f"Error: Failed to import 'image_process' modules: {e}")
    sys.exit(1)

# --- Logging & Constants ---
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

AXF2BIN_SCRIPT = THIS_DIR / 'axf2bin.py'

@dataclass
class ToolchainConfig:
    objdump: Path
    strip: Path
    objcopy: Path

# --- Infrastructure Class ---

class FirmwarePacker:
    """
    Helper class providing tools, paths, and context management.
    Also provides high-level helpers for common patterns (extract/pad/header).
    """
    def __init__(self, args, soc_name):
        self.args = args
        self.soc_name = soc_name
        self.zephyr_bin = Path(args.bin_file).resolve()
        self.out_dir = Path(args.out_dir).resolve()
        self.module_dir = Path(args.module_dir).resolve()

        # Map external SoC names to internal directory names
        internal_map = {"amebag2": "amebagreen2"}
        self.internal_name = internal_map.get(soc_name, soc_name.lower())

        self.target_dir = self.out_dir / f'{self.internal_name}_gcc_project'
        self.image_dir = self.out_dir / 'images'

        # 1. Setup Toolchain first (independent)
        self.tools = self._setup_toolchain()

        # 2. Prepare Workspace IMMEDIATELY (Create dirs and copy manifest)
        # NOTE: Context initialization DEPENDS on manifest.json5 existing in target_dir
        self._prepare_workspace_internal()

        # 3. Setup Context (Now safe because manifest exists)
        self.context = self._setup_context()

    def _setup_toolchain(self) -> ToolchainConfig:
        variant = os.environ.get("ZEPHYR_TOOLCHAIN_VARIANT")
        if not variant:
            sys.exit("Error: ZEPHYR_TOOLCHAIN_VARIANT not set")

        # Ameba builds use Realtek's GNU Arm Embedded toolchain (asdk),
        # installed via `west realtek ameba install`, which sets the
        # gnuarmemb variant. No other variant is supported.
        if variant != "gnuarmemb":
            sys.exit(
                f"Error: unsupported ZEPHYR_TOOLCHAIN_VARIANT '{variant}'; "
                "Ameba requires 'gnuarmemb' (install via `west realtek ameba install`)"
            )

        prefix = "arm-none-eabi"
        gnu_path = os.environ.get("GNUARMEMB_TOOLCHAIN_PATH")
        if not gnu_path:
            sys.exit("Error: GNUARMEMB_TOOLCHAIN_PATH not set")
        toolchain_path = Path(gnu_path) / "bin"

        return ToolchainConfig(
            objdump=toolchain_path / f"{prefix}-objdump",
            strip=toolchain_path / f"{prefix}-strip",
            objcopy=toolchain_path / f"{prefix}-objcopy"
        )

    def _prepare_workspace_internal(self):
        """Internal setup: Create dirs and copy config files."""
        if self.image_dir.exists():
            shutil.rmtree(self.image_dir)
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.target_dir.mkdir(parents=True, exist_ok=True)

        # Critical: Copy manifest BEFORE Context init
        for f in ['manifest.json5', 'ameba_layout.ld']:
            src = self.module_dir / 'ameba' / self.soc_name / f
            if src.exists():
                shutil.copy(src, self.target_dir)
            else:
                logger.warning(f"Warning: {f} not found at {src}")

    def _setup_context(self) -> Context:
        setattr(self.args, "post_build_dir", self.target_dir)
        setattr(self.args, "log_level", 'WARNING')
        setattr(self.args, "mp", 'n')
        setattr(self.args, "extern_dir", None)
        return Context(self.args, OperationEmpty)

    def run_cmd(self, cmd: list):
        cmd_str = [str(c) for c in cmd]
        try:
            subprocess.check_call(cmd_str, stdout=subprocess.DEVNULL)
        except subprocess.CalledProcessError as e:
            logger.error(f"Command failed: {' '.join(cmd_str)}")
            sys.exit(e.returncode)

    def axf2bin_run(self, mode, *args):
        cmd = [sys.executable, str(AXF2BIN_SCRIPT)]
        cmd.extend(['--post-build-dir', str(self.target_dir)])
        cmd.append(mode)
        cmd.extend([str(a) for a in args])
        self.run_cmd(cmd)

    def concat_files(self, inputs: list, output: Path):
        with open(output, 'wb') as outfile:
            for f in inputs:
                if f.exists():
                    with open(f, 'rb') as infile:
                        shutil.copyfileobj(infile, outfile)

    def copy_blob(self, blob_name: str, dest_path: Path, optional=False):
        blob_path = self.module_dir / 'zephyr' / 'blobs' / 'ameba' / self.soc_name / 'bin' / blob_name
        if blob_path.exists():
            shutil.copy(blob_path, dest_path)
        elif not optional:
            logger.error(f"Required blob not found: {blob_path}")
            sys.exit(1)

    def finalize_output(self, boot_bin: Path, app_bin: Path):
        if app_bin.exists():
            shutil.move(str(boot_bin), str(self.image_dir))
            shutil.move(str(app_bin), str(self.image_dir))
            # Create aliases expected by AmebaFlash profile (boot.bin / app.bin)
            boot_alias = self.image_dir / 'boot.bin'
            app_alias = self.image_dir / 'app.bin'
            if not boot_alias.exists():
                shutil.copy(self.image_dir / boot_bin.name, boot_alias)
            if not app_alias.exists():
                shutil.copy(self.image_dir / app_bin.name, app_alias)
            logger.info(f"========== {self.soc_name} Image Done ==========")
        else:
            logger.error("Failed to generate application binary")
            sys.exit(1)

    # --- High Level Helpers (Standard Logic) ---

    def standard_process_img2(self, entry_symbol: str) -> Path:
        """
        Standard flow for AmebaD/G2:
        1. Strip AXF -> Extract XIP/SRAM/PSRAM
        2. Pad all 3 binaries
        3. Prepend Header (using provided entry_symbol for SRAM)
        4. Concatenate
        Returns: Path to the combined image2 binary
        """
        td = self.target_dir
        axf = td / 'target_pure_img2.axf'
        map_file = td / 'target_img2.map'

        # Output bin names
        xip_bin = td / 'xip_image2.bin'
        sram_bin = td / 'sram_2.bin'
        psram_bin = td / 'psram_2.bin'

        # 1. Initial Copy & Extract
        shutil.copy(self.zephyr_bin.with_suffix('.elf'), axf)
        shutil.copy(self.zephyr_bin.with_suffix('.raw.map'), map_file)
        shutil.copy(self.zephyr_bin, xip_bin)

        self.run_cmd([self.tools.strip, axf])
        self.run_cmd([self.tools.objcopy, '-j', '.ram_image2.entry', '-Obinary', axf, sram_bin])
        self.run_cmd([self.tools.objcopy, '-j', '.null.empty', '-Obinary', axf, psram_bin])

        # 2. Pad
        self.axf2bin_run('pad', '-i', xip_bin, '-l', 32)
        self.axf2bin_run('pad', '-i', sram_bin, '-l', 32)
        self.axf2bin_run('pad', '-i', psram_bin, '-l', 32)

        # 3. Prepend Header
        xip_pre = td / 'xip_image2_prepend.bin'
        sram_pre = td / 'sram_2_prepend.bin'
        psram_pre = td / 'psram_2_prepend.bin'

        self.axf2bin_run('prepend_header', '-o', sram_pre, '-i', sram_bin, '-s', entry_symbol, '-m', map_file)
        self.axf2bin_run('prepend_header', '-o', psram_pre, '-i', psram_bin, '-s', '__rom_start_address', '-m', map_file)
        self.axf2bin_run('prepend_header', '-o', xip_pre, '-i', xip_bin, '-s', '__rom_start_address', '-m', map_file)

        # 4. Concat
        # Determine prefix based on internal name for output file
        prefix = "km4tz" if "amebagreen2" in self.internal_name else "km4"
        img2_all = td / f'{prefix}_image2_all.bin'
        self.concat_files([xip_pre, sram_pre, psram_pre], img2_all)

        return img2_all


# --- Specific Logic per SoC ---

def handle_amebadplus_mcuboot(p: FirmwarePacker):
    """
    AmebaD Plus MCUboot Bootloader Generation
    Final image structure:
        amebadplus_boot.bin = [xip_boot_prepend + xip_boot_data] + [ram_1_prepend + ram_1_data]
    """
    td = p.target_dir
    zephyr_elf = p.zephyr_bin.with_suffix('.elf')
    raw_map_file = p.zephyr_bin.parent.parent / 'zephyr' / 'zephyr.raw.map'

    axf = td / 'km4_boot.axf'
    shutil.copy(zephyr_elf, axf)
    p.run_cmd([p.tools.strip, axf])

    # 1. Generate clean XIP binary without .ram_image1.entry
    xip_all = td / 'xip_all.bin'
    p.run_cmd([
        p.tools.objcopy,
        '-O', 'binary',
        '--remove-section=.ram_image1.entry',  # Exclude RAM segment from XIP binary
        str(axf),
        str(xip_all),
    ])

    # 2. Remove padding between __rom_region_start and __km4_boot_text_start__
    pad_start = parse_map_file(str(raw_map_file), "__rom_region_start")
    pad_end   = parse_map_file(str(raw_map_file), "__km4_boot_text_start__")

    if not pad_start or not pad_end:
        logger.error("Error: Boot symbols not found in map file")
        sys.exit(1)

    pad_len = int(pad_end[0], 16) - int(pad_start[0], 16)

    xip_boot = td / 'xip_boot.bin'
    if pad_len > 0:
        op_cut.execute(p.context, str(xip_all), str(xip_boot), pad_len)
    else:
        shutil.copy(xip_all, xip_boot)

    # 3. Add padding and header to xip segment
    op_pad.execute(p.context, str(xip_boot), 32)

    xip_boot_pre = td / 'xip_boot_prepend.bin'
    op_prepend_header.execute(
        p.context,
        str(xip_boot_pre),
        str(xip_boot),
        str(raw_map_file),
        "__km4_boot_text_start__",   # XIP boot entry point (FLASH address)
        0xFFFFFFFF                   # AmebaD Plus doesn't use 0x01010101 marker
    )

    # 4. Extract RAM segment data (.ram_image1.entry) separately
    #    This section contains RamStartTable, RAM_IMG1_VALID_PATTEN, boot_export_symbol, etc.
    ram_1 = td / 'ram_1.bin'
    p.run_cmd([
        p.tools.objcopy,
        "-O", "binary",
        "--only-section=.ram_image1.entry",  # Extract only RAM segment data (~0x60 bytes)
        str(axf),
        str(ram_1),
    ])

    # 5. Add header to ram segment
    ram_1_pre = td / 'ram_1_prepend.bin'
    op_prepend_header.execute(
        p.context,
        str(ram_1_pre),
        str(ram_1),
        str(raw_map_file),
        "__ram_start_table_start__",
    )

    # 6. Concatenate to generate final_boot Image
    km4_boot = td / 'km4_boot.bin'
    p.concat_files([xip_boot_pre, ram_1_pre], km4_boot)

    final_boot = td / 'boot.bin'
    p.axf2bin_run('fw_pack', '-o', final_boot, '--image1', km4_boot)

    if final_boot.exists():
        shutil.copy(final_boot, p.image_dir)
        logger.info("========== AmebaDPlus MCUBoot Image Done ==========")
    else:
        logger.error("Failed to generate MCUBoot image for AmebaD Plus")
        sys.exit(1)

def handle_amebadplus(p: FirmwarePacker):
    """
    Handle AmebaDplus specific logic.
    Note: Has a separate 'entry.bin' section, so it doesn't use the standard helper.
    """
    td = p.target_dir
    axf = td / 'target_pure_img2.axf'
    map_file = td / 'target_img2.map'

    # 1. Custom Extract
    shutil.copy(p.zephyr_bin.with_suffix('.elf'), axf)
    shutil.copy(p.zephyr_bin.with_suffix('.raw.map'), map_file)
    xip_bin = td / 'xip_image2.bin'
    shutil.copy(p.zephyr_bin, xip_bin)

    p.run_cmd([p.tools.strip, axf])
    # Dplus specific: Extract entry section separately
    p.run_cmd([p.tools.objcopy, '-j', '.ram_image2.entry', '-Obinary', axf, td / 'entry.bin'])
    p.run_cmd([p.tools.objcopy, '-j', '.null.empty', '-Obinary', axf, td / 'sram_2.bin'])
    p.run_cmd([p.tools.objcopy, '-j', '.null.empty', '-Obinary', axf, td / 'psram_2.bin'])

    p.axf2bin_run('pad', '-i', xip_bin, '-l', 32)

    # 2. Header & Concat
    entry_pre = td / 'entry_prepend.bin'
    xip_pre = td / 'xip_image2_prepend.bin'
    sram_pre = td / 'sram_2_prepend.bin'
    psram_pre = td / 'psram_2_prepend.bin'

    p.axf2bin_run('prepend_header', '-o', entry_pre, '-i', td / 'entry.bin', '-s', '__KM4_IMG2_ENTRY_start', '-m', map_file)
    p.axf2bin_run('prepend_header', '-o', sram_pre, '-i', td / 'sram_2.bin', '-s', '_image_ram_start', '-m', map_file)
    p.axf2bin_run('prepend_header', '-o', psram_pre, '-i', td / 'psram_2.bin', '-s', '__rom_start_address', '-m', map_file)
    p.axf2bin_run('prepend_header', '-o', xip_pre, '-i', xip_bin, '-s', '__rom_start_address', '-m', map_file)

    km4_img2 = td / 'km4_image2_all.bin'
    p.concat_files([xip_pre, sram_pre, psram_pre, entry_pre], km4_img2)

    km4_tfm_ns = p.target_dir.parent / 'tfm_ns' / 'bin' / 'km4_image2_all.bin'
    if km4_tfm_ns.exists():
        shutil.copy(km4_tfm_ns, km4_img2)

    # 3. Blobs & Pack
    km4_boot = td / 'boot.bin'
    p.copy_blob('km4_boot_all.bin', km4_boot)

    km0_blob_name = 'km0_image2_all_coex.bin' if p.args.bt_coexist else 'km0_image2_all.bin'
    km0_img2 = td / km0_blob_name
    p.copy_blob(km0_blob_name, km0_img2)
    km4_img3 = td / 'km4_image3_all.bin'
    p.copy_blob('km4_image3_all.bin', km4_img3, optional=True)

    km4tz_tfm = p.target_dir.parent / 'tfm' / 'bin' / 'km4_image3_all.bin'
    if km4tz_tfm.exists():
        shutil.copy(km4tz_tfm, km4_img3)

    boot_cut = td / 'km4_boot.bin'
    p.axf2bin_run('cut', '-o', boot_cut, '-i', km4_boot, '-l', 4096)
    p.axf2bin_run('fw_pack', '-o', km4_boot, '--image1', boot_cut)

    app_bin = td / 'app.bin'
    pack_args = ['-o', app_bin, '--image2', km0_img2, km4_img2]
    if km4_img3.exists(): pack_args.extend(['--image3', km4_img3])

    p.axf2bin_run('fw_pack', *pack_args)
    p.finalize_output(km4_boot, app_bin)


def handle_amebad(p: FirmwarePacker):
    # 1. Standard Image2 Processing
    km4_img2 = p.standard_process_img2(entry_symbol='__image2_entry_func__')

    # 2. Blobs
    boot_all = p.target_dir / 'bootloader_all.bin'
    p.copy_blob('bootloader_all.bin', boot_all)

    km0_img2 = p.target_dir / 'km0_image2_all.bin'
    p.copy_blob('km0_image2_all.bin', km0_img2)

    km4_img3 = p.target_dir / 'km4_image3_all.bin'
    p.copy_blob('km4_image3_all.bin', km4_img3, optional=True)

    # 3. Pack
    app_bin = p.target_dir / 'km0_km4_app.bin'
    pack_args = ['-o', app_bin, '--image2', km0_img2, km4_img2, '--sboot-for-image', '1']
    if km4_img3.exists(): pack_args.extend(['--image3', km4_img3])

    p.axf2bin_run('fw_pack', *pack_args)
    p.finalize_output(boot_all, app_bin)


def handle_amebag2(p: FirmwarePacker):
    # 1. Standard Image2 Processing
    km4tz_img2 = p.standard_process_img2(entry_symbol='__image2_entry_func__')

    km4tz_tfm_ns = p.target_dir.parent / 'tfm_ns' / 'bin' / 'km4tz_image2_all.bin'
    if km4tz_tfm_ns.exists():
        shutil.copy(km4tz_tfm_ns, km4tz_img2)

    # 2. Blobs
    boot_bin = p.target_dir / 'boot.bin'
    p.copy_blob('amebagreen2_boot.bin', boot_bin)

    km4ns_blob_name = 'km4ns_image2_all_coex.bin' if p.args.bt_coexist else 'km4ns_image2_all.bin'
    km4ns_img2 = p.target_dir / km4ns_blob_name
    p.copy_blob(km4ns_blob_name, km4ns_img2)

    km4tz_img3 = p.target_dir / 'km4tz_image3_all.bin'
    p.copy_blob('km4tz_image3_all.bin', km4tz_img3, optional=True)

    km4tz_tfm = p.target_dir.parent / 'tfm' / 'bin' / 'km4tz_image3_all.bin'
    if km4tz_tfm.exists():
        shutil.copy(km4tz_tfm, km4tz_img3)

    # 3. Pack
    boot_cut = p.target_dir / 'km4tz_boot.bin'
    p.axf2bin_run('cut', '-o', boot_cut, '-i', boot_bin, '-l', 4096)
    p.axf2bin_run('fw_pack', '-o', boot_bin, '--image1', boot_cut)

    app_bin = p.target_dir / 'app.bin'
    pack_args = ['-o', app_bin, '--image2', km4ns_img2, km4tz_img2]
    if km4tz_img3.exists(): pack_args.extend(['--image3', km4tz_img3])

    p.axf2bin_run('fw_pack', *pack_args)
    p.finalize_output(boot_bin, app_bin)


def handle_amebag2_mcuboot(p: FirmwarePacker):
    td = p.target_dir
    raw_map_file = p.zephyr_bin.parent.parent / 'zephyr' / 'zephyr.raw.map'

    pad_start = parse_map_file(str(raw_map_file), "__rom_region_start")
    pad_end = parse_map_file(str(raw_map_file), "__km4tz_boot_text_start__")

    if not pad_start or not pad_end:
        sys.exit("Error: Symbols missing in map file for MCUBoot")

    pad_len = int(pad_end[0], 16) - int(pad_start[0], 16)
    xip_boot = td / 'xip_boot.bin'

    if pad_len > 0:
        op_cut.execute(p.context, str(p.zephyr_bin), str(xip_boot), pad_len)
    else:
        shutil.copy(p.zephyr_bin, xip_boot)

    op_pad.execute(p.context, str(xip_boot), 32)

    xip_boot_pre = td / 'xip_boot_prepend.bin'
    op_prepend_header.execute(p.context, str(xip_boot_pre), str(xip_boot), str(raw_map_file), "__km4tz_boot_text_start__", 0x01010101)

    ram_1_pre = td / 'ram_1_prepend.bin'
    # Create empty file
    with open(td / 'ram_1.bin', "wb"): pass
    op_prepend_header.execute(p.context, str(ram_1_pre), str(td / 'ram_1.bin'), str(raw_map_file), "__km4tz_boot_text_start__")

    km4tz_boot = td / 'km4tz_boot.bin'
    p.concat_files([xip_boot_pre, ram_1_pre], km4tz_boot)

    final_boot = td / 'boot.bin'
    p.axf2bin_run('fw_pack', '-o', final_boot, '--image1', km4tz_boot)

    if final_boot.exists():
        shutil.copy(final_boot, p.image_dir)
        logger.info("========== MCUBoot Image Done ==========")
    else:
        logger.error("Failed to generate MCUBoot image")
        sys.exit(1)


def _amebasmart_positional_image2(p: FirmwarePacker, slots: list, out_name: str):
    """Build a vendor-format per-core image2 with the FULL positional slot set
    and emit it as <out_name> into both the gcc_project/ dir and images/.

    IMG1 walks the concatenated km0+km4+ap image2 sub-images *sequentially by
    position* -- each per-core blob must therefore be self-contained and carry
    every positional slot (XIP / SRAM / DRAM) that the vendor blob has, with the
    real payload in the correct slot and empty 32-byte-header stubs (len=0) in
    the others.  A short blob would shift every following sub-image and cascade
    into "AP XIP IMG Invalid".

    `slots` is an ordered list of dicts:
        {'load': <addr>, 'payload': <Path or None>, 'label': <str>}
    A payload of None emits an empty stub (32-byte header, len=0) at that load
    address; a real payload is padded to 32B and carried after its header.

    Each 32-byte header is byte-for-byte the vendor image2 header format
    (op_prepend_header: 0x3831393538373131 "81958711" sig + LE length +
    LE load-addr + 16x 0xFF).
    """
    td = p.target_dir
    empty = td / '_img2_empty.bin'
    empty.write_bytes(b'')  # zero-length payload for stub slots

    parts = []
    for i, s in enumerate(slots):
        label = s.get('label', f'slot{i}')
        if s['payload'] is None:
            # Empty stub: 32-byte header only (len=0) at this load address.
            src = empty
        else:
            # Real payload: copy + pad to 32B (matches vendor / standard_process_img2).
            src = td / f'{label}_payload.bin'
            shutil.copy(s['payload'], src)
            p.axf2bin_run('pad', '-i', src, '-l', 32)
        pre = td / f'{label}_prepend.bin'
        p.axf2bin_run('prepend_header', '-o', pre, '-i', src,
                      '--address', hex(s['load']))
        parts.append(pre)

    # Concatenate the positional sub-images into the final per-core image2.
    out_img2 = td / out_name
    p.concat_files(parts, out_img2)
    shutil.copy(out_img2, p.image_dir / out_name)
    return out_img2


def handle_amebasmart_km4_app(p: FirmwarePacker):
    """
    AmebaSmart (RTL8730E) KM4 (Cortex-M55) Zephyr *application* image2.

    Mirrors the vendor km4_image2_all.bin 3-slot positional layout exactly:

        #1 XIP  stub  (len=0) @ KM4_IMG2_XIP  (0x0D000020)
        #2 SRAM stub  (len=0) @ KM4_BD_RAM    (0x20014020)
        #3 DRAM REAL          @ KM4_BD_DRAM   (0x60000020)  <- our zephyr.bin

    The KM4 rpmsg-remote app is therefore built NON-XIP (CONFIG_XIP=n): the whole
    zephyr.bin links/runs from PSRAM at KM4_BD_DRAM (0x60000020, == the km4 dts
    sram0 PSRAM window) and is carried verbatim in the DRAM slot -- matching how
    the vendor KM4 firmware is a single DRAM sub-image at 0x60000020.  IMG1 loads
    the DRAM sub-image to PSRAM and enters it there.
    """
    # Resolve all slot addresses from ameba_layout.ld (single source of truth).
    layout_addr = parse_amebasmart_layout_addrs(
        Path(__file__).resolve().parents[1] / 'amebasmart' / 'ameba_layout.ld')

    km4_img2_name = 'km4_image2_all_coex.bin' if p.args.bt_coexist else 'km4_image2_all.bin'
    slots = [
        {'load': layout_addr['km4_xip'],     'payload': None,         'label': 'km4_xip'},
        {'load': layout_addr['km4_bd_ram'],  'payload': None,         'label': 'km4_sram'},
        {'load': layout_addr['km4_bd_dram'], 'payload': p.zephyr_bin, 'label': 'km4_dram'},
    ]
    _amebasmart_positional_image2(p, slots, km4_img2_name)
    logger.info(f"========== AmebaSmart KM4 app image2 Done ({km4_img2_name}) ==========")


def handle_amebasmart_km0_app(p: FirmwarePacker):
    """
    AmebaSmart (RTL8730E) KM0 (Cortex-M23, low-power) Zephyr *application*
    image2.

    Mirrors the vendor km0_image2_all.bin 3-slot positional layout exactly:

        #1 XIP  stub  (len=0) @ KM0_IMG2_XIP  (0x0C000020)
        #2 SRAM REAL          @ KM0_BD_RAM    (0x23002020)  <- our zephyr.bin
        #3 DRAM stub  (len=0) @ KM0_BD_DRAM   (0x6FFFFFFF sentinel)

    KM0 is built CONFIG_XIP=n: the whole zephyr.bin is SRAM-resident and is
    loaded verbatim to the KM0 SRAM entry (KM0_BD_RAM 0x23002020, ==
    CONFIG_SRAM_BASE_ADDRESS, verified in the KM0 build map) -- the real payload
    goes in the SRAM slot, the XIP and DRAM slots are empty stubs.  The full
    3-slot layout keeps IMG1's sequential sub-image walk aligned.
    """
    # Resolve all slot addresses from ameba_layout.ld (single source of truth).
    layout_addr = parse_amebasmart_layout_addrs(
        Path(__file__).resolve().parents[1] / 'amebasmart' / 'ameba_layout.ld')

    km0_img2_name = 'km0_image2_all_coex.bin' if p.args.bt_coexist else 'km0_image2_all.bin'
    slots = [
        {'load': layout_addr['km0_xip'],     'payload': None,         'label': 'km0_xip'},
        {'load': layout_addr['km0_bd_ram'],  'payload': p.zephyr_bin, 'label': 'km0_sram'},
        {'load': layout_addr['km0_bd_dram'], 'payload': None,         'label': 'km0_dram'},
    ]
    _amebasmart_positional_image2(p, slots, km0_img2_name)
    logger.info(f"========== AmebaSmart KM0 app image2 Done ({km0_img2_name}) ==========")


def _find_amebasmart_sibling_image2(p: FirmwarePacker, *names: str):
    """Locate OUR per-core image2 produced by a sibling sysbuild domain (the
    km0/km4 Zephyr apps added via ExternalZephyrProject_Add).

    Each core-app domain emits its packaged image2 into <domain-build>/images/
    (see handle_amebasmart_km{0,4}_app -> p.image_dir).  In a sysbuild build the
    CA32 (default) domain's build dir (p.out_dir) is a sibling of the km0/km4
    domain build dirs under the sysbuild top build tree (p.out_dir.parent), so
    we scan <top>/ * /images/<name>, excluding the CA32 domain's own images/.

    <names> is tried in order; the first present wins.  This lets the caller pass
    both the coex and non-coex variant names so a BT_COEXIST mismatch between the
    CA32 and core-app builds still resolves.

    Returns the Path of the first match, or None (caller falls back to the
    vendor blob).
    """
    top = p.out_dir.parent  # sysbuild top-level build dir
    # Only a real sysbuild top (marked by domains.yaml) may donate sibling
    # images; a standalone build in a shared directory must not absorb images
    # from unrelated neighbouring build trees.
    if not (top / 'domains.yaml').is_file():
        return None
    self_images = p.image_dir.resolve()
    for name in names:
        for cand in sorted(top.glob(f'*/images/{name}')):
            if cand.parent.resolve() == self_images:
                continue
            if cand.is_file():
                return cand
    return None


def handle_amebasmart(p: FirmwarePacker):
    """
    AmebaSmart (RTL8730E) Cortex-A32 image generation.

    Zephyr runs from DRAM (not XIP). ATF BL2 loads BL33 (zephyr.bin) from
    FIP to NS_DRAM0_BASE=0x60300000 at runtime.

    Flow (matching ameba-rtos-github postbuild):
      1. ATF build produces bl1_sram.bin, bl1.bin, fip.bin (FIP contains BL33=zephyr.bin)
      2. xip_image2 is an empty 32-byte header (CONFIG_IMG2_PSRAM=y)
      3. Pad + Prepend Header for bl1_sram, bl1, fip
      4. ap_image_all.bin = xip_header(32B) + bl1_sram_pre + bl1_pre + fip_pre
      5. km0_km4_ca32_app.bin via fw_pack --image2
    """
    td = p.target_dir

    # 1. Copy MAP file for debug symbols
    raw_map = p.zephyr_bin.with_suffix('.raw.map')
    if not raw_map.exists():
        raw_map = p.zephyr_bin.with_suffix('.map')
    map_file = td / 'target_img2.map'
    shutil.copy(raw_map, map_file)

    # 2. Create empty xip_image2 (0 bytes - CONFIG_IMG2_PSRAM=y means no XIP code)
    xip_bin = td / 'xip_image2.bin'
    with open(xip_bin, 'wb') as f:
        pass  # Empty file, prepend_header will produce a header-only output

    # 3. Collect ATF build outputs (BL33=zephyr.bin is already inside fip.bin)
    tfa_image_dir = p.zephyr_bin.parent.parent / 'tfa' / 'project_ap' / 'image'

    bl1_sram = tfa_image_dir / 'bl1_sram.bin'
    bl1 = tfa_image_dir / 'bl1.bin'
    fip = tfa_image_dir / 'fip.bin'

    for f in [bl1_sram, bl1, fip]:
        if not f.exists():
            logger.error(f"ATF output not found: {f}")
            sys.exit(1)

    # 4. Pad to 32-byte alignment
    p.axf2bin_run('pad', '-i', bl1_sram, '-l', 32)
    p.axf2bin_run('pad', '-i', bl1, '-l', 32)
    p.axf2bin_run('pad', '-i', fip, '-l', 32)

    # 5. Generate synthetic map file with ATF layout symbols.
    layout_addr = parse_amebasmart_layout_addrs(
        Path(__file__).resolve().parents[1] / 'amebasmart' / 'ameba_layout.ld')
    atf_map = td / 'atf_layout.map'
    with open(atf_map, 'w') as f:
        f.write(f"{hex(layout_addr['xip'])}  .xip_image2.text  __flash_text_start__\n")
        f.write(f"{hex(layout_addr['bl1_sram'])}  .ca32_bl1_sram    __ca32_bl1_sram_start__\n")
        f.write(f"{hex(layout_addr['bl1_dram'])}  .ca32_bl1_dram    __ca32_bl1_dram_start__\n")
        f.write(f"{hex(layout_addr['fip'])}  .ca32_fip_dram    __ca32_fip_dram_start__\n")

    # 6. Prepend headers
    xip_pre = td / 'xip_image2_prepend.bin'
    bl1_sram_pre = td / 'bl1_sram_prepend.bin'
    bl1_pre = td / 'bl1_prepend.bin'
    fip_pre = td / 'fip_prepend.bin'

    p.axf2bin_run('prepend_header', '-o', xip_pre, '-i', xip_bin,
                  '-s', '__flash_text_start__', '-m', atf_map)
    p.axf2bin_run('prepend_header', '-o', bl1_sram_pre, '-i', bl1_sram,
                  '-s', '__ca32_bl1_sram_start__', '-m', atf_map)
    p.axf2bin_run('prepend_header', '-o', bl1_pre, '-i', bl1,
                  '-s', '__ca32_bl1_dram_start__', '-m', atf_map)
    p.axf2bin_run('prepend_header', '-o', fip_pre, '-i', fip,
                  '-s', '__ca32_fip_dram_start__', '-m', atf_map)

    # 7. Assemble ap_image_all.bin
    ap_image = td / 'ap_image_all.bin'
    p.concat_files([xip_pre, bl1_sram_pre, bl1_pre, fip_pre], ap_image)

    # 8. Gather the KM0 + KM4 image2 sub-images.
    #
    # Prefer OUR sibling-domain outputs (the km0/km4 Zephyr apps packaged by
    # handle_amebasmart_km0_app / handle_amebasmart_km4_app, emitted into each
    # domain's images/ dir under the sysbuild top build tree).  This mirrors the
    # amebadplus tfm_ns sibling-domain pattern.  Fall back to the vendor blob
    # only when no sibling output is present (e.g. CA32-only build).
    km0_img2 = td / 'km0_image2_all.bin'
    km0_sibling = _find_amebasmart_sibling_image2(
        p, 'km0_image2_all.bin', 'km0_image2_all_coex.bin')
    if km0_sibling is not None:
        logger.info(f"Using OUR KM0 image2 from sibling domain: {km0_sibling}")
        shutil.copy(km0_sibling, km0_img2)
    else:
        logger.info("No sibling KM0 image2 found; falling back to vendor blob")
        p.copy_blob('km0_image2_all.bin', km0_img2)

    km4_img2_name = 'km4_image2_all_coex.bin' if p.args.bt_coexist else 'km4_image2_all.bin'
    km4_img2 = td / km4_img2_name
    km4_alt_name = 'km4_image2_all.bin' if p.args.bt_coexist else 'km4_image2_all_coex.bin'
    km4_sibling = _find_amebasmart_sibling_image2(p, km4_img2_name, km4_alt_name)
    if km4_sibling is not None:
        logger.info(f"Using OUR KM4 image2 from sibling domain: {km4_sibling}")
        shutil.copy(km4_sibling, km4_img2)
    else:
        logger.info("No sibling KM4 image2 found; falling back to vendor blob")
        p.copy_blob(km4_img2_name, km4_img2)

    # 9. fw_pack --image2: generates cert.bin + manifest.bin, handles RSIP, and assembles final app
    km0_km4_ca32_app = td / 'km0_km4_ca32_app.bin'
    pack_args = ['-o', km0_km4_ca32_app, '--image2', km0_img2, km4_img2, ap_image]
    p.axf2bin_run('fw_pack', *pack_args)

    # 10. Process boot image
    km4_boot = td / 'km4_boot_all.bin'
    p.copy_blob('km4_boot_all.bin', km4_boot)
    boot_cut = td / 'km4_boot.bin'
    p.axf2bin_run('cut', '-o', boot_cut, '-i', km4_boot, '-l', 4096)
    p.axf2bin_run('fw_pack', '-o', km4_boot, '--image1', boot_cut)

    p.finalize_output(km4_boot, km0_km4_ca32_app)

# --- Main ---

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--soc", required=True)
    parser.add_argument("--bin-file", default='', help="Zephyr binary")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--module-dir", required=True)
    parser.add_argument("--bt-coexist", action="store_true")
    parser.add_argument("--mcuboot", action="store_true")
    parser.add_argument("--core", choices=['km4', 'km0', 'ca32'], default=None,
                        help="For multi-core SoCs (amebasmart): which core's image2 to "
                             "package. 'km4'/'km0' emit the per-core image2 (XIP) blob; "
                             "default (None) keeps the legacy CA32/ATF combined-app flow.")
    args = parser.parse_args()

    try:
        # Initialize packer (this will automatically prepare workspace and context)
        packer = FirmwarePacker(args, args.soc)
    except Exception:
        logger.exception("Initialization failed")
        sys.exit(1)

    try:
        if args.mcuboot:
            if args.soc == "amebag2":
                handle_amebag2_mcuboot(packer)
            elif args.soc == "amebadplus":
                handle_amebadplus_mcuboot(packer)
            elif args.soc == "amebasmart":
                logger.error("amebasmart MCUBoot app assembly is driven by mcuboot_app_image.cmake")
                sys.exit(1)
            else:
                logger.error(f"MCUBoot not supported for {args.soc}")
                sys.exit(1)
        else:
            if args.soc == "amebadplus":
                handle_amebadplus(packer)
            elif args.soc == "amebad":
                handle_amebad(packer)
            elif args.soc == "amebag2":
                handle_amebag2(packer)
            elif args.soc == "amebasmart":
                if args.core == "km4":
                    handle_amebasmart_km4_app(packer)
                elif args.core == "km0":
                    handle_amebasmart_km0_app(packer)
                else:
                    handle_amebasmart(packer)
            else:
                logger.error(f"Unsupported SoC: {args.soc}")
                sys.exit(1)

    except Exception:
        logger.exception("Firmware processing failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
