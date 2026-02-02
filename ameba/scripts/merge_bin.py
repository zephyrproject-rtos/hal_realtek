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
except ImportError:
    print("Error: Failed to import 'image_process' modules.")
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
        internal_map = {"amebaG2": "amebagreen2"}
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

        toolchain_path = None
        prefix = None

        if variant == "zephyr":
            prefix = "arm-zephyr-eabi"
            sdk_dir = os.environ.get("ZEPHYR_SDK_INSTALL_DIR")
            if not sdk_dir:
                sys.exit("Error: ZEPHYR_SDK_INSTALL_DIR not set")
            toolchain_path = Path(sdk_dir) / "arm-zephyr-eabi" / "bin"
        elif variant == "gnuarmemb":
            prefix = "arm-none-eabi"
            gnu_path = os.environ.get("GNUARMEMB_TOOLCHAIN_PATH")
            if not gnu_path:
                sys.exit("Error: GNUARMEMB_TOOLCHAIN_PATH not set")
            toolchain_path = Path(gnu_path) / "bin"
        else:
            sys.exit(f"Unsupported toolchain variant: {variant}")

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
        if mode == 'fw_pack':
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

    # 3. Blobs & Pack
    km4_boot = td / 'km4_boot_all.bin'
    p.copy_blob('km4_boot_all.bin', km4_boot)

    km0_blob_name = 'km0_image2_all_coex.bin' if p.args.bt_coexist else 'km0_image2_all.bin'
    km0_img2 = td / km0_blob_name
    p.copy_blob(km0_blob_name, km0_img2)
    km4_img3 = td / 'km4_image3_all.bin'
    p.copy_blob('km4_image3_all.bin', km4_img3, optional=True)

    boot_cut = td / 'km4_boot.bin'
    p.axf2bin_run('cut', '-o', boot_cut, '-i', km4_boot, '-l', 4096)
    p.axf2bin_run('fw_pack', '-o', km4_boot, '--image1', boot_cut)

    app_bin = td / 'km0_km4_app.bin'
    pack_args = ['-o', app_bin, '--image2', km0_img2, km4_img2]
    if km4_img3.exists(): pack_args.extend(['--image3', km4_img3])

    p.axf2bin_run('fw_pack', *pack_args)
    p.finalize_output(km4_boot, app_bin)


def handle_amebad(p: FirmwarePacker):
    # 1. Standard Image2 Processing
    km4_img2 = p.standard_process_img2(entry_symbol='__KM4_IMG2_ENTRY_start')

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


def handle_amebaG2(p: FirmwarePacker):
    # 1. Standard Image2 Processing
    km4tz_img2 = p.standard_process_img2(entry_symbol='__KM4TZ_IMG2_ENTRY_start')

    # 2. Blobs
    boot_bin = p.target_dir / 'amebagreen2_boot.bin'
    p.copy_blob('amebagreen2_boot.bin', boot_bin)

    km4ns_blob_name = 'km4ns_image2_all_coex.bin' if p.args.bt_coexist else 'km4ns_image2_all.bin'
    km4ns_img2 = p.target_dir / km4ns_blob_name
    p.copy_blob(km4ns_blob_name, km4ns_img2)

    km4tz_img3 = p.target_dir / 'km4tz_image3_all.bin'
    p.copy_blob('km4tz_image3_all.bin', km4tz_img3, optional=True)

    # 3. Pack
    boot_cut = p.target_dir / 'km4tz_boot.bin'
    p.axf2bin_run('cut', '-o', boot_cut, '-i', boot_bin, '-l', 4096)
    p.axf2bin_run('fw_pack', '-o', boot_bin, '--image1', boot_cut)

    app_bin = p.target_dir / 'amebagreen2_app.bin'
    pack_args = ['-o', app_bin, '--image2', km4ns_img2, km4tz_img2]
    if km4tz_img3.exists(): pack_args.extend(['--image3', km4tz_img3])

    p.axf2bin_run('fw_pack', *pack_args)
    p.finalize_output(boot_bin, app_bin)


def handle_amebaG2_mcuboot(p: FirmwarePacker):
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

    final_boot = td / 'amebagreen2_boot.bin'
    p.axf2bin_run('fw_pack', '-o', final_boot, '--image1', km4tz_boot)

    if final_boot.exists():
        shutil.copy(final_boot, p.image_dir)
        logger.info("========== MCUBoot Image Done ==========")
    else:
        logger.error("Failed to generate MCUBoot image")
        sys.exit(1)

# --- Main ---

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--soc", required=True)
    parser.add_argument("--bin-file", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--module-dir", required=True)
    parser.add_argument("--bt-coexist", action="store_true")
    parser.add_argument("--mcuboot", action="store_true")
    parser.add_argument("--sign-version")
    args = parser.parse_args()

    try:
        # Initialize packer (this will automatically prepare workspace and context)
        packer = FirmwarePacker(args, args.soc)
    except Exception:
        logger.exception("Initialization failed")
        sys.exit(1)

    try:
        if args.mcuboot:
            if args.soc == "amebaG2":
                handle_amebaG2_mcuboot(packer)
            else:
                logger.error(f"MCUBoot not supported for {args.soc}")
                sys.exit(1)
        else:
            if args.soc == "amebadplus":
                handle_amebadplus(packer)
            elif args.soc == "amebad":
                handle_amebad(packer)
            elif args.soc == "amebaG2":
                handle_amebaG2(packer)
            else:
                logger.error(f"Unsupported SoC: {args.soc}")
                sys.exit(1)

    except Exception:
        logger.exception("Firmware processing failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
