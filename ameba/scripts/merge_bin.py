#!/usr/bin/env python3
import os
import sys
import shutil
import argparse
import subprocess
from pathlib import Path

AXF2BIN_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'axf2bin.py')

GCC_OBJDUMP = None
GCC_STRIP = None
GCC_FROMELF = None

def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)

def axf2bin_pad(input_file, length):
    cmd = [sys.executable, AXF2BIN_SCRIPT, 'pad', '-i', input_file, '-l', str(length)]
    cmd_exec(cmd)

def axf2bin_prepend_head(output_file, input_file, symbol, map_file):
    cmd = [sys.executable, AXF2BIN_SCRIPT, "prepend_header",
           '-o', output_file,
           '-i', input_file,
           '-s', symbol,
           '-m', map_file]
    cmd_exec(cmd)

def ameba_axf2bin_fw_pack(target_dir, output_file, *input_files):
    cmd = [sys.executable, AXF2BIN_SCRIPT, '--post-build-dir', target_dir,
           'fw_pack', '-o', output_file, *input_files]
    cmd_exec(cmd)

def concatenate_files(input_files, output_file):
    with open(output_file, 'wb') as outfile:
        for file in input_files:
            with open(file, 'rb') as infile:
                shutil.copyfileobj(infile, outfile)

def copy_file_if_exists(src, dst):
    src_path = Path(src)
    if src_path.exists():
        shutil.copy(src_path, dst)
    else:
        pass

def copy_bin_file(src, dst):
    src_path = Path(src)
    if not src_path.is_file():
        raise FileNotFoundError(f"Source file does not exist: {src_path}, run `west blobs fetch hal_realtek`")
    shutil.copy(src_path, dst)

def path_leaf(path):
    p = Path(path)
    return p.name if p.name else p.parent.name

def get_toolchain_config():
    global GCC_OBJDUMP, GCC_STRIP, GCC_FROMELF

    variant = os.environ.get("ZEPHYR_TOOLCHAIN_VARIANT")
    if not variant:
        sys.exit("Error: ZEPHYR_TOOLCHAIN_VARIANT environment variable is not set")

    if variant == "zephyr":
        toolchain_prefix = "arm-zephyr-eabi"
        sdk_dir = os.environ.get("ZEPHYR_SDK_INSTALL_DIR")
        if not sdk_dir:
            sys.exit("Error: ZEPHYR_SDK_INSTALL_DIR is not set for 'zephyr' toolchain")
        toolchain_path = os.path.join(sdk_dir, "arm-zephyr-eabi", "bin")

    elif variant == "gnuarmemb":
        toolchain_prefix = "arm-none-eabi"
        gnuarmemb_path = os.environ.get("GNUARMEMB_TOOLCHAIN_PATH")
        if not gnuarmemb_path:
            sys.exit("Error: GNUARMEMB_TOOLCHAIN_PATH is not set for 'gnuarmemb' toolchain")
        toolchain_path = os.path.join(gnuarmemb_path, "bin")

    else:
        sys.exit(f"Unsupported ZEPHYR_TOOLCHAIN_VARIANT: {variant}")

    GCC_OBJDUMP = os.path.join(toolchain_path, toolchain_prefix + "-objdump")
    GCC_FROMELF = os.path.join(toolchain_path, toolchain_prefix + "-objcopy")
    GCC_STRIP = os.path.join(toolchain_path, toolchain_prefix + "-strip")

def if_enable_config(config_file, target_str):
    with open(config_file, 'r') as infile:
        for line in infile:
            if target_str in line:
                return True
    return False

def amebadplus_bin_handle(bt_coexist):
    target_dir = Path(out_dir) / 'amebadplus_gcc_project'
    target_dir.mkdir(parents=True, exist_ok=True)

    entry_bin = target_dir / 'entry.bin'
    xip_image2_bin = target_dir / 'xip_image2.bin'
    sram_2_bin = target_dir / 'sram_2.bin'
    psram_2_bin = target_dir / 'psram_2.bin'

    target_pure_img2_axf = target_dir / 'target_pure_img2.axf'
    target_img2_map = target_dir / 'target_img2.map'

    if bt_coexist:
        km0_image2_all_bin = target_dir / 'km0_image2_all_coex.bin'
    else:
        km0_image2_all_bin = target_dir / 'km0_image2_all.bin'
    km4_image2_all_bin = target_dir / 'km4_image2_all.bin'
    km4_image3_all_bin = target_dir / 'km4_image3_all.bin'

    km4_boot_all_bin = target_dir / 'km4_boot_all.bin'
    km0_km4_app_bin = target_dir / 'km0_km4_app.bin'

    shutil.copy(Path(zephyr_bin).with_suffix('.elf'), target_pure_img2_axf)
    shutil.copy(Path(zephyr_bin).with_suffix('.raw.map'), target_img2_map)
    shutil.copy(zephyr_bin, xip_image2_bin)

    cmd_exec((GCC_STRIP, target_pure_img2_axf))
    cmd_exec((GCC_FROMELF, '-j', '.ram_image2.entry', '-Obinary', target_pure_img2_axf, entry_bin))
    cmd_exec((GCC_FROMELF, '-j', '.null.empty', '-Obinary', target_pure_img2_axf, sram_2_bin))
    cmd_exec((GCC_FROMELF, '-j', '.null.empty', '-Obinary', target_pure_img2_axf, psram_2_bin))

    axf2bin_pad(xip_image2_bin, 32)

    entry_prepend_bin = target_dir / 'entry_prepend.bin'
    xip_image2_prepend_bin = target_dir / 'xip_image2_prepend.bin'
    sram_2_prepend_bin = target_dir / 'sram_2_prepend.bin'
    psram_2_prepend_bin = target_dir / 'psram_2_prepend.bin'

    axf2bin_prepend_head(entry_prepend_bin, entry_bin, '__KM4_IMG2_ENTRY_start', target_img2_map)
    axf2bin_prepend_head(sram_2_prepend_bin, sram_2_bin, '_image_ram_start', target_img2_map)
    axf2bin_prepend_head(psram_2_prepend_bin, psram_2_bin, '__rom_start_address', target_img2_map)
    axf2bin_prepend_head(xip_image2_prepend_bin, xip_image2_bin, '__rom_start_address', target_img2_map)

    input_files = [xip_image2_prepend_bin, sram_2_prepend_bin, psram_2_prepend_bin, entry_prepend_bin]
    concatenate_files(input_files, km4_image2_all_bin)

    blobs_dir = Path(module_dir) / 'zephyr' / 'blobs' / soc_name / 'bin'
    copy_bin_file(blobs_dir / path_leaf(km4_boot_all_bin), km4_boot_all_bin)
    copy_bin_file(blobs_dir / path_leaf(km0_image2_all_bin), km0_image2_all_bin)
    copy_file_if_exists(blobs_dir / path_leaf(km4_image3_all_bin), km4_image3_all_bin)
    shutil.copy(os.path.join(module_dir, 'ameba', soc_name, 'manifest.json5'), target_dir)
    shutil.copy(os.path.join(module_dir, 'ameba', soc_name, 'ameba_layout.ld'), target_dir)

    cmd_exec((sys.executable, AXF2BIN_SCRIPT, 'cut', '-o', target_dir / 'km4_boot.bin', '-i', km4_boot_all_bin, '-l', '4096'))
    ameba_axf2bin_fw_pack(target_dir.resolve(), km4_boot_all_bin, '--image1', target_dir / 'km4_boot.bin')

    if os.path.exists(km4_image3_all_bin):
        ameba_axf2bin_fw_pack(target_dir.resolve(), km0_km4_app_bin, '--image2', km0_image2_all_bin, km4_image2_all_bin, '--image3', km4_image3_all_bin)
    else:
        ameba_axf2bin_fw_pack(target_dir.resolve(), km0_km4_app_bin, '--image2', km0_image2_all_bin, km4_image2_all_bin)

    if os.path.exists(km0_km4_app_bin):
        shutil.move(km4_boot_all_bin, image_dir)
        shutil.move(km0_km4_app_bin, image_dir)
        print('========== Image manipulating done ==========')
    else:
        print('Error: Fail to manipulate images')
        sys.exit(1)

def amebad_bin_handle():
    target_dir = Path(out_dir) / 'amebad_gcc_project'
    target_dir.mkdir(parents=True, exist_ok=True)

    xip_image2_bin = target_dir / 'xip_image2.bin'
    sram_2_bin = target_dir / 'sram_2.bin'
    psram_2_bin = target_dir / 'psram_2.bin'

    target_pure_img2_axf = target_dir / 'target_pure_img2.axf'
    target_img2_map = target_dir / 'target_img2.map'

    km0_image2_all_bin = target_dir / 'km0_image2_all.bin'
    km4_image2_all_bin = target_dir / 'km4_image2_all.bin'
    km4_image3_all_bin = target_dir / 'km4_image3_all.bin'

    bootloader_all_bin = Path(target_dir) / 'bootloader_all.bin'
    km0_km4_app_bin = target_dir / 'km0_km4_app.bin'

    shutil.copy(Path(zephyr_bin).with_suffix('.elf'), target_pure_img2_axf)
    shutil.copy(Path(zephyr_bin).with_suffix('.raw.map'), target_img2_map)
    shutil.copy(zephyr_bin, xip_image2_bin)

    cmd_exec((GCC_STRIP, target_pure_img2_axf))
    cmd_exec((GCC_FROMELF, '-j', '.ram_image2.entry', '-Obinary', target_pure_img2_axf, sram_2_bin))
    cmd_exec((GCC_FROMELF, '-j', '.null.empty', '-Obinary', target_pure_img2_axf, psram_2_bin))

    axf2bin_pad(xip_image2_bin, 32)
    axf2bin_pad(sram_2_bin, 32)
    axf2bin_pad(psram_2_bin, 32)

    xip_image2_prepend_bin = target_dir / 'xip_image2_prepend.bin'
    sram_2_prepend_bin = target_dir / 'sram_2_prepend.bin'
    psram_2_prepend_bin = target_dir / 'psram_2_prepend.bin'

    axf2bin_prepend_head(sram_2_prepend_bin, sram_2_bin, '__KM4_IMG2_ENTRY_start', target_img2_map)
    axf2bin_prepend_head(psram_2_prepend_bin, psram_2_bin, '__rom_start_address', target_img2_map)
    axf2bin_prepend_head(xip_image2_prepend_bin, xip_image2_bin, '__rom_start_address', target_img2_map)

    input_files = [xip_image2_prepend_bin, sram_2_prepend_bin, psram_2_prepend_bin]
    concatenate_files(input_files, km4_image2_all_bin)

    blobs_dir = Path(module_dir) / 'zephyr' / 'blobs' / soc_name / 'bin'
    copy_bin_file(blobs_dir / path_leaf(bootloader_all_bin), bootloader_all_bin)
    copy_bin_file(blobs_dir / path_leaf(km0_image2_all_bin), km0_image2_all_bin)
    copy_file_if_exists(blobs_dir / path_leaf(km4_image3_all_bin), km4_image3_all_bin)
    shutil.copy(os.path.join(module_dir, 'ameba', soc_name, 'manifest.json5'), target_dir)
    shutil.copy(os.path.join(module_dir, 'ameba', soc_name, 'ameba_layout.ld'), target_dir)

    if os.path.exists(km4_image3_all_bin):
        ameba_axf2bin_fw_pack(target_dir.resolve(), km0_km4_app_bin, '--image2', km0_image2_all_bin, km4_image2_all_bin, '--image3', km4_image3_all_bin, '--sboot-for-image', '1')
    else:
        ameba_axf2bin_fw_pack(target_dir.resolve(), km0_km4_app_bin, '--image2', km0_image2_all_bin, km4_image2_all_bin, '--sboot-for-image', '1')

    if os.path.exists(km0_km4_app_bin):
        shutil.move(bootloader_all_bin, image_dir)
        shutil.move(km0_km4_app_bin, image_dir)
        print('========== Image manipulating done ==========')
    else:
        print('Error: Fail to manipulate images')
        sys.exit(1)

def amebaG2_bin_handle(bt_coexist):
    target_dir = Path(out_dir) / 'amebagreen2_gcc_project'
    target_dir.mkdir(parents=True, exist_ok=True)

    xip_image2_bin = target_dir / 'xip_image2.bin'
    sram_2_bin = target_dir / 'sram_2.bin'
    psram_2_bin = target_dir / 'psram_2.bin'

    target_pure_img2_axf = target_dir / 'target_pure_img2.axf'
    target_img2_map = target_dir / 'target_img2.map'

    if bt_coexist:
        km4ns_image2_all_bin = target_dir / 'km4ns_image2_all_coex.bin'
    else:
        km4ns_image2_all_bin = target_dir / 'km4ns_image2_all.bin'
    km4tz_image2_all_bin = target_dir / 'km4tz_image2_all.bin'
    km4tz_image3_all_bin = target_dir / 'km4tz_image3_all.bin'

    amebagreen2_boot_bin = target_dir / 'amebagreen2_boot.bin'
    amebagreen2_app_bin = target_dir / 'amebagreen2_app.bin'

    shutil.copy(Path(zephyr_bin).with_suffix('.elf'), target_pure_img2_axf)
    shutil.copy(Path(zephyr_bin).with_suffix('.raw.map'), target_img2_map)
    shutil.copy(zephyr_bin, xip_image2_bin)

    cmd_exec((GCC_STRIP, target_pure_img2_axf))
    cmd_exec((GCC_FROMELF, '-j', '.ram_image2.entry', '-Obinary', target_pure_img2_axf, sram_2_bin))
    cmd_exec((GCC_FROMELF, '-j', '.null.empty', '-Obinary', target_pure_img2_axf, psram_2_bin))

    axf2bin_pad(xip_image2_bin, 32)
    axf2bin_pad(sram_2_bin, 32)
    axf2bin_pad(psram_2_bin, 32)

    xip_image2_prepend_bin = target_dir / 'xip_image2_prepend.bin'
    sram_2_prepend_bin = target_dir / 'sram_2_prepend.bin'
    psram_2_prepend_bin = target_dir / 'psram_2_prepend.bin'

    axf2bin_prepend_head(sram_2_prepend_bin, sram_2_bin, '__KM4TZ_IMG2_ENTRY_start', target_img2_map)
    axf2bin_prepend_head(psram_2_prepend_bin, psram_2_bin, '__rom_start_address', target_img2_map)
    axf2bin_prepend_head(xip_image2_prepend_bin, xip_image2_bin, '__rom_start_address', target_img2_map)

    input_files = [xip_image2_prepend_bin, sram_2_prepend_bin, psram_2_prepend_bin]
    concatenate_files(input_files, km4tz_image2_all_bin)

    blobs_dir = Path(module_dir) / 'zephyr' / 'blobs' / soc_name / 'bin'
    copy_bin_file(blobs_dir / path_leaf(amebagreen2_boot_bin), amebagreen2_boot_bin)
    if if_enable_config(Path(zephyr_bin).parent / ".config", 'CONFIG_AMEBAGREEN2_A_CUT=y'):
        copy_bin_file(blobs_dir / 'a-cut' / path_leaf(km4ns_image2_all_bin), km4ns_image2_all_bin)
    else:
        copy_bin_file(blobs_dir / path_leaf(km4ns_image2_all_bin), km4ns_image2_all_bin)

    copy_file_if_exists(blobs_dir / path_leaf(km4tz_image3_all_bin), km4tz_image3_all_bin)
    shutil.copy(os.path.join(module_dir, 'ameba', soc_name, 'manifest.json5'), target_dir)
    shutil.copy(os.path.join(module_dir, 'ameba', soc_name, 'ameba_layout.ld'), target_dir)

    cmd_exec((sys.executable, AXF2BIN_SCRIPT, 'cut', '-o', target_dir / 'km4tz_boot.bin', '-i', amebagreen2_boot_bin, '-l', '4096'))
    ameba_axf2bin_fw_pack(target_dir.resolve(), amebagreen2_boot_bin, '--image1', target_dir / 'km4tz_boot.bin')

    if os.path.exists(km4tz_image3_all_bin):
        ameba_axf2bin_fw_pack(target_dir.resolve(), amebagreen2_app_bin, '--image2', km4ns_image2_all_bin, km4tz_image2_all_bin, '--image3', km4tz_image3_all_bin)
    else:
        ameba_axf2bin_fw_pack(target_dir.resolve(), amebagreen2_app_bin, '--image2', km4ns_image2_all_bin, km4tz_image2_all_bin)

    if os.path.exists(amebagreen2_app_bin):
        shutil.move(amebagreen2_boot_bin, image_dir)
        shutil.move(amebagreen2_app_bin, image_dir)
        print('========== Image manipulating done ==========')
    else:
        print('Error: Fail to manipulate images')
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--soc", required=True, help="SoC name (e.g., amebadplus)")
    parser.add_argument("--bin-file", required=True, help="Path to the kernel binary file")
    parser.add_argument("--out-dir", required=True, help="Output directory for generated files")
    parser.add_argument("--module-dir", required=True, help="hal realtek module directory")
    parser.add_argument("--bt-coexist", action="store_true", help="Enable Bluetooth coexistence features")
    args = parser.parse_args()

    zephyr_bin = args.bin_file
    soc_name = args.soc
    out_dir = args.out_dir
    module_dir = args.module_dir

    image_dir = Path(out_dir) / 'images'
    if image_dir.exists():
        shutil.rmtree(image_dir)
    image_dir.mkdir(parents=True, exist_ok=True)

    get_toolchain_config()

    if (soc_name == 'amebadplus'):
        amebadplus_bin_handle(args.bt_coexist)
    elif (soc_name == 'amebad'):
        amebad_bin_handle()
    elif (soc_name == 'amebaG2'):
        amebaG2_bin_handle(args.bt_coexist)
    else:
        print(f"Error: Unsupported SoC name '{soc_name}'")
        sys.exit(1)
