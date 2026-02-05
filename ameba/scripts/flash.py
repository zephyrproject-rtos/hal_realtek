#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

import sys
import argparse
import base64
import json
import subprocess
import shutil
import logging
from enum import IntEnum
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Path constants definition
THIS_DIR = Path(__file__).resolve().parent
BLOBS_ROOT = (THIS_DIR / "../../zephyr/blobs/ameba").resolve()

# Tool paths
DEFAULT_FLASH_TOOL = (THIS_DIR / 'flash' / 'AmebaFlash.py').resolve()
PROFILES_PATH = (THIS_DIR / "flash" / "Devices" / "Profiles").resolve()
FLOADERS_DEST = (THIS_DIR / "flash" / "Devices" / "Floaders").resolve()

class MemoryType(IntEnum):
    RAM = 0
    NOR = 1
    NAND = 2

# Device configuration mapping
DEVICE_MAP = {
    "amebag2": {
        "profile": "RTL8721F_NOR.rdev",
        "floader": "amebag2/bin/floader_amebagreen2.bin"
    },
    "amebadplus": {
        "profile": "RTL8721Dx.rdev",
        "floader": "amebadplus/bin/floader_amebadplus.bin"
    }
}

def prepare_flash_environment(device: str) -> str:
    """
    Validate device, copy necessary bootloader (floader), and return Profile file path.
    """
    if not device:
        raise ValueError("Device argument must not be empty.")

    key = device.lower()
    if key not in DEVICE_MAP:
        valid_options = ", ".join(sorted(DEVICE_MAP.keys()))
        raise ValueError(f"Unsupported device: '{device}'. Valid options: {valid_options}")

    config = DEVICE_MAP[key]

    # 1. Determine Profile path
    profile_path = PROFILES_PATH / config["profile"]
    if not profile_path.is_file():
        raise FileNotFoundError(f"Profile file not found: {profile_path}")

    # 2. Handle Floader copy
    floader_src = BLOBS_ROOT / config["floader"]
    if not floader_src.is_file():
        raise FileNotFoundError(f"Floader file for {key} not found at: {floader_src}")

    FLOADERS_DEST.mkdir(parents=True, exist_ok=True)

    # Copy file (overwrite to ensure correctness)
    try:
        shutil.copy2(floader_src, FLOADERS_DEST)
    except Exception as e:
        logger.error(f"Failed to copy floader: {e}")
        raise

    return str(profile_path)

def validate_tool_path(tool_path: str) -> Path:
    """Validate if the flash tool path exists."""
    path = Path(tool_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Flash tool not found: {path}")
    return path

def setup_parser(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--tool-path', help='Path to flash.py (absolute or relative)')
    parser.add_argument('--device', required=True, help='Device type (e.g., amebag2)')
    parser.add_argument('--image-dir', required=True, type=Path, help="Directory containing image files")

    # Serial configuration
    parser.add_argument('--port', action='append', required=True,
                        help='Serial port (repeatable), e.g. --port COM3')
    parser.add_argument('--baudrate', default='1500000', help='Serial baudrate (default: 1500000)')

    # Memory and addressing
    parser.add_argument('--memory-type', choices=['nor', 'nand', 'ram'], default='nor',
                        help='Memory type (default: nor)')
    parser.add_argument('--images', nargs=3, action='append', metavar=('image-name', 'start-address', 'end-address'),
                        help="User defined image layout: name start_addr end_addr")

    # Erase control
    parser.add_argument('--chip-erase', action='store_true', help='Perform Chip Erase (NOR only)')

    # Logging and remote
    parser.add_argument('--log-level', default='info', choices=['info', 'debug', 'warn', 'error'], help='Log level')
    parser.add_argument('--log-file', help='Log file path')
    parser.add_argument('--remote-server', help='Remote serial server IP')
    parser.add_argument('--remote-password', help='Remote serial server password')

def build_partition_table(image_dir: Path, images: list, memory_type_str: str) -> str:
    """
    Build partition table and encode in Base64.
    """
    partition_table = []

    # Map memory type string to Enum value
    mem_type_map = {
        "nor": MemoryType.NOR,
        "nand": MemoryType.NAND,
        "ram": MemoryType.RAM
    }
    mem_type_val = mem_type_map.get(memory_type_str, MemoryType.NOR)

    for img_name, start_addr_str, end_addr_str in images:
        full_image_path = (image_dir / img_name).resolve()

        # Pre-check if file exists to provide friendlier error
        if not full_image_path.is_file():
             raise FileNotFoundError(f"Image file not found: {full_image_path}")

        try:
            start_addr = int(start_addr_str, 16)
            end_addr = int(end_addr_str, 16)
        except ValueError as e:
            raise ValueError(f"Invalid address format for image {img_name}: {e}")

        partition_table.append({
            "ImageName": str(full_image_path),
            "StartAddress": start_addr,
            "EndAddress": end_addr,
            "FullErase": False,
            "MemoryType": int(mem_type_val),
            "Mandatory": True,
            "Description": img_name
        })

    # Serialize -> Encode -> Base64
    json_bytes = json.dumps(partition_table).encode('utf-8')
    return base64.b64encode(json_bytes).decode('utf-8')

def main(args):
    # 1. Prepare environment (Profile & Floader)
    try:
        profile_path = prepare_flash_environment(args.device)
    except Exception as e:
        logger.error(str(e))
        sys.exit(1)

    # 2. Determine tool path
    tool_path_str = args.tool_path if args.tool_path else str(DEFAULT_FLASH_TOOL)
    try:
        tool_path = validate_tool_path(tool_path_str)
    except FileNotFoundError as e:
        logger.error(str(e))
        sys.exit(1)

    # 3. Build base command
    cmd = [sys.executable, str(tool_path), '--download', '--profile', profile_path]

    # Add serial port arguments
    cmd.append('--port')
    cmd.extend(args.port) # args.port is already a list
    cmd.extend(['--baudrate', args.baudrate])
    cmd.extend(['--memory-type', args.memory_type])
    cmd.extend(['--log-level', args.log_level])

    # 4. Optional arguments
    if args.log_file:
        cmd.extend(['--log-file', args.log_file])

    if args.remote_server:
        cmd.extend(['--remote-server', args.remote_server])
        if args.remote_password:
            cmd.extend(['--remote-password', str(args.remote_password)])

    if args.chip_erase:
        cmd.extend(['--chip-erase'])

    # 5. Image and partition handling
    if not args.images:
        # Simple mode: pass directory directly
        if not args.image_dir.is_dir():
            logger.error(f"Image directory does not exist: {args.image_dir}")
            sys.exit(1)
        cmd.extend(['--image-dir', str(args.image_dir)])
    else:
        # Advanced mode: custom partition table
        try:
            b64_table = build_partition_table(args.image_dir, args.images, args.memory_type)
            cmd.extend(['--partition-table', b64_table])
        except Exception as e:
            logger.error(f"Failed to build partition table: {e}")
            sys.exit(1)

    # 6. Execute command
    logger.info(f"Executing flash tool: {' '.join(cmd)}")
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        logger.error(f"Flash tool exited with error code {e.returncode}")
        sys.exit(e.returncode)
    except KeyboardInterrupt:
        logger.warning("\nOperation cancelled by user.")
        sys.exit(130)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Realtek Ameba Flash Wrapper')
    setup_parser(parser)
    args = parser.parse_args()
    main(args)
