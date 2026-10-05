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
import tempfile
import logging
from enum import IntEnum
from pathlib import Path
import os
import pickle
from typing import Dict, Tuple, List, Optional

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
    },
    "amebasmart": {
        "profile": "RTL8730E_NOR.rdev",
        "floader": "amebasmart/bin/floader_amebasmart.bin"
    }
}

XIP_BASE: int = 0x08000000  # XIP base address
EDT_DEFAULT_NAME: str = "edt.pickle"
ZEPHYR_DIR_NAME: str = "zephyr"


def _get_file_to_label(device: str, image_dir: str = "") -> Dict[str, str]:
    """
    Return filename → partition-label mapping for the given device.

    In a sysbuild + MCUBoot build the image directories contain:
      - boot.bin : MCUBoot bootloader  (mcuboot_image.cmake)
      - app.bin  : signed application  (mcuboot_app_image.cmake)

    In a TFM BL2 build (tfm_s_signed.bin present) the mapping is:
      - boot.bin           : BL2 bootloader   → "bootloader"
      - tfm_s_signed.bin   : TFM-S image      → "image-0"
      - app.bin            : NS application   → "image-0-nonsecure"
    """
    import os
    if image_dir and os.path.isfile(os.path.join(image_dir, "tfm_s_signed.bin")):
        return {
            "boot.bin":         "bootloader",
            "tfm_s_signed.bin": "image-0",
            "app.bin":          "image-0-nonsecure",
        }
    return {
        "boot.bin": "bootloader",
        "app.bin":  "image-0",
    }

def resolve_edt_pickle_path(image_dir: str) -> str:
    """
    Resolve the path to edt.pickle based on the typical build layout:
    Prefer: <image_dir>/../zephyr/edt.pickle
    Raises FileNotFoundError if not found.
    """
    image_dir_abs = os.path.abspath(image_dir)
    p = os.path.abspath(os.path.join(image_dir_abs, os.pardir, ZEPHYR_DIR_NAME, EDT_DEFAULT_NAME))
    if not os.path.isfile(p):
        raise FileNotFoundError(f"EDT pickle not found at {p}")
    return p


def _safe_get_label(node) -> Optional[str]:
    """
    Extract the 'label' property value from a node in a robust way.
    """
    props = getattr(node, "props", {})
    label_prop = props.get("label")
    if label_prop is not None:
        # edtlib Property usually exposes .val; keep fallback to .value
        val = getattr(label_prop, "val", None)
        if val is None:
            val = getattr(label_prop, "value", None)
        if val is not None:
            return str(val)
    # Fallback if node has a direct attribute (unlikely, but safe)
    direct = getattr(node, "label", None)
    return str(direct) if direct is not None else None


def _safe_get_first_reg(node) -> Optional[Tuple[int, int]]:
    """
    Extract the first (addr, size) pair from node.regs.
    Returns None if not available.
    """
    regs = getattr(node, "regs", None)
    if not regs:
        return None
    reg0 = regs[0]
    addr = getattr(reg0, "addr", None)
    size = getattr(reg0, "size", None)
    if addr is None or size is None:
        return None
    return int(addr), int(size)

def collect_image_load_list(image_dir: str, images: List[List[str]],
                            file_to_label: Optional[Dict[str, str]] = None) -> None:
    """
    Append [filename, start_hex, end_hex] entries to 'images' for each image
    file present under image_dir.  Addresses are derived from the partition's
    reg (offset/size) in the domain-local edt.pickle plus XIP_BASE.

    file_to_label maps each expected filename to its DTS partition label.
    When omitted the caller must pass the correct mapping explicitly.

    Example result for one file:
        [["boot.bin", "0x08000000", "0x08040000"]]
    """
    if file_to_label is None:
        raise ValueError("file_to_label must be provided")

    # Resolve edt.pickle
    edt_path = resolve_edt_pickle_path(image_dir)

    # Deserialize EDT (requires edtlib in the environment)
    with open(edt_path, "rb") as f:
        edt = pickle.load(f)

    # Build label -> (offset, size) for labels we care about
    interested_labels = set(file_to_label.values())
    label_to_reg: Dict[str, Tuple[int, int]] = {}

    for node in getattr(edt, "nodes", []):
        label = _safe_get_label(node)
        if not label or label not in interested_labels:
            continue
        reg_pair = _safe_get_first_reg(node)
        if reg_pair is None:
            continue
        label_to_reg[label] = reg_pair

    # Create entries for files that actually exist under image_dir.
    # Store absolute paths so build_partition_table works regardless of which
    # domain's directory each image lives in.
    for filename, label in file_to_label.items():
        file_path = os.path.abspath(os.path.join(image_dir, filename))
        if not os.path.isfile(file_path):
            continue
        if label not in label_to_reg:
            raise KeyError(
                f"Partition label '{label}' not found in EDT for image '{filename}'."
            )
        offset, size = label_to_reg[label]
        start = XIP_BASE + offset
        end = start + size
        images.append([file_path, f"0x{start:08x}", f"0x{end:08x}"])

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
    floader_dest = FLOADERS_DEST / floader_src.name

    # Keep the existing file unless blobs is newer; skip if up to date.
    try:
        if floader_dest.exists() and floader_dest.stat().st_mtime >= floader_src.stat().st_mtime:
            logger.info(f"Floader: using existing {floader_dest}")
            return str(profile_path)

        fd, tmp_path = tempfile.mkstemp(dir=str(FLOADERS_DEST), prefix=floader_dest.name + ".")
        try:
            with os.fdopen(fd, "wb") as dst, open(floader_src, "rb") as src:
                shutil.copyfileobj(src, dst)
            os.chmod(tmp_path, 0o644)
            st = floader_src.stat()
            os.utime(tmp_path, ns=(st.st_atime_ns, st.st_mtime_ns))
            os.replace(tmp_path, floader_dest)
        except BaseException:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise
        logger.info(f"Floader: copied from blobs {floader_dest}")
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
        # img_name may be an absolute path (from collect_image_load_list) or a
        # relative name supplied by the user via --images (resolved against image_dir).
        p = Path(img_name)
        full_image_path = p.resolve() if p.is_absolute() else (image_dir / img_name).resolve()

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

def detect_multidomain(image_dir: str) -> bool:
    """
    Return True if a 'domains.yaml' file exists two directories above
    'image_dir' and contains a 'domains' list with at least two entries.

    Any exception results in False.
    """
    build_dir = Path(image_dir).resolve().parent.parent
    domains_file = build_dir / "domains.yaml"
    if not domains_file.is_file():
        return False
    try:
        import yaml
        text = domains_file.read_text()
        data = yaml.safe_load(text) or {}
        domains = data.get("domains")
        return isinstance(domains, list) and len(domains) >= 2
    except Exception:
        return False


def _is_last_flash_domain(image_dir: str) -> bool:
    """
    Return True when image_dir belongs to the last domain in domains.yaml's
    flash_order.  This is the one domain that performs the actual combined
    flash; all earlier domain invocations return early without flashing.

    Returns True on any parse error so that flashing still proceeds.
    """
    # image_dir is <domain_build_dir>/images, so parent is the domain build dir.
    # domains.yaml lives one level above that (the top-level build dir).
    current_build_dir = Path(image_dir).resolve().parent
    top_build_dir = current_build_dir.parent
    domains_file = top_build_dir / "domains.yaml"
    try:
        import yaml
        data = yaml.safe_load(domains_file.read_text()) or {}
        flash_order = data.get("flash_order") or []
        if not flash_order:
            return True
        domains = {d["name"]: d["build_dir"] for d in data.get("domains", [])}
        last_build_dir = Path(domains.get(flash_order[-1], "")).resolve()
        return current_build_dir == last_build_dir
    except Exception:
        return True


def collect_all_domain_images(image_dir: str, images: List[List[str]],
                               device: str) -> None:
    """
    In a multidomain build, collect flashable images from EVERY domain's
    image directory and append them to 'images'.

    Each domain's edt.pickle is used to resolve partition addresses for the
    files found in that domain's image directory.  Domains whose EDT does not
    contain the expected label, or whose image directory is missing, are
    skipped with a warning.
    """
    images.clear()
    file_to_label = _get_file_to_label(device, image_dir)

    build_dir = Path(image_dir).resolve().parent.parent
    domains_file = build_dir / "domains.yaml"
    try:
        import yaml
        data = yaml.safe_load(domains_file.read_text()) or {}
    except Exception as e:
        logger.warning(f"Could not parse domains.yaml: {e}; falling back to single-dir mode")
        collect_image_load_list(image_dir, images, file_to_label)
        return

    for domain in data.get("domains", []):
        domain_image_dir = (Path(domain["build_dir"]).resolve() / "images")
        if not domain_image_dir.is_dir():
            continue
        try:
            collect_image_load_list(str(domain_image_dir), images, file_to_label)
        except Exception as e:
            logger.warning(f"Domain '{domain['name']}': skipping images — {e}")

def main(args):
    multidomain = detect_multidomain(args.image_dir)

    # For TFM BL2 single-domain builds, auto-build the partition table from
    # edt.pickle so that tfm_s_signed.bin is flashed at the correct slot.
    if not multidomain and args.images is None:
        file_to_label = _get_file_to_label(args.device, str(args.image_dir))
        if "tfm_s_signed.bin" in file_to_label:
            args.images = []
            try:
                collect_image_load_list(str(args.image_dir), args.images, file_to_label)
            except Exception as e:
                logger.warning(f"TFM partition table auto-build failed: {e}; falling back to simple mode")
                args.images = None

    # In a sysbuild (multidomain) build west flash invokes this runner once per
    # domain.  We skip every domain except the last one in flash_order, then
    # the last domain collects images from ALL domains and flashes once.
    if multidomain:
        if not _is_last_flash_domain(args.image_dir):
            logger.info(
                "Multidomain build: skipping flash for this domain "
                "(will flash from last domain in flash_order)"
            )
            return
        if args.images is None:
            args.images = []
            collect_all_domain_images(args.image_dir, args.images, args.device)

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
