#!/usr/bin/env python3
#
# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

import os
import re
import subprocess
import shutil
import argparse
import logging
import hashlib
from pathlib import Path

# p = Path('/project/hal/realtek/ameba/scripts/update_blobs.py')
# p.parents[0]  # => /project/hal/realtek/ameba/scripts
# p.parents[1]  # => /project/hal/realtek/ameba
MODULE_PATH = Path(Path(__file__).resolve().parents[2], "zephyr", "module.yml")
git_url = f"https://github.com/Ameba-AIoT/nuwa_lib"

logger: logging.Logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

# Default SOC list - all supported SOCs (in folder order: amebad, amebadplus, amebag2)
DEFAULT_SOCS = ["amebad", "amebadplus", "amebag2"]

lib_item = '''
  - path: ameba/{SOC}/lib/{RELPATH}
    sha256: {SHA256}
    type: lib
    version: '1.0'
    license-path: LICENSE
    url: {URL}/raw/{REV}/{SOC}/lib/{RELPATH}
    description: "{DESCRIPTION}"
    doc-url: {URL_BASE}
'''

img_item = '''
  - path: ameba/{SOC}/bin/{RELPATH}
    sha256: {SHA256}
    type: img
    version: '1.0'
    license-path: LICENSE
    url: {URL}/raw/{REV}/{SOC}/bin/{RELPATH}
    description: "{DESCRIPTION}"
    doc-url: {URL_BASE}
'''


def cmd_exec(cmd, cwd=None, shell=False):
    """Execute a shell command."""
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)


def download_repositories(git_rev: str) -> None:
    """Download and checkout the specified revision of the repository."""
    path = os.path.dirname(os.path.abspath(__file__))

    folder = Path(path, "temp")
    if not folder.exists():
        print("Cloning into {}".format(folder))
        cmd_exec(("git", "clone", git_url, folder, "--quiet"), cwd=path)
    print("Checking out revision {} at {}".format(git_rev, folder))
    cmd_exec(("git", "-C", folder, "fetch"), cwd=path)
    cmd_exec(("git", "-C", folder, "checkout", git_rev, "--quiet"), cwd=path)


def clean_up():
    """Remove temporary files."""
    print("Deleted temporary files..")
    path = os.path.dirname(os.path.abspath(__file__))
    folder = Path(path, "temp")
    shutil.rmtree(folder)


def path_leaf(path):
    """Extract the filename from a path."""
    p = Path(path)
    return p.name if p.name else p.parent.name


def get_file_sha256(path):
    """Calculate SHA256 hash of a file."""
    with open(path,"rb") as f:
        f_byte = f.read()
        result = hashlib.sha256(f_byte)
        return result.hexdigest()


def get_relative_path(file_path: Path, base_folder: Path) -> str:
    """Get relative path from base folder.

    Args:
        file_path: Full path to the file
        base_folder: Base folder (e.g., lib/ or bin/)

    Returns:
        Relative path string (e.g., 'coex/lib_wifi.a' or 'lib_wifi.a')
    """
    try:
        return str(file_path.relative_to(base_folder))
    except ValueError:
        return file_path.name


def get_description(filename: str) -> str:
    """Generate appropriate description based on filename.

    Args:
        filename: The name of the binary file

    Returns:
        Description string for the binary
    """
    filename_lower = filename.lower()

    # Check for Wi-Fi related files
    if 'wifi' in filename_lower or 'wlan' in filename_lower:
        return "Binary libraries supporting the Ameba series Wi-Fi subsystems"

    # Check for Bluetooth related files
    if 'bt' in filename_lower or 'bluetooth' in filename_lower or 'ble' in filename_lower:
        return "Binary libraries supporting the Ameba series Bluetooth subsystems"

    # Check for RF related files
    if 'rf' in filename_lower or 'radio' in filename_lower:
        return "Binary libraries supporting the Ameba series RF subsystems"

    # Check for WPA related files
    if 'wpa' in filename_lower or 'wps' in filename_lower:
        return "Binary libraries supporting the Ameba series Wi-Fi subsystems"

    # Check for coex related files
    if 'coex' in filename_lower:
        return "Binary libraries supporting the Ameba series Wi-Fi subsystems"

    # Check for rtw related files
    if 'rtw' in filename_lower:
        return "Binary libraries supporting the Ameba series Wi-Fi subsystems"

    # Check for USB related files
    if 'usb' in filename_lower:
        return "Binary libraries supporting the Ameba series USB subsystems"

    # Check for bootloader related files
    if 'boot' in filename_lower or 'loader' in filename_lower:
        return "Binary images for the Ameba series bootloader"

    # Check for chipinfo related files
    if 'chip' in filename_lower:
        return "Binary libraries supporting the Ameba series chip information"

    # Check for PMC related files
    if 'pmc' in filename_lower:
        return "Binary libraries supporting the Ameba series power management"

    # Default description
    return "Binary libraries supporting the Ameba series subsystems"


def sort_socs(socs: list) -> list:
    """Sort SOC list based on predefined order (DEFAULT_SOCS)."""
    return sorted(socs, key=lambda x: DEFAULT_SOCS.index(x) if x in DEFAULT_SOCS else 999)


def read_existing_module_yml(path: str) -> tuple:
    """Read existing module.yml and extract header and bee blobs.

    Returns:
        tuple: (header_content, bee_blobs_content)
    """
    with open(path, 'r') as f:
        content = f.read()

    # Split into header (name, package-managers, build) and blobs section
    # Find "blobs:" line and keep everything up to and including it
    lines = content.split('\n')
    header_lines = []
    blobs_started = False
    for line in lines:
        header_lines.append(line)
        if line.strip() == 'blobs:':
            blobs_started = True
            break

    header = '\n'.join(header_lines)

    # Extract bee bins section (capture from comment line, preserving blank lines before it)
    bee_match = re.search(r'(\n  # ===== bee bins =====.*)', content, re.DOTALL)
    bee_blobs = bee_match.group(1) if bee_match else ""

    return header, bee_blobs


def generate_blob_list(output_path: str, git_rev: str, socs: list) -> None:
    """Generate blob list for specified SOCs in module.yml format."""
    # Read existing module.yml to preserve header and bee blobs
    header_content, bee_blobs_content = read_existing_module_yml(output_path)

    file_out = header_content + "\n"

    path = os.path.dirname(os.path.abspath(__file__))

    # Sort SOC list according to predefined order
    sorted_socs = sort_socs(socs)

    print("Processing SOCs in order: {}".format(', '.join(sorted_socs)))

    for s in sorted_socs:
        folder = Path(path, "temp", s)

        if not folder.exists():
            logger.warning("Folder not found: {}, skipping SOC: {}".format(folder, s))
            continue

        print("\nProcessing SOC: {}".format(s))

        # Process binary files (.bin) first
        print("  Processing binaries (.bin) for {}...".format(s))

        # Add separator comment for bins (with preceding blank line)
        file_out += "\n  # ===== {} bins =====\n".format(s)

        # Search recursively in bin/ directory
        bin_folder = Path(folder, "bin")
        pathlist = []
        if bin_folder.exists():
            pathlist.extend(bin_folder.glob('**/*.bin'))
        else:
            logger.warning("Binary folder not found: {}".format(bin_folder))

        # Remove duplicates by relative path (keep files in different subdirs)
        seen_paths = set()
        unique_pathlist = []
        for item in sorted(pathlist, key=lambda x: str(x)):
            rel_path = get_relative_path(item, bin_folder)

            if rel_path not in seen_paths:
                seen_paths.add(rel_path)
                unique_pathlist.append(item)
                logger.debug("Added: {} -> {}".format(rel_path, item))
            else:
                # This should rarely happen (exact duplicate path)
                logger.warning("Skipping exact duplicate path: {}".format(item))

        for item in unique_pathlist:
            logger.debug(item)
            path_in_str = str(item)
            rel_path = get_relative_path(item, bin_folder)
            filename = path_leaf(path_in_str)
            sha256 = get_file_sha256(path_in_str)
            description = get_description(filename)
            file_out += img_item.format(SOC=s,
                                         RELPATH=rel_path,
                                         SHA256=sha256,
                                         URL=git_url,
                                         REV=git_rev,
                                         URL_BASE=git_url,
                                         DESCRIPTION=description)

        # Process library files (.a) second
        print("  Processing libraries (.a) for {}...".format(s))

        # Add separator comment for libs (with preceding blank line)
        file_out += "\n  # ===== {} libs =====\n".format(s)

        # Search recursively in lib/ directory (to include coex subfolder)
        lib_folder = Path(folder, "lib")
        pathlist = []
        if lib_folder.exists():
            pathlist.extend(lib_folder.glob('**/*.a'))
        else:
            logger.warning("Library folder not found: {}".format(lib_folder))

        # Remove duplicates by relative path (keep files in different subdirs)
        seen_paths = set()
        unique_pathlist = []
        for item in sorted(pathlist, key=lambda x: str(x)):
            rel_path = get_relative_path(item, lib_folder)

            if rel_path not in seen_paths:
                seen_paths.add(rel_path)
                unique_pathlist.append(item)
                logger.debug("Added: {} -> {}".format(rel_path, item))
            else:
                # This should rarely happen (exact duplicate path)
                logger.warning("Skipping exact duplicate path: {}".format(item))

        for item in unique_pathlist:
            logger.debug(item)
            path_in_str = str(item)
            rel_path = get_relative_path(item, lib_folder)
            filename = path_leaf(path_in_str)
            sha256 = get_file_sha256(path_in_str)
            description = get_description(filename)
            file_out += lib_item.format(SOC=s,
                                         RELPATH=rel_path,
                                         SHA256=sha256,
                                         URL=git_url,
                                         REV=git_rev,
                                         URL_BASE=git_url,
                                         DESCRIPTION=description)

    # Add bee blobs section if exists (preserve original indentation)
    if bee_blobs_content:
        file_out += bee_blobs_content

    with open(output_path, "w") as f:
        f.write(file_out)

    print("\nmodule.yml updated successfully!")


def main() -> None:
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Generate a module.yml file for the Zephyr project."
    )
    parser.add_argument(
        "-o",
        "--output",
        default=MODULE_PATH,
        help="Path to the output YAML file.",
    )
    parser.add_argument(
        "-c",
        "--commit",
        required=True,
        help="The latest commit SHA for the nuwa_lib repository.",
    )
    parser.add_argument(
        "-s",
        "--socs",
        default=",".join(DEFAULT_SOCS),
        help="Comma-separated list of SOCs to process (e.g., 'amebad,amebadplus,amebag2'). Default: all SOCs ({})".format(','.join(DEFAULT_SOCS)),
    )
    parser.add_argument(
        "-d", "--debug", action="store_true", help="Enable debug logging."
    )

    args: argparse.Namespace = parser.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)

    # Parse SOC list from comma-separated string
    socs = [s.strip() for s in args.socs.split(",") if s.strip()]

    if not socs:
        logger.error("No valid SOCs specified!")
        return

    print("Target SOCs: {}".format(', '.join(socs)))

    download_repositories(args.commit)
    generate_blob_list(args.output, args.commit, socs)
    clean_up()

if __name__ == '__main__':
    main()
