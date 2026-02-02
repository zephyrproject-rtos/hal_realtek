#!/usr/bin/env python3
#
# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

import os
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

socs = ["amebadplus"]

module_yaml = """\
name: hal_realtek
build:
  cmake: .
  kconfig: Kconfig
blobs:"""

lib_item = '''
  - path: ameba/{SOC}/lib/{FILENAME}
    sha256: {SHA256}
    type: lib
    version: '1.0'
    license-path: LICENSE
    url: {URL}/raw/{REV}/{SOC}/lib/{FILENAME}
    description: "Binary libraries supporting the Ameba series RF subsystems"
    doc-url: {URL_BASE}'''

img_item = '''
  - path: ameba/{SOC}/bin/{FILENAME}
    sha256: {SHA256}
    type: img
    version: '1.0'
    license-path: LICENSE
    url: {URL}/raw/{REV}/{SOC}/bin/{FILENAME}
    description: "Binary libraries supporting the Ameba series RF subsystems"
    doc-url: {URL_BASE}'''


def cmd_exec(cmd, cwd=None, shell=False):
    return subprocess.check_call(cmd, cwd=cwd, shell=shell)


def download_repositories(git_rev: str) -> None:
    path = os.path.dirname(os.path.abspath(__file__))

    folder = Path(path, "temp")
    if not folder.exists():
        print("Cloning into {}".format(folder))
        cmd_exec(("git", "clone", git_url, folder, "--quiet"), cwd=path)
    print("Checking out revision {} at {}".format(git_rev, folder))
    cmd_exec(("git", "-C", folder, "fetch"), cwd=path)
    cmd_exec(("git", "-C", folder, "checkout", git_rev, "--quiet"), cwd=path)


def clean_up():
    print("deleted temporary files..")
    path = os.path.dirname(os.path.abspath(__file__))
    folder = Path(path, "temp")
    shutil.rmtree(folder)


def path_leaf(path):
    p = Path(path)
    return p.name if p.name else p.parent.name


def get_file_sha256(path):
    with open(path,"rb") as f:
        f_byte = f.read()
        result = hashlib.sha256(f_byte)
        return result.hexdigest()


def generate_blob_list(output_path: str, git_rev: str) -> None:
    file_out = module_yaml

    path = os.path.dirname(os.path.abspath(__file__))

    for s in socs:
        folder = Path(path, "temp", s)
        pathlist = []
        pathlist.extend(Path(folder).glob('**/*.a'))
        for item in pathlist:
            logger.debug(item)
            path_in_str = str(item)
            filename = path_leaf(path_in_str)
            sha256 = get_file_sha256(path_in_str)
            file_out += lib_item.format(SOC=s,
                                         FILENAME=filename,
                                         SHA256=sha256,
                                         URL=git_url,
                                         REV=git_rev,
                                         URL_BASE=git_url)

        pathlist = []
        pathlist.extend(Path(folder).glob('**/*.bin'))
        for item in pathlist:
            logger.debug(item)
            path_in_str = str(item)
            filename = path_leaf(path_in_str)
            sha256 = get_file_sha256(path_in_str)
            file_out += img_item.format(SOC=s,
                                         FILENAME=filename,
                                         SHA256=sha256,
                                         URL=git_url,
                                         REV=git_rev,
                                         URL_BASE=git_url)

    file_out += "\r\n"

    try:
        os.remove(output_path)
    except OSError:
        pass

    with open(output_path, "w+") as f:
        f.write(file_out)
        f.close()

    print("module.yml updated!")

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
        "-d", "--debug", action="store_true", help="Enable debug logging."
    )

    args: argparse.Namespace = parser.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)

    download_repositories(args.commit)
    generate_blob_list(args.output, args.commit)
    clean_up()

if __name__ == '__main__':
    main()
