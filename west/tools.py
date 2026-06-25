#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import json
import logging
import shutil
import subprocess
from pathlib import Path
import argparse
import re

from west.commands import WestCommand, Verbosity

_logger = logging.getLogger(__name__)


class _WestLogHandler(logging.Handler):
    # Routes Python logging records to WestCommand output methods so that
    # _logger calls respect west's -v/-q flags and colorized output.
    # Pattern adapted from zephyr/scripts/west_commands/build_helpers.py;
    # see https://github.com/zephyrproject-rtos/west/issues/952
    def __init__(self, command, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._command = command

    def emit(self, record):
        fmt = self.format(record)
        lvl = record.levelno
        if lvl >= logging.ERROR:
            self._command.err(fmt)
        elif lvl >= logging.WARNING:
            self._command.wrn(fmt)
        elif lvl >= logging.INFO:
            self._command.inf(fmt)
        else:
            self._command.dbg(fmt, level=Verbosity.DBG_EXTREME)


def _forward_logging_to_west(command, logger_name):
    logger = logging.getLogger(logger_name)
    logger.setLevel(1 if command.verbosity >= Verbosity.DBG else logging.INFO)
    logger.addHandler(_WestLogHandler(command))

TOOLCHAIN_DB_FILE = "../ameba/scripts/toolchain_db.json"

NUWA_SDK_TOOLCHAIN_DEFAULT_PATH_WINDOWS = r"C:\rtk-toolchain"
NUWA_SDK_TOOLCHAIN_DEFAULT_PATH_LINUX = str(Path.home() / "rtk-toolchain")


def default_toolchain_dir() -> Path:
    return Path(NUWA_SDK_TOOLCHAIN_DEFAULT_PATH_WINDOWS if os.name == "nt"
                else NUWA_SDK_TOOLCHAIN_DEFAULT_PATH_LINUX)


def ensure_dir(path: Path):
    if not path.exists():
        try:
            path.mkdir(parents=True, exist_ok=True)
            _logger.info("Created directory: %s", path)
        except PermissionError:
            sys.exit(f"Failed to create directory: {path}. Permission denied.")


def run_cmd(cmd, cwd=None, quiet=False):
    """
    Run external command.
    - When quiet=True, suppress stdout/stderr by redirecting to DEVNULL.
    - Returns: (rc, "", err), where rc is the process return code.
    """
    try:
        if not quiet:
            _logger.info("[CMD] %s", ' '.join(cmd) if isinstance(cmd, (list, tuple)) else cmd)
        proc = subprocess.Popen(
            cmd,
            shell=True,
            cwd=str(cwd) if cwd is not None else None,
            stdout=subprocess.DEVNULL if quiet else None,
            stderr=subprocess.DEVNULL if quiet else None,
        )
        rc = proc.wait()
        return rc, "", ""
    except Exception as e:
        return -1, "", str(e)


def validate_archive(archive: Path) -> bool:
    """
    Validate archive integrity using platform-specific tools:
      - Windows: 7z t <archive>
      - Linux: tar -tjf <archive>
    Returns True if archive appears valid, False otherwise.
    """
    if not archive.exists():
        return False

    if os.name == "nt":
        ret, _, _ = run_cmd(f'7z t "{archive}"', quiet=True)
        return ret == 0
    else:
        ret, _, _ = run_cmd(f'tar -tjf "{archive}"', quiet=True)
        return ret == 0


def join_url(base: str, name: str) -> str:
    return base.rstrip("/") + "/" + name.lstrip("/")


def download_from_urls(urls, dest: Path):
    dest.parent.mkdir(parents=True, exist_ok=True)
    for idx, url in enumerate(urls, start=1):
        _logger.info("Download %s from: %s ...", dest.name, url)
        ret, _, _ = run_cmd(f'wget -c --progress=bar:force -O "{dest}" "{url}"')
        if ret == 0:
            _logger.info("Download %s success from: %s", dest.name, url)
            return
        else:
            _logger.warning("Download failed from: %s", url)
    sys.exit("Download failed from all mirrors. Please check network and try again.")


def extract_archive(archive: Path, dest_dir: Path, major: str, minor: str, newly_downloaded: bool = False):
    """
    Extract archive into dest_dir and relocate:
      {dest_dir}/{major} -> {dest_dir}/{major}-{minor}
    """
    dest_dir.mkdir(parents=True, exist_ok=True)

    src_dir = dest_dir / major
    dst_dir = dest_dir / f"{major}-{minor}"

    if dst_dir.exists():
        _logger.info("Toolchain already extracted: %s", dst_dir)
        return

    if src_dir.exists():
        _logger.info("Relocating extracted layout: %s -> %s", src_dir, dst_dir)
        if dst_dir.exists():
            shutil.rmtree(dst_dir)
        try:
            shutil.move(str(src_dir), str(dst_dir))
        except Exception:
            shutil.copytree(src_dir, dst_dir, dirs_exist_ok=True)
            shutil.rmtree(src_dir)
        return

    if newly_downloaded:
        _logger.info("[Check] Validate newly downloaded archive: %s", archive.name)
        if not validate_archive(archive):
            sys.exit(f"Archive appears corrupted or incomplete: {archive}")

    if newly_downloaded:
        _logger.info("Validation passed. Start extracting: %s ...", archive.name)
    else:
        _logger.info("Start extracting: %s ...", archive.name)

    if os.name == "nt":
        ret, _, _ = run_cmd(f'7z x "{archive}" -o"{dest_dir}"')
    else:
        ret, _, _ = run_cmd(f'tar -jxf "{archive}" -C "{dest_dir}"')

    if ret != 0:
        if src_dir.exists():
            shutil.rmtree(src_dir)
        sys.exit(f"Unzip failed. Please unzip {archive} manually.")

    _logger.info("Extract completed: %s", archive.name)

    if src_dir.exists():
        if dst_dir.exists():
            shutil.rmtree(dst_dir)
        try:
            shutil.move(str(src_dir), str(dst_dir))
        except Exception:
            shutil.copytree(src_dir, dst_dir, dirs_exist_ok=True)
            shutil.rmtree(src_dir)
        _logger.info("Relocated: %s -> %s", src_dir, dst_dir)
    else:
        _logger.warning("Source directory not found after extraction: %s", src_dir)


def locate_db_default(script_file: Path) -> Path:
    return script_file.parent / TOOLCHAIN_DB_FILE


_ID_RE = re.compile(r'^(.+)-([0-9]+)$')

def parse_toolchain_id(tid: str) -> tuple[str, str]:
    """
    Parse toolchain id key '<major>-<minor>' where minor is numeric.
    Returns (major, minor_str).
    """
    m = _ID_RE.match(tid or "")
    if not m:
        sys.exit(f"Invalid toolchain id '{tid}'; expected '<major>-<minor>' with numeric minor.")
    return m.group(1), m.group(2)


def load_toolchain_db(db_path: Path) -> dict:
    if not db_path.exists():
        sys.exit(f"Toolchain DB not found: {db_path}")
    try:
        with db_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        sys.exit(f"Failed to read toolchain DB: {db_path}: {e}")
    if not isinstance(data, dict) or not data:
        sys.exit(f"Invalid toolchain DB structure: {db_path}")

    # New schema: require url_base, suffix, chips, devices; aliyun_url_base is optional.
    required_keys = {"url_base", "suffix", "chips", "devices"}
    for tid, entry in data.items():
        if not isinstance(entry, dict):
            sys.exit(f"Invalid entry for {tid}: expected object")
        # validate id format '<major>-<minor>'
        parse_toolchain_id(tid)
        missing = required_keys - set(entry.keys())
        if missing:
            sys.exit(f"Missing keys for {tid}: {', '.join(sorted(missing))}")
        # basic type checks
        if not isinstance(entry.get("url_base"), str) or not entry.get("url_base"):
            sys.exit(f"Invalid url_base for {tid}: expected non-empty string")
        if entry.get("aliyun_url_base") is not None and not isinstance(entry.get("aliyun_url_base"), str):
            sys.exit(f"Invalid aliyun_url_base for {tid}: expected string or omit")
        if not isinstance(entry.get("suffix"), str) or not entry.get("suffix"):
            sys.exit(f"Invalid suffix for {tid}: expected non-empty string")
        if not isinstance(entry.get("chips"), list) or not entry.get("chips"):
            sys.exit(f"Invalid chips for {tid}: expected non-empty list")
        if not isinstance(entry.get("devices"), list) or not entry.get("devices"):
            sys.exit(f"Invalid devices for {tid}: expected non-empty list")
    return data


def derive_toolchain_info_by_id(toolchain_id: str, db: dict, tdir: Path) -> dict:
    if toolchain_id not in db:
        valid = ", ".join(sorted(db.keys()))
        sys.exit(f"Unsupported toolchain '{toolchain_id}'. Valid values: {valid}")

    c = db[toolchain_id]

    major, minor = parse_toolchain_id(toolchain_id)
    suffix = c["suffix"]
    url_base = c["url_base"]
    aliyun_base = c.get("aliyun_url_base")

    version_str = f"{major}-{minor}"

    if os.name == "nt":
        path = tdir / version_str / "mingw32" / "newlib"
        archive_name = f"{major}-mingw32-newlib-build-{minor}{suffix}.zip"
    else:
        path = tdir / version_str / "linux" / "newlib"
        archive_name = f"{major}-linux-newlib-build-{minor}{suffix}.tar.bz2"

    url_default = join_url(url_base, archive_name)
    url_aliyun = join_url(aliyun_base, archive_name) if aliyun_base else None

    return {
        "major": major,
        "minor": minor,
        "version": version_str,
        "path": path,
        "archive_name": archive_name,
        "url_default": url_default,
        "url_aliyun": url_aliyun,
    }


def install_toolchain_by_id(toolchain_id: str, toolchain_dir: Path, db: dict, use_aliyun: bool = False) -> Path:
    ensure_dir(toolchain_dir)

    info = derive_toolchain_info_by_id(toolchain_id, db, toolchain_dir)

    t_path: Path = info["path"]
    archive_name: str = info["archive_name"]
    url_default: str = info["url_default"]
    url_aliyun: str | None = info["url_aliyun"]
    major: str = info["major"]
    minor: str = info["minor"]

    if t_path.exists():
        _logger.info("[%s] Toolchain already installed: %s", toolchain_id, t_path)
        return t_path

    _logger.warning("[%s] Toolchain path does not exist: %s", toolchain_id, t_path)

    archive = toolchain_dir / archive_name
    newly_downloaded = False

    if archive.exists():
        _logger.info("[%s] [Check] Archive exists but extracted path missing; validating archive usability: %s",
                     toolchain_id, archive.name)
        if not validate_archive(archive):
            _logger.warning("[%s] Existing archive appears corrupted; will re-download: %s", toolchain_id, archive)
            try:
                archive.unlink()
            except Exception as e:
                _logger.warning("[%s] Failed to remove corrupted archive: %s", toolchain_id, e)

    if not archive.exists():
        urls = []
        if use_aliyun and url_aliyun:
            urls.append(url_aliyun)        # prefer Aliyun first
            urls.append(url_default)       # fallback to default
        else:
            urls.append(url_default)
        download_from_urls(urls, archive)
        newly_downloaded = True

    extract_archive(archive, toolchain_dir, major, minor, newly_downloaded=newly_downloaded)
    _logger.info("[%s] Extraction complete", toolchain_id)

    if not t_path.exists():
        sys.exit(f"[{toolchain_id}] Toolchain install failed; expected path missing: {t_path}")

    _logger.info("[%s] Toolchain install success", toolchain_id)
    return t_path


def install_all_toolchains(toolchain_dir: Path, db: dict, use_aliyun: bool = False):
    if not db:
        sys.exit("No toolchains defined in toolchain DB")

    results = {}
    for tid in sorted(db.keys()):
        try:
            p = install_toolchain_by_id(tid, toolchain_dir, db, use_aliyun=use_aliyun)
            results[tid] = {"status": "OK", "path": str(p)}
        except SystemExit as e:
            results[tid] = {"status": "FAILED", "reason": str(e)}
        except Exception as e:
            results[tid] = {"status": "FAILED", "reason": repr(e)}

    _logger.info("=== Install summary ===")
    for tid, res in results.items():
        if res["status"] == "OK":
            _logger.info("  %s: OK -> %s", tid, res["path"])
        else:
            _logger.warning("  %s: FAILED -> %s", tid, res.get("reason"))


def build_help_mapping_text(db: dict) -> str:
    lines = []
    lines.append("Supported toolchains (use -t <toolchain_id>):")
    for tid in sorted(db.keys()):
        c = db[tid]
        chips = ", ".join(c.get("chips", []))
        devices = ", ".join(c.get("devices", []))
        lines.append(f"  - {tid} (chips: {chips})")
        lines.append(f"      supported devices: {devices}")
    lines.append("")
    lines.append("Usage tips:")
    lines.append("  - Install ALL toolchains: west realtek ameba install")
    lines.append("  - Install ONE toolchain: west realtek ameba install -t <toolchain_id>")
    lines.append("  - Use Aliyun mirror: add --aliyun")
    lines.append("    e.g., west realtek ameba install -t asdk-12.3.1-4568 --aliyun")
    return "\n".join(lines)


class Tools(WestCommand):
    def __init__(self):
        super().__init__(
            "realtek",
            "Realtek toolchain management",
            dedent_description(),
        )

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(
            self.name,
            help=self.help,
            description=self.description,
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )

        # add series layer
        series = parser.add_subparsers(dest="series", required=True, help="series")

        p_ameba = series.add_parser(
            "ameba",
            help="Ameba series",
            description="Ameba series toolchains",
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )
        subs = p_ameba.add_subparsers(dest="subcmd", required=True, help="subcommands")

        db_path = locate_db_default(Path(__file__).resolve())
        try:
            db_for_help = load_toolchain_db(db_path)
            mapping_text = build_help_mapping_text(db_for_help)
            toolchain_choices = sorted(db_for_help.keys())
        except SystemExit:
            mapping_text = "Toolchain DB is missing or invalid. Place toolchain_db.json next to this script."
            toolchain_choices = None
        except Exception as e:
            mapping_text = f"Failed to load toolchain DB for help: {e}"
            toolchain_choices = None

        p_install = subs.add_parser(
            "install",
            help="Install all toolchains by default; use -t to install a single toolchain",
            description=(
                "Install Realtek GNU Arm Embedded toolchains.\n"
                "- Default: install ALL toolchains.\n"
                "- With -t/--toolchain: install ONE specified toolchain (by version id).\n\n"
                + mapping_text
            ),
            formatter_class=argparse.RawDescriptionHelpFormatter,
        )
        # make "options" section title explicit
        try:
            p_install._optionals.title = "options"
        except Exception:
            pass

        p_install.add_argument(
            "-t", "--toolchain",
            choices=toolchain_choices,
            help="Toolchain id (version) to install. If omitted, installs ALL toolchains."
        )
        p_install.add_argument(
            "--toolchain-dir",
            help="Toolchain base directory (default depends on OS)"
        )
        p_install.add_argument(
            "--aliyun",
            action="store_true",
            help="Use Aliyun mirror to download toolchains (falls back to default on failure)"
        )

        # examples for top-level parser (shown in `west realtek -h`)
        parser.epilog = (
            "Examples:\n"
            "  west realtek ameba install\n"
            "  west realtek ameba install -t asdk-12.3.1-4568\n"
            "  west realtek ameba install -t asdk-10.3.1-4523 --toolchain-dir ~/rtk-toolchain\n"
            "  west realtek ameba install -t asdk-12.3.1-4568 --aliyun\n"
        )

        return parser

    def do_run(self, args, unknown_args):
        _forward_logging_to_west(self, __name__)
        logging.getLogger(__name__).propagate = False
        if getattr(args, "series", None) == "ameba":
            if args.subcmd == "install":
                toolchain_dir = Path(args.toolchain_dir) if args.toolchain_dir else default_toolchain_dir()

                db_path = locate_db_default(Path(__file__).resolve())
                db = load_toolchain_db(db_path)

                if args.toolchain:
                    tid = args.toolchain
                    self.inf(f"Installing toolchain: {tid}")
                    install_toolchain_by_id(tid, toolchain_dir, db, use_aliyun=args.aliyun)
                else:
                    self.inf("Installing ALL toolchains defined in toolchain DB ...")
                    install_all_toolchains(toolchain_dir, db, use_aliyun=args.aliyun)
            else:
                self.die(f"Unknown subcommand: {args.subcmd}")
        else:
            self.die(f"Unknown series: {getattr(args, 'series', None)}")


def dedent_description() -> str:
    return (
        "Manage Realtek GNU Arm Embedded toolchains for Zephyr builds.\n\n"
        "Series:\n"
        "  ameba   - Ameba series toolchain management\n\n"
        "Subcommands:\n"
        "  install  - Install toolchains (ALL by default, or ONE with -t).\n\n"
        "Notes:\n"
        "  - Toolchain metadata is loaded from toolchain_db.json (co-located with this script).\n"
        "  - On Windows, this script requires 7-Zip (7z) available in PATH;\n"
        "    on Linux, it requires wget and tar (with bzip2) available in PATH.\n"
        "  - Environment variable auto-configuration is intentionally removed.\n"
    )
