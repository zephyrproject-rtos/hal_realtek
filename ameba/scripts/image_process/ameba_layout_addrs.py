"""Shared amebasmart ameba_layout.ld address resolution.

Used by both merge_bin.py::handle_amebasmart (non-mcuboot image assembly) and
op_amebasmart_boot_assets.py's `resolve-addrs` subcommand (mcuboot slot0
assembly, driven from mcuboot_app_image.cmake). ameba_layout.ld is the single
source of truth for these addresses - no literals anywhere else.
"""

import re
import sys
import logging
from pathlib import Path

logger = logging.getLogger("imagetool")


def parse_amebasmart_layout_addrs(layout_ld):
    """Resolve CA32 sub-image load addresses from ameba_layout.ld (the .ld is
    the single source of truth; no literals here).

      xip/bl1_sram/bl1_dram/fip = the .ld's __ca32_*_start__ symbols.
        bl1_dram/fip resolve to the secure DRAM alias (writable on rtl8730e_evb,
        hardware-verified).
      km4_bd_dram = ORIGIN(KM4_BD_DRAM), sentinel to find the KM4 DRAM
        sub-image inside the km4 blob.
    """
    text = Path(layout_ld).read_text()

    def origin(region):
        m = re.search(rf'^\s*{region}\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)',
                      text, re.MULTILINE)
        if not m:
            logger.error(f"ORIGIN for {region} not found in {layout_ld}")
            sys.exit(1)
        return int(m.group(1), 16)

    m = re.search(r'#define\s+SECURE_ADDR_OFFSET\s+\(?(0x[0-9A-Fa-f]+)\)?', text)
    if not m:
        logger.error(f"SECURE_ADDR_OFFSET not found in {layout_ld}")
        sys.exit(1)
    secure_off = int(m.group(1), 16)

    def resolve_symbol(sym):
        """Evaluate '<sym> = ORIGIN(REGION) [+/- SECURE_ADDR_OFFSET];'."""
        d = re.search(rf'^\s*{re.escape(sym)}\s*=\s*([^;]+);', text, re.MULTILINE)
        if not d:
            logger.error(f"symbol {sym} not found in {layout_ld}")
            sys.exit(1)
        expr = re.sub(r'ORIGIN\s*\(\s*(\w+)\s*\)',
                      lambda g: hex(origin(g.group(1))), d.group(1))
        expr = re.sub(r'\bSECURE_ADDR_OFFSET\b', hex(secure_off), expr)
        try:
            return int(eval(expr, {"__builtins__": {}}, {}))
        except Exception as e:
            logger.error(f"cannot evaluate {sym} ('{d.group(1).strip()}'): {e}")
            sys.exit(1)

    return {
        'xip':          resolve_symbol('__ca32_flash_text_start__'),
        'bl1_sram':     resolve_symbol('__ca32_bl1_sram_start__'),
        'bl1_dram':     resolve_symbol('__ca32_bl1_dram_start__'),
        'fip':          resolve_symbol('__ca32_fip_dram_start__'),
        'km4_bd_dram':  origin('KM4_BD_DRAM'),
        # KM0 runs from SRAM (XIP=n): its image2 loads to KM0_BD_RAM ORIGIN
        # (0x23002020), NOT the KM0 XIP flash window.
        'km0_bd_ram':   origin('KM0_BD_RAM'),
        # KM4 SRAM-TCM ORIGIN (0x20014020) and KM0 DRAM ORIGIN (0x6fffffff
        # sentinel). Both are used as *empty stub* sub-image load addresses so
        # each per-core image2 mirrors the vendor 3-slot positional layout
        # (XIP / SRAM / DRAM) that IMG1 walks by position.
        'km4_bd_ram':   origin('KM4_BD_RAM'),
        'km0_bd_dram':  origin('KM0_BD_DRAM'),
        # Per-core flash-XIP window ORIGINs; each RSIP'd at its own address.
        'km0_xip':      origin('KM0_IMG2_XIP'),
        'km4_xip':      origin('KM4_IMG2_XIP'),
    }
