import os
import re
import struct

from op_base import OperationBase
from op_prepend_header import PrependHeader
from context import Context
from ameba_enums import *
from utility import *
from ameba_layout_addrs import parse_amebasmart_layout_addrs


class AmebasmartBootAssets(OperationBase):
    """AmebaSmart MCUboot slot0 assembly helpers driven directly from
    mcuboot_app_image.cmake (CONFIG_SOC_SERIES_AMEBASMART branch), replacing the
    amebasmart-specific bits of merge_bin.py::handle_amebasmart_app that have
    no equivalent in the generic axf2bin toolset:
      resolve-addrs - read CA32/KM4-DRAM addresses out of ameba_layout.ld
                      (the .ld is the single source of truth) so cmake can
                      pass them as --address to prepend_header/rsip.
      make-vt       - assemble the 8-byte ARM vector table {MSP, app_start}
                      MCUboot's do_boot() reads, padded to 32B for RSIP MMU
                      alignment (see amebasmart-rsip-mmu-32b-align memory).
    Both take explicit file paths - neither needs Context's manifest/layout
    auto-resolution (require_manifest_file/require_layout_file are False).
    """
    cmd_help_msg = 'AmebaSmart MCUboot slot0 assembly helpers'

    def __init__(self, context: Context) -> None:
        super().__init__(context)

    @staticmethod
    def register_args(parser) -> None:
        subparsers = parser.add_subparsers(dest='sub_operation',
                                           help='Available operations', required=True)

        sub = subparsers.add_parser('resolve-addrs',
                                    help='Resolve CA32/KM4-DRAM addresses from ameba_layout.ld')
        sub.add_argument('--layout-file', help='Path to ameba_layout.ld', required=True)
        sub.add_argument('-o', '--output-file',
                         help='Output file (key=0xValue lines); default stdout', default=None)

        sub = subparsers.add_parser('make-vt',
                                    help="Assemble KM4's ARM VT {MSP, app_start} for MCUboot do_boot")
        sub.add_argument('--km4-blob', help='km4_image2_all.bin path', required=True)
        sub.add_argument('--km4-bd-dram-addr', type=BasedIntParamType(),
                         help='KM4_BD_DRAM ORIGIN (from resolve-addrs km4_bd_dram)', required=True)
        sub.add_argument('--hal-platform-header', help='Path to hal_platform.h', required=True)
        sub.add_argument('--msp-macro', help='C macro name for the initial MSP',
                         default='MSP_RAM_HP')
        sub.add_argument('--pad-len', type=BasedIntParamType(),
                         help='Pad VT to this length (32B RSIP alignment)', default=0x20)
        sub.add_argument('-o', '--output-file', help='Output vt.bin', required=True)

    @staticmethod
    def require_manifest_file(context: Context) -> bool:
        return False

    @staticmethod
    def require_layout_file(context: Context) -> bool:
        return False

    def pre_process(self) -> Error:
        args = self.context.args
        if args.sub_operation == 'resolve-addrs':
            if not os.path.exists(args.layout_file):
                return Error(ErrorType.FILE_NOT_FOUND, args.layout_file)
        elif args.sub_operation == 'make-vt':
            if not os.path.exists(args.km4_blob):
                return Error(ErrorType.FILE_NOT_FOUND, args.km4_blob)
            if not os.path.exists(args.hal_platform_header):
                return Error(ErrorType.FILE_NOT_FOUND, args.hal_platform_header)
        return Error.success()

    def process(self) -> Error:
        args = self.context.args
        if args.sub_operation == 'resolve-addrs':
            return self.resolve_addrs(args.layout_file, args.output_file)
        elif args.sub_operation == 'make-vt':
            return self.make_vt(args.km4_blob, args.km4_bd_dram_addr,
                                args.hal_platform_header, args.msp_macro,
                                args.pad_len, args.output_file)
        return Error(ErrorType.INVALID_INPUT)

    def post_process(self) -> Error:
        return Error.success()

    @exit_on_failure(catch_exception=True)
    def resolve_addrs(self, layout_file: str, output_file) -> Error:
        addrs = parse_amebasmart_layout_addrs(layout_file)
        lines = [f"{k}={hex(v)}\n" for k, v in addrs.items()]
        if output_file:
            with open(output_file, 'w') as f:
                f.writelines(lines)
        else:
            for line in lines:
                print(line, end='')
        return Error.success()

    @exit_on_failure(catch_exception=True)
    def make_vt(self, km4_blob: str, km4_bd_dram_addr: int, hal_platform_header: str,
               msp_macro: str, pad_len: int, output_file: str) -> Error:
        # Extract KM4 DRAM entry: the sub-image with load_addr == km4_bd_dram_addr.
        # Its payload is fwlib's RAM_START_FUNCTION struct (ameba_boot.h): payload+0
        # = RamStartFun (cold-boot entry), payload+4 = RamWakeupFun (sleep-resume,
        # often NULL - do NOT use it). MAGIC = op_prepend_header.img2sign.
        km4_data = open(km4_blob, 'rb').read()
        magic = PrependHeader.img2sign.to_bytes(8, 'big')
        off, dram_entry = 0, None
        while off + 32 <= len(km4_data):
            hdr = km4_data[off:off + 32]
            if hdr[:8] != magic:
                break
            sz        = struct.unpack_from('<I', hdr,  8)[0]
            load_addr = struct.unpack_from('<I', hdr, 12)[0]
            if sz > 0 and load_addr == km4_bd_dram_addr:
                dram_entry = struct.unpack_from('<I', km4_data, off + 32)[0]  # payload+0 = RamStartFun
                break
            off += 32 + sz
        if dram_entry is None:
            self.logger.error(f"KM4 DRAM entry (load_addr=0x{km4_bd_dram_addr:08X}) "
                              f"not found in {km4_blob}")
            return Error(ErrorType.INVALID_INPUT)

        # SP = MSP_RAM_HP, a ROM-fixed constant read from hal_platform.h.
        header_text = open(hal_platform_header, 'r').read()
        m = re.search(rf'^\s*#define\s+{msp_macro}\s+(0x[0-9A-Fa-f]+)', header_text, re.MULTILINE)
        if not m:
            self.logger.error(f"{msp_macro} not found in {hal_platform_header}")
            return Error(ErrorType.INVALID_INPUT)
        msp = int(m.group(1), 16)

        # Pad the {SP, Reset} vector so the next chain in slot0 starts on a
        # 32-byte boundary: boot_prepare.c maps it through the RSIP MMU, whose
        # physical offset is rounded UP to 32 B (ameba_otf_rom.c
        # RSIP_MMU_Config) - an unpadded 8-byte VT would corrupt every
        # RSIP-window LogAddr read downstream by the shortfall.
        vt = struct.pack('<II', msp, dram_entry).ljust(pad_len, b'\x00')
        with open(output_file, 'wb') as f:
            f.write(vt)
        self.logger.info(f"VT assembled: msp=0x{msp:08X} reset=0x{dram_entry:08X} "
                         f"-> {output_file} ({len(vt)}B)")
        return Error.success()
