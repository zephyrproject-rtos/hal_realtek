#!/usr/bin/env python3

# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0

import argparse,json,os,platform,re,sys
from pathlib import Path

R_ACT=re.compile(r'^(.+)-([0-9]+)$')
R_CORE=re.compile(r'([0-9]+(?:\.[0-9]+)*)')

def err(msg,code=2):
    print(msg,file=sys.stderr);sys.exit(code)

def split_major(s):
    m=R_CORE.search(s or '')
    if not s or not m: err(f"cannot extract numeric core from '{s}'")
    base=s[:m.start()].rstrip('-_').lower()
    if not base: err(f"cannot extract base from '{s}'")
    return base,m.group(1)

def vtuple(v,w=3):
    p=[int(x) for x in v.split('.')]
    if len(p)<w: p+=[0]*(w-len(p))
    return tuple(p[:w])

def plat():
    return "mingw32/newlib" if platform.system()=="Windows" else "linux/newlib"

def guide(req_major,req_minor,rtkdir):
    base=rtkdir.strip() if rtkdir else ("C:/rtk-toolchain" if platform.system()=="Windows" else str(Path.home()/"rtk-toolchain"))
    sdk=str(Path(base)/f"{req_major}-{req_minor}"/plat()).replace("\\","/")
    base_n=base.replace("\\","/")
    rid=f"{req_major}-{req_minor}"
    return (
        "Environment setup guidance:\n\n"
        "Then set environment variables (in this order):\n"
        f"  - ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb\n"
        f"  - GNUARMEMB_TOOLCHAIN_PATH={sdk}\n"
        f"  - RTK_TOOLCHAIN_DIR={base_n}\n\n"
        "Note: If the toolchain is installed in a non-default directory, set RTK_TOOLCHAIN_DIR to the installation directory.\n\n"
        "Recommended west commands:\n"
        f"  - west realtek ameba install -t {rid}\n"
        f"  - west realtek ameba install -t {rid} --aliyun\n"
        f"  - west realtek ameba install -t {rid} --toolchain-dir {base_n}"
    )

def parse_act(path):
    id=Path(path or '').resolve().parent.parent.name
    m=R_ACT.match(id)
    if not m:
        print(f"Cannot parse actual toolchain id from '{id}'. Expected '<major>-<minor>' where minor is numeric.")
        return id,int(0)
    else:
        return m.group(1),int(m.group(2))

def pick_req(db,chip):
    k=(chip or '').strip().lower()
    cands=[]
    for tid,ent in (db or {}).items():
        chips=ent.get('chips') or []
        if any(str(x).strip().lower()==k for x in chips):
            m=R_ACT.match(tid)
            if not m:
                err(f"Invalid toolchain id key in JSON: '{tid}'. Expected '<major>-<minor>' where minor is numeric.",1)
            major_id=m.group(1)
            minor_num=int(m.group(2))
            core=split_major(major_id)[1]
            cands.append((major_id,minor_num,vtuple(core)))
    if not cands: err(f"No matching required toolchain found in JSON for chip='{chip}'",1)
    cands.sort(key=lambda x:(x[2],x[1]),reverse=True)
    major_id,minor_num,_=cands[0]
    return major_id,int(minor_num)

def main():
    ap=argparse.ArgumentParser(add_help=True)
    ap.add_argument('--chip',required=True)
    a=ap.parse_args()

    script_dir=Path(__file__).resolve().parent
    db_path=script_dir/'toolchain_db.json'
    try:
        with db_path.open('r',encoding='utf-8') as f: db=json.load(f)
    except FileNotFoundError:
        err(f"toolchain_db.json not found at: {db_path}",1)
    except Exception as ex:
        err(f"Failed to parse JSON at {db_path}: {ex}",1)

    req_major,req_minor=pick_req(db,a.chip)
    tcv=os.environ.get('ZEPHYR_TOOLCHAIN_VARIANT','')
    tcp=os.environ.get('GNUARMEMB_TOOLCHAIN_PATH','')
    rtk=os.environ.get('RTK_TOOLCHAIN_DIR','')
    g=guide(req_major,req_minor,rtk)

    if not tcv: err("ZEPHYR_TOOLCHAIN_VARIANT is not set\n\n"+g)
    if tcv!='gnuarmemb': err(f"ZEPHYR_TOOLCHAIN_VARIANT must be 'gnuarmemb', actual: '{tcv}'\n\n"+g)
    if not tcp: err("GNUARMEMB_TOOLCHAIN_PATH is not set\n\n"+g)
    if not Path(tcp).exists(): err(f"Nothing found at GNUARMEMB_TOOLCHAIN_PATH: '{tcp}'\n\n"+g)

    act_major,act_minor=parse_act(tcp)

    # strict equality on major id (e.g., 'asdk-12.3.1')
    if act_major != req_major:
        err("Toolchain major id mismatch:\n"
            f"  required major: {req_major}\n"
            f"  actual major:   {act_major}\n\n"+g)

    # compare minor build number
    if int(act_minor) < int(req_minor):
        err("Toolchain minor build is older than required:\n"
            f"  required minor: {req_minor}\n"
            f"  actual minor:   {act_minor}\n\n"+g)

if __name__=='__main__': main()
