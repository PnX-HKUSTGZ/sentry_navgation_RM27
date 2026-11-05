#!/usr/bin/env python3
"""Install Noble's GTSAM 4.2 binaries in a user prefix without sudo.

For normal deployment prefer: sudo apt install ros-jazzy-gtsam
This fallback retains distribution copyright/version records and adjusts two
packaging issues in the local CMake export only; the library binary is untouched.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--prefix',type=Path,default=Path.home()/'.local/opt/rm27-gtsam')
    args=parser.parse_args()
    if platform.machine()!='x86_64' or 'VERSION_CODENAME=noble' not in Path('/etc/os-release').read_text():
        raise SystemExit('This bootstrap is for Ubuntu 24.04 amd64; use the platform GTSAM package elsewhere.')
    prefix=args.prefix.resolve();debs=prefix/'debs';debs.mkdir(parents=True,exist_ok=True)
    packages=['libgtsam-dev','libgtsam4','libmetis5']
    subprocess.run(['apt-get','download',*packages],cwd=debs,check=True)
    manifest=[]
    for package in packages:
        matches=sorted(debs.glob(package+'_*.deb'))
        if len(matches)!=1:raise RuntimeError(f'Expected exactly one downloaded version of {package}')
        archive=matches[0]
        subprocess.run(['dpkg-deb','-x',str(archive),str(prefix)],check=True)
        manifest.append(dict(file=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest()))
    cmake=prefix/'usr/lib/cmake/GTSAM'
    p=cmake/'GTSAM-exports.cmake';s=p.read_text()
    s=s.replace('"/usr/lib/x86_64-linux-gnu/libmetis.so"','"${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libmetis.so.5"')
    s=s.replace('foreach(_cmake_target IN ITEMS metis-gtsam-if CppUnitLite gtsam)',
                'foreach(_cmake_target IN ITEMS metis-gtsam-if gtsam)')
    begin=s.index('# Create imported target CppUnitLite');end=s.index('# Create imported target gtsam',begin)
    p.write_text(s[:begin]+s[end:])
    p=cmake/'GTSAM-exports-relwithdebinfo.cmake';s=p.read_text()
    begin=s.index('# Import target "CppUnitLite"');end=s.index('# Import target "gtsam"',begin)
    p.write_text(s[:begin]+s[end:])
    (prefix/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'GTSAM_DIR={cmake}')


if __name__=='__main__':main()
