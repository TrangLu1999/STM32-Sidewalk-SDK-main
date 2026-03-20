#
##############################################################################
# file:    common_build_utils.py
# brief:   Utility methods and classes used by pre- and post-build scripts
##############################################################################
#
# Copyright (c) 2025 STMicroelectronics.
# All rights reserved.
#
# This software is licensed under terms that can be found in the LICENSE file
# in the root directory of this software component.
# If no LICENSE file comes with this software, it is provided AS-IS.
#
##############################################################################
#


import os
import re
import subprocess
import sys


class VersionNotFoundException(Exception):
    pass


def normalize_path(path):
    """Convert path to absolute and normalize slashes (always use forward slashes for GCC)."""
    return os.path.abspath(path).replace('\\', '/')


def find_package_root_dir(project_path: str, verbose: bool=False):
    """Try to get the git root directory starting from 'path' (or cwd if None).
    Returns the path or None if not a git repo or git not installed."""
    root_dir = None
    try:
        if verbose:
            print(f'[INFO] Trying to resolve package root path via git')
        cmd = ['git', 'rev-parse', '--show-toplevel']
        # Run in the specified directory or cwd
        result = subprocess.run(
            cmd,
            cwd=project_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=True,
            text=True
        )
        root_dir = result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        if verbose:
            print(f'[WARN] Unable to resolve package root path via git, falling back to analyzing project path')
        root_dir = normalize_path(os.path.join(project_path, '../../../..'))

    if verbose:
        print(f'[INFO] Package root  : {root_dir}')
    return root_dir


def parse_package_version(version_file_path: str, verbose: bool=False):
    try:
        version = None
        pattern = re.compile(
            r'_APPS_PACKAGE_VER\s*:=\s*'  # Key and assignment
            r'(\d+)\.(\d+)\.(\d+)'        # major.minor.patch (mandatory)
            r'(?:\.(\d+))?'               # optional .build (non-capturing group with capture inside)
        )

        with open(version_file_path, 'r') as f:
            for line in f:
                match = pattern.search(line)
                if match:
                    major, minor, patch, build = match.groups()
                    version = {
                        'major': int(major),
                        'minor': int(minor),
                        'patch': int(patch),
                        'build': int(build) if build is not None else None
                    }
        if version is None:
            raise VersionNotFoundException(f'Can\'t extract app version information from {version_file_path}')

        if verbose:
            print(f'[INFO] Identified app version: {version["major"]}.{version["minor"]}.{version["patch"]}' + (f'.{version["build"]}' if version["build"] is not None else ''))

        return version
    except Exception as e:
        print(f'[ERROR] Failed to write output file: {e}', file=sys.stderr)
        sys.exit(-2)


def load_intel_hex(filename, expected_start_addr: int) -> bytearray:
    data = bytearray()
    upper_addr = 0
    start_found = False
    last_processed_addr = 0

    with open(filename, 'r') as f:
        for line in f:
            if not line.startswith(':'):
                continue  # skip invalid lines

            # Parse fields
            length = int(line[1:3], 16)
            addr = int(line[3:7], 16)
            rectype = int(line[7:9], 16)
            raw_data = line[9:9+length*2]
            # checksum = int(line[9+length*2:9+length*2+2], 16)  # can be checked if needed

            if rectype == 0x00:  # data record
                full_addr = upper_addr + addr

                # Check if app image start address is found
                if full_addr == expected_start_addr:
                    start_found = True

                if start_found:
                    if last_processed_addr and full_addr != last_processed_addr: 
                        # Gap in the data detected, terminating
                        break

                    bytes_data = bytes.fromhex(raw_data)
                    # Append sequentially; for sparse files you may need to handle gaps
                    data.extend(bytes_data)
                    # Store the last address we've processed
                    last_processed_addr = full_addr + length 
            elif rectype == 0x04:  # extended linear address
                upper_addr = int(raw_data, 16) << 16
            elif rectype == 0x01:  # EOF
                break

    return data


def ieee_802_3_crc32(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF  # final XOR
