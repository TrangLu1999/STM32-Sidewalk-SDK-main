#!/usr/bin/env python3

#
##############################################################################
# file:    post_build_steps_ota.py
# brief:   Post-build step actions for STM32CubeIDE-based Sidewalk OTA projects
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


from __future__ import print_function

import argparse
import os
import sys
import subprocess


PYTHON_VERSION_MIN = (3, 6)

SCRIPT_ROOT                 = os.path.abspath(os.path.dirname(__file__))
SIGNING_TOOL_PATH_DEFAULT   = os.path.abspath(os.path.join(SCRIPT_ROOT, '../../../../../tools/firmware_signing/firmware_sign.py'))
DEPENDENCY_INSTALLER_SCRIPT = os.path.join(SCRIPT_ROOT, 'install_dependencies.py')


def main():
    parser = argparse.ArgumentParser(
        description='Post-build steps for OTA-capable Sidewalk applications'
    )

    parser.add_argument(
        '-m', '--map-file',
        default=None,
        help='Name of the .map file in the build directory. This file is used to locate firmware image footer in the binary'
    )

    parser.add_argument(
        '-e', '--elf-file',
        default=None,
        help='Name of the .elf file in the build directory. This file will be patched to include valid image signature and CRC'
    )

    parser.add_argument(
        '-x', '--hex-file',
        default=None,
        help='Name of the .hex file in the build directory. This file will be patched to include valid image signature and CRC'
    )

    parser.add_argument(
        '-b', '--bin-file',
        default=None,
        help='Name of the .bin file in the build directory. This file will be patched to include valid image signature and CRC'
    )

    parser.add_argument(
        '-k', '--key-file',
        required=True,
        help='Path to the .pem file with application signing key. If file does not exit a new key will be generated automatically'
    )

    parser.add_argument(
        '-s', '--signing-tool',
        default=SIGNING_TOOL_PATH_DEFAULT,
        help='Path to the firmware signing tool'
    )

    parser.add_argument(
        '-V', '--verbose',
        action='store_true',
        help='Enable verbose output for debugging.'
    )

    args = parser.parse_args()

    print('[INFO] Running post-build actions...')
    sys.stdout.flush()

    try:
        # Collect essential path args
        verification_key_file     = normalize_path(args.key_file)
        firmware_signing_tool     = normalize_path(args.signing_tool)

        if args.verbose:
            print('[INFO] Verification key file: {}'.format(verification_key_file))
            print('[INFO] Signing tool location: {}'.format(firmware_signing_tool))

        # Ensure Python deps for signing tool are satisfied if requirements.txt exists
        signing_tool_requirements_file = os.path.join(os.path.dirname(firmware_signing_tool), 'requirements.txt')
        if os.path.exists(signing_tool_requirements_file):
            print('[INFO] Ensuring Python dependencies are installed...')
            sys.stdout.flush()
            subprocess.run([sys.executable, DEPENDENCY_INSTALLER_SCRIPT,
                signing_tool_requirements_file,
                ],
                text=True,
                check=True
            )
            sys.stdout.flush()

        print('[INFO] Signing the firmware...')
        sys.stdout.flush()
        # Call the signing tool
        signing_tool_call_args = [sys.executable, firmware_signing_tool]
        if args.verbose:
            signing_tool_call_args.append('-V')
        
        signing_tool_call_args += [
            'sign',
            '-k', verification_key_file,
            '-g',  # Generate new key if verification_key_file does not exist or is invalid
            '-o',  # Produce OTA images on top of signing the current images
        ]
        if args.map_file:
            signing_tool_call_args += ['-m', normalize_path(args.map_file)]
        if args.elf_file:
            signing_tool_call_args += ['-e', normalize_path(args.elf_file)]
        if args.hex_file:
            signing_tool_call_args += ['-x', normalize_path(args.hex_file)]
        if args.bin_file:
            signing_tool_call_args += ['-b', normalize_path(args.bin_file)]

        subprocess.run(
            signing_tool_call_args,
            text=True,
            check=True
        )
        sys.stdout.flush()

        # Done
        print('[INFO] Post-build actions completed successfully')

    except Exception as e:
        print('[ERROR] Post-build script failed: {}'.format(e), file=sys.stderr)
        exit(-1)


if __name__ == '__main__':
    if sys.version_info < PYTHON_VERSION_MIN:
        sys.exit('Error: Python {}.{} or higher is required to run this script.'.format(PYTHON_VERSION_MIN[0], PYTHON_VERSION_MIN[1]))
    print('[INFO] Running on Python {}'.format(sys.version))

    # Redirect all stderr output to stdout as CubeIDE console may not display stderr output
    sys.stderr = sys.stdout

    # Import the dependencies requiring the minimum Python version
    from common_build_utils import normalize_path

    # Call script entry point
    main()
