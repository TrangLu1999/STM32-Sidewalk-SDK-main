#!/usr/bin/env python3

#
##############################################################################
# file:    pre_build_steps_aim.py
# brief:   Pre-build step actions for STM32CubeIDE-based Sidewalk AIM project
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
import base64
import binascii
import datetime
import os
import subprocess
import sys


PYTHON_VERSION_MIN            = (3, 6)

SCRIPT_ROOT                   = os.path.abspath(os.path.dirname(__file__))
SHARED_PRE_BUILD_STEPS_SCRIPT = 'pre_build_steps.py'
VERIFICATION_KEY_CFILE_NAME   = 'application_verification_key.c'
SIGNING_TOOL_PATH_DEFAULT     = os.path.abspath(os.path.join(SCRIPT_ROOT, '../../../../../tools/firmware_signing/firmware_sign.py'))
DEPENDENCY_INSTALLER_SCRIPT   = os.path.join(SCRIPT_ROOT, 'install_dependencies.py')


def load_public_key(key_file_path: str, verbose: bool=False) -> bytes|None:
    public_key_bytes = None

    with open(key_file_path, 'r', encoding='utf-8') as f:
        raw_key_data = f.read()

        # Extract public key
        try:
            public_key_b64   = raw_key_data.split('-----BEGIN ED25519 PUBLIC KEY-----\n')[1].split('\n-----END ED25519 PUBLIC KEY-----')[0]
            public_key_bytes = base64.b64decode(public_key_b64, validate=True)
            if verbose:
                print('[INFO] Loaded public key from the key file')
        except IndexError:
            public_key_bytes = None
            if verbose:
                print('[WARN] Specified key file does not contain the public key')
        except (binascii.Error, ValueError) as e:
            public_key_bytes = None
            print('[WARN] Public key has invalid format in the provided key file: {}'.format(e))

    # Provide back the loaded key
    return public_key_bytes


def generate_verification_key_c_file(template_path: str, output_path: str, pub_key: bytes, verbose: bool=False):
    try:
        # Load .c code template file
        with open(template_path, 'r') as fin:
            content = fin.read()

        # Compile public key C array initializer
        pk_hex_bytes = ['0x{:02X}u,'.format(b) for b in pub_key]  # list of '0xXXu' strings
        pk_lines = []
        bytes_per_line = 16
        for i in range(0, len(pk_hex_bytes), bytes_per_line):
            line = '    ' + ' '.join(pk_hex_bytes[i:i+bytes_per_line])
            pk_lines.append(line)
        public_key_array_init = '\n'.join(pk_lines)

        template_params = {
            'CURRENT_YEAR'                                 : str(datetime.datetime.now().year),
            'APP_VERIFICATION_PUBLIC_KEY' : public_key_array_init,
        }

        for key, val in template_params.items():
            placeholder = '@{}@'.format(key)
            content = content.replace(placeholder, str(val))

        # Read in the source if it exists
        if os.path.exists(output_path):
            with open(output_path, 'r', encoding='utf-8') as f:
                old_content = f.read()
        else:
            old_content = ''

        # Check if existing source file requires any updates
        if old_content != content:
            with open(output_path, 'w', encoding='utf-8') as fout:
                fout.write(content)

            if verbose:
                print('[INFO] Verification key source file written to: {}'.format(output_path))
        else:
            if verbose:
                print('[INFO] No changes to the verification key, source file update skipped')

    except Exception as e:
        print('[ERROR] Failed to write version header file: {}'.format(e), file=sys.stderr)
        sys.exit(-4)


def main():
    parser = argparse.ArgumentParser(
        description='Pre-build steps for Application Install Manager application'
    )

    parser.add_argument(
        '-a', '--app-dir',
        required=True,
        help='Path to the Sidewalk application root directory (e.g., sid_900, sid_dut, etc.).'
    )

    parser.add_argument(
        '-b', '--build-dir',
        default=os.getcwd(),
        help='Current build working directory (where the output file should go). Current working dir will be used if not specified explicitly'
    )

    parser.add_argument(
        '-v', '--version-file',
        required=True,
        help='Path to the file containing application package version'
    )

    parser.add_argument(
        '-k', '--key-file',
        required=True,
        help='Path to the .pem file with application signing key. If file does not exit a new key will be generated automatically'
    )

    parser.add_argument(
        '--version-output-dir',
        default=None,
        help='Output directory for the version header file. If skipped, the header file will be put in the Core/Inc folder of the project'
    )

    parser.add_argument(
        '--key-output-file',
        default=None,
        help='Path to store the generated verification key C file. If skipped, the file will be put in the Core/Src folder of the project'
    )

    parser.add_argument(
        '-s', '--signing-tool',
        default=SIGNING_TOOL_PATH_DEFAULT,
        help='Path to the firmware signing tool'
    )

    parser.add_argument(
        '-t', '--template-file',
        default=None,
        help='Path to the template file for the verification key C code'
    )

    parser.add_argument(
        '-V', '--verbose',
        action='store_true',
        help='Enable verbose output for debugging.'
    )

    args = parser.parse_args()

    print('[INFO] Running pre-build actions...')
    sys.stdout.flush()

    try:
        # Collect essential path args
        app_root_path             = normalize_path(args.app_dir)
        build_dir                 = normalize_path(args.build_dir)
        version_file              = normalize_path(args.version_file)
        version_header_out_dir    = normalize_path(args.version_output_dir if args.version_output_dir is not None else normalize_path(os.path.join(app_root_path, 'Core', 'Inc')))
        verification_key_file     = normalize_path(args.key_file)
        verification_key_out_file = normalize_path(args.key_output_file if args.key_output_file is not None else normalize_path(os.path.join(app_root_path, VERIFICATION_KEY_CFILE_NAME)))  #normalize_path(os.path.join(app_root_path, 'Core', 'Src', VERIFICATION_KEY_CFILE_NAME)))
        firmware_signing_tool     = normalize_path(args.signing_tool)
        c_template_file           = normalize_path(args.template_file) if args.template_file is not None else normalize_path(os.path.join(SCRIPT_ROOT, '{}.template'.format(VERIFICATION_KEY_CFILE_NAME)))

        if args.verbose:
            print('[INFO] App root               : {}'.format(app_root_path))
            print('[INFO] Build dir              : {}'.format(build_dir))
            print('[INFO] Version file           : {}'.format(version_file))
            print('[INFO] Version output dir     : {}'.format(version_header_out_dir))
            print('[INFO] Verification key file  : {}'.format(verification_key_file))
            print('[INFO] Verification key output: {}'.format(verification_key_out_file))
            print('[INFO] Signing tool location  : {}'.format(firmware_signing_tool))
            print('[INFO] Template file          : {}'.format(c_template_file))

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

        print('[INFO] Processing application version information...')
        sys.stdout.flush()
        # Call shared pre-build script
        shared_pre_build_call_args = [
            sys.executable, os.path.join(SCRIPT_ROOT, SHARED_PRE_BUILD_STEPS_SCRIPT),
            '-a', args.app_dir,
            '-b', args.build_dir,
            '-v', args.version_file,
        ]
        if args.version_output_dir:
            shared_pre_build_call_args += ['--version-output-dir', args.version_output_dir]
        if args.verbose:
            shared_pre_build_call_args.append('-V')
        subprocess.run(
            shared_pre_build_call_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        sys.stdout.flush()

        print('[INFO] Processing application verification key...')
        # Check the key file exists and call the signing tool to create a new key if needed
        if os.path.exists(verification_key_file) and os.path.isfile(verification_key_file):
            pub_key = load_public_key(verification_key_file, args.verbose)
        else:
            pub_key = None

        # Create a new key if loading failed for any reason
        if not pub_key:
            print('[INFO] Creating a new firmware signing key')
            sys.stdout.flush()
            subprocess.run([sys.executable, firmware_signing_tool,
                    'create_key',
                    '--key-file', verification_key_file,
                    '--force-override',
                ],
                text=True,
                check=True
            )
            sys.stdout.flush()

            # Try loading the newly generated key
            pub_key = load_public_key(verification_key_file, args.verbose)
            if not pub_key:
                raise FileNotFoundError('Unable to load verification key file')

        # Produce .c file with the public key
        generate_verification_key_c_file(c_template_file, verification_key_out_file, pub_key, args.verbose)

        # Done
        print('[INFO] Pre-build actions completed successfully')

    except Exception as e:
        print('[ERROR] Pre-build script failed: {}'.format(e), file=sys.stderr)
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
