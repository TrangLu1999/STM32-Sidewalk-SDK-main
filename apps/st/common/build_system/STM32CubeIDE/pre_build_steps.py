#!/usr/bin/env python3

#
##############################################################################
# file:    pre_build_steps.py
# brief:   Pre-build step actions for STM32CubeIDE-based Sidewalk projects
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
import datetime
import os
import sys
import subprocess


PYTHON_VERSION_MIN = (3, 6)


SCRIPT_ROOT = os.path.abspath(os.path.dirname(__file__))
VERSION_HEADER_FILE_NAME = 'sid_app_version_cubeide.h'


def get_git_commit_info(app_root_dir, verbose: bool=False):
    """
    Returns (commit_hash, commit_description) from a Git repo at app_root_dir.
    Annotates hash with (dirty) or (unknown state) if needed.
    """
    if verbose:
        print('[INFO] Retrieving branch and commit hash info from Git...')

    def run_git_command(args):
        try:
            result = subprocess.run(
                ['git'] + args,
                cwd=app_root_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False
            )
            return result.returncode, result.stdout.strip(), result.stderr.strip()
        except FileNotFoundError:
            return -1, '', 'git not found'
        except Exception as e:
            return -1, '', str(e)

    # Get commit hash
    code, commit_hash, err = run_git_command(['rev-parse', 'HEAD'])
    if code != 0:
        print('[WARN]: Failed to get Git commit hash. Exit code: {}. Error: {}'.format(code, err))
        return 'N/A', 'N/A'

    # Check for dirty working tree
    diff_code, diff_output, diff_err = run_git_command(['diff', '--stat', 'HEAD'])
    if diff_code == 0:
        if diff_output:
            commit_hash += ' (dirty)'
    else:
        print('[WARN]: Failed to get Git diff. Exit code: {}. Error: {}'.format(diff_code, diff_err))
        commit_hash += ' (unknown state)'

    # Get commit description
    desc_code, description, desc_err = run_git_command(['describe', '--abbrev=8', '--tags'])
    if desc_code != 0:
        print('[WARN]: Failed to get Git description. Exit code: {}. Error: {}'.format(desc_code, desc_err))
        description = 'N/A'

    return commit_hash, description


def generate_gcc_flags(package_root, build_dir, verbose: bool=False):
    try:
        output_path = os.path.join(build_dir, 'gcc_compiler_flags.txt')
        with open(output_path, 'w') as f:
            f.write('-fmacro-prefix-map=\"{}\"=\"\"\n'.format(package_root))

        if verbose:
            print('[INFO] Compiler flags written to: {}'.format(output_path))
    except Exception as e:
        print('[ERROR] Failed to write compiler flags file: {}'.format(e), file=sys.stderr)
        sys.exit(-3)


def generate_version_info_header(output_dir: str, version_info: dict, verbose: bool=False):
    try:
        template_path = os.path.join(SCRIPT_ROOT, '{}.template'.format(VERSION_HEADER_FILE_NAME))
        output_path = os.path.join(output_dir, VERSION_HEADER_FILE_NAME)

        with open(template_path, 'r') as fin:
            content = fin.read()

        # Compile version string
        version_string = '{}.{}.{}'.format(version_info['major'], version_info['minor'], version_info['patch']) + ('.{}'.format(version_info['build']) if version_info['build'] is not None else '')
        build_number_definition_str:str = '\n#define SID_APP_PROJECT_BUILD_VERSION      {}\n'.format(version_info['build']) if version_info['build'] is not None else ''

        # Try to get commit hash and description from git
        commit_hash, commit_desc = get_git_commit_info(output_dir, verbose)

        template_params = {
            'CURRENT_YEAR'                                 : str(datetime.datetime.now().year),
            'PROJECT_VERSION'                              : version_string,
            'PROJECT_VERSION_MAJOR'                        : version_info['major'],
            'PROJECT_VERSION_MINOR'                        : version_info['minor'],
            'PROJECT_VERSION_PATCH'                        : version_info['patch'],
            'OPTIONAL_DEFINE_SID_APP_PROJECT_BUILD_VERSION': build_number_definition_str,
            'PROJECT_COMMIT_HASH'                          : commit_hash,
            'PROJECT_COMMIT_DESCRIPTION'                   : commit_desc,
        }

        for key, val in template_params.items():
            placeholder = '@{}@'.format(key)
            content = content.replace(placeholder, str(val))

        # Read in the existing header if it exists
        if os.path.exists(output_path):
            with open(output_path, 'r', encoding='utf-8') as f:
                old_content = f.read()
        else:
            old_content = ''

        # Check if existing header requires any updates
        if old_content != content:
            with open(output_path, 'w', encoding='utf-8') as fout:
                fout.write(content)

            if verbose:
                print('[INFO] Version header file written to: {}'.format(output_path))
        else:
            if verbose:
                print('[INFO] No changes in the version information, header file update skipped')
    except Exception as e:
        print('[ERROR] Failed to write version header file: {}'.format(e), file=sys.stderr)
        sys.exit(-4)


def main():
    parser = argparse.ArgumentParser(
        description='Shared pre-build steps for STM32 platform'
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
        '--version-output-dir',
        default=None,
        help='Output directory for the version header file. If skipped, the header file will be put in the Core/Inc folder of the project'
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
        app_root_path          = normalize_path(args.app_dir)
        build_dir              = normalize_path(args.build_dir)
        version_file           = normalize_path(args.version_file)
        version_header_out_dir = normalize_path(args.version_output_dir if args.version_output_dir is not None else os.path.join(app_root_path, 'Core', 'Inc'))

        if args.verbose:
            print('[INFO] App root          : {}'.format(app_root_path))
            print('[INFO] Build dir         : {}'.format(build_dir))
            print('[INFO] Version file      : {}'.format(version_file))
            print('[INFO] Version output dir: {}'.format(version_header_out_dir))

        # Locate package root
        package_root = find_package_root_dir(app_root_path, args.verbose)

        # Determine app version
        version_info = parse_package_version(version_file, args.verbose)

        # Step 1 - generate project- and location-specific GCC command line flags
        generate_gcc_flags(package_root, build_dir, args.verbose)

        # Step 2 - generate header file with version information
        generate_version_info_header(version_header_out_dir, version_info, args.verbose)

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
    from common_build_utils import normalize_path, find_package_root_dir, parse_package_version

    # Call script entry point
    main()
