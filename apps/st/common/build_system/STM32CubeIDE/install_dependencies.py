#!/usr/bin/env python3

#
##############################################################################
# file:    install_dependencies.py
# brief:   Utility for installing Python dependencies
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

import subprocess
import sys


PYTHON_VERSION_MIN = (3, 6)


def main():
    if len(sys.argv) != 2:
        print('Usage: python {} path/to/requirements.txt'.format(__file__))
        sys.exit(-1)

    requirements_file = normalize_path(sys.argv[1])

    print('[INFO] Checking if Python dependencies are satisfied...')
    sys.stdout.flush()

    try:
        # Ensure pip is installed
        try:
            import pip

        except ImportError:
            print('[INFO] pip not found - trying to install via ensurepip...')
            subprocess.run(
                [sys.executable, '-m', 'ensurepip'],
                text=True,
                check=False
            )
            print('[INFO] pip installed successfully')

        # Upgrade pip first
        subprocess.run(
            [sys.executable, '-m', 'pip',
                'install', '--upgrade', 'pip'],
            text=True,
            check=False
        )

        # Now install the dependencies
        subprocess.run(
            [sys.executable, '-m', 'pip',
                'install', '--user', '-r', requirements_file],
            text=True,
            check=True
        )

        print('[INFO] Python dependencies are satisfied')

    except Exception as e:
        print('[ERROR] Failed to install Python dependencies: {}'.format(e), file=sys.stderr)
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
