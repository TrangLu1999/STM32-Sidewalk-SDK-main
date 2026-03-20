#!/usr/bin/env python3

#
##############################################################################
# file:    delete_geolocation_demo_stack.py
# brief:   Delete all the AWS resources created for geolocation demo
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

import os
import subprocess
import sys


PYTHON_VERSION_MIN = (3, 9)

SHARED_AWS_CODE_ROOT_DIR   = os.path.join('..', '..', '..', 'common', 'AWS')
AWS_TEARDOWN_SCRIPT_NAME   = 'delete_geolocation_demo_stack.py'  # The name of the deployment script located in the SHARED_AWS_CODE_ROOT_DIR. Attention: this is not the name of this file

DEPLOYMENT_CONFIG_FILE     = 'config.yaml'  # Path to the configuration file to be used for deployment


def main():
    try:
        requirements_file = os.path.join(SHARED_AWS_CODE_ROOT_DIR, 'requirements.txt')

        print('[INFO] Checking if Python dependencies are satisfied...\n')
        sys.stdout.flush()

        # Ensure pip is installed
        try:
            import pip

        except ImportError:
            print('\n[INFO] pip not found - trying to install via ensurepip...\n')
            subprocess.run(
                [sys.executable, '-m', 'ensurepip'],
                text=True,
                check=False
            )
            print('\n[INFO] pip installed successfully\n')

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
                'install', '-r', requirements_file],
            text=True,
            check=True
        )

        print('\n[INFO] Python dependencies are satisfied')

        # Now call the deletion script
        subprocess.run(
            [sys.executable, os.path.join(SHARED_AWS_CODE_ROOT_DIR, AWS_TEARDOWN_SCRIPT_NAME),
                '--config', DEPLOYMENT_CONFIG_FILE
            ],
            text=True,
            check=True
        )

    except Exception as e:
        print('[ERROR] Failed to deploy Geolocation Demo. Error {}'.format(e))


if __name__ == "__main__":
    # Ensure the minimum Python version requirement is satisfied
    if sys.version_info < PYTHON_VERSION_MIN:
        sys.exit('Error: Python {}.{} or higher is required to run this script.'.format(PYTHON_VERSION_MIN[0], PYTHON_VERSION_MIN[1]))
    print('[INFO] Running on Python {}'.format(sys.version))

    # Call script entry point
    main()
