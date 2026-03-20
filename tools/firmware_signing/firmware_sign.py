#!/usr/bin/env python3

#
##############################################################################
# file:    firmware_sign.py
# brief:   Wrapper with a safe Python version check for firmware signing tool
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
import sys


PYTHON_VERSION_MIN = (3, 6)


if __name__ == '__main__':
    # Log Python runtime version
    print('[INFO] Running on Python {}'.format(sys.version))

    # Ensure minimum Python version requirement is satisfied
    if sys.version_info < PYTHON_VERSION_MIN:
        print('Error: Python {}.{} or higher is required to run this script.'.format(PYTHON_VERSION_MIN[0], PYTHON_VERSION_MIN[1]))
        sys.exit(127)

    # Redirect all stderr output to stdout as CubeIDE console may not display stderr output
    sys.stderr = sys.stdout

    # Call script entry point
    try:
        from firmware_signing.run import main
        main()
    except Exception as e:
        print('Failed to run firmware signing tool: {}'.format(e))
        sys.exit(1)
