# Copyright 2023 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
##############################################################################
# file:    config.py
# brief:   Parser of the deployment configuration stored in config.yaml
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

import base64
import json
import os

import yaml

from libs.utils import *


class Config:
    """
    Provides methods for reading and updating config files

    Attributes
    ----------
        aws_profile: str
            AWS profile name.
        sid_demo_name: str
            Sidewalk Demo app name.
        interactive mode: bool
            Flag that enables/disables interactive mode.
    """
    DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1].joinpath('config.yaml')
    DEPLOYMENT_OUTPUTS_FILE = 'deployment_outputs.yaml'

    __instance = None
    __init_done = False

    def __new__(cls, config_path: Path|None=None):
        if cls.__instance is None:
            cls.__instance = super(Config, cls).__new__(cls)
        return cls.__instance

    def __init__(self, config_path: Path|None=None):
        if not Config.__init_done:
            self._read_config(config_path)
            Config.__init_done = True

    def _read_config(self, config_path: Path|None):
        """
        Reads config file.
        """
        try:
            self.config_path = config_path if config_path is not None else self.DEFAULT_CONFIG_PATH
            self.deployment_outputs_path = self.config_path.resolve().parents[0].joinpath(self.DEPLOYMENT_OUTPUTS_FILE)

            config = yaml.safe_load(read_file(self.config_path))

            # Configuration parameters
            self.aws_profile              = config.get('Config', {}).get('AWS_PROFILE', 'default')
            self.aws_access_key_id        = config.get('Config', {}).get('AWS_ACCESS_KEY_ID', 'default')
            self.aws_secret_access_key    = config.get('Config', {}).get('AWS_SECRET_ACCESS_KEY', 'default')
            self.aws_session_token        = config.get('Config', {}).get('AWS_SESSION_TOKEN', 'default')
            self.aws_default_region       = config.get('Config', {}).get('AWS_DEFAULT_REGION', 'default')
            self.sid_demo_name            = config.get('Config', {}).get('SID_DEMO_NAME', 'SidewalkGeolocation')
            self.sid_demo_mqtt_topic_root = config.get('Config', {}).get('SID_DEMO_MQTT_TOPIC_ROOT', 'sidewalk/geolocation')
            self.interactive_mode         = config.get('Config', {}).get('INTERACTIVE_MODE', True)
            self.manage_device_instance   = config.get('Config', {}).get('SID_DEMO_PROVISION_DEVICE', True)
            self.sid_demo_device_name     = config.get('Config', {}).get('SID_DEMO_DEVICE_NAME', f'{self.sid_demo_name}Device')
            self.target_mcu               = config.get('Config', {}).get('TARGET_MCU', ['WBA65xI', 'WBA55xG', 'WBA52xG'])

            # Handle default device name
            if self.sid_demo_device_name is None:
                self.sid_demo_device_name = f'{self.sid_demo_name}Device'

            # Handle the list of target MCUs
            if not  isinstance(self.target_mcu, list):
                self.target_mcu = [self.target_mcu]  # Convert to iterable object if only one MCU name was supplied

            # Handle AWS region selection
            if self.aws_default_region is not None and self.aws_default_region != 'us-east-1':
                terminate(f'Sidewalk availability is currently limited to us-east-1 region, but you configured the script to use {self.aws_default_region}', ErrCode.EXCEPTION)
            self.aws_default_region = 'us-east-1'  # Leave this as us-east-1 unless you know what you are doing

            self.region_name = self.aws_default_region

            # Paths
            self.provisioning_output_dir = str(os.path.normpath(os.path.join(os.getcwd(), config.get('_Paths', {}).get('PROVISION_OUTPUT_DIR', os.getcwd()))))
            self.provisioning_tool_dir = str(os.path.normpath(os.path.join(os.getcwd(), config.get('_Paths', {}).get('PROVISION_SCRIPT_DIR', os.getcwd()))))
        except yaml.YAMLError as e:
            terminate(f'Invalid structure of a config file: {e}', ErrCode.EXCEPTION)

    def set_wireless_device_profile_id(self, profile_id: str):
        self._update_deployment_outputs(self.deployment_outputs_path, 'Outputs', 'DEVICE_PROFILE_ID', profile_id, 'Sidewalk Device Profile ID of the created device instance in AWS')

    def set_wireless_device_id(self, wireless_device_id: str):
        self._update_deployment_outputs(self.deployment_outputs_path, 'Outputs', 'DEVICE_ID', wireless_device_id, 'Sidewalk Wireless Device ID of the created device instance in AWS')

    def store_device_provisioning_json(self, device_provisioning_data: dict, file_name: str):
        json_data = json.dumps(device_provisioning_data, indent=4)
        json_file_path = os.path.join(self.provisioning_output_dir, file_name)
        os.makedirs(os.path.dirname(json_file_path), exist_ok=True)
        with open(json_file_path, 'w') as f:
            f.write(json_data)

    @staticmethod
    def _update_deployment_outputs(outputs_file_path: Path, parent: str, param: str, val: str, comment: str|None = None):
        """
        Updates parameter in the deployment outputs file.

        :param outputs_file_path: Path to the deployment outputs file.
        :param parent:            Parent of the parameter it is grouped under.
        :param param:             Name of the parameter.
        :param val:               Value of the parameter.
        """
        new_lines = []
        val = val if val else 'null'

        if outputs_file_path.exists():
            with open(outputs_file_path, 'r') as outputs:
                text = outputs.read()
                outputs.seek(0)
                lines = outputs.readlines()
        else:
            text = ''
            lines = []

        if not parent in text:
            lines.append(f'{parent}:\n')

        exist = param in text
        for line in lines:
            if not exist and parent in line:
                # create new record
                new_lines.append(line)
                new_lines.append(f'    {param}: {val}{"  # " + comment if comment is not None else ""}\n')
            else:
                if param in line:
                    # update existing record
                    existing_comment = '' if '#' not in line else '  #' + line.split('#')[-1].rstrip()
                    if comment is None:
                        new_comment = existing_comment
                    else:
                        new_comment = f'  # {comment}'
                    new_lines.append(f'    {param}: {val}{new_comment}\n')
                else:
                    # do not modify the line
                    new_lines.append(line)

        with open(outputs_file_path, 'w') as outputs:
            outputs.writelines(new_lines)
