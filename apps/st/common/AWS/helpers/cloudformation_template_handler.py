#
##############################################################################
# file:    cloudformation_template_handler.py
# brief:   Helper class to manage CloudFormation template uploads and deletion
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

import hashlib

from fnmatch import fnmatch
from io import BytesIO
from os import path

from constants.geolocation_demo_constants import GeolocationDemoConstants
from libs.s3_client import S3Client
from libs.utils import *


class CloudFormationTemplateHandler:

    @staticmethod
    def upload_template(s3_client: S3Client, lambda_sources: dict):
        constants = GeolocationDemoConstants()

        if not s3_client.bucket_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME):
            s3_client.create_bucket(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME)

        s3_client.put_object(
            bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME,
            bucket_key=constants.CLOUDFORMATION_TEMPLATE_DST_FILENAME,
            file_path=str(path.join(constants.CLOUDFORMATION_TEMPLATE_SRC_ROOT_DIR, constants.CLOUDFORMATION_TEMPLATE_SRC_FILENAME)),
            content_type='text/yaml'
        )

        for lambda_name, lambda_info in lambda_sources.items():
            buffer = BytesIO()

            log_info(f'Compressing {lambda_name} files...')
            zip_top_level_files(path.join(constants.LAMBDA_SRC_ROOT_DIR, lambda_info['src_folder']), buffer)
            buffer.seek(0)

            h = hashlib.new('sha256')
            while chunk := buffer.read(8192):
                h.update(chunk)
            lambda_info['hash'] = h.hexdigest()
            buffer.seek(0)

            log_info(f'Uploading {lambda_name} files to S3 as {lambda_info["src_folder"]}_{lambda_info["hash"]}.zip...')
            s3_client.put_object(
                bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME,
                bucket_key=f'{lambda_info["src_folder"]}_{lambda_info["hash"]}.zip',
                raw_bytes=buffer,
                content_type='application/zip'
            )


    @staticmethod
    def delete_template(s3_client: S3Client, lambda_src_names: list|None=None):
        constants = GeolocationDemoConstants()

        if not s3_client.bucket_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME):
            return

        # Delete the CloudFormation template associated with this app from the S3 bucket
        if s3_client.object_exists(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=constants.CLOUDFORMATION_TEMPLATE_DST_FILENAME):
            s3_client.delete_object(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=constants.CLOUDFORMATION_TEMPLATE_DST_FILENAME)

        # Delete Lambda sources
        if lambda_src_names is not None:
            for file_name in lambda_src_names:
                try:
                    all_objects = s3_client.list_objects(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME)
                    for obj in all_objects:
                        if fnmatch(obj['Key'], f'{file_name}_*.zip'):
                            log_info(f'Deleting Lambda source file {obj['Key']}.zip from S3')
                            s3_client.delete_object(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME, bucket_key=obj['Key'])
                except:
                    # keep deleting other Lambda sources
                    continue

        # Delete the bucket itself if it is empty
        if s3_client.is_bucket_empty(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME):
            s3_client.delete_bucket(bucket_name=constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME)
        else:
            log_warn(f'Unable to delete S3 bucket {constants.CLOUDFORMATION_TEMPLATE_BUCKET_NAME}. The bucket is not empty')
