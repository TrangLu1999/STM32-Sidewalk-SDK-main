#
##############################################################################
# file:    geolocation_uplink_lambda_handler.py
# brief:   Geolocation Sidewalk uplink processor and location resolver
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

import json
import base64
import boto3
import os

import math

import traceback

from fragments_assembler import *
from tlv_data_parser import *


__SEMTECH_GNSS_RESOLVER_CUTOFF_TIME_ACCURACY: int = 16000  # GNSS resolver accepts assist time with accuracy better than 16 seconds, otherwise provided assist time is ignored


def try_resolve_wifi_position(wifi_scan_payload):
    resolved_position = None

    try:
        iotwireless_client = boto3.client('iotwireless')

        # Resolve position using WiFi scans
        get_position_estimate_response = iotwireless_client.get_position_estimate(
            WiFiAccessPoints=wifi_scan_payload
        )

        # Check the request was processed successfully
        if get_position_estimate_response['ResponseMetadata']['HTTPStatusCode'] == 200:
            # Read response body and convert into JSON object
            wifi_based_position = json.loads(get_position_estimate_response['GeoJsonPayload'].read().decode("utf-8"))

            # Report back the resolved position
            resolved_position = wifi_based_position
        else:
            print(f'Unable to resolve WiFi position: {get_position_estimate_response}')

    # React on ResourceNotFoundException
    except iotwireless_client.exceptions.ResourceNotFoundException as e:
        # Typically this means the position cannot be calculated reliably from the supplied scan data (e.g. insufficient WiFi BSSIDs supplied)
        print(f'Unable to resolve WiFi position: {e}')

    except Exception:
        print(f'Unexpected error occurred when resolving WiFi scans: {traceback.format_exc()}')

    return resolved_position


def try_resolve_nav3_gnss_position(gnss_scan_payload):
    resolved_position = None

    try:
        iotwireless_client = boto3.client('iotwireless')

        get_position_estimate_response=iotwireless_client.get_position_estimate(
            Gnss=gnss_scan_payload
        )

        # Check the request was processed successfully
        if get_position_estimate_response['ResponseMetadata']['HTTPStatusCode'] == 200:
            # Read response body and convert into JSON object
            gnss_based_position = json.loads(get_position_estimate_response['GeoJsonPayload'].read().decode("utf-8"))

            # Report back the resolved position
            resolved_position = gnss_based_position
        else:
            print(f'Unable to resolve GNSS position: {get_position_estimate_response}')

    # React on ResourceNotFoundException
    except iotwireless_client.exceptions.ResourceNotFoundException as e:
        # Typically this means the position cannot be calculated reliably from the supplied scan data (e.g. insufficient satellites detected)
        print(f'Unable to resolve GNSS position: {e}')

    except Exception:
        print(f'Unexpected error occurred when resolving GNSS scan: {traceback.format_exc()}')

    return resolved_position


def update_iot_wireless_device_position(wireless_device_id, geojson_payload):
    try:
        iotwireless_client = boto3.client('iotwireless')

        update_resource_position_response=iotwireless_client.update_resource_position(
            ResourceIdentifier=wireless_device_id,
            ResourceType='WirelessDevice',
            GeoJsonPayload=json.dumps(geojson_payload).encode('utf-8')
        )

        # Check the request was processed successfully
        if update_resource_position_response['ResponseMetadata']['HTTPStatusCode'] == 204:
            print('Successfully updated IoT Wireless device position')
        else:
            print(f'Unable to update IoT Wireless device position: {update_resource_position_response}')
 
    except Exception:
        print(f'Unexpected error occurred when updating IoT Wireless device position: {traceback.format_exc()}')


def lambda_handler(event, context):
    try:
        # ---------------------------------------------
        # Read the environment variables
        # ---------------------------------------------
        geolocation_mqtt_topic_base = os.environ.get('GEOLOCATION_MQTT_TOPIC_BASE')

        # ---------------------------------------------------------------
        # Receive and record incoming event in the CloudWatch log group.
        # ---------------------------------------------------------------
        print(f'Received event: {event}')

        uplink = event.get("uplink")
        if uplink is None:
            print("Unsupported request received {}".format(event))
            return {
                'statusCode': 400,
                'body': json.dumps('Unsupported request received. Only uplink and notification are supported')
            }

        # ---------------------------------------------
        # Read the metadata
        # ---------------------------------------------
        wireless_metadata  = uplink.get("WirelessMetadata")
        wireless_device_id = uplink.get("WirelessDeviceId")
        sidewalk           = wireless_metadata.get("Sidewalk")
        timestamp          = sidewalk.get("Timestamp")
        data               = uplink.get("PayloadData")

        # ---------------------------------------------
        # Transform Base64-encoded Sidewalk payload into a hex string
        # ---------------------------------------------
        data_bytes   = data.encode('ascii')
        decoded_data = base64.b64decode(data_bytes).decode('ascii')
        print('decoded_data:', decoded_data)

        # ---------------------------------------------
        # Republish the original uplink to the MQTT broker
        # ---------------------------------------------
        mqtt_client = boto3.client('iot-data')
        mqtt_client.publish(
            topic=f'{geolocation_mqtt_topic_base}/{wireless_device_id}/raw',
            qos=1,
            payload=json.dumps(uplink)
        )

        # ---------------------------------------------
        # Process data fragments assembly
        # ---------------------------------------------
        fragments_assembler = FragmentsAssembler(wireless_device_id)
        full_uplink_payload = fragments_assembler.process_fragment(decoded_data, timestamp)
        if full_uplink_payload:
            # ---------------------------------------------
            # Parse TLV records inside the payload
            # ---------------------------------------------
            parsed_tlv_data = TlvParser.parse(full_uplink_payload)

            if parsed_tlv_data:
                print(f'TLV data from device: {parsed_tlv_data}')

                output_json = {}

                # Process WiFi scans
                if 'wifi_scans' in parsed_tlv_data:
                    # Map each WiFi scan to a WiFiAccessPoint
                    wifi_scans = list(map(lambda scan: {
                        'MacAddress': scan['mac_address'],
                        'Rss': scan['rssi']
                    }, parsed_tlv_data['wifi_scans']))

                    # Resolve position using WiFi scans
                    resolved_wifi_position = try_resolve_wifi_position(wifi_scans)
                    if resolved_wifi_position is not None:
                        # Successfully resolved WiFi position
                        print(f'WiFi position successfully resolved: {resolved_wifi_position}')
                        output_json['wifi_position'] = resolved_wifi_position
                    else:
                        print(f'WiFi position cannot be resolved')
                else:
                    print(f'No WiFi scan results in uplink')

                # Process LR11xx GNSS scans (NAV3 messages)
                if 'nav3_messages' in parsed_tlv_data:
                    output_json['nav3_gnss_position']=[]

                    # Resolve every NAV3 record
                    for item in parsed_tlv_data['nav3_messages']:
                        # Map each GNSS scan to a PositionEstimate
                        gnss_scan = {
                            'Payload': item['nav3_payload']
                        }

                        # Scan timestamp is taken into account if is better than 16 seconds
                        if item['time_accuracy'] < __SEMTECH_GNSS_RESOLVER_CUTOFF_TIME_ACCURACY:
                            gnss_scan['CaptureTime'] = float(item['timestamp'])
                            gnss_scan['CaptureTimeAccuracy'] = float(item['time_accuracy']) / 1000.0  # Convert ms to seconds

                        # Add aiding position if available
                        if 'assist_position' in item:
                            gnss_scan['AssistPosition'] = item['assist_position']
                        else:
                            # Enrich GNSS scan data with assisted position derived from WiFi if available
                            if 'wifi_position' in output_json and 'coordinates' in output_json['wifi_position'] and len(output_json['wifi_position']['coordinates']) >= 2:
                                gnss_scan['AssistPosition'] = [output_json['wifi_position']['coordinates'][1], output_json['wifi_position']['coordinates'][0]]

                        # Provide debug output
                        print(f'gnss_scan_3D: {gnss_scan}')

                        # Resolve position using NAV3 data and 3D solver
                        resolved_gnss_position = try_resolve_nav3_gnss_position(gnss_scan)

                        if resolved_gnss_position is not None:
                            # Successfully resolved GNSS position using 3D solver
                            print(f'GNSS position successfully resolved using 3D solver: {resolved_gnss_position}')
                            output_json['nav3_gnss_position'].append(resolved_gnss_position)
                        else:
                            # Failed to resolve GNSS position, fall back to 2D solver and try again
                            print(f'Failed to resolve GNSS position using 3D solver, falling back to 2D solver')
                            gnss_scan['Use2DSolver'] = True

                            if 'AssistAltitude' not in gnss_scan:
                                if 'wifi' in output_json and 'coordinates' in output_json['wifi_position'] and len(resolved_position['wifi_position']['coordinates']) == 3:
                                    # Add assist altitude from WiFi scan
                                    gnss_scan['AssistAltitude'] = float(output_json['wifi_position']['coordinates'][2])
                                else:
                                    # Assist altitude value is not available, use substitution value
                                    gnss_scan['AssistAltitude'] = 0.0

                            # Provide debug output
                            print(f'gnss_scan_2D: {gnss_scan}')

                            # Resolve position using NAV3 data and 2D solver
                            resolved_gnss_position = try_resolve_nav3_gnss_position(gnss_scan)
                            if resolved_gnss_position is not None:
                                # Successfully resolved GNSS position using 3D solver
                                print(f'GNSS position successfully resolved using 2D solver: {resolved_gnss_position}')
                                output_json['nav3_gnss_position'].append(resolved_gnss_position)
                            else:
                                print(f'GNSS position cannot be resolved for the given NAV3 payload. Probably sky conditions are suboptimal')

                    # Update IoT Wireless device position
                    if len(output_json['nav3_gnss_position']) > 0:
                        # Select the most accurate GNSS/NAV3-based position
                        most_accurate_position = None
                        for position in output_json['nav3_gnss_position']:
                            if most_accurate_position is None:
                                # Initialize most_accurate_position with the first available position data
                                most_accurate_position = position
                            elif not ('properties' in most_accurate_position and 'horizontalAccuracy' in most_accurate_position['properties']) and ('properties' in position and 'horizontalAccuracy' in position['properties']):
                                # The stored most_accurate_position has no accuracy data (typically a sign of poor accuracy) and current position has accuracy info - update most_accurate_position with current position record
                                most_accurate_position = position
                            elif ('properties' in most_accurate_position and 'horizontalAccuracy' in most_accurate_position['properties']) and ('properties' in position and 'horizontalAccuracy' in position['properties']):
                                # Current position is more precise than the stored most_accurate_position - update most_accurate_position with current position record
                                if position['properties']['horizontalAccuracy'] < most_accurate_position['properties']['horizontalAccuracy']:
                                    most_accurate_position = position

                        if most_accurate_position is not None:
                            # Update IoT Wireless device position with the most accurate NAV3-based position we have
                            update_iot_wireless_device_position(wireless_device_id, most_accurate_position)
                    elif 'wifi_position' in output_json:
                        # Fallback to Wi-Fi-based position
                        update_iot_wireless_device_position(wireless_device_id, output_json['wifi_position'])

                    # Clean up the resolved position object
                    if len(output_json['nav3_gnss_position']) == 0:
                        del output_json['nav3_gnss_position']

                # Process resolved GNSS position reported by Teseo
                if 'position' in parsed_tlv_data:
                    output_json['position'] = {
                        'coordinates': [
                            parsed_tlv_data['position']['longitude'],
                            parsed_tlv_data['position']['latitude'],
                            parsed_tlv_data['position']['elevation'],
                        ],
                        'type': 'Point',
                        'properties': {
                            'timestamp':  # Format as ISO 8601 without fractional seconds
                                parsed_tlv_data['position']['timestamp'].strftime('%Y-%m-%dT%H:%M:%SZ') if parsed_tlv_data['position']['timestamp'].microsecond == 0
                                else parsed_tlv_data['position']['timestamp'].strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
                            # Format as ISO 8601 without fractional seconds
                        }
                    }

                    if parsed_tlv_data['position']['vertical_accuracy'] != 0.0 and not math.isnan(parsed_tlv_data['position']['vertical_accuracy']):
                        output_json['position']['properties']['verticalAccuracy'] = parsed_tlv_data['position']['vertical_accuracy']
                        output_json['position']['properties']['verticalConfidenceLevel'] = 0.68  # Fixed since 1-sigma error is always used for accuracy value

                    if parsed_tlv_data['position']['horizontal_accuracy'] != 0.0 and not math.isnan(parsed_tlv_data['position']['horizontal_accuracy']):
                        output_json['position']['properties']['horizontalAccuracy'] = parsed_tlv_data['position']['horizontal_accuracy']
                        output_json['position']['properties']['horizontalConfidenceLevel'] = 0.68  # Fixed since 1-sigma error is always used for accuracy value

                    # Update IoT Wireless device position
                    update_iot_wireless_device_position(wireless_device_id, output_json['position'])

                # Process MCU temperature value if it is included into the event payload
                if 'mcu_temperature' in parsed_tlv_data:
                    output_json['measurements'] = {
                        'mcu_temperature': parsed_tlv_data['mcu_temperature'],
                    }

                # Publish resolved position and telemetry to the MQTT broker if it is not empty
                if output_json:
                    print(f'Final resolved position and telemetry: {output_json}')
                    mqtt_client.publish(
                        topic=f'{geolocation_mqtt_topic_base}/{wireless_device_id}/telemetry',
                        qos=1,
                        payload=json.dumps(output_json)
                    )
                else:
                    print(f'No usable data discovered in the uplink')

        return {
            'statusCode': 200,
            'body': json.dumps('Sidewalk Geolocation processing done')
        }

    except Exception:
        print(f'Unexpected error occurred: {traceback.format_exc()}')
        return {
            'statusCode': 500,
            'body': json.dumps('Unexpected error occurred: ' + traceback.format_exc())
        }
