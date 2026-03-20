#
##############################################################################
# file:    run.py
# brief:   Populates image metadata (signature, CRC) and creates an OTA image
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


import argparse
import base64
import binascii
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile

from nacl.signing import SigningKey


SID_AIM_IFC_IMAGE_VALIDITY_MAGIC_WORD: int = 0x94448A29  # This magic word is expected to be put into firmware image footer by the compiler. It is used a consistency check between the linker and this script

# OTA footer layout definitions - this shall match sid_aim_ota_footer_t in the C code
IMAGE_FOOTER_SIGNATURE_SIZE_BYTES    : int = 64
IMAGE_FOOTER_SIGNATURE_OFFSET_BYTES  : int = 0
IMAGE_FOOTER_MAGIC_WORD_SIZE_BYTES   : int = 4
IMAGE_FOOTER_MAGIC_WORD_OFFSET_BYTES : int = IMAGE_FOOTER_SIGNATURE_OFFSET_BYTES + IMAGE_FOOTER_SIGNATURE_SIZE_BYTES
IMAGE_FOOTER_CRC_SIZE_BYTES          : int = 4
IMAGE_FOOTER_CRC_OFFSET_BYTES        : int = IMAGE_FOOTER_MAGIC_WORD_OFFSET_BYTES + IMAGE_FOOTER_MAGIC_WORD_SIZE_BYTES
IMAGE_FOOTER_SIZE                    : int = IMAGE_FOOTER_SIGNATURE_SIZE_BYTES + IMAGE_FOOTER_MAGIC_WORD_SIZE_BYTES + IMAGE_FOOTER_CRC_SIZE_BYTES

IMAGE_FOOTER_SECTION_NAME_DEFAULT          = '.ota_footer'  # This is the name of the section containing the footer. It shall match the .ld linker script file content

APP_ENTRY_SECTION_NAME_DEFAULT             = '.isr_vector'  # This is the name of the section containing app entry point (ISR vector table). It shall match the .ld linker script file content

STAGING_AREA_SECTION_NAME_DEFAULT          = '.staging_area'

OBJCOPY_BIN                                = 'arm-none-eabi-objcopy'
OBJDUMP_BIN                                = 'arm-none-eabi-objdump'

OTA_IMAGE_SUFFIX                           = 'ota_image'

APP_SIGNING_KEY_NAME_DEFAULT               = 'application_signing_key.pem'  # The default file name to load/store key pair if the user has not specified the name explicitly


class BinFileEmptyError(Exception):
    pass


class BinFileInsufficientDataError(Exception):
    pass


class HexFileEmptyError(Exception):
    pass


class HexFileInsufficientDataError(Exception):
    pass


class OtaFooterNotFoundError(Exception):
    pass


class OtaFooterInvalidSizeError(Exception):
    pass


class OtaFooterInvalidAlignmentError(Exception):
    pass


class OtaFooterInvalidMagicWordError(Exception):
    pass


class KeyFileValidationError(Exception):
    pass


def normalize_path(path):
    """Convert path to absolute and normalize slashes (always use forward slashes for GCC)."""
    return os.path.abspath(path).replace('\\', '/')


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
                        # Gap in the data detected, insert padding
                        data += b'\xFF' * (full_addr - last_processed_addr)

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


def get_section_info_from_map_file(map_file_path: str, section_name: str) -> tuple[int, int]:
    try:
        section_start_addr = None
        section_size = None

        with open(map_file_path, 'r') as f:
            section_pattern = re.compile(r'^\s*'+section_name+r'\s+(0x08[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)')  # example: " .ota_footer    0x080d4588       0x48"

            for line in f:
                section_match = section_pattern.search(line)
                if section_match:
                    section_start_addr = int(section_match.group(1), 16)
                    section_size       = int(section_match.group(2), 16)
                    break

            if not section_start_addr:
                raise OtaFooterNotFoundError(f'Can\'t find {section_name} section in .map file')

            if not section_size:
                raise OtaFooterNotFoundError(f'Can\'t determine the size of {section_name} section from .map file')

            return section_start_addr, section_size

    except Exception as e:
        print(f'[ERROR] Failed to read .map file: {e}', file=sys.stderr)
        sys.exit(-2)


def get_section_info_from_elf_file(elf_file_path: str, section_name: str) -> tuple[int, int]:
    try:
        section_start_addr = None
        section_size = None

        # Run objdump -h to list section headers
        result = subprocess.run(
            [OBJDUMP_BIN, '-h', elf_file_path],
            capture_output=True,
            text=True,
            check=True
        )

        # Parse the output line by line
        for line in result.stdout.splitlines():
            match = re.match(r'\s*\d+\s+(\S+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)', line)  # Example line: " 13  .ota_footer   00000048  080D4588  080D4588  00004588  2**2"
            if match:
                name, size_hex, vma_hex = match.groups()
                if name == section_name:
                    section_start_addr = int(vma_hex, 16)
                    section_size       = int(size_hex, 16)
                    return section_start_addr, section_size

        raise OtaFooterNotFoundError(f'Can\'t find {section_name} section in .elf file')

    except Exception as e:
        print(f'[ERROR] Failed to read .elf file: {e}', file=sys.stderr)
        sys.exit(-2)


def build_footer_from_binary_data(binary_image: bytearray, ota_footer_offset: int, signing_key: SigningKey, verbose: bool=False) -> bytearray:
    if len(binary_image) < ota_footer_offset + IMAGE_FOOTER_SIZE:
        raise HexFileInsufficientDataError(f'Input binary data is not long enough to include firmware image footer')

    # Extract footer data
    ota_footer_bytes = binary_image[ota_footer_offset:(ota_footer_offset + IMAGE_FOOTER_SIZE)]

    # Grab the magic word in the footer
    ota_footer_magic_word = int.from_bytes(ota_footer_bytes[IMAGE_FOOTER_MAGIC_WORD_OFFSET_BYTES:(IMAGE_FOOTER_MAGIC_WORD_OFFSET_BYTES + IMAGE_FOOTER_MAGIC_WORD_SIZE_BYTES)], byteorder='little', signed=False)
    # Check the magic word is valid
    if ota_footer_magic_word != SID_AIM_IFC_IMAGE_VALIDITY_MAGIC_WORD:
        raise OtaFooterInvalidMagicWordError(f'Invalid magic word in the firmware image footer: 0x{ota_footer_magic_word:08X}')
    if verbose:
        print(f'[INFO] Magic word in the firmware image footer area: {ota_footer_magic_word:08X}')

    # Compute image signature and put it into the footer
    print(f'[INFO] Calculating firmware image signature...')
    # Compute SHA-512 of the image first
    eddsa_covered_data = binary_image[0:ota_footer_offset]
    fw_image_digest = hashlib.sha512(eddsa_covered_data).digest()
    # Now sign the SHA-512 digest
    fw_image_signature = bytearray(signing_key.sign(fw_image_digest).signature)

    # Store calculated signature to the footer
    ota_footer_bytes[IMAGE_FOOTER_SIGNATURE_OFFSET_BYTES:IMAGE_FOOTER_SIGNATURE_OFFSET_BYTES + IMAGE_FOOTER_SIGNATURE_SIZE_BYTES] = fw_image_signature
    # Also write back to the original binary data because this data will be used for CRC calculation and signature is covered by CRC as well
    binary_image[ota_footer_offset + IMAGE_FOOTER_SIGNATURE_OFFSET_BYTES:ota_footer_offset + IMAGE_FOOTER_SIGNATURE_OFFSET_BYTES + IMAGE_FOOTER_SIGNATURE_SIZE_BYTES] = fw_image_signature

    # Compute image CRC and put it into the footer
    print(f'[INFO] Calculating firmware image CRC...')
    crc_covered_data = binary_image[0:(ota_footer_offset + IMAGE_FOOTER_CRC_OFFSET_BYTES)]
    fw_image_crc = ieee_802_3_crc32(crc_covered_data)
    print(f'[INFO] Calculated firmware image CRC: 0x{fw_image_crc:08X}')

    # Store calculated CRC to the footer
    ota_footer_bytes[IMAGE_FOOTER_CRC_OFFSET_BYTES:IMAGE_FOOTER_CRC_OFFSET_BYTES+4] = fw_image_crc.to_bytes(4, byteorder='little')

    return ota_footer_bytes


def build_footer_from_bin_file(bin_file_path: str, fw_start_address: int, ota_footer_start_address: int, signing_key: SigningKey, verbose: bool=False) -> bytearray:
    print('[INFO] Reading .bin file...')

    try:
        with open(bin_file_path, 'rb') as f:
            fw_image_bytes = bytearray(f.read())
            if not len(fw_image_bytes):
                raise BinFileEmptyError(f'.bin file does not contain any data starting at address 0x{fw_start_address:08X}')

            footer_offset = ota_footer_start_address - fw_start_address

            if len(fw_image_bytes) < footer_offset + IMAGE_FOOTER_SIZE:
                raise BinFileInsufficientDataError(f'.bin file does not contain enough data to include firmware image footer')

        # Process binary data
        ota_footer_binary_image = build_footer_from_binary_data(fw_image_bytes, footer_offset, signing_key, verbose)

        return ota_footer_binary_image

    except Exception as e:
        print(f'[ERROR] Failed to read .bin file: {e}', file=sys.stderr)
        sys.exit(-3)


def build_footer_from_hex_file(hex_file_path: str, fw_start_address: int, ota_footer_start_address: int, signing_key: SigningKey, verbose: bool=False) -> bytearray:
    print('[INFO] Reading .hex file...')

    try:
        fw_image_bytes = load_intel_hex(hex_file_path, fw_start_address)
        if not len(fw_image_bytes):
            raise HexFileEmptyError(f'.hex file does not contain any data starting at address 0x{fw_start_address:08X}')

        footer_offset = ota_footer_start_address - fw_start_address

        if len(fw_image_bytes) < footer_offset + IMAGE_FOOTER_SIZE:
            raise HexFileInsufficientDataError(f'.hex file does not contain enough data to include firmware image footer ({len(fw_image_bytes)} vs {footer_offset + IMAGE_FOOTER_SIZE})')

        # Process binary data
        ota_footer_binary_image = build_footer_from_binary_data(fw_image_bytes, footer_offset, signing_key, verbose)

        return ota_footer_binary_image

    except Exception as e:
        print(f'[ERROR] Failed to read .hex file: {e}', file=sys.stderr)
        sys.exit(-3)


def build_footer_from_elf_file(elf_file_path: str, fw_start_address: int, ota_footer_start_address: int, signing_key: SigningKey, verbose: bool=False) -> bytearray:
    print('[INFO] Reading .elf file...')

    # Create a temporary filename for binary data output
    tmp_bin_fd, tmp_bin_path = tempfile.mkstemp(suffix=".bin")
    os.close(tmp_bin_fd)  # close the descriptor so objcopy can write to it

    try:
        # Run objcopy to convert the flashing image into binary
        subprocess.run(
            [OBJCOPY_BIN, '-O', 'binary', elf_file_path, tmp_bin_path],
            text=True,
            check=True
        )

        # Read the extracted binary into a bytearray
        with open(tmp_bin_path, 'rb') as f:
            fw_image_bytes = bytearray(f.read())

            if not len(fw_image_bytes):
                raise HexFileEmptyError(f'.elf file does not contain any data starting at address 0x{fw_start_address:08X}')

            footer_offset = ota_footer_start_address - fw_start_address

            if len(fw_image_bytes) < footer_offset + IMAGE_FOOTER_SIZE:
                raise HexFileInsufficientDataError(f'.elf file does not contain enough data to include firmware image footer')

            # Process binary data
            ota_footer_binary_image = build_footer_from_binary_data(fw_image_bytes, footer_offset, signing_key, verbose)

            return ota_footer_binary_image

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] objcopy failed: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f'[ERROR] Failed to read .elf file: {e}', file=sys.stderr)
        sys.exit(-3)

    finally:
        # Clean up temporary file
        os.remove(tmp_bin_path)


def convert_elf_to_hex(elf_file_path: str, hex_file_path: str):
    print(f'[INFO] Updating .hex file...')
    try:
        # Remove existing .hex file
        if os.path.exists(hex_file_path):
            os.remove(hex_file_path)

        subprocess.run(
            [OBJCOPY_BIN, '-O', 'ihex',
                '-S',
                '--gap-fill', '0xFF',
                elf_file_path,
                hex_file_path],
            check=True
        )

    except subprocess.CalledProcessError as e:
        print(f"[ERROR] objcopy failed: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f'[ERROR] Failed to convert .elf file to a .hex: {e}', file=sys.stderr)
        sys.exit(-4)


def store_footer_to_elf(elf_file_path: str, ota_footer_section_name: str, ota_footer_binary_image: bytearray):
    print(f'[INFO] Updating .elf file...')
    tmp_bin_path = None
    tmp_elf_path = None
    try:
        # Create temp .bin for new section
        tmp_bin_fd, tmp_bin_path = tempfile.mkstemp(suffix=".bin")
        os.close(tmp_bin_fd)
        with open(tmp_bin_path, "wb") as f:
            f.write(ota_footer_binary_image)

        # Use temp ELF to safely replace original
        tmp_elf_fd, tmp_elf_path = tempfile.mkstemp(suffix=".elf")
        os.close(tmp_elf_fd)

        subprocess.run(
            [OBJCOPY_BIN, f'--update-section', f'{ota_footer_section_name}={tmp_bin_path}',
                elf_file_path,
                tmp_elf_path],
            check=True
        )

        # Replace original ELF atomically
        shutil.move(tmp_elf_path, elf_file_path)

    except Exception as e:
        print(f'[ERROR] Failed to update .elf file: {e}', file=sys.stderr)
        sys.exit(-4)

    finally:
        if tmp_bin_path and os.path.exists(tmp_bin_path):
            os.remove(tmp_bin_path)
        if tmp_elf_path and os.path.exists(tmp_elf_path):
            os.remove(tmp_elf_path)


def store_footer_to_bin(bin_file_path: str, fw_start_address: int, ota_footer_start_address: int, ota_footer_binary_image: bytearray):
    print(f'[INFO] Updating .hex file...')
    try:
        # Read in the original contents
        with open(bin_file_path, 'rb') as f:
            fw_image_bytes = bytearray(f.read())

        footer_offset = ota_footer_start_address - fw_start_address

        if len(fw_image_bytes) < footer_offset + IMAGE_FOOTER_SIZE:
            raise BinFileInsufficientDataError(f'.bin file does not contain enough data to include firmware image footer')

        if len(ota_footer_binary_image) != IMAGE_FOOTER_SIZE:
            raise OtaFooterInvalidSizeError(f'Provided firmware footer image has wrong size')

        # Replace the original footer data with the updated image
        fw_image_bytes[footer_offset:footer_offset + IMAGE_FOOTER_SIZE] = ota_footer_binary_image

        # Rewrite the .bin file
        with open(bin_file_path, 'wb') as f:
            f.write(fw_image_bytes)

    except Exception as e:
        print(f'[ERROR] Failed to update .bin file: {e}', file=sys.stderr)
        sys.exit(-4)


def store_footer_to_hex(hex_file_path: str, fw_start_address: int, ota_footer_start_address: int, ota_footer_binary_image: bytearray):
    print(f'[INFO] Updating .hex file...')
    tmp_bin_path = None
    try:
        fw_image_bytes = load_intel_hex(hex_file_path, fw_start_address)
        if not len(fw_image_bytes):
            raise HexFileEmptyError(f'.hex file does not contain any data starting at address 0x{fw_start_address:08X}')

        footer_offset = ota_footer_start_address - fw_start_address

        if len(fw_image_bytes) < footer_offset + IMAGE_FOOTER_SIZE:
            raise HexFileInsufficientDataError(f'.hex file does not contain enough data to include firmware image footer')

        if len(ota_footer_binary_image) != IMAGE_FOOTER_SIZE:
            raise OtaFooterInvalidSizeError(f'Provided firmware footer image has wrong size')

        # Replace the original footer data with the updated image
        fw_image_bytes[footer_offset:footer_offset + IMAGE_FOOTER_SIZE] = ota_footer_binary_image

        # Store temporary .bin file
        tmp_bin_fd, tmp_bin_path = tempfile.mkstemp(suffix=".bin")
        os.close(tmp_bin_fd)
        with open(tmp_bin_path, "wb") as f:
            f.write(fw_image_bytes)

        # Convert .bin to .hex
        subprocess.run(
            [OBJCOPY_BIN,
                '-I', 'binary',
                '-O', 'ihex',
                '--change-addresses', f'0x{fw_start_address:08X}',
                tmp_bin_path,
                hex_file_path
            ],
            check=True
        )

    except Exception as e:
        print(f'[ERROR] Failed to update .hex file: {e}', file=sys.stderr)
        sys.exit(-4)

    finally:
        if tmp_bin_path and os.path.exists(tmp_bin_path):
            os.remove(tmp_bin_path)


def create_ota_image_from_bin(bin_file_path: str, ota_storage_start_address: int|None=None):
    print('[INFO] Creating OTA image from .bin input')
    try:
        base, _ = os.path.splitext(bin_file_path)
        ota_bin_file_path = f'{base}_{OTA_IMAGE_SUFFIX}.bin'
        ota_hex_file_path = f'{base}_{OTA_IMAGE_SUFFIX}.hex'

        # Simply copy the produced .bin
        shutil.copy(bin_file_path, ota_bin_file_path)

        if ota_storage_start_address:
            # Generate .hex for flashing into the staging area - this mainly targets tests and debugging by allowing flashing an OTA image directly to the MCU
            subprocess.run(
                [OBJCOPY_BIN,
                    '-I', 'binary',
                    '-O', 'ihex',
                    '--change-addresses', f'0x{ota_storage_start_address:08X}',
                    ota_bin_file_path,
                    ota_hex_file_path
                ],
                check=True
            )

    except Exception as e:
        print(f'[ERROR] Failed to convert .hex file to .bin: {e}', file=sys.stderr)
        sys.exit(-4)


def create_ota_image_from_hex(hex_file_path: str, ota_storage_start_address: int|None=None):
    print('[INFO] Creating OTA image from .hex input')
    try:
        base, _ = os.path.splitext(hex_file_path)
        ota_bin_file_path = f'{base}_{OTA_IMAGE_SUFFIX}.bin'
        ota_hex_file_path = f'{base}_{OTA_IMAGE_SUFFIX}.hex'

        # Convert .hex to .bin
        subprocess.run(
            [OBJCOPY_BIN,
                '-I', 'ihex',
                '-O', 'binary',
                '--gap-fill', '0xFF',
                hex_file_path,
                ota_bin_file_path
            ],
            check=True
        )

        if ota_storage_start_address:
            # Generate .hex for flashing into the staging area - this mainly targets tests and debugging by allowing flashing an OTA image directly to the MCU
            subprocess.run(
                [OBJCOPY_BIN,
                    '-I', 'binary',
                    '-O', 'ihex',
                    '--change-addresses', f'0x{ota_storage_start_address:08X}',
                    ota_bin_file_path,
                    ota_hex_file_path
                ],
                check=True
            )

    except Exception as e:
        print(f'[ERROR] Failed to convert .hex file to .bin: {e}', file=sys.stderr)
        sys.exit(-4)


def create_ota_image_from_elf(elf_file_path: str, ota_storage_start_address: int|None=None):
    print('[INFO] Creating OTA image from .elf input')
    try:
        base, _ = os.path.splitext(elf_file_path)
        ota_bin_file_path = f'{base}_{OTA_IMAGE_SUFFIX}.bin'
        ota_hex_file_path = f'{base}_{OTA_IMAGE_SUFFIX}.hex'

        # Convert .elf to .bin
        subprocess.run(
            [OBJCOPY_BIN,
                '-O', 'binary',
                '-S'
                '--gap-fill', '0xFF',
                elf_file_path,
                ota_bin_file_path
            ],
            check=True
        )

        if ota_storage_start_address:
            # Generate .hex for flashing into the staging area - this mainly targets tests and debugging by allowing flashing an OTA image directly to the MCU
            subprocess.run(
                [OBJCOPY_BIN,
                    '-I', 'binary',
                    '-O', 'ihex',
                    '--change-addresses', f'0x{ota_storage_start_address:08X}',
                    ota_bin_file_path,
                    ota_hex_file_path
                ],
                check=True
            )

    except Exception as e:
        print(f'[ERROR] Failed to convert .elf file to .bin: {e}', file=sys.stderr)
        sys.exit(-4)


def load_signing_key(key_file_path: str, auto_create: bool=False, verbose: bool=False) -> SigningKey:
    # Check if the key file exists
    if not os.path.exists(key_file_path):
        # Key file does not exists. Create a new key if auto-create is enabled or report an unrecoverable error if not
        if not auto_create:
            raise FileNotFoundError(f'[ERROR] Key file {key_file_path} does not exist')

        # Generate new Ed25519 key pair
        sk = SigningKey.generate()

        # Save to PEM-style text file
        store_key_to_file(sk, key_file_path)

        print(f'[INFO] Generated new signing key')
    else:
        # Key file exists, try to load the key from it
        with open(key_file_path, 'r', encoding='utf-8') as f:
            raw_key_data = f.read()

            # Extract private key
            try:
                private_key_b64   = raw_key_data.split('-----BEGIN ED25519 PRIVATE KEY-----\n')[1].split('\n-----END ED25519 PRIVATE KEY-----')[0]
                private_key_bytes = base64.b64decode(private_key_b64, validate=True)
                if verbose:
                    print(f'[INFO] Loaded private key from the key file')
            except IndexError:
                raise KeyFileValidationError('No private key found in the provided key file')
            except (binascii.Error, ValueError) as e:
                raise KeyFileValidationError(f'Private key has invalid format in the provided key file: {e}')

            # Extract public key
            try:
                public_key_b64   = raw_key_data.split('-----BEGIN ED25519 PUBLIC KEY-----\n')[1].split('\n-----END ED25519 PUBLIC KEY-----')[0]
                public_key_bytes = base64.b64decode(public_key_b64, validate=True)
                if verbose:
                    print(f'[INFO] Loaded public key from the key file')
            except IndexError:
                # This is ok, the key file may contain only the private key. The public key can be easily restored from the private key
                public_key_bytes = None
            except (binascii.Error, ValueError) as e:
                raise KeyFileValidationError(f'Public key has invalid format in the provided key file: {e}')

            # Create key object
            sk = SigningKey(private_key_bytes)

            # Cross check the public key if it was included into the key file
            if public_key_bytes is not None:
                pk = sk.verify_key
                if pk.encode() != public_key_bytes:
                    raise KeyFileValidationError('Provided public key does not correlate with the private key. Ensure they belong to the same key pair')

            print(f'[INFO] Loaded existing key from file')

    # Provide back the loaded key
    return sk


def store_key_to_file(sk: SigningKey, file_path: str):
    # Get public key
    pk = sk.verify_key

    # Encode keys to Base64 (standard text-safe encoding)
    private_b64 = base64.b64encode(sk._seed).decode('ascii')     # 32 bytes private key
    public_b64  = base64.b64encode(pk.encode()).decode('ascii')  # 32 bytes public key

    # Save to PEM-style text file
    with open(file_path, 'w') as f:
        f.write('-----BEGIN ED25519 PRIVATE KEY-----\n')
        f.write(private_b64 + '\n')
        f.write('-----END ED25519 PRIVATE KEY-----\n')
        f.write('-----BEGIN ED25519 PUBLIC KEY-----\n')
        f.write(public_b64 + '\n')
        f.write('-----END ED25519 PUBLIC KEY-----\n')


def create_key(args):
    print(f'[INFO] Generating firmware signing key...')
    sys.stdout.flush()

    try:
        working_dir = normalize_path(args.working_dir)
        key_file = normalize_path(os.path.join(working_dir, args.key_file)) if os.path.basename(args.key_file) == args.key_file else normalize_path(args.key_file)

        if args.verbose:
            print(f'[INFO] Working dir: {working_dir}')
            print(f'[INFO] key file   : {key_file}')

        # Check if the key file exists already
        if os.path.exists(key_file) and os.path.isfile(key_file):
            if args.force_override:
                print(f'[INFO] Key file {key_file} exists already and will be overwritten')
            else:
                print(f'[INFO] Key file {key_file} exists already')
                sys.exit(0)

        # Generate Ed25519 key pair
        sk = SigningKey.generate()

        # Save to PEM-style text file
        store_key_to_file(sk, key_file)

    except Exception as e:
        print(f'[ERROR] Failed to convert .elf file to .bin: {e}', file=sys.stderr)
        sys.exit(-4)


def sign_firmware(args):
    print(f'[INFO] Signing firmware image...')
    sys.stdout.flush()

    try:
        working_dir = normalize_path(args.working_dir)
        map_file = normalize_path(os.path.join(working_dir, args.map_file)) if args.map_file else None
        elf_file = normalize_path(os.path.join(working_dir, args.elf_file)) if args.elf_file else None
        hex_file = normalize_path(os.path.join(working_dir, args.hex_file)) if args.hex_file else None
        bin_file = normalize_path(os.path.join(working_dir, args.bin_file)) if args.bin_file else None
        key_file = normalize_path(os.path.join(working_dir, args.key_file)) if os.path.basename(args.key_file) == args.key_file else normalize_path(args.key_file)

        if args.verbose:
            print(f'[INFO] Working dir: {working_dir}')
            print(f'[INFO] .map file  : {map_file}')
            print(f'[INFO] .elf file  : {elf_file}')
            print(f'[INFO] .hex file  : {hex_file}')
            print(f'[INFO] .bin file  : {bin_file}')
            print(f'[INFO] key file   : {key_file}')

        # Check the environment has GCC toolchain in PATH
        objcopy = shutil.which(OBJCOPY_BIN)
        if not objcopy:
            raise FileNotFoundError(f'{OBJCOPY_BIN} not found in PATH')
        objdump = shutil.which(OBJDUMP_BIN)
        if not objdump:
            raise FileNotFoundError(f'{OBJDUMP_BIN} not found in PATH')

        # Read .map or .elf file (whichever is available) to grab the location of the firmware image footer and the app entry point in the output binary
        if map_file:
            app_start_addr, _   = get_section_info_from_map_file(map_file, APP_ENTRY_SECTION_NAME_DEFAULT)
            section_info_method = '.map'
        elif elf_file:
            app_start_addr, _   = get_section_info_from_elf_file(elf_file, APP_ENTRY_SECTION_NAME_DEFAULT)
            section_info_method = '.elf'
        else:
            raise FileNotFoundError(f'Either a .map file or a .elf file has to be specified for the script to be able to detect app entry point')

        if args.verbose:
            print(f'[INFO] Discovered app start location in {section_info_method} file: 0x{app_start_addr:08X}')

        if map_file:
            ota_footer_section_start_addr, ota_footer_section_size = get_section_info_from_map_file(map_file, args.footer_section)
            section_info_method                                    = '.map'
        elif elf_file:
            ota_footer_section_start_addr, ota_footer_section_size = get_section_info_from_elf_file(elf_file, args.footer_section)
            section_info_method                                    = '.elf'
        else:
            raise FileNotFoundError(f'Either a .map file or a .elf file has to be specified for the script to be able to detect OTA footer location')

        if args.verbose:
            print(f'[INFO] Discovered firmware footer section location in {section_info_method} file: 0x{ota_footer_section_start_addr:08X}')
            print(f'[INFO] Discovered firmware footer section length in {section_info_method} file: {ota_footer_section_size}')
        if ota_footer_section_size != IMAGE_FOOTER_SIZE:
            raise OtaFooterInvalidSizeError(f'Invalid firmware footer size specified in {section_info_method} file. Expected: {IMAGE_FOOTER_SIZE}, actual: {ota_footer_section_size}')

        # See if staging area address was provided. If not, try searching in .map and .elf files
        staging_area_start_addr = args.staging_area if args.staging_area else None
        if staging_area_start_addr:
            print(f'[INFO] Staging area location provided: 0x{staging_area_start_addr:08X}')
        else:
            if map_file:
                staging_area_start_addr, _ = get_section_info_from_map_file(map_file, STAGING_AREA_SECTION_NAME_DEFAULT)
                section_info_method        = '.map'
            elif elf_file:
                staging_area_start_addr, _ = get_section_info_from_elf_file(elf_file, STAGING_AREA_SECTION_NAME_DEFAULT)
                section_info_method        = '.elf'

            if staging_area_start_addr:
                print(f'[INFO] Discovered OTA staging area location in {section_info_method} file: 0x{staging_area_start_addr:08X}')

        # Load firmware signing key
        sk = load_signing_key(key_file, auto_create=args.generate_key, verbose=args.verbose)

        # Read-in the output binary, calculate image signature, CRC, and provide back the resulting firmware footer image
        ota_footer_binary_image = None
        if bin_file and os.path.exists(bin_file) and os.path.isfile(bin_file):
            # The easiest way to process the data is to use the binary file
            ota_footer_binary_image = build_footer_from_bin_file(bin_file, app_start_addr, ota_footer_section_start_addr, sk, args.verbose)
        elif hex_file and os.path.exists(hex_file) and os.path.isfile(hex_file):
            # Use .hex if .bin is not available
            ota_footer_binary_image = build_footer_from_hex_file(hex_file, app_start_addr, ota_footer_section_start_addr, sk, args.verbose)
        elif elf_file and os.path.exists(elf_file) and os.path.isfile(elf_file):
            # The most sophisticated approach - use .elf
            ota_footer_binary_image = build_footer_from_elf_file(elf_file, app_start_addr, ota_footer_section_start_addr, sk, args.verbose)
        else:
            raise Exception('Neither .hex nor .elf file specified - no input to parse')

        # Ensure firmware footer image was created successfully
        if not ota_footer_binary_image:
            raise Exception('Unable to generate firmware footer from the provided inputs')

        # Patch .elf file to update the firmware footer
        if elf_file:
            store_footer_to_elf(elf_file, args.footer_section, ota_footer_binary_image)

        # Patch .hex file
        if hex_file:
            if elf_file:
                # Simply convert .elf to .hex
                convert_elf_to_hex(elf_file, hex_file)
            else:
                # .elf is not available, go with patching .hex
                store_footer_to_hex(hex_file, app_start_addr, ota_footer_section_start_addr, ota_footer_binary_image)

        # Create a .bin file that can be used for OTA
        if bin_file:
            create_ota_image_from_bin(bin_file, staging_area_start_addr)
        elif hex_file:
            create_ota_image_from_hex(hex_file, staging_area_start_addr)
        elif elf_file:
            create_ota_image_from_elf(elf_file, staging_area_start_addr)

        # Patch .bin file
        if bin_file:
            store_footer_to_bin(bin_file, app_start_addr, ota_footer_section_start_addr, ota_footer_binary_image)

        # Done
        print('[INFO] Firmware image signed successfully')
    except Exception as e:
        print(f'[ERROR] Firmware signing failed: {e}', file=sys.stderr)
        exit(-1)


def main():
    parser = argparse.ArgumentParser(
        description='Tool for signing firmware images and creating OTA files for STM32 Sidewalk platform'
    )

    subparsers = parser.add_subparsers(dest='command', required=True,
                                       help='Available commands'
    )

    # --- create_key command ---
    parser_create = subparsers.add_parser('create_key', help='Create a new firmware signing key')

    parser_create.add_argument(
        '-w', '--working-dir',
        default=os.getcwd(),
        help='Directory where the output files should go. Current working dir will be used if this parameter is not specified explicitly'
    )

    parser_create.add_argument(
        '-k', '--key-file',
        default=APP_SIGNING_KEY_NAME_DEFAULT,
        help='Name for the key file. This can be an absolute or relative path, or just a file name. In the latter case the working dir specified via the -w parameter is assumed as the location of the file'
    )

    parser_create.add_argument(
        '-f', '--force-override',
        action='store_true',
        default=False,
        help='Override the key file with new data if the file exists. Default behavior: key creation is aborted if output file exists'
    )

    parser_create.set_defaults(func=create_key)

    # --- sign command ---
    parser_sign = subparsers.add_parser('sign', help='Sign a firmware image')

    parser_sign.add_argument(
        '-w', '--working-dir',
        default=os.getcwd(),
        help='Directory where the output files should go. Current working dir will be used if this parameter is not specified explicitly'
    )

    parser_sign.add_argument(
        '-m', '--map-file',
        default=None,
        help='Name of the .map file in the build directory. This file is used to locate firmware image footer in the binary'
    )

    parser_sign.add_argument(
        '-e', '--elf-file',
        default=None,
        help='Name of the .elf file in the build directory. This file will be patched to include valid image signature and CRC'
    )

    parser_sign.add_argument(
        '-x', '--hex-file',
        default=None,
        help='Name of the .hex file in the build directory. This file will be patched to include valid image signature and CRC'
    )

    parser_sign.add_argument(
        '-b', '--bin-file',
        default=None,
        help='Name of the .bin file in the build directory. This file will be patched to include valid image signature and CRC'
    )

    parser_sign.add_argument(
        '-f', '--footer-section',
        default=IMAGE_FOOTER_SECTION_NAME_DEFAULT,
        help='Name of the linker section containing OTA footer. The default value is \".ota_footer\"'
    )

    parser_sign.add_argument(
        '-s', '--staging-area',
        default=None,
        help='Start address of the staging area. This is an optional parameter. If specified, a .hex OTA image will be created in addition to a .bin one'
    )

    parser_sign.add_argument(
        '-k', '--key-file',
        default=APP_SIGNING_KEY_NAME_DEFAULT,
        help='Path to the file storing firmware signing key in .pem format'
    )

    parser_sign.add_argument(
        '-g', '--generate-key',
        action='store_true',
        default=False,
        help='Automatically generate a new key if specified key file does not exist'
    )

    parser_sign.add_argument(
        '-o', '--ota-images',
        action='store_true',
        default=False,
        help='Produce OTA images from signed image. Creates .bin and .hex files that can be flashed to the staging area'
    )

    parser_sign.set_defaults(func=sign_firmware)

    # --- Common arguments ---
    parser.add_argument(
        '-V', '--verbose',
        action='store_true',
        help='Enable verbose output for debugging.'
    )

    # Parse and dispatch to command handler function
    args = parser.parse_args()
    args.func(args)
