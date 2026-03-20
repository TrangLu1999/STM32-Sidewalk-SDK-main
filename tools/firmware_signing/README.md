# Firmware Signing Tool

This Python script is a post-build utility designed for the STMicroelectronics Sidewalk SDK. Its primary role is to populate critical metadata—specifically an **Ed25519 digital signature** and a **CRC32 checksum**—into a dedicated "OTA footer" section of a compiled firmware image.

This process is essential for the Application Install Manager (AIM) bootloader, which verifies this metadata to ensure the firmware is both authentic (signed by a trusted source) and has not been corrupted.

## Features

  * **Key Generation:** Can generate a new Ed25519 private/public key pair in PEM format.
  * **Image Signing:** Injects a cryptographic signature and checksum into the firmware.
  * **File Patching:** Atomically updates multiple build artifacts (`.elf`, `.hex`, `.bin`) with the new metadata.
  * **OTA Image Generation:** Creates a separate `_ota_image.bin` file (and optionally a `.hex` file) suitable for the OTA update process.
  * **Toolchain Integration:** Uses the ARM GCC toolchain (`arm-none-eabi-objcopy` and `arm-none-eabi-objdump`) to read ELF sections and patch files.

-----

## Dependencies

1.  **Python 3.6+**
2.  **Python dependencies:** Used for Ed25519 cryptography. Install it via pip:
    ```sh
    pip install -r requirements.txt
    ```
3.  **STM32CubeIDE:** The script requires the **ARM GCC Toolchain** (`arm-none-eabi-objcopy` and `arm-none-eabi-objdump`) to be accessible in your system's PATH. This toolchain is included with the standard **STM32CubeIDE** installation.

-----

## Security Workflow

The tool's signing process is critical for establishing the firmware's chain of trust. It performs the following steps in a specific order:

1.  **Locate Footer:** It first reads the `.map` or `.elf` file to find the precise memory address and size of the OTA footer section (default: `.ota_footer`).
2.  **Check Magic Word:** It reads the footer in the binary and verifies a "magic word" to ensure the linker script and script are synchronized.
3.  **Calculate Hash:** It computes a **SHA-512** hash of the *entire firmware image*, up to (but not including) the footer.
4.  **Generate Signature:** It signs the SHA-512 hash using the provided **Ed25519** private key. This signature is then written into the `signature` field of the footer.
5.  **Calculate CRC:** It computes an **IEEE 802.3 CRC32** checksum of the *entire firmware image AND the newly added signature*. This final CRC is written into the `crc` field of the footer.

This two-level check ensures both **authenticity** (the Ed25519 signature) and **integrity** (the CRC32 checksum), which are then verified by the AIM bootloader upon every boot.

-----

## File Patching and Output

After generating the new footer, the script patches all provided build artifacts:

  * **`.elf` file:** The `.ota_footer` section in the ELF file is updated with the new binary data (signature + CRC) using `objcopy --update-section`.
  * **`.hex` file:** This file is regenerated from the patched `.elf` file to ensure it's in sync.
  * **`.bin` file:** The script directly patches the binary file at the correct offset with the new footer data.
  * **OTA Image:** A copy of the final binary (e.g., `app_ota_image.bin`) is created. This is the file intended for the Over-the-Air update process.

-----

## Usage

The tool has two main commands: `create_key` and `sign`.

### 1\. `create_key`

This command generates a new Ed25519 key pair in PEM format.

```sh
python firmware_sign.py create_key [options]
```

**Arguments:**

  * `-w`, `--working-dir`: Directory to output the key file (default: current directory).
  * `-k`, `--key-file`: Name for the key file (default: `application_signing_key.pem`).
  * `-f`, `--force-override`: Overwrite the key file if it already exists.

### 2\. `sign`

This command signs the firmware image and patches the build artifacts.

```sh
python firmware_sign.py sign [options]
```

**Arguments:**

  * `-w`, `--working-dir`: Build directory where artifact files are located (default: current directory).
  * `-m`, `--map-file`: Name of the `.map` file (used to locate sections).
  * `-e`, `--elf-file`: Name of the `.elf` file (will be patched).
  * `-x`, `--hex-file`: Name of the `.hex` file (will be patched).
  * `-b`, `--bin-file`: Name of the `.bin` file (will be patched).
  * `-f`, `--footer-section`: Linker section name for the OTA footer (default: `.ota_footer`).
  * `-s`, `--staging-area`: (Optional) Start address of the staging area. If provided, a `.hex` OTA image will also be created.
  * `-k`, `--key-file`: Path to the private key file (default: `application_signing_key.pem`).
  * `-g`, `--generate-key`: Automatically generate a new key if the specified key file does not exist.
  * `-V`, `--verbose`: Enable verbose output.

-----

## Example Workflow

1.  **One-Time Key Generation (if you don't have one):**

    ```sh
    python firmware_sign.py create_key -k my_project_key.pem
    ```

    > **Warning:** Secure this key file and **do not** commit it to your repository.

2.  **Post-Build Signing (in your build process):**

    Assuming your build outputs are in a `build/` directory:

    ```sh
    python firmware_sign.py sign \
        -w ./build \
        -m My_App.map \
        -e My_App.elf \
        -x My_App.hex \
        -b My_App.bin \
        -k ../my_project_key.pem
    ```

3.  **Automatic Key Generation (for CI/CD or development):**

    If you just want the script to create and use a key automatically:

    ```sh
    python firmware_sign.py sign -w ./build -m My_App.map -e My_App.elf -g
    ```

This will create or use `application_signing_key.pem` in the \`build/\` directory.
