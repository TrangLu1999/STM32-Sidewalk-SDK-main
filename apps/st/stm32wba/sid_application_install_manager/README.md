# Application Install Manager (AIM) for ST Sidewalk SDK

## Overview

This repository contains the advanced Application Install Manager (AIM), a secure bootloader developed for the STMicroelectronics Sidewalk SDK. It provides a secure, fail-safe mechanism for Over-the-Air (OTA) firmware updates.

Its primary function is to verify the integrity and authenticity of new firmware images before installation and to ensure that the device can always recover to a known-good state, even if an update is interrupted by a power failure or the new firmware is faulty.

## Key Features

* **Secure Image Verification:** Validates every firmware image (both the active application and the new update image) using two-level verification:
    1.  **Full-image CRC32 check** to guarantee integrity of the binary and its signature.
    2.  **Ed25519 digital signature check** to ensure authenticity and prevent unauthorized firmware.
* **Atomic & Resumable Updates:** The update process is performed on a block-by-block basis. If a reset or power loss occurs during the update, the AIM will detect the unfinished operation on the next boot and resume it from the last successfully completed block.
* **Robust Rollback Mechanism:** Before installing an update, the AIM creates a complete backup of the currently running, known-good application. If the update process fails, the bootloader can automatically restore the original firmware from this backup, preventing the device from being "bricked."
* **Application Self-Confirmation:** After a new firmware image is successfully installed, it must "confirm" its validity to the AIM (e.g., after it has successfully connected to the network). The application is given a limited number of boot attempts to set this validity marker. If it fails to do so within the limit, the AIM considers the update faulty and automatically enforces a rollback to the previous version.
* **Automatic Recovery Flow:** On every boot, the AIM performs a full system check. If the active application is found to be corrupt (fails verification) or has been rejected, it will automatically attempt to install a valid update from the staging slot. If no valid update is present, it will try to restore the backup image.
* **Secure & Flexible Key Handling:** Includes a flexible mechanism for loading the public verification key. While a default implementation is provided, a template file (`application_verification_key.c.template`) makes it easy to customize the key loading process, for example, by reading the key from a secure, immutable storage like OTP (One-Time Programmable) memory.
* **Optimized Standby LPM Wakeup:** Includes a specialized, assembly-written, high-speed wakeup path from Standby low-power mode (LPM) that preserves SRAM integrity.

---

## Core Functionality

### Boot-up Sequence

On every power-up or reset, the AIM takes control of the device and performs the following sequence:

1.  **Initialize System:** Sets up the system clock and initializes the **CMOX (X-CUBE-CRYPTOLIB)** software crypto library.
2.  **Read Install State:** It checks a dedicated flash area to determine the device's current state.
    * **Case 1: Update/Rollback in Progress?** If the state is `AIMIS_UPDATE_ONGOING` or `AIMIS_ROLLBACK_ONGOING`, it means a previous update was interrupted. The AIM immediately resumes the `install_image` function.
    * **Case 2: Awaiting Confirmation?** If the state is `AIMIS_UPDATE_CONFIRMATION_PENDING`, it checks the application boot counter. If the boot limit has been exceeded without confirmation, it triggers an `enforce_rollback`. Otherwise, it increments the boot counter and proceeds to boot the application.
    * **Case 3: Normal Boot:** If the state is `AIMIS_IDLE` or `AIMIS_FINISHED`, it proceeds to verify the active application.
3.  **Verify Active Application:** It runs the full CRC32 and Ed25519 signature check on the firmware in the active slot.
4.  **Boot or Recover:**
    * **If Valid:** The AIM de-initializes its peripherals, cleans up any sensitive data (like the public key) from RAM, and jumps to the application's entry point.
    * **If Invalid:** It triggers the **Recovery Flow**. It first scans the staging slot for a valid update image. If one is found, it installs it. If not, it scans the backup slot for a valid rollback image and installs that. If neither is available, the device enters an error state.

### Firmware Update Process

The update process is designed to be atomic and fail-safe, using three distinct flash areas (see the full flash layout in the [corresponding section](#flash-layout)):
* **Active Slot:** Contains the currently running, confirmed-good application.
* **Staging Slot:** Receives the new firmware image via OTA.
* **Backup (Rollback) Slot:** Used to store a copy of the Active Slot's application during an update.

When an update is requested:

1.  The AIM boots and verifies the new image in the **Staging Slot**.
2.  If the new image is valid, the AIM sets its internal state to `AIMIS_UPDATE_ONGOING`.
3.  It then iterates through every flash block of the application:
    a.  **Backup:** Copies block `N` from the **Active Slot** to the **Backup Slot** and verifies the copy.
    b.  **Update:** Erases block `N` in the **Active Slot** and copies block `N` from the **Staging Slot** to the **Active Slot**.
    c.  **Verify:** Verifies the new block `N` in the **Active Slot**.
    d.  **Mark Progress:** Updates the install metadata to mark block `N` as processed.
4.  After all blocks are copied, the AIM sets the state to `AIMIS_UPDATE_CONFIRMATION_PENDING` and reboots the device into the new application.

### Low-Power Mode (Standby) Support

This AIM includes a highly optimized boot path to handle wakeups from the **Standby low-power mode (LPM)**.

* **The Challenge:** Standby is the lowest power mode available, but it comes at the cost of not retaining CPU core registers. Consequently, when the MCU wakes up, its execution path is identical to a reset: the Main Stack Pointer (MSP) and Program Counter (PC) are re-initialized from the vector table. This means the AIM is entered first, rather than the user application resuming from its previous state.
* **The Solution:** The AIM intelligently detects this specific scenario.
    1.  It first checks the **reset reason**. If the wakeup event is from Standby, it proceeds.
    2.  It then verifies the **SRAM retention configuration**. A fast wakeup is only possible if at least one SRAM area is configured to retain its data during Standby. Without SRAM retention, no application state could have been saved.
* **Fast-Forward Wakeup:** If both conditions are met (wakeup from Standby + SRAM retention enabled), the AIM **bypasses the normal, time-consuming CRC and Ed25519 signature checks**. It immediately "fast-forwards" and jumps to the user application's entry point.
* **SRAM Integrity:** This entire check-and-jump procedure is implemented in **assembly language**. It is meticulously crafted to use only CPU registers and touch **zero RAM**. This guarantees that the application's data, which was preserved in the retained SRAM, is not modified or corrupted in any way by the AIM, allowing the application to resume its state seamlessly.

## Flash Layout

The device's flash memory is partitioned into five distinct sections to ensure a secure and robust update process:

1.  **AIM Code**
    * This area contains the bootloader code itself. It is constant, non-modifiable, and read-only after programming.

2.  **AIM Update Metadata**
    * This is a dedicated one-page area used exclusively by the AIM.
    * It stores the progress of any ongoing update, manages the state machine (e.g., `AIMIS_UPDATE_ONGOING`, `AIMIS_UPDATE_CONFIRMATION_PENDING`), and tracks boot counters.
    * This area is erased and written by the AIM during update and rollback processes. Direct access by the user application is not intended or supported.

3.  **Active Application Area**
    * This executable section holds the currently running, verified user application.
    * On a normal boot, the AIM performs its checks and then jumps to the entry point within this area.

4.  **Staging/Backup Area**
    * This is a non-executable space used to hold new firmware images (staging) and to back up the old application during an update.
    * Its size is **one page larger** than the Active Application Area. This specific layout is critical for the atomic update-and-backup process:
        * The **backup image** is stored starting from **page 0** of this section.
        * The **new update image** is stored starting from **page 1** of this section.
    * This one-page offset enables a page-by-page swap. For each block `N`:
        1.  Page `N` of the **Active Application Area** (old firmware) is copied to Page `N` of the **Staging/Backup Area**.
        2.  Page `N+1` of the **Staging/Backup Area** (new firmware) is copied to Page `N` of the **Active Application Area**.
    * At the end of this process, the new firmware is fully installed in the Active Application Area, and a complete backup of the previous firmware is now located in the Staging/Backup Area, ready for a potential rollback.

5.  **Non-Volatile Application Data**
    * This area is reserved for the user application to store persistent data, such as the Sidewalk Key-Value storage or the SNVMA (Sequential Non-Volatile Memory Area) for the BLE stack.
    * The AIM **never modifies or accesses** this section. It is preserved "as-is" across all AIM operations, including firmware updates and rollbacks.

---

## Security

> [!WARNING]
> ## WARNING: Do Not Upload Secrets to Your Repository
>
> Never commit or upload your private keys, key pairs, or any other security-sensitive secrets to your Git repository.
>
> This project's `.gitignore` file is already configured to ignore common key file patterns to minimize accidents. However, **you must remain vigilant**.
>
> This is especially critical if you are:
> * Using custom-named key files.
> * Implementing a custom key-loading mechanism.
>
> Always ensure your local secrets are properly excluded from version control.

### Image Verification

Security is enforced by a two-level verification process on every firmware image. This entire process is handled automatically by the **Firmware Signing Tool** (located in the [/tools/firmware_signing](/tools/firmware_signing) folder of the repository).

The signing sequence is as follows:

1.  A **SHA-512** hash is computed for the entire firmware image, from the first byte up to (but not including) the footer.
2.  An **Ed25519** digital signature is generated from this SHA-512 hash using a private key. This signature is then stored in the image footer.
3.  A **CRC32** checksum is calculated, covering *both* the full firmware image and the newly stored digital signature in the footer.
4.  This final CRC32 value is written into the footer.

This method provides robust, two-layer protection: the **digital signature guarantees the firmware's authenticity** (it comes from a trusted source), while the **CRC32 ensures the integrity of both the firmware and its signature**, protecting against data corruption.

The AIM bootloader stores the corresponding public Ed25519 key and uses it to verify this signature and checksum on every boot.

### Public Key Management

The public verification key is the root of trust for all firmware. How it is stored is critical to the device's security.

#### Default Behavior

By default, this project is configured to use a CubeIDE pre-build step. This step executes the Python **Firmware Signing Tool** which automatically:
1.  Generates a new Ed25519 key pair (public and private keys) if one doesn't exist.
2.  Creates a C-source file (`application_verification_key.c`) that contains the public key hard-coded into a `const` array.
3.  This file is then compiled into the AIM bootloader.

> **Warning:** This default method is intended for **development and demonstration purposes only**. Storing the public key in the main flash makes it susceptible to being replaced if an attacker gains physical access.

#### Production Customization

For production applications, you **must** store the public key in an immutable (write-once) or secure, access-controlled memory location, such as the **OTP (One-Time Programmable) area** of the microcontroller.

This project is designed to make this customization easy. The file `application_verification_key.c.template` provides the necessary skeleton.

To use your own secure storage:
1.  Remove the default (and auto-generated) `application_verification_key.c` file from your build.
2.  Copy `application_verification_key.c.template` to a new file named `application_verification_key.c`.
3.  Modify the function `aim_load_application_verification_key()`:
    * Instead of copying from the hard-coded array, add your custom logic to read the key from your secure storage (e.g., OTP registers or an external secure element).
    * The function must allocate a RAM buffer, copy the key into it, and return a pointer to that buffer.
4.  The function `aim_release_application_verification_key()` will be called by the AIM after verification is complete. Its job is to securely wipe the key from RAM and free the buffer, preventing any "Use-After-Free" attacks or memory leaks.

## Disclaimer

This software is provided AS-IS. All rights are reserved by STMicroelectronics. Always perform thorough testing and security validation before using this bootloader in a production environment.