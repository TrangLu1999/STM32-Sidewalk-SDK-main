/**
  ******************************************************************************
  * @file    application_verification_key.h
  * @brief   Inteface for the application verification public key storage
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __SID_STM32_AIM_APPLICATION_VERIFICATION_KEY_H_
#define __SID_STM32_AIM_APPLICATION_VERIFICATION_KEY_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <cmox_crypto.h>

/* Exported types ------------------------------------------------------------*/

typedef uint8_t ed25519_pub_key_t[CMOX_ECC_ED25519_PUBKEY_LEN];

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief User-defined method to load the app verification public key.
 *
 * @note using a pointer here allows flexibility. The actual key can be stored
 *       on the flash, in OTP area, or loaded from an external secure storage
 *
 * @warning Never store your production keys directly in the code
 *
 * @return Pointer to the key material or NULL if a failure occurred
 */
const ed25519_pub_key_t * aim_load_application_verification_key(void);

/**
 * @brief User-defined method to release the app verification public key.
 *        This method can be used to erase RAM keeping the key
 */
void aim_release_application_verification_key(void);

#ifdef __cplusplus
}
#endif

#endif /* __SID_STM32_AIM_APPLICATION_VERIFICATION_KEY_H_ */
