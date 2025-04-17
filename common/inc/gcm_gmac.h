#ifndef _GCM_GMAC_H
#define _GCM_GMAC_H

#include "mbedtls/gcm.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Initialize the security module
 * 
 * @return int 0 on success, non-zero on failure
 */
int security_init(void);

/**
 * @brief Cleanup the security module
 */
void security_cleanup(void);

/**
 * @brief Encrypt firmware block with AES-128-GCM
 * 
 * @param key The AES key (16 bytes for AES-128)
 * @param iv Initialization vector/nonce
 * @param iv_len Length of the IV/nonce
 * @param aad Additional authenticated data
 * @param aad_len Length of the AAD
 * @param input Input data to encrypt
 * @param input_len Length of the input data
 * @param output Buffer to store the encrypted data
 * @param tag Buffer to store the authentication tag
 * @param tag_len Length of the authentication tag
 * @return int 0 on success, non-zero on failure
 */
int encrypt_firmware(const uint8_t* key,
                    const uint8_t* iv, size_t iv_len,
                    const uint8_t* aad, size_t aad_len,
                    const uint8_t* input, size_t input_len,
                    uint8_t* output,
                    uint8_t* tag, size_t tag_len);

/**
 * @brief Decrypt and authenticate firmware block
 * 
 * @param key The AES key (16 bytes for AES-128)
 * @param iv Initialization vector/nonce
 * @param iv_len Length of the IV/nonce
 * @param aad Additional authenticated data
 * @param aad_len Length of the AAD
 * @param input Input data to decrypt
 * @param input_len Length of the input data
 * @param output Buffer to store the decrypted data
 * @param tag Authentication tag
 * @param tag_len Length of the authentication tag
 * @return int 0 on success, non-zero on failure
 */
int decrypt_firmware(const uint8_t* key,
                    const uint8_t* iv, size_t iv_len,
                    const uint8_t* aad, size_t aad_len,
                    const uint8_t* input, size_t input_len,
                    uint8_t* output,
                    const uint8_t* tag, size_t tag_len);

/**
 * @brief Verify firmware authenticity with GMAC
 * 
 * @param key The AES key (16 bytes for AES-128)
 * @param iv Initialization vector/nonce
 * @param iv_len Length of the IV/nonce
 * @param data Data to authenticate
 * @param data_len Length of the data
 * @param tag Authentication tag
 * @param tag_len Length of the authentication tag
 * @return int 0 on success, non-zero on failure
 */
int verify_firmware_auth(const uint8_t* key,
                       const uint8_t* iv, size_t iv_len,
                       const uint8_t* data, size_t data_len,
                       const uint8_t* tag, size_t tag_len);

#endif /* _GCM_GMAC_H */