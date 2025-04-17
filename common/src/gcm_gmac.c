#include "gcm_gmac.h"

// AES-GCM context
static mbedtls_gcm_context aes_ctx;
static int aes_initialized = 0;

int security_init(void) {
    if (!aes_initialized) {
        mbedtls_gcm_init(&aes_ctx);
        aes_initialized = 1;
    }
    return 0;
}

void security_cleanup(void) {
    if (aes_initialized) {
        mbedtls_gcm_free(&aes_ctx);
        aes_initialized = 0;
    }
}

int encrypt_firmware(const uint8_t* key,
                    const uint8_t* iv, size_t iv_len,
                    const uint8_t* aad, size_t aad_len,
                    const uint8_t* input, size_t input_len,
                    uint8_t* output,
                    uint8_t* tag, size_t tag_len) {
    if (!aes_initialized) {
        security_init();
    }
    
    // Set the key
    if (mbedtls_gcm_setkey(&aes_ctx, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        return -1;
    }
    
    // Perform encryption
    if (mbedtls_gcm_starts(&aes_ctx, MBEDTLS_GCM_ENCRYPT, iv, iv_len, aad, aad_len) != 0) {
        return -1;
    }
    
    if (mbedtls_gcm_update(&aes_ctx, input_len, input, output) != 0) {
        return -1;
    }
    
    if (mbedtls_gcm_finish(&aes_ctx, tag, tag_len) != 0) {
        return -1;
    }
    
    return 0;
}

int decrypt_firmware(const uint8_t* key,
                    const uint8_t* iv, size_t iv_len,
                    const uint8_t* aad, size_t aad_len,
                    const uint8_t* input, size_t input_len,
                    uint8_t* output,
                    const uint8_t* tag, size_t tag_len) {
    if (!aes_initialized) {
        security_init();
    }
    
    // Set the key
    if (mbedtls_gcm_setkey(&aes_ctx, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        return -1;
    }
    
    // Perform decryption
    if (mbedtls_gcm_starts(&aes_ctx, MBEDTLS_GCM_DECRYPT, iv, iv_len, aad, aad_len) != 0) {
        return -1;
    }
    
    if (mbedtls_gcm_update(&aes_ctx, input_len, input, output) != 0) {
        return -1;
    }
    
    uint8_t calculated_tag[16];
    if (mbedtls_gcm_finish(&aes_ctx, calculated_tag, tag_len) != 0) {
        return -1;
    }
    
    // Verify the tag
    if (memcmp(calculated_tag, tag, tag_len) != 0) {
        return -1; // Authentication failed
    }
    
    return 0;
}

int verify_firmware_auth(const uint8_t* key,
                       const uint8_t* iv, size_t iv_len,
                       const uint8_t* data, size_t data_len,
                       const uint8_t* tag, size_t tag_len) {
    uint8_t output_buffer[64]; // Small buffer, not actually used
    
    if (!aes_initialized) {
        security_init();
    }
    
    // Set the key
    if (mbedtls_gcm_setkey(&aes_ctx, MBEDTLS_CIPHER_ID_AES, key, 128) != 0) {
        return -1;
    }
    
    // Compute the GMAC
    if (mbedtls_gcm_starts(&aes_ctx, MBEDTLS_GCM_DECRYPT, iv, iv_len, data, data_len) != 0) {
        return -1;
    }
    
    uint8_t calculated_tag[16];
    if (mbedtls_gcm_finish(&aes_ctx, calculated_tag, tag_len) != 0) {
        return -1;
    }
    
    // Compare the tags
    return (memcmp(calculated_tag, tag, tag_len) == 0) ? 0 : -1;
}