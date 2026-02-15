/*
 * Copyright 2026 suut
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include "crapto1.h"
#include "rfal_nfc.h"

// Platform configuration for easy porting

#define MF1_READER_TYPE_ST25R3916
//#define MF1_READER_TYPE_ST25R95
//#define MF1_EXTRA_DEBUG

typedef ReturnCode MF1ReturnCode;
#define mf1_hex(x, y) hex2Str((x), (y))
#define mf1_printf(fmt, ...) platformLog((fmt), ## __VA_ARGS__)

#if defined(MF1_READER_TYPE_ST25R3916)

# define mf1_encode_parity mf1_encode_parity_st25r3916
# define mf1_decode_parity mf1_decode_parity_st25r3916
# define MF1_ENCODED_BUF_SIZE(sz) ((9 * (sz) + 7) / 8)
# define MF1_ERR(x) RFAL_ERR_ ## x
# define MF1_TXRX_FLAGS ({ \
        /* CRC stuff */ \
        RFAL_TXRX_FLAGS_CRC_TX_MANUAL | \
        RFAL_TXRX_FLAGS_CRC_RX_MANUAL | \
        RFAL_TXRX_FLAGS_CRC_RX_KEEP | \
        /* parity bits */ \
        RFAL_TXRX_FLAGS_PAR_RX_KEEP | \
        RFAL_TXRX_FLAGS_PAR_TX_NONE | \
        /* analog */ \
        RFAL_TXRX_FLAGS_AGC_ON; \
    })

#elif defined(MF1_READER_TYPE_ST25R95)

# define mf1_encode_parity mf1_encode_parity_st25r95
# define mf1_decode_parity mf1_decode_parity_st25r95
# define MF1_ENCODED_BUF_SIZE(sz) (2 * (sz))
# define MF1_ERR(x) ERR_ ## x
# define MF1_TXRX_FLAGS ({ \
        /* CRC stuff */ \
        RFAL_TXRX_FLAGS_CRC_TX_MANUAL | \
        RFAL_TXRX_FLAGS_CRC_RX_KEEP | \
        /* parity bits */ \
        RFAL_TXRX_FLAGS_PAR_RX_KEEP | \
        RFAL_TXRX_FLAGS_PAR_TX_NONE | \
        /* analog */ \
        RFAL_TXRX_FLAGS_AGC_ON; \
    })

#else

# error At least one reader type must be defined

#endif

#define MF1_NUM_MODELS ((size_t)6)

typedef struct {
    union {
        uint32_t id;
        struct {
            uint8_t sak;
            uint16_t atqa;
            uint8_t _padding;
        } __attribute__ ((__packed__));
    };
    const char *short_name;
    const char *name;
    uint8_t uid_size;
    uint16_t capacity;
} mf1_model_t;

typedef enum {
    KEY_A = 0,
    KEY_B = 1
} key_type_t;

extern mf1_model_t mifare_models[MF1_NUM_MODELS];

const mf1_model_t *mf1_identify_model(uint8_t sak, uint16_t atqa);

void mf1_decode_parity_st25r95(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len);
void mf1_encode_parity_st25r95(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len);
void mf1_decode_parity_st25r3916(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len);
void mf1_encode_parity_st25r3916(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len);

MF1ReturnCode mf1_send_receive_raw(const uint8_t *tx_buf, const uint8_t *tx_parity, uint16_t tx_data_size, uint8_t *rx_buf, uint8_t *rx_parity, uint16_t rx_buf_max_len, uint16_t *rx_data_size);
MF1ReturnCode mf1_send_receive(const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size);
MF1ReturnCode mf1_send_receive_encrypted(struct Crypto1State *cs, const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size);
MF1ReturnCode mf1_send_receive_encrypted_ex(struct Crypto1State *cs, const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size, bool decrypt);

MF1ReturnCode mf1_authenticate(struct Crypto1State *cs, bool nested, uint8_t block, key_type_t key_type, uint64_t key, const uint8_t *uid, uint8_t uid_len);

static inline uint16_t mf1_crc_a(const uint8_t *data, size_t data_len) {
    uint16_t crc = 0x6363;
    for (uint32_t i = 0; i < data_len; i++) {
        uint8_t byte = data[i];
        byte = (byte ^ (crc & 0xFF));
        byte = (byte ^ (byte << 4));
        crc = (crc >> 8) ^ ((uint16_t)byte << 8) ^ ((uint16_t)byte << 3) ^ ((uint16_t)byte >> 4);
    }
    return crc;
}
