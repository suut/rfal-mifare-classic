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
#include "crapto1.h"
#include "rfal_utils.h"

#define READER_TYPE_ST25R3916
//#define EXTRA_DEBUG

#if defined(READER_TYPE_ST25R3916)
# define encode_parity encode_parity_st25r3916
# define decode_parity decode_parity_st25r3916
# define ENCODED_BUF_SIZE(sz) ((9 * (sz) + 7) / 8)
#elif defined(READER_TYPE_ST25R95)
# define encode_parity encode_parity_st25r95
# define decode_parity decode_parity_st25r95
# define ENCODED_BUF_SIZE(sz) (2 * (sz))
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

extern mf1_model_t mifare_models[MF1_NUM_MODELS];

const mf1_model_t *identify_mf1_model(uint8_t sak, uint16_t atqa);

void decode_parity_st25r95(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len);
void encode_parity_st25r95(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len);
void decode_parity_st25r3916(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len);
void encode_parity_st25r3916(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len);

ReturnCode send_receive_raw(const uint8_t *tx_buf, const uint8_t *tx_parity, uint16_t tx_data_size, uint8_t *rx_buf, uint8_t *rx_parity, uint16_t rx_buf_max_len, uint16_t *rx_data_size);
ReturnCode send_receive(const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size);
ReturnCode send_receive_encrypted(struct Crypto1State *cs, const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size);

static inline uint16_t crc_a(const uint8_t *data, size_t data_len) {
    uint16_t crc = 0x6363;
    for (uint32_t i = 0; i < data_len; i++) {
        uint8_t byte = data[i];
        byte = (byte ^ (crc & 0xFF));
        byte = (byte ^ (byte << 4));
        crc = (crc >> 8) ^ ((uint16_t)byte << 8) ^ ((uint16_t)byte << 3) ^ ((uint16_t)byte >> 4);
    }
    return crc;
}
