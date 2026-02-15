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

#include <stdint.h>
#include <string.h>
#include "rfal_nfc.h"
#include "rfal_platform.h"
#include "crapto1.h"
#include "parity.h"
#include "mf1.h"

#define SET_POS(bit_index) ({ \
        __auto_type idx = (bit_index); \
        pos_byte = idx / 8; \
        pos_bit = idx % 8; \
    })

static MF1ReturnCode transceive_run_blocking_tx(void);
static MF1ReturnCode transceive(const rfalTransceiveContext *ctx);

// TODO: Add Mifare Plus 2K SL1 (need to read ATS)
mf1_model_t mifare_models[MF1_NUM_MODELS] = {
        {.name = "Mifare Mini", .short_name = "MF1S200", .sak = 0x09, .atqa = 0x0004, .uid_size = 4, .capacity = 320},
        {.name = "Mifare Mini", .short_name = "MF1S203", .sak = 0x09, .atqa = 0x0044, .uid_size = 7, .capacity = 320},
        {.name = "Mifare Classic 1K", .short_name = "MF1S500", .sak = 0x08, .atqa = 0x0004, .uid_size = 4, .capacity = 1024},
        {.name = "Mifare Classic 1K", .short_name = "MF1S503", .sak = 0x08, .atqa = 0x0044, .uid_size = 7, .capacity = 1024},
        {.name = "Mifare Classic 4K", .short_name = "MF1S700", .sak = 0x18, .atqa = 0x0002, .uid_size = 4, .capacity = 4096},
        {.name = "Mifare Classic 4K", .short_name = "MF1S703", .sak = 0x18, .atqa = 0x0042, .uid_size = 7, .capacity = 4096},
};

const mf1_model_t *mf1_identify_model(uint8_t sak, uint16_t atqa) {
    for (int i = 0; i < MF1_NUM_MODELS; i++) {
        if ((sak | (atqa << 8)) == (mifare_models[i].id & 0x00FFFFFF)) {
            return &mifare_models[i];
        }
    }
    return NULL;
}

#if defined(MF1_READER_TYPE_ST25R95)

void mf1_decode_parity_st25r95(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
    const size_t num_valid_bytes = buf_in_len / 2;

    for (uint32_t i = 0; i < num_valid_bytes; i++) {
        buf_out[i] = buf_in[2*i];
        parity_out[i] = (buf_in[2*i+1] >> 7);
    }

    *buf_out_len = num_valid_bytes;
}

void mf1_encode_parity_st25r95(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
    for (uint32_t i = 0; i < buf_in_len; i++) {
        buf_out[2*i] = buf_in[i];
        buf_out[2*i+1] = (parity_in[i] << 7);
    }

    *buf_out_len = 2 * buf_in_len;
}

#elif defined(MF1_READER_TYPE_ST25R3916)

void mf1_decode_parity_st25r3916(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
    const size_t num_valid_bytes = buf_in_len / 9;

    memset(buf_out, 0x00, num_valid_bytes);
    memset(parity_out, 0x00, num_valid_bytes);

    uint32_t pos_byte;
    uint32_t pos_bit;

    for (uint32_t i = 0; i < num_valid_bytes; i++) {
        // set data bits
        for (uint32_t j = 0; j < 8; j++) {
            SET_POS(j + 9 * i);
            buf_out[i] |= (((buf_in[pos_byte] >> pos_bit) & 1) << j);
        }

        // set parity bit
        SET_POS(8 + 9 * i);
        parity_out[i] = ((buf_in[pos_byte] >> pos_bit) & 1);
    }

    *buf_out_len = num_valid_bytes;
}

void mf1_encode_parity_st25r3916(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
    size_t num_out_bits = 9 * buf_in_len;
    size_t num_out_bytes = (num_out_bits + 7) / 8;
    memset(buf_out, 0x00, num_out_bytes);

    uint32_t pos_byte;
    uint32_t pos_bit;

    for (uint32_t i = 0; i < buf_in_len; i++) {
        // set data bits
        for (uint32_t j = 0; j < 8; j++) {
            SET_POS(j + 9 * i);
            buf_out[pos_byte] |= (((buf_in[i] >> j) & 1) << pos_bit);
        }

        // set parity bit
        SET_POS(8 + 9 * i);
        buf_out[pos_byte] |= (parity_in[i] << pos_bit);
    }

    *buf_out_len = num_out_bits;
}

#else

# error At least one reader type must be defined

#endif

//#pragma GCC diagnostic pop

static MF1ReturnCode transceive_run_blocking_tx(void)
{
    MF1ReturnCode ret;

    do{
        rfalWorker();
        ret = rfalGetTransceiveStatus();
    }
    while( (rfalIsTransceiveInTx()) && (ret == MF1_ERR(BUSY)) );

    if( rfalIsTransceiveInRx() )
    {
        return MF1_ERR(NONE);
    }

    return ret;
}

static MF1ReturnCode transceive(const rfalTransceiveContext *ctx) {
    MF1ReturnCode ret = rfalStartTransceive(ctx);
#ifdef MF1_EXTRA_DEBUG
    if (ret != MF1_ERR(NONE)) mf1_printf("rfalStartTransceive() -> %d\r\n", ret);
#endif
    ret = transceive_run_blocking_tx();
#ifdef MF1_EXTRA_DEBUG
    if (ret != MF1_ERR(NONE)) mf1_printf("rfalTransceiveRunBlockingTx() -> %d\r\n", ret);
#endif
    ret = rfalTransceiveBlockingRx();
#ifdef MF1_EXTRA_DEBUG
    if (ret != MF1_ERR(NONE)) mf1_printf("rfalTransceiveBlockingRx() -> %d\r\n", ret);
#endif
    return ret;
}

/**
 * tx_buf: transmit buffer of size tx_data_size
 * tx_data_size: size of the data to be transmitted, in bytes
 * rx_buf: data to be received
 * rx_data_size: size of the received data, in bytes
 */
MF1ReturnCode mf1_send_receive_raw(const uint8_t *tx_buf, const uint8_t *tx_parity, uint16_t tx_data_size, uint8_t *rx_buf, uint8_t *rx_parity, uint16_t rx_buf_max_len, uint16_t *rx_data_size) {
    *rx_data_size = 0;

    uint32_t flags = MF1_TXRX_FLAGS;

    uint8_t tx_buf_encoded[MF1_ENCODED_BUF_SIZE(tx_data_size)];
    memset(tx_buf_encoded, 0x00, MF1_ENCODED_BUF_SIZE(tx_data_size));
    uint16_t tx_buf_encoded_size_bits = 0;

    // Encode
    mf1_encode_parity(tx_buf, tx_parity, tx_buf_encoded, tx_data_size, &tx_buf_encoded_size_bits);

    uint8_t rx_buf_encoded[MF1_ENCODED_BUF_SIZE(rx_buf_max_len)];
    memset(rx_buf_encoded, 0x00, MF1_ENCODED_BUF_SIZE(rx_buf_max_len));
    uint16_t rx_buf_rcvd_len_bits = 0;

    rfalTransceiveContext ctx = {
            .flags = flags,
            .fwt = rfalConvMsTo1fc(1U),
            .rxBuf = rx_buf_encoded,
            .rxBufLen = 8 * MF1_ENCODED_BUF_SIZE(rx_buf_max_len),
            .rxRcvdLen = &rx_buf_rcvd_len_bits,
            .txBuf = tx_buf_encoded,
            .txBufLen = tx_buf_encoded_size_bits
    };

    MF1ReturnCode ret = transceive(&ctx);

#ifdef MF1_EXTRA_DEBUG
    mf1_printf("Received data: (%db) ", rx_buf_rcvd_len_bits);
    for (int i = 0; i < (rx_buf_rcvd_len_bits + 7)/8; i++) {
        mf1_printf("%02X ", rx_buf_encoded[i]);
    }
    mf1_printf("\r\n");
#endif

    if (ret != MF1_ERR(NONE)) {
#ifdef MF1_EXTRA_DEBUG
        mf1_printf("Error: %d\r\n", ret);
#endif
        return ret;
    }

    // decoding parity
    mf1_decode_parity(rx_buf_encoded, rx_buf, rx_parity, rx_buf_rcvd_len_bits, rx_data_size);

    return MF1_ERR(NONE);
}

/**
 * send or receive with automatic parity and CRC
 */
MF1ReturnCode mf1_send_receive(const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size) {
    uint8_t tx_buf_crc[tx_data_size + 2];
    uint8_t tx_parity[tx_data_size + 2];
    uint8_t rx_parity[rx_buf_max_len];

    memcpy(tx_buf_crc, tx_buf, tx_data_size);
    memset(tx_parity, 0x00, tx_data_size + 2);
    memset(rx_buf, 0x00, rx_buf_max_len);
    memset(rx_parity, 0x00, rx_buf_max_len);

    // Add CRC
    uint16_t crc = mf1_crc_a(tx_buf_crc, tx_data_size);
    tx_buf_crc[tx_data_size] = crc & 0xFF;
    tx_buf_crc[tx_data_size + 1] = (crc >> 8) & 0xFF;

    // Add parity
    for (uint16_t i = 0; i < tx_data_size + 2; i++) {
        tx_parity[i] = oddparity8(tx_buf_crc[i]);
    }

    *rx_data_size = 0;
    MF1ReturnCode ret = mf1_send_receive_raw(tx_buf_crc, tx_parity, tx_data_size + 2, rx_buf, rx_parity, rx_buf_max_len, rx_data_size);
    if (ret != MF1_ERR(NONE)) {
        return ret;
    }

    // check CRC and parity
    for (uint16_t i = 0; i < *rx_data_size; i++) {
        if (oddparity8(rx_buf[i]) != rx_parity[i]) {
            return MF1_ERR(PAR);
        }
    }

    if (mf1_crc_a(rx_buf, *rx_data_size) != 0x0000) {
        return MF1_ERR(CRC);
    }

    return MF1_ERR(NONE);
}

/**
 * send or receive encrypted message with automatic parity and CRC
 */
MF1ReturnCode mf1_send_receive_encrypted(struct Crypto1State *cs, const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf,
        uint16_t rx_buf_max_len, uint16_t *rx_data_size) {
    return mf1_send_receive_encrypted_ex(cs, tx_buf, tx_data_size, rx_buf, rx_buf_max_len, rx_data_size, true);
}

/**
 * send or receive encrypted message with automatic parity and CRC,
 * optionally do not decrypt the message or check parity or CRC
 */
MF1ReturnCode mf1_send_receive_encrypted_ex(struct Crypto1State *cs, const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf,
        uint16_t rx_buf_max_len, uint16_t *rx_data_size, bool decrypt) {
    uint8_t tx_buf_crc[tx_data_size + 2];
    uint8_t tx_enc[tx_data_size + 2];
    uint8_t tx_enc_parity[tx_data_size + 2];
    uint8_t rx_enc[rx_buf_max_len];
    uint8_t rx_enc_parity[rx_buf_max_len];
    uint8_t rx_parity[rx_buf_max_len];

    memcpy(tx_buf_crc, tx_buf, tx_data_size);
    memset(tx_enc, 0x00, tx_data_size + 2);
    memset(tx_enc_parity, 0x00, tx_data_size + 2);
    memset(rx_buf, 0x00, rx_buf_max_len);
    memset(rx_enc, 0x00, rx_buf_max_len);
    memset(rx_enc_parity, 0x00, rx_buf_max_len);
    memset(rx_parity, 0x00, rx_buf_max_len);

    // Add CRC
    uint16_t crc = mf1_crc_a(tx_buf_crc, tx_data_size);
    tx_buf_crc[tx_data_size] = crc & 0xFF;
    tx_buf_crc[tx_data_size + 1] = (crc >> 8) & 0xFF;

    // Encrypt
    for (uint16_t i = 0; i < tx_data_size + 2; i++) {
        tx_enc[i] = crypto1_byte(cs, 0, 0) ^ tx_buf_crc[i];
        tx_enc_parity[i] = filter(cs->odd) ^ oddparity8(tx_buf_crc[i]);
    }

    *rx_data_size = 0;
    MF1ReturnCode ret = mf1_send_receive_raw(tx_enc, tx_enc_parity, tx_data_size + 2, rx_enc, rx_enc_parity, rx_buf_max_len, rx_data_size);
    if (ret != MF1_ERR(NONE)) {
        return ret;
    }

    if (decrypt) {
        // Decrypt
        for (uint16_t i = 0; i < *rx_data_size; i++) {
            rx_buf[i] = crypto1_byte(cs, 0, 0) ^ rx_enc[i];
            rx_parity[i] = filter(cs->odd) ^ rx_enc_parity[i];
        }

        // check CRC and parity
        for (uint16_t i = 0; i < *rx_data_size; i++) {
            if (oddparity8(rx_buf[i]) != rx_parity[i]) {
                return MF1_ERR(PAR);
            }
        }

        if (mf1_crc_a(rx_buf, *rx_data_size) != 0x0000) {
            return MF1_ERR(CRC);
        }
    } else {
        memcpy(rx_buf, rx_enc, *rx_data_size);
        // FIXME: check parity when loading the new key in the key stream?
    }

    return MF1_ERR(NONE);
}

MF1ReturnCode mf1_authenticate(struct Crypto1State *cs, bool nested, uint8_t block, key_type_t key_type, uint64_t key, const uint8_t *uid, uint8_t uid_len) {
    uint8_t tx_buf[2] = {0x60 | key_type, block};
    uint8_t rx_buf[4] = {0};
    uint16_t rx_data_size = 0;

    MF1ReturnCode ret;

    if (nested) {
        // send command encrypted
        ret = mf1_send_receive_encrypted_ex(cs, tx_buf, 2, rx_buf, 4, &rx_data_size, false);
    } else {
        // send command in plain text
        ret = mf1_send_receive(tx_buf, 2, rx_buf, 4, &rx_data_size);
    }

    if (ret != MF1_ERR(NONE) && ret != MF1_ERR(CRC)) {
        mf1_printf("Error: %d\r\n", ret);
        return ret;
    } else if (rx_data_size != 4) {
        mf1_printf("Error: invalid nonce size (%d)\r\n", rx_data_size);
        return MF1_ERR(SEMANTIC);
    }

    // compute nr ar
    uint8_t nr_ar[8] = {0};
    uint8_t nr_ar_parity[8] = {0};

    uint32_t nt = __builtin_bswap32(*(uint32_t *)rx_buf);
#ifdef MF1_EXTRA_DEBUG
    mf1_printf("nt: %08X\r\n", nt);
#endif

    // select a very random number for nr
    uint8_t nr[4] = {0x12, 0x34, 0x56, 0x78};

    // get UID
    uint32_t uid_u32;
    size_t uid_offset = 0;
    switch (uid_len) {
    case 10:
        uid_offset += 3;
    case 7:
        uid_offset += 3;
    case 4:
    default:
        uid_u32 = __builtin_bswap32(*(uint32_t *)(uid + uid_offset));
    }

    if (nested) {
        crypto1_deinit(cs);
    }

    crypto1_init(cs, key);

    if (nested) {
        // we need to decrypt the nonce with the new key
        nt = crypto1_word(cs, nt ^ uid_u32, 1) ^ nt;
    } else {
        // initialize keystream with plain text nonce
        crypto1_word(cs, nt ^ uid_u32, 0);
    }

    // encrypt nr
    for (uint16_t i = 0; i < 4; i++) {
        nr_ar[i] = crypto1_byte(cs, nr[i], 0) ^ nr[i];
        nr_ar_parity[i] = filter(cs->odd) ^ oddparity8(nr[i]);
    }

    // skip 32 bits in the PRNG
    nt = prng_successor(nt, 32);

    // ar
    for (uint16_t i = 4; i < 8; i++) {
        nt = prng_successor(nt, 8);
        nr_ar[i] = crypto1_byte(cs, 0, 0) ^ (nt & 0xFF);
        nr_ar_parity[i] = filter(cs->odd) ^ oddparity8(nt);
    }

#ifdef MF1_EXTRA_DEBUG
    mf1_printf("nr: %s\r\n", mf1_hex(nr_ar, 4));
    mf1_printf("ar: %s\r\n", mf1_hex(nr_ar + 4, 4));
#endif

    // expected answer
    uint32_t at_expected = prng_successor(nt, 32) ^ crypto1_word(cs, 0, 0);

    uint8_t at[4] = {0};
    uint8_t at_parity[4] = {0};
    uint16_t at_size = 0;

    ret = mf1_send_receive_raw(nr_ar, nr_ar_parity, 8, at, at_parity, 4, &at_size);
    if (ret == MF1_ERR(TIMEOUT)) {
        mf1_printf("Invalid key A for sector 0\r\n");
        return MF1_ERR(SEMANTIC);
    } else if (ret != MF1_ERR(NONE)) {
        mf1_printf("Error when sending nr ar: %d\r\n", ret);
        return ret;
    } else if (at_size != 4) {
        mf1_printf("Bad at size\r\n");
        return MF1_ERR(SEMANTIC);
    }
    uint32_t at_u32 = __builtin_bswap32(*(uint32_t *)at);
#ifdef MF1_EXTRA_DEBUG
        mf1_printf("at: %08X\r\n", at_u32);
#endif
    if (at_u32 != at_expected) {
        mf1_printf("Wrong answer tag\r\n");
        return MF1_ERR(SEMANTIC);
    }
    // we are authenticated
    return MF1_ERR(NONE);
}
