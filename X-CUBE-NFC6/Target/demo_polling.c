/**
  ******************************************************************************
  * @file           : ndef_poling.c
  * @brief          : Ndef Polling file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*! \file
 *
 *  \author
 *
 *  \brief Demo application
 *
 *  This demo shows how to poll for several types of NFC cards/devices and how
 *  to exchange data with these devices, using the RFAL library.
 *
 *  This demo does not fully implement the activities according to the standards,
 *  it performs the required to communicate with a card/device and retrieve
 *  its UID. Also blocking methods are used for data exchange which may lead to
 *  long periods of blocking CPU/MCU.
 *  For standard compliant example please refer to the Examples provided
 *  with the RFAL library.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "demo.h"
#include "utils.h"
#include "rfal_nfc.h"
#include "rfal_t2t.h"
#include "crapto1.h"

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
#include "demo_ce.h"
#endif /* RFAL_FEATURE_LISTEN_MODE */

/** @addtogroup X-CUBE-NFC6_Applications
 *  @{
 */

/** @addtogroup PollingTagDetect
 *  @{
 */

/** @addtogroup PTD_Demo
 * @{
 */

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
/** @defgroup PTD_Demo_Private_Define
 * @{
 */
/* Definition of possible states the demo state machine could have */
#define DEMO_ST_NOTINIT               0     /*!< Demo State:  Not initialized        */
#define DEMO_ST_START_DISCOVERY       1     /*!< Demo State:  Start Discovery        */
#define DEMO_ST_DISCOVERY             2     /*!< Demo State:  Discovery              */

#define DEMO_NFCV_BLOCK_LEN           4     /*!< NFCV Block len                      */

#define DEMO_NFCV_USE_SELECT_MODE     false /*!< NFCV demonstrate select mode        */
#define DEMO_NFCV_WRITE_TAG           false /*!< NFCV demonstrate Write Single Block */

/**
  * @}
  */
/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
/** @defgroup PTD_Demo_Private_Variables
 * @{
 */
/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

/* For a Payment application a Select PPSE would be needed:
   ppseSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 } */

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

static rfalNfcDiscoverParam discParam;
static uint8_t              state = DEMO_ST_NOTINIT;
static bool                 multiSel;

/**
  * @}
  */

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

static void demoT2t( void );
static void demoNotif( rfalNfcState st );
ReturnCode  demoTransceiveBlocking( uint8_t *txBuf, uint16_t txBufSize, uint8_t **rxBuf, uint16_t **rcvLen, uint32_t fwt );

/** @defgroup PTD_Demo_Private_Functions
 * @{
 */
/*!
 *****************************************************************************
 * \brief Demo Notification
 *
 *  This function receives the event notifications from RFAL
 *****************************************************************************
 */
static void demoNotif( rfalNfcState st )
{
    uint8_t       devCnt;
    rfalNfcDevice *dev;

    if( st == RFAL_NFC_STATE_WAKEUP_MODE )
    {
        platformLog("Wake Up mode started \r\n");
    }
    else if( st == RFAL_NFC_STATE_POLL_TECHDETECT )
    {
        if( discParam.wakeupEnabled )
        {
            platformLog("Wake Up mode terminated. Polling for devices \r\n");
        }
    }
    else if( st == RFAL_NFC_STATE_POLL_SELECT )
    {
        /* Check if in case of multiple devices, selection is already attempted */
        if( (!multiSel) )
        {
            multiSel = true;
            /* Multiple devices were found, activate first of them */
            rfalNfcGetDevicesFound( &dev, &devCnt );
            rfalNfcSelect( 0 );

            platformLog("Multiple Tags detected: %d \r\n", devCnt);
        }
        else
        {
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
        }
    }
    else if( st == RFAL_NFC_STATE_START_DISCOVERY )
    {
        /* Clear mutiple device selection flag */
        multiSel = false;
    }
}

/*!
 *****************************************************************************
 * \brief Demo Ini
 *
 *  This function Initializes the required layers for the demo
 *
 * \return true  : Initialization ok
 * \return false : Initialization failed
 *****************************************************************************
 */
bool demoIni( void )
{
    ReturnCode err;

    err = rfalNfcInitialize();
    if( err == RFAL_ERR_NONE )
    {
        rfalNfcDefaultDiscParams( &discParam );

        discParam.devLimit      = 1U;

        ST_MEMCPY( &discParam.nfcid3, NFCID3, sizeof(NFCID3) );
        ST_MEMCPY( &discParam.GB, GB, sizeof(GB) );
        discParam.GBLen         = sizeof(GB);
        discParam.p2pNfcaPrio   = true;

        discParam.notifyCb             = demoNotif;
        discParam.totalDuration        = 1000U;
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_A;

        /* Check for valid configuration by calling Discover once */
        err = rfalNfcDiscover( &discParam );
        rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );

        if( err != RFAL_ERR_NONE )
        {
            return false;
        }

        state = DEMO_ST_START_DISCOVERY;
        return true;
    }
    return false;
}

static uint8_t parity(uint8_t byte) {
    byte ^= (byte >> 4);
    byte ^= (byte >> 2);
    return (byte ^ (byte >> 1) ^ 1) & 1;
//    return (byte ^ (byte >> 1) ^ (byte >> 2) ^ (byte >> 3) ^ (byte >> 4) ^ (byte >> 5) ^ (byte >> 6) ^ (byte >> 7) ^ 1) & 1;
}

#define SET_POS(bit_index) ({ \
        __auto_type idx = (bit_index); \
        pos_byte = idx / 8; \
        pos_bit = idx % 8; \
    })

#define READER_TYPE_ST25R3916

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static void decode_parity_st25r95(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
    const size_t num_valid_bytes = buf_in_len / 2;

    for (uint32_t i = 0; i < num_valid_bytes; i++) {
        buf_out[i] = buf_in[2*i];
        parity_out[i] = (buf_in[2*i+1] >> 7);
    }

    *buf_out_len = num_valid_bytes;
}

static void encode_parity_st25r95(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
    for (uint32_t i = 0; i < buf_in_len; i++) {
        buf_out[2*i] = buf_in[i];
        buf_out[2*i+1] = (parity_in[i] << 7);
    }

    *buf_out_len = 2 * buf_in_len;
}

static void decode_parity_st25r3916(const uint8_t *buf_in, uint8_t *buf_out, uint8_t *parity_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
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

static void encode_parity_st25r3916(const uint8_t *buf_in, const uint8_t *parity_in, uint8_t *buf_out, uint16_t buf_in_len, uint16_t *buf_out_len) {
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

#pragma GCC diagnostic pop

#if defined(READER_TYPE_ST25R3916)
# define encode_parity encode_parity_st25r3916
# define decode_parity decode_parity_st25r3916
# define ENCODED_BUF_SIZE(sz) ((9 * (sz) + 7) / 8)
#elif defined(READER_TYPE_ST25R95)
# define decode_parity encode_parity_st25r95
# define decode_parity decode_parity_st25r95
# define ENCODED_BUF_SIZE(sz) (2 * (sz))
#endif

uint16_t crc_a(const uint8_t *data, size_t data_len) {
    uint16_t crc = 0x6363;
    for (uint32_t i = 0; i < data_len; i++) {
        uint8_t byte = data[i];
        byte = (byte ^ (crc & 0xFF));
        byte = (byte ^ (byte << 4));
        crc = (crc >> 8) ^ ((uint16_t)byte << 8) ^ ((uint16_t)byte << 3) ^ ((uint16_t)byte >> 4);
    }
    return crc;
}

static ReturnCode transceive_run_blocking_tx( void )
{
    ReturnCode ret;

    do{
        rfalWorker();
        ret = rfalGetTransceiveStatus();
    }
    while( (rfalIsTransceiveInTx()) && (ret == RFAL_ERR_BUSY) );

    if( rfalIsTransceiveInRx() )
    {
        return RFAL_ERR_NONE;
    }

    return ret;
}

static ReturnCode transceive(const rfalTransceiveContext *ctx) {
    ReturnCode ret = rfalStartTransceive(ctx);
    if (ret != RFAL_ERR_NONE) platformLog("rfalStartTransceive() -> %d\r\n", ret);
    ret = transceive_run_blocking_tx();
    if (ret != RFAL_ERR_NONE) platformLog("rfalTransceiveRunBlockingTx() -> %d\r\n", ret);
    ret = rfalTransceiveBlockingRx();
    if (ret != RFAL_ERR_NONE) platformLog("rfalTransceiveBlockingRx() -> %d\r\n", ret);
    return ret;
}

/**
 * tx_buf: transmit buffer of size tx_data_size
 * tx_data_size: size of the data to be transmitted, in bytes
 * rx_buf: data to be received
 * rx_data_size: size of the received data, in bytes
 */
static ReturnCode send_receive_raw(const uint8_t *tx_buf, const uint8_t *tx_parity, uint16_t tx_data_size, uint8_t *rx_buf, uint8_t *rx_parity, uint16_t rx_buf_max_len, uint16_t *rx_data_size) {
    *rx_data_size = 0;

    uint32_t flags =
            // CRC stuff
            RFAL_TXRX_FLAGS_CRC_TX_MANUAL |
            RFAL_TXRX_FLAGS_CRC_RX_MANUAL |
            RFAL_TXRX_FLAGS_CRC_RX_KEEP |
            // parity bits
            RFAL_TXRX_FLAGS_PAR_RX_KEEP |
            RFAL_TXRX_FLAGS_PAR_TX_NONE |
            // analog
            RFAL_TXRX_FLAGS_AGC_ON;

    uint8_t tx_buf_encoded[ENCODED_BUF_SIZE(tx_data_size)];
    memset(tx_buf_encoded, 0x00, ENCODED_BUF_SIZE(tx_data_size));
    uint16_t tx_buf_encoded_size_bits = 0;

    // Encode
    encode_parity(tx_buf, tx_parity, tx_buf_encoded, tx_data_size, &tx_buf_encoded_size_bits);

    uint8_t rx_buf_encoded[ENCODED_BUF_SIZE(rx_buf_max_len)];
    memset(rx_buf_encoded, 0x00, ENCODED_BUF_SIZE(rx_buf_max_len));
    uint16_t rx_buf_rcvd_len_bits = 0;

    rfalTransceiveContext ctx = {
            .flags = flags,
            .fwt = rfalConvMsTo1fc(100U),
            .rxBuf = rx_buf_encoded,
            .rxBufLen = 8 * ENCODED_BUF_SIZE(rx_buf_max_len),
            .rxRcvdLen = &rx_buf_rcvd_len_bits,
            .txBuf = tx_buf_encoded,
            .txBufLen = tx_buf_encoded_size_bits
    };

    ReturnCode ret = transceive(&ctx);

    if (ret != RFAL_ERR_NONE) {
        platformLog("Error: %d\r\n", ret);
        return ret;
    }

//    platformLog("Received data: (%d) ", rx_buf_rcvd_len_bits);
//    for (int i = 0; i < (rx_buf_rcvd_len_bits + 7)/8; i++) {
//        platformLog("%02X", rx_buf_encoded[i]);
//    }
//    platformLog("\r\n");

    // decoding parity
    decode_parity(rx_buf_encoded, rx_buf, rx_parity, rx_buf_rcvd_len_bits, rx_data_size);

    return RFAL_ERR_NONE;
}

/**
 * send or receive with automatic parity and CRC
 */
static ReturnCode send_receive(const uint8_t *tx_buf, uint16_t tx_data_size, uint8_t *rx_buf, uint16_t rx_buf_max_len, uint16_t *rx_data_size) {
    uint8_t tx_buf_crc[tx_data_size + 2];
    uint8_t tx_parity[tx_data_size + 2];
    uint8_t rx_parity[rx_buf_max_len];

    memcpy(tx_buf_crc, tx_buf, tx_data_size);
    memset(tx_parity, 0x00, tx_data_size + 2);
    memset(rx_buf, 0x00, rx_buf_max_len);
    memset(rx_parity, 0x00, rx_buf_max_len);

    // Add CRC
    uint16_t crc = crc_a(tx_buf_crc, tx_data_size);
    tx_buf_crc[tx_data_size] = crc & 0xFF;
    tx_buf_crc[tx_data_size + 1] = (crc >> 8) & 0xFF;

    // Add parity
    for (uint16_t i = 0; i < tx_data_size + 2; i++) {
        tx_parity[i] = parity(tx_buf_crc[i]);
    }

    *rx_data_size = 0;
    ReturnCode ret = send_receive_raw(tx_buf_crc, tx_parity, tx_data_size + 2, rx_buf, rx_parity, rx_buf_max_len, rx_data_size);
    if (ret != RFAL_ERR_NONE) {
        return ret;
    }

    // check CRC and parity
    for (uint16_t i = 0; i < *rx_data_size; i++) {
        if (parity(rx_buf[i]) != rx_parity[i]) {
            return RFAL_ERR_PAR;
        }
    }

    if (crc_a(rx_buf, *rx_data_size) != 0x0000) {
        return RFAL_ERR_CRC;
    }

    return RFAL_ERR_NONE;
}

/*!
 *****************************************************************************
 * \brief Demo Cycle
 *
 *  This function executes the demo state machine.
 *  It must be called periodically
 *****************************************************************************
 */
void demoCycle( void )
{
    static rfalNfcDevice *nfcDevice;

    rfalNfcWorker();                                    /* Run RFAL worker periodically */

    switch( state )
    {
        /*******************************************************************************/
        case DEMO_ST_START_DISCOVERY:

          platformLedOff(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
          platformLedOff(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
          platformLedOff(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
          platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
          platformLedOff(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
          platformLedOff(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);

          rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );
          rfalNfcDiscover( &discParam );

          multiSel = false;
          state    = DEMO_ST_DISCOVERY;
          break;

        /*******************************************************************************/
        case DEMO_ST_DISCOVERY:

            if( rfalNfcIsDevActivated( rfalNfcGetState() ) )
            {
                rfalNfcGetActiveDevice( &nfcDevice );

                switch( nfcDevice->type )
                {
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCA:

                        platformLedOn(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
                        switch( nfcDevice->dev.nfca.type )
                        {
                            case RFAL_NFCA_T1T:
                                platformLog("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                break;

                            case RFAL_NFCA_T4T:
                                platformLog("NFCA Passive ISO-DEP device found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                break;

                            case RFAL_NFCA_T4T_NFCDEP:
                            case RFAL_NFCA_NFCDEP:
                                platformLog("NFCA Passive P2P device found. NFCID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                break;

                            default: // type 2 tag/raw
                                platformLog("ISO14443A/NFC-A card found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );

                                // Detect Mifare Classic
                                uint8_t sak = nfcDevice->dev.nfca.selRes.sak;
                                uint16_t atqa = (nfcDevice->dev.nfca.sensRes.platformInfo << 8) | nfcDevice->dev.nfca.sensRes.anticollisionInfo;

                                struct mifare_model {
                                    union {
                                        uint32_t id;
                                        struct {
                                            uint8_t sak;
                                            uint16_t atqa;
                                            uint8_t _padding;
                                        } __attribute__ ((__packed__));
                                    };
                                    const char *name;
                                    uint8_t uid_size;
                                    uint16_t capacity;
                                } mifare_models[4] = {
                                        {.name = "MF1S503", .sak = 0x08, .atqa = 0x0004, .uid_size = 4, .capacity = 1024},
                                        {.name = "MF1S500", .sak = 0x08, .atqa = 0x0044, .uid_size = 7, .capacity = 1024},
                                        {.name = "MF1S700", .sak = 0x18, .atqa = 0x0002, .uid_size = 4, .capacity = 4096},
                                        {.name = "MF1S703", .sak = 0x18, .atqa = 0x0042, .uid_size = 7, .capacity = 4096},
                                };

                                platformLog("SAK = %02X, ATQA = %04X\r\n", sak, atqa);

                                struct mifare_model *model = NULL;
                                for (int i = 0; i < sizeof (mifare_models) / sizeof (*mifare_models); i++) {
                                    if ((sak | (atqa << 8)) == mifare_models[i].id) {
                                        model = &mifare_models[i];
                                    }
                                }

                                if (model) {
                                    platformLog("Detected %s with %d bytes UID and %dK capacity\r\n", model->name, model->uid_size, model->capacity >> 10);

                                    uint8_t tx_buf[2] = {0x60, 0x00}; // use key A on block 0
                                    uint8_t rx_buf[4] = {0};
                                    uint16_t rx_data_size = 0;
                                    ReturnCode ret = send_receive(tx_buf, 2, rx_buf, 4, &rx_data_size);

                                    if (ret != RFAL_ERR_NONE && ret != RFAL_ERR_CRC) {
                                        platformLog("Error: %d\r\n", ret);
                                    } else {
                                        // compute nr ar
                                        uint8_t nr_ar[8] = {0};
                                        uint8_t nr_ar_parity[8] = {0};

                                        uint32_t nt = __builtin_bswap32(*(uint32_t *)rx_buf);
                                        platformLog("Nonce tag: %08X\r\n", nt);

                                        // select a very random number for nr
                                        uint8_t nr[4] = {0x12, 0x34, 0x56, 0x78};

                                        struct Crypto1State cs;
                                        crypto1_init(&cs, 0xFFFFFFFFFFFFULL);

                                        // get UID
                                        uint32_t uid;
                                        size_t uid_offset = 0;
                                        switch (nfcDevice->nfcidLen) {
                                        case 10:
                                            uid_offset += 3;
                                        case 7:
                                            uid_offset += 3;
                                        case 4:
                                        default:
                                            uid = __builtin_bswap32(*(uint32_t *)(nfcDevice->nfcid + uid_offset));
                                        }

                                        crypto1_word(&cs, nt ^ uid, 0);

                                        // encrypt nr
                                        for (uint16_t i = 0; i < 4; i++) {
                                            nr_ar[i] = crypto1_byte(&cs, nr[i], 0) ^ nr[i];
                                            nr_ar_parity[i] = filter(cs.odd) ^ parity(nr[i]);
                                        }

                                        // skip 32 bits in the PRNG
                                        nt = prng_successor(nt, 32);

                                        // ar
                                        for (uint16_t i = 4; i < 8; i++) {
                                            nt = prng_successor(nt, 8);
                                            nr_ar[i] = crypto1_byte(&cs, 0, 0) ^ (nt & 0xFF);
                                            nr_ar_parity[i] = filter(cs.odd) ^ parity(nt);
                                        }

                                        // expected answer
                                        uint32_t at_expected = prng_successor(nt, 32) ^ crypto1_word(&cs, 0, 0);

                                        uint8_t at[4] = {0};
                                        uint8_t at_parity[4] = {0};
                                        uint16_t at_size = 0;

                                        ret = send_receive_raw(nr_ar, nr_ar_parity, 8, at, at_parity, 4, &at_size);
                                        if (ret != RFAL_ERR_NONE) {
                                            platformLog("Error when sending nr ar: %d\r\n", ret);
                                        } else if (at_size != 4) {
                                            platformLog("Bad at size\r\n");
                                        } else {
//                                            platformLog("Received at (sz=%d): %s\r\n", at_size, hex2Str(at, at_size));
                                            uint32_t at_u32 = __builtin_bswap32(*(uint32_t *)at);
                                            platformLog("Received answer tag: %08X\r\n", at_u32);
                                            platformLog("Expected answer tag: %08X\r\n", at_expected);

                                            // read block 0
                                            uint8_t cmd_plain[4] = {0x30, 0x00, 0x00, 0x00};
                                            uint16_t cmd_plain_crc = crc_a(cmd_plain, 2);
                                            cmd_plain[2] = cmd_plain_crc & 0xFF;
                                            cmd_plain[3] = (cmd_plain_crc >> 8) & 0xFF;

                                            uint8_t cmd_enc[4];
                                            uint8_t cmd_enc_parity[4];

                                            for (uint16_t i = 0; i < 4; i++) {
                                                cmd_enc[i] = crypto1_byte(&cs, 0, 0) ^ cmd_plain[i];
                                                cmd_enc_parity[i] = (filter(cs.odd) ^ parity(cmd_plain[i])) & 1;
                                            }

                                            uint8_t resp_enc[18];
                                            uint8_t resp_enc_parity[18];
                                            uint16_t resp_enc_size = 0;

                                            ret = send_receive_raw(cmd_enc, cmd_enc_parity, 4, resp_enc, resp_enc_parity, 32, &resp_enc_size);

                                            if (ret != RFAL_ERR_NONE) {
                                                platformLog("Cannot send read block command: %d\r\n", ret);
                                            } else {
                                                platformLog("Received response: %s\r\n", hex2Str(resp_enc, resp_enc_size));

                                                uint8_t resp_plain[18];
                                                uint8_t resp_plain_parity[18];
                                                for (uint16_t i = 0; i < 18; i++) {
                                                    resp_plain[i] = crypto1_byte(&cs, 0, 0) ^ resp_enc[i];
                                                    resp_plain_parity[i] = resp_enc_parity[i] ^ filter(cs.odd);
                                                }

                                                platformLog("Decrypted response: %s\r\n", hex2Str(resp_plain, 18));

                                                // Check parity
                                                bool parity_ok = true;
                                                for (uint16_t i = 0; i < 18; i++) {
                                                    if (parity(resp_plain[i]) != resp_plain_parity[i]) {
                                                        parity_ok = false;
                                                        break;
                                                    }
                                                }

                                                if (parity_ok) {
                                                    platformLog("Parity OK\r\n");
                                                } else {
                                                    platformLog("Parity invalid\r\n");
                                                }

                                                if (crc_a(resp_plain, 18) == 0x0000) {
                                                    platformLog("CRC OK\r\n");
                                                } else {
                                                    platformLog("CRC invalid\r\n");
                                                }
                                            }
                                        }
                                    }

                                } else if (sak & 0x40) {
                                    /* PICC compliant with ISO/IEC 14443-4 */
                                    demoT2t(); // read from NTAG21x or Mifare Ultralight etc
                                } else {
                                    platformLog("Unknown tag\r\n");
                                }

                                break;
                        }
                        break;

                    default:
                        break;
                }

                rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );

#if !defined(DEMO_NO_DELAY_IN_DEMOCYCLE)
                switch( nfcDevice->type )
                {
                    case RFAL_NFC_POLL_TYPE_NFCA:
                    case RFAL_NFC_POLL_TYPE_NFCF:
                        break; /* When being in card emulation don't delay as some polling devices (phones) rely on tags to be re-discoverable */
                    default:
                        platformDelay(500); /* Delay before re-starting polling loop to not flood the UART log with re-discovered tags */
                }
#endif /* DEMO_NO_DELAY_IN_DEMOCYCLE */

                state = DEMO_ST_START_DISCOVERY;
            }
            break;

        /*******************************************************************************/
        case DEMO_ST_NOTINIT:
        default:
            break;
    }
}

/*!
 *****************************************************************************
 * \brief Demo T2T Exchange
 *
 * Example how to exchange read blocks on a T2T tag
 *
 *****************************************************************************
 */
static void demoT2t( void )
{
#if RFAL_FEATURE_T2T
    ReturnCode            err;
    uint16_t              rcvLen;
    uint8_t               blockNum = 0;
    uint8_t               rxBuf[ RFAL_T2T_READ_DATA_LEN ];

    err = rfalT2TPollerRead(blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
    platformLog(" Read Block: %s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK Data:", (err != RFAL_ERR_NONE) ? "" : hex2Str( rxBuf, RFAL_T2T_READ_DATA_LEN));

#endif
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

