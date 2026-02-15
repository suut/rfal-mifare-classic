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
#include "mf1.h"
#include "crapto1.h"
#include "parity.h"

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

        discParam.devLimit      = 4U;

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

    rfalNfcWorker();

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
                platformLog("Device activated\r\n");
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


                                platformLog("SAK = %02X, ATQA = %04X\r\n", sak, atqa);

                                const mf1_model_t *model = identify_mf1_model(sak, atqa);

                                if (model) {
                                    platformLog("Detected %s with %d bytes UID and %dB capacity\r\n", model->name, model->uid_size, model->capacity);

                                    // authenticate to sector 0
                                    struct Crypto1State cs;
                                    ReturnCode ret;

                                    bool authenticated = false;
                                    bool error = false;

                                    uint8_t blocks[1024];

                                    uint32_t start = platformGetSysTick();

                                    for (int i = 0; i < 16; i++) {
                                        ret = authenticate(&cs, authenticated, 4*i, KEY_A, 0xFFFFFFFFFFFFULL, nfcDevice->nfcid, nfcDevice->nfcidLen);
                                        if (ret != RFAL_ERR_NONE) {
                                            platformLog("Cannot authenticate, wrong key?\r\n");
                                            error = true;
                                            break;
                                        }
                                        authenticated = true;

                                        for (int j = 4*i; j < 4*i+4; j++) {
                                            // read block i
                                            uint8_t cmd[2] = {0x30, j};
                                            uint8_t resp[18] = {0};
                                            uint16_t resp_size = 0;
                                            ret = send_receive_encrypted(&cs, cmd, 2, resp, 18, &resp_size);

                                            if (ret != RFAL_ERR_NONE) {
                                                platformLog("Cannot send read block command: %d\r\n", ret);
                                                error = true;
                                                break;
                                            } else {
                                                memcpy(blocks + 16*j, resp, 16);
                                            }
                                        }
                                    }

                                    if (error) {
                                        break;
                                    }
                                    // Make tag go to sleep
                                    uint8_t hlta_plain[2] = {0x50, 0x00};
                                    uint8_t rx_buf[4];
                                    uint16_t rx_data_size = 0;

                                    ret = send_receive_encrypted(&cs, hlta_plain, 2, rx_buf, 4, &rx_data_size);

                                    if (ret != RFAL_ERR_NONE && ret != RFAL_ERR_TIMEOUT) {
                                        platformLog("Error sending HLTA: %d\r\n", ret);
                                    } else if (ret != RFAL_ERR_TIMEOUT) {
                                        platformLog("Card responded to HLTA: %s(%d)", hex2Str(rx_buf, (rx_data_size+7)/8), rx_data_size % 8);
                                    }

                                    uint32_t end = platformGetSysTick();

                                    platformLog("Card read in %u ms\r\n", end - start);

                                    // Display blocks
                                    for (int i = 0; i < 64; i++) {
                                        platformLog("%02X | %s\r\n", i, hex2Str(blocks + 16*i, 16));
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
                        platformDelay(100); /* Delay before re-starting polling loop to not flood the UART log with re-discovered tags */
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

