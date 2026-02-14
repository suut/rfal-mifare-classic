Mifare Classic implementation using RFAL on a ST25R3916
---------

The interesting code is in `mf1/`.

The CRYPTO1 implementation is in `crapto1/` and was adapted from the version in the Proxmark3 repository.

### Porting to other NFC reader ICs

This should work similarly on other ST NFC chips, if the transceive flags and `encode_parity`/`decode_parity` are adapted.

An implementation of `encode_parity`/`decode_parity` is provided as a reference for ST25R95-style parity framing.
I suspect that there are only two different styles of parity framing, the ST25R3916-style for newer ICs and the
ST25R95-style on older ICs.

Necessary drivers and RFAL versions for various chips:

* STSW-ST25RFAL001: ST25R3911B
* STSW-ST25RFAL002: ST25R3916/ST25R3916B **← this repository**
* STSW-ST25RFAL003: ST25R95
* STSW-ST25RFAL004: ST25R100/ST25R200
* STSW-ST25RFAL005: ST25R300/ST25R500

### Necessary modifications to RFAL and driver code

#### No error on incomplete byte for ST25R3916

This is a change to the ST25R3916 driver to not receive an error in case of incomplete bytes during reception if
parity framing mode is enabled:
```diff
--- rfal_rfst25r3916_orig.c 2025-04-08 09:00:20.000000000 +0200
+++ rfal_rfst25r3916.c  2026-02-14 12:34:41.172033580 +0100
@@ -2498,7 +2498,7 @@
                  * check FIFO status for malformed or incomplete frames           */
                 
                 /* Check if the reception ends with an incomplete byte (residual bits) */
-                if( rfalFIFOStatusIsIncompleteByte() )
+                if( rfalFIFOStatusIsIncompleteByte() && !(gRFAL.TxRx.ctx.flags & RFAL_TXRX_FLAGS_PAR_RX_KEEP) )
                 {
                    gRFAL.TxRx.status = RFAL_ERR_INCOMPLETE_BYTE;
                 }
```

If you use a different NFC reader IC a similar modification might be necessary.

#### Missing parity framing flags in TX/RX context for ST25R95 (and maybe others)

If you use this with ST25R95, you will need to add the parity framing mode flags to the driver as follows:

```diff
--- rfal_rfst25r95_orig.c   2026-02-14 13:59:23.776441581 +0100
+++ rfal_rfst25r95.c    2026-02-14 14:01:31.854150979 +0100
@@ -193,6 +193,7 @@
 
 #define RFAL_ISO14443A_SDD_RES_LEN      5                                             /*!< SDD_RES | Anticollision (UID CLn) length  -  rfalNfcaSddRes                     */
 
+#define RFAL_ST25R95_ISO14443A_PARITYFRAMING                                         0x10U /*!< Transmission flags bit 4: Parity framing    */
 #define RFAL_ST25R95_ISO14443A_APPENDCRC                                             0x20U /*!< Transmission flags bit 5: Append CRC        */
 #define RFAL_ST25R95_ISO14443A_SPLITFRAME                                            0x40U /*!< Transmission flags bit 6: SplitFrame        */
 #define RFAL_ST25R95_ISO14443A_TOPAZFORMAT                                           0x80U /*!< Transmission flags bit 7: Topaz send format */
@@ -963,6 +964,10 @@
             {
                 transmitFlag |= RFAL_ST25R95_ISO14443A_APPENDCRC;
             }
+            if (gRFAL.TxRx.ctx.flags & RFAL_TXRX_FLAGS_PAR_RX_KEEP)
+            {
+                transmitFlag |= RFAL_ST25R95_ISO14443A_PARITYFRAMING;
+            }
             if (gRFAL.NfcaSplitFrame) 
             {
                 transmitFlag |= RFAL_ST25R95_ISO14443A_SPLITFRAME;
```
