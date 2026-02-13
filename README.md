Mifare Classic implementation using RFAL on a ST25R3916.

This should work similarly on other ST NFC chips, if the transceive flags and `encode_parity`/`decode_parity` are adapted.

Things happen in `X-CUBE-NFC6/Target/demo_polling.c`.
