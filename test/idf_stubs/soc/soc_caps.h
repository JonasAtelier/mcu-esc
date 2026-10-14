#ifndef SOC_CAPS_H
#define SOC_CAPS_H

/*
 * Host stub: the ESP32-S3's RMT (soc/esp32s3/include/soc/soc_caps.h in IDF
 * v5.5.5, lines 60 and 282-291). STUB_RMT_NO_TX_SYNCHRO makes it the
 * original ESP32's, which has an RMT but no TX sync (soc/esp32's
 * soc_caps.h defines no SOC_RMT_SUPPORT_TX_SYNCHRO).
 */
#define SOC_RMT_SUPPORTED		1
#define SOC_RMT_TX_CANDIDATES_PER_GROUP	4
#define SOC_RMT_MEM_WORDS_PER_CHANNEL	48
#ifndef STUB_RMT_NO_TX_SYNCHRO
#define SOC_RMT_SUPPORT_TX_SYNCHRO	1
#endif

#endif
