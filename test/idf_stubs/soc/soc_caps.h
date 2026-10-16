#ifndef SOC_CAPS_H
#define SOC_CAPS_H

/*
 * Host stub: the ESP32-S3's RMT (soc/esp32s3/include/soc/soc_caps.h in IDF
 * v5.5.5, lines 60 and 282-291). STUB_RMT_NO_TX_SYNCHRO makes it the
 * original ESP32's, which has an RMT but no TX sync (soc/esp32's
 * soc_caps.h defines no SOC_RMT_SUPPORT_TX_SYNCHRO) and eight channels
 * that each send or receive; STUB_RMT_C3 the ESP32-C3's, two TX and two RX
 * channels of four (soc/esp32c3's soc_caps.h).
 */
#define SOC_RMT_SUPPORTED		1
#if defined(STUB_RMT_C3)
#define SOC_RMT_TX_CANDIDATES_PER_GROUP	2
#define SOC_RMT_RX_CANDIDATES_PER_GROUP	2
#define SOC_RMT_CHANNELS_PER_GROUP	4
#elif defined(STUB_RMT_NO_TX_SYNCHRO)
#define SOC_RMT_TX_CANDIDATES_PER_GROUP	8
#define SOC_RMT_RX_CANDIDATES_PER_GROUP	8
#define SOC_RMT_CHANNELS_PER_GROUP	8
#else
#define SOC_RMT_TX_CANDIDATES_PER_GROUP	4
#define SOC_RMT_RX_CANDIDATES_PER_GROUP	4
#define SOC_RMT_CHANNELS_PER_GROUP	8
#endif
#define SOC_RMT_MEM_WORDS_PER_CHANNEL	48
#ifndef STUB_RMT_NO_TX_SYNCHRO
#define SOC_RMT_SUPPORT_TX_SYNCHRO	1
#endif

#endif
