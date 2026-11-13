#pragma once
/**
 * True Gemini Xrossband TX — pins outside ELRS hardware.json (LOCKED gpio-map).
 * CRSF UART pins come from layout serial_tx/rx (GPIO 42/39) via GPIO_PIN_RCSIGNAL_*.
 */

#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#define TGX_ADS_SPI_HOST   HSPI
#define TGX_ADS_SCK        7
#define TGX_ADS_MOSI       6
#define TGX_ADS_MISO       5
#define TGX_ADS_CS         41
#define TGX_ADS_DRDY       40
/* ADS_RST = RC → 3V3, no GPIO */

#define TGX_ENC_A          47
#define TGX_ENC_B          21
#define TGX_ENC_SW         38

#define TGX_CRSF_LINK_TIMEOUT_MS  150
#define TGX_ADS_SPI_HZ            1000000

#endif
