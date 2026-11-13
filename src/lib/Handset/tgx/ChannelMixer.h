#pragma once

#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include <stddef.h>
#include <stdint.h>

/**
 * Mix policy v1 (LOCKED): crsfLinkAlive → prefer CRSF channels; else ADS.
 * v1.1 may add per-channel masks without changing air format.
 */
void TgxMixChannels(const uint32_t adsCh[], const uint32_t crsfCh[], bool crsfLinkAlive,
                    uint32_t outCh[], size_t n);

/** Map raw ADS code (approx 24-bit signed → usable span) into CRSF channel units. */
uint32_t TgxAdsCodeToCrsf(int32_t code, int32_t calMin, int32_t calMax);

/** Quantize a CRSF value into N positions with hysteresis (for 3/6-pos switches). */
uint32_t TgxQuantizeSwitch(uint32_t crsfVal, uint8_t positions, uint8_t &lastBin, uint16_t hyst);

#endif
