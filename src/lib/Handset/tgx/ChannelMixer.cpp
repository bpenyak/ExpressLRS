#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include "ChannelMixer.h"

#include "crsf_protocol.h"

void TgxMixChannels(const uint32_t adsCh[], const uint32_t crsfCh[], bool crsfLinkAlive,
                    uint32_t outCh[], size_t n)
{
    if (!outCh || !adsCh || n == 0)
        return;
    for (size_t i = 0; i < n; i++)
    {
        if (crsfLinkAlive && crsfCh)
            outCh[i] = crsfCh[i];
        else
            outCh[i] = adsCh[i];
    }
}

uint32_t TgxAdsCodeToCrsf(int32_t code, int32_t calMin, int32_t calMax)
{
    if (calMax <= calMin)
        return CRSF_CHANNEL_VALUE_MID;
    int32_t c = constrain(code, calMin, calMax);
    const int32_t scaled = (c - calMin) * 1023 / (calMax - calMin);
    return fmap((uint16_t)constrain(scaled, 0, 1023), 0, 1023,
                CRSF_CHANNEL_VALUE_STD_MIN, CRSF_CHANNEL_VALUE_STD_MAX);
}

uint32_t TgxQuantizeSwitch(uint32_t crsfVal, uint8_t positions, uint8_t &lastBin, uint16_t hyst)
{
    if (positions < 2)
        return crsfVal;
    const uint32_t span = CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_STD_MIN;
    const uint32_t binSize = span / (positions - 1);
    uint8_t bin = 0;
    if (crsfVal > CRSF_CHANNEL_VALUE_STD_MIN)
        bin = (uint8_t)((crsfVal - CRSF_CHANNEL_VALUE_STD_MIN + binSize / 2) / binSize);
    if (bin >= positions)
        bin = positions - 1;

    // Hysteresis around last bin edges
    if (lastBin < positions)
    {
        const int32_t center = (int32_t)CRSF_CHANNEL_VALUE_STD_MIN + (int32_t)lastBin * (int32_t)binSize;
        if (crsfVal + hyst < (uint32_t)center && lastBin > 0)
            bin = lastBin - 1;
        else if (crsfVal > (uint32_t)center + hyst && lastBin + 1 < positions)
            bin = lastBin + 1;
        else if ((int32_t)crsfVal >= center - (int32_t)hyst && (int32_t)crsfVal <= center + (int32_t)hyst)
            bin = lastBin;
    }
    lastBin = bin;
    return CRSF_CHANNEL_VALUE_STD_MIN + bin * binSize;
}

#endif
