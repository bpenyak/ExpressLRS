#pragma once

#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include <stddef.h>
#include <stdint.h>

/**
 * ADS1256 on shared SPI3 (HSPI). 8 single-ended inputs vs AINCOM.
 * AIN0-3 sticks, AIN4 6-pos, AIN5-6 3-pos, AIN7 pack sense (not RC).
 */
class Ads1256Driver
{
public:
    bool Begin();
    void End();

    /** Sample all 8 channels; returns false on SPI/DRDY timeout. */
    bool Poll();

    /** CRSF-unit channels for CH1-7 (indices 0-6); mid-filled if not ready. */
    void FillRcChannels(uint32_t outCh[], size_t n) const;

    int32_t raw(uint8_t ain) const { return (ain < 8) ? _raw[ain] : 0; }
    int32_t batteryRaw() const { return _raw[7]; }
    bool ready() const { return _ready; }

private:
    bool waitDrdy(uint32_t timeoutUs);
    void writeCmd(uint8_t cmd);
    void writeReg(uint8_t reg, uint8_t val);
    int32_t readData();
    bool readChannel(uint8_t ain, int32_t &out);

    int32_t _raw[8] = {};
    int32_t _calMin[8] = {};
    int32_t _calMax[8] = {};
    mutable uint8_t _swBin[3] = {0, 0, 0}; // AIN4/5/6 last bins
    bool _ready = false;
    bool _calInit = false;
};

#endif
