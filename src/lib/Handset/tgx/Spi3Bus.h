#pragma once

#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include <SPI.h>
#include <freertos/semphr.h>

/**
 * Shared SPI3 (HSPI) ownership for ADS1256 (+ TFT uses same host via Arduino_GFX).
 * Keep critical sections short; do not hold the lock across DRDY waits.
 */
class Spi3Bus
{
public:
    static void begin(int8_t sck, int8_t miso, int8_t mosi, uint32_t hz);
    static SPIClass &spi();
    static bool lock(TickType_t ticks = pdMS_TO_TICKS(5));
    static void unlock();

    template <typename Fn>
    static bool withLock(Fn &&fn, TickType_t ticks = pdMS_TO_TICKS(5))
    {
        if (!lock(ticks))
            return false;
        fn();
        unlock();
        return true;
    }

private:
    static SPIClass *_spi;
    static SemaphoreHandle_t _mtx;
    static bool _begun;
};

#endif
