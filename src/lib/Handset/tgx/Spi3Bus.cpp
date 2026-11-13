#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include "Spi3Bus.h"
#include "tgx_pins.h"

SPIClass *Spi3Bus::_spi = nullptr;
SemaphoreHandle_t Spi3Bus::_mtx = nullptr;
bool Spi3Bus::_begun = false;

void Spi3Bus::begin(int8_t sck, int8_t miso, int8_t mosi, uint32_t hz)
{
    if (!_mtx)
        _mtx = xSemaphoreCreateMutex();
    if (!_spi)
        _spi = new SPIClass(TGX_ADS_SPI_HOST);
    if (!_begun)
    {
        _spi->begin(sck, miso, mosi, -1);
        _begun = true;
    }
    (void)hz;
}

SPIClass &Spi3Bus::spi()
{
    return *_spi;
}

bool Spi3Bus::lock(TickType_t ticks)
{
    if (!_mtx)
        return false;
    return xSemaphoreTake(_mtx, ticks) == pdTRUE;
}

void Spi3Bus::unlock()
{
    if (_mtx)
        xSemaphoreGive(_mtx);
}

#endif
