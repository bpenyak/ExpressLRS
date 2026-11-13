#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include "Ads1256Driver.h"
#include "ChannelMixer.h"
#include "Spi3Bus.h"
#include "tgx_pins.h"

#include "crsf_protocol.h"
#include "logging.h"

#include <Arduino.h>

// ADS1256 commands / registers (datasheet)
static constexpr uint8_t CMD_WAKEUP = 0x00;
static constexpr uint8_t CMD_RDATA = 0x01;
static constexpr uint8_t CMD_SDATAC = 0x0F;
static constexpr uint8_t CMD_RREG = 0x10;
static constexpr uint8_t CMD_WREG = 0x50;
static constexpr uint8_t CMD_SYNC = 0xFC;
static constexpr uint8_t CMD_RESET = 0xFE;

static constexpr uint8_t REG_STATUS = 0x00;
static constexpr uint8_t REG_MUX = 0x01;
static constexpr uint8_t REG_ADCON = 0x02;
static constexpr uint8_t REG_DRATE = 0x03;
static constexpr uint8_t REG_IO = 0x04;

// DRATE 100 SPS (enough for sticks ≥100 Hz aggregate with multiplexing)
static constexpr uint8_t DRATE_100SPS = 0x82;
static constexpr uint8_t ADCON_CLKOUT_OFF_PGA1 = 0x00;

static SPISettings adsSpiSettings(TGX_ADS_SPI_HZ, MSBFIRST, SPI_MODE1);

bool Ads1256Driver::Begin()
{
    pinMode(TGX_ADS_CS, OUTPUT);
    digitalWrite(TGX_ADS_CS, HIGH);
    pinMode(TGX_ADS_DRDY, INPUT);

    Spi3Bus::begin(TGX_ADS_SCK, TGX_ADS_MISO, TGX_ADS_MOSI, TGX_ADS_SPI_HZ);

    bool ok = Spi3Bus::withLock([&]() {
        writeCmd(CMD_RESET);
        delay(2);
        writeCmd(CMD_SDATAC);
        writeReg(REG_STATUS, 0x01); // buffered
        writeReg(REG_ADCON, ADCON_CLKOUT_OFF_PGA1);
        writeReg(REG_DRATE, DRATE_100SPS);
        writeReg(REG_IO, 0x00);
        writeCmd(CMD_SYNC);
        delayMicroseconds(4);
        writeCmd(CMD_WAKEUP);
    });

    if (!ok)
    {
        ERRLN("TGX ADS: SPI lock failed");
        _ready = false;
        return false;
    }

    // One probe conversion; if DRDY never asserts, chip missing — stay not-ready
    int32_t probe = 0;
    if (!readChannel(0, probe))
    {
        ERRLN("TGX ADS: DRDY timeout (check wiring)");
        _ready = false;
        return false;
    }

    for (int i = 0; i < 8; i++)
    {
        _calMin[i] = -0x600000;
        _calMax[i] = 0x600000;
        _raw[i] = 0;
    }
    _ready = true;
    DBGLN("TGX ADS1256 ready");
    return true;
}

void Ads1256Driver::End()
{
    _ready = false;
}

bool Ads1256Driver::waitDrdy(uint32_t timeoutUs)
{
    const uint32_t start = micros();
    while (digitalRead(TGX_ADS_DRDY) != LOW)
    {
        if (micros() - start > timeoutUs)
            return false;
    }
    return true;
}

void Ads1256Driver::writeCmd(uint8_t cmd)
{
    auto &spi = Spi3Bus::spi();
    digitalWrite(TGX_ADS_CS, LOW);
    spi.beginTransaction(adsSpiSettings);
    spi.transfer(cmd);
    spi.endTransaction();
    digitalWrite(TGX_ADS_CS, HIGH);
    delayMicroseconds(2);
}

void Ads1256Driver::writeReg(uint8_t reg, uint8_t val)
{
    auto &spi = Spi3Bus::spi();
    digitalWrite(TGX_ADS_CS, LOW);
    spi.beginTransaction(adsSpiSettings);
    spi.transfer(CMD_WREG | (reg & 0x0F));
    spi.transfer(0x00); // 1 register
    spi.transfer(val);
    spi.endTransaction();
    digitalWrite(TGX_ADS_CS, HIGH);
    delayMicroseconds(4);
}

int32_t Ads1256Driver::readData()
{
    auto &spi = Spi3Bus::spi();
    digitalWrite(TGX_ADS_CS, LOW);
    spi.beginTransaction(adsSpiSettings);
    spi.transfer(CMD_RDATA);
    delayMicroseconds(10); // t6
    uint8_t b0 = spi.transfer(0);
    uint8_t b1 = spi.transfer(0);
    uint8_t b2 = spi.transfer(0);
    spi.endTransaction();
    digitalWrite(TGX_ADS_CS, HIGH);
    int32_t v = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    if (v & 0x800000)
        v |= 0xFF000000; // sign extend 24→32
    return v;
}

bool Ads1256Driver::readChannel(uint8_t ain, int32_t &out)
{
    if (ain > 7)
        return false;

    bool ok = false;
    // Do not hold bus lock while waiting for DRDY
    Spi3Bus::withLock([&]() {
        writeReg(REG_MUX, (uint8_t)((ain << 4) | 0x08)); // AINx vs AINCOM
        writeCmd(CMD_SYNC);
        delayMicroseconds(4);
        writeCmd(CMD_WAKEUP);
    });

    if (!waitDrdy(50000))
        return false;

    Spi3Bus::withLock([&]() {
        out = readData();
        ok = true;
    });
    return ok;
}

bool Ads1256Driver::Poll()
{
    if (!_ready)
        return false;

    bool any = false;
    for (uint8_t ain = 0; ain < 8; ain++)
    {
        int32_t v = 0;
        if (!readChannel(ain, v))
            continue;
        _raw[ain] = v;
        any = true;

        // Soft auto-cal expand on first samples
        if (!_calInit)
        {
            _calMin[ain] = v - 1000;
            _calMax[ain] = v + 1000;
        }
        else
        {
            if (v < _calMin[ain])
                _calMin[ain] = v;
            if (v > _calMax[ain])
                _calMax[ain] = v;
        }
    }
    if (any)
        _calInit = true;
    return any;
}

void Ads1256Driver::FillRcChannels(uint32_t outCh[], size_t n) const
{
    if (!outCh || n == 0)
        return;
    for (size_t i = 0; i < n; i++)
        outCh[i] = CRSF_CHANNEL_VALUE_MID;

    if (!_ready)
        return;

    // AIN0-3 → CH1-4 sticks
    for (uint8_t i = 0; i < 4 && i < n; i++)
        outCh[i] = TgxAdsCodeToCrsf(_raw[i], _calMin[i], _calMax[i]);

    // AIN4 6-pos → AUX1 (CH5)
    if (n > 4)
    {
        uint8_t bin = _swBin[0];
        uint32_t rawC = TgxAdsCodeToCrsf(_raw[4], _calMin[4], _calMax[4]);
        outCh[4] = TgxQuantizeSwitch(rawC, 6, bin, 40);
        _swBin[0] = bin;
    }
    // AIN5/6 3-pos → AUX2/3
    for (uint8_t k = 0; k < 2; k++)
    {
        const size_t ch = 5 + k;
        if (ch >= n)
            break;
        uint8_t bin = _swBin[1 + k];
        uint32_t rawC = TgxAdsCodeToCrsf(_raw[5 + k], _calMin[5 + k], _calMax[5 + k]);
        outCh[ch] = TgxQuantizeSwitch(rawC, 3, bin, 40);
        _swBin[1 + k] = bin;
    }
}

#endif
