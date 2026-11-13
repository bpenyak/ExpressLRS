#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include "CrsfExtPort.h"
#include "tgx_pins.h"

#include "crc.h"
#include "logging.h"

#include <algorithm>

static GENERIC_CRC8 tgxCrsfCrc(CRSF_CRC_POLY);

bool CrsfExtPort::Begin(int8_t rxPin, int8_t txPin, uint32_t baud)
{
    if (_begun)
        return true;
    _port = new HardwareSerial(1);
    _port->begin(baud, SERIAL_8N1, rxPin, txPin, false, 0);
    _port->setTimeout(0);
    while (_port->available())
        _port->read();
    for (auto &c : _channels)
        c = CRSF_CHANNEL_VALUE_MID;
    _begun = true;
    DBGLN("TGX CRSF ext UART %d/%d @ %u", txPin, rxPin, (unsigned)baud);
    return true;
}

void CrsfExtPort::End()
{
    if (_port)
    {
        _port->end();
        delete _port;
        _port = nullptr;
    }
    _begun = false;
    _linkAlive = false;
}

void CrsfExtPort::alignBufferToSync(uint8_t startIdx)
{
    for (unsigned i = startIdx; i < _packetPtr; i++)
    {
        if (_inBuffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER ||
            _inBuffer[i] == CRSF_ADDRESS_CRSF_TRANSMITTER ||
            _inBuffer[i] == CRSF_ADDRESS_CRSF_RECEIVER ||
            _inBuffer[i] == CRSF_SYNC_BYTE)
        {
            _packetPtr -= i;
            memmove(_inBuffer, &_inBuffer[i], _packetPtr);
            return;
        }
    }
    _packetPtr = 0;
}

void CrsfExtPort::unpackRcChannels(const uint8_t *payload)
{
    constexpr unsigned srcBits = 11;
    constexpr unsigned inputChannelMask = (1 << srcBits) - 1;
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    unsigned readByteIndex = 0;
    for (uint32_t &n : _channels)
    {
        while (bitsMerged < srcBits)
        {
            const uint8_t readByte = payload[readByteIndex++];
            readValue |= ((uint32_t)readByte) << bitsMerged;
            bitsMerged += 8;
        }
        n = (readValue & inputChannelMask);
        readValue >>= srcBits;
        bitsMerged -= srcBits;
    }
}

bool CrsfExtPort::processFrame(uint8_t totalLen)
{
    const uint8_t type = _inBuffer[2];
    if (type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        return false;
    unpackRcChannels(&_inBuffer[3]);
    _lastRcMs = millis();
    _linkAlive = true;
    return true;
}

void CrsfExtPort::Poll()
{
    if (!_begun || !_port)
        return;

    auto toRead = std::min<int>(_port->available(), CRSF_MAX_PACKET_LEN - _packetPtr);
    if (toRead > 0)
        _packetPtr += _port->readBytes(&_inBuffer[_packetPtr], toRead);
    alignBufferToSync(0);

    while (_packetPtr >= 3)
    {
        const uint32_t totalLen = _inBuffer[1] + 2;
        if (totalLen < 4 || totalLen > CRSF_MAX_PACKET_LEN)
        {
            alignBufferToSync(1);
            continue;
        }
        if (_packetPtr < totalLen)
            break;

        const uint8_t crc = tgxCrsfCrc.calc(&_inBuffer[2], totalLen - 3);
        if (crc == _inBuffer[totalLen - 1])
            processFrame(totalLen);
        _packetPtr -= totalLen;
        memmove(_inBuffer, &_inBuffer[totalLen], _packetPtr);
    }

    if (_linkAlive && (millis() - _lastRcMs > TGX_CRSF_LINK_TIMEOUT_MS))
        _linkAlive = false;
}

#endif
