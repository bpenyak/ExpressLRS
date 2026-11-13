#pragma once

#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX)

#include "common.h"
#include "crsf_protocol.h"

#include <HardwareSerial.h>
#include <stdint.h>

/**
 * External CRSF full-duplex on layout serial pins (TX42 / RX39).
 * Parses RC_CHANNELS_PACKED only; linkAlive via timeout.
 */
class CrsfExtPort
{
public:
    bool Begin(int8_t rxPin, int8_t txPin, uint32_t baud = 400000);
    void End();
    void Poll();

    bool linkAlive() const { return _linkAlive; }
    const uint32_t *channels() const { return _channels; }

private:
    void alignBufferToSync(uint8_t startIdx);
    bool processFrame(uint8_t totalLen);
    void unpackRcChannels(const uint8_t *payload);

    HardwareSerial *_port = nullptr;
    uint8_t _inBuffer[CRSF_MAX_PACKET_LEN] = {};
    uint8_t _packetPtr = 0;
    uint32_t _channels[CRSF_NUM_CHANNELS] = {};
    uint32_t _lastRcMs = 0;
    bool _linkAlive = false;
    bool _begun = false;
};

#endif
