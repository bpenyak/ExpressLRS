#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX) && defined(TARGET_TX)

#include "AdsCrsfMixHandset.h"

#include "OTA.h"
#include "common.h"
#include "crsf_protocol.h"
#include "logging.h"
#include "targets.h"
#include "tgx/ChannelMixer.h"
#include "tgx/tgx_pins.h"

void AdsCrsfMixHandset::Begin()
{
    DBGLN("AdsCrsfMixHandset begin");

    _adsOk = _ads.Begin();
    const int8_t rx = (GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN) ? (int8_t)GPIO_PIN_RCSIGNAL_RX : 39;
    const int8_t tx = (GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN) ? (int8_t)GPIO_PIN_RCSIGNAL_TX : 42;
    _crsf.Begin(rx, tx);

    if (_adsOk && !controllerConnected)
    {
        controllerConnected = true;
        if (connected)
            connected();
    }
}

void AdsCrsfMixHandset::End()
{
    _crsf.End();
    _ads.End();
    _adsOk = false;
    if (controllerConnected)
    {
        controllerConnected = false;
        if (disconnected)
            disconnected();
    }
}

void AdsCrsfMixHandset::setPacketInterval(int32_t PacketInterval)
{
    RequestedRCpacketInterval = PacketInterval;
}

void AdsCrsfMixHandset::handleInput()
{
    uint32_t adsCh[CRSF_NUM_CHANNELS];
    uint32_t mixed[CRSF_NUM_CHANNELS];

    for (unsigned i = 0; i < CRSF_NUM_CHANNELS; i++)
        adsCh[i] = CRSF_CHANNEL_VALUE_MID;

    if (_adsOk)
    {
        _ads.Poll();
        _ads.FillRcChannels(adsCh, CRSF_NUM_CHANNELS);
    }

    _crsf.Poll();
    const bool link = _crsf.linkAlive();
    TgxMixChannels(adsCh, _crsf.channels(), link, mixed, CRSF_NUM_CHANNELS);

    if (!_adsOk && !link)
        return;

    PerformChannelOverrides(mixed, CRSF_NUM_CHANNELS);

    isArmed = CRSF_to_BIT(mixed[AUX1]);
    RCDataReceived(mixed, CRSF_NUM_CHANNELS);

    if ((_adsOk || link) && !controllerConnected)
    {
        controllerConnected = true;
        if (connected)
            connected();
    }
}

#endif
