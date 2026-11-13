#pragma once

#if defined(TARGET_TRUE_GEMINI_XROSSBAND_TX) && defined(TARGET_TX)

#include "handset.h"
#include "tgx/Ads1256Driver.h"
#include "tgx/CrsfExtPort.h"

/**
 * Standalone handset: ADS1256 base + optional external CRSF mix → ChannelData.
 * Does not subclass CRSFHandset; RF path remains upstream ELRS.
 */
class AdsCrsfMixHandset final : public Handset
{
public:
    void Begin() override;
    void End() override;
    void handleInput() override;
    void setPacketInterval(int32_t PacketInterval) override;

private:
    Ads1256Driver _ads;
    CrsfExtPort _crsf;
    bool _adsOk = false;
};

#endif
