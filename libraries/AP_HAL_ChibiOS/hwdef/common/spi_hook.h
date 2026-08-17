#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#if HAL_USE_MMC_SPI == TRUE
void spiStartHook(SPIDriver *spip, const SPIConfig *config);
void spiStopHook(SPIDriver *spip);
void spiAcquireBusHook(SPIDriver *spip);
void spiReleaseBusHook(SPIDriver *spip);
void spiSelectHook(SPIDriver *spip);
void spiUnselectHook(SPIDriver *spip);
bool spiIgnoreHook(SPIDriver *spip,size_t n);
bool spiSendHook(SPIDriver *spip,size_t n, const void *txbuf);
bool spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf);
#endif

#ifdef __cplusplus
}
#endif
