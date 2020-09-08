#ifndef STM_OBJECTS_H
#define STM_OBJECTS_H

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "PinNames.h"

#include "stm_dma_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

    struct i2s_s {
	PinName pin_data;
	PinName pin_sclk;
	PinName pin_wsel;
	PinName pin_fdpx;
	PinName pin_mclk;
	uint32_t event;
	uint8_t module;
	uint8_t transfer_type;
    };

    struct dma_stream_s {
	DMA_Stream_TypeDef *dma_stream;
	IRQn_Type dma_stream_irq;
	uint32_t  channel_nr;
	uint32_t  channel_nr_fd;
	uint8_t busy;
    };

    struct dma_s {
	uint8_t dma_device;                                /**< DMA device */
	uint8_t dma_direction;                             /**< Primary DMA direction */
	const struct dma_stream_s *dma[NUM_OF_DIRECTIONS]; /**< Tx/Rx DMA devices */ // NOTE: do NOT touch contents!
    };

#ifdef __cplusplus
}
#endif

#endif
