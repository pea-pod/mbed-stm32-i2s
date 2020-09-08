#include "mbed_assert.h"
#include "mbed_error.h"
#include "stm_i2s_api.h"

#include "stm_dma_caps.h"

#if DEVICE_I2S

#include <math.h>
#include <string.h>
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "StmI2sPeripheralPins.h"

// #define DEBUG_STDIO 1

#define CONTAINER_OF(ptr, type, field) \
	((type *)(((char *)(ptr)) - offsetof(type, field)))

#ifndef DEBUG_STDIO
#   define DEBUG_STDIO 0
#endif

#if DEBUG_STDIO
#   include <stdio.h>
#   define DEBUG_PRINTF(...) do { printf(__VA_ARGS__); } while(0)
#else
#   define DEBUG_PRINTF(...) {}
#endif

typedef enum {
    I2S_TRANSFER_TYPE_TX = 1,
    I2S_TRANSFER_TYPE_RX = 2,
    I2S_TRANSFER_TYPE_TXRX = 3,
} transfer_type_t;

typedef enum {
	I2S_HALF_TRANSFER = 0,
	I2S_FULL_TRANSFER,
} dma_transfer_state_t;

typedef struct {
	DMA_HandleTypeDef dma_handle;
	dma_transfer_state_t t_state;
} dma_extra_state_t;

typedef struct {
	I2S_HandleTypeDef i2s_handle;
	dma_extra_state_t dma_handles[NUM_OF_DIRECTIONS];
} i2s_dma_handles_t;

#define I2S_NUM (5) // TODO: this approach wastes quite a bit of memory - TO BE IMPROVED!?!?

static i2s_dma_handles_t I2sHandle[I2S_NUM];

static void init_i2s(i2s_t *obj) {
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;

    __HAL_I2S_DISABLE(handle);
    HAL_I2S_Init(handle);
    __HAL_I2S_ENABLE(handle);
}

static void init_dmas(i2s_t *obj) {
    DMA_HandleTypeDef *primary_handle = NULL;
    DMA_HandleTypeDef *secondary_handle = NULL;
    DMA_HandleTypeDef *hdmatx = NULL;

    switch(obj->dma.dma_direction) {
    case DMA_TX:
	if(obj->dma.dma[DMA_TX] != NULL) {
	    hdmatx = primary_handle = &I2sHandle[obj->i2s.module].dma_handles[DMA_TX].dma_handle;
	}
	if(obj->dma.dma[DMA_RX] != NULL) {
	    secondary_handle = &I2sHandle[obj->i2s.module].dma_handles[DMA_RX].dma_handle;
	}
	break;
    case DMA_RX:
    default:
	if(obj->dma.dma[DMA_RX] != NULL) {
	    primary_handle = &I2sHandle[obj->i2s.module].dma_handles[DMA_RX].dma_handle;
	}
	if(obj->dma.dma[DMA_TX] != NULL) {
	    hdmatx = secondary_handle = &I2sHandle[obj->i2s.module].dma_handles[DMA_TX].dma_handle;
	}
	break;
    }

    if(primary_handle != NULL) {
	__HAL_DMA_DISABLE(primary_handle);
	HAL_DMA_Init(primary_handle);

	if(hdmatx == primary_handle) {
	    __HAL_LINKDMA(&I2sHandle[obj->i2s.module].i2s_handle, hdmatx, *primary_handle);
	} else {
	    __HAL_LINKDMA(&I2sHandle[obj->i2s.module].i2s_handle, hdmarx, *primary_handle);
	}
    }

    if(secondary_handle != NULL) {
	__HAL_DMA_DISABLE(secondary_handle);
	HAL_DMA_Init(secondary_handle);

	if(hdmatx == secondary_handle) {
	    __HAL_LINKDMA(&I2sHandle[obj->i2s.module].i2s_handle, hdmatx, *secondary_handle);
	} else {
	    __HAL_LINKDMA(&I2sHandle[obj->i2s.module].i2s_handle, hdmarx, *secondary_handle);
	}
    }
}

static inline uint32_t i2s_get_mode(i2s_mode_t mode, uint8_t *direction) {
    switch(mode) {
    case SLAVE_TX:
	*direction = DMA_TX;
	return I2S_MODE_SLAVE_TX;
    case SLAVE_RX:
	*direction = DMA_RX;
	return I2S_MODE_SLAVE_RX;
    case MASTER_TX:
	*direction = DMA_TX;
	return I2S_MODE_MASTER_TX;
    case MASTER_RX:
    default:
	*direction = DMA_RX;
	return I2S_MODE_MASTER_RX;
    }
}

static inline uint32_t i2s_get_priority(i2s_dma_prio_t priority) {
    switch(priority) {
    case LOW:
	return DMA_PRIORITY_LOW;
    case URGENT:
	return DMA_PRIORITY_VERY_HIGH;
    case HIGH:
	return DMA_PRIORITY_HIGH;
    default:
	return DMA_PRIORITY_MEDIUM;
    }
}

static void dma_i2s_init(i2s_t *obj, bool *use_tx, bool *use_rx, bool circular, i2s_dma_prio_t prio) {
    // DMA declarations
    DMA_HandleTypeDef *primary_handle = &I2sHandle[obj->i2s.module].dma_handles[obj->dma.dma_direction].dma_handle;
    DMA_HandleTypeDef *secondary_handle = NULL;

    // DMA initialization & configuration
    stm_dma_init();
    obj->dma.dma[DMA_TX] = obj->dma.dma[DMA_RX] = NULL;

    switch(obj->dma.dma_direction) {
    case DMA_TX:
	if(*use_tx) {
	    obj->dma.dma[DMA_TX] = stm_dma_channel_allocate(MAKE_CAP(obj->dma.dma_device, DMA_TX));
	    MBED_ASSERT(obj->dma.dma[DMA_TX] != STM_DMA_ERROR_OUT_OF_CHANNELS);
	}
	break;
    case DMA_RX:
    default:
	if(*use_rx) {
	    obj->dma.dma[DMA_RX] = stm_dma_channel_allocate(MAKE_CAP(obj->dma.dma_device, DMA_RX));
	    MBED_ASSERT(obj->dma.dma[DMA_RX] != STM_DMA_ERROR_OUT_OF_CHANNELS);
	}
	break;
    }

    // Primary DMA configuration
    if(obj->dma.dma[obj->dma.dma_direction] != NULL) {
	primary_handle->Instance = obj->dma.dma[obj->dma.dma_direction]->dma_stream;
	primary_handle->Init.Channel = obj->dma.dma[obj->dma.dma_direction]->channel_nr;
	primary_handle->Init.Direction = (obj->dma.dma_direction == DMA_TX) ? 
	    DMA_MEMORY_TO_PERIPH : DMA_PERIPH_TO_MEMORY;
	primary_handle->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	primary_handle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	primary_handle->Init.MemBurst = DMA_MBURST_SINGLE;
	primary_handle->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	primary_handle->Init.MemInc = DMA_MINC_ENABLE;
	primary_handle->Init.Mode = (circular ? DMA_CIRCULAR : DMA_NORMAL);
	primary_handle->Init.PeriphBurst = DMA_PBURST_SINGLE;
	primary_handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	primary_handle->Init.PeriphInc = DMA_PINC_DISABLE;
	primary_handle->Init.Priority = i2s_get_priority(prio);
    }

    // Allocate secondary DMA channel (if full-duplex)
    if(obj->i2s.pin_fdpx != NC) {
	switch(obj->dma.dma_direction) {
	case DMA_TX:
	    if(*use_rx) {
		obj->dma.dma[DMA_RX] = stm_dma_channel_allocate(MAKE_CAP(obj->dma.dma_device, DMA_RX));
		secondary_handle = &I2sHandle[obj->i2s.module].dma_handles[DMA_RX].dma_handle;
		MBED_ASSERT(obj->dma.dma[DMA_RX] != STM_DMA_ERROR_OUT_OF_CHANNELS);
	    }
	    break;
	case DMA_RX:
	default:
	    if(*use_tx) {
		obj->dma.dma[DMA_TX] = stm_dma_channel_allocate(MAKE_CAP(obj->dma.dma_device, DMA_TX));
		secondary_handle = &I2sHandle[obj->i2s.module].dma_handles[DMA_TX].dma_handle;
		MBED_ASSERT(obj->dma.dma[DMA_TX] != STM_DMA_ERROR_OUT_OF_CHANNELS);
	    }
	    break;
	}
    }

    // Secondary DMA configuration
    if(secondary_handle != NULL) {
	uint8_t secondary_dma_direction = (obj->dma.dma_direction == DMA_TX) ? DMA_RX : DMA_TX;

	secondary_handle->Instance = obj->dma.dma[secondary_dma_direction]->dma_stream;
	secondary_handle->Init.Channel = obj->dma.dma[secondary_dma_direction]->channel_nr_fd;
	secondary_handle->Init.Direction = (secondary_dma_direction == DMA_TX) ? 
	    DMA_MEMORY_TO_PERIPH : DMA_PERIPH_TO_MEMORY;
	secondary_handle->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	secondary_handle->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	secondary_handle->Init.MemBurst = DMA_MBURST_SINGLE;
	secondary_handle->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	secondary_handle->Init.MemInc = DMA_MINC_ENABLE;
	secondary_handle->Init.Mode = (circular ? DMA_CIRCULAR : DMA_NORMAL);
	secondary_handle->Init.PeriphBurst = DMA_PBURST_SINGLE;
	secondary_handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	secondary_handle->Init.PeriphInc = DMA_PINC_DISABLE;
	secondary_handle->Init.Priority = i2s_get_priority(prio);
    }

    if(obj->dma.dma[DMA_TX] == NULL) *use_tx = false;
    if(obj->dma.dma[DMA_RX] == NULL) *use_rx = false;

    // don't do anything, if the buffers aren't valid
    if (!use_tx && !use_rx) {
	DEBUG_PRINTF("I2S%u: No DMAs to init\n", obj->i2s.module+1);
	return;
    }

    DEBUG_PRINTF("I2S%u: DMA(s) Init\n", obj->i2s.module+1);
    init_dmas(obj);
}

static void dma_i2s_free(i2s_t *obj, uint8_t direction) {
    const struct dma_stream_s *stream = obj->dma.dma[direction];

    MBED_ASSERT(stream != NULL);

    // disable irq
    NVIC_DisableIRQ(stream->dma_stream_irq);

    // free channel
    stm_dma_channel_free((void*)stream);
    obj->dma.dma[direction] = NULL;
}

void i2s_init(i2s_t *obj, PinName data, PinName sclk, PinName wsel, PinName fdpx, PinName mclk, i2s_mode_t mode) {
    uint8_t dma_dev = 0, dma_direction = 0;

    // Determine the I2S/SPI to use
    SPIName i2s_data = (SPIName)pinmap_peripheral(data, PinMap_I2S_DATA);
    SPIName i2s_sclk = (SPIName)pinmap_peripheral(sclk, PinMap_I2S_SCLK);

    SPIName i2s_wsel = (SPIName)pinmap_peripheral(wsel, PinMap_I2S_WSEL);
    SPIName i2s_fdpx = (SPIName)pinmap_peripheral(fdpx, PinMap_I2S_FDPX);

    SPIName i2s_mclk = (SPIName)pinmap_peripheral(mclk, PinMap_I2S_MCLK);

    SPIName i2s_merge1 = (SPIName)pinmap_merge(i2s_data, i2s_sclk);
    SPIName i2s_merge2 = (SPIName)pinmap_merge(i2s_wsel, i2s_fdpx);

    SPIName i2s_merge3 = (SPIName)pinmap_merge(i2s_merge1, i2s_merge2);
    SPIName instance   = (SPIName)pinmap_merge(i2s_merge3, i2s_mclk);
    MBED_ASSERT(instance != (SPIName)NC);

    // Enable I2S/SPI clock and set the right module number
    switch(instance) {
#if defined(I2S1ext_BASE)
    case SPI_1:
	__SPI1_CLK_ENABLE();
	obj->i2s.module = 0;
	dma_dev = DMA_SPI1;
	break;
#endif
#if defined(I2S2ext_BASE)
    case SPI_2:
	__SPI2_CLK_ENABLE();
	obj->i2s.module = 1;
	dma_dev = DMA_SPI2;
	break;
#endif
#if defined(I2S3ext_BASE)
    case SPI_3:
	__SPI3_CLK_ENABLE();
	obj->i2s.module = 2;
	dma_dev = DMA_SPI3;
	break;
#endif
#if defined(I2S4ext_BASE)
    case SPI_4:
	__SPI4_CLK_ENABLE();
	obj->i2s.module = 3;
	dma_dev = DMA_SPI4;
	break;
#endif
#if defined(I2S5ext_BASE)
    case SPI_5:
	__SPI5_CLK_ENABLE();
	obj->i2s.module = 4;
	dma_dev = DMA_SPI5;
	break;
#endif
    default:
	MBED_ASSERT(0);
	break;
    }

    // Save DMA device
    obj->dma.dma_device = dma_dev;

    // Configure the I2S pins
    pinmap_pinout(data, PinMap_I2S_DATA);
    pinmap_pinout(wsel, PinMap_I2S_WSEL);
    pinmap_pinout(sclk, PinMap_I2S_SCLK);
    pinmap_pinout(fdpx, PinMap_I2S_FDPX);
    pinmap_pinout(mclk, PinMap_I2S_MCLK);

    obj->i2s.pin_wsel = wsel;
    obj->i2s.pin_data = data;
    obj->i2s.pin_sclk = sclk;
    obj->i2s.pin_fdpx = fdpx;
    obj->i2s.pin_mclk = mclk;

    /* Configure PLLI2S */
    static bool first_time = true;
    if(first_time) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/* Get RTCClockSelection */
	HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInitStruct);

	/* Set default configuration. Default frequency is 44100Hz. */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
        PeriphClkInitStruct.PLLI2S.PLLI2SN = 271; // NOTE: using values which are suggested in Table 91 of the
        PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;   //       reference manual for master clock enabled & 44100Hz.

#ifdef NDEBUG
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
#else
	HAL_StatusTypeDef ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
#endif
	MBED_ASSERT(ret == HAL_OK);
	first_time = false;
    }

    // initialize the handle for this master!
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;

    handle->Instance               = (SPI_TypeDef *)(instance);
    handle->Init.Mode              = i2s_get_mode(mode, &dma_direction);
    handle->Init.Standard          = I2S_STANDARD_PCM_SHORT;
    handle->Init.DataFormat        = I2S_DATAFORMAT_16B;
    handle->Init.MCLKOutput        = (mclk == NC ? I2S_MCLKOUTPUT_DISABLE : I2S_MCLKOUTPUT_ENABLE);
    handle->Init.AudioFreq         = I2S_AUDIOFREQ_44K;
    handle->Init.CPOL              = I2S_CPOL_LOW;
    handle->Init.ClockSource       = I2S_CLOCK_PLL;
    handle->Init.FullDuplexMode    = (fdpx == NC) ? I2S_FULLDUPLEXMODE_DISABLE : I2S_FULLDUPLEXMODE_ENABLE;

    // Save primary DMA direction
    obj->dma.dma_direction = dma_direction;

    DEBUG_PRINTF("I2S%u: Init\n", obj->i2s.module+1);

    init_i2s(obj);
}

void i2s_free(i2s_t *obj) {
    // Reset I2S and disable clock
    switch(obj->i2s.module) {
#if defined(I2S1ext_BASE)
    case 0:
	__SPI1_FORCE_RESET();
	__SPI1_RELEASE_RESET();
	__SPI1_CLK_DISABLE();
	break;
#endif
#if defined(I2S2ext_BASE)
    case 1:
	__SPI2_FORCE_RESET();
	__SPI2_RELEASE_RESET();
	__SPI2_CLK_DISABLE();
	break;
#endif
#if defined(I2S3ext_BASE)
    case 2:
	__SPI3_FORCE_RESET();
	__SPI3_RELEASE_RESET();
	__SPI3_CLK_DISABLE();
	break;
#endif
#if defined(I2S4ext_BASE)
    case 3:
	__SPI4_FORCE_RESET();
	__SPI4_RELEASE_RESET();
	__SPI4_CLK_DISABLE();
	break;
#endif
#if defined(I2S5ext_BASE)
    case 4:
	__SPI5_FORCE_RESET();
	__SPI5_RELEASE_RESET();
	__SPI5_CLK_DISABLE();
	break;
#endif
    default:
	MBED_ASSERT(0);
	break;
    }

    // TODO: what about 'PLLI2S'?!?
    //       for the moment we leave it enabled!

    // Configure GPIOs
    pin_function(obj->i2s.pin_wsel, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->i2s.pin_data, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->i2s.pin_sclk, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->i2s.pin_fdpx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->i2s.pin_mclk, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));

    DEBUG_PRINTF("I2S%u: Free\n", obj->i2s.module+1);
}

void i2s_format(i2s_t *obj, int dbits, int fbits, int polarity) {
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;

    // Save new values
    if (fbits == 16) { // format MUST be 16B
	handle->Init.DataFormat = I2S_DATAFORMAT_16B;
    } else { // format may NOT be 16B
	switch (dbits) {
	case 16:
	    handle->Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	    break;
	case 24:
	    handle->Init.DataFormat = I2S_DATAFORMAT_24B;
	    break;
	case 32:
	default:
	    handle->Init.DataFormat = I2S_DATAFORMAT_32B;
	    break;
	}
    }

    handle->Init.CPOL = (polarity == 0) ? I2S_CPOL_LOW : I2S_CPOL_HIGH;

    DEBUG_PRINTF("I2S%u: Format: %u (%u, %u), %u (%u)\n", obj->i2s.module+1,
		 (unsigned int)handle->Init.DataFormat, (unsigned int)dbits, (unsigned int)fbits,
		 (unsigned int)handle->Init.CPOL, (unsigned int)polarity);

    init_i2s(obj);
}

void i2s_set_mode(i2s_t *obj, i2s_mode_t mode) {
    uint8_t dma_direction;
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;

    handle->Init.Mode = i2s_get_mode(mode, &dma_direction);

    // Save primary DMA direction
    obj->dma.dma_direction = dma_direction;

    DEBUG_PRINTF("I2S%u: Mode: %u (%u)\n", obj->i2s.module+1,
		 (unsigned int)handle->Init.Mode, (unsigned int)mode);

    init_i2s(obj);
}

void i2s_set_protocol(i2s_t *obj, i2s_bitorder_t protocol) {
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;

    switch (protocol) {
    case PHILIPS:
	handle->Init.Standard = I2S_STANDARD_PHILIPS;
	break;
    case MSB:
	handle->Init.Standard = I2S_STANDARD_MSB;
	break;
    case LSB:
	handle->Init.Standard = I2S_STANDARD_LSB;
	break;
    case PCM_SHORT:
	handle->Init.Standard = I2S_STANDARD_PCM_SHORT;
	break;
    case PCM_LONG:
    default:
	handle->Init.Standard = I2S_STANDARD_PCM_LONG;
	break;
    }

    DEBUG_PRINTF("I2S%u: Protocol: %u (%u)\n", obj->i2s.module+1,
		 (unsigned int)handle->Init.Standard, (unsigned int)protocol);

    init_i2s(obj);
}

void i2s_audio_frequency(i2s_t *obj, uint32_t hz) {
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;

    if (IS_I2S_AUDIO_FREQ(hz) && (hz != I2S_AUDIOFREQ_DEFAULT)) {
	handle->Init.AudioFreq = hz;
    } else if (hz < I2S_AUDIOFREQ_8K) {
	handle->Init.AudioFreq = I2S_AUDIOFREQ_8K;
    } else {
	handle->Init.AudioFreq = I2S_AUDIOFREQ_192K;
    }

    DEBUG_PRINTF("I2S%u: Audio frequency: %u (%u)\n", obj->i2s.module+1,
		 (unsigned int)handle->Init.AudioFreq, (unsigned int)hz);

    init_i2s(obj);
}

uint8_t i2s_get_module(i2s_t *obj) {
    return obj->i2s.module;
}

static void i2s_start_asynch_transfer(i2s_t *obj, transfer_type_t transfer_type,
				      void *tx, void *rx, int length)
{
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;
    obj->i2s.transfer_type = transfer_type;

    // the HAL expects number of transfers instead of number of bytes
    int words;
    switch(handle->Init.DataFormat) {
    case I2S_DATAFORMAT_16B:
    case I2S_DATAFORMAT_16B_EXTENDED:
	words = length / 2;
	if(words > 0xFFFC) words = 0xFFFC; // truncate in order to respect max DMA length
	break;
    case I2S_DATAFORMAT_24B:
    case I2S_DATAFORMAT_32B:
    default:
	words = length / 4;
	if(words > 0x7FFC) words = 0x7FFC; // truncate in order to respect max DMA length
	break;
    }

    // enable the right hal transfer
    int rc = 0;
    switch(transfer_type) {
    case I2S_TRANSFER_TYPE_TXRX:
	// enable the interrupts
	NVIC_EnableIRQ(obj->dma.dma[DMA_TX]->dma_stream_irq);
	NVIC_EnableIRQ(obj->dma.dma[DMA_RX]->dma_stream_irq);
	// trigger DMA transfers
	rc = HAL_I2SEx_TransmitReceive_DMA(handle, (uint16_t*)tx, (uint16_t*)rx, (uint16_t)words);
	break;
    case I2S_TRANSFER_TYPE_TX:
	// enable the interrupt
	NVIC_EnableIRQ(obj->dma.dma[DMA_TX]->dma_stream_irq);
	// trigger DMA transfer
	rc = HAL_I2S_Transmit_DMA(handle, (uint16_t*)tx, (uint16_t)words);
	break;
    case I2S_TRANSFER_TYPE_RX:
	// enable the interrupt
	NVIC_EnableIRQ(obj->dma.dma[DMA_RX]->dma_stream_irq);
	// trigger DMA transfer
	rc = HAL_I2S_Receive_DMA(handle, (uint16_t*)rx, (uint16_t)words);
	break;
    }

    if (rc) {
	DEBUG_PRINTF("I2S%u: RC=%d\n", obj->i2s.module+1, rc);
    }

    return;
}

// asynchronous API
void i2s_transfer(i2s_t *obj,
		  void *tx, int tx_length,
		  void *rx, int rx_length,
		  bool circular, i2s_dma_prio_t prio,
		  uint32_t handler_tx, uint32_t handler_rx, uint32_t event)
{
    // check which use-case we have
    bool use_tx = (tx != NULL && tx_length > 0);
    bool use_rx = (rx != NULL && rx_length > 0);

    // Init DMAs
    dma_i2s_init(obj, &use_tx, &use_rx, circular, prio);

    // don't do anything, if the buffers aren't valid
    if (!use_tx && !use_rx)
	return;

    // copy the buffers to the I2S object
    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;
    obj->tx_buff.width = 16;

    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;
    obj->rx_buff.width = obj->tx_buff.width;

    obj->i2s.event = event;

    DEBUG_PRINTF("I2S%u: Transfer: %u, %u\n", obj->i2s.module+1, tx_length, rx_length);

    // register the thunking handler
    if(use_tx) {
	NVIC_SetVector(obj->dma.dma[DMA_TX]->dma_stream_irq, handler_tx);
    }
    if(use_rx) {
	NVIC_SetVector(obj->dma.dma[DMA_RX]->dma_stream_irq, handler_rx);
    }

    // enable the right hal transfer
    if (use_tx && use_rx) {
	int size = (tx_length < rx_length)? tx_length : rx_length;
	i2s_start_asynch_transfer(obj, I2S_TRANSFER_TYPE_TXRX, tx, rx, size);
    } else if (use_tx) {
	i2s_start_asynch_transfer(obj, I2S_TRANSFER_TYPE_TX, tx, NULL, tx_length);
    } else if (use_rx) {
	i2s_start_asynch_transfer(obj, I2S_TRANSFER_TYPE_RX, NULL, rx, rx_length);
    }
}

uint32_t i2s_irq_handler_asynch(i2s_t *obj, uint8_t direction) {
    direction = (direction == I2S_TX_EVENT) ? DMA_TX : DMA_RX;

    // use the right instance
    I2S_HandleTypeDef *i2s_handle = &I2sHandle[obj->i2s.module].i2s_handle;
    DMA_HandleTypeDef *dma_handle = (direction == DMA_TX) ? i2s_handle->hdmatx : i2s_handle->hdmarx;

    MBED_ASSERT(dma_handle != NULL);

    int event = 0;

    // call the Cube handler, this will update the handle
    HAL_DMA_IRQHandler(dma_handle);

    switch(HAL_I2S_GetState(i2s_handle)) {
    case HAL_I2S_STATE_READY: {
	// adjust buffer positions
	int tx_size = (i2s_handle->TxXferSize - i2s_handle->TxXferCount);
	int rx_size = (i2s_handle->RxXferSize - i2s_handle->RxXferCount);

	// take data format into consideration
	switch(i2s_handle->Init.DataFormat) {
	case I2S_DATAFORMAT_16B:
	case I2S_DATAFORMAT_16B_EXTENDED:
	    tx_size *= 2;
	    rx_size *= 2;
	    break;
	case I2S_DATAFORMAT_24B:
	case I2S_DATAFORMAT_32B:
	default:
	    tx_size *= 4;
	    rx_size *= 4;
	    break;
	}

	// adjust buffer positions
	if (obj->i2s.transfer_type != I2S_TRANSFER_TYPE_RX) {
	    obj->tx_buff.pos += tx_size;
	}
	if (obj->i2s.transfer_type != I2S_TRANSFER_TYPE_TX) {
	    obj->rx_buff.pos += rx_size;
	}

	if (i2s_handle->TxXferCount > 0) {
	    DEBUG_PRINTF("I2S%u: TxXferCount: %u\n", obj->i2s.module+1, i2s_handle->TxXferCount);
	}
	if (i2s_handle->RxXferCount > 0) {
	    DEBUG_PRINTF("I2S%u: RxXferCount: %u\n", obj->i2s.module+1, i2s_handle->RxXferCount);
	}
    }
	/* no break */

    case HAL_I2S_STATE_BUSY_TX:
    case HAL_I2S_STATE_BUSY_RX:
    case HAL_I2S_STATE_BUSY_TX_RX: {
	int error = HAL_I2S_GetError(i2s_handle);

	if(error != HAL_I2S_ERROR_NONE) {
	    // something went wrong and the transfer has definitely completed
	    event = ((direction == DMA_TX) ? I2S_EVENT_TX_ERROR : I2S_EVENT_RX_ERROR) | I2S_EVENT_INTERNAL_TRANSFER_COMPLETE;

	    if (error & HAL_I2S_ERROR_OVR) {
		// buffer overrun
		event |= I2S_EVENT_RX_OVERFLOW;
	    }

	    if (error & HAL_I2S_ERROR_UDR) {
		// buffer underrun
		event |= I2S_EVENT_TX_UNDERRUN;
	    }

	    // cleanup DMA (after error)
	    dma_i2s_free(obj, direction);
	} else { // no error detected
		dma_extra_state_t *extra_state = CONTAINER_OF(dma_handle, dma_extra_state_t, dma_handle);
		dma_transfer_state_t dma_state = extra_state->t_state;

	    switch(dma_state) {
	    case I2S_HALF_TRANSFER:
		event = ((direction == DMA_TX) ? I2S_EVENT_TX_HALF_COMPLETE : I2S_EVENT_RX_HALF_COMPLETE);
		break;
	    case I2S_FULL_TRANSFER:
		event = ((direction == DMA_TX) ? I2S_EVENT_TX_COMPLETE : I2S_EVENT_RX_COMPLETE);

		if(dma_handle->Init.Mode != DMA_CIRCULAR) {
		    if (!i2s_active(obj)) { // Check for full-duplex transfer complete!
			event |= I2S_EVENT_INTERNAL_TRANSFER_COMPLETE;
		    }

		    // cleanup DMA (because we are done)
		    dma_i2s_free(obj, direction);
		}
		break;
	    default:
		printf("(%s, %d): dma_state=0x%x\r\n", __func__, __LINE__, (int)dma_state);
		MBED_ASSERT(0);
		break;
	    }
	}
    }
	break;

    default:
	// nothing to do?!?
	break;
    }

    if (event) DEBUG_PRINTF("I2S%u: Event: 0x%x\n", obj->i2s.module+1, event);

    return (event & (obj->i2s.event | I2S_EVENT_INTERNAL_TRANSFER_COMPLETE));
}

uint8_t i2s_active(i2s_t *obj) {
    I2S_HandleTypeDef *handle = &I2sHandle[obj->i2s.module].i2s_handle;
    HAL_I2S_StateTypeDef state = HAL_I2S_GetState(handle);

    switch(state) {
    case HAL_I2S_STATE_RESET:
    case HAL_I2S_STATE_READY:
    case HAL_I2S_STATE_ERROR:
	return 0;
    default:
	return 1;
    }
}

void i2s_abort_asynch(i2s_t *obj) {
    I2S_HandleTypeDef *i2s_handle = &I2sHandle[obj->i2s.module].i2s_handle;

    // Stop transfer
    HAL_I2S_DMAStop(i2s_handle);

    if(obj->dma.dma[DMA_TX] != NULL) {
	DMA_HandleTypeDef *dma_handle_tx = &I2sHandle[obj->i2s.module].dma_handles[DMA_TX].dma_handle;

	// disable interrupt & free resource
	dma_i2s_free(obj, DMA_TX);
 
	//clean up
	__HAL_DMA_DISABLE(dma_handle_tx);
	HAL_DMA_DeInit(dma_handle_tx);
	HAL_DMA_Init(dma_handle_tx);
	__HAL_DMA_ENABLE(dma_handle_tx);
    }
    if(obj->dma.dma[DMA_RX] != NULL) {
	DMA_HandleTypeDef *dma_handle_rx = &I2sHandle[obj->i2s.module].dma_handles[DMA_RX].dma_handle;

	// disable interrupt & free resource
	dma_i2s_free(obj, DMA_RX);

	//clean up
	__HAL_DMA_DISABLE(dma_handle_rx);
	HAL_DMA_DeInit(dma_handle_rx);
	HAL_DMA_Init(dma_handle_rx);
	__HAL_DMA_ENABLE(dma_handle_rx);
    }

    // clean-up I2S
    __HAL_I2S_DISABLE(i2s_handle);
    HAL_I2S_DeInit(i2s_handle);
    HAL_I2S_Init(i2s_handle);
    __HAL_I2S_ENABLE(i2s_handle);
}

/*** Weak function overwrites ***/
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	i2s_dma_handles_t *i2s_dma_handle = CONTAINER_OF(hi2s, i2s_dma_handles_t, i2s_handle);
	dma_transfer_state_t *dma_t_state = &(i2s_dma_handle->dma_handles[DMA_TX].t_state);

	*dma_t_state = I2S_HALF_TRANSFER;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	i2s_dma_handles_t *i2s_dma_handle = CONTAINER_OF(hi2s, i2s_dma_handles_t, i2s_handle);
	dma_transfer_state_t *dma_t_state = &(i2s_dma_handle->dma_handles[DMA_RX].t_state);

	*dma_t_state = I2S_HALF_TRANSFER;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	i2s_dma_handles_t *i2s_dma_handle = CONTAINER_OF(hi2s, i2s_dma_handles_t, i2s_handle);
	dma_transfer_state_t *dma_t_state = &(i2s_dma_handle->dma_handles[DMA_TX].t_state);

	*dma_t_state = I2S_FULL_TRANSFER;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	i2s_dma_handles_t *i2s_dma_handle = CONTAINER_OF(hi2s, i2s_dma_handles_t, i2s_handle);
	dma_transfer_state_t *dma_t_state = &(i2s_dma_handle->dma_handles[DMA_RX].t_state);

	*dma_t_state = I2S_FULL_TRANSFER;
}

/*** Code for harmonizing frequencies ***/
static inline I2S_HandleTypeDef *i2s_get_handle(i2s_t *obj)
{
    return (I2S_HandleTypeDef *) &I2sHandle[obj->i2s.module].i2s_handle;
}

static float i2s_compute_closest_frequency(i2s_t *dev_i2s, uint32_t target_freq) {
    uint32_t i2sclk = 0U, i2sdiv = 2U, i2sodd = 0U, packetlength = 1U, tmp = 0U;
    I2S_HandleTypeDef *hi2s;

    /* Get the I2S handle. */
    hi2s = i2s_get_handle(dev_i2s);

    /* Check the frame length (For the Prescaler computing). */
    if (hi2s->Init.DataFormat != I2S_DATAFORMAT_16B)
    {
        /* Packet length is 32 bits */
        packetlength = 2U;
    }

    /* Get I2S source Clock frequency. */
    i2sclk = I2S_GetInputClock(hi2s);

    /* Compute the Real divider depending on the MCLK output state, with a floating point. */
    if (hi2s->Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE)
    {
        /* MCLK output is enabled. */
        tmp = (uint32_t)(((((i2sclk / 256U) * 10U) / target_freq)) + 5U);
    }
    else
    {
        /* MCLK output is disabled. */
        tmp = (uint32_t)(((((i2sclk / (32U * packetlength)) * 10U) / target_freq)) + 5U);
    }

    /* Remove the flatting point. */
    tmp = tmp / 10U;

    /* Check the parity of the divider. */
    i2sodd = (uint32_t)(tmp & (uint32_t)1U);

    /* Compute the i2sdiv prescaler. */
    i2sdiv = (uint32_t)((tmp - i2sodd) / 2U);

    /* Test if the divider is 1 or 0 or greater than 0xFF. */
    if ((i2sdiv < 2U) || (i2sdiv > 0xFFU))
    {
        /* Set the default values. */
        i2sdiv = 2U;
        i2sodd = 0U;
    }

    /* Compute the I2S frequencies. */
    uint32_t format_factor = (hi2s->Init.DataFormat == I2S_DATAFORMAT_16B ? 16 : 32);
    uint32_t mclk_factor = (hi2s->Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE ? (format_factor == 16 ? 8 : 4) : 1);
    float f = ((float)i2sclk / (float)(2 * format_factor * ((2 * i2sdiv) + i2sodd) * mclk_factor));

    return f;
}

/** Compute the real frequency of a given I2S objects.
 *
 *  @param dev_i2s reference to the I2S object.
 *  @return the computed real frequency.
 */
static inline float i2s_compute_real_frequency(i2s_t *dev_i2s)
{
    I2S_HandleTypeDef *hi2s;

    /* Get the I2S handle. */
    hi2s = i2s_get_handle(dev_i2s);

    return i2s_compute_closest_frequency(dev_i2s, hi2s->Init.AudioFreq);
}

static inline bool i2s_has_mclk(i2s_t *dev_i2s)
{
    I2S_HandleTypeDef *hi2s;

    /* Get the I2S handle. */
    hi2s = i2s_get_handle(dev_i2s);

    return (hi2s->Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE);
}

static inline uint32_t i2s_get_matching_freq(i2s_t *dev_i2s, float freq) {
    uint32_t freq_int_ceil = (uint32_t)ceilf(freq);
    uint32_t freq_int_floor = (uint32_t)freq;

    float real_freq = i2s_compute_closest_frequency(dev_i2s, freq_int_ceil);
    if(real_freq == freq) {
	return freq_int_ceil;
    } else {
	real_freq = i2s_compute_closest_frequency(dev_i2s, freq_int_floor);
	if(real_freq == freq) {
	    return freq_int_floor;
	} else {
	    return 0;
	}
    }
}

/** Computing the harmonized frequencies of two I2S peripherals.
 *
 *  @param[in]     i2s_t_l reference to the i2s_t structure with the lower frequency.
 *  @param[in]     i2s_t_h reference to the i2s_t structure with the higher frequency.
 *  @param[in|out] ptr_low_freq  pointer to lower frequency.
 *  @param[in|out] ptr_high_freq  pointer to higher frequency.
 *  @param[in]     real_low_freq real higher frequency.
 *  @param[in]     real_high_freq real lower frequency.
 *  @return "0" if the frequencies have been harmonized, "-1" otherwise.
 */
static int8_t i2s_compute_harmonized_frequencies(i2s_t *i2s_t_l, i2s_t *i2s_t_h, uint32_t *ptr_low_freq, uint32_t *ptr_high_freq,
						 float real_low_freq, float real_high_freq)
{
    /* Returning if the two real frequencies are already multiple one of the other. */
    float division = real_high_freq / real_low_freq;
    float rest = (division - (uint32_t)division);
    MBED_ASSERT(rest >= 0);
    if (rest == 0) {
        return 0;
    }
    uint32_t multiplier = ((rest >= 0.5f) ? ((uint32_t)division + 1) : (uint32_t)division);

    /* Get MCLK settings for both devices */
    bool low_freq_mclk = i2s_has_mclk(i2s_t_l);
    bool high_freq_mclk = i2s_has_mclk(i2s_t_h);

    uint32_t new_low_freq_int = i2s_get_matching_freq(i2s_t_l, real_low_freq);
    uint32_t new_high_freq_int = i2s_get_matching_freq(i2s_t_h, real_high_freq);
    MBED_ASSERT((new_low_freq_int != 0) && (new_high_freq_int != 0));

    /* Computing the harmonized frequencies so that they are multiple one of the
       other by a certain factor. */
    if(low_freq_mclk && !high_freq_mclk) { // start from low frequency
	float new_high_freq = real_low_freq * multiplier;
	new_high_freq_int = i2s_get_matching_freq(i2s_t_h, new_high_freq);
	if(new_high_freq_int == 0) {
	    return -1; // should never happen!
	}
    } else { // start from high frequency
	float new_low_freq = real_high_freq / multiplier;
	new_low_freq_int = i2s_get_matching_freq(i2s_t_l, new_low_freq);
	if(new_low_freq_int == 0) {
	    return -1; /* might happen for both devices with MCLK disabled & 
			  higher frequency 16-bit wide and lower frequency 32-bit wide
			  and odd multiplier!
		       */
	}
    }
    
    *ptr_low_freq = new_low_freq_int;
    *ptr_high_freq = new_high_freq_int;

    return 0;
}

int8_t i2s_harmonize(i2s_t *dev_i2s_1, uint32_t *freq_i2s_1, i2s_t *dev_i2s_2, uint32_t *freq_i2s_2)
{
    /* Returning if the two set frequencies are not multiple one of the other. */
    if((*freq_i2s_1 < *freq_i2s_2) ? ((*freq_i2s_2 % *freq_i2s_1) != 0) : ((*freq_i2s_1 % *freq_i2s_2) != 0)) {
	return -1;
    }

    /* Compute the real frequencies. */
    float real_f1 = i2s_compute_real_frequency(dev_i2s_1);
    float real_f2 = i2s_compute_real_frequency(dev_i2s_2);

    /* Computing the harmonized frequencies. */
    if(real_f1 < real_f2) {
    	return i2s_compute_harmonized_frequencies(dev_i2s_1, dev_i2s_2, freq_i2s_1, freq_i2s_2, real_f1, real_f2);
    } else {
    	return i2s_compute_harmonized_frequencies(dev_i2s_2, dev_i2s_1, freq_i2s_2, freq_i2s_1, real_f2, real_f1);
    }
}

#endif
