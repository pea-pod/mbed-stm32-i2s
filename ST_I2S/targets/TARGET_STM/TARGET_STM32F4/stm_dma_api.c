#include "stm_dma_api.h"
#include "stm_dma_caps.h"
#include "device.h"
#include "stm_objects.h"
#include "stm32f4xx.h"
#include "mbed_critical.h"

static struct dma_stream_s dma_map[NUM_OF_DEVICES][NUM_OF_DIRECTIONS];
static unsigned int dma1_ref_counter = 0;

void stm_dma_init(void) {
    static int inited = 0;

    core_util_critical_section_enter();
    if(inited) {
	core_util_critical_section_exit();
	return;
    }

    inited = 1;

    // SPI2 - TX
    dma_map[DMA_SPI2][DMA_TX].dma_stream = DMA1_Stream4;
    dma_map[DMA_SPI2][DMA_TX].dma_stream_irq = DMA1_Stream4_IRQn;
    dma_map[DMA_SPI2][DMA_TX].channel_nr = DMA_CHANNEL_0;
    dma_map[DMA_SPI2][DMA_TX].channel_nr_fd = DMA_CHANNEL_2;
    dma_map[DMA_SPI2][DMA_TX].busy = 0;

    // SPI2 - RX
    dma_map[DMA_SPI2][DMA_RX].dma_stream = DMA1_Stream3;
    dma_map[DMA_SPI2][DMA_RX].dma_stream_irq = DMA1_Stream3_IRQn;
    dma_map[DMA_SPI2][DMA_RX].channel_nr = DMA_CHANNEL_0;
    dma_map[DMA_SPI2][DMA_RX].channel_nr_fd = DMA_CHANNEL_3;
    dma_map[DMA_SPI2][DMA_RX].busy = 0;

    // SPI3 - TX
    dma_map[DMA_SPI3][DMA_TX].dma_stream = DMA1_Stream5;
    dma_map[DMA_SPI3][DMA_TX].dma_stream_irq = DMA1_Stream5_IRQn;
    dma_map[DMA_SPI3][DMA_TX].channel_nr = DMA_CHANNEL_0;
    dma_map[DMA_SPI3][DMA_TX].channel_nr_fd = DMA_CHANNEL_2;
    dma_map[DMA_SPI3][DMA_TX].busy = 0;

    // SPI3 - RX
    dma_map[DMA_SPI3][DMA_RX].dma_stream = DMA1_Stream0;
    dma_map[DMA_SPI3][DMA_RX].dma_stream_irq = DMA1_Stream0_IRQn;
    dma_map[DMA_SPI3][DMA_RX].channel_nr = DMA_CHANNEL_0;
    dma_map[DMA_SPI3][DMA_RX].channel_nr_fd = DMA_CHANNEL_3;
    dma_map[DMA_SPI3][DMA_RX].busy = 0;

    core_util_critical_section_exit();
}

channelid_t stm_dma_channel_allocate(uint32_t capabilities) {
    int device    = GET_DMA_DEVICE(capabilities);
    int direction = GET_DMA_TXRX(capabilities);

    struct dma_stream_s *channel = &dma_map[device][direction];

    core_util_critical_section_enter();
    if(channel->busy) {
	core_util_critical_section_exit();
	return STM_DMA_ERROR_OUT_OF_CHANNELS;
    } else {
	if(!(dma1_ref_counter++)) {
	    __DMA1_CLK_ENABLE();
	}
	channel->busy = 1;
	core_util_critical_section_exit();
	return (channelid_t)channel;
    }
}

void stm_dma_channel_free(channelid_t channelid) {
    struct dma_stream_s *channel = (struct dma_stream_s*)channelid;

    core_util_critical_section_enter();

    MBED_ASSERT((channel->busy == 1) && (dma1_ref_counter > 0));

    channel->busy = 0;
    if(!(--dma1_ref_counter)) {
	__DMA1_FORCE_RESET();
	__DMA1_RELEASE_RESET();
	__DMA1_CLK_DISABLE();
    }

    core_util_critical_section_exit();
    return;
}
