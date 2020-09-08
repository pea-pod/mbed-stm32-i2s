#ifndef STM_DMA_API_H
#define STM_DMA_API_H

#include <stdint.h>

#define STM_DMA_ERROR_OUT_OF_CHANNELS (NULL)

typedef void* channelid_t;

#ifdef __cplusplus
extern "C" {
#endif

    void stm_dma_init(void);

    channelid_t stm_dma_channel_allocate(uint32_t capabilities);

    void stm_dma_channel_free(channelid_t channelid);

#ifdef __cplusplus
}
#endif

#endif

/** @}*/
