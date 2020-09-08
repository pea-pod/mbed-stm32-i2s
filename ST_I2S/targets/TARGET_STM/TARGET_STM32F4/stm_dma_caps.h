// File which defines DMA capabilities (in platform dependent way for the moment)

#define NUM_OF_DEVICES           (2)
#define NUM_OF_DIRECTIONS        (2)

#define DMA_SPI2                 (0x0)
#define DMA_SPI3                 (0x1)
#define GET_DMA_DEVICE(x)        (x & 0xF)

#define DMA_TX                   (0x0)
#define DMA_RX                   (0x1)
#define GET_DMA_TXRX(x)          ((x & 0xF0) >> 4)

#define MAKE_CAP(dev, direction) (dev | (direction << 4))
