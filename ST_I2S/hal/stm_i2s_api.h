#ifndef STM_I2S_API_H
#define STM_I2S_API_H

#include <stdbool.h>

#include "device.h"
#include "stm_objects.h"
#include "hal/stm_dma_api.h"
#include "hal/buffer.h"

#if DEVICE_I2S

#define I2S_EVENT_RX_ERROR         (1 << 1)                                 // 0x2
#define I2S_EVENT_RX_COMPLETE      (1 << 2)									// 0x4
#define I2S_EVENT_RX_OVERFLOW      (1 << 3)                                 // 0x8
#define I2S_EVENT_RX_HALF_COMPLETE (1 << 4)                                 // 0x10

#define I2S_EVENT_TX_ERROR         (I2S_EVENT_RX_ERROR << 8)                // 0x200
#define I2S_EVENT_TX_COMPLETE      (I2S_EVENT_RX_COMPLETE << 8)				// 0x400
#define I2S_EVENT_TX_UNDERRUN      (I2S_EVENT_RX_OVERFLOW << 8)             // 0x800
#define I2S_EVENT_TX_HALF_COMPLETE (I2S_EVENT_RX_HALF_COMPLETE << 8)        // 0x1000

#define I2S_EVENT_ALL              ((I2S_EVENT_RX_ERROR | I2S_EVENT_RX_COMPLETE | I2S_EVENT_RX_OVERFLOW | I2S_EVENT_RX_HALF_COMPLETE) | \
                                    (I2S_EVENT_TX_ERROR | I2S_EVENT_TX_COMPLETE | I2S_EVENT_TX_UNDERRUN | I2S_EVENT_TX_HALF_COMPLETE))

#define I2S_EVENT_INTERNAL_TRANSFER_COMPLETE (1 << 30) // Internal flag to report that an event occurred

#define I2S_TX_EVENT				(0x0) // see DMA_TX
#define I2S_RX_EVENT				(0x1) // see DMA_RX

typedef enum {
    PHILIPS,
    MSB,
    LSB,
    PCM_SHORT,
    PCM_LONG
} i2s_bitorder_t;

typedef enum {
    SLAVE_TX,
    SLAVE_RX,
    MASTER_TX,
    MASTER_RX
} i2s_mode_t;

typedef enum {
    LOW,
    MEDIUM,
    HIGH,
    URGENT
} i2s_dma_prio_t;

/** Asynch I2S HAL structure
 */
typedef struct {
    struct i2s_s i2s;        /**< Target specific I2S structure */
    struct dma_s dma;        /**< Target specific DMA structure */
    struct buffer_s tx_buff; /**< Tx buffer */
    struct buffer_s rx_buff; /**< Rx buffer */
} i2s_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup hal_GeneralI2S I2S Configuration Functions
 * @{
 */

/** Initialize the I2S peripheral
 *
 * Configures the pins used by I2S, sets a default format and frequency, and enables the peripheral
 * @param[out] obj  The I2S object to initialize
 * @param[in]  data I2S data input/output pin
 * @param[in]  sclk I2S clock output pin
 * @param[in]  wsel I2S word select output pin (might be NC for PDM sources)
 * @param[in]  fdpx I2S data input pin (for full-duplex operation, default = NC)
 * @param[in]  mclk I2S master clock output pin (default = NC, enables master clock output when not NC)
 * @param[in]  mode I2S mode to be applied
 */
    void i2s_init(i2s_t *obj, PinName data, PinName sclk, PinName wsel, PinName fdpx, PinName mclk, i2s_mode_t mode);

/** Release a I2S object
 *
 * TODO: i2s_free is currently unimplemented
 * This will require reference counting at the C++ level to be safe
 *
 * Return the pins owned by the I2S object to their reset state
 * Disable the I2S peripheral
 * Disable the I2S clock
 * @param[in] obj The I2S object to deinitialize
 */
    void i2s_free(i2s_t *obj);

/** Configure the I2S format
 *
 * Set the number of bits per frame, configure clock polarity and phase, shift order and master/slave mode
 * @param[in,out] obj   The I2S object to configure
 * @param[in]     dbits    Number of data bits per I2S frame (16, 24, or 32)
 * @param[in]     fbits    Number of bits per I2S frame (16 or 32)
 * @param[in]     polarity Clock polarity (either 0 or 1, default = 0)
 */
    void i2s_format(i2s_t *obj, int dbits, int fbits, int polarity);

/** Configure the I2S protocol
 *
 * @param[in,out] obj      The I2S object to configure
 * @param[in]     protocol I2S protocol to be used
 */
    void i2s_set_protocol(i2s_t *obj, i2s_bitorder_t protocol);

/** Configure the I2S mode
 *
 * @param[in,out] obj      The I2S object to configure
 * @param[in]     mode     I2S mode to be applied
 */
    void i2s_set_mode(i2s_t *obj, i2s_mode_t mode);

/** Set the I2S audio frequency
 *
 * Actual frequency may differ from the desired frequency due to available dividers and bus clock
 * Configures the I2S audio frequency
 * @param[in,out] obj The I2S object to configure
 * @param[in]     hz  The baud rate in Hz
 */
    void i2s_audio_frequency(i2s_t *obj, uint32_t hz);

/**@}*/
/**
 * \defgroup SynchI2S Synchronous I2S Hardware Abstraction Layer
 * @{
 */

/** Get the module number
 *
 * @param[in] obj The I2S peripheral to check
 * @return The module number
 */
    uint8_t i2s_get_module(i2s_t *obj);

/**@}*/

/**
 * \defgroup AsynchI2S Asynchronous I2S Hardware Abstraction Layer
 * @{
 */

/** Begin the I2S transfer. Buffer pointers and lengths are specified in tx_buff and rx_buff
 *
 * @param[in] obj        The I2S object that holds the transfer information
 * @param[in] tx         The transmit buffer
 * @param[in] tx_length  The number of bytes to transmit
 * @param[in] rx         The receive buffer
 * @param[in] rx_length  The number of bytes to receive
 * @param[in] circular   Enable circular buffer transfer
 * @param[in] prio       DMA priority of the transfer
 * @param[in] handler_tx I2S tx interrupt handler
 * @param[in] handler_rx I2S rx interrupt handler
 * @param[in] event      The logical OR of events to be registered
 */
    void i2s_transfer(i2s_t *obj, void *tx, int tx_length, void *rx, int rx_length,
		      bool circular, i2s_dma_prio_t prio,
		      uint32_t handler_tx, uint32_t handler_rx, 
		      uint32_t event);

/** The asynchronous IRQ handler
 *
 * Reads the received values out of the RX FIFO, writes values into the TX FIFO and checks for transfer termination
 * conditions, such as buffer overflows or transfer complete.
 * @param[in] obj       The I2S object that holds the transfer information
 * @param[in] direction From whom is the irq coming (RX or TX)
 * @return Event flags if a transfer termination condition was met; otherwise 0.
 */
    uint32_t i2s_irq_handler_asynch(i2s_t *obj, uint8_t direction);

/** Attempts to determine if the I2S peripheral is already in use
 *
 * For each assigned buffer, check
 * if the corresponding buffer position is less than the buffer length.  If buffers do not indicate activity, check if
 * there are any bytes in the FIFOs.
 * @param[in] obj The I2S object to check for activity
 * @return Non-zero if the I2S port is active or zero if it is not.
 */
    uint8_t i2s_active(i2s_t *obj);

/** Abort an I2S transfer
 *
 * @param obj The I2S peripheral to stop
 */
    void i2s_abort_asynch(i2s_t *obj);

/**
 * @brief Harmonize frequencies of two I2S devices
 * TODO: doxygen description
 *  @return Zero if the frequencies have been harmonized correctly, -1
 *          otherwise.
 */
    int8_t i2s_harmonize(i2s_t *dev_i2s_1, uint32_t *freq_i2s_1, i2s_t *dev_i2s_2, uint32_t *freq_i2s_2);

/**@}*/

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // DEVICE_I2S

#endif // STM_I2S_API_H
