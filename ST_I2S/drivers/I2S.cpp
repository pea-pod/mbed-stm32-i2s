#include "drivers/I2S.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_assert.h"

#if DEVICE_I2S

I2S* I2S::_owner = NULL;
SingletonPtr<PlatformMutex> I2S::_mutex; // intentional class level lock!

events::EventQueue I2S::i2s_bh_queue;

void I2S::lock() {
    _mutex->lock(); // intentional class level lock!
}

void I2S::unlock() {
    _mutex->unlock(); // intentional class level lock!
}

I2S::I2S(PinName dpin, PinName clk, PinName wsel, PinName fdpin, PinName mck) :
    _i2s(),
    _irq_tx(this), _irq_rx(this),
    _priority(MEDIUM),
    _dbits(16),
    _fbits(16),
    _polarity(0),
    _protocol(PHILIPS),
    _mode(MASTER_TX),
    _circular(false),
    _hz(44100) {
    lock();
    /* Init instance */
    i2s_init(&_i2s, dpin, clk, wsel, fdpin, mck, _mode);
    acquire();
    unlock();
}

int I2S::format(int dbits, int fbits, int polarity) {
    lock();
    if (i2s_active(&_i2s)) {
	unlock();
	return -1;
    }
    _dbits = dbits;
    _fbits = fbits;
    _polarity = polarity;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    acquire();
    unlock();
    return 0;
}

int I2S::audio_frequency(unsigned int hz) {
    lock();
    if (i2s_active(&_i2s)) {
	unlock();
	return -1;
    }
    _hz = hz;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    acquire();
    unlock();
    return 0;
}

int I2S::protocol(i2s_bitorder_t protocol) {
    lock();
    if (i2s_active(&_i2s)) {
	unlock();
	return -1;
    }
    _protocol = protocol;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    acquire();
    unlock();
    return 0;
}

int I2S::mode(i2s_mode_t mode, bool circular) {
    lock();
    if (i2s_active(&_i2s)) {
	unlock();
	return -1;
    }
    _mode = mode;
    _circular = circular;
    I2S::_owner = NULL; // Not that elegant, but works. rmeyer
    acquire();
    unlock();
    return 0;
}

int I2S::harmonize(I2S &dev_i2s_1, I2S &dev_i2s_2) {
    dev_i2s_1.lock();
    if (i2s_active(&dev_i2s_1._i2s)) {
	dev_i2s_1.unlock();
	return -1;
    }

    dev_i2s_2.lock();
    if (i2s_active(&dev_i2s_2._i2s)) {
	dev_i2s_2.unlock();
	dev_i2s_1.unlock();
	return -1;
    }

    uint32_t hz1 = dev_i2s_1._hz;
    uint32_t hz2 = dev_i2s_2._hz;
    int8_t ret = i2s_harmonize(&dev_i2s_1._i2s, &hz1, &dev_i2s_2._i2s, &hz2);

    if(ret == 0) {
	dev_i2s_1.audio_frequency(hz1);
	dev_i2s_2.audio_frequency(hz2);
    }

    dev_i2s_2.unlock();
    dev_i2s_1.unlock();

    return ret;
}

void I2S::abort_transfer()
{
    lock();
    i2s_abort_asynch(&_i2s);
#if TRANSACTION_QUEUE_SIZE_I2S
    dequeue_transaction();
#endif
    unlock();
}


void I2S::clear_transfer_buffer()
{
#if TRANSACTION_QUEUE_SIZE_I2S
    lock();
    _transaction_buffer.reset();
    unlock();
#endif
}

void I2S::abort_all_transfers()
{
    lock();
    clear_transfer_buffer();
    abort_transfer();
    unlock();
}

int I2S::get_transfer_status()
{
    lock();
    if (i2s_active(&_i2s)) {
	unlock();
	return -1;
    }
    unlock();
    return  0;
}

unsigned int I2S::get_module()
{
    return i2s_get_module(&_i2s);
}

int I2S::dma_priority(i2s_dma_prio_t prio)
{
    lock();
    if (i2s_active(&_i2s)) {
	unlock();
	return -1;
    }
    _priority = prio;
    unlock();
    return  0;
}

int I2S::queue_transfer(const void *tx_buffer, int tx_length, void *rx_buffer, int rx_length, const mbed::event_callback_t& callback, int event)
{ // NOTE: MUST be called with lock held!
#if TRANSACTION_QUEUE_SIZE_I2S
    mbed::transaction_t t;

    t.tx_buffer = const_cast<void *>(tx_buffer);
    t.tx_length = tx_length;
    t.rx_buffer = rx_buffer;
    t.rx_length = rx_length;
    t.event = event;
    t.callback = callback;
    t.width = 16;
    mbed::Transaction<I2S> transaction(this, t);
    core_util_critical_section_enter();
    if (_transaction_buffer.full()) {
	core_util_critical_section_enter();
	return -1; // the buffer is full
    } else {
	_transaction_buffer.push(transaction);
	core_util_critical_section_exit();
	return 0;
    }
#else
    return -1;
#endif
}


// ignore the fact that there are multiple physical i2s's, and always update if it wasn't us last
void I2S::acquire() { // NOTE: MUST be called with lock held!
    if (_owner != this) {
	i2s_format(&_i2s, _dbits, _fbits, _polarity);
	i2s_audio_frequency(&_i2s, _hz);
	i2s_set_protocol(&_i2s, _protocol);
	i2s_set_mode(&_i2s, _mode);
	_owner = this;
    }
}

void I2S::start_transfer(const void *tx_buffer, int tx_length, void *rx_buffer, int rx_length, 
			 const mbed::event_callback_t& callback, int event)
{ // NOTE: MUST be called with lock held!
    acquire();
    _callback = callback;
    _irq_tx.callback(&I2S::irq_handler_asynch_tx);
    _irq_rx.callback(&I2S::irq_handler_asynch_rx);
    i2s_transfer(&_i2s,
		 const_cast<void *>(tx_buffer), tx_length, rx_buffer, rx_length,
		 _circular, _priority,
		 _irq_tx.entry(), _irq_rx.entry(),
		 event);
}

#if TRANSACTION_QUEUE_SIZE_I2S

void I2S::start_transaction(mbed::transaction_t *data)
{ // NOTE: MUST be called with lock held!
    start_transfer(data->tx_buffer, data->tx_length, data->rx_buffer, data->rx_length, data->callback, data->event);
}

void I2S::dequeue_transaction()
{
    lock();
    if (!i2s_active(&_i2s)) {
	mbed::Transaction<I2S> t;
	if (_transaction_buffer.pop(t)) {
	    I2S* obj = t.get_object();
	    mbed::transaction_t* data = t.get_transaction();
	    MBED_ASSERT(obj == this);
	    obj->start_transaction(data);
	}
    }
    unlock();
}

#endif

void I2S::irq_handler_asynch_rx(void)
{
    int event = i2s_irq_handler_asynch(&_i2s, I2S_RX_EVENT);
    if (_callback && (event & I2S_EVENT_ALL)) {
	I2sBhHandler::i2s_defer_function(_callback, event & I2S_EVENT_ALL);
    }
#if TRANSACTION_QUEUE_SIZE_I2S
    if (event & I2S_EVENT_INTERNAL_TRANSFER_COMPLETE) {
	I2sBhHandler::i2s_defer_function(mbed::Callback<void()>(this, &I2S::dequeue_transaction));
    }
#endif
}

void I2S::irq_handler_asynch_tx(void)
{
    int event = i2s_irq_handler_asynch(&_i2s, I2S_TX_EVENT);
    if (_callback && (event & I2S_EVENT_ALL)) {
	I2sBhHandler::i2s_defer_function(_callback, event & I2S_EVENT_ALL);
    }
#if TRANSACTION_QUEUE_SIZE_I2S
    if (event & I2S_EVENT_INTERNAL_TRANSFER_COMPLETE) {
	I2sBhHandler::i2s_defer_function(mbed::Callback<void()>(this, &I2S::dequeue_transaction));
    }
#endif
}

#endif // DEVICE_I2S
