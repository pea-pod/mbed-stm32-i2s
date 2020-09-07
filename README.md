# I2S Driver for mbed and the STM32 series MCUs

This repository contains all of the necesssary files to use the I2S peripheral on an STM32F4 series device.

## Driver Details

This is a fork from [https://os.mbed.com/teams/ST/code/ST_I2S/](https://os.mbed.com/teams/ST/code/ST_I2S/). This code has been modified to work with mbed-os version 6.2.0. 

This code has been verified to work with the following configuration:

* mbed-os v6.0.2
* Nucleo F429ZI, [at mbed.org](https://os.mbed.com/platforms/ST-Nucleo-F429ZI/), at st.com [for the board itself](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html), and again at st.com for the [device webpage](https://www.st.com/en/microcontrollers-microprocessors/stm32f429zi.html).
* UDP unicast data transfer
* [Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H](https://www.adafruit.com/product/3421).
* Receive mode
* Approximately 3.072 MHz bit clock
* gcc 9-2019-q4-major

### Original Known Working Boards
* NUCLEO_F401RE
* NUCLEO_F411RE
* NUCLEO_F429ZI

### Known Working Boards
* NUCLEO_F429ZI

Note: It is quite possible that other boards will work with little modification, or given the orignial working boards with any modification.

## Driver History

The driver in its original form is one of the first commits in its history. The second commit to the driver information should contain its 

## Comments

I worked with this setup for a while to get it to function properly. I discovered a number of items of interest that may interest anyone who would want to use this.

### Microphone
- The mic sends out left-justified (MSB first) 18 bit in 24 bit output.
- The mic transmits data as 2's complement signed integers.
- The STM32 I2S periperhal supports 16 and 32 bit word sizes. The data after the 18th most-significant-bit is meaningless.

### DMA
- The DMA will handle the bit-reversing, but will not handle half-word issues. This is a minor problem using libraries expecting either 16 bit integers in a contiguous array order or 32 bits.
- The STM32 DMA (as far as I can tell) wants to know the number of _transfers_ whereas the public facing driver wants to know the number of _bytes_. The internal code for the driver should take care of this, but it might prove useful for porting.

### Data processing
- ARM has a nifty library for signal processing. However, without modification, the data straight out of the DMA interrupt with this microphone (at least as far as I know). There are several reasons for this:
    - The data will always come in this form: ```L R L R```, where L and R are left and right samples (regardless of sample word size). This is the nature of I2S. If you only transfer 16 bits, you will still have to transfer a half-word every ```L R``` stride to a different array for processing with the ARM library.
    - If you do transfer them using 32-bit words (as I have), you will have to reverse the upper and lower word halves. ARM has built-in instructions for this, but it still requires doing. 

- You know you have a problem with half-word reversal and/or signedness if, with a sufficiently loud signal, you have what appears to be overflow and underflow. This will look like a peak of a sinusoidal wave dropping a number of samples but still continuing with an upward slope.

### Miscellaneous Tips
- I had a much easier time using wired ethernet with UDP to transfer the data for analysis and testing. The serial async had issues, and I was more interested in making the mic work than async serial.
- Spyder is a great environment for MATLAB-like usage. I used this with a script to do my UDP reception and sampling analysis.
- I found a tone generator for testing with my phone. This will allow you to use your computer to record a signal to verify that the device is not skipping DMA half and full transfer interrupts.

