/*
 * main.c - part of wm8731_i2s. Simple driver for WM8731 I2S codec
 * WM8731 requires full-duplex operation so the generic I2S
 * output driver provided for RPi RP2040 will not work for it.
 * 10-28-21 E. Brombaugh
 * Based on original ideas from
 * https://gist.github.com/jonbro/3da573315f066be8ea390db39256f9a7
 * and the pico-extras library.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "i2s_fulldup.pio.h"
#include "wm8731.h"

#define SAMPLES_PER_BUFFER 128

/* I2S comes out on these pins */
#define I2S_DATA_PIN_BASE 12		// data out, data in on pin+1
#define I2S_CLK_PIN_BASE 10			// BCLK, LRCK on pin+1
#define I2S_MCLK_PIN 21				// MCLK

#define LED_PIN 25

/* resources we use */
PIO pio;
uint sm;
uint dma_chan_input, dma_chan_output;
uint ib_idx, ob_idx;
uint32_t input_buf[2*SAMPLES_PER_BUFFER], output_buf[2*SAMPLES_PER_BUFFER],
	xfer_buf[SAMPLES_PER_BUFFER];

/*
 * IRQ0 handler - used only for I2S input
 * ATM this is not double-buffered, but it would be prudent to
 * do so if adding code to compute the next buffer.
 */
void dma_input_handler(void)
{
	/* Clear IRQ for I2S input */
    dma_irqn_acknowledge_channel(DMA_IRQ_0, dma_chan_input);
	
	/* save previous buffer */
	memcpy(xfer_buf, &input_buf[ib_idx*SAMPLES_PER_BUFFER],
		SAMPLES_PER_BUFFER*sizeof(uint32_t));

	/* reset write address to start of next buffer */
	ib_idx ^= 1;
	dma_channel_set_write_addr(dma_chan_input,
		&input_buf[ib_idx*SAMPLES_PER_BUFFER],
		true
	);
	
	/* start next transfer sequence */
	dma_channel_start(dma_chan_input);
}

/*
 * IRQ1 handler - used only for I2S output
 * ATM this is not double-buffered, but it would be prudent to
 * do so if adding code to compute the next buffer.
 */
void dma_output_handler()
{
	/* Clear IRQ for I2S output */
    dma_irqn_acknowledge_channel(DMA_IRQ_1, dma_chan_output);
	
	/* fill previous buffer with new data */
	memcpy(&output_buf[ob_idx*SAMPLES_PER_BUFFER], xfer_buf,
		SAMPLES_PER_BUFFER*sizeof(uint32_t));

	/* reset read address to start of next buffer */
	ob_idx ^= 1;
	dma_channel_set_read_addr(dma_chan_output,
		&output_buf[ob_idx*SAMPLES_PER_BUFFER],
		true
	);
	
	/* start next transfer sequence */
	dma_channel_start(dma_chan_output);
}

/*
 * main prog
 */
int main()
{
	int32_t i;
	int16_t *obuf = (int16_t *)output_buf;
	int16_t *ibuf = (int16_t *)input_buf;
	
    stdio_init_all();
	printf("\n\nwm8731_i2s - test WM8731 I2S driver\n");
	
	printf("Input buffer at 0x%08X\n", input_buf);

	/* load I2S output buffer with rising saw */
	for(i=0;i<2*SAMPLES_PER_BUFFER;i++)
	{
		/* right chl is 1st */
        int16_t wave = 32767 * cosf(i * 2 * (float) (M_PI / SAMPLES_PER_BUFFER));
		*obuf++ = wave;
		
		/* left chl is 2nd */
        wave = 32767 * sinf(i * 2 * (float) (M_PI / SAMPLES_PER_BUFFER));
		*obuf++ = wave;
	}
	printf("Prepped output buffer at 0x%08X\n", output_buf);
	
	/* init codec */
	WM8731_Init();
	printf("Codec Initialized\n");

    /* set up PIO */
    pio = pio0;
    uint offset = pio_add_program(pio, &i2s_fulldup_program);
    printf("loaded program at offset: %i\n", offset);
    sm = pio_claim_unused_sm(pio, true);
    printf("claimed sm: %i\n", sm);
	
	/* compute PIO divider for desired sample rate */
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    printf("System clock %u Hz\n", (uint) system_clock_frequency);
    uint32_t sample_freq = 48000;
    printf("Target sample freq %d\n", sample_freq);
    uint32_t divider = system_clock_frequency * 2 / sample_freq; // avoid arithmetic overflow
    divider = divider & ~(0x1ff); // mask off bottom 9 for exact MCLK/BCLK ratio of 4.0
    assert(divider < 0x1000000);
    printf("PIO clock divider 0x%x/256\n", divider);
	printf("Actual sample freq = %d\n", 2 * system_clock_frequency / divider);
	
	/* start the PIO and set the divider */
    i2s_fulldup_program_init(pio, sm, offset, I2S_DATA_PIN_BASE, I2S_CLK_PIN_BASE);
    pio_sm_set_enabled(pio, sm, true);
    pio_sm_set_clkdiv_int_frac(pio, sm, divider >> 8u, divider & 0xffu);
	
	/* try to generate an MCLK on GPIO */
	gpio_set_function(I2S_MCLK_PIN, GPIO_FUNC_GPCK);
	clock_gpio_init(I2S_MCLK_PIN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, divider>>9);
	printf("MCLK at %d Hz\n", system_clock_frequency/(divider>>9));
	
    /* configure dma channel for input */
	ib_idx = 0;
    dma_chan_input = dma_claim_unused_channel(true);
	printf("DMA input using chl %d\n", dma_chan_input);
    dma_channel_config c = dma_channel_get_default_config(dma_chan_input);
    channel_config_set_read_increment(&c,false);
    channel_config_set_write_increment(&c,true);
    channel_config_set_dreq(&c,pio_get_dreq(pio,sm,false));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

    dma_channel_configure(
        dma_chan_input,
        &c,
        input_buf, 			// Destination pointer
        &pio->rxf[sm], 		// Source pointer
        SAMPLES_PER_BUFFER, // Number of transfers
        true				// Start immediately
    );
    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan_input, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_input_handler);
    irq_set_enabled(DMA_IRQ_0, true);
	
    /* configure dma channel for output */
	ob_idx = 0;
	dma_chan_output = dma_claim_unused_channel(true);
	printf("DMA output using chl %d\n", dma_chan_output);
    dma_channel_config cc = dma_channel_get_default_config(dma_chan_output);
    channel_config_set_read_increment(&cc,true);
    channel_config_set_write_increment(&cc,false);
    channel_config_set_dreq(&cc,pio_get_dreq(pio,sm,true));
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    dma_channel_configure(
        dma_chan_output,
        &cc,
        &pio->txf[sm],		// Destination pointer
        output_buf,			// Source pointer
        SAMPLES_PER_BUFFER,	// Number of transfers
        true				// Start immediately
    );
    dma_channel_set_irq1_enabled(dma_chan_output, true);
    
	/* enable IRQ handler for dma output */
    irq_set_exclusive_handler(DMA_IRQ_1, dma_output_handler);
    irq_set_enabled(DMA_IRQ_1, true);

	/* blink! */
	uint8_t led = 0;
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_put(LED_PIN, (led++)&1);
	
	/* loop here forever */
	printf("Looping\n\n");
    while(true)
    {
		printf("DMA Write Addr 0x%08X ", dma_hw->ch[dma_chan_input].write_addr);
		printf("DMA Read Addr 0x%08X ", dma_hw->ch[dma_chan_output].read_addr);
		printf("\r");
		gpio_put(LED_PIN, (led++)&1);
        sleep_ms(100);
	}
    return 0;
}