#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "myringbuffer.h"

// SPI configurations
#define PIN_MISO 8
#define PIN_CS 9
#define PIN_SCK 10
#define PIN_MOSI 11
#define SPI_PORT spi1
#define LDAC_GPIO 7
// GPIO for timing the ISR
#define ISR_GPIO 6

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ timer_hardware_alarm_get_irq_num(timer_hw, ALARM_NUM)

// DDS parameters
#define two32 4294967296.0 // 2^32
#define Fs 44100           // 50000
#define DELAY 26       // 20 // 1/Fs (in microseconds): dus 20 microseconds * 50000 = 1 seconde
    // the DDS units:
    volatile unsigned int phase_accum_main;
volatile unsigned int phase_incr_main = (0.0 * two32) / Fs; // was 800.0

// SPI data
uint16_t DAC_data; // output value

// DAC parameters
//  A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

static void __not_in_flash_func(alarm_irq)(void)
{

    // Assert a GPIO when we enter the interrupt
    //gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;
    uint16_t sample =0;
    my_rb_get(&sample);
    //int dac_value = (sample * 4095) / 255;
    DAC_data = (DAC_config_chan_A | (sample & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);
    DAC_data = (DAC_config_chan_B | (sample & 0xffff));
    spi_write16_blocking(SPI_PORT, &DAC_data, 1);

    // De-assert the GPIO when we leave the interrupt
   // gpio_put(ISR_GPIO, 0);
}

void init_mcp4822()
{
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)

    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_LSB_FIRST);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    gpio_init(LDAC_GPIO);
    gpio_set_dir(LDAC_GPIO, GPIO_OUT);
    gpio_put(LDAC_GPIO, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;
}