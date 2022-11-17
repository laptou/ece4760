/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration calculates an FFT of audio input, and
 * then displays that FFT on a 640x480 VGA display.
 *
 * Core 0 computes and displays the FFT. Core 1 blinks the LED.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 26 ---> Audio input [0-3.3V]
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - ADC channel 0
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Include Pico libraries
#include "pico/multicore.h"
#include "pico/stdlib.h"
// Include hardware libraries
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pt_cornell_rp2040_v1.h"
#include "vga/vga.h"
#include "fft/fft.h"
#include "fpmath/fpmath.h"
#include "fpmath/vecmath.h"
#include "notes.h"

spin_lock_t *fft_data_lock;

// FFT thread
static PT_THREAD(protothread_fft(struct pt *pt))
{
  // Indicate beginning of thread
  PT_BEGIN(pt);

  fft_init();

  printf("starting fft capture\n");

  // Will be used to write dynamic text to screen
  static char freqtext[40];

  while (1)
  {
    // Wait for NUM_SAMPLES samples to be gathered
    // Measure wait time with timer. THIS IS BLOCKING
    dma_channel_wait_for_finish_blocking(FFT_DMA_SAMPLE_CHAN);

    PT_LOCK_WAIT(pt, fft_data_lock);

    // Copy/window elements into a fixed-point array
    for (size_t i = 0; i < NUM_SAMPLES; i++)
    {
      fft_sample_real[i] =
          fixed::from((int)fft_raw_sample_array[i]) * fft_window_lut[i];
      fft_sample_imag[i] = fixed::zero();
    }

    // Restart the sample channel, now that we have our copy of the samples
    dma_channel_start(FFT_DMA_CONTROL_CHAN);

    // Compute the FFT
    fft_fix(fft_sample_real, fft_sample_imag);
    fft_compute_magnitudes();

    // Unlock spinlock
    PT_LOCK_RELEASE(fft_data_lock);

    // A short delay to make sure the other core locks before this
    // one locks the spinlock again (experimentation shows that
    // this is necessary)
    sleep_ms(1);
  }

  PT_END(pt);
}

static PT_THREAD(protothread_vga(struct pt *pt))
{
  PT_BEGIN(pt);

  vga_fg_color(WHITE);
  vga_cursor(250, 0);
  vga_text_size(2);
  vga_write_string("Max freqency:");

  // Will be used to write dynamic text to screen
  static char freqtext[40];

  while (1)
  {
    PT_LOCK_WAIT(pt, fft_data_lock);

    // Display on VGA
    vga_fill_rect(250, 20, 176, 30, BLACK); // red box
    auto note = find_closest_note(fft_max_freq);
    sprintf(freqtext, "%d (%s)", (int)fft_max_freq, note.name.c_str());
    vga_cursor(250, 20);
    vga_write_string(freqtext);

    // Update the FFT display
    for (int i = 5; i < (NUM_SAMPLES >> 1); i++)
    {
      vga_vline(59 + i, 50, 429, BLACK);
      auto height = (int)(fft_sample_real[i] * fixed::from(36));
      vga_vline(59 + i, 479 - height, height, WHITE);
    }

    PT_LOCK_RELEASE(fft_data_lock);
  }

  PT_END(pt);
}

static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);

  printf("rp2040 recorder keyboard v0.1\n");
  static char classifier;

  while (1)
  {
    PT_YIELD_usec(100000);
    sprintf(pt_serial_out_buffer, "freq: %d\n", int(fft_max_freq));
    serial_write;
  }

  PT_END(pt);
}

// Entry point for core 1
void core1_entry()
{
  pt_add_thread(protothread_vga);
  pt_schedule_start;
}

int main()
{
  // Initialize stdio
  stdio_init_all();

  // Initialize the VGA screen
  vga_init();

  ///////////////////////////////////////////////////////////////////////////////
  // ============================== ADC CONFIGURATION ==========================
  //////////////////////////////////////////////////////////////////////////////
  // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
  adc_gpio_init(ADC_PIN);

  // Initialize the ADC harware
  // (resets it, enables the clock, spins until the hardware is ready)
  adc_init();

  // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
  adc_select_input(ADC_CHAN);

  // Setup the FIFO
  adc_fifo_setup(
      true,  // Write each completed conversion to the sample FIFO
      true,  // Enable DMA data request (DREQ)
      1,     // DREQ (and IRQ) asserted when at least 1 sample present
      false, // We won't see the ERR bit because of 8 bit reads; disable.
      true   // Shift each sample to 8 bits when pushing to FIFO
  );

  // Divisor of 0 -> full speed. Free-running capture with the divider is
  // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
  // cycles (div not necessarily an integer). Each conversion takes 96
  // cycles, so in general you want a divider of 0 (hold down the button
  // continuously) or > 95 (take samples less frequently than 96 cycle
  // intervals). This is all timed by the 48 MHz ADC clock. This is setup
  // to grab a sample at 10kHz (48Mhz/10kHz - 1)
  adc_set_clkdiv(ADCCLK / Fs);

  /////////////////////////////////////////////////////////////////////////////////
  // ============================== ADC DMA CONFIGURATION
  // =========================
  /////////////////////////////////////////////////////////////////////////////////

  // Channel configurations
  dma_channel_config c2 = dma_channel_get_default_config(FFT_DMA_SAMPLE_CHAN);
  dma_channel_config c3 = dma_channel_get_default_config(FFT_DMA_CONTROL_CHAN);

  // ADC SAMPLE CHANNEL
  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
  channel_config_set_read_increment(&c2, false);
  channel_config_set_write_increment(&c2, true);
  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&c2, DREQ_ADC);
  // Configure the channel
  dma_channel_configure(
      FFT_DMA_SAMPLE_CHAN,
      &c2,                  // channel config
      fft_raw_sample_array, // dst
      &adc_hw->fifo,        // src
      NUM_SAMPLES,          // transfer count
      false                 // don't start immediately
  );

  // CONTROL CHANNEL
  channel_config_set_transfer_data_size(&c3, DMA_SIZE_32); // 32-bit txfers
  channel_config_set_read_increment(&c3, false);           // no read incrementing
  channel_config_set_write_increment(&c3, false);          // no write incrementing
  channel_config_set_chain_to(&c3, FFT_DMA_SAMPLE_CHAN);   // chain to sample chan

  dma_channel_configure(
      FFT_DMA_CONTROL_CHAN, // Channel to be configured
      &c3,                  // The configuration we just created
      &dma_hw->ch[FFT_DMA_SAMPLE_CHAN]
           .write_addr,        // Write address (channel 0 read address)
      &sample_address_pointer, // Read address (POINTER TO AN ADDRESS)
      1,                       // Number of transfers, in this case each is 4 byte
      false                    // Don't start immediately.
  );

  // Claim and initialize a spinlock
  PT_LOCK_INIT(fft_data_lock, 26, UNLOCKED);

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // Add and schedule core 0 threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_fft);
  pt_schedule_start;
}
