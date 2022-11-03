#ifndef _FFT_H
#define _FFT_H

// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Include Pico libraries
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "../fpmath/fpmath.h"

void fft_fix(fixed fr[], fixed fi[]);
void fft_compute_magnitudes();
void fft_init();

/////////////////////////// ADC configuration ////////////////////////////////
// ADC Channel and pin
#define ADC_CHAN 0
#define ADC_PIN 26
// Number of samples per FFT
#define NUM_SAMPLES 1024
// Number of samples per FFT, minus 1
#define NUM_SAMPLES_M_1 1023
// Length of short (16 bits) minus log2 number of samples (10)
#define SHIFT_AMOUNT 6
// Log2 number of samples
#define LOG2_NUM_SAMPLES 10
// Sample rate (Hz)
#define Fs 10000.0
// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

// DMA channels for sampling ADC (VGA driver uses 0 and 1)
int fft_dma_sample_chan = 2;
int fft_control_chan = 3;

// Max and min macros
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

// Here's where we'll have the DMA channel put ADC samples
uint8_t fft_raw_sample_array[NUM_SAMPLES];

// And here's where we'll copy those samples for FFT calculation
fixed fft_sample_real[NUM_SAMPLES];
fixed fft_sample_imag[NUM_SAMPLES];

// Sine table for the FFT calculation
fixed fft_sine_lut[NUM_SAMPLES];

// Hann window table for FFT calculation
fixed fft_window_lut[NUM_SAMPLES];

// index in the sample array of the largest magnitude sample
size_t fft_max_freq_idx;
// frequency that has the largest measured magnitude
fixed fft_max_freq;

// Pointer to address of start of sample buffer
uint8_t *sample_address_pointer = &fft_raw_sample_array[0];

#endif
