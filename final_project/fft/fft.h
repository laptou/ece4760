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
int sample_chan = 2;
int control_chan = 3;

// Max and min macros
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

// Absolute Value
// 0.4 in fixed point (used for alpha max plus beta min)
fixed zero_point_4 = fixed::from(0.4f);

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES];
// And here's where we'll copy those samples for FFT calculation
fixed fr[NUM_SAMPLES];
fixed fi[NUM_SAMPLES];

// Sine table for the FFT calculation
fixed sine_lut[NUM_SAMPLES];
// Hann window table for FFT calculation
fixed window[NUM_SAMPLES];

// Pointer to address of start of sample buffer
uint8_t *sample_address_pointer = &sample_array[0];
