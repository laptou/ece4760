#include "fft.h"
#include "../fpmath/vecmath.h"

// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Include Pico libraries
#include "pico/multicore.h"
#include "pico/stdlib.h"

void fft_init()
{
  // Populate the sine table and Hann window table
  int ii;
  for (ii = 0; ii < NUM_SAMPLES; ii++) {
    fft_sine_lut[ii] =
        fixed::from(sinf(6.283f * ((float)ii) / (float)NUM_SAMPLES));
    fft_window_lut[ii] = fixed::from(
        0.5f * (1.0f - cosf(6.283 * ((float)ii) / ((float)NUM_SAMPLES)))
    );
  }
}

// Peforms an in-place FFT. For more information about how this
// algorithm works, please see https://vanhunteradams.com/FFT/FFT.html
void fft_fix(fixed fr[], fixed fi[])
{
  unsigned short m;  // one of the indices being swapped
  unsigned short mr; // the other index being swapped (r for reversed)
  fixed tr, ti; // for temporary storage while swapping, and during iteration

  int i, j; // indices being combined in Danielson-Lanczos part of the algorithm
  int L;    // length of the FFT's being combined
  int k;    // used for looking up trig values from sine table

  int istep; // length of the FFT which results from combining two FFT's

  fixed wr, wi; // trigonometric values from lookup table
  fixed qr, qi; // temporary variables used during DL part of the algorithm

  //////////////////////////////////////////////////////////////////////////
  ////////////////////////// BIT REVERSAL //////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  // Bit reversal code below based on that found here:
  // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
  for (m = 1; m < NUM_SAMPLES_M_1; m++) {
    // swap odd and even bits
    mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
    // swap consecutive pairs
    mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
    // swap nibbles ...
    mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
    // swap bytes
    mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
    // shift down mr
    mr >>= SHIFT_AMOUNT;
    // don't swap that which has already been swapped
    if (mr <= m)
      continue;
    // swap the bit-reveresed indices
    tr = fr[m];
    fr[m] = fr[mr];
    fr[mr] = tr;
    ti = fi[m];
    fi[m] = fi[mr];
    fi[mr] = ti;
  }
  //////////////////////////////////////////////////////////////////////////
  ////////////////////////// Danielson-Lanczos //////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  // Adapted from code by:
  // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
  // Length of the FFT's being combined (starts at 1)
  L = 1;
  // Log2 of number of samples, minus 1
  k = LOG2_NUM_SAMPLES - 1;
  // While the length of the FFT's being combined is less than the number
  // of gathered samples . . .
  while (L < NUM_SAMPLES) {
    // Determine the length of the FFT which will result from combining two
    // FFT's
    istep = L << 1;
    // For each element in the FFT's that are being combined . . .
    for (m = 0; m < L; ++m) {
      // Lookup the trig values for that element
      j = m << k;                             // index of the sine table
      wr = fft_sine_lut[j + NUM_SAMPLES / 4]; // cos(2pi m/N)
      wi = -fft_sine_lut[j];                  // sin(2pi m/N)
      wr = wr >> 1;                           // divide by two
      wi = wi >> 1;                           // divide by two

      // i gets the index of one of the FFT elements being combined
      for (i = m; i < NUM_SAMPLES; i += istep) {
        // j gets the index of the FFT element being combined with i
        j = i + L;
        // compute the trig terms (bottom half of the above matrix)
        // tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]) ;
        // ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]) ;
        tr = wr * fr[j] - wi * fi[j];
        ti = wr * fr[j] + wi * fi[j];

        // divide ith index elements by two (top half of above matrix)
        qr = fr[i] >> 1;
        qi = fi[i] >> 1;
        // compute the new values at each index
        fr[j] = qr - tr;
        fi[j] = qi - ti;
        fr[i] = qr + tr;
        fi[i] = qi + ti;
      }
    }
    --k;
    L = istep;
  }
}

void fft_compute_magnitudes()
{
  auto max_magnitude = fixed::zero();
  // Find the magnitudes (alpha max plus beta min)
  for (size_t i = 0; i < (NUM_SAMPLES >> 1); i++) {
    // get the approx magnitude
    fft_sample_real[i] = fft_sample_real[i].abs();
    fft_sample_imag[i] = fft_sample_imag[i].abs();

    vec2 fft_sample = {fft_sample_real[i], fft_sample_imag[i]};

    // reuse fr to hold magnitude
    fft_sample_real[i] = fft_sample.norm_ambm();

    // Keep track of maximum
    if (fft_sample_real[i] > max_magnitude && i > 4) {
      max_magnitude = fft_sample_real[i];
      fft_max_freq_idx = i;
    }
  }

  // Compute max frequency in Hz
  fft_max_freq = fixed::from((int)fft_max_freq_idx) * fixed::from((float)(Fs / NUM_SAMPLES));
}
