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

// Include VGA graphics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/spi.h"

// Define the LED pin
#define LED 25

#pragma region Fixed point math

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a)*32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

#pragma endregion

// Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0 // 2^32 (a constant)
#define DAC_Fs 40000       // sample rate

// Frequencies
#define CHIRP_FREQ 2300.0
// #define frequency_1 2500.0

// the DDS units - core 1
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_1;
volatile unsigned int phase_incr_main_1 = (unsigned int)((CHIRP_FREQ * two32) / DAC_Fs);
// the DDS units - core 2
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (unsigned int)((CHIRP_FREQ * two32) / DAC_Fs);

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size];

// Values output to DAC
int DAC_output_0;
int DAC_output_1;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1); // maximum amplitude
fix15 attack_inc;                   // rate at which sound ramps up
fix15 decay_inc;                    // rate at which sound ramps down
fix15 current_amplitude_0 = 0;      // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;      // current amplitude (modified in ISR)

// period of timer interrupt in us
#define IRQ_PERIOD 25

// Timing parameters for beeps (units of interrupts)
#define CHIRP_ATTACK_TIME 200
#define CHIRP_DECAY_TIME 200
#define CHIRP_SUSTAIN_TIME 10000

// space in between chirps and syllables
const int CHIRP_SPACE = 260000 / IRQ_PERIOD;
const int SYLLABLE_SPACE = 2000 / IRQ_PERIOD;
const int SYLLABLE_LENGTH = 17000 / IRQ_PERIOD;

typedef enum emit_state
{
    es_wait_for_syllable,
    es_wait_for_chirp,
    es_active,
    es_paused,
} emit_state_t;

// lookup table used for the integration counter function
#define CHIRP_ACCUM_THRESHOLD int2fix15(31)
#define CHIRP_ACCUM_EPSILON 4
#define CHIRP_ACCUM_LUT_SIZE 1024
fix15 CHIRP_ACCUM_LUT[CHIRP_ACCUM_LUT_SIZE];

// State machine variables
volatile emit_state_t es_core_0 = es_wait_for_chirp;
// accumulator that triggers a chirp when its value reaches CHIRP_ACCUM_THRESHOLD
volatile unsigned int es_core_0_chirp_time = 0;
volatile unsigned int es_core_0_irq_time = 0;
volatile unsigned int es_core_0_syllable_count = 0;
volatile unsigned int es_core_0_pause_count = 0;

volatile emit_state_t es_core_1 = es_wait_for_chirp;
volatile unsigned int es_core_1_irq_count = 0;
volatile unsigned int es_core_1_syllable_count = 0;
volatile unsigned int es_core_1_pause_count = 0;

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

#define GPIO_PAUSE_CORE_1 14
#define GPIO_PAUSE_CORE_0 15

// Two variables to store core number
volatile int corenum_0;
volatile int corenum_1;

// Global counter for spinlock experimenting
volatile int global_counter = 0;

// Semaphore
struct pt_sem core_1_go, core_0_go;

// This timer ISR is called on core 1
bool repeating_timer_callback_core_1(struct repeating_timer *t)
{
    es_core_1_irq_count++;

    if (es_core_1 != es_paused)
    {
        if (!gpio_get(GPIO_PAUSE_CORE_1))
        {
            es_core_1_pause_count++;

            if (es_core_1_pause_count >= 20000)
            {
                es_core_1 = es_paused;
                es_core_1_irq_count = 0;
                es_core_1_pause_count = 0;
            }
        }
        else
        {
            es_core_1_pause_count = 0;
        }
    }
    else
    {
        if (!gpio_get(GPIO_PAUSE_CORE_1))
        {
            es_core_1_pause_count++;

            if (es_core_1_pause_count >= 20000)
            {
                es_core_1 = es_wait_for_chirp;
                es_core_1_irq_count = 0;
                es_core_1_pause_count = 0;
            }
        }
        else
        {
            es_core_1_pause_count = 0;
        }
    }

    switch (es_core_1)
    {
    case es_active:
        // DDS phase and sine table lookup
        phase_accum_main_1 += phase_incr_main_1;
        DAC_output_1 = fix2int15(multfix15(current_amplitude_1,
                                           sin_table[phase_accum_main_1 >> 24])) +
                       2048;

        // Ramp up amplitude
        if (es_core_1_irq_count < CHIRP_ATTACK_TIME)
        {
            current_amplitude_1 = (current_amplitude_1 + attack_inc);
        }

        // Ramp down amplitude
        else if (es_core_1_irq_count > SYLLABLE_LENGTH - CHIRP_DECAY_TIME)
        {
            current_amplitude_1 = (current_amplitude_1 - decay_inc);
        }

        // Mask with DAC control bits
        DAC_data_1 = (DAC_config_chan_A | (DAC_output_1 & 0xffff));

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_1, 1);

        // State transition
        if (es_core_1_irq_count == SYLLABLE_LENGTH)
        {
            if (es_core_1_syllable_count >= 8)
            {
                es_core_1 = es_wait_for_chirp;
                es_core_1_syllable_count = 0;
            }
            else
            {
                es_core_1 = es_wait_for_syllable;
                es_core_1_syllable_count++;
            }

            es_core_1_irq_count = 0;
        }
        break;
    case es_wait_for_chirp:
        if (es_core_1_irq_count >= CHIRP_SPACE)
        {
            current_amplitude_1 = 0;
            es_core_1 = es_active;
            es_core_1_irq_count = 0;
        }
        break;
    case es_wait_for_syllable:
        if (es_core_1_irq_count >= SYLLABLE_SPACE)
        {
            current_amplitude_1 = 0;
            es_core_1 = es_active;
            es_core_1_irq_count = 0;
        }
        break;
    }

    // retrieve core number of execution
    corenum_1 = get_core_num();

    return true;
}

// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t)
{
    es_core_0_irq_time++;

    if (es_core_0 != es_paused)
    {
        if (!gpio_get(GPIO_PAUSE_CORE_0))
        {
            es_core_0_pause_count++;

            if (es_core_0_pause_count >= 20000)
            {
                es_core_0 = es_paused;
                es_core_0_irq_time = 0;
                es_core_0_pause_count = 0;
            }
        }
        else
        {
            es_core_0_pause_count = 0;
        }
    }
    else
    {
        if (!gpio_get(GPIO_PAUSE_CORE_0))
        {
            es_core_0_pause_count++;

            if (es_core_0_pause_count >= 20000)
            {
                es_core_0 = es_wait_for_chirp;
                es_core_0_irq_time = 0;
                es_core_0_pause_count = 0;
            }
        }
        else
        {
            es_core_0_pause_count = 0;
        }
    }

    switch (es_core_0)
    {
    case es_active:
        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                                           sin_table[phase_accum_main_0 >> 24])) +
                       2048;

        // Ramp up amplitude
        if (es_core_0_irq_time < CHIRP_ATTACK_TIME)
        {
            current_amplitude_0 = (current_amplitude_0 + attack_inc);
        }

        // Ramp down amplitude
        else if (es_core_0_irq_time > SYLLABLE_LENGTH - CHIRP_DECAY_TIME)
        {
            current_amplitude_0 = (current_amplitude_0 - decay_inc);
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

        // State transition
        if (es_core_0_irq_time >= SYLLABLE_LENGTH)
        {
            if (es_core_0_syllable_count >= 8)
            {
                es_core_0 = es_wait_for_chirp;
                es_core_0_syllable_count = 0;
            }
            else
            {
                es_core_0 = es_wait_for_syllable;
                es_core_0_syllable_count++;
            }

            es_core_0_irq_time = 0;
        }
        break;
    case es_wait_for_chirp:
        es_core_0_chirp_time++;

        // calculate current index into chirp accumulator as
        // (time since last chirp) / (time between chirps) * (length of lookup table)
        size_t chirp_accum_index = fix2int15(
            multfix15(
                divfix(
                    int2fix15(es_core_0_chirp_time),
                    int2fix15(CHIRP_SPACE)),
                int2fix15(CHIRP_ACCUM_LUT_SIZE)));

        if (chirp_accum_index >= CHIRP_ACCUM_LUT_SIZE)
        {
            chirp_accum_index = CHIRP_ACCUM_LUT_SIZE - 1;
        }

        fix15 chirp_accum = CHIRP_ACCUM_LUT[chirp_accum_index];

        // printf("waiting for chirp (time = %d, idx = %d, accum = %f)", es_core_0_chirp_time, chirp_accum_index, fix2float15(chirp_accum));

        if (chirp_accum >= CHIRP_ACCUM_THRESHOLD)
        {
            // printf("broke chirp thresh\n");
            current_amplitude_0 = 0;
            es_core_0 = es_active;
            es_core_0_irq_time = 0;
            es_core_0_chirp_time = 0;
        }

        // stdio_flush();
        break;
    case es_wait_for_syllable:
        if (es_core_0_irq_time >= SYLLABLE_SPACE)
        {
            current_amplitude_0 = 0;
            es_core_0 = es_active;
            es_core_0_irq_time = 0;
        }
        break;
    }

    // retrieve core number of execution
    corenum_0 = get_core_num();

    return true;
}

// This thread runs on core 1
static PT_THREAD(protothread_core_1(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);
    while (1)
    {
        // Wait for signal
        PT_SEM_SAFE_WAIT(pt, &core_1_go);
        // Turn off LED
        gpio_put(LED, 0);
        // Increment global counter variable
        for (int i = 0; i < 10; i++)
        {
            global_counter += 1;
            sleep_ms(250);
            printf("Core 1: %d, ISR core: %d\n", global_counter, corenum_1);
        }
        printf("\n\n");
        // signal other core
        PT_SEM_SAFE_SIGNAL(pt, &core_0_go);
    }
    // Indicate thread end
    PT_END(pt);
}

// This thread runs on core 0
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);
    while (1)
    {
        // Wait for signal
        PT_SEM_SAFE_WAIT(pt, &core_0_go);
        // Turn on LED
        gpio_put(LED, 1);
        // Increment global counter variable
        for (int i = 0; i < 10; i++)
        {
            global_counter += 1;
            sleep_ms(250);
            printf("Core 0: %d, ISR core: %d\n", global_counter, corenum_0);
        }
        printf("\n\n");
        // signal other core
        PT_SEM_SAFE_SIGNAL(pt, &core_1_go);
    }
    // Indicate thread end
    PT_END(pt);
}

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
#define ADC_Fs 10000.0
// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

// DMA channels for sampling ADC (VGA driver uses 0 and 1)
int sample_chan = 2;
int control_chan = 3;

// Max and min macros
#define max(a, b) ((a > b) ? a : b)
#define min(a, b) ((a < b) ? a : b)

// 0.4 in fixed point (used for alpha max plus beta min)
fix15 zero_point_4 = float2fix15(0.4);

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES];
// And here's where we'll copy those samples for FFT calculation
fix15 fr[NUM_SAMPLES];
fix15 fi[NUM_SAMPLES];

// Sine table for the FFT calculation
fix15 Sinewave[NUM_SAMPLES];
// Hann window table for FFT calculation
fix15 window[NUM_SAMPLES];

// Pointer to address of start of sample buffer
uint8_t *sample_address_pointer = &sample_array[0];

// Peforms an in-place FFT. For more information about how this
// algorithm works, please see https://vanhunteradams.com/FFT/FFT.html
void FFTfix(fix15 fr[], fix15 fi[])
{

    unsigned short m;  // one of the indices being swapped
    unsigned short mr; // the other index being swapped (r for reversed)
    fix15 tr, ti;      // for temporary storage while swapping, and during iteration

    int i, j; // indices being combined in Danielson-Lanczos part of the algorithm
    int L;    // length of the FFT's being combined
    int k;    // used for looking up trig values from sine table

    int istep; // length of the FFT which results from combining two FFT's

    fix15 wr, wi; // trigonometric values from lookup table
    fix15 qr, qi; // temporary variables used during DL part of the algorithm

    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here:
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    for (m = 1; m < NUM_SAMPLES_M_1; m++)
    {
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
    while (L < NUM_SAMPLES)
    {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L << 1;
        // For each element in the FFT's that are being combined . . .
        for (m = 0; m < L; ++m)
        {
            // Lookup the trig values for that element
            j = m << k;                         // index of the sine table
            wr = Sinewave[j + NUM_SAMPLES / 4]; // cos(2pi m/N)
            wi = -Sinewave[j];                  // sin(2pi m/N)
            wr >>= 1;                           // divide by two
            wi >>= 1;                           // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i = m; i < NUM_SAMPLES; i += istep)
            {
                // j gets the index of the FFT element being combined with i
                j = i + L;
                // compute the trig terms (bottom half of the above matrix)
                tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]);
                ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]);
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

// current time in ms
unsigned int now = 0;
// resolution of now in ms
#define NOW_TIMESTEP 20

typedef enum listen_state
{
    ls_active = 0,
    ls_wait,
} listen_state_t;

volatile listen_state_t listen_state = ls_active;
volatile unsigned int ls_last_time = 0;
volatile unsigned int ls_chirp_count_external = 0;
volatile unsigned int ls_chirp_count_core_0 = 0;
volatile unsigned int ls_chirp_count_core_1 = 0;

volatile emit_state_t ls_es_core_0_before;
volatile emit_state_t ls_es_core_0_after;
volatile emit_state_t ls_es_core_0_next;

volatile emit_state_t ls_es_core_1_before;
volatile emit_state_t ls_es_core_1_after;
volatile emit_state_t ls_es_core_1_next;

// Runs on core 0
static PT_THREAD(protothread_fft(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt);
    printf("Starting capture\n");

    ls_es_core_0_before = es_core_0;
    ls_es_core_1_before = es_core_1;

    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan));
    // Start the ADC
    adc_run(true);

    // Declare some static variables
    static int height;         // for scaling display
    static float max_freqency; // holds max frequency
    static int i;              // incrementing loop variable

    static fix15 max_fr;   // temporary variable for max freq calculation
    static int max_fr_dex; // index of max frequency

    // Write some text to VGA
    setTextColor(WHITE);
    setCursor(65, 0);
    setTextSize(1);
    writeString("Raspberry Pi Pico");
    setCursor(65, 10);
    writeString("FFT demo");
    setCursor(65, 20);
    writeString("Hunter Adams");
    setCursor(65, 30);
    writeString("vha3@cornell.edu");
    setCursor(250, 0);
    setTextSize(2);
    writeString("Max freqency:");

    // Will be used to write dynamic text to screen
    static char freqtext[40];
    static char chirptext[40];

    setCursor(250, 40);
    writeString("chirp debug");

    while (1)
    {
        // Wait for NUM_SAMPLES samples to be gathered
        // Measure wait time with timer. THIS IS BLOCKING
        dma_channel_wait_for_finish_blocking(sample_chan);

        ls_es_core_0_after = es_core_0;

        // Copy/window elements into a fixed-point array
        for (i = 0; i < NUM_SAMPLES; i++)
        {
            fr[i] = multfix15(int2fix15((int)sample_array[i]), window[i]);
            fi[i] = (fix15)0;
        }

        // Zero max frequency and max frequency index
        max_fr = 0;
        max_fr_dex = 0;

        ls_es_core_0_next = es_core_0;
        ls_es_core_1_next = es_core_1;

        // Restart the sample channel, now that we have our copy of the samples
        dma_channel_start(control_chan);

        // Compute the FFT
        FFTfix(fr, fi);

        // Find the magnitudes (alpha max plus beta min)
        for (int i = 0; i < (NUM_SAMPLES >> 1); i++)
        {
            // get the approx magnitude
            fr[i] = abs(fr[i]);
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = max(fr[i], fi[i]) +
                    multfix15(min(fr[i], fi[i]), zero_point_4);

            // Keep track of maximum
            if (fr[i] > max_fr && i > 4)
            {
                max_fr = fr[i];
                max_fr_dex = i;
            }
        }

        // Compute max frequency in Hz
        max_freqency = max_fr_dex * (ADC_Fs / NUM_SAMPLES);

        // Display on VGA
        fillRect(250, 20, 176, 20, BLACK); // red box
        sprintf(freqtext, "%d", (int)max_freqency);
        setCursor(250, 20);
        setTextSize(2);
        writeString(freqtext);

        // Update the FFT display
        for (int i = 5; i < (NUM_SAMPLES >> 1); i++)
        {
            drawVLine(59 + i, 100, 379, BLACK);
            height = fix2int15(multfix15(fr[i], int2fix15(36)));
            drawVLine(59 + i, 479 - height, height, WHITE);
        }

        fillRect(250, 60, 350, 40, RED);

        unsigned int listen_state_duration = ls_last_time - now;
        switch (listen_state)
        {
        case ls_wait:
            if (abs(max_freqency - 2300) < 50)
            {
                listen_state = ls_active;
                ls_last_time = now;

                bool core_0_silent = (ls_es_core_0_before == es_wait_for_chirp) && (ls_es_core_0_after == es_wait_for_chirp);
                bool core_1_silent = (ls_es_core_1_before == es_wait_for_chirp) && (ls_es_core_1_after == es_wait_for_chirp);

                if (core_0_silent)
                {
                    fix15 es_core_0_chirp_accum = CHIRP_ACCUM_LUT[es_core_0_chirp_time];
                    es_core_0_chirp_accum += CHIRP_ACCUM_EPSILON;

                    printf("external chirp detected, accum (pre epsilon) = %f, time = %d\n", fix2float15(es_core_0_chirp_accum), es_core_0_chirp_time);

                    while (CHIRP_ACCUM_LUT[es_core_0_chirp_time] < es_core_0_chirp_accum)
                    {
                        es_core_0_chirp_time++;

                        if (es_core_0_chirp_time >= CHIRP_ACCUM_LUT_SIZE)
                        {
                            es_core_0_chirp_time = 0;
                            break;
                        }
                    }

                    printf("external chirp detected, accum (post epsilon) = %f, time = %d\n", fix2float15(es_core_0_chirp_accum), es_core_0_chirp_time);

                    ls_chirp_count_external++;
                }
            }
            break;
        case ls_active:
            setCursor(250, 80);

            sprintf(chirptext, "chirp detected");
            writeString(chirptext);

            if (abs(max_freqency - 2300) > 50 && listen_state_duration > 100)
            {
                listen_state = ls_wait;
                ls_last_time = now;
                puts("chirp detected\n");
            }
            break;
        }
        sprintf(chirptext, "ext %d c0 %d c1 %d ls %d now %d", ls_chirp_count_external, ls_chirp_count_core_0, ls_chirp_count_core_1, listen_state, now);
        setCursor(250, 60);
        writeString(chirptext);

        ls_es_core_0_before = ls_es_core_0_next;
        ls_es_core_1_before = ls_es_core_1_next;
    }
    PT_END(pt);
}

static PT_THREAD(protothread_blink(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt);
    while (1)
    {
        // Toggle LED, then wait half a second
        gpio_put(LED, !gpio_get(LED));
        PT_YIELD_usec(500000);
    }
    PT_END(pt);
}

// Core 1 entry point (main() for core 1)
void core1_entry()
{

    // create an alarm pool on core 1
    alarm_pool_t *core1pool;
    core1pool = alarm_pool_create(2, 16);

    // Create a repeating timer that calls repeating_timer_callback.
    struct repeating_timer timer_core_1;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    alarm_pool_add_repeating_timer_us(core1pool, -25,
                                      repeating_timer_callback_core_1, NULL, &timer_core_1);

    // Add thread to core 1
    pt_add_thread(protothread_core_1);

    // Add and schedule threads
    // pt_add_thread(protothread_blink);
    pt_schedule_start;
}

bool now_timer_callback(repeating_timer_t *_)
{
    now += NOW_TIMESTEP;
    return true;
}

// Core 0 entry point
int main()
{

    // Initialize stdio
    stdio_init_all();
    printf("\n");

    // Initialize the VGA screen
    initVGA();

    // initialize now timer
    static struct repeating_timer now_timer;
    add_repeating_timer_ms(-NOW_TIMESTEP, now_timer_callback, NULL, &now_timer);

    // Map LED to GPIO port, make it low
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    for (size_t i = 0; i < CHIRP_ACCUM_LUT_SIZE; i++)
    {
        CHIRP_ACCUM_LUT[i] = float2fix15(sqrt((float)(i + 1)));
    }

    printf("initialized lut, %d, %d\n", CHIRP_ACCUM_LUT[0], CHIRP_ACCUM_LUT[CHIRP_ACCUM_LUT_SIZE - 1]);

#pragma region ADC config

    printf("initializing adc\n");

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
    adc_set_clkdiv(ADCCLK / ADC_Fs);

    // Populate the sine table and Hann window table
    for (int ii = 0; ii < NUM_SAMPLES; ii++)
    {
        Sinewave[ii] = float2fix15(sin(6.283 * ((float)ii) / (float)NUM_SAMPLES));
        window[ii] = float2fix15(0.5 * (1.0 - cos(6.283 * ((float)ii) / ((float)NUM_SAMPLES))));
    }

#pragma endregion

#pragma region ADC DMA config
    printf("initializing adc dma\n");

    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);

    // ADC SAMPLE CHANNEL
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
                          &c2,           // channel config
                          sample_array,  // dst
                          &adc_hw->fifo, // src
                          NUM_SAMPLES,   // transfer count
                          false          // don't start immediately
    );

    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32); // 32-bit txfers
    channel_config_set_read_increment(&c3, false);           // no read incrementing
    channel_config_set_write_increment(&c3, false);          // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);           // chain to sample chan

    dma_channel_configure(
        control_chan,                        // Channel to be configured
        &c3,                                 // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr, // Write address (channel 0 read address)
        &sample_address_pointer,             // Read address (POINTER TO AN ADDRESS)
        1,                                   // Number of transfers, in this case each is 4 byte
        false                                // Don't start immediately.
    );

#pragma endregion

#pragma region DAC config
    printf("initializing dac\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(CHIRP_ATTACK_TIME));
    decay_inc = divfix(max_amplitude, int2fix15(CHIRP_DECAY_TIME));

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    for (int ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // Initialize the intercore semaphores
    PT_SEM_SAFE_INIT(&core_0_go, 1);
    PT_SEM_SAFE_INIT(&core_1_go, 0);
#pragma endregion

#pragma region Pause button config
    printf("initializing pause button\n");

    gpio_init(GPIO_PAUSE_CORE_0);
    gpio_set_dir(GPIO_PAUSE_CORE_0, GPIO_IN);
    gpio_pull_up(GPIO_PAUSE_CORE_0);

    gpio_init(GPIO_PAUSE_CORE_1);
    gpio_set_dir(GPIO_PAUSE_CORE_1, GPIO_IN);
    gpio_pull_up(GPIO_PAUSE_CORE_1);
#pragma endregion

    printf("initializing core 0 interrupt\n");

    // Create a repeating timer that calls
    // repeating_timer_callback (defaults core 0)
    static struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    add_repeating_timer_us(-IRQ_PERIOD,
                           repeating_timer_callback_core_0, NULL, &timer_core_0);

    printf("f\n");

    // Desynchronize the beeps
    sleep_ms(500);

    printf("adding protothreads\n");

    // Launch core 1
    multicore_launch_core1(core1_entry);

    // Add and schedule core 0 threads
    pt_add_thread(protothread_fft);

    // Add core 0 threads
    pt_add_thread(protothread_core_0);

    printf("initializing core 0 thread scheduler\n");

    // Start scheduling core 0 threads
    pt_schedule_start;
}
