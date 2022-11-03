/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 *
 * HARDWARE CONNECTIONS
 *   - GPIO 4 ---> PWM output 1
 *   - GPIO 5 ---> PWM output 2

 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"

#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))

// Arrays in which raw measurements will be stored
fix15 acceleration_sample[3], gyro_sample[3];

// Synthetic angle is computed from acceleration and gyro using complementary
// filter
fix15 synthetic_angle;

// character array
char screentext[40];

// draw speed
int threshold = 10;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;

// Some paramters for PWM

// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz.
#define WRAPVAL 5000
#define CLKDIV 25.0

// Variable to hold PWM slice number
uint slice_num;

// PID parameters
volatile fix15 REACTION_P = float2fix15(4000);
volatile fix15 REACTION_I = float2fix15(4);
volatile fix15 REACTION_D = float2fix15(0);

// Set point
volatile fix15 set_angle = int2fix15(1);

fix15 reaction_err;
fix15 reaction_err_prev;
fix15 reaction_err_der;
fix15 reaction_err_int;

// PWM duty cycle
volatile int pwm_out;
volatile int pwm_out_prev;

// low-passed version pf pwm_out for display on monitor
int pwm_out_disp;
fix15 reaction_err_disp;
fix15 reaction_err_der_disp;
fix15 reaction_err_int_disp;

int sign(int x)
{
    return (x > 0) - (x < 0);
}

// Interrupt service routine
void on_pwm_wrap()
{
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration_sample, gyro_sample);

    // Accelerometer angle (degrees - 15.16 fixed point)
    fix15 accel_angle = -multfix15(divfix(acceleration_sample[0], acceleration_sample[1]), oneeightyoverpi);

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    fix15 gyro_angle_delta = multfix15(gyro_sample[2], zeropt001);

    // Complementary angle (degrees - 15.16 fixed point)
    synthetic_angle = multfix15(synthetic_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // desired angle is zero, so our error is (desired - current)
    reaction_err = set_angle - synthetic_angle;

    // if (reaction_err < 0)
    // {
    //     set_angle -= float2fix15(0.0001);
    // }
    // else
    // {
    //     set_angle += float2fix15(0.0001);
    // }

    reaction_err_der = reaction_err - reaction_err_prev;

    reaction_err_int += reaction_err;

    if (reaction_err_int > int2fix15(3500))
    {
        reaction_err_int = int2fix15(3500);
    }
    else if (reaction_err_int < int2fix15(-3500))
    {
        reaction_err_int = int2fix15(-3500);
    }

    // if (sign(reaction_err) != sign(reaction_err_prev))
    // {
    //     reaction_err_int = 0;
    // }

    fix15 output_value =
        multfix15(reaction_err, REACTION_P) +
        multfix15(reaction_err_der, REACTION_D) +
        multfix15(reaction_err_int, REACTION_I);

    pwm_out = max(-5000, min(5000, fix2int15(output_value)));

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Update duty cycle
    if (pwm_out != pwm_out_prev)
    {
        pwm_out_prev = pwm_out;

        pwm_set_chan_level(slice_num, PWM_CHAN_B, -min(0, pwm_out));
        pwm_set_chan_level(slice_num, PWM_CHAN_A, max(0, pwm_out));
    }

    reaction_err_prev = reaction_err;
}

// Thread that draws to VGA display
static PT_THREAD(protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt);

    // We will start drawing at column 81
    static int plot_xcoord = 81;

    // Rescale the measurements for display
    static const int display_range = 140; // (looks nice on VGA)
    static const float sample_min = -250.;
    static const float sample_max = 250.;
    static const float sample_range = sample_max - sample_min; // (+/- 250)

    // Control rate of drawing
    static int throttle;

    // Draw the static aspects of the display
    setTextSize(1);
    setTextColor(WHITE);

    int plot_ycoord = 320;

    // Draw bottom plot
    drawHLine(75, plot_ycoord + display_range, 5, CYAN);
    drawHLine(75, plot_ycoord + display_range / 2, 5, CYAN);
    drawHLine(75, plot_ycoord, 5, CYAN);
    drawVLine(80, plot_ycoord, display_range, CYAN);
    sprintf(screentext, "0");
    setCursor(50, plot_ycoord + display_range / 2 - 5);
    writeString(screentext);
    sprintf(screentext, "+2");
    setCursor(50, plot_ycoord);
    writeString(screentext);
    sprintf(screentext, "-2");
    setCursor(50, plot_ycoord + display_range - 5);
    writeString(screentext);

    plot_ycoord = 175;

    // Draw middle plot
    drawHLine(75, plot_ycoord + display_range, 5, CYAN);
    drawHLine(75, plot_ycoord + display_range / 2, 5, CYAN);
    drawHLine(75, plot_ycoord, 5, CYAN);
    drawVLine(80, plot_ycoord, display_range, CYAN);
    sprintf(screentext, "0");
    setCursor(50, plot_ycoord + display_range / 2 - 5);
    writeString(screentext);
    sprintf(screentext, "+250");
    setCursor(45, plot_ycoord);
    writeString(screentext);
    sprintf(screentext, "-250");
    setCursor(45, plot_ycoord + display_range - 5);
    writeString(screentext);

    plot_ycoord = 30;

    // Draw top plot
    drawHLine(75, plot_ycoord + display_range, 5, CYAN);
    drawHLine(75, plot_ycoord + display_range / 2, 5, CYAN);
    drawHLine(75, plot_ycoord, 5, CYAN);
    drawVLine(80, plot_ycoord, display_range, CYAN);
    sprintf(screentext, "0deg");
    setCursor(50, plot_ycoord + display_range / 2 - 5);
    writeString(screentext);
    sprintf(screentext, "+90");
    setCursor(45, plot_ycoord);
    writeString(screentext);
    sprintf(screentext, "-90");
    setCursor(45, plot_ycoord + display_range - 5);
    writeString(screentext);

    setCursor(100, 10);
    writeString("synthetic angle:");
    setCursor(100, 20);
    writeString("setpoint:");

    while (true)
    {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold)
        {
            // Zero drawspeed controller
            throttle = 0;

            pwm_out_disp += (pwm_out - pwm_out_disp) >> 6;

            reaction_err_disp += (reaction_err - reaction_err_disp) >> 2;
            reaction_err_der_disp += (reaction_err_der - reaction_err_der_disp) >> 2;
            reaction_err_int_disp += (reaction_err_int - reaction_err_int_disp) >> 2;

            // Erase a column
            drawVLine(plot_xcoord, 20, 480, BLACK);

            plot_ycoord = 320;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(acceleration_sample[0]) * 120.0) - sample_min) / sample_range)), WHITE);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(acceleration_sample[1]) * 120.0) - sample_min) / sample_range)), RED);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(acceleration_sample[2]) * 120.0) - sample_min) / sample_range)), GREEN);

            plot_ycoord = 175;

            // Draw middle plot
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(gyro_sample[0])) - sample_min) / sample_range)), WHITE);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(gyro_sample[1])) - sample_min) / sample_range)), RED);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(gyro_sample[2])) - sample_min) / sample_range)), GREEN);

            plot_ycoord = 30;

            // Draw top plot
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)((fix2float15(synthetic_angle) * 1.333) - sample_min) / sample_range)), WHITE);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)(((float)pwm_out_disp / 42.0) - sample_min) / sample_range)), RED);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)(fix2float15(reaction_err_disp) * 10.0 - sample_min) / sample_range)), BLUE);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)(fix2float15(reaction_err_der_disp) * 100.0 - sample_min) / sample_range)), CYAN);
            drawPixel(plot_xcoord, plot_ycoord + display_range - (int)(display_range * ((float)(fix2float15(reaction_err_int_disp) * 0.1 - sample_min) / sample_range)), MAGENTA);

            // Update horizontal cursor
            if (plot_xcoord < 609)
            {
                plot_xcoord += 1;
            }
            else
            {
                plot_xcoord = 81;
            }
        }

        fillRect(200, 10, 50, 20, BLACK);

        sprintf(screentext, "%.3f", fix2float15(synthetic_angle));
        setCursor(200, 10);
        writeString(screentext);

        sprintf(screentext, "%.3f", fix2float15(set_angle));
        setCursor(200, 20);
        writeString(screentext);
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    static char classifier;
    static int test_in;
    static float float_in;
    while (1)
    {
        sprintf(pt_serial_out_buffer, "input a command: ");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%c", &classifier);

        // num_independents = test_in ;
        if (classifier == 't')
        {
            sprintf(pt_serial_out_buffer, "timestep: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%d", &test_in);
            if (test_in > 0)
            {
                threshold = test_in;
            }
        }

        if (classifier == 'c')
        {
            sprintf(pt_serial_out_buffer, "input a duty cycle (0-5000): ");
            serial_write;
            // spawn a thread to do the non-blocking serial read
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%d", &test_in);
            if (test_in > 5000)
                continue;
            else if (test_in < 0 && test_in > -5000)
            {
                // use other motor
            }
            else if (test_in < -5000)
                continue;
            else
                pwm_out = test_in;
        }

        if (classifier == 'p')
        {
            sprintf(pt_serial_out_buffer, "input proportional constant: ");
            serial_write;
            // spawn a thread to do the non-blocking serial read
            serial_read;
            // convert input string to number
            float reaction_p;
            sscanf(pt_serial_in_buffer, "%f", &reaction_p);
            REACTION_P = float2fix15(-reaction_p);
        }

        if (classifier == 'i')
        {
            sprintf(pt_serial_out_buffer, "input integration constant: ");
            serial_write;
            // spawn a thread to do the non-blocking serial read
            serial_read;
            // convert input string to number
            float reaction_i;
            sscanf(pt_serial_in_buffer, "%f", &reaction_i);
            REACTION_I = float2fix15(-reaction_i);
        }

        if (classifier == 'd')
        {
            sprintf(pt_serial_out_buffer, "input derivative constant: ");
            serial_write;
            // spawn a thread to do the non-blocking serial read
            serial_read;
            // convert input string to number
            float reaction_d;
            sscanf(pt_serial_in_buffer, "%f", &reaction_d);
            REACTION_D = float2fix15(-reaction_d);
        }

        if (classifier == 's')
        {
            sprintf(pt_serial_out_buffer, "set angle: ");
            serial_write;
            // spawn a thread to do the non-blocking serial read
            serial_read;
            // convert input string to number
            float set_angle_f;
            sscanf(pt_serial_in_buffer, "%f", &set_angle_f);
            set_angle = float2fix15(set_angle_f);
        }
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

    // Initialize VGA
    initVGA();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration_sample, gyro_sample);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}
