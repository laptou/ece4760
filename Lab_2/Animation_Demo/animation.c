
/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

#pragma region fixed point scalar math
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

#define min(a, b) (a > b ? b : a)
#define max(a, b) (a > b ? a : b)
#define clamp(x, a, b) min(max(x, a), b)
#pragma endregion

#pragma region fixed point vector math
typedef struct vec2
{
  fix15 x;
  fix15 y;
} vec2_t;

inline vec2_t new_vec2(fix15 x, fix15 y)
{
  return (vec2_t){.x = x, .y = y};
}

inline vec2_t mul_vec2_fix15(fix15 x, vec2_t v)
{
  return (vec2_t){.x = multfix15(v.x, x), .y = multfix15(v.y, x)};
}

inline vec2_t mul_vec2_vec2(vec2_t a, vec2_t b)
{
  return (vec2_t){.x = multfix15(a.x, b.x), .y = multfix15(a.y, b.x)};
}

inline vec2_t add_vec2(vec2_t a, vec2_t b)
{
  return (vec2_t){.x = a.x + b.x, .y = a.y + b.y};
}

inline vec2_t sub_vec2(vec2_t a, vec2_t b)
{
  return (vec2_t){.x = a.x - b.x, .y = a.y - b.y};
}

inline fix15 dot_vec2(vec2_t a, vec2_t b)
{
  return multfix15(a.x, b.x) + multfix15(a.y, b.y);
}

#pragma endregion

#pragma region wall detection

const fix15 WALL_BOTTOM = int2fix15(380);
const fix15 WALL_TOP = int2fix15(100);
const fix15 WALL_LEFT = int2fix15(100);
const fix15 WALL_RIGHT = int2fix15(540);

#define hitBottom(b) (b > WALL_BOTTOM)
#define hitTop(b) (b < WALL_TOP)
#define hitLeft(a) (a < WALL_LEFT)
#define hitRight(a) (a > WALL_RIGHT)

#pragma endregion

#pragma region rendering parameters
// uS per frame
#define FRAME_PERIOD 33000

// the color of the boid
char color = WHITE;
#pragma endregion

#pragma region boid state
typedef struct boid_state
{
  vec2_t position;
  vec2_t velocity;
} boid_state_t;

boid_state_t boids[2];
#pragma endregion

// Create a boid
void spawn_boid(boid_state_t *boid, int direction)
{
  // Start in center of screen
  boid->position.x = int2fix15(320);
  boid->position.y = int2fix15(240);

  // Choose left or right
  if (direction)
    boid->velocity.x = int2fix15(3);
  else
    boid->velocity.x = int2fix15(-3);

  // Moving down
  boid->velocity.y = int2fix15(1);
}

// Draw the boundaries
void draw_arena()
{
  drawVLine(100, 100, 280, WHITE);
  drawVLine(540, 100, 280, WHITE);
  drawHLine(100, 100, 440, WHITE);
  drawHLine(100, 380, 440, WHITE);
}

// Detect wallstrikes, update velocity and position
// Delta time should be in seconds
void update_boid_motion(boid_state_t *boid)
{
  // Reverse direction if we've hit a wall
  if (hitTop(boid->position.y) || hitBottom(boid->position.y))
  {
    boid->velocity.y *= -1;
    boid->position.y = clamp(boid->position.y, WALL_TOP, WALL_BOTTOM);
  }

  if (hitRight(boid->position.x) || hitLeft(boid->position.x))
  {
    boid->velocity.x *= -1;
    boid->position.x = clamp(boid->position.x, WALL_LEFT, WALL_RIGHT);
  }
  
  // Update position using velocity
  boid->position = add_vec2(boid->position, boid->velocity);
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  // stores user input
  static int user_input;
  // wait for 0.1 sec
  PT_YIELD_usec(1000000);
  // announce the threader version
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write;
  while (1)
  {
    // print prompt
    sprintf(pt_serial_out_buffer, "input a number in the range 1-7: ");
    // non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%d", &user_input);
    // update boid color
    if ((user_input > 0) && (user_input < 8))
    {
      color = (char)user_input;
    }
  } // END WHILE(1)
  PT_END(pt);
} // timer thread

#pragma region animation

typedef struct animation_thread_state
{
  int core_num;

  // for tracking frame rate
  int last_frame_start;
  int current_frame_start;
  int spare_time;

  // for tracking which boids we are animating on this thread
  // threads should not have overlapping ranges
  size_t first_boid;
  size_t last_boid;
} animation_thread_state_t;

animation_thread_state_t animation_thread_states[2];

void init_animation_thread(animation_thread_state_t *state)
{
  for (size_t i = state->first_boid; i < state->last_boid; i++)
  {
    spawn_boid(&boids[i], i % 2);
  }
}

void update_animation_thread(animation_thread_state_t *state)
{
  // Measure time at start of thread
  int last_frame_start = state->current_frame_start;
  int current_frame_start = time_us_32();
  state->current_frame_start = current_frame_start;
  int time_delta = current_frame_start - last_frame_start;

  for (size_t i = state->first_boid; i < state->last_boid; i++)
  {
    boid_state_t *boid = &boids[i];
    // erase boid
    drawRect(fix2int15(boid->position.x), fix2int15(boid->position.y), 2, 2, BLACK);
    // update boid's position and velocity
    update_boid_motion(boid);
    // draw the boid at its new position
    drawRect(fix2int15(boid->position.x), fix2int15(boid->position.y), 2, 2, color);
  }

  // draw the boundaries
  draw_arena();

  // delay in accordance with frame rate
  state->spare_time = FRAME_PERIOD - (time_us_32() - current_frame_start);
}

// Animation on core 0
static PT_THREAD(protothread_anim0(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // spawn boids
  init_animation_thread(&animation_thread_states[0]);

  while (1)
  {
    update_animation_thread(&animation_thread_states[0]);
    PT_YIELD_usec(animation_thread_states[0].spare_time);

    // NEVER exit while
  } // END WHILE(1)

  PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD(protothread_anim1(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // spawn boids
  init_animation_thread(&animation_thread_states[1]);

  while (1)
  {
    update_animation_thread(&animation_thread_states[1]);
    PT_YIELD_usec(animation_thread_states[1].spare_time);

    // NEVER exit while
  } // END WHILE(1)

  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  // initialize stio
  stdio_init_all();

  // initialize VGA
  initVGA();

  // initialize animation thread states
  animation_thread_states[0].core_num = 0;
  animation_thread_states[0].first_boid = 0;
  animation_thread_states[0].last_boid = 1;

  animation_thread_states[1].core_num = 1;
  animation_thread_states[1].first_boid = 1;
  animation_thread_states[1].last_boid = 2;

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim0);

  // start scheduler
  pt_schedule_start;
}
