
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
#include <limits.h>
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
static char fpstext[40];

#pragma region fixed point vector math

// signed saturating 32-bit addition
fix15 ss_add_32(const fix15 a, const fix15 b)
{
  int c;
  c = a + b;
  if (((a ^ b) & INT_MIN) == 0)
  {
    if ((c ^ a) & INT_MIN)
    {
      c = (a < 0) ? INT_MIN : INT_MAX;
    }
  }
  return c;
}

fix15 ss_mult_32(const fix15 a, const fix15 b)
{
  // based on https://stackoverflow.com/a/29285223
  int sa = (a >> 31); // fills the integer with the sign bit of a
  int sb = (b >> 31); // fills the integer with the sign bit of b
  int so = (sa ^ sb); // expected sign of output

  int c = multfix15(a, b); // multiply the integers
  int sc = (c >> 31);      // fills the integer with the sign bit of c

  // if expected sign and actual sign are different, set c to 0x7FFFFFFF or 0x8000000 depending on desired sign
  // so will be 0xFFFFFFFF if desired sign is negative, and 0x00000000 if desired sign is positive
  c ^= (so ^ sc) & (c ^ so ^ 0x7FFFFFFF);
  return c;
}

typedef struct vec2
{
  fix15 x;
  fix15 y;
} vec2_t;

const vec2_t VEC2_ZERO = {int2fix15(0), int2fix15(0)};

inline vec2_t new_vec2(fix15 x, fix15 y)
{
  return (vec2_t){.x = x, .y = y};
}

inline vec2_t mul_vec2_fix15(fix15 x, vec2_t v)
{
  return (vec2_t){.x = multfix15(v.x, x), .y = multfix15(v.y, x)};
}

inline vec2_t div_vec2_fix15(vec2_t v, fix15 x)
{
  return (vec2_t){.x = divfix(v.x, x), .y = divfix(v.y, x)};
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
  return ss_add_32(ss_mult_32(a.x, b.x), ss_mult_32(a.y, b.y));
}

fix15 norm_vec2(vec2_t a)
{
  // fix15 z_max, z_min;
  // fix15 x = absfix15(a.x);
  // fix15 y = absfix15(a.x);

  // if (x > y)
  // {
  //   z_max = x;
  //   z_min = y;
  // }
  // else
  // {
  //   z_max = y;
  //   z_min = x;
  // }

  // static const fix15 alpha = float2fix15(0.96043387);
  // static const fix15 beta = float2fix15(0.3978247);

  // return multfix15(alpha, absfix15(z_max)) + multfix15(beta, absfix15(z_min));

  fix15 norm_sq = dot_vec2(a, a);
  return float2fix15(sqrtf(fix2float15(norm_sq)));
}

// squared distance between a and b
inline fix15 dist_sq_vec2(vec2_t a, vec2_t b)
{
  vec2_t d = sub_vec2(a, b);
  return dot_vec2(d, d);
}

#pragma endregion

typedef enum wrap
{
  wrap_box,
  wrap_top_bottom,
  wrap_all
} wrap_t;

static volatile wrap_t current_wrap = wrap_box;

#pragma region wall detection

const fix15 WALL_BOTTOM = int2fix15(380);
const fix15 WALL_TOP = int2fix15(100);
static volatile fix15 WALL_LEFT = int2fix15(100);
static volatile fix15 WALL_RIGHT = int2fix15(540);
const fix15 SCREEN_BOTTOM = int2fix15(480);
const fix15 SCREEN_TOP = int2fix15(0);
const fix15 SCREEN_LEFT = int2fix15(0);
const fix15 SCREEN_RIGHT = int2fix15(640);

// static volatile fix15 width = int2fix15(20);

#pragma endregion

#pragma region rendering parameters
// uS per frame
#define FRAME_PERIOD 33000

// the color of the boid
char color = WHITE;
#pragma endregion

#pragma region boid parameters

const fix15 BOID_TURN_MARGIN = int2fix15(100);
const fix15 BOID_TURN_FACTOR = float2fix15(0.2);
const fix15 BOID_VISUAL_RANGE = float2fix15(40);
const fix15 BOID_VISUAL_RANGE_SQ = multfix15(BOID_VISUAL_RANGE, BOID_VISUAL_RANGE);
const fix15 BOID_PROTECTED_RANGE = float2fix15(8);
const fix15 BOID_PROTECTED_RANGE_SQ = multfix15(BOID_PROTECTED_RANGE, BOID_PROTECTED_RANGE);
const fix15 BOID_CENTERING_FACTOR = float2fix15(0.0005);
const fix15 BOID_AVOID_FACTOR = float2fix15(0.05);
const fix15 BOID_MATCHING_FACTOR = float2fix15(0.05);
const fix15 BOID_MAX_SPEED = float2fix15(6);
const fix15 BOID_MAX_SPEED_SQ = multfix15(BOID_MAX_SPEED, BOID_MAX_SPEED);
const fix15 BOID_MIN_SPEED = float2fix15(3);
const fix15 BOID_MIN_SPEED_SQ = multfix15(BOID_MIN_SPEED, BOID_MIN_SPEED);
const fix15 BOID_MAX_BIAS = float2fix15(0.01);
const fix15 BOID_BIAS_INCREMENT = float2fix15(0.00004);
const fix15 BOID_DEFAULT_BIASVAL = float2fix15(0.001);

#pragma endregion

#pragma region boid state
typedef struct boid_state
{
  vec2_t position;
  vec2_t velocity;
} boid_state_t;

#define BOID_COUNT 30

boid_state_t boids[BOID_COUNT];
#pragma endregion

// Create a boid
void spawn_boid(boid_state_t *boid, int direction)
{
  // Start in center of screen
  boid->position.x = int2fix15(320);
  boid->position.y = int2fix15(240);

  double angle = (double)random();

  boid->velocity.y = float2fix15((float)sin(angle));
  boid->velocity.x = float2fix15((float)cos(angle));
  boid->velocity = mul_vec2_fix15(int2fix15(3), boid->velocity);
}

// Draw the boundaries
void draw_arena()
{
  switch (current_wrap)
  {
  case wrap_box:

    drawVLine(100, 100, 280, WHITE);
    drawVLine(540, 100, 280, WHITE);
    drawHLine(100, 100, 440, WHITE);
    drawHLine(100, 380, 440, WHITE);

    break;

  case wrap_top_bottom:
    drawVLine(fix2int15(WALL_LEFT), 0, 480, WHITE);
    drawVLine(fix2int15(WALL_RIGHT), 0, 480, WHITE);

    break;
  case wrap_all:
    // fillRect(0, 0, 640, 480, BLACK);

    break;
  }
}

// updates the motion of boid at index i
void update_boid_motion(size_t i)
{
  boid_state_t *boid = &boids[i];

#pragma region steer away from walls
  switch (current_wrap)
  {
  case wrap_box:
    if (boid->position.x < WALL_LEFT)
    {
      boid->velocity.x += BOID_TURN_FACTOR;
    }
    else if (boid->position.x > WALL_RIGHT)
    {
      boid->velocity.x -= BOID_TURN_FACTOR;
    }

    if (boid->position.y < WALL_TOP)
    {
      boid->velocity.y += BOID_TURN_FACTOR;
    }
    else if (boid->position.y > WALL_BOTTOM)
    {
      boid->velocity.y -= BOID_TURN_FACTOR;
    }
    break;
  case wrap_top_bottom:
    if (boid->position.x < WALL_LEFT)
    {
      boid->velocity.x += BOID_TURN_FACTOR;
    }
    else if (boid->position.x > WALL_RIGHT)
    {
      boid->velocity.x -= BOID_TURN_FACTOR;
    }

    if (boid->position.y <= SCREEN_TOP)
    {
      boid->position.y = SCREEN_BOTTOM;
    }
    else if (boid->position.y > SCREEN_BOTTOM)
    {
      boid->position.y = SCREEN_TOP;
    }
    break;
  case wrap_all:
    if (boid->position.x < SCREEN_LEFT)
    {
      boid->position.x = SCREEN_RIGHT;
    }
    else if (boid->position.x > SCREEN_RIGHT)
    {
      boid->position.x = SCREEN_LEFT;
    }

    if (boid->position.y < SCREEN_TOP)
    {
      boid->position.y = SCREEN_BOTTOM;
    }
    else if (boid->position.y > SCREEN_BOTTOM)
    {
      boid->position.y = SCREEN_TOP;
    }
    break;
  }

#pragma endregion

#pragma swarm behaviour

  vec2_t close_neighbor_rel_position = VEC2_ZERO;
  vec2_t visual_neighbor_position = VEC2_ZERO;
  vec2_t visual_neighbor_velocity = VEC2_ZERO;

  size_t visual_neighbor_count = 0;

  // Figure out what neighbouring boids are doing
  for (size_t j = 0; j < BOID_COUNT; j++)
  {
    if (i == j)
      continue;

    boid_state_t *other = &boids[j];

    fix15 dist_sq = dist_sq_vec2(boid->position, other->position);

    // assert(dist_sq >= 0);

    if (dist_sq < 0)
    {
      printf("boid->position %.3f %.3f\n", fix2float15(boid->position.x), fix2float15(boid->position.y));
      printf("other->position %.3f %.3f\n", fix2float15(other->position.x), fix2float15(other->position.y));
      vec2_t dist = sub_vec2(boid->position, other->position);
      printf("dist %.3f %.3f\n", fix2float15(dist.x), fix2float15(dist.y));

      fix15 d2x = multfix15(dist.x, dist.x);
      fix15 d2y = multfix15(dist.y, dist.y);
      fix15 d2 = d2x + d2y;

      fix15 ss_d2x = ss_mult_32(dist.x, dist.x);
      fix15 ss_d2y = ss_mult_32(dist.y, dist.y);
      fix15 ss_d2 = ss_add_32(ss_d2x, ss_d2y);

      printf("dist * dist %.3f %.3f %.3f\n", fix2float15(d2x), fix2float15(d2y), fix2float15(d2));
      printf("dist * dist ss %.3f %.3f %.3f\n", fix2float15(ss_d2x), fix2float15(ss_d2y), fix2float15(ss_d2));
      printf("dist * dist ss hex %#010x %#010x %#010x\n", ss_d2x, ss_d2y, ss_d2);
      assert(false);
    }

    if (dist_sq < BOID_VISUAL_RANGE_SQ)
    {
      // other is w/in our visual range
      visual_neighbor_count++;

      visual_neighbor_velocity = add_vec2(
          visual_neighbor_velocity,
          other->velocity);

      visual_neighbor_position = add_vec2(
          visual_neighbor_position,
          other->position);
    }
    else
    {
      continue;
    }

    if (dist_sq < BOID_PROTECTED_RANGE_SQ)
    {
      // other is w/in our protected range
      close_neighbor_rel_position = add_vec2(
          close_neighbor_rel_position,
          sub_vec2(boid->position, other->position));
    }
  }

  // divide sum of neighbour velocities by number of visual neighbours to get
  // average
  vec2_t avg_visual_neighbor_velocity = div_vec2_fix15(visual_neighbor_velocity, int2fix15(visual_neighbor_count));
  vec2_t avg_visual_neighbor_position = div_vec2_fix15(visual_neighbor_position, int2fix15(visual_neighbor_count));

  // Alignment velocity update
  boid->velocity = add_vec2(boid->velocity, mul_vec2_fix15(BOID_MATCHING_FACTOR, sub_vec2(avg_visual_neighbor_velocity, boid->velocity)));
  // Cohesion velocity update
  boid->velocity = add_vec2(boid->velocity, mul_vec2_fix15(BOID_CENTERING_FACTOR, sub_vec2(avg_visual_neighbor_position, boid->position)));
  // Avoid velocity update
  boid->velocity = add_vec2(boid->velocity, mul_vec2_fix15(BOID_AVOID_FACTOR, close_neighbor_rel_position));

#pragma endregion

#pragma region minimum and maximum speed

  fix15 boid_speed = norm_vec2(boid->velocity);
  if (boid_speed < BOID_MIN_SPEED)
  {
    fix15 speed_factor = divfix(BOID_MIN_SPEED, boid_speed);
    boid->velocity = mul_vec2_fix15(speed_factor, boid->velocity);
  }
  else if (boid_speed > BOID_MAX_SPEED)
  {
    fix15 speed_factor = divfix(BOID_MAX_SPEED, boid_speed);
    boid->velocity = mul_vec2_fix15(speed_factor, boid->velocity);
  }

#pragma endregion

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
    sprintf(pt_serial_out_buffer, "input a number 1-3 to change the wrap mode, and a number 10-500 to change the width: ");
    // non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%d", &user_input);
    // update boid color
    // if ((user_input > 1) && (user_input < 30))
    // {
    //   width = user_input;
    // }
    if (user_input == 1)
    {
      current_wrap = wrap_box;
      drawVLine(fix2int15(WALL_LEFT), 0, 480, BLACK);
      drawVLine(fix2int15(WALL_RIGHT), 0, 480, BLACK);
      WALL_LEFT = int2fix15(100);
      WALL_RIGHT = int2fix15(540);
    }
    if (user_input == 2)
    {
      current_wrap = wrap_top_bottom;
      drawHLine(100, 100, 440, BLACK);
      drawHLine(100, 380, 440, BLACK);
    }
    if (user_input == 3)
    {
      current_wrap = wrap_all;
      drawVLine(100, 100, 280, BLACK);
      drawVLine(540, 100, 280, BLACK);
      drawHLine(100, 100, 440, BLACK);
      drawHLine(100, 380, 440, BLACK);
      drawVLine(fix2int15(WALL_LEFT), 0, 480, BLACK);
      drawVLine(fix2int15(WALL_RIGHT), 0, 480, BLACK);
    }
    if (user_input >= 10 && user_input <= 500 && current_wrap == wrap_top_bottom)
    {
      drawVLine(fix2int15(WALL_LEFT), 0, 480, BLACK);
      drawVLine(fix2int15(WALL_RIGHT), 0, 480, BLACK);

      WALL_LEFT = int2fix15(320) - divfix(int2fix15(user_input), int2fix15(2));
      WALL_RIGHT = int2fix15(320) + divfix(int2fix15(user_input), int2fix15(2));
      //    WALL_LEFT = int2fix15(320 - width / 2);
      //  WALL_RIGHT = int2fix15(320 + width / 2);
    }

  } // END WHILE(1)
  PT_END(pt);
} // timer thread

#pragma region animation

typedef struct animation_thread_state
{
  uint8_t core_num;

  // for tracking frame rate
  uint32_t first_frame_start;
  uint32_t last_frame_start;
  uint32_t current_frame_start;
  uint32_t spare_time;
  uint32_t print_frame_start;
  uint32_t frames_since_print;

  fix15 fps;

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

  state->first_frame_start = time_us_32();
}

void update_animation_thread(animation_thread_state_t *state)
{
  // Measure time at start of thread
  uint32_t last_frame_start = state->current_frame_start;
  uint32_t current_frame_start = time_us_32();
  state->current_frame_start = current_frame_start;
  uint32_t time_delta = current_frame_start - last_frame_start;

  for (size_t i = state->first_boid; i < state->last_boid; i++)
  {
    assert(i < BOID_COUNT);

    boid_state_t *boid = &boids[i];
    // erase boid
    drawRect(fix2int15(boid->position.x), fix2int15(boid->position.y), 2, 2, BLACK);
    // update boid's position and velocity
    update_boid_motion(i);
    // draw the boid at its new position
    drawRect(fix2int15(boid->position.x), fix2int15(boid->position.y), 2, 2, color);
  }

  state->frames_since_print++;

  if (state->frames_since_print >= 30)
  {
    uint32_t time_delta_since_print = state->current_frame_start - state->print_frame_start;
    uint32_t time_delta_since_start = state->current_frame_start - state->first_frame_start;

    uint32_t time_delta_per_frame = time_delta_since_print / state->frames_since_print / 30;

    fix15 fps = divfix(divfix(int2fix15(1000), time_delta_per_frame), int2fix15(1000));

    // printf("fps: %f, time: %d", fix2float15(fps), time_delta_since_start / 1000000);
    state->fps = fps;
    state->print_frame_start = state->current_frame_start;
    state->frames_since_print = 0;
  }
  fillRect(65, 20, 176, 80, BLACK);
  static char fps_text[30];
  setTextColor(WHITE);
  setTextSize(1);
  setCursor(65, 20);
  sprintf(fps_text, "fps: %f", fix2float15(state->fps));
  writeString(fps_text);
  setCursor(65, 40);
  sprintf(fps_text, "boids: %d", state->last_boid - state->first_boid);
  writeString(fps_text);
  setCursor(65, 60);
  uint32_t time_delta_since_start = state->current_frame_start - state->first_frame_start;

  sprintf(fps_text, "time: %d", time_delta_since_start / 1000000);
  writeString(fps_text);
  setCursor(65, 80);

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

  animation_thread_state_t *state = &animation_thread_states[0];

  // spawn boids
  init_animation_thread(state);

  while (1)
  {
    update_animation_thread(state);

    PT_YIELD_usec(state->spare_time);

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
  animation_thread_states[0].last_boid = 30;

  animation_thread_states[1].core_num = 1;
  animation_thread_states[1].first_boid = 16;
  animation_thread_states[1].last_boid = 31;

  // start core 1
  // multicore_reset_core1();
  // multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim0);

  // start scheduler
  pt_schedule_start;
}
