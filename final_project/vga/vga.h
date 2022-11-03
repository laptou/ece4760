/**
 * Hunter Adams (vha3@cornell.edu)
 *
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
 *  - DMA channels 0, 1, 2, and 3
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 * NOTE
 *  - This is a translation of the display primitives
 *    for the PIC32 written by Bruce Land and students
 *
 */

// Give the I/O pins that we're using some names that make sense - usable in
// main()
enum vga_pins { HSYNC = 16, VSYNC, RED_PIN, GREEN_PIN, BLUE_PIN };

// We can only produce 8 (3-bit) colors, so let's give them readable names -
// usable in main()
enum colors { BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE };

// VGA primitives - usable in main
void vga_init(void);
void vga_pixel(short x, short y, char color);
void vga_vline(short x, short y, short h, char color);
void vga_hline(short x, short y, short w, char color);
void vga_line(short x0, short y0, short x1, short y1, char color);
void vga_rect(short x, short y, short w, short h, char color);
void vga_fill_circle(short x0, short y0, short r, char color);
void vga_stroke_circle(short x0, short y0, short r, char color);
void vga_fill_round_rect(
    short x, short y, short w, short h, short r, char color
);
void vga_stroke_round_rect(
    short x, short y, short w, short h, short r, char color
);
void vga_fill_rect(short x, short y, short w, short h, char color);
void vga_char(
    short x, short y, unsigned char c, char color, char bg, unsigned char size
);
void vga_cursor(short x, short y);
void vga_fg_color(char c);
void vga_bg_color(char c, char bg);
void vga_text_size(unsigned char s);
void vga_text_wrap(char w);

void vga_write_char(char c);
void vga_write_string(char const *str);
