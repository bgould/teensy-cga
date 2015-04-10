/*
The MIT License (MIT)

Copyright (c) 2015 Benjamin Gould 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <string.h>
#include "core_pins.h"
#include "cga_font.h"
#include "encoded_image.h"

#define TIMER_CLOCK       48000000
#define PIXEL_CLOCK       14318180
#define LEFT_BLANKING_PIXELS    56
// since my use of RAM is not as efficient as it should be, I need to make the
// visible area slightly smaller to be able to fit the whole framebuffer in 
// memory. If that is ever fixed then "true cga mode" could be enabled.
#ifdef TRUE_CGA_MODE
#define LEFT_OVERSCAN_PIXELS    40
#define VISIBLE_AREA_PIXELS    640
#define RIGHT_OVERSCAN_PIXELS   72
#else 
#define LEFT_OVERSCAN_PIXELS    54
#define VISIBLE_AREA_PIXELS    512
#define RIGHT_OVERSCAN_PIXELS  170
#endif
#define RIGHT_BLANKING_PIXELS   40
#define HORIZONTAL_SYNC_PIXELS  64
#define HORIZONTAL_SCAN_PIXELS ( \
        LEFT_BLANKING_PIXELS   + \
        LEFT_OVERSCAN_PIXELS   + \
        VISIBLE_AREA_PIXELS    + \
        RIGHT_OVERSCAN_PIXELS  + \
        RIGHT_BLANKING_PIXELS  + \
        HORIZONTAL_SYNC_PIXELS )

// CGA had modes for both 640 pixels and 320; change this 1 for 640 pixel width
#define PIXEL_WIDTH                     2 
#define IMAGE_AREA_X (VISIBLE_AREA_PIXELS / PIXEL_WIDTH) // (either 640 or 320)
#define IMAGE_AREA_Y                  200

#define TXT_GLYPH_X 8
#define TXT_GLYPH_Y 8
#define TXT_AREA_X (IMAGE_AREA_X / TXT_GLYPH_X)
#define TXT_AREA_Y (IMAGE_AREA_Y / TXT_GLYPH_Y)

// Can't remember how I figured out these numbers... sorry :( -- bcg
#define TIMER_CYCLES_PER_LINE     3049 
#define TIMER_CYCLES_PER_HSYNC     210

// http://forum.pjrc.com/threads/23950-Parallel-GPIO-on-Teensy-3-0#post34168
// Bitmask for GPIOD_PDOR = 0x00VHIRGB
#define PIN_VSYNC   20
#define PIN_HSYNC    6
#define PIN_COLOR_R  7
#define PIN_COLOR_G 14
#define PIN_COLOR_B  2
#define PIN_COLOR_I  8

// Using the pins above, you can set a pixel just using the CGA color numbers
// For example:  *(volatile uint8_t*)(&GPIOC_PDOR) = COLOR_RED;
#define COLOR_BLACK         0x00 
#define COLOR_BLUE          0x01
#define COLOR_GREEN         0x02
#define COLOR_CYAN          0x03
#define COLOR_RED           0x04
#define COLOR_MAGENTA       0x05
#define COLOR_BROWN         0x06
#define COLOR_LIGHT_GRAY    0x07
#define COLOR_GRAY          0x08
#define COLOR_LIGHT_BLUE    0x09
#define COLOR_LIGHT_GREEN   0x0A
#define COLOR_LIGHT_CYAN    0x0B
#define COLOR_LIGHT_RED     0x0C
#define COLOR_LIGHT_MAGENTA 0x0D
#define COLOR_YELLOW        0x0E
#define COLOR_WHITE         0x0F

volatile uint8_t* output_register = (volatile uint8_t*)(&GPIOD_PDOR);

// macros for setting the output GPIO port; using these is much faster than
// setting the pins individually
#define __out__(bits) *(output_register)=bits
#define outputColor(C) __out__(COLOR_##C)
#define vsyncHsync()  __out__(0b00110000)
#define outputVsync() __out__(0b00100000)
#define outputHsync() __out__(0b00010000)
#define outputCease() __out__(0b00000000)

typedef uint16_t row_count_t; 
typedef uint8_t framebuffer_data_t;

volatile row_count_t row_counter = -1;
volatile uint32_t second_counter = 0;

/*
 * Will use the same buffer space for both image mode and text mode.
 * In text mode, this ends up being a buffer of 200 lines of 80 characters. 
 * In image mode, this is 320 pixels by 200 pixels of 4 bit color.
 */
framebuffer_data_t framebuffer[IMAGE_AREA_Y][IMAGE_AREA_X];
framebuffer_data_t workbuffer[8][IMAGE_AREA_X];

static void configure_timers(void);
static void start_hsync(void);
static void cease_hsync(void);
static void start_scanline(void);

static void buffer_checkerboard_pattern();
static void buffer_text_pattern();

#define ENABLE_SCANLINES()  PIT_TCTRL0 = 3 
#define DISABLE_SCANLINES() PIT_TCTRL0 = 0

#define ENABLE_HSYNCS()  PIT_TCTRL1 = 3 
#define DISABLE_HSYNCS() PIT_TCTRL1 = 0

static void configure_timers() {

  // enable PIT globally
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0;

  // enable PIT 0
  PIT_LDVAL0 = TIMER_CYCLES_PER_LINE;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
 
  // enable PIT 1
#ifdef USE_TIMER_FOR_HSYNCS
  PIT_LDVAL1 = TIMER_CYCLES_PER_HSYNC;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH1);
  ENABLE_HSYNCS();
  DISABLE_HSYNCS();
#endif

  // start scanline timer 
  ENABLE_SCANLINES();
}

// fires at the start of each scanline
void pit0_isr (void) {
  PIT_TFLG0 = 1;
#ifdef USE_TIMER_FOR_HSYNCS
  ENABLE_HSYNCS();   // set a timer for when the horizontal sync ends
#endif
  start_hsync();
}

#ifdef USE_TIMER_FOR_HSYNCS
// fires after the horizontal sync just before the left blanking
void pit1_isr (void) {
  PIT_TFLG1 = 1;
  DISABLE_HSYNCS();  // reset the timer
  cease_hsync();
}
#endif

#define delayPixel(counter) asm volatile( \
    "L_%=_delayPixel:"	    	    "\n\t" /* 6-7 cycles per pixel */ \
    "subs   %0, #1"				        "\n\t" \
    "nop"	        			          "\n\t" \
    "nop"	        			          "\n\t" \
    "nop"	        			          "\n\t" \
    "nop"	        			          "\n\t" \
    "bne    L_%=_delayPixel"      "\n\t" /* branch if counter neq to 0 */ \
  : "+r" (counter) \
);

void start_hsync(void) {
#ifndef USE_TIMER_FOR_HSYNCS
  __disable_irq();
#endif
  static int counter;
  digitalWrite(13, HIGH);
  row_counter++;
  if (row_counter >= 225 && row_counter < 228) {
    vsyncHsync();
  } else {
    outputHsync();
  }
  counter = HORIZONTAL_SYNC_PIXELS; 
  delayPixel(counter);
  cease_hsync();
#ifndef USE_TIMER_FOR_HSYNCS
  __enable_irq();
#endif
}

void cease_hsync(void) {
  digitalWrite(13, LOW);
  if (row_counter >= 225 && row_counter < 228) {
    second_counter = second_counter + (row_counter == 225 ? 1 : 0);
    outputVsync();
  } else {
    outputCease();
    start_scanline();
  }
  if (row_counter > 261) {
    outputCease();
    row_counter = -1;
  }
}

// FIXME: without a doubt, this can/should be done more intelligently,
//        for example with 4-bit color we should be able to pack 8 pixels into
//        1 32-bit word; that would actually be quite meaningful because it
//        could allow for a complete frame buffer at full resolution or 
//        double buffering of a smaller resolution; also implementing a proper
//        text mode using the glyphs stored in flash could save a bunch of RAM
void start_scanline(void) {
  if (row_counter >= IMAGE_AREA_Y) {
    return;
  }
  int counter;
  framebuffer_data_t* ptr = &framebuffer[row_counter][0];
  framebuffer_data_t value = (*ptr);
  counter = LEFT_OVERSCAN_PIXELS + LEFT_BLANKING_PIXELS;
  delayPixel(counter);
  counter = (-1 * IMAGE_AREA_X);
  asm(" @ before loop");
  asm volatile(
    "L_%=_nextPixel:        @ start of pixel "	  	     "\n\t"
    "ldr    %2, [%1], #1     @ load from memory "        "\n\t"
    "and    %2,  %2, #15     @ "                         "\n\t"
    "strb   %2, [%3,  #0]    @ store to output register" "\n\t"
#if PIXEL_WIDTH == 2
    "nop                     @ "                         "\n\t"
    "nop                     @ "                         "\n\t"
    "nop                     @ "                         "\n\t"
    "nop                     @ "                         "\n\t"
    "nop                     @ "                         "\n\t"
    "nop                     @ "                         "\n\t"
    "nop                     @ "                         "\n\t"
#endif
    "adds   %0, #1           @ increment the counter "   "\n\t"
    "bne    L_%=_nextPixel  @ end of pixel "             "\n\t"
  : "+r" (counter), 
    "+r" (ptr), 
    "+r" (value), 
    "+r" (output_register)
  );
  outputColor(BLACK);
  asm(" @ after loop");
}

/* ---------------------------------------------------------------------------
 * some simple graphics experiments below; someone who knows what they're doing
 * certainly would do a nicer job with these. updating the framebuffer seems to
 * cause noticeable jitter and the updates don't look very smooth. Being more
 * careful with memory usage could allow for double buffered graphics which I
 * think would probably look a lot nicer. In the meantime the mess below is 
 * all I'm willing to muster at this point in time. :) -- bcg
 */

#define IMAGE_SIZE_X               240
#define IMAGE_SIZE_Y               180
#define IMAGE_OFFSET_X ((IMAGE_AREA_X - IMAGE_SIZE_X) / 2)
#define IMAGE_OFFSET_Y ((IMAGE_AREA_Y - IMAGE_SIZE_Y) / 2)
#define IMAGE_PIXEL_VALUE(offset, shift) ((show_and_tell[offset]&(3<<shift))>>shift)
// ^^^ wtf was I thinking with this macro?

#define APP_MODE_CHECKERBOARD 0
#define APP_MODE_IMAGE        1
#define APP_MODE_IMAGE2       2
#define APP_MODE_TEXT         3

#define BUTTON_STATE_OPEN 0
#define BUTTON_STATE_DEBOUNCE 1
#define BUTTON_STATE_PUSHED 2

#define PIN_BUTTON 12

static framebuffer_data_t pallette[] = {
  COLOR_BLACK,
  COLOR_LIGHT_RED,
  COLOR_WHITE,
};

static char app_mode = APP_MODE_TEXT;
static char button_state = BUTTON_STATE_OPEN;
static uint8_t glyph_offset = 0;
static uint32_t last_change = 0;

// simple multi-colored checkerboard pattern that slides across the screen
static void buffer_checkerboard_pattern() {
  for (int j = 0; j < IMAGE_AREA_Y; j++) {
    for (int i = 0; i < IMAGE_AREA_X; i++) {
      framebuffer[j][i] = (i/CGA_FONT_WIDTH)%16 + (j/CGA_FONT_HEIGHT)%16 + glyph_offset;
    }
  }
}

// multi-colored text that slides across the screen; hopefully will optimize 
// this into a text mode at some point in the future
static void buffer_text_pattern() {
  for (int j = 0; j < IMAGE_AREA_Y; j++) {
    int y = j % CGA_FONT_HEIGHT;
    for (int i = 0; i < IMAGE_AREA_X; i++) {
      cga_font_glyph_t glyph = CGA_FONT_ID_TO_GLYPH((i / CGA_FONT_WIDTH) + glyph_offset);
      int x = i % CGA_FONT_WIDTH;
      int pixel = CGA_FONT_XY_TO_PIXEL(x, y);
      if (CGA_FONT_PIXEL_VALUE(glyph, pixel)) {
        workbuffer[y][i] = (i/CGA_FONT_WIDTH)%16 + (j/CGA_FONT_HEIGHT)%16 + glyph_offset;
      } else {
        workbuffer[y][i] = 0;
      }
    }
    if (y == 7) {
      memcpy(framebuffer[j-7], workbuffer[y-7], IMAGE_AREA_X);
      memcpy(framebuffer[j-6], workbuffer[y-6], IMAGE_AREA_X);
      memcpy(framebuffer[j-5], workbuffer[y-5], IMAGE_AREA_X);
      memcpy(framebuffer[j-4], workbuffer[y-4], IMAGE_AREA_X);
      memcpy(framebuffer[j-3], workbuffer[y-3], IMAGE_AREA_X);
      memcpy(framebuffer[j-2], workbuffer[y-2], IMAGE_AREA_X);
      memcpy(framebuffer[j-1], workbuffer[y-1], IMAGE_AREA_X);
      memcpy(framebuffer[j-0], workbuffer[y-0], IMAGE_AREA_X);
    }
  }
}

// displays a frame buffered image that (mostly) fills the screen
static void buffer_encoded_image() {
  static int j;
  static framebuffer_data_t* row_buffer;
  for (j = 0; j < IMAGE_AREA_Y; j++) {
    row_buffer = &framebuffer[j][0];
    if (j < IMAGE_OFFSET_Y) {
      memset(row_buffer, 0, IMAGE_AREA_X);
    }
    if ((j > IMAGE_OFFSET_Y) && (j - IMAGE_OFFSET_Y) < IMAGE_SIZE_Y) {
      int offset = (((j - IMAGE_OFFSET_Y) * IMAGE_SIZE_X) / 4);
      for (int i = 0; i < IMAGE_OFFSET_X; i++) {
        row_buffer[i] = pallette[2];
      }
      for (int i = IMAGE_OFFSET_X; i < (IMAGE_OFFSET_X+IMAGE_SIZE_X); offset++) {
        row_buffer[i++] = pallette[IMAGE_PIXEL_VALUE(offset, 6)];
        row_buffer[i++] = pallette[IMAGE_PIXEL_VALUE(offset, 4)];
        row_buffer[i++] = pallette[IMAGE_PIXEL_VALUE(offset, 2)];
        row_buffer[i++] = pallette[IMAGE_PIXEL_VALUE(offset, 0)];
      }
      for (int i = (IMAGE_OFFSET_X + IMAGE_SIZE_X); i < IMAGE_AREA_X; i++) {
        row_buffer[i] = pallette[2];
      }
    } else {
      memset(row_buffer, j < IMAGE_AREA_Y-1 ? pallette[2] : 0, IMAGE_AREA_X);
    }
  }
}

static void update_app_mode(char app_mode) {
  switch (app_mode) {
    case APP_MODE_TEXT:
      buffer_text_pattern();
      break;
    case APP_MODE_IMAGE:
      pallette[1] = COLOR_LIGHT_RED;
    case APP_MODE_IMAGE2:
      buffer_encoded_image();
      break;
    case APP_MODE_CHECKERBOARD:
    default:
      buffer_checkerboard_pattern();
      break;
  }
}

extern "C" int main(void)
{
  // flash teensy's LED to show that we're running
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delayMicroseconds(1000000);
  digitalWrite(13, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_BUTTON, HIGH);

  pinMode(PIN_COLOR_R, OUTPUT);
  digitalWrite(PIN_COLOR_R, LOW);

  pinMode(PIN_COLOR_G, OUTPUT);
  digitalWrite(PIN_COLOR_G, LOW);

  pinMode(PIN_COLOR_B, OUTPUT);
  digitalWrite(PIN_COLOR_B, LOW);

  pinMode(PIN_COLOR_I, OUTPUT);
  digitalWrite(PIN_COLOR_I, LOW);

  pinMode(PIN_HSYNC,   OUTPUT);
  digitalWrite(PIN_HSYNC, LOW);

  pinMode(PIN_VSYNC,   OUTPUT);
  digitalWrite(PIN_VSYNC, LOW);

  configure_timers();
  update_app_mode(app_mode);

  while (true) {
    switch (button_state) {
      case BUTTON_STATE_PUSHED:
        if (digitalRead(PIN_BUTTON)) {
          app_mode = (app_mode + 1) % 4;
          update_app_mode(app_mode);
          button_state = BUTTON_STATE_OPEN;
        }
        break;
      case BUTTON_STATE_DEBOUNCE:
        if (!digitalRead(PIN_BUTTON)) {
          button_state = BUTTON_STATE_PUSHED;
        } else {
          button_state = BUTTON_STATE_OPEN;
        }
        break;
      default:
        if (!digitalRead(PIN_BUTTON)) {
          button_state = BUTTON_STATE_DEBOUNCE;
        } else {
          button_state = BUTTON_STATE_OPEN;
        }
        break;
    }
    if ((second_counter / 10) != last_change) {
      last_change = second_counter / 10;
      if (glyph_offset >= 200) {
        glyph_offset = 0;
      } else {
        glyph_offset += 1;
      }
      if (app_mode == APP_MODE_IMAGE2) {
        pallette[1] = (glyph_offset % 14) + 1;
      }
      if (last_change % 1 == 0) {
        update_app_mode(app_mode);
      }
    }
  }

}

