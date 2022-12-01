#ifndef _KEYBOARD_H
#define _KEYBOARD_H

#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

#include "../notes.h"

namespace keyboard
{
  void init();
  void task(const note *current_note);
}

#endif