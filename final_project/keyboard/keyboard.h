#include "bsp/board.h"
#include "tusb.h"

#include "usb_descriptors.h"

#ifndef _KEYBOARD_H
#define _KEYBOARD_H

namespace keyboard
{
  void init();
  void task();

}

#endif