#include "keyboard.h"
#include "../fft/fft.h"
#include "../notes.h"

static void send_hid_report(uint8_t report_id, uint32_t btn);
void hid_task();

namespace keyboard
{
const absolute_note *current_note;

void init()
{
  board_init();
  tusb_init();
}

// call this every 10ms to update usb stuff
void task(const absolute_note *new_note)
{
  current_note = new_note;
  tud_task(); // tinyusb device task
  hid_task();
}
} // namespace keyboard

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc
// ..) tud_hid_report_complete_cb() is used to send the next report after
// previous one is complete
void hid_task()
{
  // Poll every 10ms
  // const uint32_t interval_ms = 10;
  // static uint32_t start_ms = 0;

  // if (board_millis() - start_ms < interval_ms)
  //   return; // not enough time
  // start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if (tud_suspended() && btn) {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  } else {
    // Send the 1st of report chain, the rest will be sent by
    // tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_KEYBOARD, btn);
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {}

// Invoked when device is unmounted
void tud_umount_cb(void) {}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

bool click_lock = false;

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if (!tud_hid_ready())
    return;

  auto current_note = keyboard::current_note;

  switch (report_id) {
  case REPORT_ID_KEYBOARD: {
    // use to avoid send multiple consecutive zero report for keyboard
    static bool has_keyboard_key = false;

    if (current_note != NULL) {
      uint8_t keycode[6] = {0};

      if (current_note->octave == 5) {
        if (current_note->value == note::C && current_note->sharp == false) {
          keycode[0] = HID_KEY_W;
        } else if (current_note->value == note::C && current_note->sharp == true) {
          keycode[0] = HID_KEY_S;
        } else if (current_note->value == note::D && current_note->sharp == false) {
          keycode[0] = HID_KEY_A;
        } else if (current_note->value == note::D && current_note->sharp == true) {
          keycode[0] = HID_KEY_D;
        } else if (current_note->value == note::E && current_note->sharp == false) {
          keycode[0] = HID_KEY_SPACE;
          keycode[1] = HID_KEY_W;
        }
      }

      tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
      has_keyboard_key = true;
    } else {
      // send empty key report if previously has key pressed
      if (has_keyboard_key)
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
      has_keyboard_key = false;
    }
  } break;

  case REPORT_ID_MOUSE: {
    auto speed = 15;
    if (current_note != NULL) {
      if (current_note->octave == 5) {
        if (current_note->value == note::F && current_note->sharp == false) {
          tud_hid_mouse_report(
              REPORT_ID_MOUSE, click_lock ? 0x01 : 0x00, -speed, 0, 0, 0
          );
        } else if (current_note->value == note::F && current_note->sharp == true) {
          tud_hid_mouse_report(
              REPORT_ID_MOUSE, click_lock ? 0x01 : 0x00, speed, 0, 0, 0
          );
        } else if (current_note->value == note::G && current_note->sharp == false) {
          tud_hid_mouse_report(
              REPORT_ID_MOUSE, click_lock ? 0x01 : 0x00, 0, -speed, 0, 0
          );
        } else if (current_note->value == note::G && current_note->sharp == true) {
          tud_hid_mouse_report(
              REPORT_ID_MOUSE, click_lock ? 0x01 : 0x00, 0, speed, 0, 0
          );
        } else if (current_note->value == note::A && current_note->sharp == false) {
          click_lock = false;
          tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, 0, 0, 1, 0); // scroll
        } else if (current_note->value == note::A && current_note->sharp == true) {
          click_lock = false;
          tud_hid_mouse_report(
              REPORT_ID_MOUSE, 0x02, 0, 0, 0, 0
          ); // right click
        } else if (current_note->value == note::B && current_note->sharp == false) {
          click_lock = !click_lock;
          tud_hid_mouse_report(
              REPORT_ID_MOUSE, click_lock ? 0x01 : 0x00, 0, 0, 0, 0
          ); // left click
        }
      } else if (current_note->octave == 6) {
        click_lock = false;
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0x01, 0, 0, 0, 0); // left click
      }
    } else {
      tud_hid_mouse_report(
          REPORT_ID_MOUSE, click_lock ? 0x01 : 0x00, 0, 0, 0, 0
      );
    }
  } break;
  default:
    break;
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(
    uint8_t instance, uint8_t const *report, uint8_t len
)
{
  (void)instance;
  (void)len;

  uint8_t next_report_id = report[0] + 1;

  if (next_report_id < REPORT_ID_CONSUMER_CONTROL) {
    send_hid_report(next_report_id, board_button_read());
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(
    uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
    uint8_t *buffer, uint16_t reqlen
)
{
  // TODO not Implemented
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(
    uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
    uint8_t const *buffer, uint16_t bufsize
)
{
  (void)instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT) {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD) {
      // bufsize should be (at least) 1
      if (bufsize < 1)
        return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK) {
        // Capslock On: disable blink, turn led on
        board_led_write(true);
      } else {
        // Caplocks Off: back to normal blink
        board_led_write(false);
      }
    }
  }
}
