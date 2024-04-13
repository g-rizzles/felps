#include <Wire.h>

#include "src/board/board.h"

#include "src/os/serial_handler.hpp"

#include "src/events/event.hpp"
#include "src/events/event_table.hpp"

#include "sys.h"
#include "pwr.h"
#include "disp.h"

int handle_ping(char * buf, uint8_t len, const char delim, void * sysargs) {
  (void)buf;
  (void)len;
  (void)delim;
  (void)sysargs;
  debug.print("pong ");
  debug.println(millis());
  return FELPS_ERR_NONE;
}

int handler_disp_led_set(char * buf, uint8_t len, const char delim, void * sysargs) {
  (void)buf;
  (void)len;
  (void)delim;
  (void)sysargs;
  return FELPS_ERR_NONE;
}

int handler_disp_led_toggle(char * buf, uint8_t len, const char delim, void * sysargs) {
  (void)buf;
  (void)len;
  (void)delim;
  (void)sysargs;
  return FELPS_ERR_NONE;
}

serial_handler_cmd_dictionary_t debug_cmd_dict = {
  .num_cmds = 3,
  .cmds = {
    {.cmd_word = "ping",    .handler = &handle_ping,                NULL},
    {.cmd_word = "dslds",   .handler = &handler_disp_led_set,       NULL},
    {.cmd_word = "dsldt",   .handler = &handler_disp_led_toggle,    NULL},
  },
};

SerialHandler debug_handler(&debug, &debug_cmd_dict, '\n', ' ');

int sys_init(EventTable* evtab) {
  return 0;
}

int sys_periodic() {
  int rc = 0;
  bool ready = false;
  rc = debug_handler.process_serial(&ready);
  if (ready) {
    rc = debug_handler.process_cmd();
    if (rc != FELPS_ERR_NONE) {
      debug.print("FAILED! ");
      debug.println(rc);
    }
  }
  return 0;
}