#include <Arduino.h>
#include <Wire.h>

#include "src/board/board.h"

#include "src/events/event.hpp"
#include "src/events/event_table.hpp"

#include "disp.h"

#define TCA9555_IC2_ADDR      (0x20)

typedef struct {
} disp_ctx_t;

static disp_ctx_t disp_ctx {
};

// -------------------------------- LOCAL FUNCTIONS --------------------------------

static int disp_ioexp_init(EventTable* evtab) {
  int rc = FELPS_ERR_NONE;
  return rc;
}

// --------------------------------- API FUNCTIONS ---------------------------------

int disp_init(EventTable* evtab) {
  int rc = 0;
  return rc;
}

int disp_periodic() {
  return 0;
}

void disp_set_led(felps_leds_e led, bool on) {
  return;
}

bool disp_get_led(felps_leds_e led) {
  return false;
}

void disp_toggle_led(felps_leds_e led) {
  return;
}

// -------------------------------- DEBUG FUNCTIONS --------------------------------
