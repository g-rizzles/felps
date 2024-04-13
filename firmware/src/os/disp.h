#pragma once
#ifndef FELPS_DISP_H_
#define FELPS_DISP_H_

#include "src/events/event_table.hpp"

typedef enum {
  FELPS_LED_INVALID = 0,
  FELPS_LED_1 = 1,
  FELPS_LED_2 = 2,
  FELPS_LED_3 = 3,
  FELPS_LED_4 = 4,
  FELPS_LED_HB = 5,
} felps_leds_e;

typedef enum {
  FELPS_BTN_INVALID = 0,
  FELPS_BTN_HAZARD  = 1,
  FELPS_BTN_HORN    = 2,
  FELPS_BTN_MF_1    = 3,
  FELPS_BTN_MF_2    = 4,
  FELPS_BTN_MENU_1  = 5,
  FELPS_BTN_MENU_2  = 6,
  FELPS_BTN_MENU_3  = 7,
  FELPS_BTN_MENU_4  = 8,
} felps_btns_e; 

// --------------------------------- API FUNCTIONS ---------------------------------

int disp_init(EventTable* evtab);

int disp_periodic();

void disp_set_led(felps_leds_e led, bool on);
bool disp_get_led(felps_leds_e led);
void disp_toggle_led(felps_leds_e led);

// -------------------------------- DEBUG FUNCTIONS --------------------------------

#endif // FELPS_DISP_H_