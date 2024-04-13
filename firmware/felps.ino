#include "src/events/event_table.hpp"
#include "src/events/event.hpp"

#include "src/board/init.h"

#include "src/os/serial_handler.hpp"
#include "src/os/err.h"
#include "src/os/sys.h"
#include "src/os/pwr.h"
#include "src/os/exp.h"
#include "src/os/disp.h"

#define MAIN_LOOP_TICK_MS 20
#define HB_INTERVAL_TICK_MS 500

typedef struct {
  unsigned long last_loop;
  unsigned long last_hb;
  EventTable evtab;
} main_loop_ctx_t;

main_loop_ctx_t main_loop_ctx = {
  .last_loop = 0,
  .last_hb = 0,
  .evtab = EventTable(),
};

/**
 * @brief Implements FELPS-OS setup.
 * @details
 * Sets up peripherals, performs initial configuration and state load from memory, sets up
 * interrupts, and registers events.
 */
void setup() {
  int rc = 0;
  do {
    setup_peripherals();
    delay(50);
    rc = sys_init(&main_loop_ctx.evtab);
    if (rc != FELPS_ERR_NONE) {
      debug.print("[setup]: failed sys_init ");
      debug.println(rc);
      break;
    }
    rc = pwr_init(&main_loop_ctx.evtab);
    if (rc != FELPS_ERR_NONE) {
      debug.print("[setup]: failed pwr_init ");
      debug.println(rc);
      break;
    }
    rc = disp_init(&main_loop_ctx.evtab);
    if (rc != FELPS_ERR_NONE) {
      debug.print("[setup]: failed disp_init ");
      debug.println(rc);
      break;
    }
    rc = exp_init(&main_loop_ctx.evtab);
    if (rc != FELPS_ERR_NONE) {
      debug.print("[setup]: failed exp_init ");
      debug.println(rc);
      break;
    }
  } while (0);
  if (rc != FELPS_ERR_NONE) {
    setup_error_handler(rc);
  } else {
    debug.println("[setup]: ready!");
  }
  debug.flush();
  main_loop_ctx.last_hb = millis();
}

/**
 * @brief Implemnents the FELPS-OS polled loop.
 * @details
 * FELPS-OS is structured around a polled loop. The loop procsses events.
 *
 * Events may be set by flags or may be periodic. Events regardless of their type are stored in an
 * event table. The loop simply checks each entry in the event table and fires the callback if the
 * event is to be handled.
 */
void loop() {
  unsigned long post_loop;
  // Update time keeping context
  main_loop_ctx.last_loop = millis();
  // Call various periodic functions
  sys_periodic();
  pwr_periodic();
  exp_periodic();
  disp_periodic();
  // Process any events
  main_loop_ctx.evtab.process();
  // Blink HB if time to
  if (millis() - main_loop_ctx.last_hb >= HB_INTERVAL_TICK_MS) {
    disp_toggle_led(FELPS_LED_HB);
    main_loop_ctx.last_hb = millis();
  }
  // Compute loop iteration delay time
  post_loop = millis();
  if ((post_loop - main_loop_ctx.last_loop) >= MAIN_LOOP_TICK_MS) {
    delay(post_loop - main_loop_ctx.last_loop);
  }
}