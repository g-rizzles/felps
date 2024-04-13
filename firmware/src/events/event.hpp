#pragma once
#ifndef FELPS_EVENT_H_
#define FELPS_EVENT_H_

#include <stdint.h>

#include "src/os/err.h"

enum class EVENT_ID : uint8_t {
  EVENT_ID_INVALID = 0,
  /* sys MODULE EVENTS */
  /* pwr MODULE EVENTS */
  PWR_POLL_SENSORS_PERIODIC,    //!< Time to poll the sensors!
  PWR_LSWC_FAULT,               //!< Load switch experienced a fault
  PWR_LSWC_STATE_CHANGE,        //!< Load switch has changed on/off state
  PWR_LSWC_CONFIG_CHANGE,       //!< A load switch has had its configuration changed
  PWR_BATMGR_FAULT,             //!< BQ25672 has reported a monitored fault
  PWR_BATMGR_STATE_CHANGE,      //!< BQ25672 has reported a monitored state change
  PWR_BATMGR_SHIP_MODE,         //!< BQ25672 is to be placed into SHIP mode
  PWR_BATMGR_SOLAR_INPUT,       //!< BQ25672 is to charge / supply via solar cell input
  PWR_BATMGR_FORCE_INPUT,       //!< BQ25673 is to accept input via a specific input
  PWR_BATMGR_FORCE_VSYS,        //!< BQ25672 is to adjust system voltage regulation
  /* disp MODULE EVENTS */
  DISP_REFRESH_PERIODIC,        //!< Time to refresh any elements!
  DISP_BUTTON_STATE_CHANGE,     //!< TCA9555 reports user pressed a button
  DISP_LED_STATE_CHANGE,        //!< A LED managed by TCA9555 is to change state
  DISP_ACTIVE_MENU_CHANGE,      //!< The active menu on the display is to change
  DISP_PARAMETER_CHANGE,        //!< A displayed parameter has been changed
  DISP_NEW_MESSAGE,             //!< A new message is to be shown on the display
  DISP_NEW_ALERT,               //!< A new alert is to be shown on the display
  DISP_DISPLAY_CONTEXT_CHANGE,  //!< The display is to shift a pane to a new context
  /* exp MODULE EVENTS */
  EXP_CHECK_ALIVE_PERIODIC,     //!< Time to PING connected devices!
  EXP_PARAMETER_CHANGE,         //!< A parameter in a device's table has changed
  EXP_NOTIFY,                   //!< A notification is to be sent to a device
  EXP_REQUEST,                  //!< A request is to be sent to a device
  EVENT_ID_COUNT
};

class Event {
  public:
    Event(EVENT_ID id) : id(id), dispatched(false) {;}
    ~Event() {;}
    EVENT_ID get_id() { return this->id; }
    bool get_dispatched() { return this->dispatched; }

    /** @brief Executes _handle behavior specified by event if dispatched. Returns an error code
     * if not dispatched. */
    int handle() {
      if (!this->dispatched) {
        return -FELPS_ERR_BUSY;
      }
      this->dispatched = false;
      return this->_handle();
    }

    /** @brief Attempts to dispatch an event. Returns an error if already dispatched. */
    int dispatch() {
      if (this->dispatched) {
        return -FELPS_ERR_BUSY;
      }
      this->dispatched = true;
      return FELPS_ERR_NONE;
    }
  protected:
    /** @brief Defines behavior of the event, to be implemented by child classes. */
    int _handle() {return -FELPS_ERR_NOT_IMPLEMENTED;}
  private:
    EVENT_ID id;
    bool dispatched;
};

#endif // FELPS_EVENT_H_