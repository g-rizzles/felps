#pragma once
#ifndef FELPS_EVENT_TABLE_H_
#define FELPS_EVENT_TABLE_H_

#include <cstring>

#include "src/os/err.h"
#include "event.hpp"

/** @brief Table maintains a set of pointers to events in other contexts throughout the program.
 * If directed, the table can execute all dispatched events. No copies of events are keps in the
 * table. */
class EventTable {
  public:
    EventTable() {
      this->event_count = 0;
      memset(this->events, 0x00, sizeof(Event *) * (int)EVENT_ID::EVENT_ID_COUNT);
    }
    ~EventTable() {;}

    /** @brief Add an event to the table. */
    int enroll(Event * evt) {
      if (this->event_count >= (int)EVENT_ID::EVENT_ID_COUNT) {
        return -FELPS_ERR_OUT_OF_MEM;
      } else if ((evt->get_id() >= EVENT_ID::EVENT_ID_COUNT) ||
                 (this->get_by_id(evt->get_id()) == NULL)) {
        return -FELPS_ERR_BAD_PARAM;
      }
      this->events[this->event_count] = evt;
      this->event_count++;
    }

    /**
     * @brief Executes events contained within the table with their dispatch flags set.
     * @returns an enumerated error code, <0 for fault 0 for success, error codes can come from
     * handled events or from function logic.
     * @note Does not stop handling dispatched events if an event returns an error.
     */
    int process() {
      int rc = -FELPS_ERR_UNKNOWN;
      int ret = 0;
      for (int i = 0; i < this->event_count; ++i) {
        if (this->events[i] != NULL) {
          if (this->events[i]->get_dispatched()) {
            ret = this->events[i]->dispatch();
            if (ret != 0) {
              rc = (rc == 0) ? ret : -FELPS_ERR_MULTIPLE;
            } else {
              rc = (rc == 0) ? 0 : rc;
            }
          }
        }
      }
      return rc;
    }

  private:
    int event_count;
    Event * events[(int)EVENT_ID::EVENT_ID_COUNT];

    /** @brief Attempts to lookup a pointer to an event by event_id. Returns NULL if not found. */
    Event * get_by_id(EVENT_ID id) {
      for (int i = 0; i < this->event_count; ++i) {
        if (this->events[i]->get_id() == id) {
          return this->events[i];
        }
      }
      return NULL;
    }
};

#endif // FELPS_EVENT_TABLE_H_