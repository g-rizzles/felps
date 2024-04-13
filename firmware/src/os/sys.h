#pragma once
#ifndef FELPS_SYS_H_
#define FELPS_SYS_H_

#include "src/events/event_table.hpp"

// --------------------------------- API FUNCTIONS ---------------------------------

int sys_init(EventTable* evtab);
int sys_periodic();

// -------------------------------- DEBUG FUNCTIONS --------------------------------

#endif // FELPS_SYS_H_