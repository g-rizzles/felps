#pragma once
#ifndef FELPS_EXP_H_
#define FELPS_EXP_H_

#include "src/events/event_table.hpp"

// --------------------------------- API FUNCTIONS ---------------------------------

int exp_init(EventTable* evtab);

int exp_periodic();

// -------------------------------- DEBUG FUNCTIONS --------------------------------

#endif // FELPS_EXP_H_