#pragma once
#ifndef FELPS_PWR_H_
#define FELPS_PWR_H_

#include "src/events/event_table.hpp"

#define BAD_SENSOR_READING                  (0xFFFF)
#define FELPS_LSWC_DEFAULT_OVER_CURRENT_MA  (2500)

typedef enum : uint8_t {
  // From BQ25672 battery manager IC
  FELPS_PWR_SNS_BATMGR_BEGIN  = 0x00,
  FELPS_PWR_SNS_BATMGR_TEMP_C = 0x01,
  FELPS_PWR_SNS_BAT_TEMP_C    = 0x02,
  FELPS_PWR_SNS_BUS_I         = 0x03,
  FELPS_PWR_SNS_BAT_I         = 0x04,
  FELPS_PWR_SNS_BUS_V         = 0x05,
  FELPS_PWR_SNS_BAT_V         = 0x06,
  FELPS_PWR_SNS_SYS_V         = 0x07,
  FELPS_PWR_SNS_BATMGR_END    = 0x08,
  // Micro internal
  FELPS_PWR_SNS_UC_INT_BEGIN  = 0x10,
  FELPS_PWR_SNS_UC_TEMP_C     = 0x11,
  FELPS_PWR_SNS_UC_REF_V      = 0x12,
  FELPS_PWR_SNS_UC_INT_END    = 0x13,
  // Micro external
  FELPS_PWR_SNS_UC_EXT_BEGIN  = 0x20,
  FELPS_PWR_SNS_JACK1_I       = 0x21,
  FELPS_PWR_SNS_JACK2_I       = 0x22,
  FELPS_PWR_SNS_SYS_I         = 0x23,
  FELPS_PWR_SNS_LSWC_3_I      = 0x24,
  FELPS_PWR_SNS_LSWC_1_I      = 0x25,
  FELPS_PWR_SNS_LSWC_2_I      = 0x26,
  FELPS_PWR_SNS_LSWC_4_I      = 0x27,
  FELPS_PWR_SNS_LSWC_5_I      = 0x28,
  FELPS_PWR_SNS_LSWC_8_I      = 0x29,
  FELPS_PWR_SNS_LSWC_6_I      = 0x2A,
  FELPS_PWR_SNS_LSWC_7_I      = 0x2B,  
  FELPS_PWR_SNS_UC_EXT_END    = 0x2C,
} felps_pwr_sensors_e;

typedef enum : uint8_t {
  FELPS_LSWC_1 = 0,
  FELPS_LSWC_2 = 1,
  FELPS_LSWC_3 = 2,
  FELPS_LSWC_4 = 3,
  FELPS_LSWC_5 = 4,
  FELPS_LSWC_6 = 5,
  FELPS_LSWC_7 = 6,
  FELPS_LSWC_8 = 7,
  FELPS_LSWC_COUNT,
} felps_lswc_e;

typedef enum {
  FELPS_LSWC_OFF,
  FELPS_LSWC_ON,
  FELPS_LSWC_FLASH,
} felps_pwr_lswc_state_e;

typedef struct {
  unsigned long last_ms;
  int extra_arg;
  uint16_t on_time_ms;
  uint16_t off_time_ms;
  felps_pwr_lswc_state_e state;
  bool triggered_overcurrent;
} felps_lswc_state_t;

typedef struct {
  uint16_t over_current_ma;
  uint16_t hysteresis_ms;
} felps_lswc_config_t;

typedef struct {
  felps_lswc_e id;
  felps_pwr_sensors_e sensor;
  felps_lswc_state_t state;
  felps_lswc_config_t config;
} felps_lswc_t;

/** @brief Whole fractional representation struct with millis time measured. */
typedef struct {
  unsigned long ms;
  char const * units;
  int16_t w;   //!< whole part of the number
  uint16_t mcf; //!< fractional part of the number in millicounts
  uint8_t sensor;
} felps_wf_sensor_reading_t;

// --------------------------------- API FUNCTIONS ---------------------------------

/**
 * @brief Initialize the pwr module of the FELPS-oS
 * @param[in,out] evtab pointer to a table to enroll events in 
 * @return enumerated status code, <0 for a fault, 0 for success
 * @details
 * 
 */
int pwr_init(EventTable* evtab);

int pwr_periodic();

void pwr_set_lwsc_state(felps_lswc_e lswc, felps_lswc_state_t * state);
void pwr_get_lswc_state(felps_lswc_e lswc, felps_lswc_state_t * state);
void pwr_set_lswc_config(felps_lswc_e lswc, felps_lswc_config_t * config);
void pwr_get_lswc_config(felps_lswc_e lswc, felps_lswc_config_t * config);

/**
 * @brief Get the last measured value of the requested sensor
 * @param[in]     sensor_id enumerated sensor identifier
 * @param[in,out] q_reading optional pointer to a struct to store a Q 
 *                representation of the reading  
 * @return raw ADC reading
 */
uint16_t pwr_get_sensor_reading(felps_pwr_sensors_e sensor_id, 
                                felps_wf_sensor_reading_t * q_reading);

void pwr_reset();
void pwr_shutdown();
void pwr_ship_mode();

// -------------------------------- DEBUG FUNCTIONS --------------------------------

void pwr_debug_dump_sensors();
uint8_t pwr_debug_batmgr_read_reg(uint8_t addr);
uint8_t pwr_debug_batmgr_write_reg(uint8_t addr, uint8_t val, uint8_t mask);
void pwr_debug_dump_batmgr_stats_faults();

#endif // FELPS_PWR_H_