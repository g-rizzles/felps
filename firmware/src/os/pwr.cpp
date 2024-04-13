#include <Arduino.h>
#include <Wire.h>

#include "src/board/board.h"
#include "src/bq25672/bq25672.hpp"

#include "src/events/event.hpp"
#include "src/events/event_table.hpp"

#include "pwr.h"

#define PWR_BATMGR_SHIP_FET_PRESENT (false)
#define PWR_BATMGR_ADC_EN           (true)
#define PWR_BATMGR_CHARGE_EN        (false)

// Constants for INAx80A2 conversions
#define PWR_STM32G0_ADC_COUNT_TO_100UV    (80)
#define PWR_INAX80A2_GAIN                 (50)
#define PWR_FELPS_SHUNT_100UOHMS          (100)
#define PWR_INAX80A2_0I_OUTPUT_ADC_COUNTS (10)

char const * units_celsius = "C";
char const * units_volts = "V";
char const * units_amps = "A";
char const * units_counts = "cts";

typedef enum : uint8_t {
  PWR_ISNS_MUX_CH_0 = 0,
  PWR_ISNS_MUX_CH_1 = 1,
  PWR_ISNS_MUX_CH_2 = 2,
  PWR_ISNS_MUX_CH_3 = 3,
  PWR_ISNS_MUX_CH_4 = 4,
  PWR_ISNS_MUX_CH_5 = 5,
  PWR_ISNS_MUX_CH_6 = 6,
  PWR_ISNS_MUX_CH_7 = 7,
} pwr_isns_mux_ch_e;

typedef struct {
  unsigned long ms;
  uint16_t raw;
  felps_pwr_sensors_e sensor;
} pwr_simple_sensor_reading_t;

typedef struct {
  bq25672 batmgr;
  pwr_simple_sensor_reading_t sensors[20];
  felps_lswc_t lswcs[FELPS_LSWC_COUNT];
} pwr_ctx_t;

static pwr_ctx_t pwr_ctx = {
  .batmgr = bq25672(Wire),
};

// -------------------------------- LOCAL FUNCTIONS --------------------------------

/** @brief Set current load switch mux connected channel. */
static bool set_mux(uint8_t ch) {
  if (ch > PWR_ISNS_MUX_CH_7) {
    return false;
  }
  digitalWrite(SIG_LSW_MUX_S0, 0b001 & ch);
  digitalWrite(SIG_LSW_MUX_S0, 0b010 & ch);
  digitalWrite(SIG_LSW_MUX_S0, 0b100 & ch);
  return true;
}

static void calc_bq25672_ibus(unsigned long reading_time, uint16_t raw,
                              felps_wf_sensor_reading_t * q_reading) {
  uint16_t sign = (raw & 0x8000);
  q_reading->units = units_amps;
  q_reading->w = (raw & 0x7fff) / 1000;
  if (sign) q_reading->w *= -1;
  q_reading->mcf = ((raw & 0x7fff) % 1000);
}

static void calc_bq25672_ibat(unsigned long reading_time, uint16_t raw,
                              felps_wf_sensor_reading_t * q_reading) {
  uint16_t sign = (raw & 0x8000);
  q_reading->units = units_amps;
  q_reading->w = (raw & 0x7fff) / 1000;
  if (sign) q_reading->w *= -1;
  q_reading->mcf = ((raw & 0x7fff) % 1000);
}

static void calc_bq25672_vbus(unsigned long reading_time, uint16_t raw,
                              felps_wf_sensor_reading_t * q_reading) {
  q_reading->units = units_volts;
  q_reading->w = raw / 1000;
  q_reading->mcf = (raw % 1000);
}

static void calc_bq25672_vbat(unsigned long reading_time, uint16_t raw,
                              felps_wf_sensor_reading_t * q_reading) {
  q_reading->units = units_volts;
  q_reading->w = raw / 1000;
  q_reading->mcf = (raw % 1000);
}

static void calc_bq25672_vsys(unsigned long reading_time, uint16_t raw,
                              felps_wf_sensor_reading_t * q_reading) {
  q_reading->units = units_volts;
  q_reading->w = raw / 1000;
  q_reading->mcf = (raw % 1000);
}

static void calc_bq25672_ts(unsigned long reading_time, uint16_t raw,
                            felps_wf_sensor_reading_t * q_reading) {
  q_reading->units = units_celsius;
  // TODO
  q_reading->w = 0;
  q_reading->mcf = 0;
}

static void calc_bq25672_tdie(unsigned long reading_time, uint16_t raw,
                              felps_wf_sensor_reading_t * q_reading) {
  uint16_t sign = (raw & 0x8000);
  q_reading->units = units_celsius;
  q_reading->w = raw / 2;
  if (sign) q_reading->w *= -1;
  q_reading->mcf = 0;
}

static void calc_uc_temperature(unsigned long reading_time, uint16_t raw,
                                felps_wf_sensor_reading_t * q_reading) {
  return;
  // TODO
}

static void calc_uc_adc_voltage(unsigned long reading_time, uint16_t raw,
                                felps_wf_sensor_reading_t * q_reading) {
  return;
  // TODO
}

/** @brief Compute INAx180A2 current sensor measurement for FELPS sensors. */
static void calc_ina_x180a2_10mOhm(unsigned long reading_time, uint16_t raw,
                                   felps_wf_sensor_reading_t * q_reading) {
  // Raw reading in ADC counts. This IC has a 12b ADC, Vref = 3.3V. The
  // STM32G050 does not have an FPU. Do calculations in 100u-unit increments
  // to allow easy conversion to q_reading format. Measurements from an
  // INAx80 have a 0 current input voltage output of ~8mV depending on
  // common mode voltage.
  q_reading->units = units_amps;
  if (raw >= PWR_INAX80A2_0I_OUTPUT_ADC_COUNTS) {
    q_reading->w = 0;
    q_reading->mcf = 0;
  } else {
    // Compute current measurement in 100uA increments
    uint32_t computed = (PWR_STM32G0_ADC_COUNT_TO_100UV * raw) /
                        (PWR_INAX80A2_GAIN * PWR_FELPS_SHUNT_100UOHMS);
    q_reading->w = computed / 10000;
    q_reading->mcf = computed % 10000;
  }
}

/** @brief Convert a raw power sensor to a whole fractional reading struct.
 * Zeros out the q_reading struct before filling. */
static void raw_channel_to_wf_reading(felps_pwr_sensors_e sensor_id,
                                      unsigned long reading_time, uint16_t raw,
                                      felps_wf_sensor_reading_t * q_reading) {
  memset(q_reading, 0x00, sizeof(felps_wf_sensor_reading_t));
  q_reading->sensor = sensor_id;
  q_reading->ms = reading_time;
  if (sensor_id == FELPS_PWR_SNS_BATMGR_TEMP_C) {
    calc_bq25672_tdie(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_BAT_TEMP_C) {
    calc_bq25672_ts(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_BUS_I) {
    calc_bq25672_ibus(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_BAT_I) {
    calc_bq25672_ibat(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_BUS_V) {
    calc_bq25672_vbus(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_BAT_V) {
    calc_bq25672_vbat(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_SYS_V) {
      calc_bq25672_vsys(reading_time, raw, q_reading);
  } else if (sensor_id == FELPS_PWR_SNS_UC_TEMP_C) {
    return; // TODO
  } else if (sensor_id == FELPS_PWR_SNS_UC_REF_V) {
    return; // TODO
  } else if ((sensor_id == FELPS_PWR_SNS_JACK1_I)  ||
             (sensor_id == FELPS_PWR_SNS_JACK2_I)  ||
             (sensor_id == FELPS_PWR_SNS_SYS_I)    ||
             (sensor_id == FELPS_PWR_SNS_LSWC_1_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_2_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_3_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_4_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_5_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_6_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_7_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_8_I)) {
    calc_ina_x180a2_10mOhm(reading_time, raw, q_reading);
  }
}

static int pwr_batmgr_init(EventTable* evtab) {
  int rc = 0;
  bq25672& bmg = pwr_ctx.batmgr;
  do {
    pinMode(SNS_I_SYS, INPUT_ANALOG);
    pinMode(SNS_I_JACK1, INPUT_ANALOG);
    pinMode(SNS_I_JACK2, INPUT_ANALOG);
    pinMode(SNS_I_LSW_MUX, INPUT_ANALOG);
    pinMode(SIG_LSW_MUX_S0, OUTPUT);
    pinMode(SIG_LSW_MUX_S1, OUTPUT);
    pinMode(SIG_LSW_MUX_S2, OUTPUT);
    pinMode(SIG_CHARGE_HIGH_Z_EN, OUTPUT);
    pinMode(SIG_CE_N, OUTPUT);
    pinMode(SIG_QON_N, OUTPUT);
    //digitalWrite(SIG_CHARGE_HIGH_Z_EN, LOW);
    //digitalWrite(SIG_CE_N, HIGH);
    //digitalWrite(SIG_QON_N, LOW);
    // TODO attach interrupt
    // TODO setup events
    rc = bmg.begin(PWR_BATMGR_SHIP_FET_PRESENT);
    if (rc != FELPS_ERR_NONE) {
      #if VERBOSE_DEBUG
      debug.print("[pwr]: batmgr.begin F ");
      debug.println(rc);
      #endif
      break;
    }
    //bmg.reset();
    // ---- Configure BQ25672 SHIP Mode Config ----
    bmg.enter_idle_mode();
    // ---- Configure BQ25672 Fault Detection and Interrupt Config ----
    // ---- Configure BQ25672 ADC ----
    bmg.enable_adc(PWR_BATMGR_ADC_EN);
    // ---- Configure BQ25672 System Voltage Control ----
    // ---- Configure BQ25672 Charger Control ----
    bmg.configure_watchdog(BQ25672_WATCHDOG_CONFIG::TIMER_DISABLE);
    // ---- Other Misc BQ25672 Configuration ----
  } while (0);
  return rc;
}

static int pwr_lswc_init(EventTable* evtab) {
  int rc = FELPS_ERR_NONE;
  pinMode(SIG_EN_LSW1, OUTPUT);
  pinMode(SIG_EN_LSW2, OUTPUT);
  pinMode(SIG_EN_LSW3, OUTPUT);
  pinMode(SIG_EN_LSW4, OUTPUT);
  pinMode(SIG_EN_LSW5, OUTPUT);
  pinMode(SIG_EN_LSW6, OUTPUT);
  pinMode(SIG_EN_LSW7, OUTPUT);
  pinMode(SIG_EN_LSW8, OUTPUT);
  digitalWrite(SIG_EN_LSW1, LOW);
  digitalWrite(SIG_EN_LSW2, LOW);
  digitalWrite(SIG_EN_LSW3, LOW);
  digitalWrite(SIG_EN_LSW4, LOW);
  digitalWrite(SIG_EN_LSW5, LOW);
  digitalWrite(SIG_EN_LSW6, LOW);
  digitalWrite(SIG_EN_LSW7, LOW);
  digitalWrite(SIG_EN_LSW8, LOW);
  // TODO setup events
  for (int i = 0; i < (int)FELPS_LSWC_COUNT; ++i) {
    memset(&pwr_ctx.lswcs[i], 0x00, sizeof(felps_lswc_t ));
    pwr_ctx.lswcs[i].id = felps_lswc_e(i);
    if (i == (int)FELPS_LSWC_1) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_1_I;
    } else if (i == (int)FELPS_LSWC_2) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_2_I;
    } else if (i == (int)FELPS_LSWC_3) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_3_I;
    } else if (i == (int)FELPS_LSWC_4) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_4_I;
    } else if (i == (int)FELPS_LSWC_5) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_5_I;
    } else if (i == (int)FELPS_LSWC_6) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_6_I;
    } else if (i == (int)FELPS_LSWC_7) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_7_I;
    } else if (i == (int)FELPS_LSWC_8) {
      pwr_ctx.lswcs[i].sensor = FELPS_PWR_SNS_LSWC_8_I;
    }
    pwr_ctx.lswcs[i].state.state = FELPS_LSWC_OFF;
    pwr_ctx.lswcs[i].config.over_current_ma = FELPS_LSWC_DEFAULT_OVER_CURRENT_MA;
  }
  return rc;
}

// --------------------------------- API FUNCTIONS ---------------------------------

int pwr_init(EventTable* evtab) {
  int rc = 0;
  int ret = 0;
  ret = pwr_batmgr_init(evtab);
  if (ret != FELPS_ERR_NONE) {
    rc = ret;
  }
  ret = pwr_lswc_init(evtab);
  if (ret != FELPS_ERR_NONE) {
    if (rc != FELPS_ERR_NONE) {
      rc = -FELPS_ERR_MULTIPLE;
    } else {
      rc = ret;
    }
  }
  return rc;
}

int pwr_periodic() {
  // Update sensor values
  // Update state of load switches if required
  felps_lswc_t * lswc_p = NULL;
  for (uint8_t lswc = (uint8_t)FELPS_LSWC_1; lswc < (uint8_t)FELPS_LSWC_COUNT; ++lswc) {
    lswc_p = &pwr_ctx.lswcs[lswc];
  }
  return FELPS_ERR_NONE;
}

void pwr_set_lwsc_state(felps_lswc_e lswc, felps_lswc_state_t * state) {
  if ((state != NULL) && ((lswc >= FELPS_LSWC_1) && (lswc < FELPS_LSWC_COUNT))) {
    memset(&pwr_ctx.lswcs[(int)lswc].state, 0x00, sizeof(felps_lswc_state_t));
    memcpy(&pwr_ctx.lswcs[(int)lswc].state, state, sizeof(felps_lswc_state_t));
  }
}

void pwr_get_lswc_state(felps_lswc_e lswc, felps_lswc_state_t * state) {
  if ((state != NULL) && ((lswc >= FELPS_LSWC_1) && (lswc < FELPS_LSWC_COUNT))) {
    memset(state, 0x00, sizeof(felps_lswc_state_t));
    memcpy(state, &pwr_ctx.lswcs[(int)lswc].state, sizeof(felps_lswc_state_t));
  }
}

void pwr_set_lswc_config(felps_lswc_e lswc, felps_lswc_config_t * config) {
  if ((config != NULL) && ((lswc >= FELPS_LSWC_1) && (lswc < FELPS_LSWC_COUNT))) {
    memset(&pwr_ctx.lswcs[(int)lswc].config, 0x00, sizeof(felps_lswc_config_t));
    memcpy(&pwr_ctx.lswcs[(int)lswc].config, config, sizeof(felps_lswc_config_t));
  }
}

void pwr_get_lswc_config(felps_lswc_e lswc, felps_lswc_config_t * config) {
  if ((config != NULL) && ((lswc >= FELPS_LSWC_1) && (lswc < FELPS_LSWC_COUNT))) {
    memset(config, 0x00, sizeof(felps_lswc_config_t));
    memcpy(config, &pwr_ctx.lswcs[(int)lswc].config, sizeof(felps_lswc_config_t));
  }
}

uint16_t pwr_get_sensor_reading(felps_pwr_sensors_e sensor_id,
                                felps_wf_sensor_reading_t * q_reading) {
  uint16_t sensor_reading = 0;
  uint8_t mux_ch = 0;
  unsigned long ms = millis();
  if (sensor_id == FELPS_PWR_SNS_BATMGR_TEMP_C) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_TDIE);
  } else if (sensor_id == FELPS_PWR_SNS_BAT_TEMP_C) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_TS);
  } else if (sensor_id == FELPS_PWR_SNS_BUS_I) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_IBUS);
  } else if (sensor_id == FELPS_PWR_SNS_BAT_I) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_IBAT);
  } else if (sensor_id == FELPS_PWR_SNS_BUS_V) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_VBUS);
  } else if (sensor_id == FELPS_PWR_SNS_BAT_V) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_VBAT);
  } else if (sensor_id == FELPS_PWR_SNS_SYS_V) {
    sensor_reading = pwr_ctx.batmgr.get_adc_ch(BQ25672_ADC_CH::ADC_CH_VSYS);
  } else if (sensor_id == FELPS_PWR_SNS_UC_TEMP_C) {
    sensor_reading = BAD_SENSOR_READING;
  } else if (sensor_id == FELPS_PWR_SNS_UC_REF_V) {
    sensor_reading = BAD_SENSOR_READING;
  } else if (sensor_id == FELPS_PWR_SNS_JACK1_I) {
    sensor_reading = analogRead(SNS_I_JACK1);
  } else if (sensor_id == FELPS_PWR_SNS_JACK2_I) {
    sensor_reading = analogRead(SNS_I_JACK2);
  } else if (sensor_id == FELPS_PWR_SNS_SYS_I) {
    sensor_reading = analogRead(SNS_I_SYS);
  } else if ((sensor_id == FELPS_PWR_SNS_LSWC_1_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_2_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_3_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_4_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_5_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_6_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_7_I) ||
             (sensor_id == FELPS_PWR_SNS_LSWC_8_I)) {
    mux_ch = (uint8_t)sensor_id - (uint8_t)FELPS_PWR_SNS_LSWC_3_I;
    if (!set_mux(mux_ch)) {
      sensor_reading = BAD_SENSOR_READING;
    } else {
      sensor_reading = analogRead(SNS_I_LSW_MUX);
    }
  } else {
    sensor_reading = BAD_SENSOR_READING;
  }
  if (q_reading != NULL) {
    raw_channel_to_wf_reading(sensor_id, ms, sensor_reading, q_reading);
  }
  return sensor_reading;
}

void pwr_reset() {
  pwr_ctx.batmgr.system_power_reset(true);
}

void pwr_shutdown() {
  pwr_ctx.batmgr.enter_shutdown_mode(false);
}

void pwr_ship_mode() {
  pwr_ctx.batmgr.enter_ship_mode(false);
}

// -------------------------------- DEBUG FUNCTIONS --------------------------------

void debug_print_sensor_reading(uint16_t raw, felps_wf_sensor_reading_t * reading) {
  char buf[32] = {'\0'};
  if (snprintf(buf, 32, "%.5u %.3d.%.3u", raw, reading->w, reading->mcf) < 0) {
    debug.println("DEBUG PRINT SENSOR READING FAILED!");
  } else {
    debug.print("["); debug.print(reading->ms); debug.print("] ");
    debug.print(reading->sensor); debug.print(" ");
    debug.print(buf); debug.print(" ");
    for (size_t i = 0; i < strlen(reading->units); ++i) debug.print(reading->units[i]);
    debug.println("");
  }
}

void pwr_debug_dump_sensors() {
  uint16_t raw = 0;
  felps_wf_sensor_reading_t reading;
  for (uint8_t ch = FELPS_PWR_SNS_BATMGR_BEGIN + 1;
       ch < FELPS_PWR_SNS_BATMGR_END; ++ch) {
    raw = pwr_get_sensor_reading((felps_pwr_sensors_e)ch, &reading);
    debug_print_sensor_reading(raw, &reading);
  }
  // Still working out ADC clocking... uncomment when fixed
  /*for (uint8_t ch = (uint8_t)FELPS_PWR_SNS_UC_INT_BEGIN + 1;
       ch < (uint8_t)FELPS_PWR_SNS_UC_INT_END; ++ch) {
    raw = pwr_get_sensor_reading((felps_pwr_sensors_e)ch, &reading);
    debug_print_sensor_reading(raw, & reading);
  }*/
  /*for (uint8_t ch = FELPS_PWR_SNS_UC_EXT_BEGIN + 1;
       ch < FELPS_PWR_SNS_UC_EXT_END; ++ch) {
    raw = pwr_get_sensor_reading((felps_pwr_sensors_e)ch, &reading);
    debug_print_sensor_reading(raw, &reading);
  }*/
}

uint8_t pwr_debug_batmgr_read_reg(uint8_t addr) {
  return pwr_ctx.batmgr.read((const bq25672::REG)addr);
}

uint8_t pwr_debug_batmgr_write_reg(uint8_t addr, uint8_t val, uint8_t mask) {
  if (mask != 0xFF) {
    pwr_ctx.batmgr.rmw((bq25672::REG)addr, val, mask);
  } else {
    pwr_ctx.batmgr.write((bq25672::REG)addr, val);
  }
  return pwr_ctx.batmgr.read((bq25672::REG)addr);
}

void pwr_debug_dump_batmgr_stats_faults() {
  bq25672_charger_status_u status = {0};
  bq25672_fault_status_u faults = {0};
  uint8_t pid = pwr_ctx.batmgr.get_part_info();
  pwr_ctx.batmgr.get_status(&status);
  pwr_ctx.batmgr.get_faults(&faults);
  debug.println("--------[  INFO  ]--------");
  debug.print("  batmgr PID: 0x"); debug.println(pid, HEX);
  debug.println("--------[ STATUS ]--------");
  debug.print("  IINDPM_STAT:       0x"); debug.println(status.fields.IINDPM_STAT, HEX);
  debug.print("  VINDPM_STAT:       0x"); debug.println(status.fields.VINDPM_STAT, HEX);
  debug.print("  WD_STAT:           0x"); debug.println(status.fields.WD_STAT, HEX);
  debug.print("  PG_STAT:           0x"); debug.println(status.fields.PG_STAT, HEX);
  debug.print("  AC2_PRESENT_STAT:  0x"); debug.println(status.fields.AC2_PRESENT_STAT, HEX);
  debug.print("  AC1_PRESENT_STAT:  0x"); debug.println(status.fields.AC1_PRESENT_STAT, HEX);
  debug.print("  VBUS_PRESENT_STAT: 0x"); debug.println(status.fields.VBUS_PRESENT_STAT, HEX);
  debug.print("  CHG_STAT:          0x"); debug.println(status.fields.CHG_STAT, HEX);
  debug.print("  VBUS_STAT:         0x"); debug.println(status.fields.VBUS_STAT, HEX);
  debug.print("  BC1_2_DONE_STAT:   0x"); debug.println(status.fields.BC1_2_DONE_STAT, HEX);
  debug.print("  ICO_STAT:          0x"); debug.println(status.fields.ICO_STAT, HEX);
  debug.print("  TREG_STAT:         0x"); debug.println(status.fields.TREG_STAT, HEX);
  debug.print("  DPDM_STAT:         0x"); debug.println(status.fields.DPDM_STAT, HEX);
  debug.print("  VBAT_PRESENT_STAT: 0x"); debug.println(status.fields.VBAT_PRESENT_STAT, HEX);
  debug.print("  ACRB2_STAT:        0x"); debug.println(status.fields.ACRB2_STAT, HEX);
  debug.print("  ACRB1_STAT:        0x"); debug.println(status.fields.ACRB1_STAT, HEX);
  debug.print("  ADC_DONE_STAT:     0x"); debug.println(status.fields.ADC_DONE_STAT, HEX);
  debug.print("  VSYS_STAT:         0x"); debug.println(status.fields.VSYS_STAT, HEX);
  debug.print("  CHG_TMR_STAT:      0x"); debug.println(status.fields.CHG_TMR_STAT, HEX);
  debug.print("  TRICHG_TMR_STAT:   0x"); debug.println(status.fields.TRICHG_TMR_STAT, HEX);
  debug.print("  PRECHG_TMR_STAT:   0x"); debug.println(status.fields.PRECHG_TMR_STAT, HEX);
  debug.print("  VBATOTG_LOW_STAT:  0x"); debug.println(status.fields.VBATOTG_LOW_STAT, HEX);
  debug.print("  TS_COLD_STAT:      0x"); debug.println(status.fields.TS_COLD_STAT, HEX);
  debug.print("  TS_COOL_STAT:      0x"); debug.println(status.fields.TS_COOL_STAT, HEX);
  debug.print("  TS_WARM_STAT:      0x"); debug.println(status.fields.TS_WARM_STAT, HEX);
  debug.print("  TS_HOT_STAT:       0x"); debug.println(status.fields.TS_HOT_STAT, HEX);
  debug.println("--------[ FAULTS ]--------");
  debug.print("  IBAT_REG_STAT:     0x"); debug.println(faults.fields.IBAT_REG_STAT, HEX);
  debug.print("  VBUS_OVP_STAT:     0x"); debug.println(faults.fields.VBUS_OVP_STAT, HEX);
  debug.print("  VBAT_OVP_STAT:     0x"); debug.println(faults.fields.VBAT_OVP_STAT, HEX);
  debug.print("  IBUS_OCP_STAT:     0x"); debug.println(faults.fields.IBUS_OCP_STAT, HEX);
  debug.print("  IBAT_OCP_STAT:     0x"); debug.println(faults.fields.IBAT_OCP_STAT, HEX);
  debug.print("  CONV_OCP_STAT:     0x"); debug.println(faults.fields.CONV_OCP_STAT, HEX);
  debug.print("  VAC2_OVP_STAT:     0x"); debug.println(faults.fields.VAC2_OVP_STAT, HEX);
  debug.print("  VAC1_OVP_STAT:     0x"); debug.println(faults.fields.VAC1_OVP_STAT, HEX);
  debug.print("  VSYS_SHORT_STAT:   0x"); debug.println(faults.fields.VSYS_SHORT_STAT, HEX);
  debug.print("  VSYS_OVP_STAT:     0x"); debug.println(faults.fields.VSYS_OVP_STAT, HEX);
  debug.print("  OTG_OVP_STAT:      0x"); debug.println(faults.fields.OTG_OVP_STAT, HEX);
  debug.print("  OTG_UVP_STAT:      0x"); debug.println(faults.fields.OTG_UVP_STAT, HEX);
  debug.print("  TSHUT_STAT:        0x"); debug.println(faults.fields.TSHUT_STAT, HEX);
}