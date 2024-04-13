#pragma once
#ifndef BQ25672_H_
#define BQ25672_H_
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "src/os/err.h"

#define BQ25672_ENABLED         (HIGH)
#define BQ25672_DISABLED        (LOW)

#define BQ25672_I2C_ADDR        (0x6B)
#define BQ25672_PART_NUMBER     (0x21)

#define BQ25672_VSYSMIN_MV_MIN  (2500)
#define BQ25672_VSYSMIN_MV_MAX  (16000)
#define BQ25672_MIN_SYS_V_CALC(MV) \
    ((MV - 2500) / 250)


#define BQ25672_MASK(VAL, MASK) \
    (VAL & MASK)

/** @brief Regulation configuration parameters. To be used in API calls, not representative of
 * register structure. */
typedef struct {
    uint16_t vsys_min_mv; // minimal system voltage in mV, closest possible val used
} bq25672_regulation_config_t;

enum class BQ25672_ADC_CH : uint8_t {
    ADC_CH_IBUS,
    ADC_CH_IBAT,
    ADC_CH_VBUS,
    ADC_CH_VBAT,
    ADC_CH_VSYS,
    ADC_CH_TS,
    ADC_CH_TDIE,
};

/** @brief Enumerated cell config parameter values. Values are bit shifted appropriately. */
enum class BQ25672_CELL_CONFIG : uint8_t {
    CELL_CONFIG_1S = 0b00000000,
    CELL_CONFIG_2S = 0b01000000,
    CELL_CONFIG_3S = 0b10000000,
    CELL_CONFIG_4S = 0b11000000,
};

/** @brief Enumerate watchdog timer config parameter values. Values are bit shifted 
 * appropriately to be directly written to the appropriate register.*/
enum class BQ25672_WATCHDOG_CONFIG : uint8_t {
    TIMER_DISABLE = 0b000,
    TIMER_0_5_SEC = 0b001,
    TIMER_1_SEC   = 0b010,
    TIMER_2_SEC   = 0b011,
    TIMER_20_SEC  = 0b100,
    TIMER_40_SEC  = 0b101,
    TIMER_80_SEC  = 0b110,
    TIMER_160_SEC = 0b111,
};

typedef union {
    uint8_t regs[5];
    struct {
        // Register 1B charger status 0
        uint8_t VBUS_PRESENT_STAT : 1;
        uint8_t AC1_PRESENT_STAT : 1;
        uint8_t AC2_PRESENT_STAT : 1; 
        uint8_t PG_STAT : 1;
        uint8_t RESERVED0 : 1;
        uint8_t WD_STAT : 1;  
        uint8_t VINDPM_STAT : 1;     
        uint8_t IINDPM_STAT : 1;
        // Register 1C charger status 1
        uint8_t BC1_2_DONE_STAT : 1;
        uint8_t VBUS_STAT : 4;
        uint8_t CHG_STAT : 3;
        // Register 1D charger status 2        
        uint8_t VBAT_PRESENT_STAT : 1;
        uint8_t DPDM_STAT : 1;
        uint8_t TREG_STAT : 1;
        uint8_t RESERVED1 : 3;
        uint8_t ICO_STAT : 2;
        // Register 1E charger status 3
        uint8_t RESERVED2 : 1;
        uint8_t PRECHG_TMR_STAT : 1;
        uint8_t TRICHG_TMR_STAT : 1;
        uint8_t CHG_TMR_STAT : 1;
        uint8_t VSYS_STAT : 1;
        uint8_t ADC_DONE_STAT : 1;
        uint8_t ACRB1_STAT : 1;
        uint8_t ACRB2_STAT : 1;
        // Register 1F charger status 4        
        uint8_t TS_HOT_STAT : 1;
        uint8_t TS_WARM_STAT : 1;
        uint8_t TS_COOL_STAT : 1;
        uint8_t TS_COLD_STAT : 1;
        uint8_t VBATOTG_LOW_STAT : 1;
        uint8_t RESERVED3 : 3;
    } fields;
} bq25672_charger_status_u;

typedef union {
    uint8_t regs[2];
    struct  {
        // Register 20 fault status 4  
        uint8_t VAC1_OVP_STAT : 1;
        uint8_t VAC2_OVP_STAT : 1;
        uint8_t CONV_OCP_STAT : 1;
        uint8_t IBAT_OCP_STAT : 1;
        uint8_t IBUS_OCP_STAT : 1;
        uint8_t VBAT_OVP_STAT : 1;
        uint8_t VBUS_OVP_STAT : 1;
        uint8_t IBAT_REG_STAT : 1;
        // Register 21 fault status 4  
        uint8_t RESERVED1 : 2;
        uint8_t TSHUT_STAT : 1;
        uint8_t RESERVED0 : 1;
        uint8_t OTG_UVP_STAT : 1;
        uint8_t OTG_OVP_STAT : 1;
        uint8_t VSYS_OVP_STAT : 1;
        uint8_t VSYS_SHORT_STAT : 1;
    } fields;
} bq25672_fault_status_u;

class bq25672
{
    private:
        TwoWire* wire;
    public:
        enum class REG : uint8_t {
            REG_00_MIN_SYS_V = 0x00,
            REG_01_CHG_V_LIM = 0x01,
            REG_03_CHG_I_LIM = 0x03,
            REG_05_INP_V_LIM = 0x05,
            REG_06_INP_I_LIM = 0x06,
            REG_08_PRECHG_CTRL = 0x08,
            REG_09_TERM_CTRL = 0x09,
            REG_0A_RECHG_CTRL = 0x0A,
            REG_0B_VOTG_REG = 0x0B,
            REG_0D_IOTG_REG = 0x0D,
            REG_0E_TIMER_CTRL = 0x0E,
            REG_0F_CHG_CTRL_0 = 0x0F,
            REG_10_CHG_CTRL_1 = 0x10,
            REG_11_CHG_CTRL_2 = 0x11,
            REG_12_CHG_CTRL_3 = 0x12,
            REG_13_CHG_CTRL_4 = 0x13,
            REG_14_CHG_CTRL_5 = 0x14,
            REG_15_MPPT_CTRL = 0x15,
            REG_16_TEMP_CTRL = 0x16,
            REG_17_NTC_CTRL_0 = 0x17,
            REG_18_NTC_CTRL_1 = 0x18,
            REG_19_ICO_I_LIM = 0x19,
            REG_1B_CHG_STAT_0 = 0x1B,
            REG_1C_CHG_STAT_1 = 0x1C,
            REG_1D_CHG_STAT_2 = 0x1D,
            REG_1E_CHG_STAT_3 = 0x1E,
            REG_1F_CHG_STAT_4 = 0x1F,
            REG_20_FLT_STAT_0 = 0x20,
            REG_21_FLT_STAT_1 = 0x21,
            REG_22_CHG_FLAG_0 = 0x22,
            REG_23_CHG_FLAG_1 = 0x23,
            REG_24_CHG_FLAG_2 = 0x24,
            REG_25_CHG_FLAG_3 = 0x25,
            REG_26_FLT_FLAG_0 = 0x26,
            REG_27_FLT_FLAG_1 = 0x27,
            REG_28_CHG_MASK_0 = 0x28,
            REG_29_CHG_MASK_1 = 0x29,
            REG_2A_CHG_MASK_2 = 0x2A,
            REG_2B_CHG_MASK_3 = 0x2B,
            REG_2C_FLT_MASK_0 = 0x2C,
            REG_2D_FLT_MASK_1 = 0x2D,
            REG_2E_ADC_CTRL = 0x2E,
            REG_2F_ADC_FUNC_DIS_0 = 0x2F,
            REG_30_ADC_FUNC_DIS_1 = 0x30,
            REG_31_IBUS_ADC = 0x31,
            REG_33_IBAT_ADC = 0x33,
            REG_35_VBUS_ADC = 0x35,
            REG_37_VAC1_ADC = 0x37,
            REG_39_VAC2_ADC = 0x39,
            REG_3B_VBAT_ADC = 0x3B,
            REG_3D_VSYS_ADC = 0x3D,
            REG_3F_TS_ADC = 0x3F,
            REG_41_TDIE_ADC = 0x41,
            REG_43_DP_ADC = 0x43,
            REG_45_DN_ADC = 0x45,
            REG_47_DPDM_DRV = 0x47,
            REG_48_PART_INFO = 0x48,
        };

        void write(const REG reg, const uint8_t data, const bool stop = true) {
            wire->beginTransmission(BQ25672_I2C_ADDR);
            wire->write((uint8_t)reg);
            wire->write(data);
            wire->endTransmission(stop);
        }

        void write_(const REG reg, const uint16_t data, const bool stop = true) {
            wire->beginTransmission(BQ25672_I2C_ADDR);
            wire->write((uint8_t)reg);
            wire->write((uint8_t *)&data, 2);
            wire->endTransmission(stop);
        }

        uint8_t read(const REG reg) {
            uint8_t data = 0;
            write(reg, false);
            wire->requestFrom((uint8_t)BQ25672_I2C_ADDR, (uint8_t)1);
            if(wire->available()==1) {
                data = wire->read();
            }
            return data;
        }

        uint16_t read_(const REG reg) {
            uint16_t data = 0;
            write(reg, false);
            wire->requestFrom((uint8_t)BQ25672_I2C_ADDR, (uint8_t)2);
            if(wire->available()==2) {
                data = wire->read();
                data = (data << 8) | wire->read();
            }
            return data;
        }

        void rmw(const REG reg, const uint8_t data, const uint8_t mask, const bool stop = true) {
            uint8_t newdata = read(reg);
            newdata = (data & mask) | (newdata & ~mask);
            write(reg, newdata, stop);
        }

        void rmw_(const REG reg, const uint16_t data, const uint16_t mask, const bool stop = true) {
            uint16_t newdata = read_(reg);
            newdata = (data & mask) | (newdata & ~mask);
            write_(reg, newdata, stop);
        }

        bq25672(TwoWire& w) : wire(&w){};

        /**
         * @brief Gets part_info from the part info register.
         * @returns part info from register
         */
        uint8_t get_part_info() {
            return BQ25672_MASK(read(REG::REG_48_PART_INFO), 0b00111111);
        }

        /**
         * @brief Get the specified ADC channel raw measurement
         * @param[in]   ch enumerated channel to retrieve 
         * @return raw ADC count measured from device 
         */
        uint16_t get_adc_ch(BQ25672_ADC_CH ch) {
            uint16_t reading = 0;
            switch (ch) {
                case BQ25672_ADC_CH::ADC_CH_IBUS:
                    reading = this->read_(REG::REG_31_IBUS_ADC);
                    break;
                case BQ25672_ADC_CH::ADC_CH_IBAT:
                    reading = this->read_(REG::REG_33_IBAT_ADC);
                    break;
                case BQ25672_ADC_CH::ADC_CH_VBUS:
                    reading = this->read_(REG::REG_35_VBUS_ADC);
                    break;
                case BQ25672_ADC_CH::ADC_CH_VBAT:
                    reading = this->read_(REG::REG_3B_VBAT_ADC);
                    break;
                case BQ25672_ADC_CH::ADC_CH_VSYS:
                    reading = this->read_(REG::REG_3D_VSYS_ADC);
                    break;
                case BQ25672_ADC_CH::ADC_CH_TS:
                    reading = this->read_(REG::REG_3F_TS_ADC);
                    break;
                case BQ25672_ADC_CH::ADC_CH_TDIE:
                    reading = this->read_(REG::REG_41_TDIE_ADC);
                    break;
            }
            return reading;
        }

        /**
         * @brief Configures properties related to system voltage regulation
         * @param[in]   config pointer to structure containing config parameters
         * @param[in]   check_only if true, returns an error if current config does not match
         *              requested, if false updates the configs to requested
         * @returns enumerated error code, < 0 for fault, 0 for no fault
        */
        int configure_regulation(const bq25672_regulation_config_t * const config,
                                 const bool check_only = false) {
            uint8_t current_vsysmin = BQ25672_MASK(read(REG::REG_00_MIN_SYS_V), 0b00111111);
            uint8_t new_vsysmin = BQ25672_MIN_SYS_V_CALC(config->vsys_min_mv);
            if ((config->vsys_min_mv < BQ25672_VSYSMIN_MV_MIN) ||
                (config->vsys_min_mv > BQ25672_VSYSMIN_MV_MAX)) {
                return -FELPS_ERR_BAD_PARAM;
            }
            if (check_only) {
                if ((current_vsysmin != new_vsysmin)) {
                    return -FELPS_ERR_BAD_CONFIG;
                }
            } else {
                rmw(REG::REG_00_MIN_SYS_V, new_vsysmin, 0b00111111);
            }
            return FELPS_ERR_NONE;
        }

        /**
         * @brief Get status registers from the device.
         * @param[in,out] status pointer to a var to store read values
         * @return enumerated error code, < 0 for fault, 0 for no fault
         */
        int get_status(bq25672_charger_status_u * status) {
            if (status == NULL) {
                return -FELPS_ERR_UNKNOWN;
            }
            memset(status, 0x00, sizeof(bq25672_charger_status_u));
            status->regs[0] = read(REG::REG_1B_CHG_STAT_0);
            status->regs[1] = read(REG::REG_1C_CHG_STAT_1);
            status->regs[2] = read(REG::REG_1D_CHG_STAT_2);
            status->regs[3] = read(REG::REG_1E_CHG_STAT_3);
            status->regs[4] = read(REG::REG_1F_CHG_STAT_4);
            return FELPS_ERR_NONE;
        }

        /**
         * @brief Get fault registers from the device.
         * @param[in,out] faults pointer to a var to store read values
         * @return enumerated error code, < 0 for fault, 0 for no fault
         */
        int get_faults(bq25672_fault_status_u * faults) {
            if (faults == NULL) {
                return -FELPS_ERR_UNKNOWN;
            }
            memset(faults, 0x00, sizeof(bq25672_fault_status_u));
            faults->regs[0] = read(REG::REG_20_FLT_STAT_0);
            faults->regs[1] = read(REG::REG_21_FLT_STAT_1);
            return FELPS_ERR_NONE;
        }
        
        /**
         * @brief Enable or disable the onboard ADC.
         * @param[in]   enable true to enable, false to disable
         */
        void enable_adc(const bool enable) {
            uint8_t setting = 0x00;
            if (enable) {
                setting = (1<<7);
            }
            rmw(REG::REG_2E_ADC_CTRL, setting, (1<<7));
        }

        /**
         * @brief Enable or disable battery charging.
         * @param[in]   enable true to enable, false to disable
         */
        void enable_charging(const bool enable) {
            uint8_t setting = 0x00;
            if (enable) {
                setting = (1<<5);
            }
            rmw(REG::REG_0F_CHG_CTRL_0, setting, (1<<5));
        }
        /**
         * @brief Enable or disable hi-z mode.
         * @param[in]   enable true to enable, false to disable
         */
        void enable_hiz(const bool enable) {
            uint8_t setting = 0x00;
            if (enable) {
                setting = (1<<2);
            }
            rmw(REG::REG_0F_CHG_CTRL_0, setting, (1<<2));
        }

        /**
         * @brief Enable or disable MPPT.
         * @param[in]   enable true to enable, false to disable
         */
        void enable_mppt(const bool enable) {
            uint8_t setting = 0x00;
            if (enable) {
                setting = (0x01);
            }
            rmw(REG::REG_15_MPPT_CTRL, setting, 0x01);
        }

        /**
         * @brief Update watchdog timeout configuration.
         * @param[in]   timer watchdog timer configuration 
         */
        void configure_watchdog(BQ25672_WATCHDOG_CONFIG timer) {
            rmw(REG::REG_10_CHG_CTRL_1, (uint8_t)timer, 0b111);
        }

        /**
         * @brief Software resets the IC.
         * @note This resets all configuration! You must re-apply any desired configuration.
         */
        void reset() {
            rmw(REG::REG_09_TERM_CTRL, (1 << 6), 0b010000000);
        }

        /**
         * @brief Shuts down regulated power. External power must be applied to exit this
         * condition!
         * @param[in]   immediate if true, change happens immediately, 
         *              if false, happens in 10 seconds
         */
        void enter_shutdown_mode(bool immediate) {
            rmw(REG::REG_11_CHG_CTRL_2, 
                0b00000010 | immediate, 
                0b00000110 | immediate);
        }

        /**
         * @brief Places the battery into SHIP mode.
         * @param[in]   immediate if true, change happens immediately, 
         *              if false, happens in 10 seconds
         */
        void enter_ship_mode(bool immediate) {
            rmw(REG::REG_11_CHG_CTRL_2, 
                0b00000100 | immediate, 
                0b00000110 | immediate);
        }

        /**
         * @brief Resets system power.
         * @param[in]   immediate if true, change happens immediately, 
         *              if false, happens in 10 seconds
         */
        void system_power_reset(bool immediate) {
            rmw(REG::REG_11_CHG_CTRL_2, 
                0b00000110 | immediate, 
                0b00000110 | immediate);
        }

        /**
         * @brief Reset to IDLE mode.
         */
        void enter_idle_mode() {
            rmw(REG::REG_11_CHG_CTRL_2, 0b00000000, 0b00000111);
        }

        /**
         * @brief Begins communications and interrupt handling.
         * @returns enumerated error code, < 0 for fault, 0 for no fault
         * @details
         * Verifies device responds at address and returns the expected part number value.
         */
        int begin(bool ship_fet_present) {
            int rc = -FELPS_ERR_UNKNOWN;
            do {
                // Set I2C speed
                this->wire->setClock(100000);
                // Check that part info byte is valid
                rc = get_part_info();
                if (rc != BQ25672_PART_NUMBER) {
                    #if VERBOSE_DEBUG
                    debug.print("[bq25672]: part_info fail! ");
                    debug.println(rc);
                    #endif // VERBOSE_DEBUG
                    rc = -FELPS_ERR_BAD_PART;
                    break;
                }
                // Configure ship fet present bit
                if (ship_fet_present) {
                    this->rmw(REG::REG_14_CHG_CTRL_5, 0b10000000, 0b10000000);
                }
                rc = FELPS_ERR_NONE;
            } while (0);
            return rc;
        }
};

#endif // BQ25672_H_