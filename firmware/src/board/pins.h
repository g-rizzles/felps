#pragma once
#ifndef FELPS_PINS_H_
#define FELPS_PINS_H_

#define SNS_I_LSW_MUX           PA0
#define SNS_I_SYS               PA1
#define DBG_UART_TX             PA2
#define DBG_UART_RX             PA3
#define SIG_SPI1_CS0            PA4
#define SIG_SPI1_SCK            PA5
#define SIG_SPI1_MISO           PA6
#define SIG_SPI1_MOSI           PA7
#define EXP_ROUT                PA8
#define EXP_DIN                 PA9
#define SIG_EXP_DRV_INVALID_N   PA10
#define SNS_I_JACK1             PA11
#define SNS_I_JACK2             PA12
#define SIG_CE_N                PA15
#define SIG_SPI1_CS1            PB0
#define SIG_SPI1_CS2            PB1
#define SIG_EN_LSW5             PB2
#define SIG_EN_LSW1             PB3
#define SIG_EN_LSW2             PB4
#define SIG_EN_LSW3             PB5
#define SIG_EN_LSW4             PB6
#define SIG_LSW_MUX_S0          PB7
#define SIG_I2C_SCL             PB8
#define SIG_I2C_SDA             PB9
#define SIG_EN_LSW6             PB10
#define SIG_EN_LSW7             PB11
#define SIG_EN_LSW8             PB12
#define SIG_CHARGE_HIGH_Z_EN    PB13
#define SIG_INT_N               PB14
#define SIG_EXP_DRV_EN_N        PB15
#define SIG_EXP_DRV_FORCEOFF_N  PC6 
#define SIG_EXP_DRV_FORCE_ON    PC7
#define SIG_LSW_MUX_S1          PC13
#define SIG_QON_N               PD0
#define SIG_GPIO_EXP_INT        PD1
#define SIG_UC_EXTRA1           PD2
//#define SIG_UC_EXTRA2           PD3
//#define SIG_LSW_MUX_S2          PF3
#define SIG_LSW_MUX_S2          PD3

// Define "lines"
#define SIG_DISP_RST          SIG_SPI1_CS0
#define SIG_DISP_DC           SIG_SPI1_CS1
#define SIG_DISP_CS           SIG_SPI1_CS2
#define SIG_DISP_BUSY         SIG_SPI1_MISO

#endif // FELPS_PINS_H_