#pragma once
#ifndef FELPS_BOARD_INIT_H_
#define FELPS_BOARD_INIT_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "board.h"

#ifdef HAL_ADC_MODULE_ENABLED
#warning ADC enabled! Overwriting pins...
const PinMap PinMap_ADC[] = {
  {PA_0,  ADC1, STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 0, 0)}, // ADC1_IN0
  {PA_1,  ADC1, STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 1, 0)}, // ADC1_IN1
  {PA_11, ADC1, STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 15, 0)}, // ADC1_IN15
  {PA_12, ADC1, STM_PIN_DATA_EXT(STM_MODE_ANALOG, GPIO_NOPULL, 0, 16, 0)}, // ADC1_IN16
};
#endif

#ifdef HAL_I2C_MODULE_ENABLED
#warning I2C enabled! Overwriting pins...
const PinMap PinMap_I2C_SDA[] = {
  {PB_9,    I2C1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF6_I2C1)},
};
const PinMap PinMap_I2C_SCL[] = {
  {PB_8,   I2C1, STM_PIN_DATA(STM_MODE_AF_OD, GPIO_NOPULL, GPIO_AF6_I2C1)},
};
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#warning TIM enabled! Overwriting pins...
const PinMap PinMap_TIM[] = {
  {NC,    NP,   0}
};
#endif

#ifdef HAL_UART_MODULE_ENABLED
#warning UART enabled! Overwriting pins...
const PinMap PinMap_UART_TX[] = {
  {PA_2,   USART2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_USART2)},
  //{PA_9,   USART1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_USART1)},
};
const PinMap PinMap_UART_RX[] = {
  {PA_3,    USART2, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF1_USART2)},
  //{PA_8,    USART1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF14_USART1)},
};
const PinMap PinMap_UART_RTS[] = {
  {NC,    NP,   0}
};
const PinMap PinMap_UART_CTS[] = {
  {NC,    NP,   0}
};
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#warning SPI enabled! Overwriting pins...
const PinMap PinMap_SPI_MOSI[] = {
  {PA_7,  SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF0_SPI1)},
};
const PinMap PinMap_SPI_MISO[] = {
  {PA_6,  SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF0_SPI1)},
};
const PinMap PinMap_SPI_SCLK[] = {
  {PA_5, SPI1, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF0_SPI1)},
};
const PinMap PinMap_SPI_SSEL[] = {
  {NC,    NP,   0}
};
#endif

HardwareSerial debug(DBG_UART_RX, DBG_UART_TX);

static inline void setup_peripherals() {
  // TODO GPIO setup
#ifdef HAL_ADC_MODULE_ENABLED
  // TODO ADC setup
#endif // HAL_ADC_MODULE_ENABLED
#ifdef HAL_SPI_MODULE_ENABLED
  SPI.setMOSI(SIG_SPI1_MOSI);
  SPI.setMISO(SIG_SPI1_MISO);
  SPI.setSCLK(SIG_SPI1_SCK);
  SPI.begin();
#endif //HAL_SPI_MODULE_ENABLED
#ifdef HAL_I2C_MODULE_ENABLED
  Wire.setSDA(SIG_I2C_SDA);
  Wire.setSCL(SIG_I2C_SCL);
  Wire.begin();
#endif // HAL_I2C_MODULE_ENABLED
#ifdef HAL_UART_MODULE_ENABLED
  debug.begin(
    // See HardwareSerial source in stm32duino project for details
    115200, // baud
    SERIAL_8N1
  );
  while (!debug) {;}
  //expansion.begin(115200);
  //while (!expansion) {;}
#endif // HAL_UART_MODULE_ENABLED
}

static inline void setup_error_handler(int rc) {
  while (1) {;}
  //HAL_NVIC_SystemReset();
}

#endif // FELPS_BOARD_INIT_H_