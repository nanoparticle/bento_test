#pragma once

#include <Arduino.h>
#include "stm32g4xx_ll_system.h"
#include "usbd_cdc_if.h"
#include "SPI.h"
#include "PeripheralPins.h"
#include "core_debug.h"
#include "utility/spi_com.h"
#include "pinconfig.h"
#include "stm32yyxx_ll_spi.h"

void init_dfu_trigger_handling(void);
