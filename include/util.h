#pragma once

#include <Arduino.h>
#include "stm32g4xx_ll_system.h"
#include "usbd_cdc_if.h"

void init_dfu_trigger_handling(void);