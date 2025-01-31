#pragma once

#include <Arduino.h>
#include "stm32g4xx_ll_system.h"
#include "usbd_cdc_if.h"
#include "SPI.h"
#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "PeripheralPins.h"
#include "core_debug.h"
#include "utility/spi_com.h"
#include "pinconfig.h"
#include "stm32yyxx_ll_spi.h"

void init_dfu_trigger_handling(void);


#define MT6701_CPR 16384.0f
#define MT6701_BITORDER MSBFIRST
#define MT6701_DATA_POS 1

class CustomMT6701SSI : public Sensor {
public:
  CustomMT6701SSI(uint32_t miso, uint32_t sclk, uint32_t ssel);
  virtual ~CustomMT6701SSI();

  virtual void init(void);

  float getSensorAngle() override; // angle in radians, return current value

protected:
  uint16_t readRawAngleSSI();
  
  spi_t         _spi;

  SPISettings settings;
  SPIClass* spi;
	int nCS = PNUM_NOT_DEFINED;

private:
  // Use SPI mode 2, capture on falling edge. First bit is not valid data, so have to read 25 bits to get a full SSI frame.
  // SSI frame is 1 bit ignore, 14 bits angle, 4 bit status and 6 bit CRC.
  uint32_t clockFreq = 8000000;
  BitOrder bitOrder = MT6701_BITORDER;
  SPIMode  dataMode = SPI_MODE2;
};