#include "util.h"

unsigned successful_reset __attribute__((__section__(".noinit")));
HardwareTimer dfu_trigger_timer (TIM16);

// https://github.com/VIPQualityPost/aioli-foc/blob/main/firmware/src/drv_reset.c
void perform_system_reset(void) { // Soft reset
  __disable_irq();
  NVIC_SystemReset();
}

void jump_to_bootloader(void) { // Magic instructions that end up in DFU mode
  __enable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

  const uint32_t p = (*((uint32_t *)0x1FFF0000));
  __set_MSP(p);

  void (*SysMemBootJump)(void);
  SysMemBootJump = (void (*)(void))(*((uint32_t *)0x1FFF0004));
  SysMemBootJump();

  while (1) {
  }
}

void dfu_trigger_handler(void) {
  // Check for standard 1200 baud magic value DFU trigger on SerialUSB
  USBD_CDC_LineCodingTypeDef line_coding;
  USBD_CDC_fops.Control(CDC_GET_LINE_CODING, (uint8_t*)&line_coding, 0);
  if (line_coding.bitrate == 1200) jump_to_bootloader();
}

void init_dfu_trigger_handling(void) {
  // MCU fails to init properly when exiting DFU mode
  // This forces a single soft reset using a persistent variable in ram
  if (successful_reset) {
    successful_reset = 0;
  } else {
    successful_reset = 1;
    perform_system_reset();
  }
  // Initialize periodic check for DFU trigger
  dfu_trigger_timer.setOverflow(10, HERTZ_FORMAT); // 10 Hz
  dfu_trigger_timer.attachInterrupt(dfu_trigger_handler);
  dfu_trigger_timer.resume();
}



CustomMT6701SSI::CustomMT6701SSI(uint32_t miso, uint32_t sclk, uint32_t ssel) : nCS(ssel) {
  memset((void *)&_spi, 0, sizeof(_spi));
  _spi.pin_miso = digitalPinToPinName(miso);
  _spi.pin_mosi = digitalPinToPinName(PNUM_NOT_DEFINED);
  _spi.pin_sclk = digitalPinToPinName(sclk);
  _spi.pin_ssel = digitalPinToPinName(PNUM_NOT_DEFINED);
}


CustomMT6701SSI::~CustomMT6701SSI() {

}

void CustomMT6701SSI::init(void) {
  pinMode(nCS, OUTPUT);
  digitalWrite(nCS, HIGH);
  _spi.handle.State = HAL_SPI_STATE_RESET;
  
  uint32_t spi_freq = 0;
  uint32_t pull = 0;

  // Determine the SPI to use
  SPI_TypeDef *spi_miso = (SPI_TypeDef *)pinmap_peripheral(_spi.pin_miso, PinMap_SPI_MISO);
  SPI_TypeDef *spi_sclk = (SPI_TypeDef *)pinmap_peripheral(_spi.pin_sclk, PinMap_SPI_SCLK);
  /* Pins MISO/SCLK must not be NP. MOSI/SSEL can be NP. */
  if (spi_miso == NP || spi_sclk == NP) {
    core_debug("ERROR: at least one SPI pin has no peripheral\n");
    return;
  }

  _spi.spi = (SPI_TypeDef *)pinmap_merge_peripheral(spi_miso, spi_sclk);
  // Are all pins connected to the same SPI instance?
  if (_spi.spi == NP) {
    core_debug("ERROR: SPI pins mismatch\n");
    return;
  }
  
  _spi.handle.Init.NSS = SPI_NSS_SOFT;

  /* Fill default value */
  _spi.handle.Instance               = _spi.spi;
  _spi.handle.Init.Mode              = SPI_MODE_MASTER;

  spi_freq = spi_getClkFreq(&_spi);
  /* For SUBGHZSPI,  'SPI_BAUDRATEPRESCALER_*' == 'SUBGHZSPI_BAUDRATEPRESCALER_*' */
  if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  } else if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  } else if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  } else if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  } else if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  } else if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  } else if (clockFreq >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  } else {
    /*
     * As it is not possible to go below (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ).
     * Set prescaler at max value so get the lowest frequency possible.
     */
    _spi.handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  }

  _spi.handle.Init.Direction         = SPI_DIRECTION_2LINES;

  if ((dataMode == SPI_MODE0) || (dataMode == SPI_MODE2)) {
    _spi.handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  } else {
    _spi.handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
  }

  if ((dataMode == SPI_MODE0) || (dataMode == SPI_MODE1)) {
    _spi.handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  } else {
    _spi.handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  }

  _spi.handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  _spi.handle.Init.CRCPolynomial     = 7;
  _spi.handle.Init.DataSize          = SPI_DATASIZE_8BIT;

  if (bitOrder == 0) {
    _spi.handle.Init.FirstBit          = SPI_FIRSTBIT_LSB;
  } else {
    _spi.handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  }

  _spi.handle.Init.TIMode            = SPI_TIMODE_DISABLE;
#if defined(SPI_NSS_PULSE_DISABLE)
  _spi.handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
#endif
#ifdef SPI_MASTER_KEEP_IO_STATE_ENABLE
  _spi.handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommended setting to avoid glitches */
#endif

  /* Configure SPI GPIO pins */
  pinmap_pinout(_spi.pin_miso, PinMap_SPI_MISO);
  pinmap_pinout(_spi.pin_sclk, PinMap_SPI_SCLK);
  /*
  * According the STM32 Datasheet for SPI peripheral we need to PULLDOWN
  * or PULLUP the SCK pin according the polarity used.
  */
  pull = (_spi.handle.Init.CLKPolarity == SPI_POLARITY_LOW) ? GPIO_PULLDOWN : GPIO_PULLUP;
  pin_PullConfig(get_GPIO_Port(STM_PORT(_spi.pin_sclk)), STM_LL_GPIO_PIN(_spi.pin_sclk), pull);
  pinmap_pinout(_spi.pin_ssel, PinMap_SPI_SSEL);

#if defined SPI1_BASE
  // Enable SPI clock
  if (_spi.handle.Instance == SPI1) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
  }
#endif

#if defined SPI2_BASE
  if (_spi.handle.Instance == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
  }
#endif

#if defined SPI3_BASE
  if (_spi.handle.Instance == SPI3) {
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();
  }
#endif

#if defined SPI4_BASE
  if (_spi.handle.Instance == SPI4) {
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_SPI4_FORCE_RESET();
    __HAL_RCC_SPI4_RELEASE_RESET();
  }
#endif

#if defined SPI5_BASE
  if (_spi.handle.Instance == SPI5) {
    __HAL_RCC_SPI5_CLK_ENABLE();
    __HAL_RCC_SPI5_FORCE_RESET();
    __HAL_RCC_SPI5_RELEASE_RESET();
  }
#endif

#if defined SPI6_BASE
  if (_spi.handle.Instance == SPI6) {
    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_SPI6_FORCE_RESET();
    __HAL_RCC_SPI6_RELEASE_RESET();
  }
#endif

#if defined SUBGHZSPI_BASE
  if (_spi.handle.Instance == SUBGHZSPI) {
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();
    __HAL_RCC_SUBGHZSPI_FORCE_RESET();
    __HAL_RCC_SUBGHZSPI_RELEASE_RESET();
  }
#endif

  HAL_SPI_Init(&_spi.handle);

  /* In order to set correctly the SPI polarity we need to enable the peripheral */
  __HAL_SPI_ENABLE(&_spi.handle);

  this->Sensor::init();
}

// check 40us delay between each read?
float CustomMT6701SSI::getSensorAngle() {
    float angle_data = readRawAngleSSI();
    angle_data = ( angle_data / (float)MT6701_CPR ) * _2PI;
    // return the shaft angle
    return angle_data;
}


uint16_t CustomMT6701SSI::readRawAngleSSI() {
  digitalWrite(nCS, LOW);

  uint16_t buf = 0x0000;
  uint32_t tickstart, size = sizeof(buf);
  uint8_t *tx_buf = (uint8_t *)&buf;
  uint8_t *rx_buffer = (uint8_t *)&buf;

  tickstart = HAL_GetTick();

  while (size--) {
    while (!LL_SPI_IsActiveFlag_TXE(_spi.handle.Instance));
    LL_SPI_TransmitData8(_spi.handle.Instance, tx_buf ? *tx_buf++ : 0XFF);
    while (!LL_SPI_IsActiveFlag_RXNE(_spi.handle.Instance));
    *rx_buffer++ = LL_SPI_ReceiveData8(_spi.handle.Instance);
    if ((SPI_TRANSFER_TIMEOUT != HAL_MAX_DELAY) && (HAL_GetTick() - tickstart >= SPI_TRANSFER_TIMEOUT)) {
      break;
    }
  }
  /* Wait for end of transfer */
  while (LL_SPI_IsActiveFlag_BSY(_spi.handle.Instance));
  buf = ((buf & 0xff00) >> 8) | ((buf & 0xff) << 8);

  digitalWrite(nCS, HIGH);
  return (buf>>MT6701_DATA_POS)&0x3FFF;
};