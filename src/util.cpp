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