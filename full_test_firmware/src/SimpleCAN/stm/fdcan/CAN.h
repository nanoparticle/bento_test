#pragma once

#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "Arduino.h"
#include "SimpleCAN/BaseCAN.h"

enum STM_FDCAN_PROFILE {
  FD_CANABLE_1MBAUD_2MBAUD,
  FD_CANABLE_1MBAUD_5MBAUD,
  FD_GENERIC_1MBAUD_5MBAUD,
  GENERIC_1MBAUD,
};

class STM_FDCAN : public BaseCAN
{

public:
	STM_FDCAN(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN = CAN_NC);

	bool begin(int can_bitrate) override;
	bool begin(STM_FDCAN_PROFILE profile);
	void end() override;

	void filter(CanFilter filter) override;

	CanStatus subscribe(void (*_messageReceiveCallback)() = nullptr);
	CanStatus unsubscribe();

	int write(CanMsg const &msg) override;
	CanMsg read() override;
	size_t available() override;

	static FDCAN_HandleTypeDef hcan_;
	static void (*_callbackFunction)();
	static uint16_t pinRX_;
	static uint16_t pinTX_;
	static uint16_t pinSHDN_;

private:
	CanFilter filter_;
	void applyFilter(); // filter is applied after begin() is called
	bool started_ = false;
	uint32_t lengthToDLC(uint32_t length);
	uint32_t dlcToLength(uint32_t dlc);
	FDCAN_RxHeaderTypeDef rxHeader_;
	FDCAN_TxHeaderTypeDef txHeader_;
	CanStatus logStatus(char op, HAL_StatusTypeDef status);
};

#if CAN_HOWMANY > 0
extern STM_FDCAN CAN;
#endif

#endif