#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
// #include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

// #include "SimpleCAN.h"
#include "SimpleCAN/SimpleCAN.h"
// #include <CSE_ArduinoRS485.h>
#include <Bonezegei_RS485.h>

#include "util.h"


// ONBOARD PERIPHERALS
// DRV8316 3-Phase Driver Pin Definitions
#define SPI2_DRV_MISO PB14
#define SPI2_DRV_MOSI PB15
#define SPI2_DRV_SCK PB13
#define SPI2_DRV_CS PC6
SPIClass SPI2_DRV(SPI2_DRV_MOSI, SPI2_DRV_MISO, SPI2_DRV_SCK, PNUM_NOT_DEFINED);

#define DRV_NFAULT PC14
#define PB1_DRV_PWM_EN PB1

#define TIM1_CH1_PWM_A PA8
#define TIM1_CH2_PWM_B PA9
#define TIM1_CH3_PWM_C PA10

#define ADC2_IN5_SOA PC4
#define ADC2_IN12_SOB PB2
#define ADC2_IN17_SOC PA4

// MT6701 Angle Sensor Pin Definitions
#define SPI3_ENC_MISO PC11
#define SPI3_ENC_SCK PC10
#define SPI3_ENC_CS PC15
// SPIClass SPI3_ENC(PB5_ALT1, SPI3_ENC_MISO, SPI3_ENC_SCK, PNUM_NOT_DEFINED);

// User Button and LED Pin Definitions
#define PC13_USR_BTN PC13
#define TIM8_CH1_USR_LED PB6

// Bus Voltage and Current Sense Pin Definitions
#define ADC1_IN1_VSENSE PA0
#define ADC1_IN11_CSENSE PB12

// RS4XX Transciever Pin Definitions
#define USART2_RX PA3
#define USART2_TX PA2
#define USART2_DE PA1
HardwareSerial Serial2 (USART2_RX, USART2_TX);
Bonezegei_RS485 RS485 (Serial2, USART2_DE);

// CAN Bus Transciever Pin Definitions
#define FDCAN_RX_BOOT0 PB8
#define FDCAN_TX PB9
STM_FDCAN CAN (FDCAN_RX_BOOT0, FDCAN_TX);


// EXTERNAL PERIPHERALS
// ABZ Quadrature Encoder Input Pin Definitions
#define ENCODER_PPR 500
#define TIM3_CH1_ENCA PB4
#define TIM3_CH1_ENCB PB5
#define TIM3_ETR_ENCZ_SWO PB3
STM32HWEncoder encoder (ENCODER_PPR, TIM3_CH1_ENCA, TIM3_CH1_ENCB, TIM3_ETR_ENCZ_SWO);

// Auxiliary I2C Input Pin Definitions
#define I2C1_SDA PB7
#define I2C1_SCL PA15
TwoWire Wire1 (I2C1_SDA, I2C1_SCL);

// Auxiliary SPI Input Pin Definitions
#define SPI1_MISO PA6
#define SPI1_MOSI PA7
#define SPI1_SCK PA5
#define SPI1_CS PB0
SPIClass SPI1_AUX(SPI1_MOSI, SPI1_MISO, SPI1_SCK, PNUM_NOT_DEFINED);
// SPIClass SPI1_AUX(SPI1_MOSI, SPI1_MISO, SPI1_SCK, SPI1_CS);

// STDC14 STLINK Header UART Pin Definitions
#define USART3_RX PB11
#define USART3_TX PB10
HardwareSerial Serial3_Debug (USART3_RX, USART3_TX);


// PROGRAM CONFIGURATION PARAMETERS
#define MOTOR_POLE_PAIRS 14
#define DRV_CSA_GAIN DRV8316_CSAGain::Gain_0V15 //TODO: Fix enum labels (they are not correct)


// CONSTANTS
#define VSENSE_DIVIDER_COEFFICIENT 13.39669421487603
#define CSENSE_DIVIDER_COEFFICIENT 3.584229390681004
#define VREFBUF_VOLTAGE 2.9
#define DRV_CSA_MILLIVOLTS_PER_AMP (150 * pow(2, (int)DRV_CSA_GAIN) * (VREFBUF_VOLTAGE / 3.3))


BLDCMotor motor (MOTOR_POLE_PAIRS);
DRV8316Driver3PWM driver (TIM1_CH1_PWM_A, TIM1_CH2_PWM_B, TIM1_CH3_PWM_C, SPI2_DRV_CS, false, PB1_DRV_PWM_EN, DRV_NFAULT);
// MagneticSensorMT6701SSI sensor(SPI3_ENC_CS);
CustomMT6701SSI sensor (SPI3_ENC_MISO, SPI3_ENC_SCK, SPI3_ENC_CS);
LowsideCurrentSense current_sense (DRV_CSA_MILLIVOLTS_PER_AMP, ADC2_IN5_SOA, ADC2_IN12_SOB, ADC2_IN17_SOC);
Commander commander (SerialUSB);


float getBusVoltage() {
  return (analogRead(ADC1_IN1_VSENSE) / 4096.0) * VREFBUF_VOLTAGE * VSENSE_DIVIDER_COEFFICIENT;
}

float getBusCurrent() {
  return (analogRead(ADC1_IN11_CSENSE) / 4096.0) * VREFBUF_VOLTAGE * CSENSE_DIVIDER_COEFFICIENT;
}

void handleCommand(char* cmd) {
  commander.motor(&motor,cmd);
}


uint32_t count = 0;
uint32_t timestamp = 0;

void setup() {
  init_dfu_trigger_handling(); // Call this first in setup()

  // Turn on user LED to show code is running
  pinMode(TIM8_CH1_USR_LED, OUTPUT_OPEN_DRAIN);
  digitalWrite(TIM8_CH1_USR_LED, LOW);

  // Initialize Serial Comms
  SerialUSB.begin(115200);

  RS485.begin(115200);
  CAN.begin(1000000);
  
  // uint8_t data = 1;
  while (1) {
    RS485.println("Hi");
    uint8_t data[] = {0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x56, 0x78};
    CanMsg msg = CanMsg(0x80AB, 16, data);
    CAN.write(msg);
    // data++;
    delay(500);
  }
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&SerialUSB);
  motor.useMonitoring(SerialUSB); // Select serial interface for motor to use
  motor.monitor_downsample = 0; // Begin with real-time monitoring disabled
  commander.add('M', handleCommand, "motor command"); // Add command handler

  // Initialize Bus Voltage and Current Sensing
  LL_VREFBUF_Enable(); // Enable internal voltage reference, critical for ADC operation
  LL_VREFBUF_DisableHIZ(); // Expose internal reference onto VREF+ pin
  LL_VREFBUF_SetVoltageScaling(LL_VREFBUF_VOLTAGE_SCALE2); // Set reference to 2.9V
  analogReadResolution(12); // Set ADC resolution to max (12 bits)
  pinMode(ADC1_IN1_VSENSE, INPUT_ANALOG);
  pinMode(ADC1_IN11_CSENSE, INPUT_ANALOG);
  delay(100); // Allow ADC to Stabilize

  // Check if powered only through USB
  while (getBusVoltage() < 4.0) {
    SerialUSB.println("INFO: Only detected USB power, waiting for VAUX before continuing...");
    delay(500);
  }
  
  SerialUSB.println("INFO: VAUX detected, initializing...");

  // Initialize Motor Control System
  // sensor.init(&SPI3_ENC);
  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = getBusVoltage();
  driver.enable_active_high = true;
  driver.init(&SPI2_DRV);
  driver.setBuckVoltage(DRV8316_BuckVoltage::VB_5V); // Increase buck voltage to 5V for CAN transciever
  driver.setBuckCurrentLimit(DRV8316_BuckCurrentLimit::Limit_600mA);

  driver.setPWMMode(DRV8316_PWMMode::PWM3_Mode);
  driver.setSDOMode(DRV8316_SDOMode::SDOMode_PushPull);
  driver.setOvertemperatureReporting(true);
  driver.setSPIFaultReporting(true);
  driver.setOvervoltageProtection(true);
  driver.setSlew(DRV8316_Slew::Slew_25Vus);
  driver.setOvervoltageLevel(DRV8316_OVP::OVP_SEL_32V);
  driver.setPWM100Frequency(DRV8316_PWM100DUTY::FREQ_20KHz); // Not sure about this one
  driver.setOCPMode(DRV8316_OCPMode::Latched_Fault);
  driver.setOCPLevel(DRV8316_OCPLevel::Curr_16A);
  driver.setOCPRetryTime(DRV8316_OCPRetry::Retry500ms);
  driver.setOCPDeglitchTime(DRV8316_OCPDeglitch::Deglitch_1us6);
  driver.setCurrentSenseGain(DRV_CSA_GAIN);
  driver.setRecirculationMode(DRV8316_Recirculation::CoastMode);

  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  current_sense.init();

  motor.voltage_sensor_align = 1.5;
  motor.voltage_limit = 10;
  motor.current_limit = 3;
  motor.velocity_limit = 300;
  
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  // motor.controller = MotionControlType::angle_openloop;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.limit = 3;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 50;
  motor.P_angle.limit = 200;

  motor.init();
  motor.initFOC();

  SerialUSB.println(F("Motor ready."));
  timestamp = millis();
}

void loop() {
  if (millis() - timestamp > 1000) {
    SerialUSB.println(count);
    count = 0;
    timestamp = millis();
  }
  count++;

  driver.voltage_power_supply = getBusVoltage();
  motor.loopFOC(); // Calculate FOC phase voltages
  motor.move(); // Calculate motor control loops 
  motor.monitor(); // Send monitoring telemetry if enabled
  commander.run(); // Process incoming commands
}