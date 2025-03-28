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
#include "CustomHardwareSerial.h"

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


// Implementation inspired by https://github.com/bonezegei/Bonezegei_RS485
class Custom_RS485 {
  public:
    Custom_RS485(CustomHardwareSerial &serial, int dir) {
      for (size_t i = 0; i < MAX_NUM_INSTANCES; i++) {
        if (instances[i]) continue;
        instances[i] = this;
        break;
      }
      serial.attach_custom_tx_callback(_handle_TxCplt_Callback);

      hardserial = (CustomHardwareSerial *)&serial;
      _dir = dir;
      pinMode(_dir, OUTPUT);
      digitalWrite(_dir, LOW);
    }
  
    void begin(int baud) {
      hardserial->begin(baud);
    }
    
    void write(char ch) {
      digitalWrite(_dir, HIGH);
      hardserial->write(ch);
    }
    
    void print(const char *ch) {
      digitalWrite(_dir, HIGH);
      hardserial->print(ch);
    }
    
    void println(const char *ch) {
      digitalWrite(_dir, HIGH);
      hardserial->println(ch);
    }
    
    char available() {
      return hardserial->available();
    }
    
    int read() {
      return hardserial->read();
    }

    static void _handle_TxCplt_Callback(serial_t *obj_in) {
      for (size_t i = 0; i < MAX_NUM_INSTANCES; i++) {
        if (instances[i]) {
          // Copied from uart.c file's get_serial_obj() function
          struct serial_s *obj_s;
          serial_t *obj;
          UART_HandleTypeDef * huart = instances[i]->hardserial->getHandle();

          obj_s = (struct serial_s *)((char *)huart - offsetof(struct serial_s, handle));
          obj = (serial_t *)((char *)obj_s - offsetof(serial_t, uart));

          // Instance match found, set transceiver to receive mode
          if (obj_in == obj) {
            digitalWrite(instances[i]->_dir, LOW);
          }
        } else break;
      }
    }
    
    int _dir;
    int _baud;
    CustomHardwareSerial *hardserial;
    static const size_t MAX_NUM_INSTANCES = 10;
    inline static Custom_RS485 * instances[MAX_NUM_INSTANCES];
};