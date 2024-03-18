#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "cppINA260.h"

using namespace std::chrono;

INA260::INA260() {
}

INA260::~INA260() {
  i2c_close(gpio_commander, i2c_handle);
}

bool INA260::init() {
  gpio_commander = pigpio_start("192.168.1.5", NULL);								// start PiGPIO object on local computer
  i2c_handle = i2c_open(gpio_commander, 1, INA260_I2CADDR_DEFAULT, 0);
  if (i2c_handle < 0) {
    std::cout << "i2c bus could not be opened... Exiting." << std::endl;
    exit(0);
  }

  int16_t mfg_register, device_id;
  mfg_register = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_MFG_UID));
  device_id = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_DIE_UID) >> 4);

  // Make sure we're talking to the right chip
  if ((mfg_register != 0x5449) || (device_id != 0x207)) {
    return false;
  }
  std::cout << "Chip is correct! \nResetting..." << std::endl;

  reset();

  // Delay 2ms to give time for first measurement to finish (made it 4ms..)
  std::this_thread::sleep_for(std::chrono::milliseconds(4)); 
  return true;

}

uint16_t INA260::shuffle(uint16_t num) {
  uint16_t a, b, c;
  a = num << 8;
  a = a & 0xFF00;
  b = num >> 8;
  b = b & 0xFF;
  c = a|b;    
  return c;
}

void INA260::reset(void) {
  int16_t error;
  if (error = i2c_write_byte_data(gpio_commander, i2c_handle, INA260_REG_CONFIG, 0xE1))
    std::cout << "Write failed: " << pigpio_error(error) << std::endl;
  else {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "Reset success!" << std::endl;
  }
}

float INA260::readCurrent(void) {
  int16_t current = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CURRENT));
  return current * 1.25;
}

float INA260::readBusVoltage(void) {
  int16_t busVoltage = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_BUSVOLTAGE));
  return busVoltage * 1.25;
}

float INA260::readPower(void) {
  int16_t power = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_POWER));
  return power * 10;
}

INA260_MeasurementMode INA260::getMode(void) {
  int16_t mode = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG));
  mode = mode & 0x7;
  return (INA260_MeasurementMode)mode;
}

void INA260::setMode(INA260_MeasurementMode new_mode) {
  int16_t mode = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG));
  mode = mode >> 3;
  mode = mode << 3;
  mode = mode | new_mode;
  i2c_write_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG, shuffle(mode));;
}

INA260_AveragingCount INA260::getAveragingCount(void) {
  int8_t averaging_count = i2c_read_byte_data(gpio_commander, i2c_handle, INA260_REG_CONFIG);
  averaging_count = averaging_count << 4;
  averaging_count = averaging_count >> 5;
  averaging_count = averaging_count & 0x07;
  return (INA260_AveragingCount)averaging_count;
}

void INA260::setAveragingCount(INA260_AveragingCount count) {
  int8_t averaging_count, current = i2c_read_byte_data(gpio_commander, i2c_handle, INA260_REG_CONFIG), k;
  if (current % 2) {
    averaging_count = 97 + (count * 2);
  } else {
    averaging_count = 96 + (count * 2);
  }
  i2c_write_byte_data(gpio_commander, i2c_handle, INA260_REG_CONFIG, averaging_count);
}

INA260_ConversionTime INA260::getCurrentConversionTime(void) {
  int16_t current_conversion_time = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG));
  current_conversion_time = current_conversion_time << 10;
  current_conversion_time = current_conversion_time >> 13;
  current_conversion_time = current_conversion_time & 0x07;
  return (INA260_ConversionTime)current_conversion_time;
}

void INA260::setCurrentConversionTime(INA260_ConversionTime time) {
  uint16_t current, current_conversion_time = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG));
  int16_t error;
  current = getCurrentConversionTime();
  current_conversion_time = current_conversion_time ^ (current * 8);
  current_conversion_time = current_conversion_time + (time * 8);
  if (error = i2c_write_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG, shuffle(current_conversion_time))) {
    std::cout << "Write failed: " << pigpio_error(error) << std::endl;
  }
}

INA260_ConversionTime INA260::getVoltageConversionTime(void) {
  uint16_t voltage_conversion_time = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG));
  voltage_conversion_time = voltage_conversion_time << 7;
  voltage_conversion_time = voltage_conversion_time >> 13;
  voltage_conversion_time = voltage_conversion_time & 0x07;
  return (INA260_ConversionTime)voltage_conversion_time;
}

void INA260::setVoltageConversionTime(INA260_ConversionTime time) {
  int16_t error;
  uint16_t current, voltage_conversion_time = shuffle(i2c_read_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG));
  current = getVoltageConversionTime();
  voltage_conversion_time = voltage_conversion_time ^ (current * 64);
  voltage_conversion_time = voltage_conversion_time + (time * 64);
  if (error = i2c_write_word_data(gpio_commander, i2c_handle, INA260_REG_CONFIG, shuffle(voltage_conversion_time))) {
    std::cout << "Write failed: " << pigpio_error(error) << std::endl;
  }
}

int main(int argc, char *argv[])
{
  INA260 ina260;
  ina260.init();
  ina260.setMode(INA260_MODE_CONTINUOUS);
  std::cout << "Mode: " << ina260.getMode() << std::endl;
  ina260.setAveragingCount(INA260_COUNT_64);
  std::cout << "Averaging Mode: " << ina260.getAveragingCount() << std::endl;
  ina260.setVoltageConversionTime(INA260_TIME_2_116_ms);
  std::cout << "Voltage Conversion Mode: " << ina260.getVoltageConversionTime() << std::endl;
  ina260.setCurrentConversionTime(INA260_TIME_2_116_ms);
  std::cout << "Current Conversion Mode: " << ina260.getCurrentConversionTime() << std::endl;
  float current_ma;
  float voltage_mv;
  float power_mw;
  while(1) {
    current_ma = ina260.readCurrent();
    voltage_mv = ina260.readBusVoltage();
    power_mw = ina260.readPower();
    std::cout << "Current (mA): " << current_ma << std::endl;
    std::cout << "Voltage (mV): " << voltage_mv << std::endl;
    std::cout << "Power (mW): " << power_mw << std::endl;
    std::cout << std::endl;
  }
}
