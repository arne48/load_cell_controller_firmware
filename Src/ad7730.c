#include "ad7730.h"

void ad7730_setup_device(uint8_t device) {

}

void ad7730_setup_all() {

}

void ad7730_softreset(uint8_t device) {
  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET);

  uint8_t command[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  HAL_SPI_Transmit(&hspi1, command, 5, 100);

  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET);
}

void ad7730_system_zero_scale_calibration(uint8_t device) {

}

void ad7730_internal_zero_scale_calibration(uint8_t device) {

}

void ad7730_read_input(uint8_t device, uint8_t data[]) {

  uint8_t conversion_command[2] = {0x31,0xA0};

  ad7730_write_register(0, REG_MODE_REGISTER, conversion_command);

  ad7730_read_register(0, REG_DATA_REGISTER, data);

}

void ad7730_read_all_inputs(uint8_t data[]) {

}

void ad7730_set_read_mode(uint8_t device, AD7730_CommunicationTypeDef com_type, AD7730_RegisterTypeDef reg_type) {
  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET);

  uint8_t command[1] = { com_type | reg_type };
  HAL_SPI_Transmit(&hspi1, command, 1, 100);

  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET);
}

void ad7730_read_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[]) {

  ad7730_set_read_mode(device, OP_READ, reg);

  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET);

  HAL_SPI_Receive(&hspi1, data, AD7730_REGISTER_SIZE[reg], 100);

  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET);
}

void ad7730_write_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[]) {

  ad7730_set_read_mode(device, OP_WRITE, reg);

  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, data, AD7730_REGISTER_SIZE[reg], 100);

  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET);
}
