#include "ad7730.h"

void ad7730_setup_device(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_set_filter(uint8_t device, struct Transducer_SS_Info device_infos[]) {
  //uint8_t conversion_command[3] = {0x08,0x43, 0x00};
  uint8_t conversion_command[3] = {0x20,0x01, 0x00};
  ad7730_write_register(device, REG_FILTER_REGISTER, conversion_command, device_infos);

}

void ad7730_setup_all(struct Transducer_SS_Info device_infos[]) {

}

void ad7730_softreset(uint8_t device, struct Transducer_SS_Info device_infos[]) {
  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_RESET);

  uint8_t command[4] = { 0xFF, 0xFF, 0xFF, 0xFF};
  HAL_SPI_Transmit(&hspi1, command, 4, 10);

  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_SET);
}

void ad7730_system_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_internal_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_read_input(uint8_t device, uint8_t data[], struct Transducer_SS_Info device_infos[], struct Transducer_RDY_Info ready_infos[], AD7730_ChannelIndex channel_index) {

  uint8_t conversion_command[2] = {0x51,0xA0 | channel_index};

  ad7730_write_register(device, REG_MODE_REGISTER, conversion_command, device_infos);

  while(HAL_GPIO_ReadPin(ready_infos[device].rdy_port, ready_infos[device].rdy_pin) != GPIO_PIN_SET){}

  ad7730_read_register(device, REG_DATA_REGISTER, data, device_infos);

  while(HAL_GPIO_ReadPin(ready_infos[device].rdy_port, ready_infos[device].rdy_pin) != GPIO_PIN_SET){}

}
uint8_t t = 0;
void ad7730_read_all_inputs(uint8_t data[], struct Transducer_SS_Info device_infos[]) {

 uint8_t conversion_command_a1[2] = {0x51,0xA4 | CHANNEL_A1};
 for(uint8_t i =0; i < 8; i++){
   ad7730_write_register(i, REG_MODE_REGISTER, conversion_command_a1, device_infos);
 }

 for(uint8_t i =0; i < 8; i++){
   ad7730_read_register(i, REG_DATA_REGISTER, &data[i*6], device_infos);
 }

 uint8_t conversion_command_a2[2] = {0x51,0xA0 | CHANNEL_A2};
 for(uint8_t i =0; i < 8; i++){
   ad7730_write_register(i, REG_MODE_REGISTER, conversion_command_a2, device_infos);
 }

 for(uint8_t i =0; i < 8; i++){
   ad7730_read_register(i, REG_DATA_REGISTER, &data[(i*6)+3], device_infos);
 }

 data[7] = t;
 t++;

}

void ad7730_set_communication_mode(uint8_t device, AD7730_CommunicationTypeDef com_type, AD7730_RegisterTypeDef reg_type, struct Transducer_SS_Info device_infos[]) {

  uint8_t command[1] = { com_type | reg_type };
  HAL_SPI_Transmit(&hspi1, command, 1, 10);

}

void ad7730_read_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]) {

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

  ad7730_set_communication_mode(device, OP_READ, reg, device_infos);

  HAL_SPI_Receive(&hspi1, data, AD7730_REGISTER_SIZE[reg], 10);

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

}

void ad7730_write_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]) {

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

  ad7730_set_communication_mode(device, OP_WRITE, reg, device_infos);

  HAL_SPI_Transmit(&hspi1, data, AD7730_REGISTER_SIZE[reg], 10);

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

  //FIXME
//    HAL_Delay(14);
  //for(uint32_t wait = 0; wait < 2500; wait++){}

}
