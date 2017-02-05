#include "ad7730.h"

void ad7730_setup_device(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_setup_all(struct Transducer_SS_Info device_infos[]) {

}

void ad7730_softreset(uint8_t device, struct Transducer_SS_Info device_infos[]) {
  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_RESET);

  uint8_t command[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  HAL_SPI_Transmit(&hspi1, command, 5, 100);

  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_SET);
}

void ad7730_system_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_internal_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_read_input(uint8_t device, uint8_t data[], struct Transducer_SS_Info device_infos[], AD7730_ChannelIndex channel_index) {

  uint8_t conversion_command[2] = {0x51,0xA0 | channel_index};

  ad7730_write_register(device, REG_MODE_REGISTER, conversion_command, device_infos);

  ad7730_read_register(device, REG_DATA_REGISTER, data, device_infos);

}

void ad7730_read_all_inputs(uint8_t data[], struct Transducer_SS_Info device_infos[]) {
  for(uint8_t transd_idx = 0; transd_idx < TRANSDUCER_NUMBER; transd_idx++){
    uint8_t buffer_offset = transd_idx * 2;


  }
}

void ad7730_set_communication_mode(uint8_t device, AD7730_CommunicationTypeDef com_type, AD7730_RegisterTypeDef reg_type, struct Transducer_SS_Info device_infos[]) {
  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_RESET);

  uint8_t command[1] = { com_type | reg_type };
  HAL_SPI_Transmit(&hspi1, command, 1, 100);

  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_SET);
}

void ad7730_read_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]) {

  ad7730_set_communication_mode(device, OP_READ, reg, device_infos);

  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Receive(&hspi1, data, AD7730_REGISTER_SIZE[reg], 100);
  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_SET);
}

void ad7730_write_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]) {

  ad7730_set_communication_mode(device, OP_WRITE, reg, device_infos);

  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, data, AD7730_REGISTER_SIZE[reg], 100);
  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_SET);
}
