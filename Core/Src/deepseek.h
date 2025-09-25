#ifndef SD_CARD_PRINT_H
#define SD_CARD_PRINT_H

#include "stm32h7xx_hal.h"
#include <stdio.h>

void print_sd_card_info(HAL_SD_CardCSDTypeDef *csd, HAL_SD_CardCIDTypeDef *cid);

// Manufacturer ID to name mapping
const char* get_manufacturer_name(uint8_t mid);
const char* get_csd_structure_name(uint8_t csd_structure);
const char* get_speed_class_name(uint8_t tran_speed);
void calculate_capacity_string(uint32_t device_size, uint8_t device_size_mul,
                              uint8_t read_bl_len, uint8_t csd_structure,
                              char* buffer, size_t buffer_size);

#endif
