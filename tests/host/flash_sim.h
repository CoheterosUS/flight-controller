#ifndef FLASH_SIM_H
#define FLASH_SIM_H

#include <stddef.h>
#include <stdint.h>

void flash_reset(void);
uint8_t *flash_bytes(void);
void flash_set_program_limit(int Limit);
size_t flash_program_count(void);
void flash_corrupt(uint32_t Address, uint8_t Value);

#endif
