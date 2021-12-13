#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void init(void);
void read_fifo(void);
int read_sequential(void);