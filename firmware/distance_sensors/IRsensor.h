#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>

void initIR(void);
void get_IR_readings(uint8_t right, uint8_t left, uint8_t *rightReading, uint8_t *leftReading);

#endif
