#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include <stdint.h>

#define RX_COMPLETE   0
#define RX_INCOMPLETE 1
#define RX_OVERFLOW   2

#define CH_NL         0x0A
#define CH_CR         0x0D
#define CH_BS         0x08

int8_t ProcessCmdLine(char *cmdBuf, uint16_t bufSize);
uint16_t TokenToNum(char const *token);

void CIClear(void);
uint16_t GetCommand(void);
void DispatchCommand(void);

#endif
