#ifndef STM_USART_H_
#define STM_USART_H_

#include "stm_core.h"
#include <stdio.h>

bool Usart2Init(int baudSpeed);
bool IsUsart2Recv(void);
/*
int Usart2Send(char c);
int Usart2Recv(void);
*/

#endif /* STM_USART_H_ */
