#ifndef _APA102_RGB_H
#define _APA102_RGB_H

#include "stm_core.h"

bool APA102_init(int count);

void APA102_SetIntesity(uint8_t val);
bool APA102_LED(int id, int r, int g, int b);
int APA102_GetCount(void);

bool APA102_RainbowOffset(int offset);
int APA102_RainbowLength(void);
void APA102_RainbowNext(void);
bool APA102_GetRainbow(int pos, uint8_t *pr, uint8_t *pg, uint8_t *pb);

bool APA102_Refresh(void);

#endif // _APA102_RGB_H
