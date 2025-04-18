#ifndef _POLOLU_QTR_H_
#define _POLOLU_QTR_H_

#include "stm_core_addon.h"

typedef enum
{
  QTR_Unknown,
  QTR_Device_Zumo,
} eQTRDevices;

void Pololu_QTR_EnableMeassure(bool en);

bool Pololu_QTR_InitSensors(eQTRDevices deviceType);

bool Pololu_QTR_StepCalibrate(void);
bool Pololu_QTR_ClearCalibrate(void);

bool Pololu_QTR_FillSensors(uint16_t *pSens);
bool Pololu_QTR_FillSensRaw(uint16_t *pSens);

#endif /* _POLOLU_QTR_H_ */
