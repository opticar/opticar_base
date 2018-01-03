#ifndef OPTICAR_ECU_GPS_H_
#define OPTICAR_ECU_GPS_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

bool GPS_Init();
void GPS_Poll();


#ifdef __cplusplus
}
#endif

#endif /* OPTICAR_ECU_GPS_H_ */
