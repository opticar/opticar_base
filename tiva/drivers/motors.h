#ifndef OPTICAR_DEMO_DRIVERS_MOTORS_H_
#define OPTICAR_DEMO_DRIVERS_MOTORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
} MOTOR_Wheels;

bool MOTOR_Init();
bool MOTOR_EmergencyStop();
bool MOTOR_ReleaseEmergencyStop();
void MOTOR_SetSpeed(MOTOR_Wheels wheel, int32_t percentage);

#ifdef __cplusplus
}
#endif

#endif /* OPTICAR_DEMO_DRIVERS_MOTORS_H_ */
