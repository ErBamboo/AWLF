#include "component/driver/modules/motor/motor.h"
#include <string.h>
AwlfRet_e motor_register(const char* name, size_t nameLen, EcdMotor_t motor, EcdMotorInterface_t interface)
{
    while (name == NULL || motor == NULL || interface == NULL) {}; // TODO: assert
    memcpy(motor->name, name, nameLen);
    motor->interface = interface;

    return AWLF_OK;
}