#ifndef CAN_TOOLS_H
#define CAN_TOOLS_H

#ifdef __cplusplus
#include "Arm.hpp"
extern Arm arm;
extern "C" {
#endif

void init(void);
void armHandler(void);

#ifdef __cplusplus
}
#endif

#endif // CAN_TOOLS_H
