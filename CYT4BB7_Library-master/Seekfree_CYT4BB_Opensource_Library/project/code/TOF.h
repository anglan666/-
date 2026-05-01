#ifndef TOF_H_
#define TOF_H_

#include "zf_common_headfile.h"

void tof_init(void);
void tof_update(void);
extern uint16 g_tof_distance_mm;

#endif /* TOF_H_ */
