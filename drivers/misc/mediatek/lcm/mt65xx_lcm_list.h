#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>

extern LCM_DRIVER nt35532_fhd_boe_vdo_lcm_drv;
extern LCM_DRIVER nt35596_fhd_auo_phantom_lcm_drv;
extern LCM_DRIVER nt35596_fhd_tianma_phantom_lcm_drv;

#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif

#endif
