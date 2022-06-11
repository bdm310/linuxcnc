/********************************************************************
* Description:  hal_pi_gpio_dma.c
*               DMA based step/dir and input driver for the 
*               Raspberry Pi GPIO pins
*
* Author: Brian Moyer
* License: GPL Version 2
s********************************************************************/


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_bitops.h"
#include "rtapi_app.h"		/* RTAPI realtime module decls */
                                /* this also includes config.h */
#include "hal.h"		/* HAL public API decls */
#include "bcm2835.h"
#include "hal_pi.h"
#include "mailbox.h"
#include "dma_paced.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/fsuid.h>

MODULE_AUTHOR("Brian Moyer");
MODULE_DESCRIPTION("DMA driver for Raspberry Pi GPIO pins");
MODULE_LICENSE("GPL");

static int comp_id;		/* component ID */

int rtapi_app_main(void)
{
    comp_id = hal_init("hal_pi_gpio_dma");
    if (comp_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "HAL_PI_GPIO_DMA: ERROR: hal_init() failed\n");
	return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
	"HAL_PI_GPIO_DMA: installed driver\n");
    hal_ready(comp_id);
    
    uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
    dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);

    pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);

    uint8_t *cm_base_ptr = map_peripheral(CM_BASE, CM_LEN);
    clk_reg = (CLKCtrlReg *)(cm_base_ptr + CM_PWM);

    dma_alloc_buffers();
    usleep(100);

    dma_init_cbs();
    usleep(100);

    init_hw_clk();
    usleep(100);

    init_pwm();
    usleep(100);

    dma_start();
    usleep(100);

    uint32_t ticks[TICK_CNT];
    memcpy(ticks, ith_tick_virt_addr(0), TICK_CNT * sizeof(uint32_t));

    for (size_t i = 0; i < TICK_CNT; i++)
    {
        printf("DMA %d: %u\n", i, ticks[i]);
    }

    
    
    return 0;
}

void rtapi_app_exit(void)
{
    dma_end();
    
    hal_exit(comp_id);
}

int export_funcs(void)
{
    
}

int export_pins(void)
{
    
}
