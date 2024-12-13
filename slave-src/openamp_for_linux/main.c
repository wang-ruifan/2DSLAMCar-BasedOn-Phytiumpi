/*
 * Copyright : (C) 2024 Phytium Information Technology, Inc.
 * All Rights Reserved.
 * 
 * This program is OPEN SOURCE software: you can redistribute it and/or modify it
 * under the terms of the Phytium Public License as published by the Phytium Technology Co.,Ltd,
 * either version 1.0 of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the Phytium Public License for more details.
 * 
 * 
 * FilePath: main.c
 * Created Date: 2022-02-24 16:56:46
 * Last Modified: 2024-03-04 19:50:56
 * Description:  This file is for This file is for AMP example that running rpmsg_echo_task and open scheduler
 * 
 * Modify History: 
 *  Ver   Who        Date         Changes
 * ----- ------     --------    --------------------------------------
 *  1.0 huanghe    2022/03/25  first commit
 *  1.1 huanghe    2023/03/09  Adapt OpenAMP routines based on e2000D/Q
 *  1.2 liusm      2023/11/20  Update example
 */
#include "ftypes.h"
#include "fpsci.h"
#include "fsleep.h"
#include "fprintk.h"
#include "fdebug.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fgpio_os.h"
#include "pwm_example.h"
#include "car.h"
//#include "gpio_io_irq.h"
#define DIRECTION_PIN_STR "1-a-12"
#define PWM_PIN_STR "3-a-1"
#define ENCORDERA_PIN_STR "3-a-2"
#define ENCORDERA_B_PIN_STR "1-a-11"
#define ENCORDERB_PIN_STR "4-a-11"
#define ENCORDERB_B_PIN_STR "4-a-12"

extern int rpmsg_echo_task(void);

volatile float motor_target_speed = 0;
volatile int pwm_steer_pulse = 150;
volatile float MotorASpeed = 0.0;
volatile int front_sensor_flag = 0;

int main(void)
{
    BaseType_t ret;
    f_printk("freertos %s ,%s \r\n",__DATE__, __TIME__);
    f_printk("Starting OpenAMP task...\r\n");
    rpmsg_echo_task();
    f_printk("Starting OpenAMP task success\r\n");
    /*u32 out_pin = 0x30001, in_pin = 0x30002;
    ret = FFreeRTOSRunGpioIOIrq(out_pin,in_pin);
    if (ret != pdPASS)
    {
        // 错误处理
        goto FAIL_EXIT;
    }*/
    f_printk("Starting Car task...\r\n");
    ret = FFreeRTOSRunCar();
    if (ret != pdPASS)
    {
        // 错误处理
        goto FAIL_EXIT;
    }
    f_printk("Starting Car task success\r\n");
    
    vTaskStartScheduler(); /* 启动任务，开启调度 */
    while (1); /* 正常不会执行到这里 */

FAIL_EXIT:
    printf("failed 0x%x \r\n", ret);  
    return 0;
}

