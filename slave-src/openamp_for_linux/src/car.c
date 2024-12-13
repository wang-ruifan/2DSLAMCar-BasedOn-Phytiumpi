/*
*
* File Name: car.c
* Path: src/car.c
*
* Editor: Wang Ruifan
*
* Description: Control the car movement and direction according to the target speed and steer angle
* Based File: Phytium-FreeRTOS-SDK/example/gpio/src/gpio_io_irq.c , Phytium-FreeRTOS-SDK/example/pwm/src/pwm_example.c
*
* LastEditTime: 2024-4-24 14:58:53
*
*/
/***************************** Include Files *********************************/
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "fdebug.h"
#include "fsleep.h"
#include "fio_mux.h"

#include "fgpio_os.h"
#include "fpwm_os.h"
#include "fcpu_info.h"
#include "fio_mux.h"

#include "fdrivers_port.h"
#include "fparameters.h"

#include "fgpio_hw.h"
#include "fgpio.h"


#include "car.h"

/************************** Constant Definitions *****************************/
#define PIN_IRQ_OCCURED     (0x1 << 0)
//Car work task number
#define CAR_WORK_TASK_NUM  2U
//Motor direction control GPIO：GPIO1_5
#define OUT_PIN_INDEX_DIRECTION FFREERTOS_GPIO_PIN_INDEX(1, 0, 5)
//Motor encoder A input GPIO：GPIO1_11
#define IN_PIN_INDEX_ENCORDERA FFREERTOS_GPIO_PIN_INDEX(1, 0, 11)
//Motor encorder A_B input GPIO：GPIO3_2
#define IN_PIN_INDEX_ENCORDERA_B FFREERTOS_GPIO_PIN_INDEX(3, 0, 2)
//Motor encoder B input GPIO：GPIO1_9
#define IN_PIN_INDEX_ENCORDERB FFREERTOS_GPIO_PIN_INDEX(1, 0, 9)
//Motor encorder B_B input GPIO：GPIO1_10
#define IN_PIN_INDEX_ENCORDERB_B FFREERTOS_GPIO_PIN_INDEX(1, 0, 10)
//Front sensor input GPIO：GPIO2_10
#define IN_PIN_INDEX_FRONTSENSOR FFREERTOS_GPIO_PIN_INDEX(2, 0, 10)
//Set PWM channel
#define PWM_MOTOR_CHANNEL_USE       FPWM_CHANNEL_0
#define PWM_STEER_CHANNEL_USE       FPWM_CHANNEL_1
//PWM init parameters
#define PWM_DIV             500     //占空比50%
#define PWM_STEER_PERIOD    2000    //舵机pwm周期20ms
#define PWM_MOTOR_PERIOD    500     //电机pwm周期5ms
#define PWM_STEER_PULSE     150     //舵机初始脉冲1.5ms
#define PWM_MOTOR_PULSE     0       //电机初始脉冲0ms
//Wheel perimeter
#define	Wheel_Perimeter     0.2042
//Encoder precision
#define Encorder_Precision  390.0
//Car control period
#define Control_Period      20
//Serial debug print,1 for print,0 for not print
#define Debug_Info_Print    0       //1为打印，0为不打印
/**************************** Type Definitions *******************************/

/************************** Variable Definitions *****************************/
//Motor direction control GPIO and GPIO config
static FFreeRTOSFGpio* out_gpio_direction = NULL;
static FFreeRTOSGpioConfig out_gpio_direction_cfg;
//Encoder A GPIO and GPIO config
static FFreeRTOSFGpio* in_gpio_encorderA = NULL;
static FFreeRTOSGpioConfig in_gpio_encorderA_cfg;
//Encoder A_B GPIO and GPIO config
static FFreeRTOSFGpio* in_gpio_encorderA_B = NULL;
static FFreeRTOSGpioConfig in_gpio_encorderA_B_cfg;
//Front sensor GPIO and GPIO config
static FFreeRTOSFGpio* in_gpio_frontsensor = NULL;
static FFreeRTOSGpioConfig in_gpio_frontsensor_cfg;
//Task locker
static xSemaphoreHandle init_locker = NULL;
//Pin index definition
static u32 out_pin_direction = OUT_PIN_INDEX_DIRECTION;
static u32 in_pin_encorderA = IN_PIN_INDEX_ENCORDERA;
static u32 in_pin_encorderA_B = IN_PIN_INDEX_ENCORDERA_B;
static u32 in_pin_frontsensor = IN_PIN_INDEX_FRONTSENSOR;

//Motor direction pin config
static FFreeRTOSGpioPinConfig out_pin_direction_config =
{
	.pin_idx = OUT_PIN_INDEX_DIRECTION,
	.mode = FGPIO_DIR_OUTPUT,
	.en_irq = FALSE
};
//Encoder A pin config
static FFreeRTOSGpioPinConfig in_pin_encorderA_config =
{
	.pin_idx = IN_PIN_INDEX_ENCORDERA,
	.mode = FGPIO_DIR_INPUT,
	.en_irq = TRUE,
	.irq_type = FGPIO_IRQ_TYPE_EDGE_RISING,
	.irq_handler = NULL,
	.irq_args = NULL
};
//Encoder A_B pin config
static FFreeRTOSGpioPinConfig in_pin_encorderA_B_config =
{
	.pin_idx = IN_PIN_INDEX_ENCORDERA_B,
	.mode = FGPIO_DIR_INPUT,
	.en_irq = FALSE,
};
//Front sensor pin config
static FFreeRTOSGpioPinConfig in_pin_frontsensor_config =
{
	.pin_idx = IN_PIN_INDEX_FRONTSENSOR,
	.mode = FGPIO_DIR_INPUT,
	.en_irq = FALSE,
};

//Motor PWM
static FFreeRTOSPwm* os_pwm_ctrl_motor_pin;
//Steer PWM
static FFreeRTOSPwm* os_pwm_ctrl_steer_pin;

//Encoder A pin
FGpioPin* encorderA_B_pin;
//Front sensor pin
FGpioPin* frontsensor_pin;

//Task control
static EventGroupHandle_t event = NULL;
static TaskHandle_t control_task = NULL;
static boolean is_running = FALSE;
//Target motor speed, positive for forward, negative for backward
extern volatile float motor_target_speed;
//Target steer angle
extern volatile int pwm_steer_pulse;
//Encoder A count
int MotorACount = 0;
//Current motor speed
extern volatile float MotorASpeed;
//PI controller parameters
float Kp = 600.0;
float Ki = 40.0;
//PI controller variables
float bias = 0.0, last_bias = 0.0, bias_change = 0.0;
int pulse = 0;
//front sensor flag, 1 for front obstacle detected, 0 for no obstacle
extern volatile int front_sensor_flag;

/***************** Macros (Inline Functions) Definitions *********************/
#define FGPIO_DEBUG_TAG "MOTOR"
#define FGPIO_ERROR(format, ...) FT_DEBUG_PRINT_E(FGPIO_DEBUG_TAG, format, ##__VA_ARGS__)
#define FGPIO_WARN(format, ...)  FT_DEBUG_PRINT_W(FGPIO_DEBUG_TAG, format, ##__VA_ARGS__)
#define FGPIO_INFO(format, ...)  FT_DEBUG_PRINT_I(FGPIO_DEBUG_TAG, format, ##__VA_ARGS__)
#define FGPIO_DEBUG(format, ...) FT_DEBUG_PRINT_D(FGPIO_DEBUG_TAG, format, ##__VA_ARGS__)

/************************** Function Prototypes ******************************/

/*****************************************************************************/

/*-----------------------------------------------------------------------------*
 *  Function Name:          MotorEncorderAIrq
 *  Function Describtion:   Encorder A IRQ Callback Function
 *  Function Use:           Calculate Encorder Pulse To Get Current Motor Speed
 *  Editor:                 Wang Ruifan
 *  LastEditTime:           2024-04-21 21:22:51
 *-----------------------------------------------------------------------------*/
static void MotorEncorderAIrq(s32 vector, void* param)
{
	if (FGpioGetInputValue(encorderA_B_pin) == FGPIO_PIN_LOW) {
		MotorACount++;
	}
	else {
		MotorACount--;
	}
	FGpio* const instance = (FGpio* const)param;
	uintptr base_addr = instance->config.base_addr;
	u32 status = FGpioReadReg32(base_addr, FGPIO_INTSTATUS_OFFSET);
	u32 raw_status = FGpioReadReg32(base_addr, FGPIO_RAW_INTSTATUS_OFFSET);
	FGpioWriteReg32(base_addr, FGPIO_PORTA_EOI_OFFSET, status);
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          Pulse_limit_int
 *  Function Describtion:   Limit pulse value
 *  Function Use:           Limit pulse value between low and high
 *	Function Parameter:     int insert, int low, int high
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-14 17:03:42
 *-----------------------------------------------------------------------------*/
float Pulse_limit_int(int insert, int low, int high)
{
	if (insert <= low)
		return low;
	else if (insert >= high)
		return high;
	else
		return insert;
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          Motor_PI_Control
 *  Function Describtion:   Use PI controller to control motor speed
 *  Function Use:           Based on current speed and target speed to calculate pulse
 *	Function Parameter:     float current_speed, float target_speed
 *  Editor:                 Wang Ruifan
 *  LastEditTime:           2024-04-17 11:23:37
 *-----------------------------------------------------------------------------*/
int Motor_PI_Control(float current_speed, float target_speed) {
	bias = target_speed - current_speed;
	bias_change = bias - last_bias;
	pulse += Kp * (bias - last_bias) + Ki * bias;
	pulse = Pulse_limit_int(pulse, 1 - PWM_MOTOR_PERIOD, PWM_MOTOR_PERIOD - 1);
	if (Debug_Info_Print) printf("Kp: %f,Ki %f,bias: %f,pulse: %d\r\n", Kp, Ki, bias, pulse);
	//printf("当前偏差为：%f，上次偏差为：%f\r\n",bias,last_bias);
	last_bias = bias;
	return pulse;
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarGPIOInit
 *  Function Describtion:   Init Car GPIO
 *  Function Use:           Init Car Necessary GPIO Pin Including:
 *							Motor Direction, EncorderA, EncorderA_B, FrontSensor
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 14:55:20
 *-----------------------------------------------------------------------------*/
void CarGPIOInit() {
	static const char* irq_type_str[] = { "failling-edge", "rising-edge", "low-level", "high-level" };
	//Gpio init
	out_gpio_direction = FFreeRTOSGpioInit(FFREERTOS_GPIO_PIN_CTRL_ID(out_pin_direction), &out_gpio_direction_cfg);
	in_gpio_encorderA = FFreeRTOSGpioInit(FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_encorderA), &in_gpio_encorderA_cfg);
	in_gpio_encorderA_B = FFreeRTOSGpioInit(FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_encorderA_B), &in_gpio_encorderA_B_cfg);
	in_gpio_frontsensor = FFreeRTOSGpioInit(FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_frontsensor), &in_gpio_frontsensor_cfg);
	printf("GPIO init success!\r\n");

	/* init mio fuction */
	FIOMuxInit();
	/* init output/input pin */
	FIOPadSetGpioMux(FFREERTOS_GPIO_PIN_CTRL_ID(out_pin_direction), FFREERTOS_GPIO_PIN_ID(out_pin_direction)); /* set io pad */
	FIOPadSetGpioMux(FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_encorderA), FFREERTOS_GPIO_PIN_ID(in_pin_encorderA)); /* set io pad */
	FIOPadSetGpioMux(FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_encorderA_B), FFREERTOS_GPIO_PIN_ID(in_pin_encorderA_B)); /* set io pad */
	FIOPadSetGpioMux(FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_frontsensor), FFREERTOS_GPIO_PIN_ID(in_pin_frontsensor)); /* set io pad */
	printf("Pin init success!\r\n");

	//Motor Direction output config
	out_pin_direction_config.pin_idx = out_pin_direction;
	FFreeRTOSSetupPin(out_gpio_direction, &out_pin_direction_config, TRUE);

	//Motor EncorderA input config
	in_pin_encorderA_config.pin_idx = in_pin_encorderA;
	in_pin_encorderA_config.irq_handler = MotorEncorderAIrq;
	in_pin_encorderA_config.irq_args = NULL;
	printf("Config encorder A pin interrupt type as %s\r\n", irq_type_str[in_pin_encorderA_config.irq_type]);
	FFreeRTOSSetupPin(in_gpio_encorderA, &in_pin_encorderA_config, FALSE);

	//Motor EncorderA_B input config
	in_pin_encorderA_B_config.pin_idx = in_pin_encorderA_B;
	FFreeRTOSSetupPin(in_gpio_encorderA_B, &in_pin_encorderA_B_config, TRUE);

	//Front sensor input config
	in_pin_frontsensor_config.pin_idx = in_pin_frontsensor;
	FFreeRTOSSetupPin(in_gpio_frontsensor, &in_pin_frontsensor_config, TRUE);

	//Init Encorder A pin read
	FGpioPinId encorderA_B_pin_id;
	encorderA_B_pin_id.ctrl = FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_encorderA_B);
	encorderA_B_pin_id.port = FFREERTOS_GPIO_PIN_PORT_ID(in_pin_encorderA_B);
	encorderA_B_pin_id.pin = FFREERTOS_GPIO_PIN_ID(in_pin_encorderA_B);
	encorderA_B_pin = &in_gpio_encorderA_B->pins[encorderA_B_pin_id.port][encorderA_B_pin_id.pin];

	//Init Front sensor pin read
	FGpioPinId frontsensor_pin_id;
	frontsensor_pin_id.ctrl = FFREERTOS_GPIO_PIN_CTRL_ID(in_pin_frontsensor);
	frontsensor_pin_id.port = FFREERTOS_GPIO_PIN_PORT_ID(in_pin_frontsensor);
	frontsensor_pin_id.pin = FFREERTOS_GPIO_PIN_ID(in_pin_frontsensor);
	frontsensor_pin = &in_gpio_frontsensor->pins[frontsensor_pin_id.port][frontsensor_pin_id.pin];

	printf("MotorInit task GPIO config success.\r\n");
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarMotorPWMInit
 *  Function Describtion:   Init Car Motor PWM
 *  Function Use:           Init Car Motor PWM To Control Motor Speed
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-04-19 17:22:31
 *-----------------------------------------------------------------------------*/
void CarMotorPWMInit() {
	u32 pwm_motor_id = (u32)(uintptr)PWM_MOTOR_ID;

	FIOPadSetPwmMux(pwm_motor_id, 0);

	os_pwm_ctrl_motor_pin = FFreeRTOSPwmInit(pwm_motor_id);

	//set pwm db config
	FPwmDbVariableConfig db_cfg;
	memset(&db_cfg, 0, sizeof(db_cfg));
	db_cfg.db_rise_cycle = 500;
	db_cfg.db_fall_cycle = 500;
	db_cfg.db_polarity_sel = FPWM_DB_AHC;
	db_cfg.db_in_mode = FPWM_DB_IN_MODE_PWM0;
	db_cfg.db_out_mode = FPWM_DB_OUT_MODE_ENABLE_RISE_FALL;
	FFreeRTOSPwmDbSet(os_pwm_ctrl_motor_pin, &db_cfg);

	// start pwm config
	FPwmVariableConfig pwm_motor_cfg;
	memset(&pwm_motor_cfg, 0, sizeof(pwm_motor_cfg));
	pwm_motor_cfg.tim_ctrl_mode = FPWM_MODULO;
	pwm_motor_cfg.tim_ctrl_div = PWM_DIV - 1;
	pwm_motor_cfg.pwm_period = PWM_MOTOR_PERIOD;
	pwm_motor_cfg.pwm_pulse = PWM_MOTOR_PULSE;
	pwm_motor_cfg.pwm_mode = FPWM_OUTPUT_COMPARE;
	pwm_motor_cfg.pwm_polarity = FPWM_POLARITY_NORMAL;
	pwm_motor_cfg.pwm_duty_source_mode = FPWM_DUTY_CCR;
	FFreeRTOSPwmSet(os_pwm_ctrl_motor_pin, PWM_MOTOR_CHANNEL_USE, &pwm_motor_cfg);

	//Enable PWM
	FFreeRTOSPwmEnable(os_pwm_ctrl_motor_pin, PWM_MOTOR_CHANNEL_USE, TRUE);

	printf("Car Motor Pwm Init success.\r\n");
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarSteerPWMInit
 *  Function Describtion:   Init Car Steer PWM
 *  Function Use:           Init Car Steer PWM To Control Steer Angle
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-04-19 17:22:31
 *-----------------------------------------------------------------------------*/
void CarSteerPWMInit() {
	u32 pwm_steer_id = (u32)(uintptr)PWM_STEER_ID;

	FIOPadSetPwmMux(pwm_steer_id, 1);

	os_pwm_ctrl_steer_pin = FFreeRTOSPwmInit(pwm_steer_id);

	// start pwm config
	FPwmVariableConfig pwm_steer_cfg;
	memset(&pwm_steer_cfg, 0, sizeof(pwm_steer_cfg));
	pwm_steer_cfg.tim_ctrl_mode = FPWM_MODULO;
	pwm_steer_cfg.tim_ctrl_div = PWM_DIV - 1;
	pwm_steer_cfg.pwm_period = PWM_STEER_PERIOD;
	pwm_steer_cfg.pwm_pulse = PWM_STEER_PULSE;
	pwm_steer_cfg.pwm_mode = FPWM_OUTPUT_COMPARE;
	pwm_steer_cfg.pwm_polarity = FPWM_POLARITY_NORMAL;
	pwm_steer_cfg.pwm_duty_source_mode = FPWM_DUTY_CCR;
	FFreeRTOSPwmSet(os_pwm_ctrl_steer_pin, PWM_STEER_CHANNEL_USE, &pwm_steer_cfg);

	//Enable PWM
	FFreeRTOSPwmEnable(os_pwm_ctrl_steer_pin, PWM_STEER_CHANNEL_USE, TRUE);

	printf("Car Steer Pwm Init success.\r\n");
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarInitTask
 *  Function Describtion:   Init Car GPIO and PWM
 *  Function Use:           Init Car GPIO and PWM To Control Car Movement
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-04-19 17:22:31
 *-----------------------------------------------------------------------------*/
static void CarInitTask()
{
	//GPIO Init
	CarGPIOInit();
	//Motor PWM Init
	CarMotorPWMInit();
	//Steer PWM Init
	CarSteerPWMInit();

	FASSERT_MSG(init_locker, "Init locker NULL");
	for (u32 loop = 0U; loop < CAR_WORK_TASK_NUM; loop++)
	{
		xSemaphoreGive(init_locker);
	}
	vTaskDelete(NULL);
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarDetectFront
 *  Function Describtion:   Detect Front Obstacle
 *  Function Use:           Detect Front Obstacle And Trigger Emergency Break
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 13:13:25
 *-----------------------------------------------------------------------------*/
static void CarDetectFront()
{
	//Detect front obstacle, if front sensor is low, trigger emergency break
	if (FGpioGetInputValue(frontsensor_pin) == FGPIO_PIN_LOW && front_sensor_flag == 0) {
		//printf("检测到前方有障碍物，前紧急避障触发！！！\r\n小车此时无法前进，仅能后退！！！\r\n");
		front_sensor_flag = 1;
		//If emergency break triggered, set target speed as 0 and clear PI controller pulse, restart integral
		motor_target_speed = 0;
		pulse = 0;
	}
	//If front sensor is high, clear emergency break flag
	if (FGpioGetInputValue(frontsensor_pin) == FGPIO_PIN_HIGH && front_sensor_flag == 1) {
		front_sensor_flag = 0;
		//printf("前方已无障碍物，紧急制动解除，恢复正常。\r\n");
	}
	if (Debug_Info_Print) printf("当前紧急制动%s", (front_sensor_flag == 1) ? "触发" : "解除");
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarGetCurrentSpeed
 *  Function Describtion:   Get Current Motor Speed
 *  Function Use:           Get Current Motor Speed According To Motor Encorder Pulse
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 14:55:36
 *-----------------------------------------------------------------------------*/
static void CarGetCurrentSpeed()
{
	//Calculate current motor speed, current motor speed = (encoder pulse * (1000/control period) * wheel perimeter) / (2 * encoder precision)
	MotorASpeed = (MotorACount * (1000 / Control_Period) * Wheel_Perimeter) / (2.0 * Encorder_Precision);
	if (Debug_Info_Print) printf("编码器脉冲速为：%d,电机A速度为：%f，期望速度为：%f\r\n", MotorACount, MotorASpeed, motor_target_speed);
	MotorACount = 0;
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarCalculateDirectionAndPulse
 *  Function Describtion:   Calculate Motor Direction And Pulse
 *  Function Use:           Calculate Motor Direction And Pulse According To PI Pulse
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 14:55:40
 *-----------------------------------------------------------------------------*/
static void CarCalculateDirectionAndPulse(int PI_Pulse, int* motor_direction, u32* pwm_motor_pulse)
{
	if (PI_Pulse < 0) {
		*motor_direction = 0;
		*pwm_motor_pulse = (0 - PI_Pulse);
	}
	else {
		*motor_direction = 1;
		*pwm_motor_pulse = PI_Pulse;
	}
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarControlSteer
 *  Function Describtion:   Control Car Steer
 *  Function Use:           Control Car Steer According To Steer Pulse
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 14:55:43
 *-----------------------------------------------------------------------------*/
static void CarControlSteer()
{
	FFreeRTOSPwmPulseSet(os_pwm_ctrl_steer_pin, PWM_STEER_CHANNEL_USE, pwm_steer_pulse);
	if (Debug_Info_Print) printf("控制当前舵机脉冲为: %d\r\n", pwm_steer_pulse);
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarControlMotor
 *  Function Describtion:   Control Car Motor
 *  Function Use:           Control Car Motor According To Direction And Pulse
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 14:55:47
 *-----------------------------------------------------------------------------*/
static void CarControlMotor(int direction, int pwm_pulse)
{
	if (direction) {
		//前进
		FFreeRTOSPinWrite(out_gpio_direction, out_pin_direction, FGPIO_PIN_LOW);
		//pwm控制电机
		FFreeRTOSPwmPulseSet(os_pwm_ctrl_motor_pin, PWM_MOTOR_CHANNEL_USE, pwm_pulse);
	}
	else {
		//后退
		FFreeRTOSPinWrite(out_gpio_direction, out_pin_direction, FGPIO_PIN_HIGH);
		//pwm控制电机
		FFreeRTOSPwmPulseSet(os_pwm_ctrl_motor_pin, PWM_MOTOR_CHANNEL_USE, PWM_MOTOR_PERIOD - pwm_pulse);
	}
	if (Debug_Info_Print) printf("设置电机方向为：%s，电机脉冲为：%d\r\n", (direction == 1) ? "前进" : "后退", pwm_pulse);
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          CarControlTask
 *  Function Describtion:   Control Car Movement
 *  Function Use:           Control Car Movement According To Target Speed And Steer Angle
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-4-24 14:55:52
 *-----------------------------------------------------------------------------*/
static void CarControlTask(void* args)
{
	FASSERT(init_locker);
	xSemaphoreTake(init_locker, portMAX_DELAY);
	TickType_t lastWakeTime = xTaskGetTickCount();

	printf("Car control task started. \r\n");
	//init pwm pulse
	u32 pwm_motor_pulse = 0;//init motor pwm pulse as 0, which is stop
	//Car direction, 1 for forward, 0 for backward
	int motor_direction = 1;
	//init motor direction as forward
	FFreeRTOSPinWrite(out_gpio_direction, out_pin_direction, FGPIO_PIN_LOW);
	//enable motor encorder A irq
	(void)FFreeRTOSSetIRQ(in_gpio_encorderA, in_pin_encorderA, TRUE);

	int PI_pulse = 0;
	int Emergency_Break_Flag = 0;
	//set control period
	const TickType_t car_control_period = pdMS_TO_TICKS(Control_Period);
	while (1)
	{
		//vTaskDelayUntil(&lastWakeTime, car_control_period);
		//Detect front obstacle
		CarDetectFront();
		//Calculate current motor speed, unit is m/s
		CarGetCurrentSpeed();
		//Use PI controller to calculate motor pulse
		PI_pulse = Motor_PI_Control(MotorASpeed, motor_target_speed);
		//Calculate motor direction and pulse
		CarCalculateDirectionAndPulse(PI_pulse, &motor_direction, &pwm_motor_pulse);
		//Control steer according to target steer angle
		CarControlSteer();
		//Control motor and direction according to target speed
		CarControlMotor(motor_direction, pwm_motor_pulse);
		vTaskDelay(car_control_period);
	}
}
/*-----------------------------------------------------------------------------*
 *  Function Name:          FFreeRTOSRunCar
 *  Function Describtion:   Start Car Init Task And Control Task
 *  Function Use:           Init Car Pin And Start Control
 *	Editor:					Wang Ruifan
 *	LastEditTime:			2024-04-10 09:14:24
 *-----------------------------------------------------------------------------*/
BaseType_t FFreeRTOSRunCar()
{
	BaseType_t ret = pdPASS;
	BaseType_t xTimerStarted = pdPASS;

	if (is_running)
	{
		FGPIO_ERROR("The task is running.");
		return pdPASS;
	}

	is_running = TRUE;

	FASSERT_MSG(NULL == event, "Event group exists.");
	FASSERT_MSG((event = xEventGroupCreate()) != NULL, "Create event group failed.");

	FASSERT_MSG(NULL == init_locker, "Init locker exists.");
	FASSERT_MSG((init_locker = xSemaphoreCreateCounting(CAR_WORK_TASK_NUM, 0U)) != NULL, "Create event group failed.");

	taskENTER_CRITICAL(); /* no schedule when create task */

	ret = xTaskCreate((TaskFunction_t)CarInitTask,  //任务入口函数
		(const char*)"CarInitTask",//任务名字
		(uint16_t)1024,  //任务栈大小
		NULL, //任务入口函数参数
		(UBaseType_t)configMAX_PRIORITIES - 2,  //任务的优先级
		NULL); //任务控制

	FASSERT_MSG(pdPASS == ret, "Create task failed.");

	ret = xTaskCreate((TaskFunction_t)CarControlTask,  //任务入口函数
		(const char*)"CarControlTask",//任务名字
		(uint16_t)1024,  //任务栈大小
		NULL, //任务入口函数参数
		(UBaseType_t)configMAX_PRIORITIES - 3,  //任务的优先级
		(TaskHandle_t*)&control_task); //任务控制

	FASSERT_MSG(pdPASS == ret, "Create task failed.");

	return ret;
}