/*
*
* File Name: rpmsg-echo_os.c
* Path: src/rpmsg-echo_os.c
*
* Editor: Wang Ruifan
*
* Description:	Remote CPU running freertos code and handle car target speed data that was sent to it by the master core.
* Based File: Phytium-FreeRTOS-SDK/example/system/amp/openamp_for_linux/src/rpmsg-echo_os.c
*
* LastEditTime: 2024-4-23 00:09:23
*
*/

/***************************** Include Files *********************************/

/***************************** Include Files *********************************/

#include <stdio.h>
#include <openamp/open_amp.h>
#include <openamp/version.h>
#include <metal/alloc.h>
#include <metal/version.h>
#include "platform_info.h"
#include "rpmsg_service.h"
#include <metal/sleep.h>
#include "rsc_table.h"
#include "FreeRTOS.h"
#include "task.h"
#include "finterrupt.h"
#include "fpsci.h"
#include "fdebug.h"
#include "fgpio_os.h"
#include <stdlib.h>
#include <math.h>

/************************** Constant Definitions *****************************/
//rpmsg接收周期
#define RPMSG_POLL_PERIOD           ( pdMS_TO_TICKS( 20U ))

#define RATIO 63.656
#define WHEEL_SPACING 0.162f 
#define SPACING 0.144f
#define MINRADIUS 0.35

#define DEBUG_INFO_PRINT 0
/**************************** Type Definitions *******************************/

/************************** Variable Definitions *****************************/
static int shutdown_req;
static char temp_data[RPMSG_BUFFER_SIZE];
extern volatile FGpioPinVal out_val;
extern volatile int pwm_steer_pulse;
extern volatile float motor_target_speed;
extern volatile float MotorASpeed;
extern volatile int front_sensor_flag;
static char cmd[RPMSG_BUFFER_SIZE];
float R = 0.0;

/***************** Macros (Inline Functions) Definitions *********************/

#define OPENAMP_SLAVE_DEBUG_TAG "OPENAMP_SLAVE"
#define OPENAMP_SLAVE_ERROR(format, ...) FT_DEBUG_PRINT_E(OPENAMP_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define OPENAMP_SLAVE_WARN(format, ...)  FT_DEBUG_PRINT_W(OPENAMP_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define OPENAMP_SLAVE_INFO(format, ...)  FT_DEBUG_PRINT_I(OPENAMP_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)
#define OPENAMP_SLAVE_DEBUG(format, ...) FT_DEBUG_PRINT_D(OPENAMP_SLAVE_DEBUG_TAG, format, ##__VA_ARGS__)

#define SHUTDOWN_MSG				0xEF56A55A

/************************** Function Prototypes ******************************/
/*-----------------------------------------------------------------------------*
 *  Function Name:          target_limit_float
 *  Function Describtion:   Limit target value
 *  Function Use:           Limit int type target value between low and high
 *	Function Parameter:     float insert, float low, float high
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-13 14:32:26
 *-----------------------------------------------------------------------------*/
float target_limit_float(float insert, float low, float high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}
/*-----------------------------------------------------------------------------*
 *  Function Name:          target_limit_int
 *  Function Describtion:   Limit target value
 *  Function Use:           Limit int type target value between low and high
 *	Function Parameter:     int insert, int low, int high
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-13 14:32:26
 *-----------------------------------------------------------------------------*/
int target_limit_int(int insert, int low, int high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/*-----------------------------------------------------------------------------*
 *  Function Name:          rpmsg_endpoint_cb
 *  Function Describtion:   Callback function for rpmsg endpoint
 *  Function Use:           Receuve target speed data from master core and send back the car status data
 *	Function Parameter:     struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-4-23 00:14:12
 *-----------------------------------------------------------------------------*/
static int rpmsg_endpoint_cb(struct rpmsg_endpoint* ept, void* data, size_t len, uint32_t src, void* priv)
{
	(void)priv;
	(void)src;
	/* On reception of a shutdown we signal the application to terminate */
	if ((*(unsigned int*)data) == SHUTDOWN_MSG) {
		OPENAMP_SLAVE_INFO("shutdown message is received.");
		shutdown_req = 1;
		return RPMSG_SUCCESS;
	}

	memset(cmd, 0, len);
	memcpy(cmd, data, len);
	if(DEBUG_INFO_PRINT) printf("success 从核收到消息：%s\r\n",cmd);

	const char* str = cmd;
	char* token;
	char* remaining = (char*)str; // 使用原字符串的副本
	float numbers[2]; // 只提取两个数字
	sscanf(cmd, " %f %f", &numbers[0], &numbers[1]);
	
	//发送是否触发紧急制动和当前实际速度给主核
	float Vx_current, Vz_current;

	Vx_current = MotorASpeed;

	// 计算 Vz
	if(R == 0) Vz_current = 0;
	else Vz_current = MotorASpeed / R;
	
	uint8_t buf[32];
	snprintf(buf, sizeof(buf), " %d %.2f %.2f", front_sensor_flag, Vx_current,Vz_current);
	//printf("send: %d %.2f %.2f\r\n",front_sensor_flag,Vx_current,Vz_current);
	// 请勿直接对data指针对应的内存进行写操作，操作vring中remoteproc发送通道分配的内存，引发错误的问题

	// Send data back to master
	if (rpmsg_send(ept, buf, sizeof(buf)) < 0)
		OPENAMP_SLAVE_ERROR("rpmsg_send failed!!!\r\n"); 

	//处理主核发来的期望速度
	float Vx, Vz;
	Vx = numbers[0];
	Vz = numbers[1];
	if (front_sensor_flag == 1 && Vx > 0) {
		//printf("前紧急避障触发后，小车无法前进，仅能后退！！！\r\n请后退直到前方检测不到障碍物\r\n");
	}
	else {
		float AngleR, Angle_Servo;
		
		//阿克曼小车需要设置最小转弯半径
		//如果目标速度要求的转弯半径小于最小转弯半径，
		//会导致小车运动摩擦力大大提高，严重影响控制效果
	
		if(Vz!=0 && Vx!=0)
		{
			//如果目标速度要求的转弯半径小于最小转弯半径
			if(float_abs(Vx/Vz)<=MINRADIUS)
			{
				//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
				if(Vz>0)
					Vz= float_abs(Vx)/(MINRADIUS);
				else	
					Vz=-float_abs(Vx)/(MINRADIUS);	
			}		
			R=Vx/Vz;
			AngleR=atan(SPACING/R);
		}
		else
		{
			AngleR=0;
		}
	
		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
		AngleR = target_limit_float(AngleR, -0.4f, 0.4f);
		R = SPACING / tan(AngleR);
		//Inverse kinematics //运动学逆解
		motor_target_speed = Vx;
		//舵机PWM值，舵机控制前轮转向角度
		Angle_Servo = -0.628f * pow(AngleR, 3) + 1.269f * pow(AngleR, 2) - 1.772f * AngleR + 1.573f;
		pwm_steer_pulse = 150 + (Angle_Servo - 1.572f) * RATIO;
		//电机目标速度限幅
		motor_target_speed = target_limit_float(motor_target_speed, -1.5, 1.5);
		pwm_steer_pulse = target_limit_int(pwm_steer_pulse, 80, 220);	//Servo PWM value limit //舵机PWM值限幅
		//printf("success!设置小车方向为：%s,设置期望速度为：%f，设置舵机脉冲为：%d\r\n", (motor_target_speed >= 0) ? "前进" : "后退", motor_target_speed, pwm_steer_pulse);
	}
	

		return RPMSG_SUCCESS;
}
/*-----------------------------------------------------------------------------*
 *  Function Name:          rpmsg_service_unbind
 *  Function Describtion:   Callback function for rpmsg endpoint
 *  Function Use:           Receuve target speed data from master core and send back the car status data
 *	Function Parameter:     struct rpmsg_endpoint *ept
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-10 09:12:56
 *-----------------------------------------------------------------------------*/
static void rpmsg_service_unbind(struct rpmsg_endpoint* ept)
{
	(void)ept;
	OPENAMP_SLAVE_INFO("unexpected Remote endpoint destroy.");
	shutdown_req = 1;
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          FRpmsgEchoApp
 *  Function Describtion:   Main function for rpmsg echo application
 *  Function Use:           Create rpmsg endpoint and handle the data
 *	Function Parameter:     struct rpmsg_device *rdev, void *priv
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-10 09:12:56
 *-----------------------------------------------------------------------------*/
int FRpmsgEchoApp(struct rpmsg_device* rdev, void* priv)
{
	int ret;
	struct rpmsg_endpoint lept;
	shutdown_req = 0;
	OPENAMP_SLAVE_INFO("Try to create rpmsg endpoint.");
	/* Initialize RPMSG framework */
	ret = rpmsg_create_ept(&lept, rdev, RPMSG_SERVICE_NAME, 0, RPMSG_ADDR_ANY, rpmsg_endpoint_cb, rpmsg_service_unbind);
	if (ret)
	{
		OPENAMP_SLAVE_ERROR("Failed to create endpoint,ret = %d.\r\n", ret);
		return -1;
	}

	while (1)
	{
		
		platform_poll_loop(priv);
		/* we got a shutdown request, exit */
		if (shutdown_req)
		{
			break;
		}
		vTaskDelay(RPMSG_POLL_PERIOD);
	}

	rpmsg_destroy_ept(&lept);
	return 0;
}
/*-----------------------------------------------------------------------------*
 *  Function Name:          rpmsg_echo
 *  Function Describtion:   Main function for rpmsg echo application
 *  Function Use:           Create rpmsg endpoint and handle the data
 *	Function Parameter:     int argc, char *argv[]
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-10 09:12:56
 *-----------------------------------------------------------------------------*/
int rpmsg_echo(int argc, char* argv[])
{
	void* platform;
	struct rpmsg_device* rpdev;
	int ret = 0;

	OPENAMP_SLAVE_INFO("openamp lib version: %s (", openamp_version());
	OPENAMP_SLAVE_INFO("Major: %d, ", openamp_version_major());
	OPENAMP_SLAVE_INFO("Minor: %d, ", openamp_version_minor());
	OPENAMP_SLAVE_INFO("Patch: %d)\r\n", openamp_version_patch());

	OPENAMP_SLAVE_INFO("libmetal lib version: %s (", metal_ver());
	OPENAMP_SLAVE_INFO("Major: %d, ", metal_ver_major());
	OPENAMP_SLAVE_INFO("Minor: %d, ", metal_ver_minor());
	OPENAMP_SLAVE_INFO("Patch: %d)\r\n", metal_ver_patch());

	/* Initialize platform */
	OPENAMP_SLAVE_INFO("start application...");
	ret = platform_init(argc, argv, &platform);
	if (ret)
	{
		OPENAMP_SLAVE_ERROR("Failed to initialize platform.\r\n");
		platform_cleanup(platform);
		ret = -1;
	}

	rpdev = platform_create_rpmsg_vdev(platform, 0, VIRTIO_DEV_SLAVE, NULL, NULL);
	if (!rpdev)
	{
		OPENAMP_SLAVE_ERROR("Failed to create rpmsg virtio device.\r\n");
		platform_cleanup(platform);
		ret = -1;
	}

	ret = FRpmsgEchoApp(rpdev, platform);
	if (ret)
	{
		OPENAMP_SLAVE_ERROR("Failed to running echoapp.\r\n");
		platform_cleanup(platform);
		ret = -1;
	}

	platform_release_rpmsg_vdev(rpdev, platform);
	OPENAMP_SLAVE_INFO("Stopping application...");
	platform_cleanup(platform);

	return ret;
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          RpmsgEchoTask
 *  Function Describtion:   Task function for rpmsg echo application
 *  Function Use:           Create rpmsg endpoint and handle the data
 *	Function Parameter:     void * args
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-10 09:12:56
 *-----------------------------------------------------------------------------*/
void RpmsgEchoTask(void* args)
{
	int ret;
	ret = rpmsg_echo(0, NULL);
	if (ret)
	{
		OPENAMP_SLAVE_ERROR("Failed to running rpmsg_echo.\r\n");
	}
	vTaskDelete(NULL);
}

/*-----------------------------------------------------------------------------*
 *  Function Name:          rpmsg_echo_task
 *  Function Describtion:   Task function for rpmsg echo application
 *  Function Use:           Create rpmsg endpoint and handle the data
 *	Function Parameter:     void
 *	Editor:                 Wang Ruifan
 *	LastEditTime:           2024-04-10 09:12:56
 *-----------------------------------------------------------------------------*/
int rpmsg_echo_task(void)
{
	BaseType_t ret;

	ret = xTaskCreate((TaskFunction_t)RpmsgEchoTask, /* 任务入口函数 */
		(const char*)"RpmsgEchoTask",/* 任务名字 */
		(uint16_t)(4096 * 2), /* 任务栈大小 */
		(void*)NULL,/* 任务入口函数参数 */
		(UBaseType_t)configMAX_PRIORITIES - 1, /* 任务的优先级 */
		NULL); /* 任务控制块指针 */

	if (ret != pdPASS)
	{
		OPENAMP_SLAVE_ERROR("Failed to create a rpmsg_echo task. \r\n");
		return -1;
	}
	return 0;
}