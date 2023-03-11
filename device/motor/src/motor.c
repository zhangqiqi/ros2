#include "motor.h"
#include "motor_encoder.h"
#include "motor_internal.h"


/**
 * @brief 设置电机的编码器值读取接口
 * @param motor 目标电机对象
 * @param encoder_handle 编码器读取句柄
 * @param read_if 编码器读取接口
 * @return int32_t 0 设置成功 其它 设置失败
 */
int32_t motor_set_encoder_read_if(struct MOTOR *motor, void *encoder_handle, MOTOR_READ_ENCODER read_if)
{
	int32_t ret = 0;

	if (NULL == motor)
	{
		return -1;
	}

	motor->encoder.encoder_handle = encoder_handle;
	motor->encoder.encoder_read = read_if;

	return ret;
}


/**
 * @brief 设置电机的控制输出接口
 * @param motor 目标电机对象
 * @param ctrl_out_handle 电机控制输出接口操作句柄
 * @param ctrl_out_if 电机控制输出接口
 * @return 0 成功 其它 失败
 */
int32_t motor_set_ctrl_out_if(struct MOTOR *motor, void *ctrl_out_handle, MOTOR_CTRL_OUT ctrl_out_if)
{
	if (NULL == motor)
	{
		return -1;
	}

	motor->ctrl_out_handle = ctrl_out_handle;
	motor->ctrl_out = ctrl_out_if;

	return 0;
}


/**
 * @brief 设置目标电机的输出值
 * @param motor 目标电机对象
 * @param target 目标值
 */
void motor_set_target(struct MOTOR *motor, float target)
{
	motor->target = target;
}


/**
 * @brief 执行指定电机的控制过程
 * @param motor 待执行的电机对象
 */
static void __motor_exec(struct MOTOR *motor)
{
	/**< 更新编码器采样值 */
	float cur_cnt = motor_encoder_update(&motor->encoder);
	
	PIDController_Update(&motor->pid, motor->target, cur_cnt);
	MOTOR_DEBUG("target cnt: %f, cur cnt %f, out cnt: %f\r\n", motor->target, cur_cnt, motor->pid.out);

	if (NULL != motor->ctrl_out)
	{
		motor->ctrl_out(motor->ctrl_out_handle, motor->pid.out);
	}
}


/**
 * @brief 执行模块内的电机控制
 * @param handle 电机模块管理对象
 * @param elapsed_us 距上次电机模块调用的间隔
 * @return 下次调用电机模块的执行间隔
 */
int32_t motor_exec(struct MOTOR_MANAGER *handle, int32_t elapsed_us)
{
	int32_t _next_interval = 1000 * 1000;      /**< 默认最大的间隔时间是 1s */
	struct MOTOR *_var_motor = NULL;

	SIMPLEQ_FOREACH(_var_motor, &handle->motors, MOTOR)
	{
		_var_motor->elapsed_us += elapsed_us;
		if (_var_motor->elapsed_us >= _var_motor->interval_us)
		{
			/**< 该电机本轮到达执行时间 */
			_var_motor->elapsed_us = 0;
			__motor_exec(_var_motor);
		}
	}
}


/**
 * @brief 电机模块初始化
 * @return 初始化完成的电机模块管理句柄
 */
struct MOTOR_MANAGER *motor_init(void)
{
	struct MOTOR_MANAGER *_new_manager = (struct MOTOR_MANAGER *)malloc(sizeof(struct MOTOR_MANAGER));

	if (NULL != _new_manager)
	{
		memset(_new_manager, 0, sizeof(struct MOTOR_MANAGER));
		SIMPLEQ_INIT(&_new_manager->motors);
	}
	else
	{
		MOTOR_ERROR("motor init failed, malloc manager failed\r\n");
	}

	return _new_manager;
}


/**
 * @brief 创建新的电机对象
 * @param handle 电机模块管理句柄
 * @param interval_us 创建的电机的控制间隔时间
 * @param pid 控制该电机使用的pid参数
 * @return 创建完成的电机对象指针 NULL 创建失败
 */
struct MOTOR *motor_create(struct MOTOR_MANAGER *handle, int32_t interval_us, PIDController *pid)
{
	struct MOTOR *_new_motor = (struct MOTOR *)malloc(sizeof(struct MOTOR));

	if (NULL != _new_motor)
	{
		memset(_new_motor, 0, sizeof(struct MOTOR));
		_new_motor->interval_us = interval_us;

		if (NULL != pid)
		{
			/**< 若外部传递了pid控制参数，则使用外部传递的控制参数 */
			memcpy(&_new_motor->pid, pid, sizeof(PIDController));
		}
		else
		{
			/**< 若外部没有传递控制参数，则使用默认的pid控制参数 */
			PID_SET_DEFAULT_PARAMS(&_new_motor->pid);
		}

		_new_motor->pid.T = (float)interval_us / (1000 * 1000);

		PIDController_Init(&_new_motor->pid);

		MOTOR_NOTICE("motor create success, pid param: \r\n");
		MOTOR_NOTICE("                             Kp: %f\r\n", _new_motor->pid.Kp);
		MOTOR_NOTICE("                             Ki: %f\r\n", _new_motor->pid.Ki);
		MOTOR_NOTICE("                             Kd: %f\r\n", _new_motor->pid.Kd);
		MOTOR_NOTICE("                              T: %f\r\n", _new_motor->pid.T);

		SIMPLEQ_INSERT_TAIL(&handle->motors, _new_motor, MOTOR);
	}
	else
	{
		MOTOR_ERROR("motor create failed, malloc motor failed\r\n");
	}

	return _new_motor;
}
