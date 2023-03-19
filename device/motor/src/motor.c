#include "motor.h"
#include "motor_internal.h"


/**
 * @brief 设置电机的编码器值读取接口
 * @param motor 目标电机对象
 * @param counter_handle 编码器读取句柄
 * @param update_if 编码器读取接口
 * @return int32_t 0 设置成功 其它 设置失败
 */
int32_t motor_set_counter_update_if(struct MOTOR *motor, void *counter_handle, MOTOR_READ_COUNTER update_if)
{
	if (NULL == motor)
	{
		return -1;
	}

	motor->counter_update_handle = counter_handle;
	motor->counter_update = update_if;

	return 0;
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
 * @brief 获取目标电机的设定值
 * @param motor 目标电机
 * @return 得到的目标电机设定值
 */
int32_t motor_get_target(struct MOTOR *motor)
{
	return motor->target;
}


/**
 * @brief 清除目标电机的所有累计状态
 * @param motor 目标电机句柄
 */
void motor_state_clear(struct MOTOR *motor)
{

}


/**
 * @brief 电机的速度环控制过程
 * @param motor 目标电机
 */
static void __motor_speed_ring_exec(struct MOTOR *motor)
{
	/**< 更新编码器采样值 */
	if (NULL == motor->counter_update)
	{
		return;
	}

	/**< 更新编码器采样值到计数器中 */
	motor->counter_update(motor->counter, motor->counter_update_handle);

	/**< 获取本次编码器增量值 作为速度值，相关速度和编码器值的转换，后续再做，先跑通流程 */
	counter_get_rel_value(motor->counter, 2, &motor->cur_value);
	
	PIDController_Update(&motor->pid, motor->target, motor->cur_value);
	MOTOR_DEBUG("speed ring exec: target cnt: %d, cur cnt: %d, out cnt: %f\r\n", motor->target, motor->cur_value, motor->pid.out);

	if (NULL != motor->ctrl_out)
	{
		motor->ctrl_out(motor, motor->ctrl_out_handle, motor->pid.out);
	}
}


/**
 * @brief 电机的位置环控制过程
 * @param motor 目标电机
 */
static void __motor_position_ring_exec(struct MOTOR *motor)
{
	/**< 更新编码器采样值 */
	if (NULL == motor->counter_update)
	{
		return;
	}

	/**< 更新编码器采样值到计数器中 */
	motor->counter_update(motor->counter, motor->counter_update_handle);

	/**< 获取本次编码器增量值 并累计作为位置值，相关位置和编码器值的转换，后续再做，先跑通流程 */
	int32_t cur_value = 0;
	counter_get_rel_value(motor->counter, 2, &cur_value);
	motor->cur_value += cur_value;
	
	PIDController_Update(&motor->pid, motor->target, motor->cur_value);
	MOTOR_DEBUG("position ring exec: target cnt: %d, cur cnt: %d, out cnt: %f\r\n", motor->target, motor->cur_value, motor->pid.out);

	if (NULL != motor->ctrl_out)
	{
		motor->ctrl_out(motor, motor->ctrl_out_handle, motor->pid.out);
	}
}


/**
 * @brief 执行指定电机的控制过程
 * @param motor 待执行的电机对象
 */
static void __motor_invalid_exec(struct MOTOR *motor)
{

}


/**
 * @brief 执行模块内的电机控制
 * @param handle 电机模块管理对象
 * @param elapsed_us 距上次电机模块调用的间隔
 * @return 下次调用电机模块的执行间隔
 */
int32_t motor_exec(struct MOTOR_MANAGER *handle, int32_t elapsed_us)
{
	struct MOTOR *_var_motor = NULL;

	SIMPLEQ_FOREACH(_var_motor, &handle->motors, MOTOR)
	{
		_var_motor->elapsed_us += elapsed_us;
		if (_var_motor->elapsed_us >= _var_motor->interval_us)
		{
			/**< 该电机本轮到达执行时间 */
			_var_motor->elapsed_us = 0;

			if (NULL != _var_motor->exec)
			{
				_var_motor->exec(_var_motor);
			}
		}
	}

	return 0;
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
 * @param type 控制过程类型
 * @return 创建完成的电机对象指针 NULL 创建失败
 */
struct MOTOR *motor_create(struct MOTOR_MANAGER *handle, int32_t interval_us, PIDController *pid, enum MOTOR_CTRL_TYPE type)
{
	struct MOTOR *_new_motor = (struct MOTOR *)malloc(sizeof(struct MOTOR));

	if (NULL != _new_motor)
	{
		memset(_new_motor, 0, sizeof(struct MOTOR));
		_new_motor->interval_us = interval_us;

		if (MCT_SPEED_RING_CTRL == type)
		{
		}

		switch (type)
		{
		case MCT_SPEED_RING_CTRL:
			_new_motor->exec = __motor_speed_ring_exec;
			break;
		case MCT_POSITION_RING_CTRL:
			_new_motor->exec = __motor_position_ring_exec;
			break;
		default:
			_new_motor->exec = __motor_invalid_exec;
			break;
		}

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


/**
 * @brief 为目标电机设置编码计数器对象
 * @param motor 目标电机对象
 * @param counter 计数器对象
 * @return 0 设置成功 其它 失败
 */
int32_t motor_set_counter(struct MOTOR *motor, struct COUNTER *counter)
{
	if (NULL == motor || NULL == counter)
	{
		return -1;
	}

	motor->counter = counter;
	return 0;
}


/**
 * @brief 获取目标电机的目标完成率
 * @param motor 获取目标电机的目标完成率
 * @return 得到的目标完成率 百分比
 */
float motor_get_target_ratio(struct MOTOR *motor)
{
	return (float)motor->cur_value / motor->target;
}


/**
 * @brief 获取目标电机的当前值
 * @param motor 目标电机
 * @return int32_t 得到的当前值
 */
int32_t motor_get_cur_value(struct MOTOR *motor)
{
	return motor->cur_value;
}
