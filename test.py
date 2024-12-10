import numpy as np
# import module motor
from sim2real.module.motor.motors import MOTOR

# 创建MOTOR类的实例
motor = MOTOR()

# 获取电机IP，检查电机连接情况
motor.get_motors_ip()


# 设置电机进入位置控制模式（使能电机并设置为位置控制模式）
motor.set_position_mode()

# 构造目标位置数组，假设这里所有电机都要转到10度位置，根据原MOTOR类的set_position_control方法要求，需要是和电机数量对应的numpy数组
target_position = np.full((12,), 5, dtype=np.double)  # 创建一个长度为12，元素都为10的numpy数组，表示12个电机的目标位置都是10度

# 调用设置位置控制方法，传入目标位置，让电机转动到目标位置
motor.set_position_control(target_position)
