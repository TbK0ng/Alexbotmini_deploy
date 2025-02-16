
import numpy as np
from fi_fsa import fi_fsa_v2
import time
import numpy as np
import time
from module.motor.motors import MOTOR

kps = np.array([225, 150, 150, 225, 60, 60,225, 150, 150, 225, 60, 60,], dtype=np.double)
kds = np.array([12.5, 10, 10, 12.5, 4, 4, 12.5, 10, 10, 12.5, 4, 4,], dtype=np.double)

if __name__ == "__main__":
    motor = MOTOR()
    
    server_ip_list = motor.get_motors_ip()
    motor.set_position_mode()
    time.sleep(1)
    target2_position = np.array([-60, 0, 0, 100, 36,36, 60, 0, 0, -100, -36, 36])
    # target2_position = np.array([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8])
    init_position = np.array([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8])
    num_interpolation = 45  # 增加插值点数，使运动轨迹更细腻
    # 生成从 init_position 到 target2_position 的插值序列
    interpolation_sequence1 = np.linspace(init_position, target2_position, num_interpolation + 1)
    # 生成从 target2_position 回到 init_position 的插值序列
    interpolation_sequence2 = np.linspace(target2_position, init_position, num_interpolation + 1)

    while True:
        # 从 init_position 到 target2_position 的柔顺运动
        for position in interpolation_sequence1:
            motor.set_position(position)
            time.sleep(0.008)  # 减小时间间隔，加快位置更新频率
        # 从 target2_position 回到 init_position 的柔顺运动
        for position in interpolation_sequence2:
            motor.set_position(position)
            time.sleep(0.008)

