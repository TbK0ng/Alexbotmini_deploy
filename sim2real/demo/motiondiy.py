import numpy as np
from fi_fsa import fi_fsa_v2
import time
from scipy.interpolate import CubicSpline
import numpy as np
import time
from scipy.interpolate import interp1d

class MOTOR:
    def __init__(self):
        self.server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                            '192.168.137.104', '192.168.137.105', '192.168.137.106',
                            '192.168.137.107', '192.168.137.108', '192.168.137.109',
                            '192.168.137.110', '192.168.137.111', '192.168.137.112']
        self.motors_num = len(self.server_ip_list)
        self.q = []
        self.dq = []
        self.current_positions = []

    def get_motors_ip(self):
        """
        获取电机对应的服务器IP地址列表，并进行相关验证
        """
        server_ip_list_test = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
        if not isinstance(server_ip_list_test, list):
            raise TypeError("Can not find motor, please check the wire or reboot motor.")
        if len(server_ip_list_test)!= self.motors_num:
            print('Lost connection of motors')
            print("Server IP List:", server_ip_list_test)
            print("Motors Num:", len(server_ip_list_test))
            raise ValueError('Lost connection of motors. We only find',len(server_ip_list_test),'motor(s)')
        return server_ip_list_test

    def get_pvc(self):#(degree)
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        """
        self.q = []
        self.dq = []
        for ip in self.server_ip_list:
            position, velocity, current = fi_fsa_v2.fast_get_pvc(ip)
            # pos = np.array(position).astype(np.double) * (np.pi / 180)
            # vel = np.array(velocity).astype(np.double) * (np.pi / 180)
            pos = np.array(position).astype(np.double)
            vel = np.array(velocity).astype(np.double)
            self.q.append(pos)
            self.dq.append(vel)
        self.current_positions = np.array([pos.item() if np.ndim(pos) == 0 else pos[0] for pos in self.q])
        return self.q, self.dq
    

    def set_position_mode(self):
        """
        设置所有电机为位置控制模式
        """
        for ip in self.server_ip_list:
            fi_fsa_v2.fast_set_enable(ip)
            fi_fsa_v2.fast_set_mode_of_operation(ip, fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL)

    def set_position(self, target_position, num_interpolation=10):
        # 获取当前位置
        current_positions = self.get_pvc()[0]
        # change motors into target position through interpolation
        for i in range(len(self.server_ip_list)):
            # 从 target_position 列表中取出一个元素作为最终位置参数，并确保它是浮点数
            target_pos = target_position[i]
            # 从 current_positions 列表中取出一个元素作为起始位置参数，并确保它是浮点数
            current_pos = current_positions[i]
            # 生成从当前位置到目标位置的插值序列
            interpolation_sequence = np.linspace(current_pos, target_pos, num_interpolation)
            for position in interpolation_sequence:
                # 将电机设置到相应的插值位置
                fi_fsa_v2.fast_set_position_control(self.server_ip_list[i], position)
                time.sleep(0.0005)

    def sampling_loop(self):
        all_data = []
        try:
            while True:
                # 调用 get_pvc 方法获取电机的位置、速度和电流信息
                current_positions = self.get_pvc()
                print("current_position is ", current_positions)
                # 将当前位置信息添加到 all_data 列表中
                all_data.append(current_positions)
                print(f"Data appended to list")
                # 等待用户按下回车键
                input("Press Enter to continue...")
        except KeyboardInterrupt:
            # 生成唯一的文件名，使用时间戳
            timestamp = time.strftime("%Y%m%d%H%M%S")
            filename = f"sampled_data_{timestamp}.npy"
            # 将采样结果存储为 numpy 数组文件
            np.save(filename, np.array(all_data))
            print(f"Data saved to {filename}")
            print("Sampling stopped.")

# if __name__ == "__main__":
#     motor = MOTOR()
#     server_ip_list = motor.get_motors_ip()
#     motor.set_position_mode()
#     target2_position = np.array([-40, 0, 0, 68, -28, 28, 40, 0, 0, -68, 28, 28])
#     target1_position = np.array([-10, 0, 0, 18, -8, 8, 10, 0, 0, -18, 8, 8])
#     init_position = np.array([-0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#     num_interpolation = 20  # 增加插值点数，使运动轨迹更细腻
#     # 生成从 init_position 到 target2_position 的插值序列
#     interpolation_sequence1 = np.linspace(init_position, target2_position, num_interpolation + 1)
#     # 生成从 target2_position 回到 init_position 的插值序列
#     interpolation_sequence2 = np.linspace(target2_position, init_position, num_interpolation + 1)

#     while True:
#         # 从 init_position 到 target2_position 的柔顺运动
#         for position in interpolation_sequence1:
#             motor.set_position(position)
#             time.sleep(0.005)  # 减小时间间隔，加快位置更新频率
#         # 从 target2_position 回到 init_position 的柔顺运动
#         for position in interpolation_sequence2:
#             motor.set_position(position)
#             time.sleep(0.005)

# if __name__ == "__main__":
#     motor = MOTOR()
#     server_ip_list = motor.get_motors_ip()
#     current_positions = motor.get_pvc()
#     current_positions_array = np.array(current_positions)
#     print("current_position is ", current_positions_array)

# if __name__ == "__main__":
#     motor = MOTOR()
#     server_ip_list = motor.get_motors_ip()
    
#     # 启动采样循环
#     motor.sampling_loop()

if __name__ == "__main__":
    motor = MOTOR()
    motor.get_motors_ip()
    motor.set_position_mode()
    while True:
        # 采样并获取存储数据的文件名
        filename = "sampled_data_20250119011119.npy"
        # 从存储的数据文件中读取数据
        sampled_data = np.load(filename)
        # sampled_data 是一个包含元组的列表，每个元组包含 (q, dq)
        # 我们只使用 q 作为目标位置
        target_positions = []
        for data in sampled_data:
            q = data[0]  # 假设 q 是一个包含十二个电机位置数据的列表
            # 将 q 列表中的每个元素都变成一个简单的浮点数列表
            simple_q = [float(item) for item in q]
            target_positions.append(simple_q)
        
        # 对目标位置进行插值
        num_interpolation = 50  # 插值点数
        interpolated_target_positions = []
        for i in range(len(target_positions[0])):  # 假设每个目标位置列表长度相同
            # 提取每个电机的位置序列
            positions = [target_position[i] for target_position in target_positions]
            x = np.linspace(0, 1, len(positions))
            interp_func = interp1d(x, positions)
            interpolation_sequence = interp_func(np.linspace(0, 1, num_interpolation))
            interpolated_target_positions.append(interpolation_sequence)
        
        # 转置 interpolated_target_positions 以匹配电机顺序
        interpolated_target_positions = np.array(interpolated_target_positions).T
        
        # 设置电机位置
        for target_position in interpolated_target_positions:
            motor.set_position(target_position)
        