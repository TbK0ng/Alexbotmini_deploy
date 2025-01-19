import numpy as np
from fi_fsa import fi_fsa_v2
import time
from scipy.interpolate import CubicSpline


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
            raise ValueError('Lost connection of motors. The number of motors is not correct.')
        return server_ip_list_test

    def get_pvc(self):
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        """
        self.q = []
        self.dq = []
        for ip in self.server_ip_list:
            position, velocity, current = fi_fsa_v2.fast_get_pvc(ip)
            pos = np.array(position).astype(np.double) * (np.pi / 180)
            vel = np.array(velocity).astype(np.double) * (np.pi / 180)
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
                


if __name__ == "__main__":
    motor = MOTOR()
    server_ip_list = motor.get_motors_ip()
    motor.set_position_mode()

    # 假设要设置的目标位置，这里示例为全0位置，你可以替换为实际需要的目标位置数组
    while 1:
        target_position1 = [10,10,10,10,10,10,10,10,10,10,10,10]
        target_position2 = [0,0,0,0,0,0,0,0,0,0,0,0]
        motor.set_position(target_position1)
        motor.set_position(target_position2)

    
