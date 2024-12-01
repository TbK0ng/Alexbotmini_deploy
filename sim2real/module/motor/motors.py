import numpy as np
from fi_fsa import fi_fsa_v2
import time

server_ip_list = ['192.168.137.101','192.168.137.102','192.168.137.103',
                  '192.168.137.104','192.168.137.105','192.168.137.106',
                  '192.168.137.107','192.168.137.108','192.168.137.109',
                  '192.168.137.110','192.168.137.111','192.168.137.112'] 
motors_num = 12 
server_ip_list_test =  []

class MOTOR:
    def __init__(self):
        self.q = []  # 初始化类的属性q为一个空列表
        self.dq = []  # 初始化类的属性dq为一个空列表

    def get_motors_ip(self):
        """
        初始化电机对象，在初始化时获取该电机对应的服务器IP地址
        """
        server_ip_list_test = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
        motors_num_test = len(server_ip_list_test)
        if motors_num_test == 12:
            print("Server IP List:", server_ip_list)
            print("Motors Num:", 12)   
        else:
            print('Lost connection of motors' )
            print("Server IP List:", server_ip_list_test)
            print("Motors Num:", motors_num_test)
            raise ValueError('Lost connection of motors. The number of motors is not 12.')


    def get_pvc(self):
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        返回值：
        - position：以numpy.ndarray（float64类型）表示的电机位置信息，对应类似q变量（关节位置）的概念
        - velocity：以numpy.ndarray（float64类型）表示的电机速度信息，对应类似dq变量（关节速度）的概念
        - current：电机的电流值（具体类型根据fi_fsa_v2模块返回确定，这里暂未做类型转换处理）
        """
        for i in range(len(server_ip_list)):
            position, velocity, current = fi_fsa_v2.fast_get_pvc(server_ip_list[i])
            # 将position转换为指定格式（numpy.double类型）
            pos = np.array(position).astype(np.double)* (np.pi / 180)
            # 将velocity转换为指定格式（numpy.double类型）
            vel = np.array(velocity).astype(np.double)* (np.pi / 180)
            self.q.append(pos)
            self.dq.append(vel)
        self.q = np.array(self.q) if self.q else np.array([])
        self.dq = np.array(self.dq) if self.dq else np.array([])
        print("Position array:", self.q[-12:])  # 打印位置数组的后12个元素
        print("Velocity array:", self.dq[-12:])  # 打印速度数组的后12个元素
        return self.q, self.dq


    # def set_position_control():
    #     """
    #     设置电机的位置控制模式，将电机设置到目标位置
    #     参数：
    #     - target_position：目标位置值（具体单位等需根据实际情况确定，示例中传入的单位类似角度[deg]）
    #     """
    #     fi_fsa_v2.fast_set_enable(self.server_ip)
    #     fi_fsa_v2.fast_set_mode_of_operation(
    #         self.server_ip, fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL
    #     )
    #     fi_fsa_v2.fast_set_position_control(self.server_ip, target_position)


if __name__ == "__main__":
    motor = Motor()
    motor.get_motors_ip()
    motor.get_pvc()
    
    