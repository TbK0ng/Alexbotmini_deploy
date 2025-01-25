from fi_fsa import fi_fsa_v2
import time
import math
from scipy.interpolate import CubicSpline
import numpy as np

class MOTOR:
    server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']
    motors_num = 12
    
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

    def set_position_mode(self):
        """
        设置所有电机为位置控制模式
        """
        for i in range(len(motor.server_ip_list)):
            fi_fsa_v2.fast_set_enable(motor.server_ip_list[i])
            # fi_fsa_v2.fast_set_mode_of_operation(ip, fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL)
            fi_fsa_v2.fast_set_mode_of_operation(motor.server_ip_list[i], fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL_PD)
        time.sleep(0.1)
        # self.set_pd_imm()
        time.sleep(0.1)


    def set_position(self, target_position):
        for i in range(len(self.server_ip_list)):
            fi_fsa_v2.fast_set_pd_control(self.server_ip_list[i], target_position[i])

if __name__ == "__main__":
    # 目标位置，这里示例为全 0 位置，可根据实际修改
    motor=MOTOR()
    motor.get_motors_ip()
    motor.set_position_mode()
    # target_position = [20,20,20,20,20,20,20,20,20,20,20,20]
    target_position = [0,0,0,0,0,0,0,0,0,0,0,0]
    for i in range(len(motor.server_ip_list)):
        motor.set_position(target_position)
    
