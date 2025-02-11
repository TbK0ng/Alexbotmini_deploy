import numpy as np
from fi_fsa import fi_fsa_v2
fi_fsa_v2.fsa_flag_debug = False
import time
from scipy.interpolate import CubicSpline
import struct

import socket
fsa_socket = fi_fsa_v2.fsa_socket
fsa_port_fast = fi_fsa_v2.fsa_port_fast
kps = np.array([120, 90, 90, 120, 30, 30, 120, 90, 90, 120, 30, 30], dtype=np.double)
kds = np.array([10, 8, 8, 10, 2.5, 2.5, 10, 8, 8, 10, 2.5, 2.5,], dtype=np.double)

def fast_get_pvc_group(server_ips):
    # send request
    for i in range(len(server_ips)):
        server_ip = server_ips[i]
        tx_messages = struct.pack(">B", 0x1A)
        try:
            fsa_socket.sendto(tx_messages, (server_ip, fsa_port_fast))
        except Exception as e:
            print('send error', server_ip, e)
    # get response
    response = {}
    for i in range(len(server_ips)):
        server_ip = server_ips[i]
        response.update({server_ip: {}})
    for i in range(len(server_ips)):
        server_ip = server_ips[i]
        try:
            data, address = fsa_socket.recvfrom(1024)
            recv_ip, recv_port = address
            if recv_ip not in server_ips:
                continue
            # print(recv_ip)
            response.get(recv_ip).update({"data": data})
        except socket.timeout:  # fail after 1 second of no activity
            print('recv timeout', server_ip)
            continue
    # data parse
    feedbacks = []
    positions = []
    velocitys = []
    currents = []
    for i in range(len(server_ips)):
        server_ip = server_ips[i]
        data = response.get(server_ip).get("data")
        if data is None:
            # feedback, position, velocity, current = None, None, None, None
            feedback, position, velocity, current = 0, 0, 0, 0
        else:
            feedback, position, velocity, current = struct.unpack(
                ">Bfff", data[0 : 1 + 4 + 4 + 4]
            )
        feedbacks.append(feedback)
        positions.append(position)
        velocitys.append(velocity)
        currents.append(current)
    return positions, velocitys, currents

class MOTOR:
    def __init__(self):
        self.server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                        '192.168.137.104', '192.168.137.105', '192.168.137.106',
                        '192.168.137.107', '192.168.137.108', '192.168.137.109',
                        '192.168.137.110', '192.168.137.111', '192.168.137.112'   
                        ]
        self.motors_num = len(self.server_ip_list)
        print("Motor Num:", self.motors_num)
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
            print("Server IP List(len):", len(server_ip_list_test))
            print("Server IP List:", server_ip_list_test)
            print("Motors Num:", len(server_ip_list_test))
            raise ValueError('Lost connection of motors. We only find',len(server_ip_list_test),'motor(s)')
        return server_ip_list_test
    
    def set_pd_imm_all(self, kps, kds):
        for i in range(len(self.server_ip_list)):
            _ = fi_fsa_v2.fast_set_pd_imm(self.server_ip_list[i], kps[i], kds[i])

    def get_pvc(self):#(degree)
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        """
        self.q = []
        self.dq = []
        t0=time.time()
        position, velocity, current = fast_get_pvc_group(self.server_ip_list)
        print("Get PVC Time:", time.time()-t0)
        self.q = position
        self.dq = velocity
   
        return self.q, self.dq
    
    def set_position_mode(self):
        """
        设置所有电机为位置控制模式
        """
        for i in range(len(self.server_ip_list)):
            fi_fsa_v2.fast_set_enable(self.server_ip_list[i])
            # fi_fsa_v2.fast_set_mode_of_operation(ip, fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL)
            fi_fsa_v2.fast_set_mode_of_operation(self.server_ip_list[i], fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL_PD)
        time.sleep(0.1)
        # self.set_pd_imm()
        time.sleep(0.1)


    def set_position(self, target_position):
        for i in range(len(self.server_ip_list)):
            fi_fsa_v2.fast_set_pd_control(self.server_ip_list[i], target_position[i])


if __name__ == "__main__":
    motor = MOTOR()
    server_ip_list = motor.get_motors_ip()
    motor.set_position_mode()

    # 假设要设置的目标位置，这里示例为全0位置，你可以替换为实际需要的目标位置数组
    while 1:
        # target_position1 = [10,10,10,10,10,10,10,10,10,10,10,10]
        target_position2 = [0,0,0,0,0,0,0,0,0,0,0,0]
        # motor.set_position(target_position1)
        motor.set_position(target_position2)

    
