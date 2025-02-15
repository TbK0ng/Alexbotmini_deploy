import numpy as np
import math
from fi_fsa import fi_fsa_v2
import time
from scipy.interpolate import CubicSpline
from motor.motors import MOTOR
# 常量定义
ANKLE_MOTOR5_OFFSET = math.pi - 0.046181  # 请替换为实际值
ANKLE_MOTOR6_OFFSET = math.pi - 0.046181  # 请替换为实际值
kps = np.array([180, 180, 180, 180, 30, 30, 180, 180, 180, 180, 30, 30], dtype=np.double)*0.4
kds = np.array([ 10, 12, 12, 10, 4, 4, 10,12, 12, 10, 4, 4,], dtype=np.double)*0.5
class AnkleJointModule:
    # 对于左腿而言
    @staticmethod
    def ankle_joint_fk(motor5, motor6):
        """
        输出结果为pitch角和roll角，单位为rad
        """
        result_n0 = np.zeros(2)
        result_n1 = np.zeros(2)
        f = np.zeros(2)

        error = 1
        i = 0
        while error > 1e-7:
            result_ik = AnkleJointModule.ankle_joint_ik(result_n0[0], result_n0[1])

            f[0] = result_ik[0] - motor5
            f[1] = result_ik[1] - motor6

            jacobian = AnkleJointModule.ankle_joint_ik_Jacobian(result_n0[0], result_n0[1])
            result_n1 = result_n0 - np.linalg.inv(jacobian) @ f

            error = np.linalg.norm(result_n0 - result_n1)

            result_n0 = result_n1
            i += 1

        print(f"迭代次数为: {i}")
        return (result_n1[0], result_n1[1])

    @staticmethod
    def ankle_joint_ik(pitch, roll):
        """
        输出结果为第五个，第六个关节电机的角度
        """
        matrix_roll = np.array([[1, 0, 0],
                                [0, np.cos(roll), -np.sin(roll)],
                                [0, np.sin(roll), np.cos(roll)]])

        matrix_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                                 [0, 1, 0],
                                 [-np.sin(pitch), 0, np.cos(pitch)]])

        matrix_rotation = matrix_roll @ matrix_pitch  # 机器人末端实际上是roll嵌套pitch

        foot_coordinate_left = np.array([-50, 46, 0])  # 从后往前看，脚左连杆中心在脚坐标系下的位置
        foot_coordinate_right = np.array([-50, -80, 0])  # 从后往前看，脚右连杆中心在脚坐标系下的位置
        motor5_coordinate = np.array([-0, -0, 90])  # ID为5的电机在小腿坐标系下的位置
        motor6_coordinate = np.array([-0, 0, 165])  # ID为6的电机在小腿坐标系下的位置
        pole5_length = 99.7 #并联杆5的长度
        pole6_length = 180 #并联杆6的长度
        R = 45  # 电机与连杆接触部分旋转半径

        foot_coordinate_left_ = matrix_rotation @ foot_coordinate_left
        foot_coordinate_right_ = matrix_rotation @ foot_coordinate_right

        # print(f"foot_coordinate_left_: {foot_coordinate_left_}")
        # print(f"foot_coordinate_right_: {foot_coordinate_right_}")

        cycle_para1 = np.array([motor5_coordinate[0], motor5_coordinate[2], R])
        # print(motor5_coordinate[1] - foot_coordinate_right_[1])
        cycle_para2 = np.array([foot_coordinate_right_[0], foot_coordinate_right_[2],
                                np.sqrt(pole5_length ** 2 - (motor5_coordinate[1] - foot_coordinate_right_[1]) ** 2)])

        cycle_para3 = np.array([motor6_coordinate[0], motor6_coordinate[2], R])
        cycle_para4 = np.array([foot_coordinate_left_[0], foot_coordinate_left_[2],
                                np.sqrt(pole6_length ** 2 - (motor6_coordinate[1] - foot_coordinate_left_[1]) ** 2)])

        result5 = AnkleJointModule.cycle_cycle_compute(cycle_para1, cycle_para2)
        result6 = AnkleJointModule.cycle_cycle_compute(cycle_para3, cycle_para4)

        x5, z5 = (result5[0], result5[1]) if result5[0] < result5[2] else (result5[2], result5[3])
        x6, z6 = (result6[0], result6[1]) if result6[0] < result6[2] else (result6[2], result6[3])

        if (x5 - motor5_coordinate[0]) >= 0 and (z5 - motor5_coordinate[2]) >= 0:
            motor5_angle = np.arcsin((z5 - motor5_coordinate[2]) / R)
        elif (x5 - motor5_coordinate[0]) < 0 and (z5 - motor5_coordinate[2]) >= 0:
            motor5_angle = np.arccos((x5 - motor5_coordinate[0]) / R)
        else:
            # print("x5 / R: ", x5 / R)
            motor5_angle = 2 * np.pi - np.arccos((x5 - motor5_coordinate[0]) / R)

        if (x6 - motor6_coordinate[0]) >= 0 and (z6 - motor6_coordinate[2]) >= 0:
            motor6_angle = np.arcsin((z6 - motor6_coordinate[2]) / R)
        elif (x6 - motor6_coordinate[0]) < 0 and (z6 - motor6_coordinate[2]) >= 0:
            motor6_angle = np.arccos((x6 - motor6_coordinate[0]) / R)
        else:
            motor6_angle = 2 * np.pi - np.arccos((x6 - motor6_coordinate[0]) / R)

        return (motor5_angle - ANKLE_MOTOR5_OFFSET, motor6_angle - ANKLE_MOTOR6_OFFSET)

    @staticmethod
    def ankle_joint_ik_Jacobian(pitch, roll):
        delta = 0.001
        Jacobian = np.zeros((2, 2))

        ik_plus_pitch = AnkleJointModule.ankle_joint_ik(pitch + delta, roll)
        ik_minus_pitch = AnkleJointModule.ankle_joint_ik(pitch - delta, roll)
        ik_plus_roll = AnkleJointModule.ankle_joint_ik(pitch, roll + delta)
        ik_minus_roll = AnkleJointModule.ankle_joint_ik(pitch, roll - delta)

        Jacobian[0, 0] = (ik_plus_pitch[0] - ik_minus_pitch[0]) / (2 * delta)
        Jacobian[0, 1] = (ik_plus_roll[0] - ik_minus_roll[0]) / (2 * delta)
        Jacobian[1, 0] = (ik_plus_pitch[1] - ik_minus_pitch[1]) / (2 * delta)
        Jacobian[1, 1] = (ik_plus_roll[1] - ik_minus_roll[1]) / (2 * delta)

        return Jacobian

    @staticmethod
    def cycle_cycle_compute(cycle1_para, cycle2_para):
        """
        圆的三个参数，分别为圆心x，y坐标，半径
        直线的三个参数，分别为Ax+By+C=0中的A，B，C
        """
        linear_para = np.zeros(3)
        linear_para[0] = 2 * (cycle2_para[0] - cycle1_para[0])
        linear_para[1] = 2 * (cycle2_para[1] - cycle1_para[1])
        linear_para[2] = (cycle1_para[0] ** 2 - cycle2_para[0] ** 2 +
                          cycle1_para[1] ** 2 - cycle2_para[1] ** 2 -
                          cycle1_para[2] ** 2 + cycle2_para[2] ** 2)

        return AnkleJointModule.cycle_linear_compute(cycle1_para, linear_para)

    @staticmethod
    def cycle_linear_compute(cycle_para, linear_para):
        """
        圆的三个参数，分别为圆心x，y坐标，半径
        直线的三个参数，分别为Ax+By+C=0中的A，B，C
        """
        if linear_para[1] == 0 or abs(linear_para[0] / linear_para[1]) > 1e6:
            # 考虑直线平行于y轴的情况
            x1 = x2 = -linear_para[2] / linear_para[0]
            y1 = cycle_para[1] + np.sqrt(cycle_para[2] ** 2 - (x1 - cycle_para[0]) ** 2)
            y2 = cycle_para[1] - np.sqrt(cycle_para[2] ** 2 - (x1 - cycle_para[0]) ** 2)
        else:
            # 一般情况
            k = -linear_para[0] / linear_para[1]
            b = -linear_para[2] / linear_para[1]

            # 一元二次方程的三个参数
            A = 1 + k ** 2
            B = 2 * k * (b - cycle_para[1]) - 2 * cycle_para[0]
            C = cycle_para[0] ** 2 - cycle_para[2] ** 2 + (b - cycle_para[1]) ** 2

            x1 = (-B + np.sqrt(B ** 2 - 4 * A * C)) / (2 * A)
            x2 = (-B / A) - x1
            y1 = k * x1 + b
            y2 = k * x2 + b

        return np.array([x1, y1, x2, y2])


if __name__ == "__main__":
    # 初始化电机对象
    motor = MOTOR()
    # 获取电机IP地址
    server_ip_list = motor.get_motors_ip()
    # 设置电机为位置控制模式
    motor.set_position_mode()
    # 设置PD参数
    motor.set_pd_imm_all(kps,kds)

    # 假设给定的pitch和roll角度（单位：rad）
    pitch = 0.4
    roll = 0
    print("pitch:", pitch*180/3.14)
    print("roll:", roll*180/3.14)
    # 计算第五个和第六个关节电机的角度
    motor5_angle, motor6_angle = AnkleJointModule.ankle_joint_ik(pitch, roll)
    print("motor5_angle:", motor5_angle*180/3.14)
    print("motor6_angle:", motor6_angle*180/3.14)
    # 假设前四个电机角度为0，后两个电机使用计算得到的角度
    target_position = [0, 0, 0, 0, motor5_angle*180/3.14, motor6_angle*180/3.14, 0, 0, 0, 0, 0, 0]
    # target_position = [0, 0, 0, 0, -0, 0, 0, 0, 0, 0, 0, 0]
    # 设置电机位置

    motor.set_pd_position(target_position)
