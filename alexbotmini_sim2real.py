import math
import time
import numpy as np
import torch
from tqdm import tqdm
from collections import deque

# import robot config
# from scipy.spatial.transform import Rotation as R
from config.custom.alexbotmini_config import alexbotminiCfg as cfg

# import module motor
from sim2real.module.motor.motors import MOTOR
from sim2real.module.utils import utils

# import module imu
from sim2real.module.imu.imu import IMU

######################################################################
# init robot & add robot config
# motor init
server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']
motors_num = 12
server_ip_list_test = []
motor = MOTOR()

# imu init
# If there are multiple USB devices here, replace them with the actual serial port names.
# sudo chmod a+rw /dev/ttyUSB0
port = "/dev/ttyUSB0"
baudrate = 115200
n = 1
imu = IMU()

# robot init
target_q = np.zeros((cfg.env.num_actions), dtype=np.double)
action = np.zeros((cfg.env.num_actions), dtype=np.double)

# 定义默认关节角度
default_joint_angles = {
    # = target angles [degree] when action = 0.0
    # in real, not in urdf
    'leftjoint1': -10,
    'leftjoint2': 0.,
    'leftjoint3': 0.,
    'leftjoint4': 18,
    'leftjoint5': -8,
    'leftjoint6': 8.,
    'rightjoint1': 10,
    'rightjoint2': 0.,
    'rightjoint3': 0.,
    'rightjoint4': -18,
    'rightjoint5': 8,
    'rightjoint6': 8,
}

class robot_config:
    kps = np.array([145, 115, 115, 145, 55, 55, 145, 115, 115, 145, 55, 55], dtype=np.double)
    kds = np.array([10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10], dtype=np.double)
    tau_limit = np.array([30, 20, 20, 30, 0, 0, 30, 20, 20, 30, 0, 0], dtype=np.double)
    target_q_limit = np.array([1, 0.35, 0.5, 1, 0, 0, 1, 0.35, 0.5, 1, 0, 0], dtype=np.double)
   
class cmd:
    # TODO: changed into joystick
    vx = 0.4
    vy = 0.0
    dyaw = 0.0


######################################################################
class robot:
    def __init__(self):
        self.init()

    def init(self):
        # motor init， FFTAIZ_fsa position control is base on current(force) control
        motor.get_motors_ip()
        motor.set_position_mode()
        motor.get_pvc()
        

        # 设置电机初始条件
        self.set_initial_angles()
        # motor.get_pvc()
        # imu init
        # imu.cmd_read(port, baudrate)

    def set_initial_angles(self):
        """
        设置电机的初始角度，使其运行在默认的关节角度状态
        """
        joint_angle_list = []
        for joint_name in ['leftjoint1', 'leftjoint2', 'leftjoint3', 'leftjoint4', 'leftjoint5', 'leftjoint6',
                           'rightjoint1', 'rightjoint2', 'rightjoint3', 'rightjoint4', 'rightjoint5', 'rightjoint6']:
            joint_angle_list.append(default_joint_angles[joint_name])
        joint_angles = np.array(joint_angle_list, dtype=np.float64)
        print("joint_angles的数据格式:", joint_angles.dtype)
        print("joint_angles的数据内容:", joint_angles)
        motor.set_position(joint_angles)
        

    def get_obs(self):
        motor.get_pvc()
        quat, gvec = imu.cmd_read(port, baudrate)
        q = motor.q
        dq = motor.dq
        print("quat value:", quat)
        print("gvec value:", gvec)
        return (q, dq, quat, gvec)

    def run_alexbotmini(self, policy, cfg):
        """
        Run the alexbotmini robot using the provided policy and configuration.
        Args:
            policy: The policy used for controlling the simulation.
            cfg: The configuration object containing simulation settings.
        Returns:
        None
        """
        hist_obs = deque()
        for _ in range(cfg.env.frame_stack):
            hist_obs.append(np.zeros([1, cfg.env.num_single_obs], dtype=np.double))

        target_q = np.zeros((cfg.env.num_actions), dtype=np.double)
        action = np.zeros((cfg.env.num_actions), dtype=np.double)

        count_lowlevel = 0
        while True:
            # Obtain an observation
            q, dq, quat, gvec = self.get_obs()
            q = q[-12:]
            dq = dq[-12:]
            q = np.array(q)
            dq = np.array(dq)

            if count_lowlevel % 10 == 0:
                obs = np.zeros([1, cfg.env.num_single_obs], dtype=np.float32)
                # 这里假设quat是符合要求的格式（比如一维数组等，如果不符合还需进一步处理，如前面提到的解包相关调整）
                eu_ang = utils.quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * 0.001 / 0.64)
                obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * 0.001 / 0.64)
                obs[0, 2] = cmd.vx * cfg.normalization.obs_scales.lin_vel
                obs[0, 3] = cmd.vy * cfg.normalization.obs_scales.lin_vel
                obs[0, 4] = cmd.dyaw * cfg.normalization.obs_scales.ang_vel
                obs[0, 5:17] = q * cfg.normalization.obs_scales.dof_pos
                obs[0, 17:29] = dq * cfg.normalization.obs_scales.dof_vel
                obs[0, 29:41] = action
                obs[0, 41:44] = gvec
                obs[0, 44:47] = eu_ang
                # Limit the data within the range from -cfg.normalization.clip_observations to cfg.normalization.clip_observations
                obs = np.clip(obs, -cfg.normalization.clip_observations, cfg.normalization.clip_observations)

                hist_obs.append(obs)
                hist_obs.popleft()

                policy_input = np.zeros([1, cfg.env.num_observations], dtype=np.float32)
                for i in range(cfg.env.frame_stack):
                    policy_input[0, i * cfg.env.num_single_obs:(i + 1) * cfg.env.num_single_obs] = hist_obs[i][0, :]
                policy_input = torch.tensor(policy_input)

                action = policy(policy_input)[0].detach().numpy()
                action = np.clip(action, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)
                target_q = action * cfg.control.action_scale
                # 并联脚
                # 以下是新增代码，保持特定电机的位置不变
                fixed_indices = [4, 5, 10, 11]  # 对应motor5、motor6、motor11、motor12的索引（假设索引从0开始）
                for index in fixed_indices:
                    target_q[index] = default_joint_angles[f'leftjoint{index + 1}'] if index < 6 else default_joint_angles[f'rightjoint{index - 5}']
                    target_q[index] = target_q[index] 

                target_q = np.clip(target_q, -robot_config.target_q_limit, robot_config.target_q_limit)  # rad
                # motor.set_position(target_q * 180 / 3.14)  # 设置电机位置时转换回角度

                print('target_q = :(rad)', target_q)  # rad
                print('target_q = :(deg)', target_q * 180 / 3.14)  # deg
                count_lowlevel += 1

            # target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
            # motor.set_position(target_q)
            # # Generate PD control
            # tau = utils.pd_control(target_q, q, robot_config.kps, target_dq, dq, robot_config.kds)  # Calc torques
            # tau = np.clip(tau, -robot_config.tau_limit, robot_config.tau_limit)  # Clamp torques


if __name__ == '__main__':
    device = torch.device("cpu")
    policy = torch.jit.load('sim2real/loadmodel/policy_1.pt', map_location=device)
    robot = robot()  # 创建robot类实例
    robot.run_alexbotmini(policy, cfg)
