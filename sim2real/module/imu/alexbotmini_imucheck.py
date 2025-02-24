import math
import time
import numpy as np
import torch
from tqdm import tqdm
from collections import deque
import matplotlib.pyplot as plt
from scipy.fft import fft

# 假设以下模块已正确定义
from sim2real.module.motor.motors import MOTOR
from sim2real.module.utils import utils
from sim2real.module.imu.imu import IMU


class robot_config:
    kps = np.array([180, 200, 120, 180, 120, 120, 180, 200, 120, 180, 120, 120], dtype=np.double) * 0.4
    kds = np.array([10, 8, 8, 10, 6, 6, 10, 8, 8, 10, 6, 6], dtype=np.double) * 0.7
    target_q_limit = np.array([60, 18, 18, 60, 45, 45, 60, 18, 18, 60, 45, 45], dtype=np.double)
    target_q_limit = np.deg2rad(target_q_limit)
    initial_position = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.double)
    default_joint_angles = np.array([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8], dtype=np.double)
    num_actions = 12
    action_scale = 0.25

    class normalization:
        class obs_scales:
            lin_vel = 2.
            ang_vel = 1.
            dof_pos = 1.
            dof_vel = 0.05
            quat = 1.
            height_measurements = 5.0

        clip_observations = 18.
        clip_actions = 18.

    class env:
        frame_stack = 15
        c_frame_stack = 3
        num_single_obs = 47
        num_observations = int(frame_stack * num_single_obs)
        single_num_privileged_obs = 73
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)
        num_actions = 12
        num_envs = 2048
        episode_length_s = 24
        use_ref_actions = False


server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']
motors_num = 12
server_ip_list_test = []
motor = MOTOR()

__import__('os').system('sudo chmod a+rw /dev/ttyUSB0')
port = "/dev/ttyUSB0"
baudrate = 115200
n = 1
imu = IMU(port, baudrate)

target_q = np.zeros((robot_config.num_actions), dtype=np.double)
action = np.zeros((robot_config.num_actions), dtype=np.double)


class cmd:
    vx = 0.0
    vy = 0.0
    dyaw = 0.0


class robot:
    def __init__(self):
        self.init()
        self.sim_actions = []
        self.real_actions = []
        self.quat_history = []
        self.omega_history = []
        self.timestamps = []
        self.euler_history = []
        self.start_time = time.time()

    def init(self):
        motor.get_motors_ip()
        motor.set_position_mode()
        motor.get_pvc()
        motor.set_pd_imm_all(robot_config.kps, robot_config.kds)
        motor.set_position(robot_config.initial_position)
        time.sleep(1)
        motor.set_position(robot_config.default_joint_angles)
        time.sleep(1)

    def get_obs(self):
        motor.get_pvc()
        quat, omega = imu.cmd_read()
        q = motor.q
        dq = motor.dq
        return q, dq, quat, omega

    def run_alexbotmini(self, policy):
        hist_obs = deque()
        for _ in range(robot_config.env.frame_stack):
            hist_obs.append(np.zeros([1, robot_config.env.num_single_obs], dtype=np.double))

        target_q = np.zeros((robot_config.num_actions), dtype=np.double)
        action = np.zeros((robot_config.num_actions), dtype=np.double)
        count_lowlevel = 0
        default_angle = np.zeros((robot_config.num_actions), dtype=np.double)
        default_angle[:] = robot_config.default_joint_angles
        default_angle_rad = np.deg2rad(default_angle)
        dt = 0.01
        policy_input = np.zeros([1, robot_config.env.num_observations], dtype=np.float32)

        while True:
            start_time = time.time()
            q, dq, quat, omega = self.get_obs()
            q = q[-12:]
            dq = dq[-12:]
            q = np.array(q)
            dq = np.array(dq)
            q = np.deg2rad(q)
            dq = np.deg2rad(dq)
            omega = np.deg2rad(omega)

            if count_lowlevel % 1 == 0:
                obs = np.zeros([1, robot_config.env.num_single_obs], dtype=np.float32)
                eu_ang = utils.quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * dt / 0.64)
                obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * dt / 0.64)
                obs[0, 2] = cmd.vx * robot_config.normalization.obs_scales.lin_vel
                obs[0, 3] = cmd.vy * robot_config.normalization.obs_scales.lin_vel
                obs[0, 4] = cmd.dyaw * robot_config.normalization.obs_scales.ang_vel
                obs[0, 5:17] = (q - default_angle_rad) * robot_config.normalization.obs_scales.dof_pos
                obs[0, 17:29] = dq * robot_config.normalization.obs_scales.dof_vel
                obs[0, 29:41] = action
                obs[0, 41:44] = omega
                obs[0, 44:47] = eu_ang
                obs = np.clip(obs, -robot_config.normalization.clip_observations,
                              robot_config.normalization.clip_observations)

                hist_obs.append(obs)
                hist_obs.popleft()

                for i in range(robot_config.env.frame_stack):
                    policy_input[0, i * robot_config.env.num_single_obs:(i + 1) * robot_config.env.num_single_obs] = \
                        hist_obs[i][0, :]
                policy_input_tensor = torch.from_numpy(policy_input)
                action = policy(policy_input_tensor)[0].detach().numpy()
                action = np.clip(action, -robot_config.normalization.clip_actions,
                                 robot_config.normalization.clip_actions)
                target_q = action * robot_config.action_scale + default_angle_rad
                target_q = np.clip(target_q, -robot_config.target_q_limit, robot_config.target_q_limit)
                # motor.set_position(np.rad2deg(target_q))

                end_time = time.time()
                execution_time = end_time - start_time
                if execution_time < dt:
                    time.sleep(dt - execution_time)

                self.plot_imu_data(quat, omega, count_lowlevel)

            count_lowlevel += 1

    def plot_imu_data(self, quat, omega, count):
        self.timestamps.append(time.time() - self.start_time)
        self.quat_history.append(quat)
        self.omega_history.append(omega)
        euler = utils.quaternion_to_euler_array(quat)
        self.euler_history.append(euler)

    def save_final_plots(self):
        quats = np.array(self.quat_history)
        omegas = np.array(self.omega_history)
        times = np.array(self.timestamps)
        eulers = np.array(self.euler_history)

        plt.figure(figsize=(12, 16))

        plt.subplot(4, 1, 1)
        for i in range(4):
            plt.plot(times, quats[:, i], label=f'q{i}')
        plt.title('Quaternion History')
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.legend()
        plt.grid(True)

        plt.subplot(4, 1, 2)
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            plt.plot(times, omegas[:, i], label=f'Omega {labels[i]}')
        plt.title('Angular Velocity History')
        plt.xlabel('Time (s)')
        plt.ylabel('rad/s')
        plt.legend()
        plt.grid(True)

        plt.subplot(4, 1, 3)
        for i in range(3):
            N = len(omegas[:, i])
            T = 0.01
            yf = fft(omegas[:, i])
            xf = np.linspace(0.0, 1.0 / (2.0 * T), N // 2)
            plt.plot(xf, 2.0 / N * np.abs(yf[0:N // 2]), label=f'Freq {labels[i]}')
        plt.title('Frequency Spectrum')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.legend()
        plt.grid(True)

        plt.subplot(4, 1, 4)
        euler_labels = ['Roll', 'Pitch', 'Yaw']
        eulers_deg = np.rad2deg(eulers)  # 转换单位
        for i in range(3):
            plt.plot(times, eulers_deg[:, i], label=euler_labels[i])
        plt.title('Euler Angles History')
        plt.xlabel('Time (s)')
        plt.ylabel('Degrees')  # 修改标签
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig('imu_final_plot.png')
        plt.close()
        print("IMU数据图表已保存为 imu_final_plot.png")

    def save_raw_data(self):
        data = np.hstack((
            np.array(self.timestamps)[:, None],
            np.array(self.quat_history),
            np.array(self.omega_history),
            np.array(self.euler_history)
        ))
        header = "timestamp,q0,q1,q2,q3,omega_x,omega_y,omega_z,roll,pitch,yaw"
        np.savetxt("imu_raw_data.csv", data, delimiter=",", header=header)
        print("原始数据已保存为 imu_raw_data.csv")


if __name__ == '__main__':
    device = torch.device("cpu")
    policy = torch.jit.load('sim2real/loadmodel/test07_20250219/policy_1.pt', map_location=device)
    robot = robot()
    try:
        robot.run_alexbotmini(policy)
    except KeyboardInterrupt:
        print("程序被手动中断。")
    finally:
        robot.save_final_plots()
        robot.save_raw_data()
