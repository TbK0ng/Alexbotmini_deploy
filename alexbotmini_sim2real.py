import math
import numpy as np
import torch
from tqdm import tqdm
from collections import deque

# import robot config
from scipy.spatial.transform import Rotation as R
from config.custom.alexbotmini_config import alexbotminiCfg as cfg

# import module motor
from sim2real.module.motor.motors import Motor
from sim2real.module.utils import utils

# import module imu
from sim2real.module.imu.imu import imu


# init robot & add robot config
server_ip_list = ['192.168.137.101','192.168.137.102','192.168.137.103',
                  '192.168.137.104','192.168.137.105','192.168.137.106',
                  '192.168.137.107','192.168.137.108','192.168.137.109',
                  '192.168.137.110','192.168.137.111','192.168.137.112'] 
motors_num = 12 
server_ip_list_test =  []
motor = Motor()
target_q = np.zeros((cfg.env.num_actions), dtype=np.double)
action = np.zeros((cfg.env.num_actions), dtype=np.double)
data = []
######################################################################

class sim2real_robot_config():
    kps = np.array([200, 200, 350, 350, 15, 15, 200, 200, 350, 350, 15, 15], dtype=np.double)
    kds = np.array([10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10], dtype=np.double)
    tau_limit = 200. * np.ones(12, dtype=np.double)


class cmd:
    vx = 0.4
    vy = 0.0
    dyaw = 0.0
    #change into joystick

class robot:
    def init():
        # motor init
        motor.get_motors_ip()
        # imu init
        # If there are multiple USB devices here, 
        # replace them with the actual serial port names.
        port = "/dev/ttyUSB0"  
        baudrate = 115200
        imu = imu.cmd_read(port, baudrate)

        hist_obs = deque()
        for _ in range(cfg.env.frame_stack):
            hist_obs.append(np.zeros([1, cfg.env.num_single_obs], dtype=np.double))

    def get_obs(data):
        '''Extracts an observation from the real data structure
        '''
        motor.get_pvc()
        q = motor.q.astype(np.double)
        dq = motor.dq.astype(np.double)
        quat = data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.double)
        # r = R.from_quat(quat)
        # v = r.apply(motor.dq[:3], inverse=True).astype(np.double)  # In the base frame
        # omega = data.sensor('angular-velocity').data.astype(np.double)
        gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
        return (q, dq, quat, gvec)
    

    def run_alexbotmini(policy, cfg):
        """
        Run the alexbotmini robot using the provided policy and configuration.

        Args:
            policy: The policy used for controlling the simulation.
            cfg: The configuration object containing simulation settings.

        Returns:
            None
        """

    # model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)
    # model.opt.timestep = cfg.sim_config.dt
    # data = mujoco.MjData(model)
    # # mujoco.mj_step(model, data)
    # # viewer = mujoco_viewer.MujocoViewer(model, data)

    # target_q = np.zeros((cfg.env.num_actions), dtype=np.double)
    # action = np.zeros((cfg.env.num_actions), dtype=np.double)

    # hist_obs = deque()
    # for _ in range(cfg.env.frame_stack):
    #     hist_obs.append(np.zeros([1, cfg.env.num_single_obs], dtype=np.double))

        count_lowlevel = 0
        while(True):
            # Obtain an observation
            q, dq, quat, v, omega, gvec = robot.get_obs(data)
            q = q[-cfg.env.num_actions:]
            dq = dq[-cfg.env.num_actions:]
            if count_lowlevel % cfg.sim_config.decimation == 0:

                obs = np.zeros([1, cfg.env.num_single_obs], dtype=np.float32)
                eu_ang = utils.quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * cfg.sim_config.dt  / 0.64)
                obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * cfg.sim_config.dt  / 0.64)
                obs[0, 2] = cmd.vx * cfg.normalization.obs_scales.lin_vel
                obs[0, 3] = cmd.vy * cfg.normalization.obs_scales.lin_vel
                obs[0, 4] = cmd.dyaw * cfg.normalization.obs_scales.ang_vel
                obs[0, 5:17] = q * cfg.normalization.obs_scales.dof_pos
                obs[0, 17:29] = dq * cfg.normalization.obs_scales.dof_vel
                obs[0, 29:41] = action
                obs[0, 41:44] = omega
                obs[0, 44:47] = eu_ang

                obs = np.clip(obs, -cfg.normalization.clip_observations, cfg.normalization.clip_observations)

                robot.hist_obs.append(obs)
                robot.hist_obs.popleft()

                policy_input = np.zeros([1, cfg.env.num_observations], dtype=np.float32)
                for i in range(cfg.env.frame_stack):
                    policy_input[0, i * cfg.env.num_single_obs : (i + 1) * cfg.env.num_single_obs] = robot.hist_obs[i][0, :]
                action[:] = policy(torch.tensor(policy_input))[0].detach().numpy()
                action = np.clip(action, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)

                target_q = action * cfg.control.action_scale


            target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
            # Generate PD control
            tau = utils.pd_control(target_q, q, cfg.robot_config.kps,
                            target_dq, dq, cfg.robot_config.kds)  # Calc torques
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)  # Clamp torques

            # tell motors in mujoco is tau mode 
            # data.ctrl = tau
            
            # sleep 10ms
            count_lowlevel += 1

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--load_model', type=str, required=True,
                        help='Run to load from.')
    parser.add_argument('--terrain', action='store_true', help='terrain or plane')
    args = parser.parse_args()

    policy = torch.jit.load(args.load_model)
    robot.run_alexbotmini(policy, cfg)
# if __name__ == "__main__":
#     # If there are multiple USB devices here, 
#     # replace them with the actual serial port names.
#     port = "/dev/ttyUSB0"  
#     baudrate = 115200
#     imu = imu.cmd_read(port, baudrate)