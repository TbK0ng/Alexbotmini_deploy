"""
Sim2Real Prechecks:
1. Ensure that the robot is in the home position (all joints at 0 degrees/default position)
2. Ensure that the motors are connected to the robot and working properly
3. Ensure that the IMU is connected to the robot and working properly(sudo chmod 666 /dev/ttyUSB0)
4. Ensure that the robot is not in a collision state
5. Ensure that the robot is not in a physical damage state

Open loop control:(policy is created in sim2sim in alexbormini_gait)
    Execute pre-recorded action sequence in open-loop mode
    This implementation:
    1. Loads action sequence from file
    2. Converts actions to target joint positions
    3. Sends commands to motors at 100Hz frequency
    4. Cycles through the action sequence indefinitely
    Designed for the Alexbotmini robot.
"""
import time
import numpy as np
import torch
from collections import deque
from sim2real.module.motor.motors import MOTOR  # Motor control module
from sim2real.module.imu.imu import IMU         # IMU sensor module
from sim2real.module.utils import utils        # Utility functions

# Robot configuration parameters
class robot_config:
    # PID control parameters (proportional and derivative gains scaled)
    kps = np.array([180, 180, 120, 180, 120, 120, 180, 180, 120, 180, 120, 120], dtype=np.double)*0.35
    kds = np.array([10, 8, 8, 10, 5, 5, 10, 8, 8, 10, 5, 5], dtype=np.double)*0.8
    
    # Joint angle limits in radians (converted from degrees)
    target_q_limit = np.deg2rad(np.array([90, 36, 36, 90, 45, 45, 90, 36, 36 , 90, 45, 45], dtype=np.double))
    
    # Default joint positions in degrees (home position)
    default_joint_angles = np.array([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8], dtype=np.double)
    
    num_actions = 12             # Number of robot joints
    action_scale = 0.25          # Scaling factor for action values
    
    # Observation normalization parameters
    class normalization:
        class obs_scales: 
            lin_vel = 2.0        # Linear velocity scale
            ang_vel = 1.0        # Angular velocity scale
            dof_pos = 1.0        # Joint position scale
            dof_vel = 0.05       # Joint velocity scale
        clip_observations = 18.0 # Observation clipping value
        clip_actions = 18.0      # Action clipping value

    # Environment configuration
    class env:
        frame_stack = 15         # Number of stacked frames for temporal information
        num_single_obs = 47      # Dimension of single observation
        num_observations = int(frame_stack * num_single_obs)  # Total observation dimension

# Initialize hardware interfaces
motor = MOTOR()                  # Create motor controller instance
imu = IMU("/dev/ttyUSB0", 115200) # Create IMU interface (requires proper serial port permissions)

# Placeholder for velocity commands (currently not used)
class cmd:
    vx = 0.1   # Linear velocity x (m/s)
    vy = 0.0   # Linear velocity y (m/s)
    dyaw = 0.0 # Angular velocity yaw (rad/s)

# Main robot control class
class robot:
    def __init__(self):
        self.init()  # Initialize hardware during instantiation
        
    def init(self):
        """Initialize motor controllers and set default position"""
        motor.set_position_mode()  # Set motors to position control mode
        motor.set_pd_imm_all(robot_config.kps, robot_config.kds)  # Set PID parameters
        motor.set_position(robot_config.default_joint_angles)  # Move to default position
        time.sleep(1)  # Wait for motors to stabilize

    def get_obs(self):
        """Get current observation from sensors"""
        motor.get_pvc()  # Update motor positions/velocities/currents
        quat, gvec = imu.cmd_read()  # Read IMU data (quaternion and gravity vector)
        return motor.q[-12:], motor.dq[-12:], quat, gvec  # Return last 12 joints' data

    def run_alexbotmini(self):
        """
        Execute pre-recorded action sequence in open-loop mode
        This implementation:
        1. Loads action sequence from file
        2. Converts actions to target joint positions
        3. Sends commands to motors at 100Hz frequency
        4. Cycles through the action sequence indefinitely
        """
        # Load pre-recorded action sequence (saved as PyTorch tensor)
        loaded_actions = torch.load(
            '/home/alexhuge/Documents/GitHub/Alexbotmini_deploy/all_actions_mujoco.pt',
            map_location=torch.device('cpu')
        ).numpy()  # Convert to numpy array
        
        pt = 0  # Action sequence pointer
        dt = 0.01  # Control period (100Hz frequency)
        
        while True:
            start_time = time.time()
            
            # Get next action (loop using modulo when reaching end)
            action = loaded_actions[pt % len(loaded_actions)]
            pt += 1
            
            # Convert action to target joint angles
            default_angle = np.deg2rad(robot_config.default_joint_angles)
            target_q = np.clip(
                action * robot_config.action_scale + default_angle,
                -robot_config.target_q_limit,
                robot_config.target_q_limit
            )
            
            # Send position command to motors (convert radians to degrees)
            motor.set_position(np.rad2deg(target_q))
            
            # Maintain control frequency
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
            print(f"Executing action {pt}/{len(loaded_actions)}")

# Main entry point
if __name__ == '__main__':
    robot().run_alexbotmini()  # Create robot instance and start execution