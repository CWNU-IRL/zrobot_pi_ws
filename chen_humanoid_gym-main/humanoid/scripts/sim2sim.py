# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2024 Beijing RobotEra TECHNOLOGY CO.,LTD. All rights reserved.


import math
import numpy as np
import mujoco, mujoco_viewer
from tqdm import tqdm
from collections import deque
from scipy.spatial.transform import Rotation as R
from humanoid import LEGGED_GYM_ROOT_DIR
from humanoid.envs import XBotLCfg,ZRobotCfg
import torch


class cmd:
    vx = 0.3
    vy = 0.0
    dyaw = 0.0


def quaternion_to_euler_array(quat):
    # Ensure quaternion is in the correct format [x, y, z, w]
    x, y, z, w = quat
    
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    
    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    # Returns roll, pitch, yaw in a NumPy array in radians
    return np.array([roll_x, pitch_y, yaw_z])

def get_obs(data):
    '''Extracts an observation from the mujoco data structure
    '''
    q = data.qpos.astype(np.double)
    dq = data.qvel.astype(np.double)
    quat = data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.double)
    r = R.from_quat(quat)
    v = r.apply(data.qvel[:3], inverse=True).astype(np.double)  # In the base frame
    omega = data.sensor('angular-velocity').data.astype(np.double)
    gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
    return (q, dq, quat, v, omega, gvec)

def pd_control(target_q, q, kp, target_dq, dq, kd):
    '''Calculates torques from position commands
    '''
    return (target_q - q) * kp + (target_dq - dq) * kd

def run_mujoco(policy, cfg):
    """
    Run the Mujoco simulation using the provided policy and configuration.

    Args:
        policy: The policy used for controlling the simulation.
        cfg: The configuration object containing simulation settings.

    Returns:
        None
    """
    model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)
    model.opt.timestep = cfg.sim_config.dt
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    viewer = mujoco_viewer.MujocoViewer(model, data)

    target_q = np.zeros((cfg.env.num_actions), dtype=np.double)
    action = np.zeros((cfg.env.num_actions), dtype=np.double)

    hist_obs = deque()
    for _ in range(cfg.env.frame_stack):
        hist_obs.append(np.zeros([1, cfg.env.num_single_obs], dtype=np.double))

    count_lowlevel = 0

    default_angle =np.zeros((cfg.env.num_actions),dtype=np.double)

    default_angle[0]=cfg.init_state.default_joint_angles['L_hip_roll_joint']
    default_angle[1]=cfg.init_state.default_joint_angles['L_hip_yaw_joint']
    default_angle[2]=cfg.init_state.default_joint_angles['L_hip_pitch_joint']
    default_angle[3]=cfg.init_state.default_joint_angles['L_knee_joint']
    default_angle[4]=cfg.init_state.default_joint_angles['L_foot_pitch_joint']
    default_angle[5]=cfg.init_state.default_joint_angles['L_foot_roll_joint']
    default_angle[6]=cfg.init_state.default_joint_angles['R_hip_roll_joint']
    default_angle[7]=cfg.init_state.default_joint_angles['R_hip_yaw_joint']
    default_angle[8]=cfg.init_state.default_joint_angles['R_hip_pitch_joint']
    default_angle[9]=cfg.init_state.default_joint_angles['R_knee_joint']
    default_angle[10]=cfg.init_state.default_joint_angles['R_foot_pitch_joint']
    default_angle[11]=cfg.init_state.default_joint_angles['R_foot_roll_joint']


    joint_names = ['L_hip_roll', 'L_hip_yaw', 'L_hip_pitch', 'L_knee', 'L_foot_pitch', 'L_foot_roll',
                   'R_hip_roll', 'R_hip_yaw', 'R_hip_pitch', 'R_knee', 'R_foot_pitch', 'R_foot_roll']

    # 打开监控日志文件
    import time
    log_path = f"sim2sim_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_path, 'w')
    log_file.write("step,joint,action,target_q,q_curr,tau\n")

    for _ in tqdm(range(int(cfg.sim_config.sim_duration / cfg.sim_config.dt)), desc="Simulating..."):

        # Obtain an observation
        q, dq, quat, v, omega, gvec = get_obs(data)
        q = q[-cfg.env.num_actions:]
        dq = dq[-cfg.env.num_actions:]

        # 1000hz -> 100hz
        if count_lowlevel % cfg.sim_config.decimation == 0:

            obs = np.zeros([1, cfg.env.num_single_obs], dtype=np.float32)
            eu_ang = quaternion_to_euler_array(quat)
            eu_ang[eu_ang > math.pi] -= 2 * math.pi

            obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * cfg.sim_config.dt  / 0.64)
            obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * cfg.sim_config.dt  / 0.64)
            # if count_lowlevel>3000:
            #     obs[0, 2] = (cmd.vx * 2) * cfg.normalization.obs_scales.lin_vel
            # else:
            #     obs[0, 2] = cmd.vx * cfg.normalization.obs_scales.lin_vel
            obs[0, 2] = cmd.vx * cfg.normalization.obs_scales.lin_vel
            obs[0, 3] = cmd.vy * cfg.normalization.obs_scales.lin_vel
            obs[0, 4] = cmd.dyaw * cfg.normalization.obs_scales.ang_vel
            obs[0, 5:17] = (q - default_angle) * cfg.normalization.obs_scales.dof_pos
            obs[0, 17:29] = dq * cfg.normalization.obs_scales.dof_vel
            obs[0, 29:41] = action
            obs[0, 41:44] = omega
            obs[0, 44:47] = eu_ang

            obs = np.clip(obs, -cfg.normalization.clip_observations, cfg.normalization.clip_observations)

            hist_obs.append(obs)
            hist_obs.popleft()

            policy_input = np.zeros([1, cfg.env.num_observations], dtype=np.float32)
            for i in range(cfg.env.frame_stack):
                policy_input[0, i * cfg.env.num_single_obs : (i + 1) * cfg.env.num_single_obs] = hist_obs[i][0, :]
            action[:] = policy(torch.tensor(policy_input))[0].detach().numpy()
            action = np.clip(action, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)

            if count_lowlevel>1000:
                as_scale = cfg.control.action_scale
                if isinstance(as_scale, dict):
                    # joint order: hip_roll, hip_yaw, hip_pitch, knee, foot_pitch, foot_roll (x2 L/R)
                    per_joint_scale = np.array([
                        as_scale['hip_roll'], as_scale['hip_yaw'], as_scale['hip_pitch'],
                        as_scale['knee'], as_scale['foot'], as_scale['foot'],
                        as_scale['hip_roll'], as_scale['hip_yaw'], as_scale['hip_pitch'],
                        as_scale['knee'], as_scale['foot'], as_scale['foot'],
                    ])
                    # for i, name in enumerate(joint_names):
                    #     for key, val in as_scale.items():
                    #         if key in name:
                    #             action_scaled[0, i] = action[0, i] * val
                    #             break
                    target_q = action * per_joint_scale + default_angle
                else:
                    target_q = action * as_scale + default_angle
            else:
                target_q = default_angle

            # target_q = action * cfg.control.action_scale


        target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
        # Generate PD control
        tau = pd_control(target_q, q, cfg.robot_config.kps,
                        target_dq, dq, cfg.robot_config.kds)  # Calc torques
        tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)  # Clamp torques
        data.ctrl = tau

        # --- 实时监控：每 10 步写入 CSV ---
        if count_lowlevel % 10 == 0:
            for j in range(12):
                log_file.write(f"{count_lowlevel},{joint_names[j]},{action[j]:.6f},{target_q[j]:.6f},{q[j]:.6f},{tau[j]:.6f}\n")

        mujoco.mj_step(model, data)
        viewer.render()
        count_lowlevel += 1

    viewer.close()
    log_file.close()
    print(f"日志已保存到: {log_path}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--load_model', type=str, default='/home/c112/codes/chen_human/humanoid26_6_8/humanoid-gym/logs/zrobot_ppo/exported/policies/policy_1.pt',
                        help='Run to load from.')
    parser.add_argument('--terrain', action='store_true', default='plane', help='terrain or plane')
    args = parser.parse_args()

    class Sim2simCfg(ZRobotCfg):

        class sim_config:
            if args.terrain:
                mujoco_model_path = f'{LEGGED_GYM_ROOT_DIR}/resources/robots/zrobot/mjcf/zrobot.xml'
            else:
                mujoco_model_path = f'{LEGGED_GYM_ROOT_DIR}/resources/robots/zrobot/mjcf/zrobot.xml'
            sim_duration = 60.0
            dt = 0.001
            decimation = 10

        class robot_config:
            kps = np.array([200, 120, 120, 180, 80, 80, 200, 120, 120, 180, 80, 80], dtype=np.double)
            kds = np.array([3, 1, 1, 3, 1, 1, 3, 1, 1, 3, 1, 1], dtype=np.double)
            # kps = np.array([120, 120, 120, 120, 60, 60, 120, 120, 120, 120, 60, 60], dtype=np.double)
            # kds = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], dtype=np.double)
            # kps = np.array([200, 200, 200, 200, 80, 80, 200, 200, 200, 200, 80, 80], dtype=np.double)
            # kds = np.array([8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8], dtype=np.double)
            # kps = np.array([200, 200, 200, 200, 80, 80, 200, 200, 200, 200, 80, 80], dtype=np.double)
            # kds = np.array([3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3], dtype=np.double)
            tau_limit = np.array([51, 51, 102, 102, 14.45, 14.45,
                                   51, 51, 102, 102, 14.45, 14.45], dtype=np.double)

    policy = torch.jit.load(args.load_model)
    run_mujoco(policy, Sim2simCfg())
