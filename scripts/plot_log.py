#!/usr/bin/env python3
"""可视化运行日志 CSV：自动识别 sim2sim 长表 / locomotion 宽表格式。

用法:
  python3 plot_log.py <log.csv>                    # 弹窗显示
  python3 plot_log.py <log.csv> --save out_dir     # 保存 PNG（无显示器也可用）
  python3 plot_log.py <log.csv> --window 5 10      # 只看 t∈[5,10]s
"""
import argparse
import csv
import os
import sys
from collections import OrderedDict

import numpy as np

JOINTS = ['L_hip_roll', 'L_hip_yaw', 'L_hip_pitch', 'L_knee', 'L_foot_pitch', 'L_foot_roll',
          'R_hip_roll', 'R_hip_yaw', 'R_hip_pitch', 'R_knee', 'R_foot_pitch', 'R_foot_roll']


def grid_axes(fig, n, title):
    cols = 4
    rows = (n + cols - 1) // cols
    axes = []
    for i in range(n):
        ax = fig.add_subplot(rows, cols, i + 1)
        axes.append(ax)
    fig.suptitle(title)
    return axes


def apply_window(t, arrays, t0, t1):
    if t0 is None and t1 is None:
        return arrays
    mask = np.ones_like(t, dtype=bool)
    if t0 is not None:
        mask &= t >= t0
    if t1 is not None:
        mask &= t <= t1
    out = {'t': t[mask]}
    out.update({k: v[mask] for k, v in arrays.items()})
    return out


# ---------------- sim2sim 长表: step,joint,action,target_q,q_curr,tau ----------------
def load_sim2sim(path):
    data = OrderedDict((j, {'action': [], 'target_q': [], 'q_curr': [], 'tau': [], 'step': []})
                       for j in JOINTS)
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        assert header[:2] == ['step', 'joint'], f'意外的表头: {header}'
        for row in reader:
            if len(row) < 6:
                continue
            step, joint = int(row[0]), row[1]
            if joint not in data:
                continue
            d = data[joint]
            d['step'].append(step)
            for k, v in zip(('action', 'target_q', 'q_curr', 'tau'), row[2:6]):
                d[k].append(float(v))
    # step 是 1kHz 计数器，t = step * 0.001s
    t = np.array(data[JOINTS[0]]['step'], dtype=float) * 0.001
    arrays = {f'{j}/{k}': np.array(data[j][k], dtype=float)
              for j in JOINTS for k in ('action', 'target_q', 'q_curr', 'tau')}
    return t, arrays


def plot_sim2sim(t, arrays, save_dir, stem):
    import matplotlib.pyplot as plt

    # 1. 目标跟踪: target_q vs q_curr
    fig = plt.figure(figsize=(16, 9))
    axes = grid_axes(fig, len(JOINTS), f'{stem}: target_q vs q_curr')
    for ax, j in zip(axes, JOINTS):
        ax.plot(t, arrays[f'{j}/target_q'], label='target_q', lw=1)
        ax.plot(t, arrays[f'{j}/q_curr'], label='q_curr', lw=0.8, alpha=0.8)
        ax.set_title(j, fontsize=9)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    # 2. 策略原始输出 action（含 0~1s hold 期间的输出）
    fig2 = plt.figure(figsize=(16, 9))
    axes = grid_axes(fig2, len(JOINTS), f'{stem}: policy action (raw)')
    for ax, j in zip(axes, JOINTS):
        ax.plot(t, arrays[f'{j}/action'], lw=0.8)
        ax.set_title(j, fontsize=9)
        ax.grid(alpha=0.3)
    fig2.tight_layout(rect=[0, 0, 1, 0.96])

    # 3. 力矩
    fig3 = plt.figure(figsize=(16, 9))
    axes = grid_axes(fig3, len(JOINTS), f'{stem}: tau')
    for ax, j in zip(axes, JOINTS):
        ax.plot(t, arrays[f'{j}/tau'], lw=0.8, color='tab:red')
        ax.set_title(j, fontsize=9)
        ax.grid(alpha=0.3)
    fig3.tight_layout(rect=[0, 0, 1, 0.96])

    if save_dir:
        for fig_i, name in ((fig, 'tracking'), (fig2, 'action'), (fig3, 'tau')):
            fig_i.savefig(os.path.join(save_dir, f'{stem}_{name}.png'), dpi=120)
        plt.close('all')
    else:
        plt.show()


# ---------------- locomotion 宽表: seq,t,obs_*,act_policy_*,act_applied_*,target_q_*,q_curr_*,tau_motor_* ----------------
def load_locomotion(path):
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = {name: i for i, name in enumerate(header)}
        rows = np.array([[float(v) for v in row] for row in reader if row], dtype=float)
    t = rows[:, cols['t']] if 't' in cols else rows[:, cols['seq']] * 0.01
    return t, cols, rows


def plot_locomotion(t, cols, rows, save_dir, stem):
    import matplotlib.pyplot as plt

    def col(name):
        return rows[:, cols[name]]

    # 1. 目标跟踪 + 施加动作对比
    fig = plt.figure(figsize=(16, 9))
    axes = grid_axes(fig, len(JOINTS), f'{stem}: target_q vs q_curr')
    for i, (ax, j) in enumerate(zip(axes, JOINTS)):
        ax.plot(t, col(f'target_q_{i}'), label='target_q', lw=1)
        ax.plot(t, col(f'q_curr_{i}'), label='q_curr', lw=0.8, alpha=0.8)
        ax.set_title(j, fontsize=9)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    # 2. obs 关键段：相位/指令/姿态/角速度/动作历史
    fig2 = plt.figure(figsize=(16, 10))
    ax = fig2.add_subplot(4, 2, 1)
    ax.plot(t, col('obs_0'), label='sin(phase)')
    ax.plot(t, col('obs_1'), label='cos(phase)')
    ax.set_title('gait phase'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 2)
    for k, name in enumerate(('vx*2', 'vy*2', 'wz')):
        ax.plot(t, col(f'obs_{2 + k}'), label=name)
    ax.set_title('commands (scaled)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 3)
    for k, name in enumerate(('roll', 'pitch', 'yaw_rel')):
        ax.plot(t, np.degrees(col(f'obs_{44 + k}')), label=name)
    ax.set_title('euler [deg]'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 4)
    for k, name in enumerate(('wx', 'wy', 'wz')):
        ax.plot(t, col(f'obs_{41 + k}'), label=name)
    ax.set_title('angular velocity [rad/s]'); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 5)
    for i, j in enumerate(JOINTS):
        ax.plot(t, col(f'obs_{5 + i}'), lw=0.7, label=j, alpha=0.8)
    ax.set_title('obs dof_pos (q-default)'); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 6)
    for i, j in enumerate(JOINTS):
        ax.plot(t, col(f'obs_{17 + i}'), lw=0.7, alpha=0.8)
    ax.set_title('obs dof_vel (scaled)'); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 7)
    for i, j in enumerate(JOINTS):
        ax.plot(t, col(f'obs_{29 + i}'), lw=0.7, alpha=0.8)
    ax.set_title('obs action history'); ax.grid(alpha=0.3)
    ax = fig2.add_subplot(4, 2, 8)
    for i, j in enumerate(JOINTS):
        ax.plot(t, col(f'act_policy_{i}'), lw=0.7, alpha=0.8)
    ax.set_title('act_policy (raw output)'); ax.grid(alpha=0.3)
    fig2.suptitle(f'{stem}: observations')
    fig2.tight_layout(rect=[0, 0, 1, 0.96])

    # 3. 滤波前后动作对比（低通+渐入效果）
    fig3 = plt.figure(figsize=(16, 9))
    axes = grid_axes(fig3, len(JOINTS), f'{stem}: act_policy vs act_applied')
    for i, (ax, j) in enumerate(zip(axes, JOINTS)):
        ax.plot(t, col(f'act_policy_{i}'), label='policy', lw=0.8, alpha=0.7)
        ax.plot(t, col(f'act_applied_{i}'), label='applied', lw=0.8)
        ax.set_title(j, fontsize=9)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig3.tight_layout(rect=[0, 0, 1, 0.96])

    # 4. 力矩
    fig4 = plt.figure(figsize=(16, 9))
    axes = grid_axes(fig4, len(JOINTS), f'{stem}: tau_motor')
    for i, (ax, j) in enumerate(zip(axes, JOINTS)):
        ax.plot(t, col(f'tau_motor_{i}'), lw=0.8, color='tab:red')
        ax.set_title(j, fontsize=9)
        ax.grid(alpha=0.3)
    fig4.tight_layout(rect=[0, 0, 1, 0.96])

    if save_dir:
        for fig_i, name in ((fig, 'tracking'), (fig2, 'obs'), (fig3, 'action'), (fig4, 'tau')):
            fig_i.savefig(os.path.join(save_dir, f'{stem}_{name}.png'), dpi=120)
        plt.close('all')
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='可视化 sim2sim / locomotion 日志')
    parser.add_argument('csv', help='日志 CSV 路径')
    parser.add_argument('--save', metavar='DIR', default=None, help='保存 PNG 到目录而非弹窗')
    parser.add_argument('--window', nargs=2, type=float, metavar=('T0', 'T1'), default=None,
                        help='时间窗（秒）')
    args = parser.parse_args()

    if args.save:
        import matplotlib
        matplotlib.use('Agg')
        os.makedirs(args.save, exist_ok=True)

    with open(args.csv) as f:
        header = f.readline().strip().split(',')
    stem = os.path.splitext(os.path.basename(args.csv))[0]

    if header[:2] == ['step', 'joint']:
        t, arrays = load_sim2sim(args.csv)
        if args.window:
            mask = (t >= args.window[0]) & (t <= args.window[1])
            t = t[mask]
            arrays = {k: v[mask] for k, v in arrays.items()}
        plot_sim2sim(t, arrays, args.save, stem)
    elif header[0] == 'seq' and 'obs_0' in header:
        t, cols, rows = load_locomotion(args.csv)
        if args.window:
            mask = (t >= args.window[0]) & (t <= args.window[1])
            t = t[mask]
            rows = rows[mask]
        plot_locomotion(t, cols, rows, args.save, stem)
    else:
        sys.exit(f'无法识别的日志格式: {header[:5]}')

    if args.save:
        print(f'已保存 PNG 到 {args.save}/')


if __name__ == '__main__':
    main()
