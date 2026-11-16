"""绘制 sim2sim 监控数据曲线（无需 pandas）"""
import csv
import matplotlib.pyplot as plt
import sys
from collections import defaultdict

csv_path = sys.argv[1] if len(sys.argv) > 1 else 'sim2sim_log_20260716_145022.csv'

# 按关节分组读取
data = defaultdict(lambda: {'step': [], 'target_q': [], 'q_curr': []})
with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        name = row['joint']
        data[name]['step'].append(float(row['step']))
        data[name]['target_q'].append(float(row['target_q']))
        data[name]['q_curr'].append(float(row['q_curr']))

joint_names = list(data.keys())
n = len(joint_names)

fig, axes = plt.subplots(n, 1, figsize=(14, 3 * n), sharex=True)
if n == 1:
    axes = [axes]

for ax, name in zip(axes, joint_names):
    d = data[name]
    ax.plot([s / 1000 for s in d['step']], d['target_q'], label='target_q', linewidth=0.8, alpha=0.8)
    ax.plot([s / 1000 for s in d['step']], d['q_curr'], label='q_curr', linewidth=0.8, alpha=0.8)
    ax.set_ylabel(f'{name}\n(rad)')
    ax.legend(loc='upper right', fontsize=7)
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel('time (s)')
fig.suptitle(f'Joint Position Tracking — {csv_path}', fontsize=12)
plt.tight_layout()
plt.savefig(csv_path.replace('.csv', '.png'), dpi=150)
plt.show()
print(f"图片已保存: {csv_path.replace('.csv', '.png')}")
