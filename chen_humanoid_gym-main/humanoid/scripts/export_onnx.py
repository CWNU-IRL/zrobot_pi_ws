"""将 TorchScript policy_1.pt 转换为 ONNX 格式"""
import torch
import sys
import os

# 用法: python scripts/export_onnx.py <policy.pt路径> [输出路径]
pt_path = sys.argv[1] if len(sys.argv) > 1 else 'logs/zrobot_ppo/exported/policies/policy_1.pt'
onnx_path = sys.argv[2] if len(sys.argv) > 2 else pt_path.replace('.pt', '.onnx')

if not os.path.exists(pt_path):
    print(f"错误: 文件不存在: {pt_path}")
    sys.exit(1)

# 加载 TorchScript 模型
model = torch.jit.load(pt_path)
model.eval()

# 输入: observation (1, 705)  — frame_stack=15 × num_single_obs=47
dummy_input = torch.randn(1, 705)

print(f"模型: {pt_path}")
print(f"输入形状: {dummy_input.shape}")

# 导出 ONNX
torch.onnx.export(
    model,
    dummy_input,
    onnx_path,
    export_params=True,
    opset_version=11,
    input_names=['obs'],
    output_names=['action'],
    dynamic_axes={
        'obs': {0: 'batch'},
        'action': {0: 'batch'},
    },
)
print(f"已导出: {onnx_path}")

# 验证
import onnx
onnx_model = onnx.load(onnx_path)
onnx.checker.check_model(onnx_model)
print("ONNX 模型验证通过")

# 推理验证
import onnxruntime as ort
ort_session = ort.InferenceSession(onnx_path)
with torch.no_grad():
    torch_out = model(dummy_input).numpy()

ort_input = {'obs': dummy_input.numpy()}
ort_out = ort_session.run(None, ort_input)[0]

max_diff = abs(torch_out - ort_out).max()
print(f"TorchScript vs ONNX 最大误差: {max_diff:.8f}")
