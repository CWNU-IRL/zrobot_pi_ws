import argparse
import os
import sys
import torch


def export_to_onnx(jit_model_path, onnx_model_path, obs_dim, opset=13):
    model = torch.jit.load(jit_model_path, map_location="cpu")
    model.eval()

    dummy_input = torch.randn(1, obs_dim, dtype=torch.float32)

    torch.onnx.export(
        model,
        dummy_input,
        onnx_model_path,
        export_params=True,
        opset_version=opset,
        do_constant_folding=True,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={
            "obs": {0: "batch_size"},
            "action": {0: "batch_size"},
        },
    )
    print(f"[OK] ONNX exported to: {onnx_model_path}")


def check_onnx_model(onnx_model_path):
    try:
        import onnx
    except ImportError:
        print("[WARN] onnx 未安装，跳过结构校验。可执行: pip install onnx")
        return False

    model = onnx.load(onnx_model_path)
    onnx.checker.check_model(model)
    print("[OK] ONNX checker passed.")
    return True


def test_with_onnxruntime(onnx_model_path, obs_dim):
    try:
        import onnxruntime as ort
        import numpy as np
    except ImportError:
        print("[WARN] onnxruntime 或 numpy 未安装，跳过运行时推理测试。可执行: pip install onnxruntime numpy")
        return False

    sess = ort.InferenceSession(onnx_model_path, providers=["CPUExecutionProvider"])
    obs = np.random.randn(1, obs_dim).astype("float32")
    out = sess.run(["action"], {"obs": obs})[0]
    print(f"[OK] ONNXRuntime inference passed. output shape={out.shape}")
    return True


def compare_torch_onnx_outputs(
    jit_model_path,
    onnx_model_path,
    obs_dim,
    num_tests=10,
    atol=1e-5,
    rtol=1e-4,
    seed=42,
):
    """
    比较 TorchScript 与 ONNXRuntime 输出误差
    返回: (passed, stats_dict)
    """
    try:
        import onnxruntime as ort
        import numpy as np
    except ImportError:
        print("[WARN] 缺少 onnxruntime 或 numpy，跳过输出误差对比。")
        return False, None

    torch.manual_seed(seed)
    np.random.seed(seed)

    # load torch model
    torch_model = torch.jit.load(jit_model_path, map_location="cpu")
    torch_model.eval()

    # load onnx session
    sess = ort.InferenceSession(onnx_model_path, providers=["CPUExecutionProvider"])

    max_abs = 0.0
    max_rel = 0.0
    allclose_pass = True

    with torch.no_grad():
        for i in range(num_tests):
            obs_np = np.random.randn(1, obs_dim).astype(np.float32)
            obs_torch = torch.from_numpy(obs_np)

            torch_out = torch_model(obs_torch).cpu().numpy()
            onnx_out = sess.run(["action"], {"obs": obs_np})[0]

            abs_diff = np.max(np.abs(torch_out - onnx_out))
            # 避免除0
            denom = np.maximum(np.abs(torch_out), 1e-12)
            rel_diff = np.max(np.abs(torch_out - onnx_out) / denom)

            max_abs = max(max_abs, float(abs_diff))
            max_rel = max(max_rel, float(rel_diff))

            if not np.allclose(torch_out, onnx_out, atol=atol, rtol=rtol):
                allclose_pass = False
                print(f"[WARN] Test#{i} not allclose (atol={atol}, rtol={rtol})")

    stats = {
        "num_tests": num_tests,
        "max_abs_diff": max_abs,
        "max_rel_diff": max_rel,
        "atol": atol,
        "rtol": rtol,
        "allclose": allclose_pass,
    }

    if allclose_pass:
        print(
            f"[OK] Torch vs ONNX compare passed. "
            f"max_abs_diff={max_abs:.6e}, max_rel_diff={max_rel:.6e}"
        )
    else:
        print(
            f"[WARN] Torch vs ONNX compare has mismatch. "
            f"max_abs_diff={max_abs:.6e}, max_rel_diff={max_rel:.6e}"
        )

    return allclose_pass, stats


def main():
    parser = argparse.ArgumentParser(description="Convert TorchScript (.pt) to ONNX and validate.")
    parser.add_argument("--jit_model", type=str, required=True, help="Path to TorchScript model (policy_1.pt)")
    parser.add_argument("--onnx_model", type=str, required=True, help="Output ONNX file path")
    parser.add_argument("--obs_dim", type=int, required=True, help="Observation dimension")
    parser.add_argument("--opset", type=int, default=13, help="ONNX opset version (default: 13)")
    parser.add_argument("--skip_ort_test", action="store_true", help="Skip ONNX Runtime inference test")
    parser.add_argument("--skip_compare", action="store_true", help="Skip Torch vs ONNX output comparison")
    parser.add_argument("--num_tests", type=int, default=10, help="Number of random tests for output comparison")
    parser.add_argument("--atol", type=float, default=1e-5, help="Absolute tolerance for np.allclose")
    parser.add_argument("--rtol", type=float, default=1e-4, help="Relative tolerance for np.allclose")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducible comparison")
    args = parser.parse_args()

    if not os.path.isfile(args.jit_model):
        print(f"[ERR] jit_model 不存在: {args.jit_model}")
        sys.exit(1)

    os.makedirs(os.path.dirname(args.onnx_model) or ".", exist_ok=True)

    export_to_onnx(args.jit_model, args.onnx_model, args.obs_dim, args.opset)

    check_ok = check_onnx_model(args.onnx_model)

    ort_ok = True
    if not args.skip_ort_test:
        ort_ok = test_with_onnxruntime(args.onnx_model, args.obs_dim)

    compare_ok = True
    if not args.skip_compare:
        compare_ok, _ = compare_torch_onnx_outputs(
            args.jit_model,
            args.onnx_model,
            args.obs_dim,
            num_tests=args.num_tests,
            atol=args.atol,
            rtol=args.rtol,
            seed=args.seed,
        )

    if check_ok and ort_ok and compare_ok:
        print("[DONE] 导出、校验、推理与误差对比全部通过。")
    else:
        print("[DONE] 导出完成，但有校验/对比未通过或被跳过，请查看日志。")


if __name__ == "__main__":
    main()