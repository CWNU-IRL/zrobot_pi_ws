/**
 * @file test_ankle_ik.cpp
 * @brief 踝关节 IK 独立验证程序 — 脱离 ROS 框架，直接编译运行
 *
 * 编译: g++ -std=c++17 -O2 -o test_ankle_ik test_ankle_ik.cpp && ./test_ankle_ik
 * 或:  cd build && cmake .. && make test_ankle_ik && ./test_ankle_ik
 */

#include "zrobot_deploy/paralleljoint_z1.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

static int tests_passed = 0;
static int tests_failed = 0;

#define CHECK(cond, fmt, ...)                                          \
    do {                                                               \
        if (cond) {                                                    \
            ++tests_passed;                                            \
            printf("  [PASS] " fmt "\n", ##__VA_ARGS__);               \
        } else {                                                       \
            ++tests_failed;                                            \
            printf("  [FAIL] " fmt "\n", ##__VA_ARGS__);               \
        }                                                              \
    } while (0)

int main()
{
    ParallelAnkle ankle;

    printf("=== 踝关节 IK 验证测试 ===\n\n");

    // ── 测试 1: 零位 (pitch=0, roll=0) ──
    // 策略默认姿态: L_foot_pitch=0.4, L_foot_roll=0.0, R_foot_pitch=-0.4, R_foot_roll=0.0
    // 此处测零位作为基准
    {
        const auto ik = ankle.ik(0.0, 0.0);
        printf("--- 测试 1: 零位 (pitch=0, roll=0) ---\n");
        CHECK(ik.error_state == 0, "error_state=%d (期望 0)", ik.error_state);
        CHECK(std::isfinite(ik.theta1), "theta1=%.6f (有限的)", ik.theta1);
        CHECK(std::isfinite(ik.theta2), "theta2=%.6f (有限的)", ik.theta2);
        printf("    theta1=%.6f rad (%.2f°)\n", ik.theta1, ik.theta1 * 180.0 / M_PI);
        printf("    theta2=%.6f rad (%.2f°)\n", ik.theta2, ik.theta2 * 180.0 / M_PI);
        printf("\n");
    }

    // ── 测试 2: 默认姿态 — 左踝 (pitch=0.4, roll=0.0) ──
    {
        const auto ik = ankle.ik(0.4, 0.0);
        printf("--- 测试 2: 默认姿态左脚 (pitch=0.4, roll=0.0) ---\n");
        CHECK(ik.error_state == 0, "error_state=%d (期望 0)", ik.error_state);
        CHECK(std::isfinite(ik.theta1), "theta1=%.6f", ik.theta1);
        CHECK(std::isfinite(ik.theta2), "theta2=%.6f", ik.theta2);
        printf("    theta1=%.6f rad (%.2f°)\n", ik.theta1, ik.theta1 * 180.0 / M_PI);
        printf("    theta2=%.6f rad (%.2f°)\n", ik.theta2, ik.theta2 * 180.0 / M_PI);
        printf("\n");
    }

    // ── 测试 3: 默认姿态 — 右踝 (pitch=-0.4, roll=0.0) ──
    {
        const auto ik = ankle.ik(-0.4, 0.0);
        printf("--- 测试 3: 默认姿态右脚 (pitch=-0.4, roll=0.0) ---\n");
        CHECK(std::isfinite(ik.theta1), "theta1=%.6f", ik.theta1);
        CHECK(std::isfinite(ik.theta2), "theta2=%.6f", ik.theta2);
        printf("    theta1=%.6f rad (%.2f°)\n", ik.theta1, ik.theta1 * 180.0 / M_PI);
        printf("    theta2=%.6f rad (%.2f°)\n", ik.theta2, ik.theta2 * 180.0 / M_PI);
        printf("\n");
    }

    // ── 测试 4: 极限位置 (pitch=0.35, roll=-0.2) — 原始 main() 注释中的测试用例 ──
    {
        const auto ik = ankle.ik(0.35, -0.2);
        printf("--- 测试 4: 极限位置 (pitch=0.35, roll=-0.2) ---\n");
        CHECK(std::isfinite(ik.theta1), "theta1=%.6f", ik.theta1);
        CHECK(std::isfinite(ik.theta2), "theta2=%.6f", ik.theta2);
        printf("    theta1=%.6f rad (%.2f°)\n", ik.theta1, ik.theta1 * 180.0 / M_PI);
        printf("    theta2=%.6f rad (%.2f°)\n", ik.theta2, ik.theta2 * 180.0 / M_PI);
        printf("\n");
    }

    // ── 测试 5: pitch 范围扫描 — 验证值域合理且连续 ──
    {
        printf("--- 测试 5: pitch 范围扫描 [-0.35, 0.35], roll=0 ---\n");
        int errors = 0;
        double prev_t1 = -999, prev_t2 = -999;
        for (int i = 0; i <= 20; ++i) {
            const double p = -0.35 + 0.7 * i / 20.0;
            const auto ik = ankle.ik(p, 0.0);
            if (ik.error_state != 0) ++errors;
            // 检查没有剧烈跳变（相邻步差值 < 0.5 rad）
            if (i > 0) {
                if (std::abs(ik.theta1 - prev_t1) > 0.5) ++errors;
                if (std::abs(ik.theta2 - prev_t2) > 0.5) ++errors;
            }
            prev_t1 = ik.theta1;
            prev_t2 = ik.theta2;
            // 检查值域在合理范围 [-2, 2] rad
            if (ik.theta1 < -2.0 || ik.theta1 > 2.0) ++errors;
            if (ik.theta2 < -2.0 || ik.theta2 > 2.0) ++errors;
        }
        CHECK(errors == 0, "pitch扫描连续且值域合理 (errors=%d)", errors);
        printf("\n");
    }

    // ── 测试 6: roll 范围扫描 [-0.2, 0.2], pitch=0 ──
    {
        printf("--- 测试 6: roll 范围扫描 [-0.2, 0.2], pitch=0 ---\n");
        int errors = 0;
        for (int i = 0; i <= 20; ++i) {
            const double r = -0.2 + 0.4 * i / 20.0;
            const auto ik = ankle.ik(0.0, r);
            if (ik.error_state != 0) ++errors;
            CHECK(std::isfinite(ik.theta1) && std::isfinite(ik.theta2),
                  "  pitch=0 roll=%.3f: theta1=%.4f theta2=%.4f",
                  r, ik.theta1, ik.theta2);
        }
        CHECK(errors == 0, "roll扫描无异常 (errors=%d)", errors);
        printf("\n");
    }

    // ── 测试 7: 全范围网格扫描 — 确保整个工作空间无崩溃 ──
    {
        printf("--- 测试 7: 全范围网格扫描 (11x11) ---\n");
        int nan_count = 0, error_count = 0;
        for (int i = 0; i <= 10; ++i) {
            const double p = -0.35 + 0.7 * i / 10.0;
            for (int j = 0; j <= 10; ++j) {
                const double r = -0.2 + 0.4 * j / 10.0;
                const auto ik = ankle.ik(p, r);
                if (!std::isfinite(ik.theta1) || !std::isfinite(ik.theta2)) ++nan_count;
                if (ik.error_state != 0) ++error_count;
            }
        }
        CHECK(nan_count == 0, "全范围无 NaN (count=%d)", nan_count);
        CHECK(error_count == 0, "全范围无异常状态 (count=%d)", error_count);
        printf("\n");
    }

    // ── 汇总 ──
    printf("========================================\n");
    printf("  通过: %d,  失败: %d,  总计: %d\n",
           tests_passed, tests_failed, tests_passed + tests_failed);
    printf("========================================\n");

    return tests_failed > 0 ? 1 : 0;
}
