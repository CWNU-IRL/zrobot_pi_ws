#pragma once

#include <array>
#include <cmath>
#include <limits>

/**
 * @brief 并联踝关节运动学解算器（含正/逆运动学与雅可比）
 *
 * - ik():  逆运动学，pitch/roll → 电机角度 (theta1, theta2)
 * - fw():  正运动学，电机角度 (theta1, theta2) → pitch/roll（Newton-Raphson 迭代）
 * - Jac(): 雅可比矩阵 Jc，满足 [dθ₁, dθ₂]ᵀ = Jc · [d(roll), d(pitch)]ᵀ
 *           注意：Jc 的列顺序为 [roll, pitch]
 */
class ParallelAnkle {
public:
    // 踝关节机械限位 (rad)
    static constexpr double PITCH_MIN = -0.35;
    static constexpr double PITCH_MAX =  0.35;
    static constexpr double ROLL_MIN  = -0.2;
    static constexpr double ROLL_MAX  =  0.2;

    struct IKResult {
        double theta1;   // 电机1角度 (rad)
        double theta2;   // 电机2角度 (rad)
        int error_state; // 0=正常, 非0=求解异常
    };

    struct FWResult {
        double pitch;     // 正运动学求解的 pitch (rad)
        double roll;      // 正运动学求解的 roll (rad)
        int error_state;  // 0=正常, 非0=未收敛
    };

    struct JacResult {
        std::array<std::array<double, 2>, 2> Jc;  // 2×2 雅可比
        int error_state;
    };

    using Vec2 = std::array<double, 2>;
    using Vec3 = std::array<double, 3>;
    using Mat2 = std::array<std::array<double, 2>, 2>;
    using Mat3 = std::array<std::array<double, 3>, 3>;

    ParallelAnkle() {
        h0_      = {0.0, 0.0, 0.0};
        ra01_    = {0.04876, 0.0, 0.15833};
        ra02_    = {0.04876, 0.0, 0.24138};

        rb01_    = {0.04876, 0.02498, 0.15935};
        rb02_    = {0.04876, -0.0246, 0.2458};
        lbar1_   = norm(sub(rb01_, ra01_));
        lbar2_   = norm(sub(rb02_, ra02_));

        const double cz = 0.0;
        lspace_  = 0.036;
        rc01_    = {0.05, lspace_ * 0.5, cz};
        rc02_    = {0.05, -lspace_ * 0.5, cz};
        hc01_    = sub(rc01_, h0_);
        hc02_    = sub(rc02_, h0_);

        lrod1_   = norm(sub(rb01_, rc01_));
        lrod2_   = norm(sub(rb02_, rc02_));

        re01_    = {0.07, lspace_ * 0.5, cz};
        re02_    = {0.07, -lspace_ * 0.5, cz};
        he01_    = sub(re01_, h0_);
        he02_    = sub(re02_, h0_);

        rab01_y_ = {0.0, lbar1_, 0.0};
        rab02_y_ = {0.0, -lbar2_, 0.0};

        m01_     = 0.04080999528324341;
        m02_     = -0.17777792304213386;

        pitch_range_ = {PITCH_MIN, PITCH_MAX};
        roll_range_  = {ROLL_MIN, ROLL_MAX};
    }

    // ========== 逆运动学 ==========
    IKResult ik(double pitch, double roll) const {
        int error_state = 0;

        const Vec3 rh   = mul(ry(-pitch), h0_);
        const Vec3 rc1  = add(rh, mul(mul(ry(-pitch), rx(roll)), hc01_));
        const Vec3 rac1 = sub(rc1, ra01_);
        const double c1 = -(lrod1_ * lrod1_ - lbar1_ * lbar1_
                            - rac1[0] * rac1[0] - rac1[1] * rac1[1] - rac1[2] * rac1[2])
                          / (2.0 * lbar1_);

        const double a1 = rac1[1];
        const double b1 = rac1[2];

        double disc = b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1);
        if (disc < 0.0) {
            disc = 1e-4;
            error_state = 1;
        }

        double ss = (b1 * c1 + std::sqrt(disc)) / (a1 * a1 + b1 * b1);
        if (ss < -1.0) { ss = -1.0; error_state = 2; }
        else if (ss > 1.0) { ss = 1.0; error_state = 3; }
        const double theta1 = std::asin(ss) - m01_;

        const Vec3 rc2  = add(rh, mul(mul(ry(-pitch), rx(roll)), hc02_));
        const Vec3 rac2 = sub(rc2, ra02_);
        const double c2 = (lrod2_ * lrod2_ - rac2[0] * rac2[0] - rac2[1] * rac2[1]
                           - lbar2_ * lbar2_ - rac2[2] * rac2[2])
                          / (2.0 * lbar2_);

        const double a2 = rac2[1];
        const double b2 = rac2[2];

        disc = b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2);
        if (disc < 0.0) {
            disc = 0.0;
            error_state = 4;
        }

        ss = (b2 * c2 - std::sqrt(disc)) / (a2 * a2 + b2 * b2);
        if (ss < -1.0) { ss = -1.0; error_state = 5; }
        else if (ss > 1.0) { ss = 1.0; error_state = 6; }

        const double theta2 = std::asin(ss) - m02_;
        return {-theta1, -theta2, error_state};
    }

    // ========== 正运动学 (Newton-Raphson) ==========
    FWResult fw(double pitchRef, double rollRef, double theta1, double theta2) const {
        const double epsilon = 1e-6;
        const int max_iterations = 100;
        const double dt = 1.0;

        double pitch = pitchRef;
        double roll  = rollRef;

        for (int i = 0; i < max_iterations; ++i) {
            pitch = clamp(pitch, pitch_range_[0], pitch_range_[1]);
            roll  = clamp(roll,  roll_range_[0],  roll_range_[1]);

            const IKResult ikr = ik(pitch, roll);
            if (std::isnan(ikr.theta1) || std::isnan(ikr.theta2)) {
                return {0.0, 0.0, 1};
            }

            const Vec2 error = {theta1 - ikr.theta1, theta2 - ikr.theta2};
            if (std::sqrt(error[0] * error[0] + error[1] * error[1]) < epsilon) {
                return {pitch, roll, 0};
            }

            const JacResult jr = Jac(pitch, roll, -ikr.theta1, -ikr.theta2);
            Mat2 invJc{};
            if (!invert2x2(jr.Jc, invJc)) {
                return {0.0, 0.0, 1};
            }

            const Vec2 v = mul(invJc, error);
            pitch -= v[1] * dt;   // Jc 列顺序: [roll, pitch]
            roll  -= v[0] * dt;
        }

        return {0.0, 0.0, 1};
    }

    // ========== 雅可比矩阵 ==========
    JacResult Jac(double pitch, double roll, double theta1, double theta2) const {
        int error_state = 0;

        const Vec3 s11 = {1.0, 0.0, 0.0};
        const Vec3 s21 = s11;

        const Vec3 rb1   = add(ra01_, mul(rx(theta1 + m01_), rab01_y_));
        const Vec3 rh    = mul(ry(-pitch), h0_);
        const Vec3 rc1   = add(rh, mul(mul(ry(-pitch), rx(roll)), hc01_));

        const Vec3 rbar1 = sub(rb1, ra01_);
        const Vec3 rrod1 = sub(rc1, rb1);

        const Vec3 rb2   = add(ra02_, mul(rx(theta2 + m02_), rab02_y_));
        const Vec3 rc2   = add(rh, mul(mul(ry(-pitch), rx(roll)), hc02_));

        const Vec3 rbar2 = sub(rb2, ra02_);
        const Vec3 rrod2 = sub(rc2, rb2);

        const double jtheta11 = dot(s11, cross(rbar1, rrod1));
        const double jtheta22 = dot(s21, cross(rbar2, rrod2));

        if (!std::isfinite(jtheta11) || std::abs(jtheta11) < 1e-12 ||
            !std::isfinite(jtheta22) || std::abs(jtheta22) < 1e-12) {
            return {{{{0.0, 0.0}, {0.0, 0.0}}}, 1};
        }

        std::array<std::array<double, 6>, 2> Jx{};
        Jx[0][0] = rrod1[0]; Jx[0][1] = rrod1[1]; Jx[0][2] = rrod1[2];
        const Vec3 cr1 = cross(rc1, rrod1);
        Jx[0][3] = cr1[0]; Jx[0][4] = cr1[1]; Jx[0][5] = cr1[2];

        Jx[1][0] = rrod2[0]; Jx[1][1] = rrod2[1]; Jx[1][2] = rrod2[2];
        const Vec3 cr2 = cross(rc2, rrod2);
        Jx[1][3] = cr2[0]; Jx[1][4] = cr2[1]; Jx[1][5] = cr2[2];

        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);

        std::array<std::array<double, 2>, 6> G{};
        G[3][0] =  cp;    G[5][0] =  sp;   // ry(-pitch): dω/droll = [cos(pitch), 0, sin(pitch)]
        G[4][1] = -1.0;                     // ry(-pitch): dω/dpitch = [0, -1, 0]

        Mat2 JxG{};
        for (int r = 0; r < 2; ++r) {
            for (int c = 0; c < 2; ++c) {
                double sum = 0.0;
                for (int k = 0; k < 6; ++k) { sum += Jx[r][k] * G[k][c]; }
                JxG[r][c] = sum;
            }
        }

        Mat2 Jc{};
        Jc[0][0] = JxG[0][0] / jtheta11;  Jc[0][1] = JxG[0][1] / jtheta11;
        Jc[1][0] = JxG[1][0] / jtheta22;  Jc[1][1] = JxG[1][1] / jtheta22;

        const double det = Jc[0][0] * Jc[1][1] - Jc[0][1] * Jc[1][0];
        if (!std::isfinite(det) || std::abs(det) < 1e-12) { error_state = 1; }

        return {Jc, error_state};
    }

    // ========== 电机速度 → 机构关节速度 ==========
    // 将两个电机角速度经雅可比变换为机构 pitch/roll 角速度（机构约定）
    // theta1/theta2: 电机反馈角度（与 ik() 输出同约定，即 -θ_raw）
    // vel1/vel2:     对应电机的角速度反馈（与 ik() 输出同约定，即 -θ̇_raw）
    // 输出 pitch_vel / roll_vel: 机构约定关节角速度 (rad/s)
    // 返回 false 表示输入非法或雅可比奇异（输出保持 0，调用方应回退处理）
    bool motorVelToJointVel(double pitch, double roll, double theta1, double theta2,
                            double vel1, double vel2,
                            double& pitch_vel, double& roll_vel) const {
        if (!std::isfinite(pitch) || !std::isfinite(roll) ||
            !std::isfinite(theta1) || !std::isfinite(theta2) ||
            !std::isfinite(vel1) || !std::isfinite(vel2)) {
            return false;
        }

        const JacResult jr = Jac(pitch, roll, -theta1, -theta2);  // 反馈角→原始角
        if (jr.error_state != 0) {
            return false;
        }

        Mat2 invJc{};
        if (!invert2x2(jr.Jc, invJc)) {
            return false;
        }

        // 反馈速度 = -θ̇_raw: [ṙ, ṗ]ᵀ = -inv(Jc) · [vel1, vel2]ᵀ
        roll_vel  = -(invJc[0][0] * vel1 + invJc[0][1] * vel2);
        pitch_vel = -(invJc[1][0] * vel1 + invJc[1][1] * vel2);
        return true;
    }

private:
    // 几何参数
    Vec3 h0_{};
    Vec3 ra01_{}, ra02_{};
    Vec3 rb01_{}, rb02_{};
    double lbar1_ = 0.0, lbar2_ = 0.0;

    double lspace_ = 0.0;
    Vec3 rc01_{}, rc02_{};
    Vec3 hc01_{}, hc02_{};

    double lrod1_ = 0.0, lrod2_ = 0.0;

    Vec3 re01_{}, re02_{};
    Vec3 he01_{}, he02_{};

    Vec3 rab01_y_{}, rab02_y_{};

    double m01_ = 0.0, m02_ = 0.0;

    std::array<double, 2> pitch_range_{};
    std::array<double, 2> roll_range_{};

    // ========== 旋转矩阵 ==========
    static Mat3 ry(double a) {
        const double c = std::cos(a), s = std::sin(a);
        return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
    }
    static Mat3 rx(double a) {
        const double c = std::cos(a), s = std::sin(a);
        return {{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
    }

    // ========== 向量/矩阵运算 ==========
    static Vec3 add(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
    static Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
    static double dot(const Vec3& a, const Vec3& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    static double norm(const Vec3& v) { return std::sqrt(dot(v, v)); }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]};
    }

    static Vec3 mul(const Mat3& m, const Vec3& v) {
        return {m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
                m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
                m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]};
    }

    static Mat3 mul(const Mat3& a, const Mat3& b) {
        Mat3 out{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) out[i][j] += a[i][k] * b[k][j];
        return out;
    }

    static Vec2 mul(const Mat2& m, const Vec2& v) {
        return {m[0][0]*v[0] + m[0][1]*v[1], m[1][0]*v[0] + m[1][1]*v[1]};
    }

    static bool invert2x2(const Mat2& m, Mat2& out) {
        const double det = m[0][0]*m[1][1] - m[0][1]*m[1][0];
        if (std::abs(det) < 1e-12 || !std::isfinite(det)) {
            out = {{{0.0, 0.0}, {0.0, 0.0}}};
            return false;
        }
        const double inv_det = 1.0 / det;
        out[0][0] =  m[1][1] * inv_det;  out[0][1] = -m[0][1] * inv_det;
        out[1][0] = -m[1][0] * inv_det;  out[1][1] =  m[0][0] * inv_det;
        return true;
    }

    static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
};
