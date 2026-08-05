#include <array>
#include <cmath>
#include <iostream>
#include <limits>

class ParallelAnkle {
public:
    struct IKResult {
        double theta1;
        double theta2;
        int error_state;
    };

    struct FWResult {
        double pitch;
        double roll;
        int error_state;
    };

    struct JacResult {
        std::array<std::array<double, 2>, 2> Jc;
        int error_state;
    };

    using Vec2 = std::array<double, 2>;
    using Vec3 = std::array<double, 3>;
    using Mat2 = std::array<std::array<double, 2>, 2>;
    using Mat3 = std::array<std::array<double, 3>, 3>;

    ParallelAnkle() {
        h0 = {0.0, 0.0, 0.0};
        ra01 = {0.04876, 0.0, 0.15833};
        ra02 = {0.04876, 0.0, 0.24138};

        rb01 = {0.04876, 0.02498, 0.15935};
        rb02 = {0.04876, -0.0246, 0.2458};
        lbar1 = norm(sub(rb01, ra01));
        lbar2 = norm(sub(rb02, ra02));

        const double cz = 0.0;
        lspace = 0.036;
        rc01 = {0.05, lspace * 0.5, cz};
        rc02 = {0.05, -lspace * 0.5, cz};
        hc01 = sub(rc01, h0);
        hc02 = sub(rc02, h0);

        lrod1 = norm(sub(rb01, rc01));
        lrod2 = norm(sub(rb02, rc02));

        re01 = {0.07, lspace * 0.5, cz};
        re02 = {0.07, -lspace * 0.5, cz};
        he01 = sub(re01, h0);
        he02 = sub(re02, h0);

        rab01_y = {0.0, lbar1, 0.0};
        rab02_y = {0.0, -lbar2, 0.0};

        m01 = 0.04080999528324341;
        m02 = -0.17777792304213386;

        pitch_range = {-0.35, 0.35};
        roll_range = {-0.2, 0.2};
    }

    Mat3 RyPlot(double pitch) const {
        const double c = std::cos(pitch);
        const double s = std::sin(pitch);
        return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
    }

    Mat3 RxPlot(double roll) const {
        const double c = std::cos(roll);
        const double s = std::sin(roll);
        return {{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
    }

    Mat3 xrot(double pitch, double roll) const {
        return mul(RyPlot(pitch), RxPlot(roll));
    }

    IKResult ik(double pitch, double roll) const {
        int error_state = 0;

        const Vec3 rh = mul(RyPlot(pitch), h0);
        const Vec3 rc1 = add(rh, mul(xrot(pitch, roll), hc01));
        const Vec3 rac1 = sub(rc1, ra01);
        const double c1 = -(lrod1 * lrod1 - lbar1 * lbar1 - rac1[0] * rac1[0] - rac1[1] * rac1[1] - rac1[2] * rac1[2]) /
                          (2.0 * lbar1);

        const double a1 = rac1[1];
        const double b1 = rac1[2];

        double disc = b1 * b1 * c1 * c1 - (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1);
        if (disc < 0.0) {
            disc = 1e-4;
            error_state = 1;
        }

        double ss = (b1 * c1 + std::sqrt(disc)) / (a1 * a1 + b1 * b1);
        if (ss < -1.0) {
            ss = -1.0;
            error_state = 2;
        } else if (ss > 1.0) {
            ss = 1.0;
            error_state = 3;
        }
        const double theta1 = std::asin(ss) - m01;

        const Vec3 rc2 = add(rh, mul(xrot(pitch, roll), hc02));
        const Vec3 rac2 = sub(rc2, ra02);
        const double c2 = (lrod2 * lrod2 - rac2[0] * rac2[0] - rac2[1] * rac2[1] - lbar2 * lbar2 - rac2[2] * rac2[2]) /
                          (2.0 * lbar2);

        const double a2 = rac2[1];
        const double b2 = rac2[2];

        disc = b2 * b2 * c2 * c2 - (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2);
        if (disc < 0.0) {
            disc = 0.0;
            error_state = 4;
        }

        ss = (b2 * c2 - std::sqrt(disc)) / (a2 * a2 + b2 * b2);
        if (ss < -1.0) {
            ss = -1.0;
            error_state = 5;
        } else if (ss > 1.0) {
            ss = 1.0;
            error_state = 6;
        }

        const double theta2 = std::asin(ss) - m02;
        return {theta1, theta2, error_state};
    }

    JacResult Jac(double pitch, double roll, double theta1, double theta2) const {
        int error_state = 0;

        const Vec3 s11 = {1.0, 0.0, 0.0};
        const Vec3 s21 = s11;

        const Vec3 rb1 = add(ra01, mul(RxPlot(theta1 + m01), rab01_y));
        const Vec3 rh = mul(RyPlot(pitch), h0);
        const Vec3 rc1 = add(rh, mul(xrot(pitch, roll), hc01));

        const Vec3 rbar1 = sub(rb1, ra01);
        const Vec3 rrod1 = sub(rc1, rb1);

        const Vec3 rb2 = add(ra02, mul(RxPlot(theta2 + m02), rab02_y));
        const Vec3 rc2 = add(rh, mul(xrot(pitch, roll), hc02));

        const Vec3 rbar2 = sub(rb2, ra02);
        const Vec3 rrod2 = sub(rc2, rb2);

        const double jtheta11 = dot(s11, cross(rbar1, rrod1));
        const double jtheta22 = dot(s21, cross(rbar2, rrod2));

        if (std::abs(jtheta11) < 1e-12 || std::abs(jtheta22) < 1e-12) {
            error_state = 1;
            return {{{{0.0, 0.0}, {0.0, 0.0}}}, error_state};
        }

        std::array<std::array<double, 6>, 2> Jx{};
        Jx[0][0] = rrod1[0];
        Jx[0][1] = rrod1[1];
        Jx[0][2] = rrod1[2];
        const Vec3 cr1 = cross(rc1, rrod1);
        Jx[0][3] = cr1[0];
        Jx[0][4] = cr1[1];
        Jx[0][5] = cr1[2];

        Jx[1][0] = rrod2[0];
        Jx[1][1] = rrod2[1];
        Jx[1][2] = rrod2[2];
        const Vec3 cr2 = cross(rc2, rrod2);
        Jx[1][3] = cr2[0];
        Jx[1][4] = cr2[1];
        Jx[1][5] = cr2[2];

        const double cp = std::cos(pitch);
        const double sp = std::sin(pitch);

        std::array<std::array<double, 2>, 6> G{};
        G[3][0] = cp;
        G[5][0] = -sp;
        G[4][1] = 1.0;

        Mat2 JxG{};
        for (int r = 0; r < 2; ++r) {
            for (int c = 0; c < 2; ++c) {
                double sum = 0.0;
                for (int k = 0; k < 6; ++k) {
                    sum += Jx[r][k] * G[k][c];
                }
                JxG[r][c] = sum;
            }
        }

        Mat2 Jc{};
        Jc[0][0] = JxG[0][0] / jtheta11;
        Jc[0][1] = JxG[0][1] / jtheta11;
        Jc[1][0] = JxG[1][0] / jtheta22;
        Jc[1][1] = JxG[1][1] / jtheta22;

        const double det = Jc[0][0] * Jc[1][1] - Jc[0][1] * Jc[1][0];
        if (std::abs(det) < 1e-12) {
            error_state = 1;
        }

        return {Jc, error_state};
    }

    FWResult fw(double pitchRef, double rollRef, double theta1, double theta2) const {
        const double epsilon = 1e-6;
        const int max_iterations = 100;
        const double dt = 1.0;

        double pitch = pitchRef;
        double roll = rollRef;

        for (int i = 0; i < max_iterations; ++i) {
            pitch = clamp(pitch, pitch_range[0], pitch_range[1]);
            roll = clamp(roll, roll_range[0], roll_range[1]);

            const IKResult ikr = ik(pitch, roll);
            if (std::isnan(ikr.theta1) || std::isnan(ikr.theta2)) {
                return {0.0, 0.0, 1};
            }

            const Vec2 error = {ikr.theta1 - theta1, ikr.theta2 - theta2};
            if (std::sqrt(error[0] * error[0] + error[1] * error[1]) < epsilon) {
                return {pitch, roll, 0};
            }

            const JacResult jr = Jac(pitch, roll, ikr.theta1, ikr.theta2);
            Mat2 invJc{};
            if (!invert2x2(jr.Jc, invJc)) {
                return {0.0, 0.0, 1};
            }

            const Vec2 v = mul(invJc, error);
            pitch -= v[1] * dt;
            roll -= v[0] * dt;
        }

        return {0.0, 0.0, 1};
    }

    int forceVelJac(double pitch, double roll, double theta1, double theta2) const {
        const JacResult jr = Jac(pitch, roll, theta1, theta2);
        const Vec2 jointVel = {0.1, 0.4};
        const Vec2 motorVel = mul(jr.Jc, jointVel);

        const Vec2 motorForce = {0.1, 0.2};
        Mat2 jt{};
        jt[0][0] = jr.Jc[0][0];
        jt[0][1] = jr.Jc[1][0];
        jt[1][0] = jr.Jc[0][1];
        jt[1][1] = jr.Jc[1][1];
        const Vec2 jointForce = mul(jt, motorForce);

        (void)motorVel;
        (void)jointForce;
        return 1;
    }

private:
    Vec3 h0{};
    Vec3 ra01{}, ra02{};
    Vec3 rb01{}, rb02{};
    double lbar1 = 0.0;
    double lbar2 = 0.0;

    double lspace = 0.0;
    Vec3 rc01{}, rc02{};
    Vec3 hc01{}, hc02{};

    double lrod1 = 0.0;
    double lrod2 = 0.0;

    Vec3 re01{}, re02{};
    Vec3 he01{}, he02{};

    Vec3 rab01_y{}, rab02_y{};

    double m01 = 0.0;
    double m02 = 0.0;

    std::array<double, 2> pitch_range{};
    std::array<double, 2> roll_range{};

    static Vec3 add(const Vec3& a, const Vec3& b) {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    static Vec3 sub(const Vec3& a, const Vec3& b) {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    static double dot(const Vec3& a, const Vec3& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static double norm(const Vec3& v) {
        return std::sqrt(dot(v, v));
    }

    static Vec3 mul(const Mat3& m, const Vec3& v) {
        return {
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
        };
    }

    static Mat3 mul(const Mat3& a, const Mat3& b) {
        Mat3 out{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                out[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    out[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        return out;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    static Vec2 mul(const Mat2& m, const Vec2& v) {
        return {
            m[0][0] * v[0] + m[0][1] * v[1],
            m[1][0] * v[0] + m[1][1] * v[1]
        };
    }

    static bool invert2x2(const Mat2& m, Mat2& out) {
        const double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        if (std::abs(det) < 1e-12 || !std::isfinite(det)) {
            out = {{{0.0, 0.0}, {0.0, 0.0}}};
            return false;
        }

        const double inv_det = 1.0 / det;
        out[0][0] = m[1][1] * inv_det;
        out[0][1] = -m[0][1] * inv_det;
        out[1][0] = -m[1][0] * inv_det;
        out[1][1] = m[0][0] * inv_det;
        return true;
    }

    static double clamp(double v, double lo, double hi) {
        if (v < lo) {
            return lo;
        }
        if (v > hi) {
            return hi;
        }
        return v;
    }

};

// int main() {
//     ParallelAnkle par;

//     const auto ik0 = par.ik(0.35, -0.2); //输入换关节位置 pitch, roll，输出换关节角度 theta1, theta2 和 error_state
//     std::cout << "ik(0,0): theta1=" << ik0.theta1 << ", theta2=" << ik0.theta2
//               << ", state=" << ik0.error_state << '\n';

//     return 0;
// }
