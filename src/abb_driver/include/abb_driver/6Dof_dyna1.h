#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

using namespace Eigen;
using namespace std;

// 常量定义
const double PI = M_PI;
const Vector3d GRAVITY(0, 0, 9.8); // 重力向量
const Vector3d Z_AXIS(0, 0, 1);    // Z轴单位向量

// 质量 (从URDF提取)
const double MASS[6] = {
    0.12465,   // link1
    0.83418,   // link2
    0.49894,   // link3
    0.17215,   // link4
    0.38198,   // link5
    0.0045985  // link6
};

// 改进型D-H参数 (从URDF关节变换提取)
// a_{i-1},     alpha_{i-1},    d_i,      theta_i
const double DH[6][4] = {
    {0,          0,              0.06475,  0},      // joint1
    {0.00325,    PI/2,           0.04575,  0},      // joint2
    {0.3,        0,              0,        0},      // joint3
    {-0.00325,   PI/2,           0.283,    0},      // joint4 (d = 0.218+0.065)
    {-0.00275,   -PI/2,          0.036,    0},      // joint5
    {0.00275,    PI/2,           0.082,    0}       // joint6
};

// 各关节位置向量 (从URDF的origin提取)
const Vector3d p[7] = {
    Vector3d(0.0, 0.0, 0.06475),        // p10: base_link to link1
    Vector3d(0.00325, 0.0, 0.04575),    // p21: link1 to link2
    Vector3d(0.0, -0.3, 0.0),           // p32: link2 to link3
    Vector3d(-0.00325, 0.218, 0.065),   // p43: link3 to link4
    Vector3d(-0.00275, 0.036, 0.0),     // p54: link4 to link5
    Vector3d(0.00275, 0.082, 0.0),      // p65: link5 to link6
    Vector3d(0.0, 0.0, 0.0)             // p76: end effector offset
};

// 质心位置 (相对于连杆坐标系，从URDF的inertial origin提取)
const Vector3d pc[6] = {
    Vector3d(0.0027728, 2.1768e-14, 0.017932),      // link1
    Vector3d(-0.0035914, -0.14996, 9.5822e-09),     // link2
    Vector3d(-0.00257, 0.09923, 0.051397),          // link3
    Vector3d(0.0034984, 0.028153, -1.0623e-06),     // link4
    Vector3d(-0.00068876, 0.076386, -1.3572e-05),   // link5
    Vector3d(1.8842e-15, 0.0065, -1.5543e-15)       // link6
};

// 惯性矩阵 (从URDF提取)
const Matrix3d INERTIA[6] = {
    // link1
    (Matrix3d() << 
        4.9774e-05,  3.7599e-19,  4.964e-07,
        3.7599e-19,  3.9409e-05, -1.4791e-17,
        4.964e-07,  -1.4791e-17,  4.9343e-05
    ).finished(),
    
    // link2
    (Matrix3d() << 
        0.0043271,  -6.9375e-08, -2.086e-10,
       -6.9375e-08,  0.00024179, -2.1753e-08,
       -2.086e-10,  -2.1753e-08,  0.0043582
    ).finished(),
    
    // link3
    (Matrix3d() << 
        0.0011822,  -2.0778e-06, -1.411e-06,
       -2.0778e-06,  0.0003301,  -0.00031014,
       -1.411e-06,  -0.00031014,  0.00096288
    ).finished(),
    
    // link4
    (Matrix3d() << 
        6.9729e-05,  3.8219e-09, -1.909e-12,
        3.8219e-09,  5.8234e-05, -2.1963e-08,
       -1.909e-12,  -2.1963e-08,  5.0848e-05
    ).finished(),
    
    // link5
    (Matrix3d() << 
        0.00011865, -9.3104e-08,  2.9613e-06,
       -9.3104e-08,  0.00021653,  1.1425e-08,
        2.9613e-06,  1.1425e-08,  0.00020595
    ).finished(),
    
    // link6
    (Matrix3d() << 
        3.5845e-07, -4.139e-22,  1.9791e-23,
       -4.139e-22,  6.9773e-07, -1.448e-21,
        1.9791e-23, -1.448e-21,  3.5845e-07
    ).finished()
};

// 计算重力补偿力矩 (静态情况)(只用到重心和质量)
VectorXd compute_gravity_compensation(const double q[6])
{
    // 初始化运动学链
    Matrix3d R01, R12, R23, R34, R45, R56; // R_i^{i-1}
    Matrix3d R10, R21, R32, R43, R54, R65; // R_{i-1}^{i}
    Vector3d w11, w22, w33, w44, w55, w66;
    Vector3d w11d, w22d, w33d, w44d, w55d, w66d;
    Vector3d v11d, v22d, v33d, v44d, v55d, v66d;
    Vector3d vc11d, vc22d, vc33d, vc44d, vc55d, vc66d;
    Vector3d F11, F22, F33, F44, F55, F66;
    Vector3d f11, f22, f33, f44, f55, f66;
    Vector3d n11, n22, n33, n44, n55, n66;

    // 关节角度 (根据URDF的joint limits，这里假设输入q已经是正确的角度)
    double q_offset[6];
    q_offset[0] = q[0];
    q_offset[1] = q[1];
    q_offset[2] = q[2];
    q_offset[3] = q[3];
    q_offset[4] = q[4];
    q_offset[5] = q[5];

    // 根据改进型DH参数计算旋转矩阵
    // Joint 1: alpha=0, a=0
    R01 << cos(q_offset[0]), -sin(q_offset[0]), 0.0,
           sin(q_offset[0]),  cos(q_offset[0]), 0.0,
           0.0,               0.0,              1.0;

    // Joint 2: alpha=PI/2, a=0.00325
    R12 << cos(q_offset[1]), -sin(q_offset[1]), 0.0,
           0.0,               0.0,              -1.0,
           sin(q_offset[1]),  cos(q_offset[1]), 0.0;

    // Joint 3: alpha=0, a=0.3
    R23 << cos(q_offset[2]), -sin(q_offset[2]), 0.0,
           sin(q_offset[2]),  cos(q_offset[2]), 0.0,
           0.0,               0.0,              1.0;

    // Joint 4: alpha=PI/2, a=-0.00325
    R34 << cos(q_offset[3]), -sin(q_offset[3]), 0.0,
           0.0,               0.0,              -1.0,
           sin(q_offset[3]),  cos(q_offset[3]), 0.0;

    // Joint 5: alpha=-PI/2, a=-0.00275
    R45 << cos(q_offset[4]), -sin(q_offset[4]), 0.0,
           0.0,               0.0,               1.0,
          -sin(q_offset[4]), -cos(q_offset[4]), 0.0;

    // Joint 6: alpha=PI/2, a=0.00275
    R56 << cos(q_offset[5]), -sin(q_offset[5]), 0.0,
           0.0,               0.0,              -1.0,
           sin(q_offset[5]),  cos(q_offset[5]), 0.0;

    R10 = R01.transpose();
    R21 = R12.transpose();
    R32 = R23.transpose();
    R43 = R34.transpose();
    R54 = R45.transpose();
    R65 = R56.transpose();

    // 外推初始化 (基座)
    Vector3d w00 = Vector3d::Zero();  // 基座角速度
    Vector3d v00 = Vector3d::Zero();  // 基座线速度
    Vector3d w00d = Vector3d::Zero(); // 基座角加速度
    Vector3d v00d = GRAVITY;          // 基座线加速度 (包含重力)

    w11 = Vector3d::Zero();
    w22 = Vector3d::Zero();
    w33 = Vector3d::Zero();
    w44 = Vector3d::Zero();
    w55 = Vector3d::Zero();
    w66 = Vector3d::Zero();

    w11d = Vector3d::Zero();
    w22d = Vector3d::Zero();
    w33d = Vector3d::Zero();
    w44d = Vector3d::Zero();
    w55d = Vector3d::Zero();
    w66d = Vector3d::Zero();

    // 向外传播线加速度
    v11d = R10 * v00d;
    v22d = R21 * v11d;
    v33d = R32 * v22d;
    v44d = R43 * v33d;
    v55d = R54 * v44d;
    v66d = R65 * v55d;

    // 质心加速度 (静态情况下等于线加速度)
    vc11d = v11d + w11d.cross(pc[0]) + w11.cross(w11.cross(pc[0]));
    vc22d = v22d + w22d.cross(pc[1]) + w22.cross(w22.cross(pc[1]));
    vc33d = v33d + w33d.cross(pc[2]) + w33.cross(w33.cross(pc[2]));
    vc44d = v44d + w44d.cross(pc[3]) + w44.cross(w44.cross(pc[3]));
    vc55d = v55d + w55d.cross(pc[4]) + w55.cross(w55.cross(pc[4]));
    vc66d = v66d + w66d.cross(pc[5]) + w66.cross(w66.cross(pc[5]));

    // 计算惯性力
    F11 = MASS[0] * vc11d;
    F22 = MASS[1] * vc22d;
    F33 = MASS[2] * vc33d;
    F44 = MASS[3] * vc44d;
    F55 = MASS[4] * vc55d;
    F66 = MASS[5] * vc66d;

    // 末端外力和力矩 (无外载)
    Vector3d f77 = Vector3d::Zero();
    Vector3d n77 = Vector3d::Zero();
    VectorXd tau(6);

    // 向内传播力
    f66 = R65 * f77 + F66;
    f55 = R54 * f66 + F55;
    f44 = R43 * f55 + F44;
    f33 = R32 * f44 + F33;
    f22 = R21 * f33 + F22;
    f11 = R10 * f22 + F11;

    // 向内传播力矩
    n66 = R65 * n77 + pc[5].cross(F66) + p[6].cross(R65 * f77);
    n55 = R54 * n66 + pc[4].cross(F55) + p[5].cross(R54 * f66);
    n44 = R43 * n55 + pc[3].cross(F44) + p[4].cross(R43 * f55);
    n33 = R32 * n44 + pc[2].cross(F33) + p[3].cross(R32 * f44);
    n22 = R21 * n33 + pc[1].cross(F22) + p[2].cross(R21 * f33);
    n11 = R10 * n22 + pc[0].cross(F11) + p[1].cross(R10 * f22);

    // 提取关节力矩 (投影到关节轴上)
    tau[0] = n11.dot(Z_AXIS);
    tau[1] = n22.dot(Vector3d(1, 0, 0));  // joint2绕X轴
    tau[2] = n33.dot(Vector3d(1, 0, 0));  // joint3绕X轴
    tau[3] = n44.dot(Vector3d(0, -1, 0)); // joint4绕-Y轴
    tau[4] = n55.dot(Vector3d(1, 0, 0));  // joint5绕X轴
    tau[5] = n66.dot(Vector3d(0, -1, 0)); // joint6绕-Y轴

    return tau;
}

// int main()
// {
//     // 测试示例
//     double q[6] = {0, -PI/4, PI/4, 0, 0, 0};
//     VectorXd tau = compute_gravity_compensation(q);
    
//     cout << "重力补偿力矩:" << endl;
//     cout << tau.transpose() << endl;
    
//     return 0;
// }