#ifndef DOF6_DYNA_H
#define DOF6_DYNA_H

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

// 质量 (来自URDF)
// link1: 0.12465, link2: 0.83418, link3: 0.49894, link4: 0.17215, link5: 0.38198, link6: 0.0045985
const double MASS[6] = {0.12465, 0.83418, 0.49894, 0.17215, 0.38198, 0.0045985};

// 关节偏移 (来自URDF joint origin xyz)
// joint1: (0, 0, 0.06475) from base_link
// joint2: (0.00325, 0, 0.04575) from link1
// joint3: (0, -0.3, 0) from link2
// joint4: (-0.00325, 0.218, 0.065) from link3
// joint5: (-0.00275, 0.036, 0) from link4
// joint6: (0.00275, 0.082, 0) from link5

// 各关节之间的位置向量 (在各自的坐标系下)
const Vector3d p[7] = {
    Vector3d(0.0, 0.0, 0.06475),       // p_base_to_1
    Vector3d(0.00325, 0.0, 0.04575),   // p_1_to_2
    Vector3d(0.0, -0.3, 0.0),          // p_2_to_3
    Vector3d(-0.00325, 0.218, 0.065),  // p_3_to_4
    Vector3d(-0.00275, 0.036, 0.0),    // p_4_to_5
    Vector3d(0.00275, 0.082, 0.0),     // p_5_to_6
    Vector3d(0.0, 0.0, 0.0)            // p_6_to_end
};

// 质心位置 (相对于连杆坐标系, 来自URDF link inertial origin xyz)
const Vector3d pc[6] = {
    Vector3d(0.0027728, 0.0, 0.017932),      // link1 质心
    Vector3d(-0.0035914, -0.14996, 0.0),     // link2 质心
    Vector3d(-0.00257, 0.09923, 0.051397),   // link3 质心
    Vector3d(0.0034984, 0.028153, 0.0),      // link4 质心
    Vector3d(-0.00068876, 0.076386, 0.0),    // link5 质心
    Vector3d(0.0, 0.0065, 0.0)               // link6 质心
};

// 惯性矩阵 (来自URDF, 用于完整动力学计算)
const Matrix3d INERTIA[6] = {
    (Matrix3d() << 4.9774e-05, 0, 4.964e-07, 0, 3.9409e-05, 0, 4.964e-07, 0, 4.9343e-05).finished(),
    (Matrix3d() << 0.0043271, -6.9375e-08, -2.086e-10, -6.9375e-08, 0.00024179, -2.1753e-08, -2.086e-10, -2.1753e-08, 0.0043582).finished(),
    (Matrix3d() << 0.0011822, -2.0778e-06, -1.411e-06, -2.0778e-06, 0.0003301, -0.00031014, -1.411e-06, -0.00031014, 0.00096288).finished(),
    (Matrix3d() << 6.9729e-05, 3.8219e-09, -1.909e-12, 3.8219e-09, 5.8234e-05, -2.1963e-08, -1.909e-12, -2.1963e-08, 5.0848e-05).finished(),
    (Matrix3d() << 0.00011865, -9.3104e-08, 2.9613e-06, -9.3104e-08, 0.00021653, 1.1425e-08, 2.9613e-06, 1.1425e-08, 0.00020595).finished(),
    (Matrix3d() << 3.5845e-07, 0, 0, 0, 6.9773e-07, 0, 0, 0, 3.5845e-07).finished()
};

// 关节轴向 (来自URDF joint axis, 在关节坐标系下)
// joint1: (0,0,1) - Z轴
// joint2: (1,0,0) - X轴
// joint3: (1,0,0) - X轴
// joint4: (0,-1,0) - 负Y轴
// joint5: (1,0,0) - X轴
// joint6: (0,-1,0) - 负Y轴

/**
 * @brief 计算绕X轴旋转矩阵
 */
inline Matrix3d Rx(double theta) {
    Matrix3d R;
    double c = cos(theta);
    double s = sin(theta);
    R << 1, 0, 0,
         0, c, -s,
         0, s, c;
    return R;
}

/**
 * @brief 计算绕Y轴旋转矩阵
 */
inline Matrix3d Ry(double theta) {
    Matrix3d R;
    double c = cos(theta);
    double s = sin(theta);
    R << c, 0, s,
         0, 1, 0,
        -s, 0, c;
    return R;
}

/**
 * @brief 计算绕Z轴旋转矩阵
 */
inline Matrix3d Rz(double theta) {
    Matrix3d R;
    double c = cos(theta);
    double s = sin(theta);
    R << c, -s, 0,
         s, c, 0,
         0, 0, 1;
    return R;
}

/**
 * @brief 计算重力补偿力矩 (牛顿-欧拉递推)
 * @param q 6个关节角度 (弧度)
 * @return 6个关节的重力补偿力矩
 * 
 * 基于URDF的关节轴向:
 * joint1: Z轴, joint2: X轴, joint3: X轴
 * joint4: -Y轴, joint5: X轴, joint6: -Y轴
 */
VectorXd compute_gravity_compensation(const double q[6])
{
    // 旋转矩阵 R_i^{i-1} (从第i个坐标系到第i-1个坐标系)
    Matrix3d R01, R12, R23, R34, R45, R56;
    Matrix3d R10, R21, R32, R43, R54, R65;
    
    // 根据URDF的关节轴向计算旋转矩阵
    // joint1绕Z轴旋转
    R01 = Rz(q[0]);
    
    // joint2绕X轴旋转
    R12 = Rx(q[1]);
    
    // joint3绕X轴旋转
    R23 = Rx(q[2]);
    
    // joint4绕-Y轴旋转 (等于绕Y轴旋转-q[3])
    R34 = Ry(-q[3]);
    
    // joint5绕X轴旋转
    R45 = Rx(q[4]);
    
    // joint6绕-Y轴旋转 (等于绕Y轴旋转-q[5])
    R56 = Ry(-q[5]);
    
    // 转置得到逆旋转矩阵
    R10 = R01.transpose();
    R21 = R12.transpose();
    R32 = R23.transpose();
    R43 = R34.transpose();
    R54 = R45.transpose();
    R65 = R56.transpose();
    
    // ========== 外推: 计算各连杆的线加速度 ==========
    // 对于静态重力补偿，角速度和角加速度都为0
    // 只有线加速度需要考虑重力
    
    Vector3d v00d = GRAVITY;  // 基座虚拟加速度 (等于重力)
    
    // 各连杆质心的线加速度
    Vector3d v11d, v22d, v33d, v44d, v55d, v66d;
    Vector3d vc11d, vc22d, vc33d, vc44d, vc55d, vc66d;
    
    // 静态情况下，各连杆原点的加速度只是重力的传递
    v11d = R10 * v00d;
    v22d = R21 * v11d;
    v33d = R32 * v22d;
    v44d = R43 * v33d;
    v55d = R54 * v44d;
    v66d = R65 * v55d;
    
    // 质心加速度 (静态情况下等于连杆原点加速度)
    vc11d = v11d;
    vc22d = v22d;
    vc33d = v33d;
    vc44d = v44d;
    vc55d = v55d;
    vc66d = v66d;
    
    // ========== 各连杆受到的惯性力 ==========
    Vector3d F11, F22, F33, F44, F55, F66;
    F11 = MASS[0] * vc11d;
    F22 = MASS[1] * vc22d;
    F33 = MASS[2] * vc33d;
    F44 = MASS[3] * vc44d;
    F55 = MASS[4] * vc55d;
    F66 = MASS[5] * vc66d;
    
    // ========== 内推: 计算关节力和力矩 ==========
    Vector3d f11, f22, f33, f44, f55, f66;
    Vector3d n11, n22, n33, n44, n55, n66;
    
    // 从末端向基座递推
    // 连杆6
    f66 = F66;
    n66 = pc[5].cross(F66);
    
    // 连杆5
    f55 = R56 * f66 + F55;
    n55 = R56 * n66 + pc[4].cross(F55) + p[5].cross(R56 * f66);
    
    // 连杆4
    f44 = R45 * f55 + F44;
    n44 = R45 * n55 + pc[3].cross(F44) + p[4].cross(R45 * f55);
    
    // 连杆3
    f33 = R34 * f44 + F33;
    n33 = R34 * n44 + pc[2].cross(F33) + p[3].cross(R34 * f44);
    
    // 连杆2
    f22 = R23 * f33 + F22;
    n22 = R23 * n33 + pc[1].cross(F22) + p[2].cross(R23 * f33);
    
    // 连杆1
    f11 = R12 * f22 + F11;
    n11 = R12 * n22 + pc[0].cross(F11) + p[1].cross(R12 * f22);
    
    // ========== 提取关节力矩 ==========
    // 力矩在关节轴向上的投影
    VectorXd tau(6);
    
    // joint1: Z轴
    tau[0] = n11.dot(Vector3d(0, 0, 1));
    
    // joint2: X轴
    tau[1] = n22.dot(Vector3d(1, 0, 0));
    
    // joint3: X轴
    tau[2] = n33.dot(Vector3d(1, 0, 0));
    
    // joint4: -Y轴
    tau[3] = n44.dot(Vector3d(0, -1, 0));
    
    // joint5: X轴
    tau[4] = n55.dot(Vector3d(1, 0, 0));
    
    // joint6: -Y轴
    tau[5] = n66.dot(Vector3d(0, -1, 0));
    
    return tau;
}

#endif // DOF6_DYNA_H