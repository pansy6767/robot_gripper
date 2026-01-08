// /**
//  * @file zero_force_control.cpp
//  * @brief 零力控制(重力补偿) 
//  * 
//  * 修复：移除不稳定的库仑摩擦补偿，只保留粘性阻尼
//  */

// #include <ros/ros.h>
// #include <sensor_msgs/JointState.h>
// #include <signal.h>
// #include <cmath>
// #include <array>
// #include <atomic>
// #include <eigen3/Eigen/Dense>

// #include "dmbot_serial/protocol/damiao.h"

// using namespace Eigen;

// //=============================================================================
// // 常量
// //=============================================================================
// constexpr int DOF = 6;
// constexpr double G = 9.8;

// const double TAU_MAX[DOF] = {28.0, 28.0, 28.0, 10.0, 10.0, 10.0};
// const double JOINT_LOWER[DOF] = {-2.7, -3.3, -0.2, -3.2, -1.5, -0.2};
// const double JOINT_UPPER[DOF] = { 2.7,  0.2,  2.2,  3.2,  1.5,  1.3};
// const double MASS[DOF] = {0.138, 1.379, 0.727, 0.370, 0.391, 0.179};

// const Vector3d COM[DOF] = {
//     Vector3d(0.0027728, 0.0, 0.017932),
//     Vector3d(-0.0035914, -0.14996, 0.0),
//     Vector3d(-0.00257, 0.09923, 0.051397),
//     Vector3d(0.0034984, 0.028153, 0.0),
//     Vector3d(-0.00068876, 0.076386, 0.0),
//     Vector3d(0.0, 0.0065, 0.0)
// };

// // 电机方向系数
// const int MOTOR_DIR[DOF] = {1, -1, 1, -1, -1, 1};

// //=============================================================================
// // 全局变量
// //=============================================================================
// std::atomic<bool> g_running{true};
// std::shared_ptr<damiao::Motor_Control> g_motor_ctrl;

// int g_debug_count = 0;
// bool g_debug_mode = true;
// double g_damping = 0.5;
// double g_scale = 1.0;

// void signalHandler(int) { g_running = false; }

// inline double clamp(double v, double lo, double hi) {
//     return v < lo ? lo : (v > hi ? hi : v);
// }

// Matrix3d rotX(double t) {
//     Matrix3d R; double c=cos(t), s=sin(t);
//     R << 1,0,0, 0,c,-s, 0,s,c; return R;
// }
// Matrix3d rotY(double t) {
//     Matrix3d R; double c=cos(t), s=sin(t);
//     R << c,0,s, 0,1,0, -s,0,c; return R;
// }
// Matrix3d rotZ(double t) {
//     Matrix3d R; double c=cos(t), s=sin(t);
//     R << c,-s,0, s,c,0, 0,0,1; return R;
// }

// //=============================================================================
// // 重力补偿
// //=============================================================================
// void computeGravityCompensation(const std::array<double, DOF>& q, 
//                                  std::array<double, DOF>& tau_g) {
//     tau_g.fill(0.0);
    
//     Matrix4d T[DOF + 1];
//     T[0] = Matrix4d::Identity();
    
//     // J1: Z轴
//     Matrix4d T1 = Matrix4d::Identity();
//     T1.block<3,3>(0,0) = rotZ(q[0]);
//     T1(2,3) = 0.06475;
//     T[1] = T[0] * T1;
    
//     // J2: X轴
//     Matrix4d T2 = Matrix4d::Identity();
//     T2.block<3,3>(0,0) = rotX(q[1]);
//     T2(0,3) = 0.00325; T2(2,3) = 0.04575;
//     T[2] = T[1] * T2;
    
//     // J3: X轴
//     Matrix4d T3 = Matrix4d::Identity();
//     T3.block<3,3>(0,0) = rotX(q[2]);
//     T3(1,3) = -0.3;
//     T[3] = T[2] * T3;
    
//     // J4: -Y轴
//     Matrix4d T4 = Matrix4d::Identity();
//     T4.block<3,3>(0,0) = rotY(-q[3]);
//     T4(0,3) = -0.00325; T4(1,3) = 0.218; T4(2,3) = 0.065;
//     T[4] = T[3] * T4;
    
//     // J5: X轴
//     Matrix4d T5 = Matrix4d::Identity();
//     T5.block<3,3>(0,0) = rotX(q[4]);
//     T5(0,3) = -0.00275; T5(1,3) = 0.036;
//     T[5] = T[4] * T5;
    
//     // J6: -Y轴
//     Matrix4d T6 = Matrix4d::Identity();
//     T6.block<3,3>(0,0) = rotY(-q[5]);
//     T6(0,3) = 0.00275; T6(1,3) = 0.082;
//     T[6] = T[5] * T6;
    
//     Vector3d axis[DOF] = {
//         {0,0,1}, {1,0,0}, {1,0,0}, {0,-1,0}, {1,0,0}, {0,-1,0}
//     };
    
//     Vector3d g_world(0, 0, -G);
    
//     for (int link = 0; link < DOF; link++) {
//         Vector4d com_local(COM[link](0), COM[link](1), COM[link](2), 1.0);
//         Vector4d com_world = T[link + 1] * com_local;
//         Vector3d p_com(com_world(0), com_world(1), com_world(2));
//         Vector3d F = MASS[link] * g_world;
        
//         for (int j = 0; j <= link; j++) {
//             Vector3d p_joint(T[j+1](0,3), T[j+1](1,3), T[j+1](2,3));
//             Vector3d r = p_com - p_joint;
//             Vector3d torque = r.cross(F);
//             Vector3d axis_world = T[j+1].block<3,3>(0,0) * axis[j];
//             tau_g[j] += torque.dot(axis_world);
//         }
//     }
    
//     for (int i = 0; i < DOF; i++)
//         tau_g[i] = -tau_g[i];
// }

// //=============================================================================
// // 电机控制
// //=============================================================================
// bool initMotors() {
//     std::vector<damiao::DmActData> data;
//     for (int i = 0; i < 3; ++i)
//         data.push_back({damiao::DM4340, damiao::MIT_MODE, 
//                        (uint16_t)(i+1), (uint16_t)(0x11+i)});
//     for (int i = 3; i < 6; ++i)
//         data.push_back({damiao::DM4310, damiao::MIT_MODE, 
//                        (uint16_t)(i+1), (uint16_t)(0x11+i)});
//     try {
//         g_motor_ctrl = std::make_shared<damiao::Motor_Control>(
//             1000000, 5000000, "14AA044B241402B10DDBDAFE448040BB", &data);
//         ros::Duration(0.5).sleep();
//         for (int i = 0; i < DOF; ++i) {
//             g_motor_ctrl->switchControlMode(
//                 *g_motor_ctrl->getMotor(i+1), damiao::MIT);
//             usleep(5000);
//         }
//         return true;
//     } catch (...) { return false; }
// }

// void readMotors(std::array<double, DOF>& pos, std::array<double, DOF>& vel) {
//     for (int i = 0; i < DOF; ++i)
//         g_motor_ctrl->refresh_motor_status(*g_motor_ctrl->getMotor(i+1));
//     usleep(500);
//     for (int i = 0; i < DOF; ++i) {
//         auto m = g_motor_ctrl->getMotor(i+1);
//         if (m) {
//             pos[i] = m->Get_Position() * MOTOR_DIR[i];
//             vel[i] = m->Get_Velocity() * MOTOR_DIR[i];
//         }
//     }
// }

// void sendTorque(const std::array<double, DOF>& tau) {
//     for (int i = 0; i < DOF; ++i) {
//         auto m = g_motor_ctrl->getMotor(i+1);
//         if (m) {
//             double t = tau[i] * MOTOR_DIR[i];
//             t = clamp(t, -TAU_MAX[i], TAU_MAX[i]);
//             g_motor_ctrl->control_mit(*m, 0, 0, 0, 0, t);
//         }
//     }
// }

// void sendZero() {
//     for (int i = 0; i < DOF; ++i) {
//         auto m = g_motor_ctrl->getMotor(i+1);
//         if (m) g_motor_ctrl->control_mit(*m, 0, 0, 0, 0, 0);
//     }
// }

// //=============================================================================
// // 主函数
// //=============================================================================
// int main(int argc, char** argv) {
//     ros::init(argc, argv, "zero_force_node");
//     ros::NodeHandle nh("~");
    
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);

//     double rate;
//     nh.param("damping", g_damping, 0.5);
//     nh.param("scale", g_scale, 1.0);
//     nh.param("rate", rate, 500.0);
//     nh.param("debug", g_debug_mode, true);

//     ROS_INFO("========================================");
//     ROS_INFO("  Zero-Force Control (No Friction)");
//     ROS_INFO("========================================");
//     ROS_INFO("  damping: %.2f", g_damping);
//     ROS_INFO("  scale:   %.2f", g_scale);
//     ROS_INFO("  rate:    %.0f Hz", rate);
//     ROS_INFO("========================================");

//     if (!initMotors()) { ROS_ERROR("Motor init failed"); return -1; }
//     ROS_INFO("Motors OK");

//     ros::Publisher pub_js = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
//     sensor_msgs::JointState js;
//     js.name = {"joint1","joint2","joint3","joint4","joint5","joint6"};
//     js.position.resize(DOF); js.velocity.resize(DOF); js.effort.resize(DOF);

//     // 软启动
//     const int RAMP_STEPS = 500;
//     int ramp_count = 0;
//     ROS_INFO("Soft start...");

//     ros::Rate loop(rate);
//     std::array<double, DOF> q, dq;
    
//     while (ros::ok() && g_running) {
//         readMotors(q, dq);
//         g_debug_count++;
        
//         // 安全检查
//         bool emergency = false;
//         for (int i = 0; i < DOF; ++i) {
//             if (q[i] < JOINT_LOWER[i] || q[i] > JOINT_UPPER[i]) {
//                 ROS_ERROR("J%d=%.2f out!", i+1, q[i]);
//                 emergency = true;
//             }
//         }
//         if (emergency) { sendZero(); break; }
        
//         // 重力补偿
//         std::array<double, DOF> tau_g;
//         computeGravityCompensation(q, tau_g);
        
//         // 合成力矩：只有重力补偿 + 阻尼（无摩擦补偿）
//         std::array<double, DOF> tau_cmd;
//         for (int i = 0; i < DOF; ++i) {
//             tau_cmd[i] = g_scale * tau_g[i] - g_damping * dq[i];
//         }
        
//         // 软启动
//         double ramp = 1.0;
//         if (ramp_count < RAMP_STEPS) {
//             ramp = (double)ramp_count / RAMP_STEPS;
//             ramp_count++;
//             if (ramp_count == RAMP_STEPS) ROS_INFO("Soft start done.");
//         }
//         for (int i = 0; i < DOF; ++i) tau_cmd[i] *= ramp;
        
//         // 调试输出
//         if (g_debug_mode && g_debug_count % 500 == 0) {
//             ROS_INFO("q:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
//                      q[0],q[1],q[2],q[3],q[4],q[5]);
//             ROS_INFO("dq:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
//                      dq[0],dq[1],dq[2],dq[3],dq[4],dq[5]);
//             ROS_INFO("tau_g:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
//                      tau_g[0],tau_g[1],tau_g[2],tau_g[3],tau_g[4],tau_g[5]);
//             ROS_INFO("tau_cmd:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
//                      tau_cmd[0],tau_cmd[1],tau_cmd[2],tau_cmd[3],tau_cmd[4],tau_cmd[5]);
//             ROS_INFO("----");
//         }
        
//         sendTorque(tau_cmd);
        
//         js.header.stamp = ros::Time::now();
//         for (int i = 0; i < DOF; ++i) {
//             js.position[i] = q[i]; js.velocity[i] = dq[i]; js.effort[i] = tau_cmd[i];
//         }
//         pub_js.publish(js);
        
//         ros::spinOnce();
//         loop.sleep();
//     }

//     sendZero();
//     ROS_INFO("Stopped.");
//     return 0;
// }


/**
 * @file zero_force_node.cpp
 * @brief 零力控制启动节点 - 通过话题控制AbbRobot的力矩模式
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <signal.h>
#include <atomic>

std::atomic<bool> g_running{true};
ros::Publisher* g_mode_pub = nullptr;

void signalHandler(int)
{
    g_running = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zero_force_node");
    ros::NodeHandle nh("~");

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    double damping, scale;
    nh.param("damping", damping, 0.5);
    nh.param("scale", scale, 1.0);

    ROS_INFO("========================================");
    ROS_INFO("  Zero-Force Control Node");
    ROS_INFO("========================================");
    ROS_INFO("  damping: %.2f", damping);
    ROS_INFO("  scale:   %.2f", scale);
    ROS_INFO("========================================");

    // 发布器
    ros::Publisher mode_pub = nh.advertise<std_msgs::Bool>("/abb/torque_mode_enable", 10);
    ros::Publisher params_pub = nh.advertise<std_msgs::Float64MultiArray>("/abb/torque_params", 10);
    g_mode_pub = &mode_pub;

    // 等待连接
    ROS_INFO("Waiting for subscribers...");
    ros::Duration(1.0).sleep();

    // 发送参数
    std_msgs::Float64MultiArray params_msg;
    params_msg.data.resize(2);
    params_msg.data[0] = scale;
    params_msg.data[1] = damping;
    for (int i = 0; i < 5; ++i)
    {
        params_pub.publish(params_msg);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Parameters sent.");

    // 启用力矩模式
    ROS_INFO("Enabling torque mode...");
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    for (int i = 0; i < 10; ++i)
    {
        mode_pub.publish(enable_msg);
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    ROS_INFO("Torque mode enabled!");

    // 主循环 - 保持运行直到收到退出信号
    ros::Rate loop(10);
    while (ros::ok() && g_running)
    {
        ros::spinOnce();
        loop.sleep();
    }

    // 退出：禁用力矩模式
    ROS_INFO("Disabling torque mode...");
    enable_msg.data = false;
    for (int i = 0; i < 10; ++i)
    {
        mode_pub.publish(enable_msg);
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    ROS_INFO("Zero-force control stopped.");
    return 0;
}