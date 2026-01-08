/**
 * @file teach_replay.cpp
 * @brief 零重力示教拖动回放功能
 * 
 * 功能：
 * 1. 零重力模式下手动拖动示教
 * 2. 记录轨迹数据
 * 3. 回放示教轨迹
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <cmath>
#include <array>
#include <vector>
#include <atomic>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "dmbot_serial/protocol/damiao.h"

using namespace Eigen;

//=============================================================================
// 常量定义
//=============================================================================
constexpr int DOF = 6;
constexpr double G = 9.8;

// 控制参数
constexpr double RECORD_RATE = 500.0;       // 记录频率 Hz
constexpr double RECORD_DURATION = 20.0;    // 记录时长 秒
constexpr double RETURN_SPEED = 0.3;        // 返回速度比例
constexpr int RETURN_STEPS = 1000;          // 返回初始位置的步数

// 电机参数
const double TAU_MAX[DOF] = {28.0, 28.0, 28.0, 10.0, 10.0, 10.0};
const double JOINT_LOWER[DOF] = {-2.7, -3.3, -0.2, -3.2, -1.5, -0.2};
const double JOINT_UPPER[DOF] = { 2.7,  0.2,  2.2,  3.2,  1.5,  1.3};
const double MASS[DOF] = {0.138, 1.379, 0.727, 0.370, 0.391, 0.179};

const Vector3d COM[DOF] = {
    Vector3d(0.0027728, 0.0, 0.017932),
    Vector3d(-0.0035914, -0.14996, 0.0),
    Vector3d(-0.00257, 0.09923, 0.051397),
    Vector3d(0.0034984, 0.028153, 0.0),
    Vector3d(-0.00068876, 0.076386, 0.0),
    Vector3d(0.0, 0.0065, 0.0)
};

// 电机方向系数
const int MOTOR_DIR[DOF] = {1, -1, 1, -1, -1, 1};

// 回放PD控制参数
const double KP[DOF] = {25.0, 52.0, 65.0, 40.0, 10.0, 10.0};
const double KD[DOF] = {1.5, 1.0, 1.5, 1.0, 1.0, 1.0};

//=============================================================================
// 数据结构
//=============================================================================
struct JointState {
    double position;
    double velocity;
};

struct JointFrame {
    double timestamp;
    std::array<JointState, DOF> joints;
};

//=============================================================================
// 全局变量
//=============================================================================
std::atomic<bool> g_running{true};
std::shared_ptr<damiao::Motor_Control> g_motor_ctrl;

// 轨迹数据存储
std::vector<JointFrame> g_motion_buffer;
std::array<double, DOF> g_initial_positions;

// 控制参数
double g_damping = 0.5;
double g_scale = 1.0;
bool g_debug_mode = true;

void signalHandler(int) { g_running = false; }

inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

//=============================================================================
// 旋转矩阵辅助函数
//=============================================================================
Matrix3d rotX(double t) {
    Matrix3d R; double c=cos(t), s=sin(t);
    R << 1,0,0, 0,c,-s, 0,s,c; return R;
}
Matrix3d rotY(double t) {
    Matrix3d R; double c=cos(t), s=sin(t);
    R << c,0,s, 0,1,0, -s,0,c; return R;
}
Matrix3d rotZ(double t) {
    Matrix3d R; double c=cos(t), s=sin(t);
    R << c,-s,0, s,c,0, 0,0,1; return R;
}

//=============================================================================
// 重力补偿计算
//=============================================================================
void computeGravityCompensation(const std::array<double, DOF>& q, 
                                 std::array<double, DOF>& tau_g) {
    tau_g.fill(0.0);
    
    Matrix4d T[DOF + 1];
    T[0] = Matrix4d::Identity();
    
    // J1: Z轴
    Matrix4d T1 = Matrix4d::Identity();
    T1.block<3,3>(0,0) = rotZ(q[0]);
    T1(2,3) = 0.06475;
    T[1] = T[0] * T1;
    
    // J2: X轴
    Matrix4d T2 = Matrix4d::Identity();
    T2.block<3,3>(0,0) = rotX(q[1]);
    T2(0,3) = 0.00325; T2(2,3) = 0.04575;
    T[2] = T[1] * T2;
    
    // J3: X轴
    Matrix4d T3 = Matrix4d::Identity();
    T3.block<3,3>(0,0) = rotX(q[2]);
    T3(1,3) = -0.3;
    T[3] = T[2] * T3;
    
    // J4: -Y轴
    Matrix4d T4 = Matrix4d::Identity();
    T4.block<3,3>(0,0) = rotY(-q[3]);
    T4(0,3) = -0.00325; T4(1,3) = 0.218; T4(2,3) = 0.065;
    T[4] = T[3] * T4;
    
    // J5: X轴
    Matrix4d T5 = Matrix4d::Identity();
    T5.block<3,3>(0,0) = rotX(q[4]);
    T5(0,3) = -0.00275; T5(1,3) = 0.036;
    T[5] = T[4] * T5;
    
    // J6: -Y轴
    Matrix4d T6 = Matrix4d::Identity();
    T6.block<3,3>(0,0) = rotY(-q[5]);
    T6(0,3) = 0.00275; T6(1,3) = 0.082;
    T[6] = T[5] * T6;
    
    Vector3d axis[DOF] = {
        {0,0,1}, {1,0,0}, {1,0,0}, {0,-1,0}, {1,0,0}, {0,-1,0}
    };
    
    Vector3d g_world(0, 0, -G);
    
    for (int link = 0; link < DOF; link++) {
        Vector4d com_local(COM[link](0), COM[link](1), COM[link](2), 1.0);
        Vector4d com_world = T[link + 1] * com_local;
        Vector3d p_com(com_world(0), com_world(1), com_world(2));
        Vector3d F = MASS[link] * g_world;
        
        for (int j = 0; j <= link; j++) {
            Vector3d p_joint(T[j+1](0,3), T[j+1](1,3), T[j+1](2,3));
            Vector3d r = p_com - p_joint;
            Vector3d torque = r.cross(F);
            Vector3d axis_world = T[j+1].block<3,3>(0,0) * axis[j];
            tau_g[j] += torque.dot(axis_world);
        }
    }
    
    for (int i = 0; i < DOF; i++)
        tau_g[i] = -tau_g[i];
}

//=============================================================================
// 电机控制函数
//=============================================================================
bool initMotors() {
    std::vector<damiao::DmActData> data;
    for (int i = 0; i < 3; ++i)
        data.push_back({damiao::DM4340, damiao::MIT_MODE, 
                       (uint16_t)(i+1), (uint16_t)(0x11+i)});
    for (int i = 3; i < 6; ++i)
        data.push_back({damiao::DM4310, damiao::MIT_MODE, 
                       (uint16_t)(i+1), (uint16_t)(0x11+i)});
    try {
        g_motor_ctrl = std::make_shared<damiao::Motor_Control>(
            1000000, 5000000, "14AA044B241402B10DDBDAFE448040BB", &data);
        ros::Duration(0.5).sleep();
        for (int i = 0; i < DOF; ++i) {
            g_motor_ctrl->switchControlMode(
                *g_motor_ctrl->getMotor(i+1), damiao::MIT);
            usleep(5000);
        }
        return true;
    } catch (...) { return false; }
}

void readMotors(std::array<double, DOF>& pos, std::array<double, DOF>& vel) {
    for (int i = 0; i < DOF; ++i)
        g_motor_ctrl->refresh_motor_status(*g_motor_ctrl->getMotor(i+1));
    usleep(500);
    for (int i = 0; i < DOF; ++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) {
            pos[i] = m->Get_Position() * MOTOR_DIR[i];
            vel[i] = m->Get_Velocity() * MOTOR_DIR[i];
        }
    }
}

void sendTorque(const std::array<double, DOF>& tau) {
    for (int i = 0; i < DOF; ++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) {
            double t = tau[i] * MOTOR_DIR[i];
            t = clamp(t, -TAU_MAX[i], TAU_MAX[i]);
            g_motor_ctrl->control_mit(*m, 0, 0, 0, 0, t);
        }
    }
}

void sendMIT(int joint_idx, double kp, double kd, double pos, double vel, double tau) {
    auto m = g_motor_ctrl->getMotor(joint_idx + 1);
    if (m) {
        double t = tau * MOTOR_DIR[joint_idx];
        double p = pos * MOTOR_DIR[joint_idx];
        double v = vel * MOTOR_DIR[joint_idx];
        t = clamp(t, -TAU_MAX[joint_idx], TAU_MAX[joint_idx]);
        g_motor_ctrl->control_mit(*m, kp, kd, p, v, t);
    }
}

void sendZero() {
    for (int i = 0; i < DOF; ++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) g_motor_ctrl->control_mit(*m, 0, 0, 0, 0, 0);
    }
}

void switchToMIT() {
    for (int i = 0; i < DOF; ++i) {
        g_motor_ctrl->switchControlMode(
            *g_motor_ctrl->getMotor(i+1), damiao::MIT);
        usleep(5000);
    }
}

void switchToPosVel() {
    for (int i = 0; i < DOF; ++i) {
        g_motor_ctrl->switchControlMode(
            *g_motor_ctrl->getMotor(i+1), damiao::POS_VEL);
        usleep(5000);
    }
}

//=============================================================================
// 安全检查
//=============================================================================
bool checkSafety(const std::array<double, DOF>& q) {
    for (int i = 0; i < DOF; ++i) {
        if (q[i] < JOINT_LOWER[i] || q[i] > JOINT_UPPER[i]) {
            ROS_ERROR("J%d=%.2f out of bounds [%.2f, %.2f]!", 
                     i+1, q[i], JOINT_LOWER[i], JOINT_UPPER[i]);
            return false;
        }
    }
    return true;
}

//=============================================================================
// 阶段1：返回初始位置
//=============================================================================
void returnToInitial(bool slow_return = true) {
    ROS_INFO("Returning to initial positions...");
    
    std::array<double, DOF> q, dq;
    readMotors(q, dq);
    
    ros::Rate rate(200);
    
    for (int step = 0; step < RETURN_STEPS && ros::ok() && g_running; ++step) {
        readMotors(q, dq);
        
        // 计算重力补偿
        std::array<double, DOF> tau_g;
        computeGravityCompensation(q, tau_g);
        
        double ratio = slow_return ? 
            static_cast<double>(step + 1) / RETURN_STEPS : 1.0;
        
        for (int i = 0; i < DOF; ++i) {
            // 线性插值计算目标位置
            double target_pos = g_initial_positions[i] * ratio + q[i] * (1.0 - ratio);
            
            // 使用MIT模式带PD控制回到初始位置
            sendMIT(i, KP[i], KD[i], target_pos, 0.0, g_scale * tau_g[i]);
        }
        
        rate.sleep();
    }
    
    ROS_INFO("Return to initial completed.");
    ros::Duration(0.5).sleep();
}

//=============================================================================
// 阶段2：示教记录（零重力拖动模式）
//=============================================================================
void recordPhase() {
    ROS_INFO("========================================");
    ROS_INFO("  Start recording for %.1f seconds", RECORD_DURATION);
    ROS_INFO("  Drag the robot freely...");
    ROS_INFO("========================================");
    
    g_motion_buffer.clear();
    
    // 确保在MIT模式（力矩控制模式，便于被动拖动）
    switchToMIT();
    
    // 软启动
    const int RAMP_STEPS = 500;
    int ramp_count = 0;
    
    ros::Rate rate(RECORD_RATE);
    ros::Time start_time = ros::Time::now();
    ros::Time last_print_time = start_time;
    
    std::array<double, DOF> q, dq;
    
    while ((ros::Time::now() - start_time).toSec() < RECORD_DURATION && 
           ros::ok() && g_running) {
        
        readMotors(q, dq);
        
        // 安全检查
        if (!checkSafety(q)) {
            sendZero();
            ROS_ERROR("Safety limit reached during recording!");
            break;
        }
        
        // 计算重力补偿
        std::array<double, DOF> tau_g;
        computeGravityCompensation(q, tau_g);
        
        // 合成力矩：重力补偿 + 简单阻尼
        std::array<double, DOF> tau_cmd;
        for (int i = 0; i < DOF; ++i) {
            tau_cmd[i] = g_scale * tau_g[i] - g_damping * dq[i];
        }
        
        // 软启动
        double ramp = 1.0;
        if (ramp_count < RAMP_STEPS) {
            ramp = static_cast<double>(ramp_count) / RAMP_STEPS;
            ramp_count++;
        }
        for (int i = 0; i < DOF; ++i) tau_cmd[i] *= ramp;
        
        // 发送力矩
        sendTorque(tau_cmd);
        
        // 构建并存储当前帧数据
        JointFrame frame;
        frame.timestamp = (ros::Time::now() - start_time).toSec();
        for (int i = 0; i < DOF; ++i) {
            frame.joints[i].position = q[i];
            frame.joints[i].velocity = dq[i];
        }
        g_motion_buffer.push_back(frame);
        
        // 每秒打印进度
        if ((ros::Time::now() - last_print_time).toSec() >= 1.0) {
            ROS_INFO("Recording: %.1f/%.1f s | q:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                     frame.timestamp, RECORD_DURATION,
                     q[0], q[1], q[2], q[3], q[4], q[5]);
            last_print_time = ros::Time::now();
        }
        
        rate.sleep();
    }
    
    sendZero();
    ROS_INFO("Recording completed. Saved %lu frames", g_motion_buffer.size());
}

//=============================================================================
// 阶段3：轨迹回放
//=============================================================================
void replayPhase() {
    if (g_motion_buffer.empty()) {
        ROS_WARN("No recorded data to replay!");
        return;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("  Starting motion replay...");
    ROS_INFO("  Total frames: %lu", g_motion_buffer.size());
    ROS_INFO("========================================");
    
    // 确保在MIT模式
    switchToMIT();
    
    ros::Rate rate(RECORD_RATE);
    size_t frame_counter = 0;
    std::array<double, DOF> q, dq;
    
    while (frame_counter < g_motion_buffer.size() && ros::ok() && g_running) {
        readMotors(q, dq);
        
        // 安全检查
        if (!checkSafety(q)) {
            sendZero();
            ROS_ERROR("Safety limit reached during replay!");
            break;
        }
        
        const auto& frame = g_motion_buffer[frame_counter];
        
        // 计算当前位置的重力补偿
        std::array<double, DOF> tau_g;
        computeGravityCompensation(q, tau_g);
        
        // 使用MIT模式带PD控制进行轨迹跟踪
        for (int i = 0; i < DOF; ++i) {
            sendMIT(i, KP[i], KD[i], 
                   frame.joints[i].position, 
                   frame.joints[i].velocity, 
                   g_scale * tau_g[i]);
        }
        
        // 定时打印状态
        if (frame_counter % 500 == 0) {
            ROS_INFO("Replay: frame %lu/%lu | pos:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                     frame_counter, g_motion_buffer.size(),
                     q[0], q[1], q[2], q[3], q[4], q[5]);
        }
        
        ++frame_counter;
        rate.sleep();
    }
    
    sendZero();
    ROS_INFO("Replay completed.");
}

//=============================================================================
// 保存轨迹数据到CSV文件
//=============================================================================
void saveRecording(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }
    
    // 写入表头
    file << "timestamp";
    for (int i = 0; i < DOF; ++i) {
        file << ",j" << (i+1) << "_pos,j" << (i+1) << "_vel";
    }
    file << "\n";
    
    // 写入数据
    for (const auto& frame : g_motion_buffer) {
        file << frame.timestamp;
        for (int i = 0; i < DOF; ++i) {
            file << "," << frame.joints[i].position 
                 << "," << frame.joints[i].velocity;
        }
        file << "\n";
    }
    
    file.close();
    ROS_INFO("Data saved to %s", filename.c_str());
}

//=============================================================================
// 从CSV文件加载轨迹数据
//=============================================================================
bool loadRecording(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return false;
    }
    
    g_motion_buffer.clear();
    
    std::string line;
    std::getline(file, line); // 跳过表头
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        JointFrame frame;
        char comma;
        
        ss >> frame.timestamp;
        for (int i = 0; i < DOF; ++i) {
            ss >> comma >> frame.joints[i].position 
               >> comma >> frame.joints[i].velocity;
        }
        g_motion_buffer.push_back(frame);
    }
    
    file.close();
    ROS_INFO("Loaded %lu frames from %s", g_motion_buffer.size(), filename.c_str());
    return !g_motion_buffer.empty();
}

//=============================================================================
// 主函数
//=============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "teach_replay_node");
    ros::NodeHandle nh("~");
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 读取参数
    double rate;
    std::string save_file, load_file;
    bool skip_record = false;
    
    nh.param("damping", g_damping, 0.5);
    nh.param("scale", g_scale, 1.0);
    nh.param("rate", rate, 500.0);
    nh.param("debug", g_debug_mode, true);
    nh.param("save_file", save_file, std::string("motion_data.csv"));
    nh.param("load_file", load_file, std::string(""));
    nh.param("skip_record", skip_record, false);
    
    ROS_INFO("========================================");
    ROS_INFO("  Teach & Replay Node");
    ROS_INFO("========================================");
    ROS_INFO("  damping:    %.2f", g_damping);
    ROS_INFO("  scale:      %.2f", g_scale);
    ROS_INFO("  rate:       %.0f Hz", rate);
    ROS_INFO("  save_file:  %s", save_file.c_str());
    ROS_INFO("  load_file:  %s", load_file.empty() ? "none" : load_file.c_str());
    ROS_INFO("  skip_record: %s", skip_record ? "true" : "false");
    ROS_INFO("========================================");
    
    // 初始化电机
    if (!initMotors()) {
        ROS_ERROR("Motor initialization failed!");
        return -1;
    }
    ROS_INFO("Motors initialized successfully.");
    
    // 发布关节状态
    ros::Publisher pub_js = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
    // 设置初始位置（零位）
    g_initial_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    // 如果指定了加载文件，则加载轨迹数据
    if (!load_file.empty()) {
        if (loadRecording(load_file)) {
            skip_record = true;
        }
    }
    
    //=========================================================================
    // 主执行流程
    //=========================================================================
    
    // 阶段0：返回初始位置
    ROS_INFO("\n[Phase 0] Return to initial position...");
    returnToInitial(true);
    ros::Duration(1.0).sleep();
    
    // 阶段1：示教记录（如果未跳过）
    if (!skip_record) {
        ROS_INFO("\n[Phase 1] Recording phase - drag the robot freely...");
        ROS_INFO("Press Enter to start recording...");
        std::cin.get();
        recordPhase();
        ros::Duration(1.0).sleep();
    }
    
    // 阶段2：返回初始位置
    ROS_INFO("\n[Phase 2] Return to initial position...");
    returnToInitial(true);
    ros::Duration(1.0).sleep();
    
    // 阶段3：轨迹回放
    ROS_INFO("\n[Phase 3] Replay phase...");
    ROS_INFO("Press Enter to start replay...");
    std::cin.get();
    replayPhase();
    ros::Duration(1.0).sleep();
    
    // 阶段4：返回初始位置
    ROS_INFO("\n[Phase 4] Return to initial position...");
    returnToInitial(true);
    
    // 保存轨迹数据
    if (!skip_record && !save_file.empty()) {
        saveRecording(save_file);
    }
    
    // 安全停止
    sendZero();
    ROS_INFO("Teach & Replay node stopped.");
    
    return 0;
}