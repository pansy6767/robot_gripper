/**
 * @file zero_force_control.cpp
 * @brief 零力控制(重力补偿) - 方案A修复版
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <cmath>
#include <array>
#include <atomic>

#include "dmbot_serial/protocol/damiao.h"

//=============================================================================
// 常量
//=============================================================================
constexpr int DOF = 6;
constexpr double G = 9.81;

const int DIR[DOF] = {1, -1, -1, -1, -1, 1};
const double TAU_MAX[DOF] = {28.0, 28.0, 28.0, 10.0, 10.0, 10.0};

const double JOINT_LOWER[DOF] = {-1.57, -3.14, -2.0,  -3.0,  -1.28, -0.5};
const double JOINT_UPPER[DOF] = { 1.57,  0.0,   2.0,   3.0,   1.28,  1.1};

const double MASS[DOF] = {0.138, 1.379, 0.727, 0.370, 0.391, 0.179};

const double COM[DOF][3] = {
    {0.0027728,  0.0,       0.017932},
    {-0.0035914, -0.14996,  0.0},
    {-0.00257,   0.09923,   0.051397},
    {0.0034984,  0.028153,  0.0},
    {-0.00068876, 0.076386, 0.0},
    {0.0,        0.0065,    0.0}
};

const double JOINT_OFFSET[DOF][3] = {
    {0.0,      0.0,    0.06475},
    {0.00325,  0.0,    0.04575},
    {0.0,     -0.3,    0.0},
    {-0.00325, 0.218,  0.065},
    {-0.00275, 0.036,  0.0},
    {0.00275,  0.082,  0.0}
};

const int JOINT_AXIS[DOF] = {3, 1, 1, -2, 1, -2};

//=============================================================================
// 全局变量
//=============================================================================
std::atomic<bool> g_running{true};
std::shared_ptr<damiao::Motor_Control> g_motor_ctrl;
std::array<double, DOF> g_pos{}, g_vel{}, g_tau{};

int g_debug_count = 0;
bool g_debug_mode = true;
double g_damping = 0.3;
double g_scale = 1.0;
bool g_friction_enable = false;  // ★★★ 默认关闭摩擦力补偿 ★★★

// 摩擦力补偿参数 (减小数值，避免突然运动)
double g_friction_offset[DOF] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double g_friction_vel_coef[DOF] = {0.02, 0.02, 0.02, 0.02, 0.02, 0.0};
double g_friction_threshold = 0.01;

void signalHandler(int) { 
    ROS_WARN("Shutdown..."); 
    g_running = false; 
}

//=============================================================================
// 工具函数
//=============================================================================
inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void matMul(const double A[4][4], const double B[4][4], double C[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 4; ++k) C[i][j] += A[i][k] * B[k][j];
        }
}

void computeTransform(int idx, double q, double T[4][4]) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T[i][j] = (i == j) ? 1.0 : 0.0;

    T[0][3] = JOINT_OFFSET[idx][0];
    T[1][3] = JOINT_OFFSET[idx][1];
    T[2][3] = JOINT_OFFSET[idx][2];

    double c = cos(q), s = sin(q);
    switch (JOINT_AXIS[idx]) {
        case  1: T[1][1]=c; T[1][2]=-s; T[2][1]=s; T[2][2]=c; break;
        case -1: T[1][1]=c; T[1][2]=s; T[2][1]=-s; T[2][2]=c; break;
        case  2: T[0][0]=c; T[0][2]=s; T[2][0]=-s; T[2][2]=c; break;
        case -2: T[0][0]=c; T[0][2]=-s; T[2][0]=s; T[2][2]=c; break;
        case  3: T[0][0]=c; T[0][1]=-s; T[1][0]=s; T[1][1]=c; break;
        case -3: T[0][0]=c; T[0][1]=s; T[1][0]=-s; T[1][1]=c; break;
    }
}

//=============================================================================
// 重力补偿
//=============================================================================
void computeGravity(const std::array<double,DOF>& q, std::array<double,DOF>& tau_g) {
    tau_g.fill(0.0);
    double T[DOF+1][4][4];
    
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) T[0][i][j]=(i==j)?1.0:0.0;
    
    for (int i = 0; i < DOF; ++i) {
        double Ti[4][4];
        computeTransform(i, q[i], Ti);
        matMul(T[i], Ti, T[i+1]);
    }
    
    for (int link = 0; link < DOF; ++link) {
        double cl[4] = {COM[link][0], COM[link][1], COM[link][2], 1.0};
        double cw[3] = {0,0,0};
        for (int i=0;i<3;++i) for (int j=0;j<4;++j) cw[i] += T[link+1][i][j]*cl[j];
        
        double Fz = -MASS[link] * G;
        
        for (int jt = 0; jt <= link; ++jt) {
            int ax = JOINT_AXIS[jt];
            int col = abs(ax) - 1;
            double sg = (ax > 0) ? 1.0 : -1.0;
            
            double z[3] = {sg*T[jt][0][col], sg*T[jt][1][col], sg*T[jt][2][col]};
            double r[3] = {cw[0]-T[jt+1][0][3], cw[1]-T[jt+1][1][3], cw[2]-T[jt+1][2][3]};
            
            double tau_contrib = z[0]*(r[1]*Fz) + z[1]*(-r[0]*Fz);
            tau_g[jt] += tau_contrib;
        }
    }
    
    if (g_debug_mode && g_debug_count % 500 == 0) {
        ROS_INFO("tau_g raw: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 tau_g[0], tau_g[1], tau_g[2], tau_g[3], tau_g[4], tau_g[5]);
    }
    
    // 关节3取反
    tau_g[2] = -tau_g[2];
    
    // 应用方向系数
    for (int i = 0; i < DOF; ++i) tau_g[i] *= DIR[i];
    
    if (g_debug_mode && g_debug_count % 500 == 0) {
        ROS_INFO("tau_g final: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 tau_g[0], tau_g[1], tau_g[2], tau_g[3], tau_g[4], tau_g[5]);
    }
}

//=============================================================================
// 摩擦力补偿 (简化版，只补偿粘性摩擦)
//=============================================================================
void computeFriction(const std::array<double,DOF>& vel, std::array<double,DOF>& tau_f) {
    tau_f.fill(0.0);
    
    if (!g_friction_enable) return;
    
    for (int i = 0; i < DOF; ++i) {
        // 只有速度超过阈值才补偿
        if (fabs(vel[i]) > g_friction_threshold) {
            tau_f[i] = g_friction_vel_coef[i] * vel[i];
        }
    }
}

//=============================================================================
// 电机
//=============================================================================
bool initMotors() {
    std::vector<damiao::DmActData> data;
    for (int i=0;i<3;++i) data.push_back({damiao::DM4340,damiao::MIT_MODE,(uint16_t)(i+1),(uint16_t)(0x11+i)});
    for (int i=3;i<6;++i) data.push_back({damiao::DM4310,damiao::MIT_MODE,(uint16_t)(i+1),(uint16_t)(0x11+i)});

    try {
        g_motor_ctrl = std::make_shared<damiao::Motor_Control>(
            1000000, 5000000, "14AA044B241402B10DDBDAFE448040BB", &data);
        ros::Duration(0.5).sleep();
        for (int i=0;i<DOF;++i) {
            g_motor_ctrl->switchControlMode(*g_motor_ctrl->getMotor(i+1), damiao::MIT);
            usleep(5000);
        }
        ROS_INFO("Motors OK");
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Motor init failed: %s", e.what());
        return false;
    }
}

void readMotors() {
    for (int i=0;i<DOF;++i) g_motor_ctrl->refresh_motor_status(*g_motor_ctrl->getMotor(i+1));
    usleep(500);
    for (int i=0;i<DOF;++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) {
            g_pos[i] = m->Get_Position() * DIR[i];
            g_vel[i] = m->Get_Velocity() * DIR[i];
            g_tau[i] = m->Get_tau() * DIR[i];
        }
    }
}

void sendTorque(const std::array<double,DOF>& tau) {
    for (int i=0;i<DOF;++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) {
            double t = clamp(tau[i], -TAU_MAX[i], TAU_MAX[i]);
            g_motor_ctrl->control_mit(*m, 0,0,0,0, t*DIR[i]);
        }
    }
}

void sendZero() {
    for (int i=0;i<DOF;++i) {
        auto m = g_motor_ctrl->getMotor(i+1);
        if (m) g_motor_ctrl->control_mit(*m, 0,0,0,0,0);
    }
}

//=============================================================================
// 主函数
//=============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "zero_force_node");
    ros::NodeHandle nh("~");
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    double rate;
    nh.param("damping", g_damping, 0.3);
    nh.param("scale", g_scale, 1.0);
    nh.param("rate", rate, 500.0);
    nh.param("debug", g_debug_mode, true);
    nh.param("friction", g_friction_enable, false);

    ROS_INFO("========================================");
    ROS_INFO("  Zero-Force Control (Plan A v2)");
    ROS_INFO("========================================");
    ROS_INFO("  damping:  %.2f", g_damping);
    ROS_INFO("  scale:    %.2f", g_scale);
    ROS_INFO("  rate:     %.0f Hz", rate);
    ROS_INFO("  friction: %s", g_friction_enable ? "ON" : "OFF");
    ROS_INFO("----------------------------------------");

    if (!initMotors()) return -1;

    ros::Publisher pub_js = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
    sensor_msgs::JointState js;
    js.name = {"joint1","joint2","joint3","joint4","joint5","joint6"};
    js.position.resize(DOF);
    js.velocity.resize(DOF);
    js.effort.resize(DOF);

    ROS_INFO("========================================");
    ROS_INFO("  RUNNING - Drag the arm!");
    ROS_INFO("  Ctrl+C to stop");
    ROS_INFO("========================================");

    ros::Rate loop(rate);
    
    while (ros::ok() && g_running) {
        readMotors();
        g_debug_count++;
        
        // 安全检查
        bool emergency = false;
        for (int i=0;i<DOF;++i) {
            if (g_pos[i] < JOINT_LOWER[i]-0.1 || g_pos[i] > JOINT_UPPER[i]+0.1) {
                ROS_ERROR("J%d pos %.2f out of limit!", i+1, g_pos[i]);
                emergency = true;
            }
            if (fabs(g_vel[i]) > 35.0) {
                ROS_ERROR("J%d vel %.2f too fast!", i+1, g_vel[i]);
                emergency = true;
            }
        }
        if (emergency) { sendZero(); break; }
        
        // 计算力矩
        std::array<double,DOF> tau_g, tau_f, tau_cmd;
        computeGravity(g_pos, tau_g);
        computeFriction(g_vel, tau_f);
        
        for (int i=0;i<DOF;++i) {
            tau_cmd[i] = g_scale * tau_g[i] + tau_f[i] - g_damping * g_vel[i];
            tau_cmd[i] = clamp(tau_cmd[i], -TAU_MAX[i], TAU_MAX[i]);
        }
        
        if (g_debug_mode && g_debug_count % 500 == 0) {
            ROS_INFO("pos: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                     g_pos[0], g_pos[1], g_pos[2], g_pos[3], g_pos[4], g_pos[5]);
            ROS_INFO("tau: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                     tau_cmd[0], tau_cmd[1], tau_cmd[2], tau_cmd[3], tau_cmd[4], tau_cmd[5]);
        }
        
        sendTorque(tau_cmd);
        
        js.header.stamp = ros::Time::now();
        for (int i=0;i<DOF;++i) {
            js.position[i] = g_pos[i];
            js.velocity[i] = g_vel[i];
            js.effort[i] = tau_cmd[i];
        }
        pub_js.publish(js);
        
        ros::spinOnce();
        loop.sleep();
    }

    ROS_WARN("Stopping...");
    sendZero();
    ros::Duration(0.01).sleep();
    ROS_INFO("Stopped.");
    return 0;
}