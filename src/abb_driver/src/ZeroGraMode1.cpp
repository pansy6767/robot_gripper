#include "ros/ros.h"
#include "abb_driver/6Dof_dyna.h"
#include "dmbot_serial/protocol/damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <thread>
#include <boost/thread/shared_mutex.hpp>
#include "sensor_msgs/JointState.h"

// 电机控制接口 (使用shared_ptr，在main中初始化)
std::shared_ptr<damiao::Motor_Control> motorsInterface;

// 共享数据保护
boost::shared_mutex data_mutex_;
double q[6] = {0.0};      // 关节位置
double vel[6] = {0.0};    // 关节速度
double tau_ff[6] = {0.0}; // 摩擦力补偿力矩

// 电机方向 (根据AbbRobot.h的定义)
const std::vector<int> directionMotor = {1, -1, 1, -1, -1, 1};

// 关节限位 (从URDF提取)
const double JOINT_LIMITS[6][2] = {
    {-1.57, 1.57},   // Joint 1
    {-3.14, 0.0},    // Joint 2
    {0.0, 2.0},      // Joint 3
    {-3.0, 3.0},     // Joint 4
    {-1.28, 1.28},   // Joint 5
    {0.0, 1.1}       // Joint 6
};

// 摩擦力补偿参数 (可根据实际情况调整)
const double FRICTION_VEL_THRESHOLD = 0.006;  // 速度阈值
const double FRICTION_COEFF[6] = {
    0.06,   // Joint 1
    0.05,   // Joint 2
    0.10,   // Joint 3
    0.20,   // Joint 4
    0.05,   // Joint 5
    0.03    // Joint 6
};
const double FRICTION_STATIC[6] = {
    0.0,    // Joint 1
    0.10,   // Joint 2
    0.20,   // Joint 3
    0.30,   // Joint 4
    0.03,   // Joint 5
    0.0     // Joint 6
};

// 函数声明
void gravityCompensationThread(ros::Rate rate);
void jointStatePublisherThread(ros::Rate rate, ros::Publisher* joint_state_pub);
void calculateFrictionCompensation();
void clampJointPosition(double* q_array);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ZeroGraMode");
    ros::NodeHandle nh;
    
    ROS_INFO("Initializing Zero Gravity Mode...");
    
    // 创建关节状态发布者
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/abb/joint_states", 10);
    
    // 初始化电机控制接口 (使用CAN-FD通信)
    uint32_t nom_baud = 1000000;  // 仲裁域 1M
    uint32_t dat_baud = 5000000;  // 数据域 5M
    
    std::vector<damiao::DmActData> init_data;
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id = 0x01,
        .mst_id = 0x11
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id = 0x02,
        .mst_id = 0x12
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id = 0x03,
        .mst_id = 0x13
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id = 0x04,
        .mst_id = 0x14
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id = 0x05,
        .mst_id = 0x15
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id = 0x06,
        .mst_id = 0x16
    });
    
    ROS_INFO("Attempting to initialize motor interface...");
    try {
        motorsInterface = std::make_shared<damiao::Motor_Control>(
            nom_baud, 
            dat_baud, 
            "14AA044B241402B10DDBDAFE448040BB",  // CAN设备ID
            &init_data
        );
        ROS_INFO("Motor interface initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize motor interface: %s", e.what());
        ROS_ERROR("Please check:");
        ROS_ERROR("  1. Is another program using the CAN device?");
        ROS_ERROR("  2. Run: killall -9 abb_driver");
        ROS_ERROR("  3. Check USB permissions: sudo chmod 666 /dev/bus/usb/*/*");
        return -1;
    }
    
    ROS_INFO("Motors initialized, waiting for stabilization...");
    usleep(1000000);  // 等待1秒
    
    // 先禁用所有电机，确保干净状态
    ROS_INFO("Disabling all motors first...");
    motorsInterface->disable_all();
    usleep(500000);
    
    // 使能所有电机
    ROS_INFO("Enabling all motors...");
    motorsInterface->enable_all();
    usleep(2000000);  // 等待2秒让电机稳定
    
    // 读取初始位置
    ROS_INFO("Reading initial positions...");
    for (int i = 1; i <= 6; i++) {
        motorsInterface->refresh_motor_status(*motorsInterface->getMotor(i));
        double pos = motorsInterface->getMotor(i)->Get_Position();
        double vel = motorsInterface->getMotor(i)->Get_Velocity();
        double tau = motorsInterface->getMotor(i)->Get_tau();
        ROS_INFO("Motor %d - Pos: %.3f, Vel: %.3f, Tau: %.3f", i, pos, vel, tau);
    }
    
    ROS_INFO("All motors enabled, starting zero gravity mode...");
    
    // 创建线程
    ros::Rate gc_rate(500);  // 500Hz for gravity compensation
    ros::Rate js_rate(100);  // 100Hz for joint state publishing
    
    std::thread gc_thread(gravityCompensationThread, gc_rate);
    std::thread js_thread(jointStatePublisherThread, js_rate, &joint_state_pub);
    
    // 等待线程结束
    gc_thread.join();
    js_thread.join();
    
    ROS_INFO("Shutting down zero gravity mode...");
    
    return 0;
}


// 重力补偿线程
void gravityCompensationThread(ros::Rate rate) 
{
    ROS_INFO("Gravity compensation thread started");
    
    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        
        // 刷新所有电机状态
        for (int i = 1; i <= 6; i++) {
            motorsInterface->refresh_motor_status(*motorsInterface->getMotor(i));
        }
        
        // 读取关节位置和速度 (考虑电机方向)
        {
            boost::unique_lock<boost::shared_mutex> lock(data_mutex_);
            for (int i = 0; i < 6; i++) {
                double raw_pos = motorsInterface->getMotor(i + 1)->Get_Position();
                double raw_vel = motorsInterface->getMotor(i + 1)->Get_Velocity();
                
                q[i] = raw_pos * directionMotor[i];
                vel[i] = raw_vel * directionMotor[i];
            }
        }
        
        // 检查关节位置是否在安全范围内（只警告）
        clampJointPosition(q);
        
        // 计算摩擦力补偿
        calculateFrictionCompensation();
        
        // 计算重力补偿力矩
        VectorXd tau_gravity = compute_gravity_compensation(q);
        
        // 总力矩 = 重力补偿 + 摩擦补偿
        double tau_send[6];  // 实际发送的力矩
        for (int i = 0; i < 6; i++) {
            double tau_total_i = tau_gravity[i] + tau_ff[i];
            tau_send[i] = tau_total_i * directionMotor[i];
            
            // 限制力矩范围，防止过大
            const double MAX_TORQUE = 10.0;
            if (tau_send[i] > MAX_TORQUE) tau_send[i] = MAX_TORQUE;
            if (tau_send[i] < -MAX_TORQUE) tau_send[i] = -MAX_TORQUE;
        }
        
        // 发送力矩指令到电机
        for (int i = 0; i < 6; i++) {
            // 读取当前位置
            double current_pos = motorsInterface->getMotor(i + 1)->Get_Position();
            
            // MIT模式控制
            motorsInterface->control_mit(
                *motorsInterface->getMotor(i + 1),
                0.0,           // kp
                0.5,           // kd
                current_pos,   // q
                0.0,           // dq
                tau_send[i]    // tau
            );
        }
        
        // 计算控制频率
        ros::Time current_time1 = ros::Time::now();
        double time_diff = (current_time1 - current_time).toSec();
        
        // 每隔一段时间打印一次信息
        static int counter = 0;
        if (counter++ % 100 == 0) {
            ROS_INFO("GC Freq: %.1f Hz", 1.0 / time_diff);
            ROS_INFO("Tau: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                     tau_gravity[0], tau_gravity[1], tau_gravity[2], 
                     tau_gravity[3], tau_gravity[4], tau_gravity[5]);
            ROS_INFO("Pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                     q[0], q[1], q[2], q[3], q[4], q[5]);
        }
        
        // 每500次循环打印一次调试信息
        static int debug_counter = 0;
        if (debug_counter++ % 500 == 0) {
            ROS_INFO("Raw pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                     motorsInterface->getMotor(1)->Get_Position(),
                     motorsInterface->getMotor(2)->Get_Position(),
                     motorsInterface->getMotor(3)->Get_Position(),
                     motorsInterface->getMotor(4)->Get_Position(),
                     motorsInterface->getMotor(5)->Get_Position(),
                     motorsInterface->getMotor(6)->Get_Position());
            ROS_INFO("Send tau: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                     tau_send[0], tau_send[1], tau_send[2],
                     tau_send[3], tau_send[4], tau_send[5]);
        }
        
        rate.sleep();
    }
    
    ROS_INFO("Gravity compensation thread stopped");
}

// 关节状态发布线程
void jointStatePublisherThread(ros::Rate rate, ros::Publisher* joint_state_pub) 
{
    ROS_INFO("Joint state publisher thread started");
    
    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    joint_state.position.resize(6);
    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);
    
    while (ros::ok()) {
        // 线程安全地读取数据
        double positions[6];
        double velocities[6];
        double efforts[6];
        
        {
            boost::shared_lock<boost::shared_mutex> lock(data_mutex_);
            for (int i = 0; i < 6; i++) {
                positions[i] = q[i];
                velocities[i] = vel[i];
                
                // 读取实际力矩反馈
                double raw_tau = motorsInterface->getMotor(i + 1)->Get_tau();
                efforts[i] = raw_tau * directionMotor[i];
            }
        }
        
        // 更新并发布关节状态
        for (int i = 0; i < 6; i++) {
            joint_state.position[i] = positions[i];
            joint_state.velocity[i] = velocities[i];
            joint_state.effort[i] = efforts[i];
        }
        
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "/abb";
        joint_state_pub->publish(joint_state);
        
        rate.sleep();
    }
    
    ROS_INFO("Joint state publisher thread stopped");
}

// 计算摩擦力补偿
void calculateFrictionCompensation() 
{
    for (int i = 0; i < 6; i++) {
        if (std::abs(vel[i]) < FRICTION_VEL_THRESHOLD) {
            // 速度很小时，不补偿摩擦力（避免振荡）
            tau_ff[i] = 0.0;
        } else {
            // 库伦摩擦 + 粘性摩擦
            double sign = (vel[i] > 0) ? 1.0 : -1.0;
            tau_ff[i] = sign * FRICTION_STATIC[i] + FRICTION_COEFF[i] * vel[i];
        }
    }
}

// 检查关节位置是否在安全范围内（只警告，不修改）
void clampJointPosition(double* q_array) 
{
    static int warn_counter = 0;
    if (warn_counter++ % 500 != 0) return; // 降低警告频率
    
    for (int i = 0; i < 6; i++) {
        if (q_array[i] < JOINT_LIMITS[i][0]) {
            ROS_WARN_THROTTLE(5.0, "Joint %d (%.3f) below lower limit: %.3f", 
                             i + 1, q_array[i], JOINT_LIMITS[i][0]);
        } else if (q_array[i] > JOINT_LIMITS[i][1]) {
            ROS_WARN_THROTTLE(5.0, "Joint %d (%.3f) above upper limit: %.3f", 
                             i + 1, q_array[i], JOINT_LIMITS[i][1]);
        }
    }
}