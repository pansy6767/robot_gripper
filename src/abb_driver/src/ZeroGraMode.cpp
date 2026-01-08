#include "ros/ros.h"
#include "abb_driver/6Dof_dyna.h"
#include "dmbot_serial/protocol/damiao.h"
#include "unistd.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <thread>
#include <atomic>
#include <boost/thread/shared_mutex.hpp>
#include "sensor_msgs/JointState.h"

// 全局变量
boost::shared_mutex data_mutex_;
double q[6] = {0};
double vel[6] = {0};
double f[6] = {0};
VectorXd tau = VectorXd::Zero(6);

// 电机方向 (与AbbRobot.cpp一致)
std::vector<int> directionMotor = {1, -1, 1, -1, -1, 1};

// 电机控制接口
std::shared_ptr<damiao::Motor_Control> motorsInterface;

// 原子标志用于安全退出
std::atomic<bool> running(true);

// Function prototypes
void gravityCompensationThread(ros::Rate rate);
void jointStatePublisherThread(ros::Rate rate, ros::Publisher* joint_state_pub);
void signalHandler(int signum);

void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received. Shutting down...", signum);
    running = false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ZeroGraMode");
    ros::NodeHandle nh;
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    
    // 初始化电机配置 (与AbbRobot.cpp一致)
    uint16_t canid1 = 0x01, mstid1 = 0x11;
    uint16_t canid2 = 0x02, mstid2 = 0x12;
    uint16_t canid3 = 0x03, mstid3 = 0x13;
    uint16_t canid4 = 0x04, mstid4 = 0x14;
    uint16_t canid5 = 0x05, mstid5 = 0x15;
    uint16_t canid6 = 0x06, mstid6 = 0x16;
    
    uint32_t nom_baud = 1000000;  // 仲裁域 1M
    uint32_t dat_baud = 5000000;  // 数据域 5M
    
    std::vector<damiao::DmActData> init_data;
    
    // 电机1-3: DM4340 (大扭矩电机)
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id = canid1,
        .mst_id = mstid1
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id = canid2,
        .mst_id = mstid2
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id = canid3,
        .mst_id = mstid3
    });
    
    // 电机4-6: DM4310 (小扭矩电机)
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id = canid4,
        .mst_id = mstid4
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id = canid5,
        .mst_id = mstid5
    });
    init_data.push_back(damiao::DmActData{
        .motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id = canid6,
        .mst_id = mstid6
    });
    
    // 创建电机控制接口 (注意: 需要修改USB设备序列号为实际值)
    motorsInterface = std::make_shared<damiao::Motor_Control>(
        nom_baud, dat_baud, 
        "14AA044B241402B10DDBDAFE448040BB",  // USB设备序列号，需根据实际情况修改
        &init_data
    );
    
    ROS_INFO("Motor interface initialized.");
    usleep(1000000);  // 等待1秒让电机稳定
    
    // 切换到MIT模式进行力矩控制
    for(int i = 1; i <= 6; i++) {
        motorsInterface->switchControlMode(*motorsInterface->getMotor(i), damiao::MIT);
        usleep(10000);
    }
    ROS_INFO("All motors switched to MIT mode.");
    
    // Create threads with different rates
    ros::Rate gc_rate(500);   // 500Hz for gravity compensation
    ros::Rate js_rate(100);   // 100Hz for joint state publishing
    
    std::thread gc_thread(gravityCompensationThread, gc_rate);
    std::thread js_thread(jointStatePublisherThread, js_rate, &joint_state_pub);
    
    // Wait for threads to finish
    gc_thread.join();
    js_thread.join();
    
    // 禁用所有电机
    motorsInterface->disable_all();
    ROS_INFO("All motors disabled. Exiting.");
    
    return 0;
}

void gravityCompensationThread(ros::Rate rate) {
    ROS_INFO("Gravity compensation thread started at 500Hz.");
    
    while(ros::ok() && running) {
        ros::Time current_time = ros::Time::now();
        
        // 刷新所有电机状态
        for(int i = 1; i <= 6; i++) {
            motorsInterface->refresh_motor_status(*motorsInterface->getMotor(i));
        }
        
        // 读取关节位置和速度
        {
            boost::unique_lock<boost::shared_mutex> lock(data_mutex_);
            for(int i = 0; i < 6; i++) {
                // 读取电机位置并应用方向修正
                double raw_pos = motorsInterface->getMotor(i+1)->Get_Position();
                double raw_vel = motorsInterface->getMotor(i+1)->Get_Velocity();
                
                q[i] = raw_pos * directionMotor[i];
                vel[i] = raw_vel * directionMotor[i];
            }
        }
        
        // 计算摩擦力补偿 (简单的库仑+粘性摩擦模型)
        // 根据原代码的摩擦补偿逻辑
        for(int i = 0; i < 6; i++) {
            f[i] = 0.0;
        }
        
        // 关节1摩擦补偿
        if(vel[0] < -0.006 || vel[0] > 0.006) {
            f[0] = 0.06 * vel[0];
        }
        
        // 关节2摩擦补偿
        if(vel[1] < -0.006) {
            f[1] = -0.1 + 0.05 * vel[1];
        } else if(vel[1] > 0.006) {
            f[1] = 0.1 + 0.05 * vel[1];
        }
        
        // 关节3摩擦补偿
        if(vel[2] < -0.006) {
            f[2] = -0.2 + 0.1 * vel[2];
        } else if(vel[2] > 0.006) {
            f[2] = 0.2 + 0.1 * vel[2];
        }
        
        // 关节4摩擦补偿
        if(vel[3] < -0.006) {
            f[3] = -0.30 + 0.2 * vel[3];
        } else if(vel[3] > 0.006) {
            f[3] = 0.30 + 0.2 * vel[3];
        }
        
        // 关节5摩擦补偿
        if(vel[4] < -0.01) {
            f[4] = 0.03 - 0.05 * vel[4];
        } else if(vel[4] > 0.01) {
            f[4] = -0.03 - 0.05 * vel[4];
        }
        
        // 关节6摩擦补偿 (可选)
        // f[5] = 0.0;
        
        // 计算重力补偿
        VectorXd tau = compute_gravity_compensation(q);
        
        // 叠加摩擦补偿
        for(int i = 0; i < 6; i++) {
            tau[i] = tau[i] + f[i];
        }
        
        // 发送力矩命令到电机
        // 注意: 力矩方向需要考虑电机方向
        for(int i = 0; i < 6; i++) {
            double cmd_tau = tau[i] * directionMotor[i];
            motorsInterface->control_mit(
                *motorsInterface->getMotor(i+1),
                0.0,      // kp
                0.0,      // kd
                0.0,      // q_des
                0.0,      // dq_des
                cmd_tau   // tau_ff (前馈力矩)
            );
        }
        
        // 调试输出
        ros::Time current_time1 = ros::Time::now();
        double time_diff = (current_time1 - current_time).toSec();
        ROS_DEBUG("GC Freq: %.1f Hz", 1.0/time_diff);
        ROS_DEBUG("Tau: [%.3f] [%.3f] [%.3f] [%.3f] [%.3f] [%.3f]", 
                  tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);
        
        rate.sleep();
    }
    
    ROS_INFO("Gravity compensation thread exiting.");
}

void jointStatePublisherThread(ros::Rate rate, ros::Publisher* joint_state_pub) {
    ROS_INFO("Joint state publisher thread started at 100Hz.");
    
    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    joint_state.position.resize(6);
    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);
    
    while(ros::ok() && running) {
        // 获取当前位置 (线程安全)
        {
            boost::shared_lock<boost::shared_mutex> lock(data_mutex_);
            for(int i = 0; i < 6; i++) {
                joint_state.position[i] = q[i];
                joint_state.velocity[i] = vel[i];
                joint_state.effort[i] = tau[i];
            }
        }
        
        // 发布关节状态
        joint_state.header.stamp = ros::Time::now();
        joint_state_pub->publish(joint_state);
        
        rate.sleep();
    }
    
    ROS_INFO("Joint state publisher thread exiting.");
}