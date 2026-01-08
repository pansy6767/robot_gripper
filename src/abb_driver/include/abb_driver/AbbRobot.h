#ifndef ABBROBOT_H
#define ABBROBOT_H

#include "abb_driver/common.h"
#include "dmbot_serial/protocol/damiao.h"
#include <std_msgs/Float64MultiArray.h>
#include <chrono>
#include <thread>

using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;
struct AbbJoint
{
	double pos, vel, tau ,acc;
  	double pos_des, vel_des, kp, kd, ff;
	double vel_limit;
};

struct AbbManipulator
{
	int axes;	//自由度
	std::vector<AbbJoint> jointList;		//关节列表
};
 
//初始化
void static AbbManipulatorInit(AbbManipulator & abbManipulator)
{
	AbbJoint sj ;
	for (size_t i = 0; i < abbManipulator.axes; i++)
	{
		abbManipulator.jointList.push_back(sj);
	}
};


class AbbRobot
{
public:
	//传入action的名称
	AbbRobot( string name );       
	virtual ~AbbRobot();

	/*
	//timer回调函数，用于接收下位机数据
    void timerCallback(const ros::TimerEvent& e);
    */
   void clamp(double* value, double min_value, double max_value);

    //goal回调函数
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

	//发布消息
	void jointStateUpdate();

	//重排序
	void reorder(trajectory_msgs::JointTrajectory trajectory);

	//路径执行
	void trackMove();

	//数据写入下位机并执行
	void abbWrite(trajectory_msgs::JointTrajectoryPoint point);

	//从下位机读取数据
	void abbRead();

	void printJointTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point);
	
	void printTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>& result);

	vector<trajectory_msgs::JointTrajectoryPoint> quinticInterpolation(vector<trajectory_msgs::JointTrajectoryPoint>& points, double dt);

private:
	std::shared_ptr<damiao::Motor_Control> motorsInterface;
	//自由度
    const int joint_count;
    std::vector<string> joint_name;

    //句柄实例
    ros::NodeHandle nh_;

    //ros系统时间
	ros::Time time_current, time_prev;

    //ros定时器
    //ros::Timer timer;

	//定时器周期
    double period;
	
	//action名称
    string action_name;

    //定义action服务端实例
	//actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>  as_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>  as_;

	/*客户端                        服务端
	--------                      --------
	发布 /goal   -------->      订阅 /goal
	发布 /cancel -------->      订阅 /cancel

	订阅 /status <--------      发布 /status
	订阅 /feedback <------      发布 /feedback
	订阅 /result   <------      发布 /result*/
	
    //反馈实例
    control_msgs::FollowJointTrajectoryFeedback feedback_;

    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::FollowJointTrajectoryResult result_; 

	//路径点容器
    vector<trajectory_msgs::JointTrajectoryPoint> waypoints;
	//插值后的路径点容器
	vector<trajectory_msgs::JointTrajectoryPoint> inter_waypoints;
    //关节状态发布者 消息实例
    ros::Publisher joint_pub_;
	ros::Publisher pos_pub_, vel_pub_, acc_pub_;
    sensor_msgs::JointState msg;

	//虚拟下位机机器人实例
	AbbManipulator abbManipulator_;
	std::atomic<bool> pub_flag ;
	std_msgs::Float64MultiArray ctrl_pos_msg, ctrl_vel_msg, ctrl_acc_msg;

	std::vector<int> directionMotor_{ 1, -1, 1, -1, -1, 1};//65432
};

#endif // ABBROBOT_H


// #ifndef ABBROBOT_H
// #define ABBROBOT_H
// #include "abb_driver/common.h"
// #include "dmbot_serial/protocol/damiao.h"
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Bool.h>
// #include <chrono>
// #include <thread>
// #include <mutex>
// #include <eigen3/Eigen/Dense>

// using Clock = std::chrono::high_resolution_clock;
// using Duration = std::chrono::duration<double>;

// struct AbbJoint
// {
//     double pos, vel, tau, acc;
//     double pos_des, vel_des, kp, kd, ff;
//     double vel_limit;
// };

// struct AbbManipulator
// {
//     int axes;
//     std::vector<AbbJoint> jointList;
// };

// void static AbbManipulatorInit(AbbManipulator& abbManipulator)
// {
//     AbbJoint sj;
//     for (size_t i = 0; i < abbManipulator.axes; i++)
//     {
//         abbManipulator.jointList.push_back(sj);
//     }
// };

// class AbbRobot
// {
// public:
//     AbbRobot(string name);
//     virtual ~AbbRobot();
//     void clamp(double* value, double min_value, double max_value);
//     void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
//     void jointStateUpdate();
//     void reorder(trajectory_msgs::JointTrajectory trajectory);
//     void trackMove();
//     void abbWrite(trajectory_msgs::JointTrajectoryPoint point);
//     void abbRead();
//     void printJointTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point);
//     void printTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>& result);
//     vector<trajectory_msgs::JointTrajectoryPoint> quinticInterpolation(
//         vector<trajectory_msgs::JointTrajectoryPoint>& points, double dt);

//     // 力矩控制接口
//     void torqueModeEnableCallback(const std_msgs::Bool::ConstPtr& msg);
//     void torqueParamsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
//     void sendTorqueToMotors();
//     void computeGravityCompensation(const std::array<double, 6>& q,
//                                     std::array<double, 6>& tau_g);

// private:
//     std::shared_ptr<damiao::Motor_Control> motorsInterface;
//     const int joint_count;
//     std::vector<string> joint_name;
//     ros::NodeHandle nh_;
//     ros::Time time_current, time_prev;
//     double period;

//     string action_name;
//     actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

//     control_msgs::FollowJointTrajectoryFeedback feedback_;
//     control_msgs::FollowJointTrajectoryResult result_;
//     vector<trajectory_msgs::JointTrajectoryPoint> waypoints;
//     vector<trajectory_msgs::JointTrajectoryPoint> inter_waypoints;
//     ros::Publisher joint_pub_;
//     ros::Publisher pos_pub_, vel_pub_, acc_pub_;
//     sensor_msgs::JointState msg;
//     AbbManipulator abbManipulator_;
//     std::atomic<bool> pub_flag;
//     std_msgs::Float64MultiArray ctrl_pos_msg, ctrl_vel_msg, ctrl_acc_msg;
//     std::vector<int> directionMotor_{1, -1, 1, -1, -1, 1};

//     // 力矩控制相关变量
//     ros::Subscriber torque_mode_sub_;
//     ros::Subscriber torque_params_sub_;
//     std::atomic<bool> torque_mode_enabled_{false};
//     std::mutex torque_mutex_;
//     ros::Time last_torque_time_;
    
//     // 控制参数
//     double gravity_scale_ = 1.0;
//     double damping_ = 0.5;
//     int ramp_count_ = 0;
//     const int RAMP_STEPS = 500;
//     const double TORQUE_TIMEOUT = 0.5;
//     const double TAU_MAX[6] = {28.0, 28.0, 28.0, 10.0, 10.0, 10.0};
    
//     // 速度滤波
//     std::array<double, 6> prev_dq_{0, 0, 0, 0, 0, 0};
//     bool first_read_ = true;
    
//     // 调试
//     int debug_count_ = 0;
//     bool debug_mode_ = true;
// };
// #endif // ABBROBOT_H