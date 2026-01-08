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