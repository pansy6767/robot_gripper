#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

// 定义机械臂参数
constexpr double PI = 3.141592653589793;
constexpr int DOF = 6;                    // 六自由度
constexpr double AMPLITUDE = 1.0;         // 关节运动幅值 (弧度)
constexpr double FREQUENCY = 0.5;         // 运动频率 (Hz)
constexpr double DURATION = 10.0;         // 总运动时间 (秒)
constexpr double TIME_STEP = 0.01;         // 时间步长 (秒)

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

// 生成正弦波轨迹点
trajectory_msgs::JointTrajectory generate_sine_trajectory(
    const std::vector<std::string>& joint_names)
{
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = joint_names;
    
    const int num_points = static_cast<int>(DURATION / TIME_STEP);
    const double omega = 2 * PI * FREQUENCY;  // 角频率
    
    for (int i = 0; i <= num_points; ++i) {
        const double t = i * TIME_STEP;
        trajectory_msgs::JointTrajectoryPoint point;
        
        // 为每个关节生成正弦位置
        for (int j = 0; j < DOF; ++j) {
            double phase = (j * PI/DOF);  // 各关节相位差
            if(j==0)
            {
                point.positions.push_back(1.57* sin(omega * t + phase));
                point.velocities.push_back(1.57* omega*cos(omega * t + phase));
                point.accelerations.push_back(-1.57* omega*omega*sin(omega * t + phase));
            }
            else if(j==1)
            {
                point.positions.push_back(1.57* sin(omega * t + phase)-1.57);
                point.velocities.push_back(1.57* omega*cos(omega * t + phase));
                point.accelerations.push_back(-1.57* omega*omega*sin(omega * t + phase));
            }
            else if(j==2)
            {
                point.positions.push_back(1.57* sin(omega * t + phase)+1.57);
                point.velocities.push_back(1.57* omega*cos(omega * t + phase));
                point.accelerations.push_back(-1.57* omega*omega*sin(omega * t + phase));
            }
            else if(j==3||j==4||j==5)
            {
                point.positions.push_back(1.4* sin(omega * t + phase));
                point.velocities.push_back(1.4* omega*cos(omega * t + phase));
                point.accelerations.push_back(-1.4* omega*omega*sin(omega * t + phase));  
            }        
        } 
        point.time_from_start = ros::Duration(t);
        trajectory.points.push_back(point);
    }
    
    return trajectory;
}



trajectory_msgs::JointTrajectory generate_snake_trajectory_limited(
    const std::vector<std::string>& joint_names)
{
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = joint_names;

    const int num_points = static_cast<int>(DURATION / TIME_STEP);
    const double omega = 2 * PI * FREQUENCY;
    
    // 设置每个关节的幅度（不超过其限制）
    const double amp_j1 = 1.57;   // yaw
    const double amp_j2 = 1.5;    // pitch downward swing within [-3.14, 0]
    const double amp_j3 = 1.5;    // pitch upward swing within [0, 3.14]
    const double amp_j4 = 1.5;    // yaw
    const double amp_j5 = 1.0;    // pitch, 最后补偿姿态用
    const double amp_j6 = 1.0;    // yaw 末端

    const double phase_diff = PI / 3;

    for (int i = 0; i <= num_points; ++i)
    {
        double t = i * TIME_STEP;
        trajectory_msgs::JointTrajectoryPoint point;

        // Joint1 (yaw)
        point.positions.push_back(amp_j1 * sin(omega * t));
        point.velocities.push_back(amp_j1 * omega * cos(omega * t));
        point.accelerations.push_back(-amp_j1 * omega * omega * sin(omega * t));

        // Joint2 (pitch) downward swing
        double j2 = -amp_j2 * fabs(sin(omega * t + phase_diff));  // 保证在[-3.14, 0]
        point.positions.push_back(j2);
        point.velocities.push_back(-amp_j2 * omega * cos(omega * t + phase_diff) * ((sin(omega * t + phase_diff) >= 0) ? 1 : -1));
        point.accelerations.push_back(-amp_j2 * omega * omega * sin(omega * t + phase_diff));

        // Joint3 (pitch) upward swing
        double j3 = amp_j3 * fabs(sin(omega * t + 2 * phase_diff));  // 保证在[0, 3.14]
        point.positions.push_back(j3);
        point.velocities.push_back(amp_j3 * omega * cos(omega * t + 2 * phase_diff) * ((sin(omega * t + 2 * phase_diff) >= 0) ? 1 : -1));
        point.accelerations.push_back(-amp_j3 * omega * omega * sin(omega * t + 2 * phase_diff));

        // Joint4 (yaw)
        point.positions.push_back(amp_j4 * sin(omega * t + phase_diff));
        point.velocities.push_back(amp_j4 * omega * cos(omega * t + phase_diff));
        point.accelerations.push_back(-amp_j4 * omega * omega * sin(omega * t + phase_diff));

        // Joint5 (pitch) 姿态补偿 = - (j2 + j3)
        //double j5 = -(j2 + j3);
        double j5 = -j3-j2;
       // j5 = std::clamp(j5, -1.28, 1.28);  // 加入角度限制
        point.positions.push_back(j5);
        point.velocities.push_back(-(point.velocities[1] + point.velocities[2]));
        point.accelerations.push_back(-(point.accelerations[1] + point.accelerations[2]));

        // Joint6 (末端 yaw，自由摆动)
        point.positions.push_back(amp_j6 * sin(omega * t + 3 * phase_diff));
        point.velocities.push_back(amp_j6 * omega * cos(omega * t + 3 * phase_diff));
        point.accelerations.push_back(-amp_j6 * omega * omega * sin(omega * t + 3 * phase_diff));

        point.time_from_start = ros::Duration(t);
        trajectory.points.push_back(point);
    }

    return trajectory;
}





//clamp(&pos[0], -3.14, 3.14);//第一个关节
//clamp(&pos[1], -3.14, 0.0);//第二个关节
//clamp(&pos[2], 0.0, 3.14);//第三个关节
//clamp(&pos[3], -1.57, 1.57);//第四个关节
//clamp(&pos[4], -1.4, 1.4);//第五个关节



int main(int argc, char** argv) {
    ros::init(argc, argv, "periodic_motion_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // 初始化MoveIt接口
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    move_group.setPlannerId("RRTConnect");
    move_group.setMaxVelocityScalingFactor(0.5);  // 限制最大速度

    // 获取初始关节状态
    std::vector<double> initial_joints = {0.0,0.0,0.0,0.0,0.0,0.0};
    
    // 创建Action客户端
    TrajectoryClient client("/abb/abb_controller/follow_joint_trajectory", true);
    if (!client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Unable to connect to Action server!");
        return 1;
    }

    // 生成轨迹
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = generate_sine_trajectory(move_group.getJointNames());
    //goal.trajectory = generate_snake_trajectory_limited(move_group.getJointNames());
    // 执行轨迹循环
    ros::Rate loop_rate(1.0/FREQUENCY);
    while (ros::ok()) {
        // 发送轨迹目标
        client.sendGoal(goal);
        bool finished = client.waitForResult(ros::Duration(DURATION + 2.0));
        
        if (!finished) {
            ROS_WARN("Execution timed out, trying to restore the initial position...");
            move_group.setJointValueTarget(initial_joints);
            move_group.move();
            break;
        }
        
        loop_rate.sleep();
    }

    //move_group.setJointValueTarget(initial_joints);
   // move_group.move();
   // sleep(100);

    return 0;
}
