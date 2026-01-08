#include "abb_driver/AbbRobot.h"

using namespace std;

//传入action的名称
AbbRobot::AbbRobot( string name ) 
    : as_(nh_, name, boost::bind(&AbbRobot::executeCB, this,  _1), false) ,joint_count(6)       
{
    //动作名
    action_name = name;
    ros::NodeHandle nh_private("~");
    pub_flag=1;
    //初始化关节变量
    joint_name.resize(joint_count);
    msg.name.resize(joint_count);
    msg.position.resize(joint_count);
    msg.velocity.resize(joint_count);
    msg.effort.resize(joint_count);
    msg.header.frame_id = "/abb";

    //关节命名
    stringstream ss; ss.clear(); ss.str("");
    for (size_t i = 0; i < joint_count; i++)
    {
        ss << "joint" << i + 1;
        joint_name[i] = ss.str();
        msg.name[i] = joint_name[i];
        ss.clear();ss.str("");
    }

    /*通讯等初始化开始*/
    abbManipulator_.axes = joint_count;
    AbbManipulatorInit(abbManipulator_);

    //cout << "abbManipulator_.jointList.size() " << abbManipulator_.jointList.size() << endl;
    /*通讯等初始化结束*/
    

    uint16_t canid1 = 0x01;
    uint16_t mstid1 = 0x11;
    uint16_t canid2 = 0x02;
    uint16_t mstid2 = 0x12;
    uint16_t canid3 = 0x03;
    uint16_t mstid3 = 0x13;
    uint16_t canid4 = 0x04;
    uint16_t mstid4 = 0x14;
    uint16_t canid5 = 0x05;
    uint16_t mstid5 = 0x15;
    uint16_t canid6 = 0x06;
    uint16_t mstid6 = 0x16;

    uint32_t nom_baud =1000000;//仲裁域 1M
    uint32_t dat_baud =5000000;//数据域 5M  
    std::vector<damiao::DmActData> init_data;
    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
                                                .mode = damiao::MIT_MODE,
                                                .can_id=canid1,
                                                .mst_id=mstid1 });

    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id=canid2,
        .mst_id=mstid2 });

    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
        .mode = damiao::MIT_MODE,
        .can_id=canid3,
        .mst_id=mstid3 });

    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid4,
        .mst_id=mstid4 });

    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid5,
        .mst_id=mstid5 });

    init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid6,
        .mst_id=mstid6 });
        
    motorsInterface = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,"14AA044B241402B10DDBDAFE448040BB",&init_data);
    
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/abb/joint_states", 10);

    usleep(1000000); 
    for(int i=0;i<1500;i++)   
    {
        abbRead();
        msg.header.stamp = ros::Time::now();
        joint_pub_.publish(msg);
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid1),0.0,1.0);
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid2),0.0,1.0);
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid3),0.0,1.0);
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid4),0.0,1.0);
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid5),0.0,1.0);
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid6),0.0,1.0);
        usleep(2000); 
    } 
    //关节发布者初始化
    

    pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cmd_joint_pos", 10);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cmd_joint_vel", 10);
    acc_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cmd_joint_acc", 10);
    ctrl_pos_msg.data.resize(6);
    ctrl_vel_msg.data.resize(6);
    ctrl_acc_msg.data.resize(6);
    //定时器启动，定时器周期为0.1秒
    //period = 0.1;
    //timer = nh_.createTimer(ros::Duration(period), &AbbRobot::timerCallback, this);

    //服务端启动
    as_.start();
}

AbbRobot::~AbbRobot()
{
    //释放资源
    cout << "\033[32m机械臂程序已退出，请等待，并关掉电源，以免电机过热。\033[0m"<< endl;
}


/*
//timer回调函数，用于接收下位机数据
void AbbRobot::timerCallback(const ros::TimerEvent& e)
{

}*/

void AbbRobot::clamp(double* value, double min_value, double max_value) 
{
  if (*value < min_value) {
      *value = min_value;
  } else if (*value > max_value) {
      *value = max_value;
  }
}


//goal回调函数
void AbbRobot::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    cout << "\033[1m\033[32mmexecuteCB\033[0m : " << endl;

    cout<<"executeCB666"<<endl;
    //调整关节顺序
    reorder(goal->trajectory);

    //数据发送给下位机，下位机用于执行
    trackMove();

    //动作完成，反馈结果，设置完成状态
    result_.error_code = result_.SUCCESSFUL;
    as_.setSucceeded(result_);

    /*动作未完成
    result_.error_code = result_.GOAL_TOLERANCE_VIOLATED;
    as_.setAborted(result_);
    */
}

//发布者函数
void AbbRobot::jointStateUpdate()
{
   // cout << "jointStateUpdate " << endl;
    if(pub_flag==1)
    {
        // 修复：刷新所有6个电机状态（原代码仅5个）
        for(int i=0;i<6;i++)  // 改为i<6
        {
            motorsInterface->refresh_motor_status(*motorsInterface->getMotor(i+1));
        }
        // 仅在轨迹执行时发布控制指令，避免冗余
        if(!waypoints.empty()){  // 若有轨迹点才发布
            pos_pub_.publish(ctrl_pos_msg);
            vel_pub_.publish(ctrl_vel_msg);
            acc_pub_.publish(ctrl_acc_msg);
        }
    }
    abbRead();
    msg.header.stamp = ros::Time::now();
    joint_pub_.publish(msg);
}

//重排序(参考)
void AbbRobot::reorder(trajectory_msgs::JointTrajectory trajectory)
{
    feedback_.header.frame_id = trajectory.header.frame_id;
    //添加路点到容器中(参考)
    cout << "\033[1m\033[35mPoints count : \033[0m" << trajectory.points.size() << endl;
    for (size_t seq = 0; seq < trajectory.points.size(); seq ++)
    {
        waypoints.push_back(trajectory.points[seq]);
    }

    //清空
    feedback_.joint_names.clear();

    //根据名称进行排序  (参考)
    for (size_t index = 0; index < joint_count; index ++)
    {//joint_names中，逐个索引
        const char *p = trajectory.joint_names[index].c_str();
        int degree_id = 0;
        for (size_t i = 0; i < trajectory.joint_names[index].length(); i ++)
        {//string -> int。 将编号提取出来
            degree_id = degree_id * 10 + (int)(p[i + 5] - '0');
        }
        feedback_.joint_names.push_back(trajectory.joint_names[index]);
    }
}

void AbbRobot::printJointTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point) {
    std::cout << "  positions: [ ";
    for (double v : point.positions) std::cout << std::setprecision(3) << v << " ";
    std::cout << "]\n";

    std::cout << "  velocities: [ ";
    for (double v : point.velocities) std::cout << std::setprecision(3) << v << " ";
    std::cout << "]\n";

    std::cout << "  accelerations: [ ";
    for (double v : point.accelerations) std::cout << std::setprecision(3) << v << " ";
    std::cout << "]\n";

    std::cout << "  effort: [ ";
    for (double v : point.effort) std::cout << std::setprecision(3) << v << " ";
    std::cout << "]\n";

    std::cout << "  time_from_start: " 
              << point.time_from_start.sec << "s + " 
              << point.time_from_start.nsec << "ns\n";
}

void AbbRobot::printTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>& result) {
    std::cout << "Trajectory size: " << result.size() << std::endl;
    for (size_t i = 0; i < result.size(); ++i) {
        std::cout << "Point " << i << ":\n";
        printJointTrajectoryPoint(result[i]);
    }
}
vector<trajectory_msgs::JointTrajectoryPoint> AbbRobot::quinticInterpolation(
    vector<trajectory_msgs::JointTrajectoryPoint>& points, 
    double dt
) {
    vector<trajectory_msgs::JointTrajectoryPoint> result;

    for (size_t seq = 0; seq < points.size() - 1; ++seq) { 
        auto& p0 = points[seq];
        auto& p1 = points[seq + 1];

        // 计算时间间隔
        ros::Duration T_duration = p1.time_from_start - p0.time_from_start;
        double T = T_duration.toSec();
        if (T < 1e-6) { // 修正2: 处理无效时间间隔
            continue;
        }

        size_t steps = static_cast<size_t>(T / dt);

        // 预计算所有关节的系数
        struct Coefficients { double a0, a1, a2, a3, a4, a5; };
        array<Coefficients, 6> coeffs;

        for (int dof = 0; dof < 6; ++dof) {
            double p0_pos = p0.positions[dof];
            double p1_pos = p1.positions[dof];
            double v0 = p0.velocities[dof];
            double v1 = p1.velocities[dof];
            double a0 = p0.accelerations[dof];
            double a1 = p1.accelerations[dof];

            double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
            double delta_p = p1_pos - p0_pos;

            coeffs[dof].a0 = p0_pos;
            coeffs[dof].a1 = v0;
            coeffs[dof].a2 = 0.5 * a0;
            coeffs[dof].a3 = (20 * delta_p - (8*v1 + 12*v0)*T - (3*a0 - a1)*T2) / (2*T3);
            coeffs[dof].a4 = (-30*delta_p + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T2) / (2*T4);
            coeffs[dof].a5 = (12*delta_p - (6*v1 + 6*v0)*T - (a0 - a1)*T2) / (2*T5);
        }

        // 生成插值点
        for (size_t s = 0; s <= steps; ++s) {
            trajectory_msgs::JointTrajectoryPoint point;
            double t = s * dt;
            double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;

            // 填充所有关节数据
            for (int dof = 0; dof < 6; ++dof) {
                // 位置
                double pos = coeffs[dof].a0 + coeffs[dof].a1 * t + coeffs[dof].a2 * t2 +
                            coeffs[dof].a3 * t3 + coeffs[dof].a4 * t4 + coeffs[dof].a5 * t5;
                point.positions.push_back(pos);

                // 速度
                double vel = coeffs[dof].a1 + 2*coeffs[dof].a2*t + 
                            3*coeffs[dof].a3*t2 + 4*coeffs[dof].a4*t3 + 5*coeffs[dof].a5*t4;
                point.velocities.push_back(vel);

                // 加速度
                double acc = 2*coeffs[dof].a2 + 6*coeffs[dof].a3*t + 
                            12*coeffs[dof].a4*t2 + 20*coeffs[dof].a5*t3;
                point.accelerations.push_back(acc);
            }

            point.time_from_start = p0.time_from_start + ros::Duration(t);
            result.push_back(point); // 修正3: 按顺序追加点
        }
    }

    return result;
}



//路径执行
void AbbRobot::trackMove()
{
    cout << "\033[1m\033[32mTrackMoving \033[0m" << endl;
    pub_flag=0;
    // ros::Rate rate(500);
    //写入下位机，且下位机按照moveit要求执行运动
    inter_waypoints=quinticInterpolation(waypoints,0.01);
    //printTrajectory(inter_waypoints);
    //std_msgs::Float64MultiArray ctrl_pos_msg, ctrl_vel_msg, ctrl_acc_msg;
    //ctrl_pos_msg.data.resize(6);
    //ctrl_vel_msg.data.resize(6);
    //ctrl_acc_msg.data.resize(6);

    for (size_t seq = 0; seq < inter_waypoints.size(); seq ++)
    {   
        const auto currentTime = Clock::now();
         const Duration desiredDuration(1.0 / 100.0);
        abbWrite(inter_waypoints[seq]);
        for(size_t j = 0; j< 6; j++)
        {
          ctrl_pos_msg.data[j]=inter_waypoints[seq].positions[j];
          ctrl_vel_msg.data[j]=inter_waypoints[seq].velocities[j];
          ctrl_acc_msg.data[j]=inter_waypoints[seq].accelerations[j];
          pos_pub_.publish(ctrl_pos_msg);
          vel_pub_.publish(ctrl_vel_msg);
          acc_pub_.publish(ctrl_acc_msg);
        }
        //当前路径点的反馈,按照每个关节进行写入
        feedback_.desired.positions.clear();
        feedback_.actual.positions.clear();
        feedback_.actual.velocities.clear();
        feedback_.actual.effort.clear();

        for (size_t i = 0; i < joint_count; i++)
        {   
            feedback_.desired.positions.push_back(inter_waypoints[seq].positions[i]);
            feedback_.actual.positions.push_back(msg.position[i]);
            feedback_.actual.velocities.push_back(msg.velocity[i]);
            feedback_.actual.effort.push_back(msg.effort[i]);
        }
        feedback_.header.stamp = msg.header.stamp;
        as_.publishFeedback(feedback_);
        const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
        std::this_thread::sleep_until(sleepTill);
        //rate.sleep();   
    }

    //路径点执行完毕
    cout << "\033[1m\033[32mstop Moving\033[0m" << endl;
    waypoints.clear();
    pub_flag=1;
}


//数据写入下位机并执行(参考)
void AbbRobot::abbWrite(trajectory_msgs::JointTrajectoryPoint point)
{
    //cout << "abbWrite " << endl;
    //cout << point.positions.size() << " " << point.velocities.size() << " " << point.accelerations.size() << " " << point.effort.size() << endl;
    for (int i = 0; i < joint_count; i++)
    {     
        abbManipulator_.jointList[i].pos_des = point.positions[i];     
    }
 
    clamp(&abbManipulator_.jointList[0].pos_des, -1.57, 1.57);//第一个关节
    clamp(&abbManipulator_.jointList[1].pos_des, -3.14, 0.0);//第二个关节
    clamp(&abbManipulator_.jointList[2].pos_des, 0.0, 2.0);//第三个关节
    clamp(&abbManipulator_.jointList[3].pos_des, -3.00, 3.00);//第四个关节
    clamp(&abbManipulator_.jointList[4].pos_des, -1.28, 1.28);//第五个关节
    clamp(&abbManipulator_.jointList[5].pos_des, 0.0, 1.1);//第五个关节
    for (int i = 0; i < joint_count; ++i)
    {
        abbManipulator_.jointList[i].pos_des = abbManipulator_.jointList[i].pos_des * directionMotor_[i];
        // if(i==5)
        // {
        //   motorsInterface->control_pos_vel(*motorsInterface->getMotor(i+1),abbManipulator_.jointList[i].pos_des,20.0);
        // }
        // else
        motorsInterface->control_pos_vel(*motorsInterface->getMotor(i+1),abbManipulator_.jointList[i].pos_des,2);
    }

}

//从下位机读取数据(参考)
void AbbRobot::abbRead()
{
    //cout << "abbRead\n";
    //for (size_t i = 0; i < joint_count; i++)
   // {        
    //    msg.position[i] = abbManipulator_.jointList[i].position;
    //    msg.velocity[i] = abbManipulator_.jointList[i].velocity;
   //     msg.effort[i] = abbManipulator_.jointList[i].effort;
   //     abbManipulator_.jointList[i].accelerations = 0;
   // }
   double pos,vel,tau;
   for (int i = 0; i < joint_count; ++i)
   {  
       pos=motorsInterface->getMotor(i+1)->Get_Position();
       vel=motorsInterface->getMotor(i+1)->Get_Velocity();
       tau=motorsInterface->getMotor(i+1)->Get_tau();

       abbManipulator_.jointList[i].pos = pos * directionMotor_[i];
       abbManipulator_.jointList[i].vel = vel * directionMotor_[i];
       abbManipulator_.jointList[i].tau = tau * directionMotor_[i];

       msg.position[i]=abbManipulator_.jointList[i].pos;
       msg.velocity[i]=abbManipulator_.jointList[i].vel;
       msg.effort[i]=abbManipulator_.jointList[i].tau;
   }
    //cout << "abbRead exit\n";
}



// #include "abb_driver/AbbRobot.h"

// using namespace std;
// using namespace Eigen;

// // 重力补偿常量
// constexpr double G = 9.8;
// const double MASS[6] = {0.138, 1.379, 0.727, 0.370, 0.391, 0.179};
// const Vector3d COM[6] = {
//     Vector3d(0.0027728, 0.0, 0.017932),
//     Vector3d(-0.0035914, -0.14996, 0.0),
//     Vector3d(-0.00257, 0.09923, 0.051397),
//     Vector3d(0.0034984, 0.028153, 0.0),
//     Vector3d(-0.00068876, 0.076386, 0.0),
//     Vector3d(0.0, 0.0065, 0.0)
// };

// static Matrix3d rotX(double t) {
//     Matrix3d R; double c = cos(t), s = sin(t);
//     R << 1, 0, 0, 0, c, -s, 0, s, c;
//     return R;
// }
// static Matrix3d rotY(double t) {
//     Matrix3d R; double c = cos(t), s = sin(t);
//     R << c, 0, s, 0, 1, 0, -s, 0, c;
//     return R;
// }
// static Matrix3d rotZ(double t) {
//     Matrix3d R; double c = cos(t), s = sin(t);
//     R << c, -s, 0, s, c, 0, 0, 0, 1;
//     return R;
// }

// AbbRobot::AbbRobot(string name)
//     : as_(nh_, name, boost::bind(&AbbRobot::executeCB, this, _1), false), joint_count(6)
// {
//     action_name = name;
//     ros::NodeHandle nh_private("~");
//     pub_flag = 1;

//     joint_name.resize(joint_count);
//     msg.name.resize(joint_count);
//     msg.position.resize(joint_count);
//     msg.velocity.resize(joint_count);
//     msg.effort.resize(joint_count);
//     msg.header.frame_id = "/abb";

//     stringstream ss;
//     for (size_t i = 0; i < joint_count; i++)
//     {
//         ss << "joint" << i + 1;
//         joint_name[i] = ss.str();
//         msg.name[i] = joint_name[i];
//         ss.clear();
//         ss.str("");
//     }

//     abbManipulator_.axes = joint_count;
//     AbbManipulatorInit(abbManipulator_);

//     uint16_t canid1 = 0x01, mstid1 = 0x11;
//     uint16_t canid2 = 0x02, mstid2 = 0x12;
//     uint16_t canid3 = 0x03, mstid3 = 0x13;
//     uint16_t canid4 = 0x04, mstid4 = 0x14;
//     uint16_t canid5 = 0x05, mstid5 = 0x15;
//     uint16_t canid6 = 0x06, mstid6 = 0x16;

//     uint32_t nom_baud = 1000000;
//     uint32_t dat_baud = 5000000;
//     std::vector<damiao::DmActData> init_data;
//     init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
//                                           .mode = damiao::MIT_MODE,
//                                           .can_id = canid1,
//                                           .mst_id = mstid1});
//     init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
//                                           .mode = damiao::MIT_MODE,
//                                           .can_id = canid2,
//                                           .mst_id = mstid2});
//     init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340,
//                                           .mode = damiao::MIT_MODE,
//                                           .can_id = canid3,
//                                           .mst_id = mstid3});
//     init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
//                                           .mode = damiao::MIT_MODE,
//                                           .can_id = canid4,
//                                           .mst_id = mstid4});
//     init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
//                                           .mode = damiao::MIT_MODE,
//                                           .can_id = canid5,
//                                           .mst_id = mstid5});
//     init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
//                                           .mode = damiao::MIT_MODE,
//                                           .can_id = canid6,
//                                           .mst_id = mstid6});

//     motorsInterface = std::make_shared<damiao::Motor_Control>(
//         nom_baud, dat_baud, "14AA044B241402B10DDBDAFE448040BB", &init_data);

//     joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/abb/joint_states", 10);

//     usleep(1000000);
//     for (int i = 0; i < 1500; i++)
//     {
//         abbRead();
//         msg.header.stamp = ros::Time::now();
//         joint_pub_.publish(msg);
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid1), 0.0, 1.0);
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid2), 0.0, 1.0);
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid3), 0.0, 1.0);
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid4), 0.0, 1.0);
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid5), 0.0, 1.0);
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(canid6), 0.0, 1.0);
//         usleep(2000);
//     }

//     pos_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cmd_joint_pos", 10);
//     vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cmd_joint_vel", 10);
//     acc_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("cmd_joint_acc", 10);
//     ctrl_pos_msg.data.resize(6);
//     ctrl_vel_msg.data.resize(6);
//     ctrl_acc_msg.data.resize(6);

//     // 力矩控制订阅器
//     torque_mode_sub_ = nh_.subscribe("torque_mode_enable", 10, 
//                                       &AbbRobot::torqueModeEnableCallback, this);
//     torque_params_sub_ = nh_.subscribe("torque_params", 10, 
//                                         &AbbRobot::torqueParamsCallback, this);
//     last_torque_time_ = ros::Time::now();

//     ROS_INFO("\033[1;32m[AbbRobot] Zero-force control interface ready\033[0m");
//     ROS_INFO("\033[1;32m[AbbRobot]   - Subscribe: /abb/torque_mode_enable (Bool)\033[0m");
//     ROS_INFO("\033[1;32m[AbbRobot]   - Subscribe: /abb/torque_params (Float64MultiArray: [scale, damping])\033[0m");

//     as_.start();
// }

// AbbRobot::~AbbRobot()
// {
//     if (torque_mode_enabled_)
//     {
//         for (int i = 0; i < joint_count; ++i)
//         {
//             auto m = motorsInterface->getMotor(i + 1);
//             if (m) motorsInterface->control_mit(*m, 0, 0, 0, 0, 0);
//         }
//     }
//     cout << "\033[32m机械臂程序已退出。\033[0m" << endl;
// }

// void AbbRobot::computeGravityCompensation(const std::array<double, 6>& q,
//                                           std::array<double, 6>& tau_g)
// {
//     tau_g.fill(0.0);

//     Matrix4d T[7];
//     T[0] = Matrix4d::Identity();

//     // J1: Z轴
//     Matrix4d T1 = Matrix4d::Identity();
//     T1.block<3, 3>(0, 0) = rotZ(q[0]);
//     T1(2, 3) = 0.06475;
//     T[1] = T[0] * T1;

//     // J2: X轴
//     Matrix4d T2 = Matrix4d::Identity();
//     T2.block<3, 3>(0, 0) = rotX(q[1]);
//     T2(0, 3) = 0.00325;
//     T2(2, 3) = 0.04575;
//     T[2] = T[1] * T2;

//     // J3: X轴
//     Matrix4d T3 = Matrix4d::Identity();
//     T3.block<3, 3>(0, 0) = rotX(q[2]);
//     T3(1, 3) = -0.3;
//     T[3] = T[2] * T3;

//     // J4: -Y轴
//     Matrix4d T4 = Matrix4d::Identity();
//     T4.block<3, 3>(0, 0) = rotY(-q[3]);
//     T4(0, 3) = -0.00325;
//     T4(1, 3) = 0.218;
//     T4(2, 3) = 0.065;
//     T[4] = T[3] * T4;

//     // J5: X轴
//     Matrix4d T5 = Matrix4d::Identity();
//     T5.block<3, 3>(0, 0) = rotX(q[4]);
//     T5(0, 3) = -0.00275;
//     T5(1, 3) = 0.036;
//     T[5] = T[4] * T5;

//     // J6: -Y轴
//     Matrix4d T6 = Matrix4d::Identity();
//     T6.block<3, 3>(0, 0) = rotY(-q[5]);
//     T6(0, 3) = 0.00275;
//     T6(1, 3) = 0.082;
//     T[6] = T[5] * T6;

//     Vector3d axis[6] = {
//         {0, 0, 1}, {1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {1, 0, 0}, {0, -1, 0}
//     };

//     Vector3d g_world(0, 0, -G);

//     for (int link = 0; link < 6; link++)
//     {
//         Vector4d com_local(COM[link](0), COM[link](1), COM[link](2), 1.0);
//         Vector4d com_world = T[link + 1] * com_local;
//         Vector3d p_com(com_world(0), com_world(1), com_world(2));
//         Vector3d F = MASS[link] * g_world;

//         for (int j = 0; j <= link; j++)
//         {
//             Vector3d p_joint(T[j + 1](0, 3), T[j + 1](1, 3), T[j + 1](2, 3));
//             Vector3d r = p_com - p_joint;
//             Vector3d torque = r.cross(F);
//             Vector3d axis_world = T[j + 1].block<3, 3>(0, 0) * axis[j];
//             tau_g[j] += torque.dot(axis_world);
//         }
//     }

//     for (int i = 0; i < 6; i++)
//         tau_g[i] = -tau_g[i];
// }

// void AbbRobot::torqueModeEnableCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//     bool new_state = msg->data;
//     bool old_state = torque_mode_enabled_.load();

//     if (new_state && !old_state)
//     {
//         ROS_INFO("\033[1;33m[AbbRobot] === TORQUE MODE ON ===\033[0m");
//         for (int i = 0; i < joint_count; ++i)
//         {
//             motorsInterface->switchControlMode(*motorsInterface->getMotor(i + 1), damiao::MIT);
//             usleep(5000);
//         }
//         ramp_count_ = 0;
//         first_read_ = true;
//         prev_dq_.fill(0.0);
//         last_torque_time_ = ros::Time::now();
//         torque_mode_enabled_ = true;
//     }
//     else if (!new_state && old_state)
//     {
//         ROS_INFO("\033[1;33m[AbbRobot] === TORQUE MODE OFF ===\033[0m");
//         torque_mode_enabled_ = false;

//         for (int i = 0; i < joint_count; ++i)
//         {
//             auto m = motorsInterface->getMotor(i + 1);
//             if (m) motorsInterface->control_mit(*m, 0, 0, 0, 0, 0);
//         }
//         usleep(10000);

//         abbRead();
//         for (int i = 0; i < joint_count; ++i)
//         {
//             double current_pos = this->msg.position[i] * directionMotor_[i];
//             motorsInterface->control_pos_vel(*motorsInterface->getMotor(i + 1), current_pos, 1.0);
//         }
//     }
// }

// void AbbRobot::torqueParamsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
// {
//     if (msg->data.size() >= 2)
//     {
//         std::lock_guard<std::mutex> lock(torque_mutex_);
//         gravity_scale_ = msg->data[0];
//         damping_ = msg->data[1];
//         ROS_INFO("[AbbRobot] Params updated: scale=%.2f, damping=%.2f", 
//                  gravity_scale_, damping_);
//     }
// }

// void AbbRobot::sendTorqueToMotors()
// {
//     if (!torque_mode_enabled_) return;

//     debug_count_++;

//     // 直接从电机读取最新状态
//     std::array<double, 6> q, dq;
//     for (int i = 0; i < joint_count; ++i)
//     {
//         auto m = motorsInterface->getMotor(i + 1);
//         if (m)
//         {
//             q[i] = m->Get_Position() * directionMotor_[i];
//             dq[i] = m->Get_Velocity() * directionMotor_[i];
//         }
//     }

//     // 速度滤波：检测跳变
//     if (!first_read_)
//     {
//         for (int i = 0; i < joint_count; ++i)
//         {
//             double dq_change = std::abs(dq[i] - prev_dq_[i]);
//             if (dq_change > 5.0)  // 速度跳变阈值
//             {
//                 ROS_WARN_THROTTLE(0.5, "[AbbRobot] J%d velocity jump: %.2f -> %.2f", 
//                                   i + 1, prev_dq_[i], dq[i]);
//                 dq[i] = prev_dq_[i];  // 使用上一次的速度
//             }
//         }
//     }
//     prev_dq_ = dq;
//     first_read_ = false;

//     // 获取参数
//     double scale, damp;
//     {
//         std::lock_guard<std::mutex> lock(torque_mutex_);
//         scale = gravity_scale_;
//         damp = damping_;
//     }

//     // 计算重力补偿
//     std::array<double, 6> tau_g;
//     computeGravityCompensation(q, tau_g);

//     // 合成力矩
//     std::array<double, 6> tau_cmd;
//     for (int i = 0; i < joint_count; ++i)
//     {
//         // 限制阻尼力矩最大值
//         double damping_torque = damp * dq[i];
//         double max_damping = 3.0;
//         if (damping_torque > max_damping) damping_torque = max_damping;
//         if (damping_torque < -max_damping) damping_torque = -max_damping;

//         tau_cmd[i] = scale * tau_g[i] - damping_torque;
//     }

//     // 软启动
//     double ramp = 1.0;
//     if (ramp_count_ < RAMP_STEPS)
//     {
//         ramp = (double)ramp_count_ / RAMP_STEPS;
//         ramp_count_++;
//         if (ramp_count_ == RAMP_STEPS)
//             ROS_INFO("[AbbRobot] Soft start done.");
//     }

//     // 发送力矩
//     for (int i = 0; i < joint_count; ++i)
//     {
//         auto m = motorsInterface->getMotor(i + 1);
//         if (m)
//         {
//             double tau = tau_cmd[i] * ramp * directionMotor_[i];
//             if (tau > TAU_MAX[i]) tau = TAU_MAX[i];
//             if (tau < -TAU_MAX[i]) tau = -TAU_MAX[i];
//             motorsInterface->control_mit(*m, 0, 0, 0, 0, tau);
//         }
//     }

//     last_torque_time_ = ros::Time::now();

//     // 调试输出
//     if (debug_mode_ && debug_count_ % 500 == 0)
//     {
//         ROS_INFO("q:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
//                  q[0], q[1], q[2], q[3], q[4], q[5]);
//         ROS_INFO("dq:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
//                  dq[0], dq[1], dq[2], dq[3], dq[4], dq[5]);
//         ROS_INFO("tau_g:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
//                  tau_g[0], tau_g[1], tau_g[2], tau_g[3], tau_g[4], tau_g[5]);
//         ROS_INFO("tau_cmd:[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
//                  tau_cmd[0], tau_cmd[1], tau_cmd[2], tau_cmd[3], tau_cmd[4], tau_cmd[5]);
//         ROS_INFO("----");
//     }
// }

// void AbbRobot::clamp(double* value, double min_value, double max_value)
// {
//     if (*value < min_value)
//         *value = min_value;
//     else if (*value > max_value)
//         *value = max_value;
// }

// void AbbRobot::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
// {
//     cout << "\033[1m\033[32mexecuteCB\033[0m" << endl;

//     if (torque_mode_enabled_)
//     {
//         ROS_WARN("[AbbRobot] Cannot execute trajectory in torque mode!");
//         result_.error_code = result_.INVALID_GOAL;
//         as_.setAborted(result_);
//         return;
//     }

//     reorder(goal->trajectory);
//     trackMove();
//     result_.error_code = result_.SUCCESSFUL;
//     as_.setSucceeded(result_);
// }

// void AbbRobot::jointStateUpdate()
// {
//     if (pub_flag == 1)
//     {
//         for (int i = 0; i < 6; i++)
//         {
//             motorsInterface->refresh_motor_status(*motorsInterface->getMotor(i + 1));
//         }

//         if (torque_mode_enabled_)
//         {
//             sendTorqueToMotors();
//         }
//         else
//         {
//             if (!waypoints.empty())
//             {
//                 pos_pub_.publish(ctrl_pos_msg);
//                 vel_pub_.publish(ctrl_vel_msg);
//                 acc_pub_.publish(ctrl_acc_msg);
//             }
//         }
//     }
//     abbRead();
//     msg.header.stamp = ros::Time::now();
//     joint_pub_.publish(msg);
// }

// void AbbRobot::reorder(trajectory_msgs::JointTrajectory trajectory)
// {
//     feedback_.header.frame_id = trajectory.header.frame_id;
//     cout << "\033[1m\033[35mPoints count : \033[0m" << trajectory.points.size() << endl;
//     for (size_t seq = 0; seq < trajectory.points.size(); seq++)
//     {
//         waypoints.push_back(trajectory.points[seq]);
//     }

//     feedback_.joint_names.clear();

//     for (size_t index = 0; index < joint_count; index++)
//     {
//         const char* p = trajectory.joint_names[index].c_str();
//         int degree_id = 0;
//         for (size_t i = 0; i < trajectory.joint_names[index].length(); i++)
//         {
//             degree_id = degree_id * 10 + (int)(p[i + 5] - '0');
//         }
//         feedback_.joint_names.push_back(trajectory.joint_names[index]);
//     }
// }

// void AbbRobot::printJointTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point)
// {
//     std::cout << "  positions: [ ";
//     for (double v : point.positions) std::cout << std::setprecision(3) << v << " ";
//     std::cout << "]\n";

//     std::cout << "  velocities: [ ";
//     for (double v : point.velocities) std::cout << std::setprecision(3) << v << " ";
//     std::cout << "]\n";

//     std::cout << "  accelerations: [ ";
//     for (double v : point.accelerations) std::cout << std::setprecision(3) << v << " ";
//     std::cout << "]\n";

//     std::cout << "  effort: [ ";
//     for (double v : point.effort) std::cout << std::setprecision(3) << v << " ";
//     std::cout << "]\n";

//     std::cout << "  time_from_start: "
//               << point.time_from_start.sec << "s + "
//               << point.time_from_start.nsec << "ns\n";
// }

// void AbbRobot::printTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>& result)
// {
//     std::cout << "Trajectory size: " << result.size() << std::endl;
//     for (size_t i = 0; i < result.size(); ++i)
//     {
//         std::cout << "Point " << i << ":\n";
//         printJointTrajectoryPoint(result[i]);
//     }
// }

// vector<trajectory_msgs::JointTrajectoryPoint> AbbRobot::quinticInterpolation(
//     vector<trajectory_msgs::JointTrajectoryPoint>& points,
//     double dt)
// {
//     vector<trajectory_msgs::JointTrajectoryPoint> result;

//     for (size_t seq = 0; seq < points.size() - 1; ++seq)
//     {
//         auto& p0 = points[seq];
//         auto& p1 = points[seq + 1];

//         ros::Duration T_duration = p1.time_from_start - p0.time_from_start;
//         double T = T_duration.toSec();
//         if (T < 1e-6) continue;

//         size_t steps = static_cast<size_t>(T / dt);

//         struct Coefficients { double a0, a1, a2, a3, a4, a5; };
//         array<Coefficients, 6> coeffs;

//         for (int dof = 0; dof < 6; ++dof)
//         {
//             double p0_pos = p0.positions[dof];
//             double p1_pos = p1.positions[dof];
//             double v0 = p0.velocities[dof];
//             double v1 = p1.velocities[dof];
//             double a0 = p0.accelerations[dof];
//             double a1 = p1.accelerations[dof];

//             double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
//             double delta_p = p1_pos - p0_pos;

//             coeffs[dof].a0 = p0_pos;
//             coeffs[dof].a1 = v0;
//             coeffs[dof].a2 = 0.5 * a0;
//             coeffs[dof].a3 = (20 * delta_p - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3);
//             coeffs[dof].a4 = (-30 * delta_p + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T4);
//             coeffs[dof].a5 = (12 * delta_p - (6 * v1 + 6 * v0) * T - (a0 - a1) * T2) / (2 * T5);
//         }

//         for (size_t s = 0; s <= steps; ++s)
//         {
//             trajectory_msgs::JointTrajectoryPoint point;
//             double t = s * dt;
//             double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

//             for (int dof = 0; dof < 6; ++dof)
//             {
//                 double pos = coeffs[dof].a0 + coeffs[dof].a1 * t + coeffs[dof].a2 * t2 +
//                              coeffs[dof].a3 * t3 + coeffs[dof].a4 * t4 + coeffs[dof].a5 * t5;
//                 point.positions.push_back(pos);

//                 double vel = coeffs[dof].a1 + 2 * coeffs[dof].a2 * t +
//                              3 * coeffs[dof].a3 * t2 + 4 * coeffs[dof].a4 * t3 + 5 * coeffs[dof].a5 * t4;
//                 point.velocities.push_back(vel);

//                 double acc = 2 * coeffs[dof].a2 + 6 * coeffs[dof].a3 * t +
//                              12 * coeffs[dof].a4 * t2 + 20 * coeffs[dof].a5 * t3;
//                 point.accelerations.push_back(acc);
//             }

//             point.time_from_start = p0.time_from_start + ros::Duration(t);
//             result.push_back(point);
//         }
//     }

//     return result;
// }

// void AbbRobot::trackMove()
// {
//     cout << "\033[1m\033[32mTrackMoving \033[0m" << endl;
//     pub_flag = 0;

//     inter_waypoints = quinticInterpolation(waypoints, 0.01);

//     for (size_t seq = 0; seq < inter_waypoints.size(); seq++)
//     {
//         const auto currentTime = Clock::now();
//         const Duration desiredDuration(1.0 / 100.0);
//         abbWrite(inter_waypoints[seq]);
//         for (size_t j = 0; j < 6; j++)
//         {
//             ctrl_pos_msg.data[j] = inter_waypoints[seq].positions[j];
//             ctrl_vel_msg.data[j] = inter_waypoints[seq].velocities[j];
//             ctrl_acc_msg.data[j] = inter_waypoints[seq].accelerations[j];
//             pos_pub_.publish(ctrl_pos_msg);
//             vel_pub_.publish(ctrl_vel_msg);
//             acc_pub_.publish(ctrl_acc_msg);
//         }

//         feedback_.desired.positions.clear();
//         feedback_.actual.positions.clear();
//         feedback_.actual.velocities.clear();
//         feedback_.actual.effort.clear();

//         for (size_t i = 0; i < joint_count; i++)
//         {
//             feedback_.desired.positions.push_back(inter_waypoints[seq].positions[i]);
//             feedback_.actual.positions.push_back(msg.position[i]);
//             feedback_.actual.velocities.push_back(msg.velocity[i]);
//             feedback_.actual.effort.push_back(msg.effort[i]);
//         }
//         feedback_.header.stamp = msg.header.stamp;
//         as_.publishFeedback(feedback_);
//         const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
//         std::this_thread::sleep_until(sleepTill);
//     }

//     cout << "\033[1m\033[32mstop Moving\033[0m" << endl;
//     waypoints.clear();
//     pub_flag = 1;
// }

// void AbbRobot::abbWrite(trajectory_msgs::JointTrajectoryPoint point)
// {
//     for (int i = 0; i < joint_count; i++)
//     {
//         abbManipulator_.jointList[i].pos_des = point.positions[i];
//     }

//     clamp(&abbManipulator_.jointList[0].pos_des, -1.57, 1.57);
//     clamp(&abbManipulator_.jointList[1].pos_des, -3.14, 0.0);
//     clamp(&abbManipulator_.jointList[2].pos_des, 0.0, 2.0);
//     clamp(&abbManipulator_.jointList[3].pos_des, -3.00, 3.00);
//     clamp(&abbManipulator_.jointList[4].pos_des, -1.28, 1.28);
//     clamp(&abbManipulator_.jointList[5].pos_des, 0.0, 1.1);

//     for (int i = 0; i < joint_count; ++i)
//     {
//         abbManipulator_.jointList[i].pos_des = abbManipulator_.jointList[i].pos_des * directionMotor_[i];
//         motorsInterface->control_pos_vel(*motorsInterface->getMotor(i + 1),
//                                          abbManipulator_.jointList[i].pos_des, 2);
//     }
// }

// void AbbRobot::abbRead()
// {
//     double pos, vel, tau;
//     for (int i = 0; i < joint_count; ++i)
//     {
//         pos = motorsInterface->getMotor(i + 1)->Get_Position();
//         vel = motorsInterface->getMotor(i + 1)->Get_Velocity();
//         tau = motorsInterface->getMotor(i + 1)->Get_tau();

//         abbManipulator_.jointList[i].pos = pos * directionMotor_[i];
//         abbManipulator_.jointList[i].vel = vel * directionMotor_[i];
//         abbManipulator_.jointList[i].tau = tau * directionMotor_[i];

//         msg.position[i] = abbManipulator_.jointList[i].pos;
//         msg.velocity[i] = abbManipulator_.jointList[i].vel;
//         msg.effort[i] = abbManipulator_.jointList[i].tau;
//     }
// }