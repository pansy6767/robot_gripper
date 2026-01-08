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