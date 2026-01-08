#include "abb_driver/AbbRobot.h"

Clock::time_point lastTime_;
ros::Duration elapsedTime_;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_node");
    AbbRobot robot("abb_controller/follow_joint_trajectory");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // ros::Rate rate(500);
    double cycleTimeErrorThreshold_{0.01};
    while (ros::ok())
    {
      const auto currentTime = Clock::now();
      const Duration desiredDuration(1.0 / 100.0);
      Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
      elapsedTime_ = ros::Duration(time_span.count());
      lastTime_ = currentTime;
    
      const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
      if (cycle_time_error > cycleTimeErrorThreshold_)
      {
        ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                                   << "cycle time: " << elapsedTime_ << "s, "
                                                                   << "threshold: " << cycleTimeErrorThreshold_ << "s");
      }
      robot.jointStateUpdate();
      const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
      std::this_thread::sleep_until(sleepTill);
      //rate.sleep();
    }

    cout << "\033[1m\033[32m机械臂程序正在关闭...\033[0m" << endl;

    return 0;
}