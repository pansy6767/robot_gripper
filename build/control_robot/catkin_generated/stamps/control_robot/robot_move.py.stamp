# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
# import rospy, sys
# import moveit_commander
# from geometry_msgs.msg import PoseStamped, Pose
# import math

# class MoveItIkDemo:
#     def __init__(self):
        
#         # 初始化move_group的API
#         moveit_commander.roscpp_initialize(sys.argv)
        
#         # 初始化ROS节点
#         rospy.init_node('moveit_ik_demo')
        
#         # 初始化需要使用move group控制的机械臂中的arm group
#         arm = moveit_commander.MoveGroupCommander('arm')
        
#         # 获取终端link的名称
#         end_effector_link = arm.get_end_effector_link()
#         rospy.loginfo("End effector link: %s" % end_effector_link)
        
#         # 设置目标位置所使用的参考坐标系
#         reference_frame = 'base_link'
#         arm.set_pose_reference_frame(reference_frame)
        
#         # 当运动规划失败后,允许重新规划
#         arm.allow_replanning(True)
        
#         # 放宽容差,帮助规划成功
#         arm.set_goal_position_tolerance(0.01)  # 从0.001放宽到0.01
#         arm.set_goal_orientation_tolerance(0.05)  # 从0.01放宽到0.05
        
#         # 设置允许的最大速度和加速度
#         arm.set_max_acceleration_scaling_factor(0.5)
#         arm.set_max_velocity_scaling_factor(0.5)
        
#         # 增加规划时间
#         arm.set_planning_time(15)
        
#         # 设置规划尝试次数
#         arm.set_num_planning_attempts(10)
        
#         # 控制机械臂先回到初始化位置
#         rospy.loginfo("Moving to home position...")
#         arm.set_named_target('home')
#         arm.go()
#         rospy.sleep(1)
        
#         # 设置机械臂工作空间中的目标位姿
#         target_pose = PoseStamped()
#         target_pose.header.frame_id = reference_frame
#         target_pose.header.stamp = rospy.Time.now()
        
#         # 末端位置  0.0080, 0.4493, 0.2067
   
#         target_pose.pose.position.x = 0.0157
#         target_pose.pose.position.y = 0.3594
#         target_pose.pose.position.z = -0.0054
        
#         # 四元数归一化
#         qx, qy, qz, qw = -0.4940, 0.0253, 0.0545, 0.8674
#         norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
#         rospy.loginfo("Original quaternion norm: %.6f" % norm)
        
#         # 归一化后的四元数
#         target_pose.pose.orientation.x = qx / norm
#         target_pose.pose.orientation.y = qy / norm
#         target_pose.pose.orientation.z = qz / norm
#         target_pose.pose.orientation.w = qw / norm
        
#         rospy.loginfo("Target pose: position(%.3f, %.3f, %.3f)" % 
#                      (target_pose.pose.position.x, 
#                       target_pose.pose.position.y, 
#                       target_pose.pose.position.z))
        
#         # 设置机器臂当前的状态作为运动初始状态
#         arm.set_start_state_to_current_state()
        
#         # 设置机械臂终端运动的目标位姿
#         arm.set_pose_target(target_pose, end_effector_link)
        
#         # 规划运动路径
#         rospy.loginfo("Planning trajectory...")
#         plan = arm.plan()
        
#         # 处理不同版本的plan()返回值
#         if isinstance(plan, tuple):
#             success = plan[0]
#             trajectory = plan[1]
#             rospy.loginfo("Planning result: %s" % ("SUCCESS" if success else "FAILED"))
#         else:
#             trajectory = plan
#             success = len(trajectory.joint_trajectory.points) > 0
#             rospy.loginfo("Trajectory points: %d" % len(trajectory.joint_trajectory.points))
        
#         if success:
#             rospy.loginfo("Planning successful! Executing trajectory...")
#             arm.execute(trajectory, wait=True)
#             rospy.sleep(1)
#             rospy.loginfo("Execution completed!")
#         else:
#             rospy.logerr("Planning failed! Trying with Cartesian path...")
            
#             # 尝试笛卡尔路径规划
#             waypoints = []
#             waypoints.append(target_pose.pose)
            
#             (plan_cartesian, fraction) = arm.compute_cartesian_path(
#                 waypoints,   # 路径点
#                 0.01,        # 步长 1cm
#                 0.0)         # 跳跃阈值
            
#             rospy.loginfo("Cartesian path completion: %.2f%%" % (fraction * 100.0))
            
#             if fraction > 0.9:  # 如果90%以上的路径可规划
#                 rospy.loginfo("Executing Cartesian path...")
#                 arm.execute(plan_cartesian, wait=True)
#                 rospy.sleep(1)
#             else:
#                 rospy.logerr("Cartesian path planning also failed!")
        
#         # # 控制机械臂回到初始化位置
#         # rospy.loginfo("Returning to home position...")
#         # arm.set_named_target('home')
#         # arm.go()
        
#         # 关闭并退出moveit
#         moveit_commander.roscpp_shutdown()
#         moveit_commander.os._exit(0)

# if __name__ == "__main__":
#     try:
#         MoveItIkDemo()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import math

class MoveItIkDemo:
    def __init__(self):
        
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        rospy.loginfo("End effector link: %s" % end_effector_link)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        # 当运动规划失败后,允许重新规划
        arm.allow_replanning(True)
        
        # 放宽容差,帮助规划成功
        arm.set_goal_position_tolerance(0.01)  # 从0.001放宽到0.01
        arm.set_goal_orientation_tolerance(0.05)  # 从0.01放宽到0.05
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 增加规划时间
        arm.set_planning_time(15)
        
        # 设置规划尝试次数
        arm.set_num_planning_attempts(10)
        
        # 控制机械臂先回到初始化位置
        rospy.loginfo("Moving to home position...")
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # 设置机械臂工作空间中的目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        
        # 末端位置
        target_pose.pose.position.x = 0.0157
        target_pose.pose.position.y = 0.3594
        target_pose.pose.position.z = -0.0054
        
        # 四元数归一化
        qx, qy, qz, qw = -0.4940, 0.0253, 0.0545, 0.8674
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        rospy.loginfo("Original quaternion norm: %.6f" % norm)
        
        # 归一化后的四元数
        target_pose.pose.orientation.x = qx / norm
        target_pose.pose.orientation.y = qy / norm
        target_pose.pose.orientation.z = qz / norm
        target_pose.pose.orientation.w = qw / norm
        
        rospy.loginfo("Target pose: position(%.3f, %.3f, %.3f)" % 
                     (target_pose.pose.position.x, 
                      target_pose.pose.position.y, 
                      target_pose.pose.position.z))
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        rospy.loginfo("Planning trajectory...")
        plan = arm.plan()
        
        # 处理不同版本的plan()返回值
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
            rospy.loginfo("Planning result: %s" % ("SUCCESS" if success else "FAILED"))
        else:
            trajectory = plan
            success = len(trajectory.joint_trajectory.points) > 0
            rospy.loginfo("Trajectory points: %d" % len(trajectory.joint_trajectory.points))
        
        if success:
            rospy.loginfo("Planning successful! Executing trajectory...")
            arm.execute(trajectory, wait=True)
            rospy.sleep(1)
            rospy.loginfo("Execution completed!")
        else:
            rospy.logerr("Planning failed! Trying with Cartesian path...")
            
            # 尝试笛卡尔路径规划
            waypoints = []
            waypoints.append(target_pose.pose)
            
            # 修复: compute_cartesian_path 正确的参数格式
            # 参数: waypoints, eef_step, jump_threshold, avoid_collisions
            # 第三个参数应该是 bool 类型 (avoid_collisions)，而不是 float
            (plan_cartesian, fraction) = arm.compute_cartesian_path(
                waypoints,   # 路径点列表
                0.01,        # eef_step: 步长 1cm
                True         # avoid_collisions: 避免碰撞 (这里是布尔值，不是0.0)
            )
            
            rospy.loginfo("Cartesian path completion: %.2f%%" % (fraction * 100.0))
            
            if fraction > 0.9:  # 如果90%以上的路径可规划
                rospy.loginfo("Executing Cartesian path...")
                arm.execute(plan_cartesian, wait=True)
                rospy.sleep(1)
            else:
                rospy.logerr("Cartesian path planning also failed!")
                rospy.logerr("Possible reasons:")
                rospy.logerr("  1. Target pose is outside workspace")
                rospy.logerr("  2. Target pose causes collision")
                rospy.logerr("  3. No valid IK solution exists")
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItIkDemo()
    except rospy.ROSInterruptException:
        pass