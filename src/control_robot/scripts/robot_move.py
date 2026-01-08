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

from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, PositionIKRequest
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_msgs.msg import ObjectInfo
import math
from moveit_commander import MoveGroupCommander

# 全局变量
arm = None
ik_service = None

# ===================== 逆运动学函数 =====================
def compute_ik_from_pose(target_pose, timeout=5.0):

    global arm, ik_service
    
    try:
        # 等待IK服务
        if ik_service is None:
            rospy.loginfo("等待IK服务 /compute_ik ...")
            rospy.wait_for_service('/compute_ik', timeout=5.0)
            ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
            rospy.loginfo("IK服务已连接")
        
        # 构建IK请求
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.robot_state = RobotState()
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout = rospy.Duration(timeout)
        
        # 设置目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = target_pose
        ik_request.ik_request.pose_stamped = pose_stamped
        
        # 调用IK服务
        response = ik_service(ik_request)
        
        if response.error_code.val == response.error_code.SUCCESS:
            joint_names = response.solution.joint_state.name
            joint_values = list(response.solution.joint_state.position)
            
            # 只取arm组的关节
            arm_joint_names = arm.get_active_joints()
            arm_joint_values = []
            for name in arm_joint_names:
                if name in joint_names:
                    idx = joint_names.index(name)
                    arm_joint_values.append(joint_values[idx])
            
            rospy.loginfo("IK计算成功!")
            rospy.loginfo("关节角: %s" % arm_joint_values)
            return arm_joint_values
        else:
            rospy.logwarn("IK计算失败，错误码: %d" % response.error_code.val)
            return None
            
    except rospy.ServiceException as e:
        rospy.logerr("IK服务调用失败: %s" % e)
        return None
    except rospy.ROSException as e:
        rospy.logerr("等待IK服务超时: %s" % e)
        return None
    

def normalize_quaternion(qx, qy, qz, qw):
    """归一化四元数"""
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx/norm, qy/norm, qz/norm, qw/norm


def compute_ik_from_xyz_quat(x, y, z, qx, qy, qz, qw):

    # 归一化四元数
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
    
    # 构建Pose
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw
    
    return compute_ik_from_pose(target_pose)


def compute_ik_from_pose_stamped(pose_stamped):
    
    return compute_ik_from_xyz_quat(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z,
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w
    )

# ===================== 运动控制 =====================
def movej_type(joint_positions, speed=0.5):

    """关节空间运动"""
    global arm
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    arm.set_joint_value_target(joint_positions)
    success = arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    return success


if __name__ == '__main__':
    # 初始化MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 初始化ROS节点
    rospy.init_node('get_end_effector_pose_py')
    
    # 定义规划组的名称
    planning_group = "arm"
    
    # 创建MoveGroupCommander对象
    arm = MoveGroupCommander(planning_group)
    
    # 获取当前末端执行器的位姿 (返回的是PoseStamped对象)
    current_pose = arm.get_current_pose()
    
    # 打印当前位姿信息
    rospy.loginfo("=" * 60)
    rospy.loginfo("当前末端位姿:")
    rospy.loginfo("  位置: x=%.4f, y=%.4f, z=%.4f" % (
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    ))
    rospy.loginfo("  姿态: qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f" % (
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    ))
    rospy.loginfo("=" * 60)
    
    # 方法1: 使用新增的compute_ik_from_pose_stamped函数
    rospy.loginfo("方法1: 使用 compute_ik_from_pose_stamped")
    joints = compute_ik_from_pose_stamped(current_pose)
    
    if joints is not None:
        rospy.loginfo("计算得到的关节角:")
        print(joints)
        
        # 对比当前实际关节角
        current_joints = arm.get_current_joint_values()
        rospy.loginfo("当前实际关节角:")
        print(current_joints)
    else:
        rospy.logerr("IK计算失败!")
    
    joints[2] = joints[2] + 0.2
    print(joints)

    success = movej_type(joints)
    
    # 关闭MoveIt
    moveit_commander.roscpp_shutdown()