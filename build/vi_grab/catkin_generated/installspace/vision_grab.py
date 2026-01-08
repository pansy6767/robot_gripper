#!/home/airlab1/miniconda3/envs/ros1/bin/python3
# -*- coding=UTF-8 -*-
from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_msgs.msg import ObjectInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from geometry_msgs.msg import Point, Quaternion
import math

# # ===================== 手眼标定参数 d415 =====================
# rotation_matrix = np.array([[1.0, 0.0, 0.0],
#                             [0.0, 0.0, 1.0],
#                             [0.0, -1.0, 0.0]])
# translation_vector = np.array([-0.017, -0.118, 0.0815])

# ===================== 手眼标定参数 d435if =====================
rotation_matrix = np.array([[1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0]])
translation_vector = np.array([-0.0125, -0.113, 0.0815])

# 全局变量：MoveIt arm对象
arm = None

# 相机坐标系物体到机械臂基坐标系转换函数
def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    """
    函数功能：我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
    机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z, rx, ry, rz）
    输入参数：深度相机识别到的物体坐标（x, y, z）和机械臂末端的位姿（x1,y1,z1,rx,ry,rz）
    返回值：物体在机械臂基座坐标系下的位置（x, y, z）
    """
    global rotation_matrix, translation_vector
    obj_camera_coordinates = np.array([x, y, z])

    # 机械臂末端的位姿，单位为弧度
    end_effector_pose = np.array([x1, y1, z1, rx, ry, rz])
    
    # 将旋转矩阵和平移向量转换为齐次变换矩阵
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector
    
    # 机械臂末端的位姿转换为齐次变换矩阵
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position
    
    # 计算物体相对于机械臂基座的位姿
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])
    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
    obj_base_coordinates = obj_base_coordinates_homo[:3]
    
    # 计算物体的旋转
    obj_orientation_matrix = T_base_to_end_effector[:3, :3].dot(rotation_matrix)
    obj_orientation_euler = R.from_matrix(obj_orientation_matrix).as_euler('xyz', degrees=False)
    
    # 组合结果
    obj_base_pose = np.hstack((obj_base_coordinates, obj_orientation_euler))
    obj_base_pose[3:] = rx, ry, rz
    return obj_base_pose


def normalize_quaternion(qx, qy, qz, qw):
    """
    函数功能：归一化四元数
    输入参数：四元数的四个分量 (qx, qy, qz, qw)
    返回值：归一化后的四元数 (qx, qy, qz, qw)
    """
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-6:
        rospy.logwarn("Quaternion norm too small, using identity quaternion")
        return 0.0, 0.0, 0.0, 1.0
    return qx/norm, qy/norm, qz/norm, qw/norm


# 接收到识别物体的回调函数
def object_pose_callback(data):
    """
    函数功能：每帧图像经过识别后的回调函数，若有抓取指令，则判断当前画面帧中是否有被抓物体，如果有则将物体坐标进行转换，并让机械臂执行抓取动作
    输入参数：无
    返回值：无
    """
    global object_msg, arm
    
    # 判断当前帧的识别结果是否有要抓取的物体
    if data.object_class == object_msg.data and len(object_msg.data) > 0:
        # 使用MoveIt获取当前机械臂末端位姿
        current_pose = arm.get_current_pose().pose
        print("=" * 60)
        print("Current end effector pose:")
        print(current_pose)
        rospy.sleep(1)
        
        # 使用MoveIt获取当前关节角度
        current_joints = arm.get_current_joint_values()
        print("Current joint values (rad):")
        print(current_joints)
        print("=" * 60)
        
        # 将四元数转换为欧拉角（用于convert函数）
        quat = [current_pose.orientation.x, current_pose.orientation.y, 
                current_pose.orientation.z, current_pose.orientation.w]
        euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        
        # 计算机械臂基坐标系下的物体坐标
        result = convert(data.x, data.y, data.z, 
                        current_pose.position.x, current_pose.position.y, current_pose.position.z,
                        euler[0], euler[1], euler[2])
        
        print("=" * 60)
        print("Object '%s' calculated position:" % data.object_class)
        print("x: %.4f, y: %.4f, z: %.4f" % (result[0], result[1], result[2]))
        print("=" * 60)
        
        # 抓取物体（传递当前位姿和关节角度）
        catch(result, current_pose, current_joints)
        
        # 清除object_msg的信息，之后二次发布抓取物体信息可以再执行
        object_msg.data = ''


def movej_type(joint_positions, speed=0.5):
    '''
    函数功能：通过输入机械臂每个关节的数值（弧度），让机械臂运动到指定姿态
    输入参数：joint_positions [joint1,joint2,joint3,joint4,joint5,joint6]、speed (0-1)
    返回值：无
    '''
    global arm
    
    print("=" * 60)
    print("movej_type - Target joint values (rad):")
    print(joint_positions)
    print("=" * 60)
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    
    # 设置机械臂的目标位置
    arm.set_joint_value_target(joint_positions)
    
    # 控制机械臂完成运动
    success = arm.go(wait=True)
    arm.stop()
    
    if success:
        rospy.loginfo("movej_type succeeded!")
    else:
        rospy.logwarn("movej_type failed!")
    
    rospy.sleep(0.5)
    return success


def movejp_type(pose, speed=0.5):
    '''
    函数功能：通过输入机械臂末端的位姿数值，让机械臂运动到指定位姿（关节空间规划）
    输入参数：pose（position.x、position.y、position.z、orientation.x、orientation.y、orientation.z、orientation.w）、speed
    返回值：无
    '''
    global arm
    
    # 四元数归一化
    qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
    qx_norm, qy_norm, qz_norm, qw_norm = normalize_quaternion(qx, qy, qz, qw)
    
    print("=" * 60)
    print("movejp_type - Target pose:")
    print("Position (x, y, z): %.4f, %.4f, %.4f" % (pose[0], pose[1], pose[2]))
    print("Quaternion (x, y, z, w): %.4f, %.4f, %.4f, %.4f" % (qx_norm, qy_norm, qz_norm, qw_norm))
    print("Quaternion norm: %.6f" % math.sqrt(qx_norm**2 + qy_norm**2 + qz_norm**2 + qw_norm**2))
    print("=" * 60)
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
    
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    
    # 放宽容差
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.05)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    
    # 增加规划时间和尝试次数
    arm.set_planning_time(15)
    arm.set_num_planning_attempts(10)
    
    # 设置机械臂工作空间中的目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = pose[0]
    target_pose.pose.position.y = pose[1]
    target_pose.pose.position.z = pose[2]
    target_pose.pose.orientation.x = qx_norm
    target_pose.pose.orientation.y = qy_norm
    target_pose.pose.orientation.z = qz_norm
    target_pose.pose.orientation.w = qw_norm
    
    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()
    
    # 设置机械臂终端运动的目标位姿
    arm.set_pose_target(target_pose, end_effector_link)
    
    # 规划运动路径
    rospy.loginfo("Planning trajectory (movejp)...")
    plan = arm.plan()
    
    # ROS Noetic: plan() 返回 (success, trajectory, planning_time, error_code)
    if isinstance(plan, tuple):
        success = plan[0]
        trajectory = plan[1]
        rospy.loginfo("Planning result: %s" % ("SUCCESS" if success else "FAILED"))
    else:
        # 兼容旧版本
        trajectory = plan
        success = len(trajectory.joint_trajectory.points) > 0
        rospy.loginfo("Trajectory points: %d" % len(trajectory.joint_trajectory.points))
    
    if success:
        rospy.loginfo("Executing trajectory...")
        arm.execute(trajectory, wait=True)
        rospy.loginfo("movejp_type succeeded!")
    else:
        rospy.logwarn("movejp_type failed!")
    
    arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(0.5)
    return success


def arm_ready_pose():
    '''
    函数功能：执行整个抓取流程前先运动到一个能够稳定获取物体坐标信息的姿态
    输入参数：无
    返回值：无
    '''
    global arm
    
    rospy.loginfo("Moving to ready pose...")
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    
    # 设置拍照姿态的关节角度
    pic_joint = [-0.0, -0.804, 0.633, 0.073, -0.361, 0.0]
    arm.set_joint_value_target(pic_joint)
    
    # 控制机械臂完成运动
    success = arm.go(wait=True)
    arm.stop()
    
    if success:
        rospy.loginfo("Ready pose reached!")
    else:
        rospy.logwarn("Failed to reach ready pose!")
    
    rospy.sleep(0.5)


def catch(result, current_pose, current_joints):
    '''
    函数功能：机械臂执行抓取动作
    输入参数：经过convert函数转换得到的'result'、当前末端位姿'current_pose'和当前关节角度'current_joints'
    返回值：无
    '''
    # 流程第一步：运动到物体附近（偏移7cm）
    print('\n' + '=' * 60)
    print('CATCHING STEP 1: Move to 7cm above object')
    print('Target z: %.4f' % (result[2] + 0.07))
    print('=' * 60)
    success = movejp_type([result[0], result[1], result[2]+0.07, 
                           current_pose.orientation.x, current_pose.orientation.y,
                           current_pose.orientation.z, current_pose.orientation.w], 0.3)
    if not success:
        rospy.logerr("Step 1 failed! Aborting catch sequence.")
        return

    # 抓取第二步：直线运动到物体坐标处
    print('\n' + '=' * 60)
    print('CATCHING STEP 2: Move down to object')
    print('Target z: %.4f' % result[2])
    print('=' * 60)
    success = movejp_type([result[0], result[1], result[2]-0.01, 
                          current_pose.orientation.x, current_pose.orientation.y,
                          current_pose.orientation.z, current_pose.orientation.w], 0.3)
    if not success:
        rospy.logerr("Step 2 failed! Aborting catch sequence.")
        return

    # 抓取第三步：闭合夹爪
    print('\n' + '=' * 60)
    print('CATCHING STEP 3: Close gripper')
    print('=' * 60)
    gripper_close()

    # # 倒水第一步：抬起物体5厘米
    # print('\n' + '=' * 60)
    # print('POUR STEP 1: Lift object 5cm')
    # print('Target z: %.4f' % (result[2] + 0.05))
    # print('=' * 60)
    # success = movejp_type([result[0], result[1], result[2] + 0.07, 
    #                       current_pose.orientation.x, current_pose.orientation.y,
    #                       current_pose.orientation.z, current_pose.orientation.w], 0.3)

    # 
    current_joints = arm.get_current_joint_values()
    print("Current joint values (rad):")
    current_joints[0] = -1.53
    # current_joints[2] = 1.412

    print(current_joints)

    success = movej_type(current_joints)


    if not success:
        rospy.logerr("Pour step 1 failed!")
        return
    

    gripper_open()
    
    rospy.loginfo("Catch and pour sequence completed!")


def gripper_open():
    '''
    函数功能：打开夹爪（通过设置关节6的角度为0）
    输入参数：无
    返回值：无
    '''
    global arm
    
    # 获取当前所有关节角度
    current_joints = arm.get_current_joint_values()
    
    # 设置关节6（索引5）为0弧度（打开夹爪）
    current_joints[5] = 0.0
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    
    # 设置目标关节角度
    arm.set_joint_value_target(current_joints)
    
    # 执行运动
    arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    rospy.loginfo("Gripper opened (joint6 = 0.0)")


def gripper_close():
    '''
    函数功能：闭合夹爪（通过设置关节6的角度为0.6）
    输入参数：无
    返回值：无
    '''
    global arm
    
    # 获取当前所有关节角度
    current_joints = arm.get_current_joint_values()
    
    # 设置关节6（索引5）为0.6弧度（闭合夹爪）
    current_joints[5] = 0.7
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    
    # 设置目标关节角度
    arm.set_joint_value_target(current_joints)
    
    # 执行运动
    arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    rospy.loginfo("Gripper closed (joint6 = 0.6)")


if __name__ == '__main__':
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 初始化ROS节点
    rospy.init_node('object_catch', anonymous=True)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("Object Catch Node Started - ROS Noetic")
    rospy.loginfo("=" * 60)
    
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = moveit_commander.MoveGroupCommander('arm')
    
    # 移动到准备姿态
    arm_ready_pose()
    
    # 初始化打开夹爪
    # gripper_open()
    
    rospy.loginfo("Waiting for object selection on /choice_object topic...")
    
    # 等待接收要抓取的物体信息
    object_msg = rospy.wait_for_message('/choice_object', String, timeout=None)
    
    rospy.loginfo("Object selected: %s" % object_msg.data)
    rospy.loginfo("Subscribing to /object_pose topic...")
    
    # 订阅物体位姿信息
    sub_object_pose = rospy.Subscriber("/object_pose", ObjectInfo, object_pose_callback, queue_size=1)
    
    # 保持节点运行
    rospy.spin()
    
    # 关闭并退出moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)