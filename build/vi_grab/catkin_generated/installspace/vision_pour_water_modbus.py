#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from vi_grab.msg import ObjectInfo
from geometry_msgs.msg import TransformStamped, PointStamped
from geometry_msgs.msg import Point, Quaternion

# 全局变量：MoveIt arm对象
arm = None

# 抓取标志位，只抓取一次的时候才用
catch_flag = True

# 相机坐标系物体到机械臂基坐标系转换函数
def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    """
    我们需要将旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
    机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z, rx, ry, rz）
    """
    # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量
    rotation_matrix = np.array([[0.01206237, 0.99929647, 0.03551135],
                                [-0.99988374, 0.01172294, 0.00975125],
                                [0.00932809, -0.03562485, 0.9993217]])
    translation_vector = np.array([-0.08039019, 0.03225555, -0.08256825])
    
    # 深度相机识别物体返回的坐标
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
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
    obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标
    
    # 计算物体的旋转
    obj_orientation_matrix = T_base_to_end_effector[:3, :3].dot(rotation_matrix)
    obj_orientation_euler = R.from_matrix(obj_orientation_matrix).as_euler('xyz', degrees=False)
    
    # 组合结果
    obj_base_pose = np.hstack((obj_base_coordinates, obj_orientation_euler))
    obj_base_pose[3:] = rx, ry, rz
    return obj_base_pose


# 接收到识别物体的回调函数
def object_pose_callback(data):
    # 标志位设置为全局变量
    global catch_flag, arm, object_msg
    
    # 判断当前帧的识别结果是否有要抓取的物体
    if data.object_class == object_msg.data and catch_flag:
        print(data)
        
        # 使用MoveIt获取当前机械臂末端位姿
        current_pose = arm.get_current_pose().pose
        print("Current end effector pose:")
        print(current_pose)
        rospy.sleep(1)
        
        # 使用MoveIt获取当前关节角度
        current_joints = arm.get_current_joint_values()
        print("Current joint values:")
        print(current_joints)
        
        # 将四元数转换为欧拉角（用于convert函数）
        quat = [current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w]
        euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        
        # 计算机械臂基坐标系下的物体坐标
        result = convert(data.x, data.y, data.z,
                        current_pose.position.x, current_pose.position.y, current_pose.position.z,
                        euler[0], euler[1], euler[2])
        print(data.object_class, ':', result)
        
        # 运动到准备抓取姿态
        before_catch()
        rospy.sleep(3)
        
        # 抓取物体
        catch(result, current_pose)
        print('---------------------------ok------------------------------')
        catch_flag = False
        
        # 抓取完一次就关闭ros
        rospy.signal_shutdown("****************catch completed****************")


def before_catch():
    '''
    函数功能：运动到准备抓取姿态
    输入参数：无
    返回值：无
    '''
    global arm
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    
    # 设置准备抓取姿态的关节角度
    pic_joint = [-0.09342730045318604, -0.8248963952064514, 1.5183943510055542,
                 0.06789795309305191, 0.8130478262901306, 0.015879500657320023]
    arm.set_joint_value_target(pic_joint)
    
    # 控制机械臂完成运动
    arm.go()
    rospy.sleep(0.5)
    
    # 打开夹爪
    gripper_open()


def catch(result, current_pose):
    '''
    函数功能：机械臂执行抓取和倒水动作
    输入参数：经过convert函数转换得到的'result'和机械臂当前的位姿'current_pose'
    返回值：无
    '''
    global arm
    
    # 第一步：运动到物体附近（偏移5cm）
    print('*************************catching*************************')
    movejp_type([result[0] + 0.05, result[1], result[2],
                 current_pose.orientation.x, current_pose.orientation.y,
                 current_pose.orientation.z, current_pose.orientation.w], 0.3)
    rospy.sleep(5.0)
    
    # 第二步：直线运动到物体坐标处
    movel_type([result[0], result[1], result[2],
                current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w], 0.2)
    rospy.sleep(3)
    
    # 第三步：闭合夹爪
    gripper_close()
    
    print('*************************moving*************************')
    rospy.sleep(5.0)
    
    # 第四步：运动到倒水位置
    movejp_type([-0.118, -0.26, 0.558,
                 current_pose.orientation.x, current_pose.orientation.y,
                 current_pose.orientation.z, current_pose.orientation.w], 0.2)
    rospy.sleep(3)
    print('*************************waiting*************************')
    
    rospy.sleep(5.0)
    
    # 第五步：获取当前关节角度，旋转关节6进行倒水
    current_joints = arm.get_current_joint_values()
    movej_type([current_joints[0], current_joints[1], current_joints[2],
                current_joints[3], current_joints[4], current_joints[5] - 1], 0.2)
    rospy.sleep(3)
    
    # 第六步：恢复关节6角度
    movej_type([current_joints[0], current_joints[1], current_joints[2],
                current_joints[3], current_joints[4], current_joints[5]], 0.2)
    rospy.sleep(3)
    print('*************************pouring*************************')


def movej_type(joint_positions, speed=0.5):
    '''
    函数功能：通过输入机械臂每个关节的数值（弧度），让机械臂运动到指定姿态
    输入参数：joint_positions [joint1,joint2,joint3,joint4,joint5,joint6]、speed (0-1)
    返回值：无
    '''
    global arm
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    
    # 设置机械臂的目标位置
    arm.set_joint_value_target(joint_positions)
    
    # 控制机械臂完成运动
    arm.go()
    rospy.sleep(0.5)


def movejp_type(pose, speed=0.5):
    '''
    函数功能：通过输入机械臂末端的位姿数值，让机械臂运动到指定位姿（关节空间规划）
    输入参数：pose（position.x、position.y、position.z、orientation.x、orientation.y、orientation.z、orientation.w）、speed
    返回值：无
    '''
    global arm
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
    
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    
    # 设置位置和姿态的允许误差
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    
    # 设置机械臂工作空间中的目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = pose[0]
    target_pose.pose.position.y = pose[1]
    target_pose.pose.position.z = pose[2]
    target_pose.pose.orientation.x = pose[3]
    target_pose.pose.orientation.y = pose[4]
    target_pose.pose.orientation.z = pose[5]
    target_pose.pose.orientation.w = pose[6]
    
    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()
    
    # 设置机械臂终端运动的目标位姿
    arm.set_pose_target(target_pose, end_effector_link)
    
    # 规划并执行运动路径
    arm.go()
    rospy.sleep(0.5)


def movel_type(pose, speed=0.5):
    '''
    函数功能：通过输入机械臂末端的位姿数值，让机械臂直线运动到指定位姿（笛卡尔空间规划）
    输入参数：pose（position.x、position.y、position.z、orientation.x、orientation.y、orientation.z、orientation.w）、speed
    返回值：无
    '''
    global arm
    
    # 获取终端link的名称
    end_effector_link = arm.get_end_effector_link()
    
    # 设置目标位置所使用的参考坐标系
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)
    
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    
    # 设置位置和姿态的允许误差
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.01)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    
    # 设置机械臂工作空间中的目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = pose[0]
    target_pose.pose.position.y = pose[1]
    target_pose.pose.position.z = pose[2]
    target_pose.pose.orientation.x = pose[3]
    target_pose.pose.orientation.y = pose[4]
    target_pose.pose.orientation.z = pose[5]
    target_pose.pose.orientation.w = pose[6]
    
    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()
    
    # 设置机械臂终端运动的目标位姿
    arm.set_pose_target(target_pose, end_effector_link)
    
    # 规划运动路径
    traj = arm.plan()
    
    # 按照规划的运动路径控制机械臂运动
    arm.execute(traj)
    rospy.sleep(0.5)


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
    arm.go()
    rospy.sleep(0.5)
    print("Gripper opened (joint6 = 0.0)")


def gripper_close():
    '''
    函数功能：闭合夹爪（通过设置关节6的角度为0.8）
    输入参数：无
    返回值：无
    '''
    global arm
    
    # 获取当前所有关节角度
    current_joints = arm.get_current_joint_values()
    
    # 设置关节6（索引5）为0.8弧度（闭合夹爪）
    current_joints[5] = 0.8
    
    # 设置机械臂运动的允许误差值
    arm.set_goal_joint_tolerance(0.001)
    
    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    
    # 设置目标关节角度
    arm.set_joint_value_target(current_joints)
    
    # 执行运动
    arm.go()
    rospy.sleep(0.5)
    print("Gripper closed (joint6 = 0.8)")


if __name__ == '__main__':
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 初始化ROS节点
    rospy.init_node('object_catch', anonymous=True)
    
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = moveit_commander.MoveGroupCommander('manipulator')
    
    # 等待接收要抓取的物体信息
    object_msg = rospy.wait_for_message('/choice_object', String, timeout=None)
    
    # 订阅物体位姿信息
    sub_object_pose = rospy.Subscriber("/object_pose", ObjectInfo, object_pose_callback, queue_size=1)
    
    # 保持节点运行
    rospy.spin()
    
    print('---------------------------final----------------------------')
    
    # 关闭并退出moveit
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)