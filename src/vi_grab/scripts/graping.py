# #!/home/airlab1/miniconda3/envs/ros1/bin/python3
# # -*- coding=UTF-8 -*-
# """
# 颜色形状抓取节点
# 通过发送 "颜色_形状" 信号（如 "red_circle"）控制机械臂抓取对应物体
# """
# from std_msgs.msg import String, Bool, Empty
# import rospy, sys
# import moveit_commander
# from geometry_msgs.msg import Pose, PoseStamped
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# from vi_msgs.msg import ObjectInfo
# import math

# # ===================== 手眼标定参数 =====================
# # 相机坐标系到机械臂末端坐标系的旋转矩阵
# rotation_matrix = np.array([[1.0, 0.0, 0.0],
#                             [0.0, 0.0, 1.0],
#                             [0.0, -1.0, 0.0]])
# # 相机坐标系到机械臂末端坐标系的平移向量
# translation_vector = np.array([-0.017, -0.118, 0.0815])

# # 全局变量
# arm = None
# target_object = ""  # 目标物体，格式: "颜色_形状"，如 "red_circle"


# # ===================== 坐标转换 =====================
# def convert(x, y, z, x1, y1, z1, rx, ry, rz):
#     """相机坐标系物体坐标 → 机械臂基坐标系"""
#     global rotation_matrix, translation_vector
#     obj_camera_coordinates = np.array([x, y, z])
#     end_effector_pose = np.array([x1, y1, z1, rx, ry, rz])
    
#     # 相机到末端的齐次变换矩阵
#     T_camera_to_end_effector = np.eye(4)
#     T_camera_to_end_effector[:3, :3] = rotation_matrix
#     T_camera_to_end_effector[:3, 3] = translation_vector
    
#     # 末端到基座的齐次变换矩阵
#     position = end_effector_pose[:3]
#     orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
#     T_base_to_end_effector = np.eye(4)
#     T_base_to_end_effector[:3, :3] = orientation
#     T_base_to_end_effector[:3, 3] = position
    
#     # 计算物体在基坐标系下的位置
#     obj_camera_homo = np.append(obj_camera_coordinates, [1])
#     obj_end_effector_homo = T_camera_to_end_effector.dot(obj_camera_homo)
#     obj_base_homo = T_base_to_end_effector.dot(obj_end_effector_homo)
#     obj_base_coordinates = obj_base_homo[:3]
    
#     obj_base_pose = np.hstack((obj_base_coordinates, [rx, ry, rz]))
#     return obj_base_pose


# def normalize_quaternion(qx, qy, qz, qw):
#     """归一化四元数"""
#     norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
#     if norm < 1e-6:
#         return 0.0, 0.0, 0.0, 1.0
#     return qx/norm, qy/norm, qz/norm, qw/norm


# # ===================== 回调函数 =====================
# def object_pose_callback(data):
#     """接收物体位姿，匹配目标后执行抓取"""
#     global target_object, arm
    
#     # 检查是否匹配目标物体（格式: "颜色_形状"）
#     if data.object_class == target_object and len(target_object) > 0:
#         rospy.loginfo("=" * 60)
#         rospy.loginfo("目标物体匹配: %s" % data.object_class)
#         rospy.loginfo("相机坐标: (%.3f, %.3f, %.3f)" % (data.x, data.y, data.z))
        
#         # 获取当前末端位姿
#         current_pose = arm.get_current_pose().pose
#         current_joints = arm.get_current_joint_values()
        
#         # 四元数转欧拉角
#         quat = [current_pose.orientation.x, current_pose.orientation.y,
#                 current_pose.orientation.z, current_pose.orientation.w]
#         euler = R.from_quat(quat).as_euler('xyz', degrees=False)
        
#         # 坐标转换
#         result = convert(data.x, data.y, data.z,
#                         current_pose.position.x, current_pose.position.y, current_pose.position.z,
#                         euler[0], euler[1], euler[2])
        
#         result[2] = result[2]-0.02
        
#         rospy.loginfo("基坐标系位置: (%.4f, %.4f, %.4f)" % (result[0], result[1], result[2]))
#         rospy.loginfo("=" * 60)
        
#         # 执行抓取
#         catch(result, current_pose, current_joints)
        
#         # 清除目标，防止重复抓取
#         target_object = ""


# def choice_callback(msg):
#     """接收抓取目标选择"""
#     global target_object
#     target_object = msg.data
#     rospy.loginfo("收到抓取指令: %s" % target_object)


# # ===================== 运动控制 =====================
# def movej_type(joint_positions, speed=0.5):
#     """关节空间运动"""
#     global arm
#     arm.set_goal_joint_tolerance(0.001)
#     arm.set_max_acceleration_scaling_factor(speed)
#     arm.set_max_velocity_scaling_factor(speed)
#     arm.set_joint_value_target(joint_positions)
#     success = arm.go(wait=True)
#     arm.stop()
#     rospy.sleep(0.5)
#     return success


# def movejp_type(pose, speed=0.5):
#     """末端位姿运动（关节空间规划）"""
#     global arm
    
#     # 四元数归一化
#     qx, qy, qz, qw = normalize_quaternion(pose[3], pose[4], pose[5], pose[6])
    
#     end_effector_link = arm.get_end_effector_link()
#     arm.set_pose_reference_frame('base_link')
#     arm.allow_replanning(True)
#     arm.set_goal_position_tolerance(0.01)
#     arm.set_goal_orientation_tolerance(0.05)
#     arm.set_max_acceleration_scaling_factor(speed)
#     arm.set_max_velocity_scaling_factor(speed)
#     arm.set_planning_time(15)
#     arm.set_num_planning_attempts(10)
    
#     # 设置目标位姿
#     target_pose = PoseStamped()
#     target_pose.header.frame_id = 'base_link'
#     target_pose.header.stamp = rospy.Time.now()
#     target_pose.pose.position.x = pose[0]
#     target_pose.pose.position.y = pose[1]
#     target_pose.pose.position.z = pose[2]
#     target_pose.pose.orientation.x = qx
#     target_pose.pose.orientation.y = qy
#     target_pose.pose.orientation.z = qz
#     target_pose.pose.orientation.w = qw
    
#     arm.set_start_state_to_current_state()
#     arm.set_pose_target(target_pose, end_effector_link)
    
#     # 规划并执行
#     plan = arm.plan()
#     if isinstance(plan, tuple):
#         success, trajectory = plan[0], plan[1]
#     else:
#         trajectory = plan
#         success = len(trajectory.joint_trajectory.points) > 0
    
#     if success:
#         arm.execute(trajectory, wait=True)
    
#     arm.stop()
#     arm.clear_pose_targets()
#     rospy.sleep(0.5)
#     return success


# def arm_ready_pose():
#     """移动到拍照准备姿态"""
#     global arm
#     rospy.loginfo("移动到准备姿态...")
#     arm.set_goal_joint_tolerance(0.001)
#     arm.set_max_acceleration_scaling_factor(0.3)
#     arm.set_max_velocity_scaling_factor(0.3)
    
#     pic_joint = [0.12,-1.14,0.69,-0.11,-0.59,-0.00]

#     arm.set_joint_value_target(pic_joint)
#     success = arm.go(wait=True)
#     arm.stop()
#     rospy.sleep(0.5)
#     return success


# def gripper_open():
#     """打开夹爪"""
#     global arm
#     current_joints = arm.get_current_joint_values()
#     current_joints[5] = 0.0
#     arm.set_goal_joint_tolerance(0.001)
#     arm.set_max_acceleration_scaling_factor(0.3)
#     arm.set_max_velocity_scaling_factor(0.3)
#     arm.set_joint_value_target(current_joints)
#     arm.go(wait=True)
#     arm.stop()
#     rospy.sleep(0.5)
#     rospy.loginfo("夹爪已打开")


# def gripper_close():
#     """闭合夹爪"""
#     global arm
#     current_joints = arm.get_current_joint_values()
#     current_joints[5] = 0.8
#     arm.set_goal_joint_tolerance(0.001)
#     arm.set_max_acceleration_scaling_factor(0.3)
#     arm.set_max_velocity_scaling_factor(0.3)
#     arm.set_joint_value_target(current_joints)
#     arm.go(wait=True)
#     arm.stop()
#     rospy.sleep(0.5)
#     rospy.loginfo("夹爪已闭合")


# def catch(result, current_pose, current_joints):
#     """执行抓取动作序列"""
#     rospy.loginfo("开始抓取序列...")
    
#     # 第1步：移动到物体上方7cm
#     rospy.loginfo("Step 1: 移动到物体上方 7cm")
#     success = movejp_type([result[0], result[1], result[2] + 0.07,
#                            current_pose.orientation.x, current_pose.orientation.y,
#                            current_pose.orientation.z, current_pose.orientation.w], 0.3)
#     if not success:
#         rospy.logerr("Step 1 失败!")
#         return
    
#     # 第2步：下降到物体位置
#     rospy.loginfo("Step 2: 下降到物体位置")
#     success = movejp_type([result[0], result[1], result[2],
#                            current_pose.orientation.x, current_pose.orientation.y,
#                            current_pose.orientation.z, current_pose.orientation.w], 0.3)
#     if not success:
#         rospy.logerr("Step 2 失败!")
#         return
    
#     # 第3步：闭合夹爪
#     rospy.loginfo("Step 3: 闭合夹爪")
#     gripper_close()
    
#     # # 第4步：抬起并移动到放置位置
#     # rospy.loginfo("Step 4: 抬起并移动到放置位置")
#     # current_joints = arm.get_current_joint_values()
#     # current_joints[0] = -1.53  # 旋转到放置位置
#     # success = movej_type(current_joints)
#     # if not success:
#     #     rospy.logerr("Step 4 失败!")
#     #     return
    
#     # # 第5步：打开夹爪释放物体
#     # rospy.loginfo("Step 5: 打开夹爪")
#     # gripper_open()
    
#     rospy.loginfo("抓取序列完成!")
    
#     # 返回准备姿态
#     # arm_ready_pose()


# # ===================== 主程序 =====================
# if __name__ == '__main__':
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('color_shape_catch', anonymous=True)
    
#     rospy.loginfo("=" * 60)
#     rospy.loginfo("颜色形状抓取节点启动")
#     rospy.loginfo("=" * 60)
#     rospy.loginfo("使用方法:")
#     rospy.loginfo("  rostopic pub /choice_object std_msgs/String \"red_circle\"")
#     rospy.loginfo("  rostopic pub /choice_object std_msgs/String \"red_square\"")
#     rospy.loginfo("  rostopic pub /choice_object std_msgs/String \"green_rectangle\"")
#     rospy.loginfo("=" * 60)
    
#     # 初始化机械臂
#     arm = moveit_commander.MoveGroupCommander('arm')
    
#     # 移动到准备姿态
#     arm_ready_pose()
#     gripper_open()
    
#     # 订阅话题
#     rospy.Subscriber("/choice_object", String, choice_callback, queue_size=1)
#     rospy.Subscriber("/object_pose", ObjectInfo, object_pose_callback, queue_size=1)
    
#     rospy.loginfo("等待抓取指令...")
#     rospy.spin()
    
#     moveit_commander.roscpp_shutdown()



#!/home/airlab1/miniconda3/envs/ros1/bin/python3
# -*- coding=UTF-8 -*-
"""
颜色形状抓取节点（带瑕疵过滤）
功能：
1. 通过发送 "颜色_形状" 信号控制机械臂抓取对应物体
2. 支持瑕疵过滤模式：只抓取合格品或不合格品
3. 支持质量评分过滤

使用方法：
  rostopic pub /choice_object std_msgs/String "red_circle"           # 抓取红色圆形
  rostopic pub /choice_object std_msgs/String "red_circle:ok"        # 只抓合格的红色圆形
  rostopic pub /choice_object std_msgs/String "red_circle:ng"        # 只抓不合格的红色圆形
  rostopic pub /choice_object std_msgs/String "any:ng"               # 抓取任意不合格品
"""
from std_msgs.msg import String, Bool, Empty
import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, PositionIKRequest
from scipy.spatial.transform import Rotation as R
from vi_msgs.msg import ObjectInfo
import math

# # ===================== 手眼标定参数 =====================
# rotation_matrix = np.array([[1.0, 0.0, 0.0],
#                             [0.0, 0.0, 1.0],
#                             [0.0, -1.0, 0.0]])
# translation_vector = np.array([-0.0125, -0.113, 0.0815])

# ===================== 手眼标定参数 =====================
rotation_matrix = np.array([[1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0]])
translation_vector = np.array([-0.017, -0.118, 0.0815])

# 全局变量
arm = None
target_object = ""        # 目标物体类型
target_quality = None     # 目标质量要求：None=不限, "ok"=合格, "ng"=不合格
quality_threshold = 80.0  # 质量评分阈值
ik_service = None


# ===================== 坐标转换 =====================
def convert(x, y, z, x1, y1, z1, rx, ry, rz):

    """相机坐标系物体坐标 → 机械臂基坐标系"""
    global rotation_matrix, translation_vector
    obj_camera_coordinates = np.array([x, y, z])
    end_effector_pose = np.array([x1, y1, z1, rx, ry, rz])
    
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector
    
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position
    
    obj_camera_homo = np.append(obj_camera_coordinates, [1])
    obj_end_effector_homo = T_camera_to_end_effector.dot(obj_camera_homo)
    obj_base_homo = T_base_to_end_effector.dot(obj_end_effector_homo)
    obj_base_coordinates = obj_base_homo[:3]
    
    obj_base_pose = np.hstack((obj_base_coordinates, [rx, ry, rz]))
    return obj_base_pose


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


def normalize_quaternion(qx, qy, qz, qw):

    """归一化四元数"""
    norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx/norm, qy/norm, qz/norm, qw/norm


# ===================== 回调函数 =====================
def parse_target(msg_data):

    """
    解析目标指令
    格式: "颜色_形状" 或 "颜色_形状:质量要求"
    示例: "red_circle", "red_circle:ok", "any:ng"
    """
    parts = msg_data.strip().split(':')
    obj_type = parts[0]
    quality_req = parts[1].lower() if len(parts) > 1 else None
    
    return obj_type, quality_req


def check_quality_match(quality_score, quality_requirement):

    """检查质量是否符合要求"""
    if quality_requirement is None:
        return True
    elif quality_requirement == "ok":
        return quality_score >= quality_threshold
    elif quality_requirement == "ng":
        return quality_score < quality_threshold
    return True


def object_pose_callback(data):
    """接收物体位姿，匹配目标后执行抓取"""
    global target_object, target_quality, arm
    
    # 打印收到的消息，方便调试
    rospy.loginfo_throttle(2, "收到物体: %s, 位置: (%.3f, %.3f, %.3f)" % 
                          (data.object_class, data.x, data.y, data.z))
    
    if len(target_object) == 0:
        return
    
    # 检查物体类型匹配
    type_match = (target_object == "any" or data.object_class == target_object)
    
    rospy.loginfo("检查匹配: 目标=%s, 检测到=%s, 匹配=%s" % 
                 (target_object, data.object_class, type_match))
    
    if not type_match:
        return
    
    # 注意：当前ObjectInfo消息没有quality_score字段
    # 如果使用:ok或:ng过滤，需要扩展消息类型
    # 暂时忽略质量过滤，直接抓取匹配的物体
    quality_score = 100.0  # 默认合格
    
    # 如果指定了质量要求但消息不支持，给出警告
    if target_quality is not None:
        rospy.logwarn("注意: ObjectInfo消息不包含quality_score字段，质量过滤已忽略")
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("目标物体匹配: %s" % data.object_class)
    if target_quality:
        rospy.loginfo("质量要求: %s, 实际评分: %.1f" % (target_quality.upper(), quality_score))
    rospy.loginfo("相机坐标: (%.3f, %.3f, %.3f)" % (data.x, data.y, data.z))
    
    # 获取当前末端位姿
    current_pose = arm.get_current_pose().pose
    current_joints = arm.get_current_joint_values()
    
    # 四元数转欧拉角
    quat = [current_pose.orientation.x, current_pose.orientation.y,
            current_pose.orientation.z, current_pose.orientation.w]
    euler = R.from_quat(quat).as_euler('xyz', degrees=False)
    
    # 坐标转换
    result = convert(data.x, data.y, data.z,
                    current_pose.position.x, current_pose.position.y, current_pose.position.z,
                    euler[0], euler[1], euler[2])
    
    result[0] = result[0]-0.015
    result[2] = result[2]-0.015
    
    rospy.loginfo("基坐标系位置: (%.4f, %.4f, %.4f)" % (result[0], result[1], result[2]))
    rospy.loginfo("=" * 60)
    
    # 执行抓取
    catch(result, current_pose, current_joints, is_defective=(quality_score < quality_threshold))
    
    # 清除目标，防止重复抓取
    target_object = ""
    target_quality = None


def choice_callback(msg):

    """接收抓取目标选择"""
    global target_object, target_quality
    
    target_object, target_quality = parse_target(msg.data)
    
    quality_info = f", 质量要求: {target_quality.upper()}" if target_quality else ""
    rospy.loginfo("收到抓取指令: %s%s" % (target_object, quality_info))


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


def movejp_type(pose, speed=0.5):

    """末端位姿运动"""
    global arm
    
    qx, qy, qz, qw = normalize_quaternion(pose[3], pose[4], pose[5], pose[6])
    
    end_effector_link = arm.get_end_effector_link()
    arm.set_pose_reference_frame('base_link')
    arm.allow_replanning(True)
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.05)
    arm.set_max_acceleration_scaling_factor(speed)
    arm.set_max_velocity_scaling_factor(speed)
    arm.set_planning_time(15)
    arm.set_num_planning_attempts(10)

    rospy.loginfo("  位置 - X: %.4f, Y: %.4f, Z: %.4f" % (pose[0], pose[1], pose[2]))
    rospy.loginfo("  姿态 - qx: %.4f, qy: %.4f, qz: %.4f, qw: %.4f" % (qx, qy, qz, qw))
    
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.header.stamp = rospy.Time.now()
    target_pose.pose.position.x = pose[0]
    target_pose.pose.position.y = pose[1]
    target_pose.pose.position.z = pose[2]
    target_pose.pose.orientation.x = qx
    target_pose.pose.orientation.y = qy
    target_pose.pose.orientation.z = qz
    target_pose.pose.orientation.w = qw
    
    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, end_effector_link)

    
    
    plan = arm.plan()
    if isinstance(plan, tuple):
        success, trajectory = plan[0], plan[1]
    else:
        trajectory = plan
        success = len(trajectory.joint_trajectory.points) > 0
    
    if success:
        arm.execute(trajectory, wait=True)
    
    arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(0.5)
    return success


def arm_ready_pose():

    """移动到拍照准备姿态"""
    global arm
    rospy.loginfo("移动到准备姿态...")
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    
    pic_joint = [0.12, -1.14, 0.69, -0.11, -0.59, -0.00]
    arm.set_joint_value_target(pic_joint)
    success = arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    return success


def arm_ok_place_pose():

    """移动到合格品放置位置"""
    global arm
    rospy.loginfo("移动到合格品放置位置...")
    current_joints = arm.get_current_joint_values()
    current_joints[0] = -1.0  # 放置位置1
    return movej_type(current_joints, 0.3)


def arm_ng_place_pose():

    """移动到不合格品放置位置"""
    global arm
    rospy.loginfo("移动到不合格品放置位置...")
    current_joints = arm.get_current_joint_values()
    current_joints[0] = 1.0  # 放置位置2
    return movej_type(current_joints, 0.3)


def gripper_open():

    """打开夹爪"""
    global arm
    current_joints = arm.get_current_joint_values()
    current_joints[5] = 0.0
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_joint_value_target(current_joints)
    arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    rospy.loginfo("夹爪已打开")


def gripper_close():
    """闭合夹爪"""
    global arm
    current_joints = arm.get_current_joint_values()
    current_joints[5] = 0.8
    arm.set_goal_joint_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.3)
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_joint_value_target(current_joints)
    arm.go(wait=True)
    arm.stop()
    rospy.sleep(0.5)
    rospy.loginfo("夹爪已闭合")


def catch(result, current_pose, current_joints, is_defective=False):

    """
    执行抓取动作序列
    
    Args:
        result: 目标位置
        current_pose: 当前位姿
        current_joints: 当前关节角
        is_defective: 是否是不合格品
    """
    rospy.loginfo("开始抓取序列...")
    if is_defective:
        rospy.loginfo("[!] 该物体为不合格品，将放置到NG区")
    
    # 第1步：移动到物体上方7cm
    rospy.loginfo("Step 1: 移动到物体上方 7cm")
    
    joints1 = compute_ik_from_xyz_quat(result[0], result[1], result[2] + 0.07,
                           current_pose.orientation.x, current_pose.orientation.y,
                           current_pose.orientation.z, current_pose.orientation.w)
    print(joints1)

    success1 = movej_type(joints1)
    
    if not success1:
        rospy.logerr("Step 1 失败!")
        return
    
    # 第2步：下降到物体位置
    rospy.loginfo("Step 2: 下降到物体位置")
    joints2 = compute_ik_from_xyz_quat(result[0], result[1], result[2],
                           current_pose.orientation.x, current_pose.orientation.y,
                           current_pose.orientation.z, current_pose.orientation.w)
    print(joints2)
    
    joints2[3] = -0.11
    # joints2[4] = -0.59
    joints2[5] = 0.2

    success2 = movej_type(joints2)
    if not success2:
        rospy.logerr("Step 2 失败!")
        return
    
    # 第3步：闭合夹爪
    rospy.loginfo("Step 3: 闭合夹爪")
    gripper_close()
    
    
    # 第4步
    current_joints = arm.get_current_joint_values()
    print("Current joint values (rad):")
    current_joints[2] = current_joints[2] +0.2
    print(current_joints)
    success3 = movej_type(current_joints)

    if not success3:
        rospy.logerr("Step 4 失败!")
        return
    
    # 第5步
    current_joints = arm.get_current_joint_values()
    print("Current joint values (rad):")
    current_joints[0] = -1.512
    print(current_joints)
    success4 = movej_type(current_joints)

    if not success4:
        rospy.logerr("Step 5 失败!")
        return
    
    rospy.loginfo("抓取序列完成!")

    # 第3步：闭合夹爪
    rospy.loginfo("Step 6")
    gripper_open()
    
    #返回准备姿态
    arm_ready_pose()


# ===================== 主程序 =====================
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('color_shape_catch', anonymous=True)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("颜色形状抓取节点启动（带瑕疵过滤）")
    rospy.loginfo("=" * 60)
    rospy.loginfo("使用方法:")
    rospy.loginfo("  基本抓取:")
    rospy.loginfo("    rostopic pub /choice_object std_msgs/String \"red_circle\"")
    rospy.loginfo("  只抓合格品:")
    rospy.loginfo("    rostopic pub /choice_object std_msgs/String \"red_circle:ok\"")
    rospy.loginfo("  只抓不合格品:")
    rospy.loginfo("    rostopic pub /choice_object std_msgs/String \"red_circle:ng\"")
    rospy.loginfo("  抓取任意不合格品:")
    rospy.loginfo("    rostopic pub /choice_object std_msgs/String \"any:ng\"")
    rospy.loginfo("=" * 60)
    
    # 初始化机械臂
    arm = moveit_commander.MoveGroupCommander('arm')
    
    # 移动到准备姿态
    arm_ready_pose()
    gripper_open()
    
    # 订阅话题
    rospy.Subscriber("/choice_object", String, choice_callback, queue_size=1)
    rospy.Subscriber("/object_pose", ObjectInfo, object_pose_callback, queue_size=1)
    
    rospy.loginfo("等待抓取指令...")
    rospy.spin()
    
    moveit_commander.roscpp_shutdown()