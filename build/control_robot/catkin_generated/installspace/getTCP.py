import rospy
from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('get_end_effector_pose_py')
    
    # 定义规划组的名称
    planning_group = "arm"
    
    # 创建MoveGroupCommander对象
    move_group = MoveGroupCommander(planning_group)
    
    # 设置循环频率（Hz），例如10Hz表示每秒打印10次
    rate = rospy.Rate(1) 
    
    # 使用rospy.is_shutdown()来判断节点是否应该退出
    while not rospy.is_shutdown():
        # 获取当前末端执行器的位姿
        current_pose = move_group.get_current_pose()
        
        # 打印结果
        rospy.loginfo("当前末端执行器位姿:")
        rospy.loginfo(f"参考坐标系: {current_pose.header.frame_id}")
        rospy.loginfo(f"位置 x: {current_pose.pose.position.x:.4f}, y: {current_pose.pose.position.y:.4f}, z: {current_pose.pose.position.z:.4f}")
        rospy.loginfo(f"姿态四元数 x: {current_pose.pose.orientation.x:.4f}, y: {current_pose.pose.orientation.y:.4f}, z: {current_pose.pose.orientation.z:.4f}, w: {current_pose.pose.orientation.w:.4f}")
        rospy.loginfo("-" * 50) # 打印分隔线，方便阅读
        
        # 按照设定的频率休眠
        rate.sleep()

    # 节点退出时的清理工作（MoveGroupCommander会自动处理）
    rospy.loginfo("节点已退出。")

