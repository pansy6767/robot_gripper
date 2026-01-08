# #!/usr/bin/env python3 
# import rospy, sys
# import moveit_commander
 
# class MoveItFkDemo:
#     def __init__(self):
 
       
#         moveit_commander.roscpp_initialize(sys.argv)
 
        
#         rospy.init_node('moveit_fk_demo', anonymous=True)       
 
        
#         arm = moveit_commander.MoveGroupCommander('arm')
        
        
#         arm.set_goal_joint_tolerance(0.01)
 
        
#         arm.set_max_acceleration_scaling_factor(0.5)
#         arm.set_max_velocity_scaling_factor(0.5)
        
        
#         arm.set_named_target('home')
#         arm.go()  
#         rospy.sleep(0.1)
         
        
#         # joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go()   
#         # rospy.sleep(0.1)
 

#         # joint_positions = [1.189, -0.921, 0.518, 3.0, 0.0, 0]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go() 
#         # rospy.sleep(0.1)


#         # joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go() 
#         # rospy.sleep(0.1)

#         # joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 1.1]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go() 
#         # rospy.sleep(0.1)

#         # joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0.0]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go() 
#         # rospy.sleep(0.1)

#         # joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 1.1]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go() 
#         # rospy.sleep(0.1)


#         # arm.set_named_target('home')
#         # arm.go()

#         # joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0]
#         # arm.set_joint_value_target(joint_positions)  
#         # arm.go()   
#         # rospy.sleep(0.1)
#         # -1.36,-1.26,0.87,0.09,-0.95,0.34
        
        
      
#         moveit_commander.roscpp_shutdown()
#         moveit_commander.os._exit(0)
 
# if __name__ == "__main__":
#     try:
#         MoveItFkDemo()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
import tf2_ros      # 2025/12/12 lk

class SmartSampler:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('smart_sampler')
        
        self.group = moveit_commander.MoveGroupCommander("arm")
        
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()        # 2025/12/12 lk
        self.tool_offset_y = 0.08                                   # 2025/12/12 lk      


        # 配置规划参数（你说MoveIt工作正常，这些应该够用）
        self.group.set_planning_time(10.0)
        self.group.set_num_planning_attempts(10)
        
        rospy.loginfo("Smart Sampler Ready!")
        
    def get_predefined_poses(self):
        """预定义一些好的标定姿态（关节角度）"""
        # 根据你的机械臂调整这些角度
        # poses = [
        #     [-1.36,-1.26,0.87,0.09,-0.95,0.34],  
        #     [-1.38,-1.271,0.876,0.0,-0.93,0.35],  
        #     [-1.422, -1.277, 0.885, -0.139, -0.91, 0.379],
        #     [-1.452, -1.247, 0.843, -0.137, -0.90, 0.349],       
        #     [-1.485, -1.237, 0.792, -0.138, -0.885, 0.319], 
        #     [-1.495, -1.307, 0.787, -0.038, -0.865, 0.305], 
        #     [-1.505, -1.372, 0.781, 0.059, -0.846, 0.293],
        #     [-1.510, -1.405, 0.785, 0.180, -0.835, 0.295],    
        #     [-1.515, -1.443, 0.788, 0.330, -0.829, 0.297], 
        #     [-1.512, -1.395, 0.739, 0.396, -0.801, 0.320], 
        #     [-1.523, -1.374, 0.729, 0.373, -0.776, 0.332],
        #     [-1.533, -1.344, 0.713, 0.353, -0.741, 0.335],
        #     [-1.547, -1.329, 0.700, 0.345, -0.727, 0.330], 
        #     [-1.542, -1.340, 0.732, 0.346, -0.739, 0.312], 
        #     [-1.537, -1.352, 0.764, 0.448, -0.750, 0.296], 
            
        # ]
        poses = [
            [-1.36,-1.26,0.87,0.09,-0.95,0.34],  
            [-1.38,-1.466,0.812,0.345,-0.933,0.35],  
            [-1.432, -1.299, 0.825, 0.041, -0.912, 0.382],
            [-1.452, -1.325, 0.843, -0.137, -0.90, 0.349],       
            [-1.485, -1.237, 0.792, 0.110, -0.827, 0.319], 
            [-1.476, -1.322, 0.831, -0.161, -0.872, 0.323], 
            [-1.505, -1.372, 0.781, 0.059, -0.846, 0.293],
            [-1.555, -1.395, 0.796, -0.109, -0.832, 0.311],    
            [-1.515, -1.443, 0.788, 0.330, -0.905, 0.297], 
            [-1.329, -1.395, 0.797, 0.412, -0.916, 0.310], 
            [-1.523, -1.374, 0.729, 0.373, -0.856, 0.332],
            [-1.287, -1.388, 0.736, 0.531, -1.036, 0.307],
            [-1.547, -1.329, 0.700, 0.345, -0.727, 0.330], 
            [-1.235, -1.566, 0.733, 0.501, -0.855, 0.450], 
            [-1.437, -1.352, 0.764, 0.448, -0.901, 0.296], 
            
        ]
        return poses
    
    def move_to_joint_values(self, joint_values):
        """移动到指定关节角度"""
        self.group.set_joint_value_target(joint_values)
        success = self.group.go(wait=True)
        self.group.stop()
        return success
    
    def wait_for_user_confirmation(self):
        """等待用户确认"""
        input("Press Enter after taking sample, or 's' to skip: ")
    
    def run_sampling_sequence(self):
        """执行采样序列"""
        poses = self.get_predefined_poses()
        
        rospy.loginfo(f"Will move through {len(poses)} predefined poses")
        rospy.loginfo("After each move, manually click 'Take Sample' in RQt")
        rospy.loginfo("="*50)
        
        for i, pose in enumerate(poses):
            rospy.loginfo(f"\n>>> Moving to pose {i+1}/{len(poses)}")
            rospy.loginfo(f"Target joints: {pose}")
            
            success = self.move_to_joint_values(pose)
            
            if success:
                rospy.loginfo("✓ Robot moved to position")
                rospy.loginfo("Now click 'Take Sample' in the RQt window")
                self.wait_for_user_confirmation()
            else:
                rospy.logwarn("✗ Failed to reach this pose, skipping...")
                rospy.sleep(1.0)
        
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("Sampling complete! Now click 'Compute' in RQt")
        rospy.loginfo("="*50)

if __name__ == '__main__':
    try:
        sampler = SmartSampler()
        sampler.run_sampling_sequence()
    except rospy.ROSInterruptException:
        pass