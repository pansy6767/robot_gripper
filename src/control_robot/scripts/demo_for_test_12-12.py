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


        self.group.set_planning_time(10.0)
        self.group.set_num_planning_attempts(10)
        
        rospy.loginfo("Smart Sampler Ready!")
    
    def move_to_joint_values(self, joint_values):
        """移动到指定关节角度"""
        self.group.set_joint_value_target(joint_values)
        success = self.group.go(wait=True)
        self.group.stop()
        return success
    
if __name__ == '__main__':
    try:
        sampler = SmartSampler()
        sampler.move_to_joint_values([0,-0.308,0.499,0,-0.204,0.157])
        input("Press Enter to exit...")
    except rospy.ROSInterruptException:
        pass