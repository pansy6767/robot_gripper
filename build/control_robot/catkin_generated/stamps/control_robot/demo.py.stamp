import rospy, sys
import moveit_commander
 
class MoveItFkDemo:
    def __init__(self):
 
       
        moveit_commander.roscpp_initialize(sys.argv)
 
        
        rospy.init_node('moveit_fk_demo', anonymous=True)       
 
        
        arm = moveit_commander.MoveGroupCommander('arm')
        
        
        arm.set_goal_joint_tolerance(0.01)
 
        
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        
        arm.set_named_target('home')
        arm.go()  
        rospy.sleep(0.1)
         
        
        joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0]
        arm.set_joint_value_target(joint_positions)  
        arm.go()   
        rospy.sleep(0.1)
 

        joint_positions = [1.189, -0.921, 0.518, 3.0, 0.0, 0]
        arm.set_joint_value_target(joint_positions)  
        arm.go() 
        rospy.sleep(0.1)


        joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0]
        arm.set_joint_value_target(joint_positions)  
        arm.go() 
        rospy.sleep(0.1)

        joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 1.1]
        arm.set_joint_value_target(joint_positions)  
        arm.go() 
        rospy.sleep(0.1)

        joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(joint_positions)  
        arm.go() 
        rospy.sleep(0.1)

        joint_positions = [1.189, -0.921, 0.518, 0.0, 0.0, 1.1]
        arm.set_joint_value_target(joint_positions)  
        arm.go() 
        rospy.sleep(0.1)


        arm.set_named_target('home')
        arm.go()
        
        
      
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
 
if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
