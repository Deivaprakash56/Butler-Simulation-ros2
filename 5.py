#!/usr/bin/env python3
import rclpy
from inputimeout import inputimeout
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.2
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose




def main():
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()
    
    # --- Set initial pose ---
    initial_pose = create_pose_stamped(nav, 0.1 , -0.4 , -0.4)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active()

    # --- Create some Nav2 goal poses ---
    kitchen = create_pose_stamped(nav, 1.6 ,-3.9 , -0.7)
    table_1 = create_pose_stamped(nav, -2.2 , 0.7 ,-0.7)
    table_2 = create_pose_stamped(nav, -4.4 ,0.7 ,-0.4)
    table_3 = create_pose_stamped(nav,-2.5 , -0.9 , 0.9)
    robo_hp = create_pose_stamped(nav, 0.1 , -0.4 , -0.7)


    n = input("Place your order").split()
    nav.goToPose(kitchen)
    while not nav.isTaskComplete():
        print("Approaching kitchen")
    for i in range(len(n)):
        print(i)
        if n[i] == '1':
            nav.goToPose(table_1)
            while not nav.isTaskComplete():
                print("Going to table 1")
            
        elif n[i] == '2':
            nav.goToPose(table_2)
            while not nav.isTaskComplete():
                print("Going to table 2")
            
        elif n[i] == '3':
            nav.goToPose(table_3)
            while not nav.isTaskComplete():
                print("Going to table 3")
        else : 
            print("SORRY,please do check the order placed.")
            

    nav.goToPose(robo_hp)
    print(nav.getResult())

    # --- Shutdown ROS2 communications ---
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 