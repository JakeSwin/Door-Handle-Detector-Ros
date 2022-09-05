#!/usr/bin/python
import rospy
import copy
import sys
import tf
import moveit_commander

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import PoseStamped, Pose, Twist
from moveit_msgs.msg import DisplayTrajectory

END_EFFECTOR_LINK = "gripper_grasping_frame"

class tiago_door_opener:
    def __init__(self, end_effector_link="gripper_grasping_frame"):
        self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.move_group.set_end_effector_link(end_effector_link)
        self.move_group.set_goal_tolerance(0.05)

        self.display_trajectory_publisher = rospy.Publisher("/display_planned_path", DisplayTrajectory, queue_size=20)
        self.move_base_publisher = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        self.trans = [0, 0, 0]
        self.rot = [0, 0, 0, 0]
        
    def display_startup_info(self, print_robot_state=False):
        planning_frame = self.move_group.get_planning_frame()
        print("Planning Frame: %s" % planning_frame)
        
        eef_link = self.move_group.get_end_effector_link()
        print("End effector link: %s" % eef_link)

        group_names = self.robot.get_group_names()
        print("Available Planning Groups: %s" % group_names)
        
        if (print_robot_state):
            print("=== Printing Robot State ===")
            print(self.robot.get_current_state())
            print("=== Stopped Printing Robot State ===")

    def update_handle_frame(self):
        while not rospy.is_shutdown():
            try:
                (self.trans, self.rot) = self.listener.lookupTransform('/base_footprint', '/handle', rospy.Time(0))
                print("Found Handle")
                print("Handle translation: %s" % self.trans)
                print("Handle rotation: %s" % self.rot)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(5)
                continue
            
    def get_handle_pose(self):
        pose = Pose()

        # define position 1 - hand moves to infront of handle
        pose.position.x = self.trans[0]
        pose.position.y = self.trans[1]
        pose.position.z = self.trans[2]

        pose.orientation.x = self.rot[0]
        pose.orientation.y = self.rot[1]
        pose.orientation.z = self.rot[2]
        pose.orientation.w = self.rot[3]

        return pose
    
    def ready_handle_position(self, pose=self.get_handle_pose()):

        # define position 1 - hand moves to infront of handle
        pose.position.x = self.trans[0] - 0.3 
        
        return pose
    
    def above_handle_position(self, pose=self.get_handle_pose()):

        (rot_x, rot_y, rot_z) = euler_from_quaternion(self.rot)

        pose.position.x = self.trans[0] - 0.1
        pose.position.z = self.trans[2] + 0.05

        rot_y += 0.5854
        new_rot = quaternion_from_euler(rot_x, rot_y, rot_z)
        
        pose.orientation.x = new_rot[0]
        pose.orientation.y = new_rot[1]
        pose.orientation.z = new_rot[2]
        pose.orientation.w = new_rot[3] 
        
        return pose
        
    def down_handle_position(self, pose=self.get_handle_pose()):

        pose.position.x = self.trans[0] - 0.06
        pose.position.z = self.trans[2] - 0.12
        
        return pose
    
    def grab_handle_waypoints(self):
        waypoints = []

        ready_handle_position = self.ready_handle_position()
        waypoints.append(copy.deepcopy(ready_handle_position))
        
        above_handle_position = self.above_handle_position()
        waypoints.append(copy.deepcopy(above_handle_position))
        
        down_handle_position = self.down_handle_positon(above_handle_position)
        waypoints.append(copy.deepcopy(down_handle_position))
        
        return waypoints
    
    def compute_waypoint_path(self, waypoints):
        print("planning")
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 2.0, 0.0, avoid_collisions=False)
        print("found_plan")
        print("fraction: %s" % fraction)
        return plan, fraction
        
    def publsh_trajectory(self, plan):
        # publish found trajectory
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
    
    def execute_plan(self, plan, fraction):
        if fraction == 1.0:
            self.move_group.execute(plan, wait=True)

            self.move_group.stop()

            self.move_group.clear_pose_targets()
        else:
            print("Fraction was not 1.0, not executing path")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("open_door_node", anonymous=True)
    
    robot_controller = tiago_door_opener()
    
    robot_controller.display_startup_info()
    
    robot_controller.update_handle_frame()
    
    waypoints = robot_controller.grab_handle_waypoints()
    
    (path, fraction) = robot_controller.compute_waypoint_path(waypoints)
    
    robot_controller.publish_trajectory(plan)
    
    robot_controller.execute_plan(plan, fraction)

# def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("open_door_node", anonymous=True)
    # listener = tf.TransformListener()
# 
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
 #  #  #  # 
    # group_name = "arm_torso"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
# 
    # planning_frame = move_group.get_planning_frame()
    # print("Planning Frame: %s" % planning_frame)
# 
    # print("Setting custom end effector")
    # move_group.set_end_effector_link(END_EFFECTOR_LINK)
# 
    # eef_link = move_group.get_end_effector_link()
    # print("End effector link: %s" % eef_link)
 #  #  #  # 
    # group_names = robot.get_group_names()
    # print("Available Planning Groups: %s" % group_names)
# 
    # print("=== Printing Robot State ===")
    # print(robot.get_current_state())
    # print("=== Stopped Printing Robot State ===")
# 
    # display_trajectory_publisher = rospy.Publisher("/display_planned_path", DisplayTrajectory, queue_size=20)
    # move_base_publisher = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
# 
    # while not rospy.is_shutdown():
        # try:
            # (trans, rot) = listener.lookupTransform('/base_footprint', '/handle', rospy.Time(0))
            # print(trans)
            # print(rot)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # continue
# 
        # waypoints = []
# 
        # pose_goal = Pose()
# 
        # # define position 1 - hand moves to infront of handle
        # pose_goal.position.x = trans[0] - 0.3 
        # pose_goal.position.y = trans[1] 
        # pose_goal.position.z = trans[2]  
# 
        # pose_goal.orientation.x = rot[0]
        # pose_goal.orientation.y = rot[1]
        # pose_goal.orientation.z = rot[2]
        # pose_goal.orientation.w = rot[3] 
# 
        # # save position 1
        # waypoints.append(copy.deepcopy(pose_goal))
# 
        # # define position 2 - hand rotades down and moves to above handle
        # (rot_x, rot_y, rot_z) = euler_from_quaternion(rot)
# 
        # pose_goal.position.x = trans[0] - 0.10
        # pose_goal.position.z = trans[2] + 0.05
# 
        # rot_y += 0.5854
        # new_rot = quaternion_from_euler(rot_x, rot_y, rot_z)
 #  #  #  #  #  #  #  # 
        # pose_goal.orientation.x = new_rot[0]
        # pose_goal.orientation.y = new_rot[1]
        # pose_goal.orientation.z = new_rot[2]
        # pose_goal.orientation.w = new_rot[3] 
# 
        # # save position 2
        # waypoints.append(copy.deepcopy(pose_goal))
# 
        # # define position 3 - hand pushes handle down
        # pose_goal.position.x = trans[0] - 0.06
        # pose_goal.position.z = trans[2] - 0.12
 #  #  #  #  #  #  #  # 
        # # save positon 3
        # waypoints.append(copy.deepcopy(pose_goal))
# 
        # print("planning")
        # (plan, fraction) = move_group.compute_cartesian_path(waypoints, 2.0, 0.0, avoid_collisions=False)
        # print("found_plan")
        # print("fraction: %s" % fraction)
# 
        # # publish found trajectory
        # display_trajectory = DisplayTrajectory()
        # display_trajectory.trajectory_start = robot.get_current_state()
        # display_trajectory.trajectory.append(plan)
        # display_trajectory_publisher.publish(display_trajectory)
# 
        # move_group.set_goal_tolerance(0.05)
# 
        # if fraction == 1.0:
            # move_group.execute(plan, wait=True)
# 
            # move_group.stop()
# 
            # count = 0
            # r = rospy.Rate(10)
            # t = Twist()
            # t.linear.x = 0.1
            # t.angular.z = -0.02
            # while count < 70:
                # move_base_publisher.publish(t)
                # r.sleep()
                # count += 1
# 
            # move_group.clear_pose_targets()
        # else:
            # print("Fraction was not 1.0, not executing path")
        # break
        # rospy.sleep(1)
    # print("Finished")

if __name__ == "__main__":
    main()
