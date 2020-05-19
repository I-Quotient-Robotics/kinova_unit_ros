#!/usr/bin/env python

import sys
import time
import rospy
import math3d
import moveit_commander
import actionlib

import geometry_msgs.msg
from std_msgs.msg import Float64
from kinova_msgs.msg import SetFingersPositionAction
from kinova_msgs.msg import SetFingersPositionActionGoal

from kinova_unit_app.srv import GetObject


class PickPlace:
    def __init__(self):
        self.__arm_group = moveit_commander.MoveGroupCommander('arm')
        self.__gripper_command_pub = rospy.Publisher('/j2n6s200_driver/fingers_action/finger_positions/goal', SetFingersPositionActionGoal, queue_size=10)
        self._grasp_client = actionlib.SimpleActionClient("j2n6s200_driver/finger_action/finger_positions", SetFingersPositionAction)
        self.object_pose_sub = rospy.Subscriber("/object_pose", geometry_msgs.msg.PoseArray, self._callback)
        self.refresh = False
        self.pose_objects = geometry_msgs.msg.PoseArray()
        rospy.loginfo("arm setup")
        self.__set_gripper(False)
        rospy.sleep(1.0)
        self.__set_gripper(True)
        rospy.sleep(1.0)
        self.__move_by_name('home')
        rospy.sleep(1.0)

        self.__object_service_proxy = rospy.ServiceProxy('/object_detect_node/request_first_object', GetObject)

    def _callback(self, msg):
        self.pose_objects = msg
        self.refresh = True

    def __convert_to_transform(self, pose_stamped):
        quaternion = math3d.UnitQuaternion(pose_stamped.pose.orientation.w, pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z)
        transform = math3d.Transform()
        transform.set_pos((pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z))
        transform.set_orient(quaternion.orientation)
        return transform

    def __convert_to_pose(self, t):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = t.pos.x
        pose.pose.position.y = t.pos.y
        pose.pose.position.z = t.pos.z
        pose.pose.orientation.w = t.orient.quaternion[0]
        pose.pose.orientation.x = t.orient.quaternion[1]
        pose.pose.orientation.y = t.orient.quaternion[2]
        pose.pose.orientation.z = t.orient.quaternion[3]
        return pose

    def __set_gripper(self, state):
        finger_goal = SetFingersPositionActionGoal()
        if state:
            finger_goal.goal.fingers.finger1 = 0.0
            finger_goal.goal.fingers.finger2 = 0.0
            self.__gripper_command_pub.publish(finger_goal)
        else:
            finger_goal.goal.fingers.finger1 = 4500.0
            finger_goal.goal.fingers.finger2 = 4500.0
            self.__gripper_command_pub.publish(finger_goal)

    def __move_by_pose(self, target_pose):
        self.__arm_group.set_pose_target(target_pose)
        return self.__arm_group.go(wait=True)

    def __move_by_name(self, target_name):
        self.__arm_group.set_named_target(target_name)
        return self.__arm_group.go(wait=True)

    def pick_place(self, target_pose):
        pass

    def run(self):
        rospy.loginfo("request object pose...")
        pose_objects_detect = geometry_msgs.msg.PoseArray()

        if len(self.pose_objects.poses) != 0:
            pose_objects_detect = self.pose_objects
            rospy.loginfo("%d", len(self.pose_objects.poses))
        else:
            return

        pose_objects_detect.header.frame_id = "depth_camera_link"
        rospy.loginfo("found object")
        
        pose_place = geometry_msgs.msg.PoseStamped()
        pose_place.header.frame_id = "j2n6s200_link_base"
        pose_place.pose.position.x = -0.404268950224
        pose_place.pose.position.y = -0.457900762558
        pose_place.pose.position.z = 0.203838279247
        pose_place.pose.orientation.w = 0.0326578058302
        pose_place.pose.orientation.x =  0.98425257206
        pose_place.pose.orientation.y = -0.173656985164
        pose_place.pose.orientation.z = -0.00484579382464

        pose_place_stanby = geometry_msgs.msg.PoseStamped()
        pose_place_stanby.header.frame_id = "j2n6s200_link_base"
        pose_place_stanby.pose.position.x = -0.404268950224
        pose_place_stanby.pose.position.y = -0.457900762558
        pose_place_stanby.pose.position.z = 0.283838279247
        pose_place_stanby.pose.orientation.w = 0.0326578058302
        pose_place_stanby.pose.orientation.x =  0.98425257206
        pose_place_stanby.pose.orientation.y = -0.173656985164
        pose_place_stanby.pose.orientation.z = -0.00484579382464


        # rospy.loginfo("object pose: %s, %s, %s", res.object_pose.pose.position.x, res.object_pose.pose.position.y, res.object_pose.pose.position.z)
        # rospy.loginfo("standby pose: %s, %s, %s", standby_pose_stamped.pose.position.x, standby_pose_stamped.pose.position.y, standby_pose_stamped.pose.position.z)
        # rospy.loginfo("%s, %s, %s", pick_pose_stamped.pose.position.x, pick_pose_stamped.pose.position.y, pick_pose_stamped.pose.position.z)

        if self.__move_by_name('home') is not True:
            rospy.loginfo("unable to get home pose")
            return

        for x in range(len(pose_objects_detect.poses)):

            pose_pick_stanby = geometry_msgs.msg.PoseStamped()
            pose_pick_stanby.header.frame_id = "depth_camera_link"
            pose_pick_stanby.pose.position.x = pose_objects_detect.poses[x].position.x
            pose_pick_stanby.pose.position.y = pose_objects_detect.poses[x].position.y
            pose_pick_stanby.pose.position.z = pose_objects_detect.poses[x].position.z - 0.08
            pose_pick_stanby.pose.orientation.w = pose_objects_detect.poses[x].orientation.w
            pose_pick_stanby.pose.orientation.x =  pose_objects_detect.poses[x].orientation.x
            pose_pick_stanby.pose.orientation.y = pose_objects_detect.poses[x].orientation.y
            pose_pick_stanby.pose.orientation.z = pose_objects_detect.poses[x].orientation.z

            pose_pick = geometry_msgs.msg.PoseStamped()
            pose_pick.header.frame_id = "depth_camera_link"
            pose_pick.pose.position.x = pose_objects_detect.poses[x].position.x
            pose_pick.pose.position.y = pose_objects_detect.poses[x].position.y
            pose_pick.pose.position.z = pose_objects_detect.poses[x].position.z + 0.01
            pose_pick.pose.orientation.w = pose_objects_detect.poses[x].orientation.w
            pose_pick.pose.orientation.x =  pose_objects_detect.poses[x].orientation.x
            pose_pick.pose.orientation.y = pose_objects_detect.poses[x].orientation.y
            pose_pick.pose.orientation.z = pose_objects_detect.poses[x].orientation.z

            self.__set_gripper(True)
            rospy.sleep(2.0)

            # rospy.loginfo("%d", x)
            # rospy.loginfo("%s", pose_objects_detect.header.frame_id)
            # rospy.loginfo("%f %f %f %f %f %f %f", pose_objects_detect.poses[x].position.x, pose_objects_detect.poses[x].position.y, pose_objects_detect.poses[x].position.z, pose_objects_detect.poses[x].orientation.w, pose_objects_detect.poses[x].orientation.x,pose_objects_detect.poses[x].orientation.y,pose_objects_detect.poses[x].orientation.z)
            
            if self.__move_by_pose(pose_pick_stanby) is not True:
                rospy.loginfo("unable to get standby pose")
                return
            # rospy.sleep(2.0)
            if self.__move_by_pose(pose_pick) is not True:
                rospy.loginfo("unable to get object pose")
                return
            # rospy.sleep(2.0)

            self.__set_gripper(False)
            rospy.sleep(2.0)

            if self.__move_by_pose(pose_pick_stanby) is not True:
                rospy.loginfo("unable to get standby pose")
                return

            # rospy.sleep(2.0)

            if self.__move_by_pose(pose_place_stanby) is not True:
                rospy.loginfo("unable to get place standby pose")
                return

            if self.__move_by_pose(pose_place) is not True:
                rospy.loginfo("unable to get place pose")
                return

            self.__set_gripper(True)
            rospy.sleep(2.0)

            if self.__move_by_pose(pose_place_stanby) is not True:
                rospy.loginfo("unable to get place standby pose")
                return

            rospy.loginfo("finish")

        if self.__move_by_name('home') is not True:
            rospy.loginfo("unable to get home pose")
            return
        self.refresh = False
        while not self.refresh or rospy.is_shutdown():
            time.sleep(0.1)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinova_unit_app_node')

    pick_place = PickPlace()

    rate = rospy.Rate(0.4)
    while not rospy.is_shutdown():
        pick_place.run()
        rate.sleep()


if __name__ == '__main__':
    main()
