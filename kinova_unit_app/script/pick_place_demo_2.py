#!/usr/bin/env python

import sys
import time
import math

import PyKDL

import rospy
import moveit_commander
from tf_conversions import posemath

from geometry_msgs.msg import Pose, PoseArray
from control_msgs.msg import GripperCommandActionGoal

from rail_manipulation_msgs.srv import SegmentObjects
from rail_manipulation_msgs.msg import SegmentedObject, SegmentedObjectList

from kortex_driver.srv import *
from kortex_driver.msg import *

standby_angles = [-0.014987737982748328, -0.027526965659276037, -3.0575376969486032, -1.6082968307365206, 0.012235710536044116, -1.515626281652815, 1.6584779762085564]
place_angles = [0.9832577165848853, 0.6053833874101963, -2.7782615808119178, -1.9881051474948253, -0.39862088669862406, -0.6114372852001644, 2.48761749524612]


class PickPlace:
    def __init__(self):
        self.__robot = moveit_commander.RobotCommander('robot_description')
        self.__scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())

        self.__arm_group = moveit_commander.MoveGroupCommander('arm', ns=rospy.get_namespace())
        self.__arm_group.set_max_velocity_scaling_factor(0.4)
        self.__arm_group.set_max_acceleration_scaling_factor(0.6)

        # self.__gripper_command_pub = rospy.Publisher(rospy.get_namespace()+'robotiq_2f_85_gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)

        send_gripper_command_full_name = rospy.get_namespace() + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.__send_gripper_command_srv = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        self.__object_pose_array_pub = rospy.Publisher('object_pose_array', PoseArray, queue_size=10)
        self.__rail_segmentation_proxy = rospy.ServiceProxy('/rail_segmentation_node/segment_objects', SegmentObjects)

    def __send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo('Sending the gripper command...')

        # Call the service 
        try:
            self.__send_gripper_command_srv(req)
        except rospy.ServiceException:
            rospy.logerr('Failed to call SendGripperCommand')
            return False
        else:
            time.sleep(0.5)
            return True

    def __set_gripper(self, state):
        if state:
            self.__send_gripper_command(0.0)
        else:
            self.__send_gripper_command(0.7)

        # self.__gripper_command_pub.publish(goal)

    def __move_by_pose(self, target_pose):
        self.__arm_group.set_pose_target(target_pose)

        result = False

        try:
            result = self.__arm_group.go(wait=True)
        except Exception as e:
            rospy.logerr('move by pose failed: %s', e)
            result = False

        return result

    def __move_by_name(self, target_name):
        self.__arm_group.set_named_target(target_name)
        return self.__arm_group.go(wait=True)

    def __move_by_joint_angle(self, target_angle):
        self.__arm_group.set_joint_value_target(target_angle)
        return self.__arm_group.go(wait=True)

    def run(self):
        try:
            rospy.loginfo('get objects')
            seg_response = self.__rail_segmentation_proxy()
            rospy.loginfo('get %d object', len(seg_response.segmented_objects.objects))

            if len(seg_response.segmented_objects.objects) is 0:
                rospy.loginfo('no object found')
                return

            # publish object axis
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = 'base_link'
            for obj in seg_response.segmented_objects.objects:
                pose_array.poses.append(obj.bounding_volume.pose.pose)
            self.__object_pose_array_pub.publish(pose_array)

            # use first object as target object
            p = posemath.fromMsg(seg_response.segmented_objects.objects[0].bounding_volume.pose.pose)

            rospy.loginfo('open gripper')
            self.__set_gripper(True)
            rospy.sleep(0.2)

            rospy.loginfo('pre touch')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.05, 0, 0))
            self.__move_by_pose(posemath.toMsg(p*f))
            rospy.sleep(0.2)

            rospy.loginfo('touch')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.01, 0, 0))
            self.__move_by_pose(posemath.toMsg(p*f))
            rospy.sleep(0.2)

            rospy.loginfo('close gripper')
            self.__set_gripper(False)
            rospy.sleep(0.2)

            rospy.loginfo('leave up')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.05, 0, 0))
            self.__move_by_pose(posemath.toMsg(p*f))
            rospy.sleep(0.2)

            rospy.loginfo('standby')
            self.__move_by_joint_angle(standby_angles)
            rospy.sleep(0.2)

            # put to other side
            place_pose = seg_response.segmented_objects.objects[0].bounding_volume.pose.pose
            place_pose.position.y *= -1.0
            place_p = posemath.fromMsg(place_pose)

            rospy.loginfo('place')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.01, 0, 0))
            self.__move_by_pose(posemath.toMsg(place_p*f))
            # self.__move_by_joint_angle(place_angles)
            rospy.sleep(0.2)

            rospy.loginfo('open gripper')
            self.__set_gripper(True)
            rospy.sleep(0.2)

            rospy.loginfo('leave up')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.05, 0, 0))
            self.__move_by_pose(posemath.toMsg(place_p*f))
            rospy.sleep(0.2)            

            rospy.loginfo('standby')
            self.__move_by_joint_angle(standby_angles)
            rospy.sleep(1.0)

        except rospy.ServiceException as exc:
            rospy.logwarn('rail segmentation service did not process request: ' + str(exc))

    def initialize(self):
        rospy.loginfo('standby')
        self.__move_by_joint_angle(standby_angles)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_place_demo_2_node')

    pick_place = PickPlace()
    pick_place.initialize()

    count = 0
    rate = rospy.Rate(0.4)
    while not rospy.is_shutdown():
        pick_place.run()
        rospy.loginfo('count: %d', count)
        count += 1
        rate.sleep()


if __name__ == '__main__':
    main()
