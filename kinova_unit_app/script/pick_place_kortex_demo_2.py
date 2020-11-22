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
        # action finish flag
        self.__action_finish = False

        # set speed limit
        self.__translation_speed = 0.1   # speed limit, m/s
        self.__orientation_speed = 30.0  # speed limit, degree/s

        rospy.loginfo('Activate arm action state notification')
        self.__activate_notifications()

        rospy.loginfo('Init the action topic subscriber')
        self.__action_topic_sub = rospy.Subscriber(rospy.get_namespace() + 'action_topic', ActionNotification, self.__cb_action_topic)
        self.__last_action_notif_type = None

        rospy.loginfo('Setup gripper service')
        send_gripper_command_full_name = rospy.get_namespace() + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.__send_gripper_command_srv = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        rospy.loginfo('Setup cartesian trajectory service')
        send_cartesian_command_full_name = rospy.get_namespace() + '/base/play_cartesian_trajectory'
        rospy.wait_for_service(send_cartesian_command_full_name)
        self.__send_cartesian_command_srv = rospy.ServiceProxy(send_cartesian_command_full_name, PlayCartesianTrajectory)

        rospy.loginfo('Setup joint angle trajectory service')
        send_joint_command_full_name = rospy.get_namespace() + '/base/play_joint_trajectory'
        rospy.wait_for_service(send_joint_command_full_name)
        self.__send_joint_command_srv = rospy.ServiceProxy(send_joint_command_full_name, PlayJointTrajectory)

        rospy.loginfo('Setup object detect service')
        self.__object_pose_array_pub = rospy.Publisher('object_pose_array', PoseArray, queue_size=10)
        self.__rail_segmentation_proxy = rospy.ServiceProxy('/rail_segmentation_node/segment_objects', SegmentObjects)

        rospy.loginfo('Pick place demo initialized.')

    def __cb_action_topic(self, notif):
        # rospy.loginfo(notif.action_event)

        if notif.action_event == ActionEvent.ACTION_END or self.__last_action_notif_type == ActionEvent.ACTION_ABORT:
            # rospy.loginfo('Received ACTION_END notification')
            self.__action_finish = True

    def __activate_notifications(self):
        activate_publishing_of_action_notification_full_name = rospy.get_namespace() + '/base/activate_publishing_of_action_topic'
        rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
        activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

        req = OnNotificationActionTopicRequest()
        rospy.loginfo('Activating the action notifications...')
        try:
            activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr('Failed to call OnNotificationActionTopic')
            return False
        else:
            rospy.loginfo('Successfully activated the Action Notifications!')

        rospy.sleep(1.0)
        return True

    def __wait_for_action_end_or_abort(self):
        # Reset notification flag
        self.__action_finish = False

        while not rospy.is_shutdown() and not self.__action_finish:
            rospy.sleep(0.1)

    def __send_gripper_command(self, value):
        # Initialize the request
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        # Call the service 
        try:
            self.__send_gripper_command_srv(req)
            # self.__wait_for_action_end_or_abort()
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

    def __move_by_pose(self, target_pose):
        result = False

        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = target_pose.position.x
        req.input.target_pose.y = target_pose.position.y
        req.input.target_pose.z = target_pose.position.z

        rotation = PyKDL.Rotation.Quaternion(target_pose.orientation.x,
                                             target_pose.orientation.y,
                                             target_pose.orientation.z,
                                             target_pose.orientation.w)
        req.input.target_pose.theta_x = math.degrees(rotation.GetRPY()[0])
        req.input.target_pose.theta_y = math.degrees(rotation.GetRPY()[1])
        req.input.target_pose.theta_z = math.degrees(rotation.GetRPY()[2])

        pose_speed = CartesianSpeed()
        pose_speed.translation = self.__translation_speed
        pose_speed.orientation = self.__orientation_speed

        req.input.constraint.oneof_type.speed.append(pose_speed)

        # rospy.loginfo(req)

        try:
            self.__send_cartesian_command_srv(req)
            # rospy.sleep(0.01)
            self.__wait_for_action_end_or_abort()
            result = True
        except Exception as e:
            result = False
            rospy.logerr('Failed to call play_cartesian_trajectory: %s', e)

        return result

    def __move_by_joint_angle(self, joint_angles):
        result = False

        try:
            req = PlayJointTrajectoryRequest()
            for i in range(7):
                if joint_angles[i] < 0.0:
                    j = math.degrees(math.pi*2.0+joint_angles[i])
                else:
                    j = math.degrees(joint_angles[i])

                temp_angle = JointAngle()
                temp_angle.joint_identifier = i
                temp_angle.value = j
                req.input.joint_angles.joint_angles.append(temp_angle)

            # rospy.loginfo(req)

            self.__send_joint_command_srv(req)
            # rospy.sleep(0.01)
            self.__wait_for_action_end_or_abort()
            result = True
        except Exception as e:
            result = False
            rospy.logerr('Failed to call play_joint_trajectory: %s', e)

        return result

    def run(self):
        try:
            rospy.loginfo('00 Get objects')
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

            rospy.loginfo('01 Open gripper')
            self.__set_gripper(True)
            rospy.sleep(0.2)

            rospy.loginfo('02 Pre touch')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.05, 0, 0))
            self.__move_by_pose(posemath.toMsg(p*f))
            rospy.sleep(0.2)

            rospy.loginfo('03 Touch')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.01, 0, 0))
            self.__move_by_pose(posemath.toMsg(p*f))
            rospy.sleep(0.2)

            rospy.loginfo('04 Close gripper')
            self.__set_gripper(False)
            rospy.sleep(0.5)

            rospy.loginfo('05 Leave up')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.03, 0, 0))
            self.__move_by_pose(posemath.toMsg(p*f))
            rospy.sleep(0.2)

            rospy.loginfo('06 Standby')
            self.__move_by_joint_angle(standby_angles)
            rospy.sleep(0.2)

            # put to other side
            place_pose = seg_response.segmented_objects.objects[0].bounding_volume.pose.pose
            place_pose.position.y *= -1.0
            place_p = posemath.fromMsg(place_pose)

            rospy.loginfo('07 Place')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.01, 0, 0))
            self.__move_by_pose(posemath.toMsg(place_p*f))
            # self.__move_by_joint_angle(place_angles)
            rospy.sleep(0.2)

            rospy.loginfo('08 Open gripper')
            self.__set_gripper(True)
            rospy.sleep(0.2)

            rospy.loginfo('09 Leave up')
            f = PyKDL.Frame(PyKDL.Rotation.RPY(-math.pi/2.0, 0, math.pi/2.0), PyKDL.Vector(0.05, 0, 0))
            self.__move_by_pose(posemath.toMsg(place_p*f))
            rospy.sleep(0.2)            

            rospy.loginfo('10 Standby')
            self.__move_by_joint_angle(standby_angles)
            rospy.sleep(1.0)

        except rospy.ServiceException as exc:
            rospy.logwarn('rail segmentation service did not process request: ' + str(exc))

    def initialize(self):
        rospy.loginfo('standby')
        self.__move_by_joint_angle(standby_angles)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_place_kortex_demo_2_node')

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
