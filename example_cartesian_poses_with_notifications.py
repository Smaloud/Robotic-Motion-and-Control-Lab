#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import copy
from kortex_driver.srv import *
from kortex_driver.msg import *

class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    # 新增：封装发送夹爪命令
    def example_send_gripper_command(self, value):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION
        rospy.loginfo("Sending gripper command with value %.2f", value)
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        rospy.sleep(2.0)
        return True

    # 新增：封装发送姿态（使用 ConstrainedPose）的函数
    def send_pose(self, constrained_pose, identifier, pose_name):
        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose = [constrained_pose]
        req.input.name = pose_name
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = identifier
        rospy.loginfo("Sending pose '%s'...", pose_name)
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose '%s'", pose_name)
            return False
        rospy.loginfo("Waiting for pose '%s' to finish...", pose_name)
        self.wait_for_action_end_or_abort()
        return True

    # 新增：实现抓取放下的功能
    def example_pick_and_place(self):
        rospy.loginfo("Starting pick and place operation...")

        # 定义一个笛卡尔运动速度（抓取放置过程中采用）
        cart_speed = CartesianSpeed()
        cart_speed.translation = 0.2 # m/s
        cart_speed.orientation = 30   # deg/s

        # 定义抓取位姿（假设抓取物体为正方体)
        pick_pose = ConstrainedPose()
        pick_pose.constraint.oneof_type.speed.append(cart_speed)
        pick_pose.target_pose.x = 0.4    # 根据实际抓取位置调整
        pick_pose.target_pose.y = 0.0
        pick_pose.target_pose.z = 0.015   # 物体高度（可调整）
        # 设定夹爪朝下的姿态
        pick_pose.target_pose.theta_x = 90
        pick_pose.target_pose.theta_y = 0
        pick_pose.target_pose.theta_z = 90

        # 定义抓取上方位姿（在抓取位姿上方增加一个偏移，比如 0.1m）
        above_pick_pose = copy.deepcopy(pick_pose)
        above_pick_pose.target_pose.z += 0.1

        # 定义放置位姿（目标位置）
        place_pose = ConstrainedPose()
        place_pose.constraint.oneof_type.speed.append(cart_speed)
        place_pose.target_pose.x = 0.6   # 根据实际放置位置调整
        place_pose.target_pose.y = 0.2
        place_pose.target_pose.z = 0.015   # 放置高度
        # 同样设置夹爪垂直于物体的角度
        place_pose.target_pose.theta_x = 90
        place_pose.target_pose.theta_y = 0
        place_pose.target_pose.theta_z = 90

        # 定义放置上方位姿
        above_place_pose = copy.deepcopy(place_pose)
        above_place_pose.target_pose.z += 0.1

        # 执行抓取放置动作步骤
        # 1. 移动到抓取上方
        if not self.send_pose(above_pick_pose, 1101, "Above Pick Pose"):
            return False

        # 2. 下降到抓取位姿
        if not self.send_pose(pick_pose, 1102, "Pick Pose"):
            return False

        # 3. 关闭夹爪，抓取物体（根据实际需要调整闭合值）
        if not self.example_send_gripper_command(0.7):
            return False

        # 4. 提升物体，返回到抓取上方
        if not self.send_pose(above_pick_pose, 1103, "Lift After Pick"):
            return False

        # 5. 移动到放置上方
        if not self.send_pose(above_place_pose, 1104, "Above Place Pose"):
            return False

        # 6. 下降到放置位姿
        if not self.send_pose(place_pose, 1105, "Place Pose"):
            return False

        # 7. 打开夹爪，放下物体
        if not self.example_send_gripper_command(0):
            return False

        # 8. 再次提升，离开放置位置
        if not self.send_pose(above_place_pose, 1106, "Lift After Place"):
            return False

        rospy.loginfo("Pick and place operation completed successfully!")
        return True

    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            # 清除故障并回到 Home 位置
            success &= self.example_clear_faults()
            # success &= self.example_home_the_robot()

            # 设置参考坐标系和订阅动作通知
            success &= self.example_set_cartesian_reference_frame()
            success &= self.example_subscribe_to_a_robot_notification()

            # 执行抓取放置操作
            success &= self.example_pick_and_place()

            # 最后可选：回到 Home 位置
            success &= self.example_home_the_robot()

        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleCartesianActionsWithNotifications()
    ex.main()
