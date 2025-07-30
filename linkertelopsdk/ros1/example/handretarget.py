#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from threading import Thread, Event
from pathlib import Path
from queue import Empty
from typing import Optional
import enum
import signal, sys


from linkerhand.utils import *
from linkerhand.vtrdyncore import *
from linkerhand.handcore import HandCore
from linkerhand.config import HandConfig
from linkerhand.constants import RetargetingType, DataSource, MotionSource, RobotName

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray

vr_pose_cache_r = []
vr_pose_cache_l = []
video_pose_cache_r = []
video_pose_cache_l = []
reangle_r = []
reangle_l = []
right_hand_pose_end = []
left_hand_pose_end = []


def signal_handler(sig, frame):
    rospy.signal_shutdown("Signal Received")
    sys.exit(0)


class HandRetargetNode:
    def __init__(self):
        rospy.loginfo("Ready Create HandRetargetNode!")
        rospy.init_node('handretarget_node', anonymous=True)
        self.robot_dir = Path(__file__).absolute().parent / "assets" / "robots" / "hands"
        self.base_config = Path(__file__).absolute().parent

        self.handconfig = HandConfig(str(self.robot_dir), str(self.base_config))
        self.handcore = HandCore(self.handconfig)

        self.baseconfig = self.handconfig.baseconfig
        self.retagetconfig = self.handconfig.retagetconfig

        #
        self.scene, self.retargeting_r, self.retargeting_l, self.config_r, self.config_l = None, None, None, None, None
        self.robot_name_r, self.robot_name_l = None, None
        self.retargeting_type = None
        self.datasource_type = None
        self.motion_type = None
        self.udp_ip, self.udp_port, self.use_can, self.motion_device = None, None, None, None

        self.calibration = None
        self.calibrationopen_r, self.calibrationopen_l, self.calibrationclose_r, self.calibrationclose_l = None, None, None, None
        self.retarget = None
        self.datasource_type = DataSource[self.baseconfig["system"]["datasource_type"]]
        self.retargeting_type = RetargetingType[self.baseconfig["system"]["retargeting_type"]]
        self.motion_type = MotionSource[self.baseconfig["system"]["motion_type"]]
        self.robot_name_r = RobotName[self.baseconfig["system"]["robotname_r"]]
        self.robot_name_l = RobotName[self.baseconfig["system"]["robotname_l"]]

        self.udp_ip = self.baseconfig["udp"]["ip"]
        self.udp_port = int(self.baseconfig["udp"]["port"])
        self.use_can = bool(self.baseconfig["system"]["usecan"])
        self.motion_device = self.baseconfig["system"]["motion_device"]

        self.righthandprint = bool(self.baseconfig["debug"]["joint_motor_debug_r"])
        self.lefthandprint = bool(self.baseconfig["debug"]["joint_motor_debug_l"])

        # if self.datasource_type == DataSource.vr:
        #     rospy.Subscriber("/vr_right_hand_pose", JointState, vr_right_pose_callback)
        #     rospy.Subscriber("/vr_left_hand_pose", JointState, vr_left_pose_callback)
        # elif self.datasource_type == DataSource.video:
        #     rospy.Subscriber("/video_right_hand_pose", JointState, video_right_pose_callback)
        #     rospy.Subscriber("/video_left_hand_pose", JointState, video_left_pose_callback)
        
        self.pubprintcount = 0

    def retargetrun(self,calibrate):
        sender = None
        if self.motion_type == MotionSource.udexrealv2t:
            from motion.udexrealv2t.retarget import Retarget
            self.retarget = Retarget(ip=self.udp_ip,
                                     port=self.udp_port,
                                     deviceid=self.motion_device,
                                     righthand=self.robot_name_r,
                                     lefthand=self.robot_name_l,
                                     handcore=self.handcore,
                                     lefthandpubprint=self.lefthandprint,
                                     righthandpubprint=self.righthandprint,
                                     calibration = self.calibration)
        elif self.motion_type == MotionSource.udexreal:
            from motion.udexreal.retarget import Retarget
            self.retarget = Retarget(ip=self.udp_ip,
                                     port=self.udp_port,
                                     deviceid=self.motion_device,
                                     righthand=self.robot_name_r,
                                     lefthand=self.robot_name_l,
                                     handcore=self.handcore,
                                     lefthandpubprint=self.lefthandprint,
                                     righthandpubprint=self.righthandprint,
                                     calibration = self.calibration)
        elif self.motion_type == MotionSource.linkerforce:
            from motion.linkerforce.retarget import Retarget
            self.retarget = Retarget(port=self.udp_port,
                                     baudrate=self.motion_device,
                                     righthand=self.robot_name_r,
                                     lefthand=self.robot_name_l,
                                     handcore=self.handcore,
                                     lefthandpubprint=self.lefthandprint,
                                     righthandpubprint=self.righthandprint,
                                     calibration = self.calibration)
        elif self.motion_type == MotionSource.vtrdyn:
            from motion.vtrdyn.retarget import Retarget
            self.retarget = Retarget(ip=self.udp_ip,
                                     port=self.udp_port,
                                     righthand=self.robot_name_r,
                                     lefthand=self.robot_name_l,
                                     handcore=self.handcore,
                                     lefthandpubprint=self.lefthandprint,
                                     righthandpubprint=self.righthandprint,
                                     calibration = self.calibration)
        if self.retarget is None:
            rospy.logerr("未正确创建应用实例")
        else:
            rospy.loginfo("启动应用实例")
            self.retarget.process()


    def vr_right_pose_callback(self, msg):
        global vr_pose_cache_r
        for pose in msg, poses:
            vr_pose_cache_r = [pose.position.x, pose.position.y, pose.position.z]


    def vr_left_pose_callback(self, msg):
        global vr_pose_cache_l
        for pose in msg, poses:
            vr_pose_cache_l = [pose.position.x, pose.position.y, pose.position.z]


    def video_right_pose_callback(self, msg):
        global video_pose_cache_r
        for pose in msg, poses:
            video_pose_cache_r = [pose.position.x, pose.position.y, pose.position.z]


    def video_left_pose_callback(self, msg):
        global video_pose_cache_l
        for pose in msg, poses:
            video_pose_cache_l = [pose.position.x, pose.position.y, pose.position.z]


if __name__ == '__main__':
    stop_event = Event()
    try:
        signal.signal(signal.SIGINT, signal_handler)
        node = HandRetargetNode()
        calibrate = rospy.get_param('~calibrate', default=None)

        if calibrate is None:
            rospy.logwarn("Secondary calibration not set (parameter '~calibrate' is unconfigured).")
        else:
            rospy.loginfo(f"Secondary calibration mode: {calibrate}")
        node.retargetrun(calibrate)
    except rospy.ROSInterruptException:
        print("this node is exit !")
        pass
