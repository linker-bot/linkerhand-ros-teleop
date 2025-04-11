#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from threading import Thread, Event
from pathlib import Path
from queue import Empty
from typing import Optional
import numpy as np
import enum
import signal, sys

from fastapi import FastAPI
import uvicorn
from pydantic import BaseModel

from linkerhand.utils import *
from linkerhand.vtrdyncore import *
from linkerhand.handcore import HandCore
from linkerhand.config import HandConfig
from linkerhand.constants import RetargetingType, DataSource, MotionSource
from linkerhand.udexrealcore import UdexRealScoketUdp, UdexRealSData

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

app = FastAPI()


class Item(BaseModel):
    size: int


@app.post("/")
async def get_data(item: Item):
    global reangle_r, reangle_l
    return {
        "angle_right": [val for val in reangle_r],
        "angle_left": [val for val in reangle_l],
        "position_right":
            {
                "x": [x for x, y, z in right_hand_pose_end],
                "y": [y for x, y, z in right_hand_pose_end],
                "z": [z for x, y, z in right_hand_pose_end],
            },
        "position_left":
            {
                "x": [x for x, y, z in left_hand_pose_end],
                "y": [y for x, y, z in left_hand_pose_end],
                "z": [z for x, y, z in left_hand_pose_end],
            }
    }


def start_fastapi(stop_event):
    uvicorn.run(app, host="192.168.11.101", port=6679)


def signal_handler(sig, frame):
    rospy.signal_shutdown("Signal Received")
    sys.exit(0)


class HandRetargetNode:
    def __init__(self):
        rospy.loginfo("Ready Create HandRetargetNode!")
        rospy.init_node('handretarget_node', anonymous=True)
        self.robot_dir = Path(__file__).absolute().parent / "assets" / "robots" / "hands"
        self.base_config = Path(__file__).absolute().parent
        print()
        self.handconfig = HandConfig(str(self.robot_dir), str(self.base_config))
        self.handcore = HandCore(self.handconfig)

        self.baseconfig = self.handconfig.baseconfig
        self.retagetconfig = self.handconfig.retagetconfig

        self.scene, self.retargeting_r, self.retargeting_l, self.config_r, self.config_l = None, None, None, None, None
        self.robot_name_str_r, self.robot_name_str_l = None, None
        self.retargeting_type = None
        self.datasource_type = None
        self.motion_type = None
        self.udp_ip, self.udp_port, self.use_can, self.motion_device = None, None, None, None
        datasouce_conf = self.baseconfig["system"]["datasource_type"]
        retargeting_conf = self.baseconfig["system"]["retargeting_type"]
        motion_conf = self.baseconfig["system"]["motion_type"]
        self.datasource_type = DataSource[datasouce_conf]
        self.retargeting_type = RetargetingType[retargeting_conf]
        self.motion_type = MotionSource[motion_conf]
        self.robot_name_str_r = self.baseconfig["system"]["robotname_r"]
        self.robot_name_str_l = self.baseconfig["system"]["robotname_l"]

        self.udp_ip = self.baseconfig["udp"]["ip"]
        self.udp_port = int(self.baseconfig["udp"]["port"])
        self.use_can = bool(self.baseconfig["system"]["usecan"])
        self.motion_device = self.baseconfig["system"]["motion_device"]
        if self.datasource_type == DataSource.vr:
            rospy.Subscriber("/vr_right_hand_pose", JointState, vr_right_pose_callback)
            rospy.Subscriber("/vr_left_hand_pose", JointState, vr_left_pose_callback)
        elif self.datasource_type == DataSource.video:
            rospy.Subscriber("/video_right_hand_pose", JointState, video_right_pose_callback)
            rospy.Subscriber("/video_left_hand_pose", JointState, video_left_pose_callback)
        self.publisher_r = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_r)
        self.publisher_l = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_l)
        self.rate = rospy.Rate(120)
        self.pubprintcount = 0

    def UdexRealRetargeting(self):
        rospy.loginfo("HandRetargetNode initialized and use udexreal device!")
        udp_datacapture = UdexRealScoketUdp(host=self.udp_ip, port=self.udp_port, device_id=self.motion_device)
        if udp_datacapture.udp_initial():
            g_jointpositions_r = [255] * int(self.handcore.hand_numjoints_r)
            g_jointpositions_l = [255] * int(self.handcore.hand_numjoints_l)
            g_jointvelocity_r = [255] * int(self.handcore.hand_numjoints_r)
            g_jointvelocity_l = [255] * int(self.handcore.hand_numjoints_l)
            last_jointpositions_r = [255] * int(self.handcore.hand_numjoints_r)
            last_jointpositions_l = [255] * int(self.handcore.hand_numjoints_l)
            g_jointpositions_r[6:4] = [128, 128, 128, 128]
            g_jointpositions_l[6:4] = [128, 128, 128, 128]
            last_scalepositions_r = [255] * int(self.handcore.hand_numjoints_r)
            last_scalepositions_l = [255] * int(self.handcore.hand_numjoints_l)
            last_jointvel_r = [255] * int(self.handcore.hand_numjoints_r)
            last_jointvel_l = [255] * int(self.handcore.hand_numjoints_l)
            state_l = [0] * int(self.handcore.hand_numjoints_l)
            state_r = [0] * int(self.handcore.hand_numjoints_r)
            while not rospy.is_shutdown():
                if not udp_datacapture.udp_is_onnect():
                    print("侦测到UDP断开状态，正在重连！")
                    udp_datacapture.udp_initial()
                    time.sleep(2)
                    continue
                mocapdata = udp_datacapture.realmocapdata
                if mocapdata.is_update:
                    qpos_r = np.zeros(25)
                    qpos_l = np.zeros(25)
                    if 't25' in str(self.robot_name_str_r):
                        qpos_r[15] = mocapdata.jointangle_rHand[3] * -1
                        qpos_r[16] = mocapdata.jointangle_rHand[20] * -2.013
                        qpos_r[17] = mocapdata.jointangle_rHand[2] * -1
                        qpos_r[18] = mocapdata.jointangle_rHand[1] * -1
                        qpos_r[19] = mocapdata.jointangle_rHand[0] * -1

                        qpos_r[0] = mocapdata.jointangle_rHand[7]
                        qpos_r[1] = mocapdata.jointangle_rHand[6] * -1
                        qpos_r[2] = mocapdata.jointangle_rHand[5] * -1
                        qpos_r[3] = mocapdata.jointangle_rHand[4] * -1

                        qpos_r[4] = mocapdata.jointangle_rHand[19]
                        qpos_r[5] = mocapdata.jointangle_rHand[18] * -1
                        qpos_r[6] = mocapdata.jointangle_rHand[17] * -1
                        qpos_r[7] = mocapdata.jointangle_rHand[16] * -1

                        qpos_r[8] = mocapdata.jointangle_rHand[10] * -1
                        qpos_r[9] = mocapdata.jointangle_rHand[9] * -1
                        qpos_r[10] = mocapdata.jointangle_rHand[8] * -1

                        qpos_r[11] = mocapdata.jointangle_rHand[15]
                        qpos_r[12] = mocapdata.jointangle_rHand[14] * -1
                        qpos_r[13] = mocapdata.jointangle_rHand[13] * -1
                        qpos_r[14] = mocapdata.jointangle_rHand[12] * -1
                    if 't25' in str(self.robot_name_str_l):
                        qpos_l[15] = mocapdata.jointangle_lHand[3]
                        qpos_l[16] = mocapdata.jointangle_lHand[20] * 2.013
                        qpos_l[17] = mocapdata.jointangle_lHand[2] * 0.5
                        qpos_l[18] = mocapdata.jointangle_lHand[1] * 0.7
                        qpos_l[19] = mocapdata.jointangle_lHand[0] * 0.9

                        qpos_l[0] = mocapdata.jointangle_lHand[7] * -1
                        qpos_l[1] = mocapdata.jointangle_lHand[6] * -1
                        qpos_l[2] = mocapdata.jointangle_lHand[5] * -1
                        qpos_l[3] = mocapdata.jointangle_lHand[4] * -1

                        qpos_l[4] = mocapdata.jointangle_lHand[19] * -1
                        qpos_l[5] = mocapdata.jointangle_lHand[18] * -1
                        qpos_l[6] = mocapdata.jointangle_lHand[17] * -1
                        qpos_l[7] = mocapdata.jointangle_lHand[16] * -1

                        qpos_l[8] = mocapdata.jointangle_lHand[10] * -1
                        qpos_l[9] = mocapdata.jointangle_lHand[9] * -1
                        qpos_l[10] = mocapdata.jointangle_lHand[8] * -1

                        qpos_l[11] = mocapdata.jointangle_lHand[15] * -1
                        qpos_l[12] = mocapdata.jointangle_lHand[14] * -1
                        qpos_l[13] = mocapdata.jointangle_lHand[13] * -1
                        qpos_l[14] = mocapdata.jointangle_lHand[12] * -1
                    if 'l25' in str(self.robot_name_str_r):
                        qpos_r[15] = mocapdata.jointangle_rHand[3] * -1
                        qpos_r[16] = mocapdata.jointangle_rHand[20] * -2.013
                        qpos_r[17] = mocapdata.jointangle_rHand[2] * -1
                        qpos_r[18] = mocapdata.jointangle_rHand[1] * -1
                        qpos_r[19] = mocapdata.jointangle_rHand[0] * -1

                        qpos_r[0] = mocapdata.jointangle_rHand[7]
                        qpos_r[1] = mocapdata.jointangle_rHand[6] * -1
                        qpos_r[2] = mocapdata.jointangle_rHand[5] * -1
                        qpos_r[3] = mocapdata.jointangle_rHand[4] * -1

                        qpos_r[4] = mocapdata.jointangle_rHand[19]
                        qpos_r[5] = mocapdata.jointangle_rHand[18] * -1
                        qpos_r[6] = mocapdata.jointangle_rHand[17] * -1
                        qpos_r[7] = mocapdata.jointangle_rHand[16] * -1

                        qpos_r[20] = mocapdata.jointangle_rHand[11]
                        qpos_r[8] = mocapdata.jointangle_rHand[10] * -1
                        qpos_r[9] = mocapdata.jointangle_rHand[9] * -1
                        qpos_r[10] = mocapdata.jointangle_rHand[8] * -1

                        qpos_r[11] = mocapdata.jointangle_rHand[15]
                        qpos_r[12] = mocapdata.jointangle_rHand[14] * -1
                        qpos_r[13] = mocapdata.jointangle_rHand[13] * -1
                        qpos_r[14] = mocapdata.jointangle_rHand[12] * -1
                    if 'l25' in str(self.robot_name_str_l):
                        qpos_l[15] = mocapdata.jointangle_lHand[3]
                        qpos_l[16] = mocapdata.jointangle_lHand[20] * 2.013
                        qpos_l[17] = mocapdata.jointangle_lHand[2] * 0.5
                        qpos_l[18] = mocapdata.jointangle_lHand[1] * 0.7
                        qpos_l[19] = mocapdata.jointangle_lHand[0] * 0.9

                        qpos_l[0] = mocapdata.jointangle_lHand[7] * -1
                        qpos_l[1] = mocapdata.jointangle_lHand[6] * -1
                        qpos_l[2] = mocapdata.jointangle_lHand[5] * -1
                        qpos_l[3] = mocapdata.jointangle_lHand[4] * -1

                        qpos_l[4] = mocapdata.jointangle_lHand[19] * -1
                        qpos_l[5] = mocapdata.jointangle_lHand[18] * -1
                        qpos_l[6] = mocapdata.jointangle_lHand[17] * -1
                        qpos_l[7] = mocapdata.jointangle_lHand[16] * -1

                        qpos_l[20] = mocapdata.jointangle_lHand[11] * -1
                        qpos_l[8] = mocapdata.jointangle_lHand[10] * -1
                        qpos_l[9] = mocapdata.jointangle_lHand[9] * -1
                        qpos_l[10] = mocapdata.jointangle_lHand[8] * -1

                        qpos_l[11] = mocapdata.jointangle_lHand[15] * -1
                        qpos_l[12] = mocapdata.jointangle_lHand[14] * -1
                        qpos_l[13] = mocapdata.jointangle_lHand[13] * -1
                        qpos_l[14] = mocapdata.jointangle_lHand[12] * -1
                    if 'l10' in str(self.robot_name_str_r) or 'o7' in str(self.robot_name_str_r):
                        qpos_r[15] = mocapdata.jointangle_rHand[3] * -1
                        qpos_r[16] = mocapdata.jointangle_rHand[20] * -2.2144
                        qpos_r[17] = mocapdata.jointangle_rHand[2] * -0.3878
                        qpos_r[18] = mocapdata.jointangle_rHand[1] * -0.66845
                        qpos_r[19] = mocapdata.jointangle_rHand[0] * -0.66845

                        qpos_r[0] = mocapdata.jointangle_rHand[7]
                        qpos_r[1] = mocapdata.jointangle_rHand[6] * -1.0098
                        qpos_r[2] = mocapdata.jointangle_rHand[5] * -1
                        qpos_r[3] = mocapdata.jointangle_rHand[4] * -1

                        qpos_r[4] = mocapdata.jointangle_rHand[19]
                        qpos_r[5] = mocapdata.jointangle_rHand[18] * -1.077160
                        qpos_r[6] = mocapdata.jointangle_rHand[17] * -1
                        qpos_r[7] = mocapdata.jointangle_rHand[16] * -1

                        qpos_r[8] = mocapdata.jointangle_rHand[10] * -1.0098
                        qpos_r[9] = mocapdata.jointangle_rHand[9] * -1
                        qpos_r[10] = mocapdata.jointangle_rHand[8] * -1

                        qpos_r[11] = mocapdata.jointangle_rHand[15]
                        qpos_r[12] = mocapdata.jointangle_rHand[14] * -1.0098
                        qpos_r[13] = mocapdata.jointangle_rHand[13] * -1
                        qpos_r[14] = mocapdata.jointangle_rHand[12] * -1
                    if 'l10' in str(self.robot_name_str_l) or 'o7' in str(self.robot_name_str_l):
                        qpos_l[15] = mocapdata.jointangle_lHand[3]
                        qpos_l[16] = mocapdata.jointangle_lHand[20] * 2.2144
                        qpos_l[17] = mocapdata.jointangle_lHand[2] * 0.3878
                        qpos_l[18] = mocapdata.jointangle_lHand[1] * 0.66845
                        qpos_l[19] = mocapdata.jointangle_lHand[0] * 0.66845

                        qpos_l[0] = mocapdata.jointangle_lHand[7] * -1
                        qpos_l[1] = mocapdata.jointangle_lHand[6] * -1.0098
                        qpos_l[2] = mocapdata.jointangle_lHand[5] * -1
                        qpos_l[3] = mocapdata.jointangle_lHand[4] * -10.87

                        qpos_l[4] = mocapdata.jointangle_lHand[19] * -1
                        qpos_l[5] = mocapdata.jointangle_lHand[18] * -1.077160
                        qpos_l[6] = mocapdata.jointangle_lHand[17] * -1
                        qpos_l[7] = mocapdata.jointangle_lHand[16] * -1

                        qpos_l[8] = mocapdata.jointangle_lHand[10] * -1.0098
                        qpos_l[9] = mocapdata.jointangle_lHand[9] * -1
                        qpos_l[10] = mocapdata.jointangle_lHand[8] * -1

                        qpos_l[11] = mocapdata.jointangle_lHand[15] * -1
                        qpos_l[12] = mocapdata.jointangle_lHand[14] * -1.0098
                        qpos_l[13] = mocapdata.jointangle_lHand[13] * -1
                        qpos_l[14] = mocapdata.jointangle_lHand[12] * -1
                    if 'l20' in str(self.robot_name_str_r):
                        qpos_r[16] = mocapdata.jointangle_rHand[3] * -1
                        qpos_r[17] = mocapdata.jointangle_rHand[20] * -2.570567
                        qpos_r[18] = mocapdata.jointangle_rHand[1] * -0.76688
                        qpos_r[19] = mocapdata.jointangle_rHand[0] * -1.231859

                        qpos_r[0] = mocapdata.jointangle_rHand[7]
                        qpos_r[1] = mocapdata.jointangle_rHand[6] * -1
                        qpos_r[2] = mocapdata.jointangle_rHand[5] * -1
                        qpos_r[3] = mocapdata.jointangle_rHand[4] * -1

                        qpos_r[4] = mocapdata.jointangle_rHand[19]
                        qpos_r[5] = mocapdata.jointangle_rHand[18] * -1.0
                        qpos_r[6] = mocapdata.jointangle_rHand[17] * -1
                        qpos_r[7] = mocapdata.jointangle_rHand[16] * -1

                        qpos_r[8] = mocapdata.jointangle_rHand[11]
                        qpos_r[9] = mocapdata.jointangle_rHand[10] * -1
                        qpos_r[10] = mocapdata.jointangle_rHand[9] * -1
                        qpos_r[11] = mocapdata.jointangle_rHand[8] * -1

                        qpos_r[12] = mocapdata.jointangle_rHand[15]
                        qpos_r[13] = mocapdata.jointangle_rHand[14] * -1
                        qpos_r[14] = mocapdata.jointangle_rHand[13] * -1
                        qpos_r[15] = mocapdata.jointangle_rHand[12] * -1
                    if 'l20' in str(self.robot_name_str_l):
                        qpos_l[16] = mocapdata.jointangle_lHand[3]
                        qpos_l[17] = mocapdata.jointangle_lHand[20] * 2.570567
                        qpos_l[18] = mocapdata.jointangle_lHand[2] * 0.76688
                        qpos_l[19] = mocapdata.jointangle_lHand[1] * 1.231859

                        qpos_l[0] = mocapdata.jointangle_lHand[7] * -1
                        qpos_l[1] = mocapdata.jointangle_lHand[6] * -1
                        qpos_l[2] = mocapdata.jointangle_lHand[5] * -1
                        qpos_l[3] = mocapdata.jointangle_lHand[4] * -1

                        qpos_l[4] = mocapdata.jointangle_lHand[19] * -1
                        qpos_l[5] = mocapdata.jointangle_lHand[18] * -1
                        qpos_l[6] = mocapdata.jointangle_lHand[17] * -1
                        qpos_l[7] = mocapdata.jointangle_lHand[16] * -1

                        qpos_l[8] = mocapdata.jointangle_lHand[11] * -1
                        qpos_l[9] = mocapdata.jointangle_lHand[10] * -1
                        qpos_l[10] = mocapdata.jointangle_lHand[9] * -1
                        qpos_l[11] = mocapdata.jointangle_lHand[8] * -1

                        qpos_l[12] = mocapdata.jointangle_lHand[15] * -1
                        qpos_l[13] = mocapdata.jointangle_lHand[14] * -1
                        qpos_l[14] = mocapdata.jointangle_lHand[13] * -1
                        qpos_l[15] = mocapdata.jointangle_lHand[12] * -1
                    g_jointpositions_r = self.handcore.trans_to_motor_right(qpos_r)
                    g_jointpositions_l = self.handcore.trans_to_motor_left(qpos_l)
                    # print(g_jointpositions_l, qpos_l[17], mocapdata.jointangle_lHand[20],
                    #       mocapdata.jointangle_lHand[20] * 180 / 3.14, )
                    for i in range(len(last_scalepositions_r)):
                        val = g_jointpositions_r[i] * 0.7 + last_scalepositions_r[i] * 0.3
                        last_scalepositions_r[i] = g_jointpositions_r[i]
                        g_jointpositions_r[i] = int(val)
                    for i in range(len(last_scalepositions_l)):
                        val = g_jointpositions_l[i] * 0.7 + last_scalepositions_l[i] * 0.3
                        last_scalepositions_l[i] = g_jointpositions_l[i]
                        g_jointpositions_l[i] = int(val)

                    if 'l10' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            position_derict = 1 if g_jointpositions_r[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_r[i] * 2)
                            mid_vel = int(last_jointvel_r[i] * 0.7)
                            min_vel = int(last_jointvel_r[i] * 0.5)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 5
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 5 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 3 + 10
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 20
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 3 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_r[i] = 1
                            if i == 0 or i == 1 or i == 9:
                                target_vel = 25
                            g_jointvelocity_r[i] = int(target_vel)

                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                    if 'o7' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            position_derict = 1 if g_jointpositions_r[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_r[i] * 2)
                            mid_vel = int(last_jointvel_r[i] * 0.7)
                            min_vel = int(last_jointvel_r[i] * 0.5)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 14 + 30
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 28 + 60
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 28 + 80
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 28 + 40
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_r[i] = 1
                            if position_derict == -1:
                                target_vel = 2 * target_vel
                            if i == 1 or i == 6:
                                target_vel = 25
                            g_jointvelocity_r[i] = int(target_vel)
                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                    if 't25' in str(self.robot_name_str_r) or 'l25' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            slow_limit = 1
                            fast_limit = 5
                            max_vel = int(last_jointvel_r[i] * 1.5)
                            min_vel = int(last_jointvel_r[i] * 0.995)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 50
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 100
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 3 + 100
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 5 + 200
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 200
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                else:
                                    target_vel = min_vel
                                    if min_vel < 150:
                                        state_r[i] = 1
                            g_jointvelocity_r[i] = int(target_vel)
                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                        g_jointvelocity_r[6] = 255
                        g_jointvelocity_r[7] = 255
                        g_jointvelocity_r[8] = 255
                        g_jointvelocity_r[9] = 255
                    if 't25' in str(self.robot_name_str_l) or 'l25' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            slow_limit = 1
                            fast_limit = 5
                            max_vel = int(last_jointvel_l[i] * 1.5)
                            min_vel = int(last_jointvel_l[i] * 0.995)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 50
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 100
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 3 + 100
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 5 + 200
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 200
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                else:
                                    target_vel = min_vel
                                    if min_vel < 150:
                                        state_l[i] = 1
                            g_jointvelocity_l[i] = int(target_vel)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                        g_jointvelocity_l[6] = 255
                        g_jointvelocity_l[7] = 255
                        g_jointvelocity_l[8] = 255
                        g_jointvelocity_l[9] = 255
                    if 'l10' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            position_derict = 1 if g_jointpositions_l[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_l[i] * 2)
                            mid_vel = int(last_jointvel_l[i] * 0.7)
                            min_vel = int(last_jointvel_l[i] * 0.5)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 5
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 3 + 10
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 20
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 3 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_l[i] = 1
                            if position_derict == -1:
                                target_vel = 2 * target_vel
                            if i == 0 or i == 1 or i == 9:
                                target_vel = 25
                            g_jointvelocity_l[i] = int(target_vel * 1)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                    if 'o7' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            position_derict = 1 if g_jointpositions_r[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_l[i] * 2)
                            mid_vel = int(last_jointvel_l[i] * 0.7)
                            min_vel = int(last_jointvel_l[i] * 0.5)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 14 + 30
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 28 + 60
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 28 + 80
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 28 + 40
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_l[i] = 1
                            if position_derict == -1:
                                target_vel = 2 * target_vel
                            if i == 1 or i == 6:
                                target_vel = 25
                            g_jointvelocity_l[i] = int(target_vel * 1)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                    if 'l20' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_l[i] * 2)
                            mid_vel = int(last_jointvel_l[i] * 0.8)
                            min_vel = int(last_jointvel_l[i] * 0.6)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 10 + 5
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 10 + 10
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 10 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 10 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_l[i] = 1
                            g_jointvelocity_l[i] = int(target_vel)

                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                        
                    if 'l20' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            slow_rimit = 2
                            fast_rimit = 10
                            max_vel = int(last_jointvel_r[i] * 2)
                            mid_vel = int(last_jointvel_r[i] * 0.8)
                            min_vel = int(last_jointvel_r[i] * 0.6)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 10 + 5
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_rimit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 10 + 10
                            else:  # fast
                                if position_error >= fast_rimit:
                                    target_vel = position_error * 10 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_rimit < position_error < fast_rimit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 3
                                elif 0 < position_error <= slow_rimit:
                                    target_vel = position_error * 10 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_r[i] = 1
                            g_jointvelocity_r[i] = int(target_vel)

                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                    if bool(self.baseconfig["debug"]["joint_motor_debug_r"]) and self.pubprintcount % 1 == 0:
                        print(g_jointpositions_r)
                    if bool(self.baseconfig["debug"]["joint_motor_debug_l"]) and self.pubprintcount % 1 == 0:
                        print(g_jointpositions_l)
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = [f'joint{i + 1}' for i in range(len(g_jointpositions_r))]
                    msg.position = [float(num) for num in g_jointpositions_r]
                    #print(msg.position)
                    msg.velocity = [float(num) for num in g_jointvelocity_r]
                    self.publisher_r.publish(msg)
                    msg = JointState()
                    msg.name = [f'joint{i + 1}' for i in range(len(g_jointpositions_l))]
                    msg.position = [float(num) for num in g_jointpositions_l]
                    msg.velocity = [float(num) for num in g_jointvelocity_l]
                    self.publisher_l.publish(msg)
                self.pubprintcount = self.pubprintcount + 1
                self.rate.sleep()
            print("Udp Is Ready To Closeing!")
            udp_datacapture.udp_close()
            print("Udp Is Closed!")

    def VtrdynRetargeting(self):
        rospy.loginfo("HandRetargetNode initialized and use vtrdyn device!")
        udp_datacapture = VtrdynSocketUdp()
        last_jointpositions_r = [255] * int(self.handcore.hand_numjoints_r)
        last_jointpositions_l = [255] * int(self.handcore.hand_numjoints_l)
        if udp_datacapture.udp_initial(2223):
            g_jointpositions_r = [255] * int(self.handcore.hand_numjoints_r)
            g_jointpositions_l = [255] * int(self.handcore.hand_numjoints_l)
            g_jointvelocity_r = [255] * int(self.handcore.hand_numjoints_r)
            g_jointvelocity_l = [255] * int(self.handcore.hand_numjoints_l)
            g_jointpositions_r[6:4] = [128, 128, 128, 128]
            g_jointpositions_l[6:4] = [128, 128, 128, 128]
            dstAddr = udp_datacapture.udp_getsockaddr(self.udp_ip, self.udp_port)
            udp_datacapture.udp_send_request_connect(dstAddr)
            last_scalepositions_r = [255] * int(self.handcore.hand_numjoints_r)
            last_scalepositions_l = [255] * int(self.handcore.hand_numjoints_l)
            last_jointvel_r = [255] * int(self.handcore.hand_numjoints_r)
            last_jointvel_l = [255] * int(self.handcore.hand_numjoints_l)
            state_l = [0] * int(self.handcore.hand_numjoints_l)
            state_r = [0] * int(self.handcore.hand_numjoints_r)
            while not rospy.is_shutdown():
                if not udp_datacapture.udp_is_onnect():
                    print("侦测到UDP断开状态，正在重连！")
                    if udp_datacapture.udp_initial(2223):
                        udp_datacapture.udp_send_request_connect(dstAddr)
                    time.sleep(2)
                mocapdata = MocapData()
                udp_datacapture.udp_recv_mocap_data(mocapdata)  # 接收数据
                if mocapdata.is_update:  # 数据更新
                    right_hand_pose, left_hand_pose = self.handcore.generate_position(
                        mocapdata.quaternion_rHand,
                        mocapdata.quaternion_lHand)
                    qpos_r = np.zeros(25)
                    qpos_l = np.zeros(25)
                    pro_qpos_r = self.handcore.projection_process(right_hand_pose)
                    pro_qpos_l = self.handcore.projection_process(left_hand_pose)
                    # print(pro_qpos_l)
                    pro_qpos_r[0] = pro_qpos_r[0] - 0.11
                    pro_qpos_r[1] = pro_qpos_r[1] - 0.09
                    pro_qpos_r[2] = pro_qpos_r[2] - 0.25
                    pro_qpos_r[3] = pro_qpos_r[3] - 0.12
                    pro_qpos_r[4] = pro_qpos_r[4] - 0.08
                    pro_qpos_l[0] = pro_qpos_l[0] - 0.15
                    pro_qpos_l[1] = pro_qpos_l[1] - 0.21
                    pro_qpos_l[2] = pro_qpos_l[2] - 0.15
                    pro_qpos_l[3] = pro_qpos_l[3] - 0.21
                    pro_qpos_l[4] = pro_qpos_l[4] - 0.11
                    if 't25' in str(self.robot_name_str_r):
                        qpos_r[15] = pro_qpos_r[0]
                        qpos_r[16] = pro_qpos_r[1] * -1
                        qpos_r[17] = pro_qpos_r[2]
                        qpos_r[18] = pro_qpos_r[3]
                        qpos_r[19] = pro_qpos_r[4]

                        qpos_r[0] = pro_qpos_r[5] * -1.2
                        qpos_r[1] = pro_qpos_r[7]
                        qpos_r[2] = pro_qpos_r[8]
                        qpos_r[3] = pro_qpos_r[9]

                        qpos_r[4] = pro_qpos_r[20] * 1.2
                        qpos_r[5] = pro_qpos_r[22]
                        qpos_r[6] = pro_qpos_r[23]
                        qpos_r[7] = pro_qpos_r[24]

                        qpos_r[8] = pro_qpos_r[12]
                        qpos_r[9] = pro_qpos_r[13]
                        qpos_r[10] = pro_qpos_r[14]

                        qpos_r[11] = pro_qpos_r[15] * 1.5
                        qpos_r[12] = pro_qpos_r[17]
                        qpos_r[13] = pro_qpos_r[18]
                        qpos_r[14] = pro_qpos_r[19]
                    if 't25' in str(self.robot_name_str_l):
                        qpos_l[15] = pro_qpos_l[0] * -0.6
                        qpos_l[16] = pro_qpos_l[1] * 0.6
                        qpos_l[17] = pro_qpos_l[2] * -0.5
                        qpos_l[18] = pro_qpos_l[3] * -1
                        qpos_l[19] = pro_qpos_l[4] * -1
                        # 食指
                        qpos_l[0] = pro_qpos_l[5] * 1.2
                        qpos_l[1] = pro_qpos_l[7]
                        qpos_l[2] = pro_qpos_l[8]
                        qpos_l[3] = pro_qpos_l[9]
                        # 小指
                        qpos_l[4] = pro_qpos_l[20] * -1.2
                        qpos_l[5] = pro_qpos_l[22]
                        qpos_l[6] = pro_qpos_l[23]
                        qpos_l[7] = pro_qpos_l[24]
                        # 中指
                        qpos_l[8] = pro_qpos_l[12]
                        qpos_l[9] = pro_qpos_l[13]
                        qpos_l[10] = pro_qpos_l[14]
                        # 无名指
                        qpos_l[11] = pro_qpos_l[15] * -1.5
                        qpos_l[12] = pro_qpos_l[17]
                        qpos_l[13] = pro_qpos_l[18]
                        qpos_l[14] = pro_qpos_l[19]
                    if 'l25' in str(self.robot_name_str_r):
                        qpos_r[15] = pro_qpos_r[0]
                        qpos_r[16] = pro_qpos_r[1] * -1
                        qpos_r[17] = pro_qpos_r[2]
                        qpos_r[18] = pro_qpos_r[3]
                        qpos_r[19] = pro_qpos_r[4]

                        qpos_r[0] = pro_qpos_r[5] * -1.2
                        qpos_r[1] = pro_qpos_r[7]
                        qpos_r[2] = pro_qpos_r[8]
                        qpos_r[3] = pro_qpos_r[9]

                        qpos_r[4] = pro_qpos_r[20] * 1.2
                        qpos_r[5] = pro_qpos_r[22]
                        qpos_r[6] = pro_qpos_r[23]
                        qpos_r[7] = pro_qpos_r[24]

                        qpos_r[20] = pro_qpos_r[10]
                        qpos_r[8] = pro_qpos_r[12]
                        qpos_r[9] = pro_qpos_r[13]
                        qpos_r[10] = pro_qpos_r[14]

                        qpos_r[11] = pro_qpos_r[15] * 1.5
                        qpos_r[12] = pro_qpos_r[17]
                        qpos_r[13] = pro_qpos_r[18]
                        qpos_r[14] = pro_qpos_r[19]
                    if '125' in str(self.robot_name_str_l):
                        qpos_l[15] = pro_qpos_l[0] * -0.6
                        qpos_l[16] = pro_qpos_l[1] * 0.6
                        qpos_l[17] = pro_qpos_l[2] * -0.5
                        qpos_l[18] = pro_qpos_l[3] * -1
                        qpos_l[19] = pro_qpos_l[4] * -1
                        # 食指
                        qpos_l[0] = pro_qpos_l[5] * 1.2
                        qpos_l[1] = pro_qpos_l[7]
                        qpos_l[2] = pro_qpos_l[8]
                        qpos_l[3] = pro_qpos_l[9]
                        # 小指
                        qpos_l[4] = pro_qpos_l[20] * -1.2
                        qpos_l[5] = pro_qpos_l[22]
                        qpos_l[6] = pro_qpos_l[23]
                        qpos_l[7] = pro_qpos_l[24]
                        # 中指
                        qpos_l[20] = pro_qpos_l[10]
                        qpos_l[8] = pro_qpos_l[12]
                        qpos_l[9] = pro_qpos_l[13]
                        qpos_l[10] = pro_qpos_l[14]
                        # 无名指
                        qpos_l[11] = pro_qpos_l[15] * -1.5
                        qpos_l[12] = pro_qpos_l[17]
                        qpos_l[13] = pro_qpos_l[18]
                        qpos_l[14] = pro_qpos_l[19]
                    if 'l10' in str(self.robot_name_str_r) or 'o7' in str(self.robot_name_str_r):
                        qpos_r[15] = pro_qpos_r[0] * 1.2
                        qpos_r[16] = pro_qpos_r[1] * -0.8
                        qpos_r[17] = pro_qpos_r[2]
                        qpos_r[18] = pro_qpos_r[3]
                        qpos_r[19] = pro_qpos_r[4]

                        qpos_r[0] = pro_qpos_r[5] * -1.2
                        qpos_r[1] = pro_qpos_r[7]
                        qpos_r[2] = pro_qpos_r[8]
                        qpos_r[3] = pro_qpos_r[9]

                        qpos_r[4] = pro_qpos_r[20] * 2
                        qpos_r[5] = pro_qpos_r[22]
                        qpos_r[6] = pro_qpos_r[23]
                        qpos_r[7] = pro_qpos_r[24]

                        qpos_r[8] = pro_qpos_r[12]
                        qpos_r[9] = pro_qpos_r[13]
                        qpos_r[10] = pro_qpos_r[14]

                        qpos_r[11] = pro_qpos_r[15] * 1.5
                        qpos_r[12] = pro_qpos_r[17]
                        qpos_r[13] = pro_qpos_r[18]
                        qpos_r[14] = pro_qpos_r[19]
                    if 'l10' in str(self.robot_name_str_l) or 'o7' in str(self.robot_name_str_l):
                        qpos_l[15] = pro_qpos_l[0] * -0.7
                        qpos_l[16] = pro_qpos_l[1] * 0.4
                        qpos_l[17] = (pro_qpos_l[3] + pro_qpos_l[4]) * -0.4
                        qpos_l[18] = qpos_l[17]
                        qpos_l[19] = qpos_l[17]

                        qpos_l[0] = pro_qpos_l[5] * 1.2
                        qpos_l[1] = pro_qpos_l[7]
                        qpos_l[2] = pro_qpos_l[8]
                        qpos_l[3] = pro_qpos_l[9]

                        qpos_l[4] = pro_qpos_l[20] * -2
                        qpos_l[5] = pro_qpos_l[22]
                        qpos_l[6] = pro_qpos_l[23]
                        qpos_l[7] = pro_qpos_l[24]

                        qpos_l[8] = pro_qpos_l[12]
                        qpos_l[9] = pro_qpos_l[13]
                        qpos_l[10] = pro_qpos_l[14]

                        qpos_l[11] = pro_qpos_l[15] * -1.4
                        qpos_l[12] = pro_qpos_l[17]
                        qpos_l[13] = pro_qpos_l[18]
                        qpos_l[14] = pro_qpos_l[19]
                    # print(qpos_l[0],qpos_l[11],qpos_l[4])
                    if 'l20' in str(self.robot_name_str_r):
                        qpos_r[16] = pro_qpos_r[0] * 0.4
                        qpos_r[17] = pro_qpos_r[1] * -0.8
                        qpos_r[18] = pro_qpos_r[2] * 0.3
                        qpos_r[19] = pro_qpos_r[3] * 1

                        qpos_r[0] = pro_qpos_r[5] * -1.2
                        qpos_r[1] = (pro_qpos_r[7] + pro_qpos_r[8] + pro_qpos_r[9]) * 0.25
                        qpos_r[1] = exponential_growth_fun(qpos_r[1], 3, 0, 1.38)
                        qpos_r[2] = qpos_r[1]
                        qpos_r[3] = qpos_r[1]

                        qpos_r[4] = pro_qpos_r[20] * 1.2
                        qpos_r[5] = (pro_qpos_r[22] + pro_qpos_r[23] + pro_qpos_r[24]) * 0.25
                        qpos_r[5] = exponential_growth_fun(qpos_r[5], 3, 0, 1.41)
                        qpos_r[6] = qpos_r[5]
                        qpos_r[7] = qpos_r[5]

                        qpos_r[8] = pro_qpos_r[10] * -1.2
                        qpos_r[9] = (pro_qpos_r[12] + pro_qpos_r[13] + pro_qpos_r[14]) * 0.25
                        qpos_r[9] = exponential_growth_fun(qpos_r[9], 3, 0, 1.41)
                        qpos_r[10] = qpos_r[9]
                        qpos_r[11] = qpos_r[9]

                        qpos_r[12] = pro_qpos_r[15] * 1.5
                        qpos_r[13] = (pro_qpos_r[17] + pro_qpos_r[18] + pro_qpos_r[19]) * 0.25
                        qpos_r[13] = exponential_growth_fun(qpos_r[13], 3, 0, 1.75)
                        qpos_r[14] = qpos_r[13]
                        qpos_r[15] = qpos_r[13]
                    if 'l20' in str(self.robot_name_str_l):
                        qpos_l[16] = pro_qpos_l[0] * -0.4
                        qpos_l[17] = pro_qpos_l[1] * 0.8
                        qpos_l[18] = pro_qpos_l[2] * -0.3
                        qpos_l[19] = pro_qpos_l[3] * -1

                        qpos_l[0] = pro_qpos_l[5] * 1.2
                        qpos_l[1] = (pro_qpos_l[7] + pro_qpos_l[8] + pro_qpos_l[9]) * 0.25
                        qpos_l[1] = exponential_growth_fun(qpos_l[1], 3, 0, 1.38)
                        qpos_l[2] = qpos_l[1]
                        qpos_l[3] = qpos_l[1]

                        qpos_l[4] = pro_qpos_l[20] * -1.2
                        qpos_l[5] = (pro_qpos_l[22] + pro_qpos_l[23] + pro_qpos_l[24]) * 0.25
                        qpos_l[5] = exponential_growth_fun(qpos_l[5], 3, 0, 1.41)
                        qpos_l[6] = qpos_l[5]
                        qpos_l[7] = qpos_l[5]

                        qpos_l[8] = pro_qpos_l[10] * 1.2
                        qpos_l[9] = (pro_qpos_l[12] + pro_qpos_l[13] + pro_qpos_l[14]) * 0.25
                        qpos_l[9] = exponential_growth_fun(qpos_l[9], 3, 0, 1.41)
                        qpos_l[10] = qpos_l[9]
                        qpos_l[11] = qpos_l[9]

                        qpos_l[12] = pro_qpos_l[15] * -1.5
                        qpos_l[13] = (pro_qpos_l[17] + pro_qpos_l[18] + pro_qpos_l[19]) * 0.25
                        qpos_l[13] = exponential_growth_fun(qpos_l[13], 3, 0, 1.75)
                        qpos_l[14] = qpos_l[13]
                        qpos_l[15] = qpos_l[13]
                    g_jointpositions_r = self.handcore.trans_to_motor_right(qpos_r)
                    g_jointpositions_l = self.handcore.trans_to_motor_left(qpos_l)
                    # print(g_jointpositions_r, qpos_r[0], qpos_r[11], qpos_r[4])
                    for i in range(len(last_scalepositions_r)):
                        val = g_jointpositions_r[i] * 0.7 + last_scalepositions_r[i] * 0.3
                        last_scalepositions_r[i] = g_jointpositions_r[i]
                        g_jointpositions_r[i] = int(val)
                    for i in range(len(last_scalepositions_l)):
                        val = g_jointpositions_l[i] * 0.7 + last_scalepositions_l[i] * 0.3
                        last_scalepositions_l[i] = g_jointpositions_l[i]
                        g_jointpositions_l[i] = int(val)

                    if 'l10' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            position_derict = 1 if g_jointpositions_r[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_r[i] * 2)
                            mid_vel = int(last_jointvel_r[i] * 0.7)
                            min_vel = int(last_jointvel_r[i] * 0.5)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 5
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 5 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 3 + 10
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 20
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 3 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_r[i] = 1
                            if i == 0 or i == 1 or i == 9:
                                target_vel = 25
                            g_jointvelocity_r[i] = int(target_vel)

                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                    if 'o7' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            position_derict = 1 if g_jointpositions_r[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_r[i] * 2)
                            mid_vel = int(last_jointvel_r[i] * 0.7)
                            min_vel = int(last_jointvel_r[i] * 0.5)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 14 + 30
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 28 + 60
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 28 + 80
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 28 + 40
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_r[i] = 1
                            if position_derict == -1:
                                target_vel = 2 * target_vel
                            if i == 1 or i == 6:
                                target_vel = 25
                            g_jointvelocity_r[i] = int(target_vel)
                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                    if 't25' in str(self.robot_name_str_r) or 'l25' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            slow_limit = 1
                            fast_limit = 5
                            max_vel = int(last_jointvel_r[i] * 1.5)
                            min_vel = int(last_jointvel_r[i] * 0.995)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 50
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 100
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 3 + 100
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 5 + 200
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 200
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                else:
                                    target_vel = min_vel
                                    if min_vel < 150:
                                        state_r[i] = 1
                            g_jointvelocity_r[i] = int(target_vel)
                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                        g_jointvelocity_r[6] = 255
                        g_jointvelocity_r[7] = 255
                        g_jointvelocity_r[8] = 255
                        g_jointvelocity_r[9] = 255
                    if 't25' in str(self.robot_name_str_l) or 'l25' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            slow_limit = 1
                            fast_limit = 5
                            max_vel = int(last_jointvel_l[i] * 1.5)
                            min_vel = int(last_jointvel_l[i] * 0.995)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 50
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 100
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 3 + 100
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 5 + 200
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 200
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                else:
                                    target_vel = min_vel
                                    if min_vel < 150:
                                        state_l[i] = 1
                            g_jointvelocity_l[i] = int(target_vel)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                        g_jointvelocity_l[6] = 255
                        g_jointvelocity_l[7] = 255
                        g_jointvelocity_l[8] = 255
                        g_jointvelocity_l[9] = 255
                    if 'l10' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            position_derict = 1 if g_jointpositions_l[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_l[i] * 2)
                            mid_vel = int(last_jointvel_l[i] * 0.7)
                            min_vel = int(last_jointvel_l[i] * 0.5)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 3 + 5
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 3 + 10
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 3 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 3 + 20
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 3 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_l[i] = 1
                            if position_derict == -1:
                                target_vel = 2 * target_vel
                            if i == 0 or i == 1 or i == 9:
                                target_vel = 25
                            g_jointvelocity_l[i] = int(target_vel * 1)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                    if 'o7' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            position_derict = 1 if g_jointpositions_r[i] - lastpos > 0 else -1
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_l[i] * 2)
                            mid_vel = int(last_jointvel_l[i] * 0.7)
                            min_vel = int(last_jointvel_l[i] * 0.5)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 14 + 30
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 28 + 60
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 28 + 120
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 28 + 80
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 28 + 40
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_l[i] = 1
                            if position_derict == -1:
                                target_vel = 2 * target_vel
                            if i == 1 or i == 6:
                                target_vel = 25
                            g_jointvelocity_l[i] = int(target_vel * 1)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                    if 'l20' in str(self.robot_name_str_l):
                        for i in range(len(g_jointpositions_l)):
                            lastpos = last_jointpositions_l[i]
                            position_error = int(abs(g_jointpositions_l[i] - lastpos))
                            slow_limit = 2
                            fast_limit = 10
                            max_vel = int(last_jointvel_l[i] * 2)
                            mid_vel = int(last_jointvel_l[i] * 0.8)
                            min_vel = int(last_jointvel_l[i] * 0.6)
                            target_vel = last_jointvel_l[i]
                            if state_l[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 10 + 5
                                    state_l[i] = 1
                            elif state_l[i] == 1:  # slow
                                if position_error >= fast_limit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 2
                                elif position_error == 0:
                                    state_l[i] = 0
                                else:
                                    target_vel = position_error * 10 + 10
                            else:  # fast
                                if position_error >= fast_limit:
                                    target_vel = position_error * 10 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_limit < position_error < fast_limit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_l[i] = 3
                                elif 0 < position_error <= slow_limit:
                                    target_vel = position_error * 10 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_l[i] = 1
                            g_jointvelocity_l[i] = int(target_vel)
                            if g_jointvelocity_l[i] > 255:
                                g_jointvelocity_l[i] = 255
                            last_jointvel_l[i] = g_jointvelocity_l[i]
                            last_jointpositions_l[i] = g_jointpositions_l[i]
                        
                    if 'l20' in str(self.robot_name_str_r):
                        for i in range(len(g_jointpositions_r)):
                            lastpos = last_jointpositions_r[i]
                            position_error = int(abs(g_jointpositions_r[i] - lastpos))
                            slow_rimit = 2
                            fast_rimit = 10
                            max_vel = int(last_jointvel_r[i] * 2)
                            mid_vel = int(last_jointvel_r[i] * 0.8)
                            min_vel = int(last_jointvel_r[i] * 0.6)
                            target_vel = last_jointvel_r[i]
                            if state_r[i] == 0:  # stop
                                if 0 < position_error:
                                    target_vel = position_error * 10 + 5
                                    state_r[i] = 1
                            elif state_r[i] == 1:  # slow
                                if position_error >= fast_rimit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel > mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 2
                                elif position_error == 0:
                                    state_r[i] = 0
                                else:
                                    target_vel = position_error * 10 + 10
                            else:  # fast
                                if position_error >= fast_rimit:
                                    target_vel = position_error * 10 + 50
                                    if target_vel > max_vel:
                                        target_vel = max_vel
                                elif slow_rimit < position_error < fast_rimit:
                                    target_vel = position_error * 10 + 30
                                    if target_vel < mid_vel:
                                        target_vel = mid_vel
                                    state_r[i] = 3
                                elif 0 < position_error <= slow_rimit:
                                    target_vel = position_error * 10 + 10
                                    if target_vel < min_vel:
                                        target_vel = min_vel
                                    state_r[i] = 1
                            g_jointvelocity_r[i] = int(target_vel)
                            if g_jointvelocity_r[i] > 255:
                                g_jointvelocity_r[i] = 255
                            last_jointvel_r[i] = g_jointvelocity_r[i]
                            last_jointpositions_r[i] = g_jointpositions_r[i]
                    if bool(self.baseconfig["debug"]["joint_motor_debug_r"]) and self.pubprintcount % 1 == 0:
                        print(g_jointpositions_r)
                    if bool(self.baseconfig["debug"]["joint_motor_debug_l"]) and self.pubprintcount % 1 == 0:
                        print(g_jointpositions_l)
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = [f'joint{i + 1}' for i in range(len(g_jointpositions_r))]
                    msg.position = [float(num) for num in g_jointpositions_r]
                    msg.velocity = [float(num) for num in g_jointvelocity_r]
                    self.publisher_r.publish(msg)
                    msg = JointState()
                    msg.name = [f'joint{i + 1}' for i in range(len(g_jointpositions_l))]
                    msg.position = [float(num) for num in g_jointpositions_l]
                    msg.velocity = [float(num) for num in g_jointvelocity_l]
                    self.publisher_l.publish(msg)
                self.pubprintcount = self.pubprintcount + 1
                self.rate.sleep()
            print("Udp Is Ready To Closeing!")
            udp_datacapture.udp_close(dstAddr)
            print("Udp Is Closed!")

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
        fastapi_thread = Thread(target=start_fastapi, args=(stop_event,))
        fastapi_thread.start()

        node = HandRetargetNode()
        if node.motion_type == MotionSource.vtrdyn:
            node.VtrdynRetargeting()
        elif node.motion_type == MotionSource.udexreal:
            node.UdexRealRetargeting()        
        fastapi_thread.join()
    except rospy.ROSInterruptException:
        print("this node is exit !")
        pass
