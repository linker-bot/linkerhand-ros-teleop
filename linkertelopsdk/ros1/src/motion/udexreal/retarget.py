import time
import rospy
from sensor_msgs.msg import JointState
from linkerhand.udexrealcore import UdexRealScoketUdp
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore
import numpy as np


class Retarget:
    def __init__(self, ip, port, deviceid, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                lefthandpubprint:bool, righthandpubprint: bool, calibration = None):
        self.udp_ip = ip
        self.udp_port = port
        self.motion_device = deviceid
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint

        
        if self.righthandtype == RobotName.l7:
            from .hand.udexreal_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.udexreal_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10:
            from .hand.udexreal_l10 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.udexreal_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        if self.lefthandtype == RobotName.l7:
            from .hand.udexreal_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.udexreal_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10:
            from .hand.udexreal_l10 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.udexreal_l21 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])

        self.publisher_r = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_r)
        self.publisher_l = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_l)
        self.rate = rospy.Rate(120)
        self.pubprintcount = 0

    def process(self):
        udp_datacapture = UdexRealScoketUdp(host=self.udp_ip, port=self.udp_port, device_id=self.motion_device)
        if udp_datacapture.udp_initial():
            
            
            while self.runing:
                if not udp_datacapture.udp_is_onnect():
                    print("侦测到UDP断开状态，正在重连！")
                    udp_datacapture.udp_initial()
                    time.sleep(2)
                    continue
                mocapdata = udp_datacapture.realmocapdata
                
                if mocapdata.is_update:
                    # 处理左右手原始数据+电机重定向
                    self.lefthand.joint_update(mocapdata.jointangle_lHand)
                    self.righthand.joint_update(mocapdata.jointangle_rHand)
                    
                    # 速度环节处理
                    self.lefthand.speed_update()
                    self.righthand.speed_update()
                    # print(self.lefthand.g_jointpositions)

                    if self.lefthandpubprint and self.pubprintcount % 1 == 0:
                        print(self.lefthand.g_jointpositions)
                    if self.righthandpubprint and self.pubprintcount % 1 == 0:
                        print(self.righthand.g_jointpositions )
                    msg_right = JointState()
                    msg_right.header.stamp = rospy.Time.now()
                    msg_right.name = [f'joint{i + 1}' for i in range(len(self.righthand.g_jointpositions ))]
                    msg_right.position = [float(num) for num in self.righthand.g_jointpositions ]
                    msg_right.velocity = [float(num) for num in self.righthand.g_jointvelocity ]
                    
                    msg_left = JointState()
                    msg_left.name = [f'joint{i + 1}' for i in range(len(self.lefthand.g_jointpositions ))]
                    msg_left.position = [float(num) for num in self.lefthand.g_jointpositions ]
                    msg_left.velocity = [float(num) for num in self.lefthand.g_jointvelocity ]

                    self.righthand.glove_torch(msg_right) 
                    self.lefthand.glove_torch(msg_left)
                       
                             
                    self.publisher_r.publish(msg_right)
                    self.publisher_l.publish(msg_left)
                    
                self.pubprintcount = self.pubprintcount + 1
                
                self.rate.sleep()
        else:
            print("初始化配置网络失败")
