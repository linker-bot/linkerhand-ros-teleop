import time
import rospy
from sensor_msgs.msg import JointState
from linkerhand.vtrdyncore import VtrdynSocketUdp, MocapData
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore


class Retarget:
    def __init__(self,
                ip,
                port,
                lefthand: RobotName,
                righthand: RobotName,
                handcore: HandCore,
                lefthandpubprint: bool,
                righthandpubprint: bool,
                calibration=None):
        self.udp_ip = ip
        self.udp_port = port
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        if self.righthandtype == RobotName.o7 \
            or self.righthandtype == RobotName.l7 \
            or self.righthandtype == RobotName.o7v1 \
            or self.righthandtype == RobotName.o7v2:
            from .hand.vtrdyn_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o6:
            from .hand.vtrdyn_o6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l6:
            from .hand.vtrdyn_l6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l25:
            from .hand.vtrdyn_l25 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.vtrdyn_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10v6 :
            from .hand.vtrdyn_l10v6 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])        
        elif self.righthandtype == RobotName.l10 \
            or self.righthandtype == RobotName.l10v7 :
            from .hand.vtrdyn_l10v7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.vtrdyn_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        
        if self.lefthandtype == RobotName.o7 \
            or self.lefthandtype == RobotName.l7 \
            or self.lefthandtype == RobotName.o7v1 \
            or self.lefthandtype == RobotName.o7v2:
            from .hand.vtrdyn_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o6:
            from .hand.vtrdyn_o6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l6:
            from .hand.vtrdyn_l6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.vtrdyn_l25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.vtrdyn_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10v6 :
            from .hand.vtrdyn_l10v6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10 \
            or self.lefthandtype == RobotName.l10v7 :
            from .hand.vtrdyn_l10v7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.vtrdyn_l21 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])

        self.publisher_r = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_r)
        self.publisher_l = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_l)
        self.rate = rospy.Rate(120)
        self.pubprintcount = 0

    def process(self):
        udp_datacapture = VtrdynSocketUdp()
        if udp_datacapture.udp_initial(2223):
            dstAddr = udp_datacapture.udp_getsockaddr(self.udp_ip, self.udp_port)
            udp_datacapture.udp_send_request_connect(dstAddr)
            while self.runing:
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
                    qpos_r = self.handcore.projection_process(right_hand_pose)
                    qpos_l = self.handcore.projection_process(left_hand_pose)
                    qpos_r[0] = qpos_r[0] - 0.11
                    qpos_r[1] = qpos_r[1] - 0.09
                    qpos_r[2] = qpos_r[2] - 0.25
                    qpos_r[3] = qpos_r[3] - 0.12
                    qpos_r[4] = qpos_r[4] - 0.08
                    qpos_l[0] = qpos_l[0] - 0.15
                    qpos_l[1] = qpos_l[1] - 0.21
                    qpos_l[2] = qpos_l[2] - 0.15
                    qpos_l[3] = qpos_l[3] - 0.21
                    qpos_l[4] = qpos_l[4] - 0.11
                    # 处理左右手原始数据+重定向
                    self.lefthand.joint_update(qpos_l)
                    self.righthand.joint_update(qpos_r)
                    # 速度环节处理
                    self.lefthand.speed_update()
                    self.righthand.speed_update()

                    if self.lefthandpubprint and self.pubprintcount % 1 == 0:
                        print(self.lefthand.g_jointpositions)
                    if self.righthandpubprint and self.pubprintcount % 1 == 0:
                        print(self.righthand.g_jointpositions )
                    msg = JointState()
                    msg.header.stamp = rospy.Time.now()
                    msg.name = [f'joint{i + 1}' for i in range(len(self.righthand.g_jointpositions ))]
                    msg.position = [float(num) for num in self.righthand.g_jointpositions ]
                    msg.velocity = [float(num) for num in self.righthand.g_jointvelocity ]
                    self.publisher_r.publish(msg)
                    msg = JointState()
                    msg.name = [f'joint{i + 1}' for i in range(len(self.lefthand.g_jointpositions ))]
                    msg.position = [float(num) for num in self.lefthand.g_jointpositions ]
                    msg.velocity = [float(num) for num in self.lefthand.g_jointvelocity ]
                    self.publisher_l.publish(msg)
                self.pubprintcount = self.pubprintcount + 1
                self.rate.sleep()               
        else:
            print("初始化配置网络失败")
