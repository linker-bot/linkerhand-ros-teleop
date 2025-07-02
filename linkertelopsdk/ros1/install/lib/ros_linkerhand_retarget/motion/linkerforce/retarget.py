
import time
import rospy
from sensor_msgs.msg import JointState
from linkerhand.linkerforce import ForceSerialReader
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore



class Retarget:
    def __init__(self, port, baudrate, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                 lefthandpubprint:bool, righthandpubprint: bool):
        self.port = port
        self.baudrate = baudrate
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        if self.righthandtype == RobotName.o7:
            from .hand.linkerforce_o7 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l25:
            from .hand.linkerforce_l25 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.t25:
            from .hand.linkerforce_t25 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10:
            from .hand.linkerforce_l10 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.linkerforce_l21 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])

        if self.lefthandtype == RobotName.o7:
            from .hand.linkerforce_o7 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.linkerforce_l25 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.t25:
            from .hand.linkerforce_t25 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10:
            from .hand.linkerforce_l10 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.linkerforce_l21 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        self.publisher_r = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_r)
        self.publisher_l = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_l)
        self.rate = rospy.Rate(120)
        self.pubprintcount = 0

    def process(self):
        forcecapture = ForceSerialReader("COM100", 2000000)
        forcecapture.start()
        forcecapture.serial_port.write(forcecapture.pack_01_data())
        forcecapture.serial_port.write(forcecapture.pack_02_data(1))
        while True:
            if len(forcecapture.poslist) == 0: continue
            pro_qpos_r = forcecapture.poslist
            pro_qpos_l = forcecapture.poslist

            joint_l = self.lefthand.joint_update(pro_qpos_l)
            joint_r = self.righthand.joint_update(pro_qpos_r)

            # 重定向到电机数据
            self.lefthand.g_jointpositions = self.handcore.trans_to_motor_left(joint_l)
            self.righthand.g_jointpositions = self.handcore.trans_to_motor_right(joint_r)
            print(joint_l[3],self.lefthand.g_jointpositions[21])
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