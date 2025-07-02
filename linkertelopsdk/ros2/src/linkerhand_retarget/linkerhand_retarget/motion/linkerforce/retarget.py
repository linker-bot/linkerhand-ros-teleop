import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from linkerhand.linkerforcecore import linkerforceScoketUdp
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore


class Retarget():
    def __init__(self, node, ip, port, deviceid, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                lefthandpubprint: bool, righthandpubprint: bool):
        self.node = node
        self.udp_ip = ip
        self.udp_port = port
        self.motion_device = deviceid
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        
        # 根据右手类型初始化
        if self.righthandtype == RobotName.o7:
            from .hand.linkerforce_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l25:
            from .hand.linkerforce_l25 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.t25:
            from .hand.linkerforce_t25 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10:
            from .hand.linkerforce_l10 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.linkerforce_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l30:
            from .hand.linkerforce_l30 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        # 根据左手类型初始化
        if self.lefthandtype == RobotName.o7:
            from .hand.linkerforce_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.linkerforce_l25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.t25:
            from .hand.linkerforce_t25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10:
            from .hand.linkerforce_l10 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.linkerforce_l21 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l30:
            from .hand.linkerforce_l30 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])

        # ROS2 发布器
        self.publisher_r = self.node.create_publisher(
            JointState,
            '/cb_right_hand_control_cmd',
            self.handcore.hand_numjoints_r)
            
        self.publisher_l = self.node.create_publisher(
            JointState,
            '/cb_left_hand_control_cmd',
            self.handcore.hand_numjoints_l)
            
        self.timer = self.node.create_timer(1.0/120, self.process_callback)  # 120Hz
        self.pubprintcount = 0
        self.udp_datacapture = None

    def initialize_udp(self):
        """初始化UDP连接"""
        self.udp_datacapture = linkerforceScoketUdp(
            host=self.udp_ip,
            port=self.udp_port,
            device_id=self.motion_device)
        return self.udp_datacapture.udp_initial()

    def process_callback(self):
        if not self.runing:
            return
        
        """定时器回调函数，处理数据并发布"""
        if not self.udp_datacapture or not self.udp_datacapture.udp_is_onnect():
            self.node.get_logger().warning("侦测到UDP断开状态，正在重连！")
            if not self.initialize_udp():
                self.get_logger().error("UDP重连失败")
                return
            time.sleep(2)
            return

        mocapdata = self.udp_datacapture.realmocapdata
        if not mocapdata.is_update:
            return

        # 处理左右手原始数据
        self.lefthand.joint_update(mocapdata.jointangle_lHand)
        self.righthand.joint_update(mocapdata.jointangle_rHand)

        # 速度环节处理
        self.lefthand.speed_update()
        self.righthand.speed_update()

        # 调试打印
        if self.lefthandpubprint and self.pubprintcount % 1 == 0:
            self.node.get_logger().info(f"左手位置: {self.lefthand.g_jointpositions}")
        if self.righthandpubprint and self.pubprintcount % 1 == 0:
            self.node.get_logger().info(f"右手位置: {self.righthand.g_jointpositions}")

        # 发布右手数据
        msg_r = JointState()
        msg_r.header.stamp = self.node.get_clock().now().to_msg()
        msg_r.name = [f'joint{i + 1}' for i in range(len(self.righthand.g_jointpositions))]
        msg_r.position = [float(num) for num in self.righthand.g_jointpositions]
        msg_r.velocity = [float(num) for num in self.righthand.g_jointvelocity]
        self.publisher_r.publish(msg_r)

        # 发布左手数据
        msg_l = JointState()
        msg_l.header.stamp = self.node.get_clock().now().to_msg()
        msg_l.name = [f'joint{i + 1}' for i in range(len(self.lefthand.g_jointpositions))]
        msg_l.position = [float(num) for num in self.lefthand.g_jointpositions]
        msg_l.velocity = [float(num) for num in self.lefthand.g_jointvelocity]
        self.publisher_l.publish(msg_l)

        self.pubprintcount += 1

    def process(self):
        """主处理函数"""
        if not self.initialize_udp():
            self.get_logger().error("初始化配置网络失败")
            return

        rclpy.spin(self.node)