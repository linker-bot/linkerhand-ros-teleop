import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from linkerhand.linkerforce import ForceSerialReader
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore


class Retarget(Node):
    def __init__(self, port, baudrate, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                 lefthandpubprint: bool, righthandpubprint: bool):
        super().__init__('retarget_force_node')
        
        self.port = port
        self.baudrate = baudrate
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.running = True
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        
        # 根据右手类型初始化
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

        # 根据左手类型初始化
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

        # ROS2 发布器
        self.publisher_r = self.create_publisher(
            JointState,
            '/cb_right_hand_control_cmd',
            self.handcore.hand_numjoints_r)
            
        self.publisher_l = self.create_publisher(
            JointState,
            '/cb_left_hand_control_cmd',
            self.handcore.hand_numjoints_l)
            
        self.timer = self.create_timer(1.0/120, self.process_callback)  # 120Hz
        self.pubprintcount = 0
        self.forcecapture = None

    def initialize_serial(self):
        """初始化串口连接"""
        self.forcecapture = ForceSerialReader(self.port, self.baudrate)
        self.forcecapture.start()
        self.forcecapture.serial_port.write(self.forcecapture.pack_01_data())
        self.forcecapture.serial_port.write(self.forcecapture.pack_02_data(1))
        return True

    def process_callback(self):
        """定时器回调函数，处理数据并发布"""
        if not self.forcecapture or len(self.forcecapture.poslist) == 0:
            return

        pro_qpos_r = self.forcecapture.poslist
        pro_qpos_l = self.forcecapture.poslist

        joint_l = self.lefthand.joint_update(pro_qpos_l)
        joint_r = self.righthand.joint_update(pro_qpos_r)

        # 重定向到电机数据
        self.lefthand.g_jointpositions = self.handcore.trans_to_motor_left(joint_l)
        self.righthand.g_jointpositions = self.handcore.trans_to_motor_right(joint_r)
        
        # 调试打印
        self.get_logger().info(f"关节位置: {joint_l[3]}, 电机位置: {self.lefthand.g_jointpositions[21]}")

        # 速度环节处理
        self.lefthand.speed_update()
        self.righthand.speed_update()

        if self.lefthandpubprint and self.pubprintcount % 1 == 0:
            self.get_logger().info(f"左手位置: {self.lefthand.g_jointpositions}")
        if self.righthandpubprint and self.pubprintcount % 1 == 0:
            self.get_logger().info(f"右手位置: {self.righthand.g_jointpositions}")

        # 发布右手数据
        msg_r = JointState()
        msg_r.header.stamp = self.get_clock().now().to_msg()
        msg_r.name = [f'joint{i + 1}' for i in range(len(self.righthand.g_jointpositions))]
        msg_r.position = [float(num) for num in self.righthand.g_jointpositions]
        msg_r.velocity = [float(num) for num in self.righthand.g_jointvelocity]
        self.publisher_r.publish(msg_r)

        # 发布左手数据
        msg_l = JointState()
        msg_l.header.stamp = self.get_clock().now().to_msg()
        msg_l.name = [f'joint{i + 1}' for i in range(len(self.lefthand.g_jointpositions))]
        msg_l.position = [float(num) for num in self.lefthand.g_jointpositions]
        msg_l.velocity = [float(num) for num in self.lefthand.g_jointvelocity]
        self.publisher_l.publish(msg_l)

        self.pubprintcount += 1

    def process(self):
        """主处理函数"""
        if not self.initialize_serial():
            self.get_logger().error("串口初始化失败")
            return

        rclpy.spin(self)

    def destroy_node(self):
        """节点销毁时清理资源"""
        if self.forcecapture:
            self.forcecapture.stop()
        super().destroy_node()