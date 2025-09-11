import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from linkerhand.vtrdyncore import VtrdynSocketUdp, MocapData
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore


class Retarget:
    def __init__(self,
                node, 
                ip,
                port,
                lefthand: RobotName,
                righthand: RobotName,
                handcore: HandCore,
                lefthandpubprint: bool,
                righthandpubprint: bool,
                calibration=None):
        self.node = node
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
        self.pubprintcount = 0
        self.udp_datacapture,self.dstAddr  = None,None

    def initialize_udp(self):
        """初始化UDP连接"""
        self.node.get_logger().info(f"正在初始化UDP连接 -> IP: {self.udp_ip}, 端口: {self.udp_port}")
        self.udp_datacapture = VtrdynSocketUdp()
        if self.udp_datacapture.udp_initial(2223):
            self.dstAddr = self.udp_datacapture.udp_getsockaddr(self.udp_ip, self.udp_port)
            self.node.get_logger().info(f"UDP连接初始化成功 -> 目标地址: {self.udp_ip}:{self.udp_port}")
            self.udp_datacapture.udp_send_request_connect(self.dstAddr)
            return True
        else:
            self.node.get_logger().error("UDP连接初始化失败！")
            return False
    

    def process_callback(self):
        if not self.runing:
            return

        """定时器回调函数，处理数据并发布"""
        if not self.udp_datacapture.udp_is_onnect():
            self.node.get_logger().warning("侦测到UDP断开状态，正在重连！")
            if self.udp_datacapture.udp_initial(2223):
                if self.udp_datacapture.udp_send_request_connect(self.dstAddr):
                    self.node.get_logger().info("与服务器建立链路，启动接收线程......")
                else:
                    self.node.get_logger().info("未与服务器正常通讯，请检查通讯连接......")
                    time.sleep(2)
                    return
            else:
                self.node.get_logger().error("UDP重连初始化失败！")
                time.sleep(2)
                return
        
        mocapdata = MocapData()
        self.udp_datacapture.udp_recv_mocap_data(mocapdata)  # 接收数据
        if not mocapdata.is_update:
            return

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