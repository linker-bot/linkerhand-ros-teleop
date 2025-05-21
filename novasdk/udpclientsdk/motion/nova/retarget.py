import time
import pickle
import copy
import os
from linkerhand_core.canfun import CanMessageSender
from linkerhand_core.sensenovacore import SenseNovaData, SenseNovaScoketUdp
from linkerhand_core.constants import RobotName, ROBOT_LEN_MAP
from linkerhand_core.handcore import HandCore

from datetime import datetime, timedelta
from tqdm import tqdm

TMP_FILE_PATH = "/tmp/jointangle_data.tmp"


class Retarget:
    def __init__(self, ip, port, deviceid, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                 cansender: CanMessageSender = None):
        self.udp_ip = ip
        self.udp_port = port
        self.motion_device = deviceid
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.sender = cansender
        self.calibration = None
        self.calibrationopen_r, self.calibrationopen_l, self.calibrationclose_r, self.calibrationclose_l = None, None, None, None
        if self.righthandtype == RobotName.o7:
            from .hand.nova_o7 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l25:
            from .hand.nova_l25 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.t25:
            from .hand.nova_t25 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.nova_l20 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10:
            from .hand.nova_l10 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.nova_l21 import RightHand
            self.righthand = RightHand(length=ROBOT_LEN_MAP[righthand])

        if self.lefthandtype == RobotName.o7:
            from .hand.nova_o7 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.nova_l25 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.t25:
            from .hand.nova_t25 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.nova_l20 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10:
            from .hand.nova_l10 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.nova_l21 import LeftHand
            self.lefthand = LeftHand(length=ROBOT_LEN_MAP[lefthand])

    def process(self):
        udp_datacapture = SenseNovaScoketUdp(host=self.udp_ip, port=self.udp_port)
        if udp_datacapture.udp_initial():
            if self.sender is not None:
                # 启动发送线程
                self.sender.start_send_thread()
            mocapopen = SenseNovaData()
            mocapclose = SenseNovaData()
            if self.load_from_tmp() is True:
                mocapclose.jointangle_rHand = self.calibrationclose_r
                mocapclose.jointangle_lHand = self.calibrationclose_l
                mocapopen.jointangle_rHand = self.calibrationopen_r
                mocapopen.jointangle_lHand = self.calibrationopen_l
                self.calibration = 1
            while self.runing:
                if not udp_datacapture.udp_is_onnect():
                    print("侦测到UDP断开状态，正在重连！")
                    udp_datacapture.udp_initial()
                    time.sleep(2)
                    continue
                if self.calibration is None:
                    print("\n请四指并拢，张开双手...")
                    if self.hand_calibration():
                        mocapopen = copy.deepcopy(udp_datacapture.realmocapdata)
                        self.calibrationopen_r = mocapopen.jointangle_rHand
                        self.calibrationopen_l = mocapopen.jointangle_lHand
                        self.calibration = 0
                    continue
                elif self.calibration == 0:
                    print("\n请握紧双手...")
                    if self.hand_calibration():
                        mocapclose = copy.deepcopy(udp_datacapture.realmocapdata)
                        self.calibrationclose_r = mocapclose.jointangle_rHand
                        self.calibrationclose_l = mocapclose.jointangle_lHand
                        self.calibration = 1
                        self.save_to_tmp()
                    continue
                mocapdata = udp_datacapture.realmocapdata
                if mocapdata.is_update:
                    # 处理左右手原始数据
                    joint_l = self.lefthand.joint_update(mocapdata.jointangle_lHand,
                                                         mocapopen.jointangle_lHand,
                                                         mocapclose.jointangle_lHand)
                    joint_r = self.righthand.joint_update(mocapdata.jointangle_rHand,
                                                          mocapopen.jointangle_rHand,
                                                          mocapclose.jointangle_rHand)
                    # print(joint_r[20])
                    # 重定向到电机数据
                    self.lefthand.g_jointpositions = self.handcore.trans_to_motor_left(joint_l)
                    self.righthand.g_jointpositions = self.handcore.trans_to_motor_right(joint_r)
                    # 速度环节处理
                    self.lefthand.speed_update()
                    self.righthand.speed_update()
                    # print(self.lefthand.g_jointpositions)
                    if self.sender is not None:
                        self.sender.g_jointpositions_r, self.sender.g_jointpositions_l = self.righthand.g_jointpositions, self.lefthand.g_jointpositions
                        self.sender.g_jointvel_r, self.sender.g_jointvel_l = self.righthand.g_jointvelocity, self.lefthand.g_jointvelocity
                time.sleep(0.001)
        else:
            print("初始化配置网络失败")

    def hand_calibration(self) -> bool:
        self.progress_bar(5)
        time.sleep(0.01)
        return True

    @staticmethod
    def progress_bar(duration=5):
        # 使用tqdm创建进度条
        for _ in tqdm(range(duration * 10), desc="进度", ncols=100):
            time.sleep(0.1)  # 每次迭代暂停0.1秒，总共5秒

    def save_to_tmp(self):
        """
        保存 jointangle_r 数据和时间戳到临时文件
        """
        data = {
            'timestamp': datetime.now(),  # 当前时间
            'jointangleopen_r': self.calibrationopen_r,
            'jointangleopen_l': self.calibrationopen_l,
            'jointangleclose_r': self.calibrationclose_r,
            'jointangleclose_l': self.calibrationclose_l
        }

        try:
            with open(TMP_FILE_PATH, 'wb') as f:
                pickle.dump(data, f)
            return True
        except Exception as e:
            print(f"保存失败: {e}")
            return False

    def load_from_tmp(self):
        """
        从临时文件读取数据，检查时间戳有效性
        - 如果文件不存在、时间戳为空或超过1小时，返回 False
        - 否则返回 jointangle_r 数据
        """
        if not os.path.exists(TMP_FILE_PATH):
            return False

        try:
            with open(TMP_FILE_PATH, 'rb') as f:
                data = pickle.load(f)
        except Exception as e:
            print(f"读取失败: {e}")
            return False

        # 检查时间戳
        if 'timestamp' not in data or not data['timestamp']:
            return False

        # 检查是否超过1小时
        time_diff = datetime.now() - data['timestamp']
        if time_diff > timedelta(hours=1):
            return False

        self.calibrationopen_r = data['jointangleopen_r']
        self.calibrationopen_l = data['jointangleopen_l']
        self.calibrationclose_r = data['jointangleclose_r']
        self.calibrationclose_l = data['jointangleclose_l']

        return True
