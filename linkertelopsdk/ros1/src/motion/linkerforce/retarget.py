import time
import rospy
import pickle
import copy
import os
import json
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from linkerhand.linkerforce import ForceSerialReader
from linkerhand.constants import RobotName, ROBOT_LEN_MAP
from linkerhand.handcore import HandCore
from tqdm import tqdm
from colorama import Fore, init
from datetime import datetime, timedelta

TMP_FILE_PATH = "/tmp/jointangle_data.tmp"

init(autoreset=True)

class Retarget:
    def __init__(self, port, baudrate, lefthand: RobotName, righthand: RobotName, handcore: HandCore,
                lefthandpubprint:bool, righthandpubprint: bool, calibration = None):
        self.port = port
        self.baudrate = baudrate
        self.lefthandtype = lefthand
        self.righthandtype = righthand
        self.handcore = handcore
        self.runing = True
        self.pro_qpos_r, self.pro_qpos_l = None, None
        self.lefthandpubprint = lefthandpubprint
        self.righthandpubprint = righthandpubprint
        self.calibration = calibration
        # self.calibrationnormal, self.calibrationnormal = None, None
        # self.calibrationopen_r, self.calibrationopen_l = None, None
        # self.calibrationclose_r, self.calibrationclose_l = None, None
        # self.calibrationindexclose_r, self.calibrationindexclose_l = None, None
        # self.calibrationmiddleclose_r, self.calibrationmiddleclose_l = None, None
        # self.calibrationringclose_r, self.calibrationringclose_l = None, None
        # self.calibrationpinkyclose_r, self.calibrationpinkyclose_l = None, None

        if self.righthandtype == RobotName.o7 \
            or self.righthandtype == RobotName.l7 \
            or self.righthandtype == RobotName.o7v1 \
            or self.righthandtype == RobotName.o7v2:
            from .hand.linkerforce_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.o7:
            from .hand.linkerforce_l7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.l25:
        #     from .hand.linkerforce_l25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        # elif self.righthandtype == RobotName.t25:
        #     from .hand.linkerforce_t25 import RightHand
        #     self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l10v6 :
            from .hand.linkerforce_l10v6 import LeftHand
            self.righthand = LeftHand(handcore, length=ROBOT_LEN_MAP[righthand])        
        elif self.righthandtype == RobotName.l10 \
            or self.righthandtype == RobotName.l10v7 :
            from .hand.linkerforce_l10v7 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])
        elif self.righthandtype == RobotName.l21:
            from .hand.linkerforce_l21 import RightHand
            self.righthand = RightHand(handcore, length=ROBOT_LEN_MAP[righthand])

        if self.lefthandtype == RobotName.o7 \
            or self.lefthandtype == RobotName.l7 \
            or self.lefthandtype == RobotName.o7v1 \
            or self.lefthandtype == RobotName.o7v2:
            from .hand.linkerforce_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.o7:
            from .hand.linkerforce_l7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l25:
            from .hand.linkerforce_l25 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        # elif self.lefthandtype == RobotName.t25:
        #     from .hand.linkerforce_t25 import LeftHand
        #     self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l20:
            from .hand.linkerforce_l20 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10v6 :
            from .hand.linkerforce_l10v6 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l10 \
            or self.lefthandtype == RobotName.l10v7 :
            from .hand.linkerforce_l10v7 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
        elif self.lefthandtype == RobotName.l21:
            from .hand.linkerforce_l21 import LeftHand
            self.lefthand = LeftHand(handcore, length=ROBOT_LEN_MAP[lefthand])
            
        self.publisher_r = rospy.Publisher('/cb_right_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_r)
        self.publisher_l = rospy.Publisher('/cb_left_hand_control_cmd', JointState, queue_size=self.handcore.hand_numjoints_l)
        self.rate = rospy.Rate(120)
        self.pubprintcount = 0

        # 创建订阅者，订阅/cb_left_hand_matrix_touch话题
        rospy.Subscriber("/cb_left_hand_matrix_touch", String, self.touch_left_callback)
        rospy.Subscriber("/cb_right_hand_matrix_touch", String, self.touch_right_callback)
        # 初始化统计结果
        self.results = {
            'left':{},
            'right':{}
        }

    def process_touch_data(self, json_str, hand_type):
        try:
            data = json.loads(json_str)    
            self.results[hand_type] = {}
            # 处理每个手指的矩阵
            for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                matrix = np.array(data[finger])
                # 计算接触面积（非零元素数量）
                contact_area = np.count_nonzero(matrix)         
                # 计算总接触力
                total_force = np.sum(matrix)            
                # 计算平均接触力（避免除以零）
                avg_force = total_force / contact_area if contact_area > 0 else 0
                self.results[hand_type][finger] = {
                    'contact_area': contact_area,
                    'total_force': total_force,
                    'avg_force': avg_force,
                    'max_force': np.max(matrix) if contact_area > 0 else 0
                }
        except Exception as e:
            rospy.logerr("Error processing touch data: %s", str(e))
            return None
        
    def hand_calibration(self) -> bool:
        self.progress_bar(10)
        time.sleep(0.01)
        return True

    def touch_left_callback(self, msg):
        self.process_touch_data(msg.data,'left')   
           
    def touch_right_callback(self, msg):
        self.process_touch_data(msg.data,'right')

    @staticmethod
    def progress_bar(duration=5):
        # 使用tqdm创建进度条
        with tqdm(total=100, desc=f"{Fore.GREEN}进度{Fore.RESET}", bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt}") as pbar:
            for _ in range(100):
                time.sleep(duration/100)
                pbar.update(1)

    def save_to_tmp(self):
        """
        保存 jointangle_r 数据和时间戳到临时文件
        """
        data = {
            'timestamp': datetime.now(),  # 当前时间
            'jointanglenormal_r': self.righthand.calibrationnormal,
            'jointanglenormal_l': self.lefthand.calibrationnormal,
            'jointangleopen_r': self.righthand.calibrationopen,
            'jointangleopen_l': self.lefthand.calibrationopen,
            'jointangleclose_r': self.righthand.calibrationclose,
            'jointangleclose_l': self.lefthand.calibrationclose,
            'jointangleindexclose_r': self.righthand.calibrationindexclose,
            'jointangleindexclose_l': self.lefthand.calibrationindexclose,
            'jointanglemiddleclose_r': self.righthand.calibrationmiddleclose,
            'jointanglemiddleclose_l': self.lefthand.calibrationmiddleclose,
            'jointangleringclose_r': self.righthand.calibrationringclose,
            'jointangleringclose_l': self.lefthand.calibrationringclose,
            'jointanglepinkyclose_r': self.righthand.calibrationpinkyclose,
            'jointanglepinkyclose_l': self.lefthand.calibrationpinkyclose
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
        # if 'timestamp' not in data or not data['timestamp']:
        #     return False

        # # 检查是否超过1小时                     
        # time_diff = datetime.now() - data['timestamp']
        # if time_diff > timedelta(hours=1):
        #     return False
        self.righthand.calibrationnormal = data['jointanglenormal_r']
        self.lefthand.calibrationnormal = data['jointanglenormal_l']
        self.righthand.calibrationopen = data['jointangleopen_r']
        self.lefthand.calibrationopen = data['jointangleopen_l']
        self.righthand.calibrationclose = data['jointangleclose_r']
        self.lefthand.calibrationclose = data['jointangleclose_l']
        self.righthand.calibrationindexclose = data['jointangleindexclose_r']
        self.lefthand.calibrationindexclose = data['jointangleindexclose_l']
        self.righthand.calibrationmiddleclose = data['jointanglemiddleclose_r']
        self.lefthand.calibrationmiddleclose = data['jointanglemiddleclose_l']
        self.righthand.calibrationringclose = data['jointangleringclose_r']
        self.lefthand.calibrationringclose = data['jointangleringclose_l']
        self.righthand.calibrationpinkyclose = data['jointanglepinkyclose_r']
        self.lefthand.calibrationpinkyclose = data['jointanglepinkyclose_l']
        return True
    
    def process(self):
        forcecapture = ForceSerialReader('/dev/ttyUSB0', 2000000)
        forcecapture.start()
        # forcecapture.serial_port.write(forcecapture.pack_01_data())
        # forcecapture.serial_port.write(forcecapture.pack_02_data(1))
        if self.load_from_tmp() is True and self.calibration is not None:
            self.calibration = -1
            # self.righthand.get_joint_diff(self.calibrationnormal,self.calibrationopen,self.calibrationclose)
            # self.lefthand.get_joint_diff(self.calibrationnormal,self.calibrationopen,self.calibrationclose)
            # print(self.calibrationclose)
            # print(self.calibrationopen)
            # print(self.calibrationnormal)
        while True:
            if len(forcecapture.rightposlist) == 0 and len(forcecapture.rightposlist) == 0 : continue
            if self.calibration is None:
                print("请张开双手,五指并拢，...")
                if self.hand_calibration():
                    self.righthand.calibrationnormal = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationnormal = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = 0
                continue
            elif self.calibration == 0:
                print("请握紧双手...")
                if self.hand_calibration():
                    self.righthand.calibrationclose = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationclose = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = 1
                continue
            elif self.calibration == 1:
                print("请张开双手,五指伸展开，...")
                if self.hand_calibration():
                    self.righthand.calibrationopen = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationopen = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = 2
                continue
            elif self.calibration == 2:
                print("请闭合食指和拇指...")
                if self.hand_calibration():
                    self.righthand.calibrationindexclose = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationindexclose = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = 3
                continue
            elif self.calibration == 3:
                print("请闭合中指和拇指...")
                if self.hand_calibration():
                    self.righthand.calibrationmiddleclose = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationmiddleclose = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = 4
                continue
            elif self.calibration == 4:
                print("请闭合无名指和拇指...")
                if self.hand_calibration():
                    self.righthand.calibrationringclose = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationringclose = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = 5
                continue
            elif self.calibration == 5:
                print("请闭合小指和拇指...")
                if self.hand_calibration():
                    self.righthand.calibrationpinkyclose = copy.deepcopy(forcecapture.rightposlist)
                    self.lefthand.calibrationpinkyclose = copy.deepcopy(forcecapture.rightposlist)
                    self.calibration = -1
                    self.save_to_tmp()
                    # self.righthand.get_joint_diff(self.calibrationnormal,self.calibrationopen,self.calibrationclose)
                    # self.lefthand.get_joint_diff(self.calibrationnormal,self.calibrationopen,self.calibrationclose)
                continue
            if self.results['right']:
                forcecapture.rightforcelist = [
                    self.results['right']['thumb_matrix']['max_force'],
                    self.results['right']['index_matrix']['max_force'],
                    self.results['right']['middle_matrix']['max_force'],
                    self.results['right']['ring_matrix']['max_force'],
                    self.results['right']['little_matrix']['max_force']
                ]
                # print(forcecapture.rightforcelist)
            if self.results['left']:
                forcecapture.leftforcelist = [
                    self.results['left']['thumb_matrix']['max_force'],
                    self.results['left']['index_matrix']['max_force'],
                    self.results['left']['middle_matrix']['max_force'],
                    self.results['left']['ring_matrix']['max_force'],
                    self.results['left']['little_matrix']['max_force']
                ]
                # print(forcecapture.leftforcelist)
            self.pro_qpos_r = forcecapture.rightposlist
            self.pro_qpos_l = forcecapture.rightposlist

            self.lefthand.joint_update(self.pro_qpos_l)
            self.righthand.joint_update(self.pro_qpos_r)

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
