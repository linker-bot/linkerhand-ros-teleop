import numpy as np
from linkerhand.handcore import HandCore


class RightHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.calibrationnormal = None
        self.calibrationopen = None
        self.calibrationclose = None
        self.calibrationindexclose = None
        self.calibrationmiddleclose = None
        self.calibrationringclose = None
        self.calibrationpinkyclose = None
        self.handretarget = np.array(handcore.retagetconfig["right_hand"]["l20"])
        self.fourroll_diff = None
        self.fourroll_index = np.array([1,2,5,9,13,17])
        
    # def get_joint_diff(self):
    #     all_diff = np.array(self.calibrationclose ) - np.array(self.calibrationopen)
    #     self.thumb_diff = all_diff[:5]
    #     self.index_diff = all_diff[6:9]
    #     self.middle_diff = all_diff[10:13]
    #     self.ring_diff = all_diff[14:17]
    #     self.pinky_diff = all_diff[18:21]
    #     fourroll_diff_array = np.array(self.calibrationopen) - np.array(self.calibrationnormal)
    #     self.fourroll_diff = fourroll_diff_array[self.fourroll_index]   
    #     print(self.thumb_diff ,self.index_diff ,self.middle_diff ,self.ring_diff,self.pinky_diff )
    #     print(self.fourroll_diff)

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        joint_diff = np.array(joint_arc) - np.array(self.calibrationopen)
        fourroll_diff_array = np.array(joint_arc) - np.array(self.calibrationnormal)
        fourroll_diff = fourroll_diff_array[self.fourroll_index]   
    #     fourroll_diff_array = np.array(open) - np.array(normal)
    #     self.fourroll_diff = fourroll_diff_array[fourroll_index]  

        qpos[0] = 0
        if joint_arc[6] < 0.2 and joint_arc[7] < 0.2 and joint_arc[8] < 0.2:
            # print("手指伸直状态", joint_arc[6], joint_arc[7], joint_arc[8])
            qpos[1] = 0
            qpos[2] = 0
            qpos[3] = 0
        elif joint_arc[6] > joint_arc[7] > joint_arc[8]:
            # print("2，3关节伸直状态", joint_arc[6],joint_arc[7],joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[1] = (joint_arc[6] - 0.6) * 4
            qpos[2] = joint_arc[8] / 2
            qpos[3] = joint_arc[8] / 2
        else:
            # print("2，3关节弯曲状态", joint_arc[6], joint_arc[7], joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[1] = (joint_arc[6] - 0.6) * 4
            qpos[2] = joint_arc[8]
            qpos[3] = joint_arc[8]

        qpos[4] = 0
        if joint_arc[18] < 0.2 and joint_arc[19] < 0.2 and joint_arc[20] < 0.2:
            # print("手指伸直状态", joint_arc[6], joint_arc[7], joint_arc[8])
            qpos[5] = 0
            qpos[6] = 0
            qpos[7] = 0
        elif joint_arc[18] > joint_arc[19] > joint_arc[20]:
            # print("2，3关节伸直状态", joint_arc[6],joint_arc[7],joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[5] = (joint_arc[18] - 0.6) * 4
            qpos[6] = joint_arc[20] / 2
            qpos[7] = joint_arc[20] / 2
        else:
            # print("2，3关节弯曲状态", joint_arc[6], joint_arc[7], joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[5] = (joint_arc[18] - 0.6) * 4
            qpos[6] = joint_arc[20]
            qpos[7] = joint_arc[20]

        qpos[8] = 0
        if joint_arc[10] < 0.2 and joint_arc[11] < 0.2 and joint_arc[12] < 0.2:
            # print("手指伸直状态", joint_arc[10], joint_arc[11], joint_arc[12])
            qpos[9] = 0
            qpos[10] = 0
            qpos[11] = 0
        elif joint_arc[10] > joint_arc[11] > joint_arc[12]:
            # print("2，3关节伸直状态", joint_arc[10],joint_arc[11],joint_arc[12])
            qpos[9] = (joint_arc[10] - 0.7) * 4
            qpos[10] = joint_arc[10] / 2
            qpos[11] = joint_arc[10] / 2
        else:
            if joint_arc[11] < 0.6 and joint_arc[10] < 0.5:
                qpos[9] = (joint_arc[10] - 0.7) * 4
                qpos[10] = joint_arc[10] / 2
                qpos[11] = joint_arc[10] / 2
            elif joint_arc[11] < joint_arc[10] < 0.6 and 0.5 > joint_arc[10] > joint_arc[12]:
                qpos[9] = (joint_arc[10] - 0.7) * 4
                qpos[10] = joint_arc[10] / 2
                qpos[11] = joint_arc[10] / 2
            else:
                qpos[9] = (joint_arc[10] - 0.7) * 4
                qpos[10] = joint_arc[12]
                qpos[11] = joint_arc[12]

        qpos[12] = 0
        if joint_arc[14] < 0.2 and joint_arc[15] < 0.2 and joint_arc[16] < 0.2:
            # print("手指伸直状态", joint_arc[14], joint_arc[15], joint_arc[16])
            qpos[13] = 0
            qpos[14] = joint_arc[14] / 2
            qpos[15] = joint_arc[14] / 2
        elif joint_arc[14] > joint_arc[15] > joint_arc[16]:
            # print("2，3关节伸直状态1", joint_arc[14],joint_arc[15],joint_arc[16])
            qpos[13] = (joint_arc[14] - 0.7) * 4
            qpos[14] = joint_arc[14] / 2
            qpos[15] = joint_arc[14] / 2
        else:
            if joint_arc[11] < joint_arc[10] < 0.9 and 0.9 > joint_arc[10] > joint_arc[12]:
                # print("2，3关节伸直状态2", joint_arc[14], joint_arc[15], joint_arc[16])
                qpos[13] = (joint_arc[14] - 0.7) * 4
                qpos[14] = joint_arc[14] / 2
                qpos[15] = joint_arc[14] / 2
            else:
                # print("2，3关节弯曲状态", joint_arc[14], joint_arc[15], joint_arc[16])
                qpos[13] = (joint_arc[14] - 0.7) * 4
                qpos[14] = joint_arc[16]
                qpos[15] = joint_arc[16]

        thumb_diff = joint_diff[:5]
        # print(thumb_diff)
        qpos[16] = thumb_diff[1] * -1.5
        qpos[17] = thumb_diff[1] * -2.5
        qpos[18] = 0
        qpos[19] = 0
        qpos[20] = 0
        if thumb_diff[2] < 0.2 and thumb_diff[3] < 0.2 and thumb_diff[4] < 0.2:
            # print("手指伸直状态", joint_arc[1],joint_arc[2], joint_arc[3], joint_arc[4])
            qpos[18] = 0
            qpos[19] = 0
        elif joint_arc[2] < joint_arc[4]:
            # print("2关节伸直状态", joint_arc[2], joint_arc[3], joint_arc[4])
            qpos[19] = thumb_diff[2] / 2
            qpos[20] = thumb_diff[4]
        else:
            if thumb_diff[2] > thumb_diff[4]:
                # print("1关节弯曲状态", joint_arc[18], joint_arc[19], joint_arc[20])
                qpos[18] = thumb_diff[2] / 4
                qpos[19] = thumb_diff[2]
                qpos[20] = thumb_diff[4]
            else:
                # print("3关节弯曲状态", joint_arc[2], joint_arc[3], joint_arc[4])
                qpos[19] = thumb_diff[2]
                qpos[20] = thumb_diff[4]

        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slowimit = 2
            fastimit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.8)
            min_vel = int(self.last_jointvelocity[i] * 0.6)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 10 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fastimit:
                    target_vel = position_error * 10 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 10 + 10
            else:  # fast
                if position_error >= fastimit:
                    target_vel = position_error * 10 + 50
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slowimit < position_error < fastimit:
                    target_vel = position_error * 10 + 30
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slowimit:
                    target_vel = position_error * 10 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity [i] = int(255)

            if self.g_jointvelocity [i] > 255:
                self.g_jointvelocity [i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity [i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


class LeftHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.calibrationnormal = None
        self.calibrationopen = None
        self.calibrationclose = None
        self.calibrationindexclose = None
        self.calibrationmiddleclose = None
        self.calibrationringclose = None
        self.calibrationpinkyclose = None

    # def get_joint_diff(self,normal,open,close):
    #     all_diff = np.array(close) - np.array(open)
    #     self.thumb_diff = all_diff[:5]
    #     self.index_diff = all_diff[6:9]
    #     self.middle_diff = all_diff[10:13]
    #     self.ring_diff = all_diff[14:17]
    #     self.pinky_diff = all_diff[18:21]
    #     fourroll_index = np.array([1,2,5,9,13,17])
    #     fourroll_diff_array = np.array(open) - np.array(normal)
    #     self.fourroll_diff = fourroll_diff_array[fourroll_index]   

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)

        qpos[0] = 0
        if joint_arc[6] < 0.2 and joint_arc[7] < 0.2 and joint_arc[8] < 0.2:
            # print("手指伸直状态", joint_arc[6], joint_arc[7], joint_arc[8])
            qpos[1] = 0
            qpos[2] = 0
            qpos[3] = 0
        elif joint_arc[6] > joint_arc[7] > joint_arc[8]:
            # print("2，3关节伸直状态", joint_arc[6],joint_arc[7],joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[1] = (joint_arc[6] - 0.6) * 4
            qpos[2] = joint_arc[8] / 2
            qpos[3] = joint_arc[8] / 2
        else:
            # print("2，3关节弯曲状态", joint_arc[6], joint_arc[7], joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[1] = (joint_arc[6] - 0.6) * 4
            qpos[2] = joint_arc[8]
            qpos[3] = joint_arc[8]

        qpos[4] = 0
        if joint_arc[18] < 0.2 and joint_arc[19] < 0.2 and joint_arc[20] < 0.2:
            # print("手指伸直状态", joint_arc[6], joint_arc[7], joint_arc[8])
            qpos[5] = 0
            qpos[6] = 0
            qpos[7] = 0
        elif joint_arc[18] > joint_arc[19] > joint_arc[20]:
            # print("2，3关节伸直状态", joint_arc[6],joint_arc[7],joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[5] = (joint_arc[18] - 0.6) * 4
            qpos[6] = joint_arc[20] / 2
            qpos[7] = joint_arc[20] / 2
        else:
            # print("2，3关节弯曲状态", joint_arc[6], joint_arc[7], joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[5] = (joint_arc[18] - 0.6) * 4
            qpos[6] = joint_arc[20]
            qpos[7] = joint_arc[20]

        qpos[8] = 0
        if joint_arc[10] < 0.2 and joint_arc[11] < 0.2 and joint_arc[12] < 0.2:
            # print("手指伸直状态", joint_arc[10], joint_arc[11], joint_arc[12])
            qpos[9] = 0
            qpos[10] = 0
            qpos[11] = 0
        elif joint_arc[10] > joint_arc[11] > joint_arc[12]:
            # print("2，3关节伸直状态", joint_arc[10],joint_arc[11],joint_arc[12])
            qpos[9] = (joint_arc[10] - 0.7) * 4
            qpos[10] = joint_arc[10] / 2
            qpos[11] = joint_arc[10] / 2
        else:
            if joint_arc[11] < 0.6 and joint_arc[10] < 0.5:
                qpos[9] = (joint_arc[10] - 0.7) * 4
                qpos[10] = joint_arc[10] / 2
                qpos[11] = joint_arc[10] / 2
            elif joint_arc[11] < joint_arc[10] < 0.6 and 0.5 > joint_arc[10] > joint_arc[12]:
                qpos[9] = (joint_arc[10] - 0.7) * 4
                qpos[10] = joint_arc[10] / 2
                qpos[11] = joint_arc[10] / 2
            else:
                qpos[9] = (joint_arc[10] - 0.7) * 4
                qpos[10] = joint_arc[12]
                qpos[11] = joint_arc[12]

        qpos[12] = 0
        if joint_arc[14] < 0.2 and joint_arc[15] < 0.2 and joint_arc[16] < 0.2:
            # print("手指伸直状态", joint_arc[14], joint_arc[15], joint_arc[16])
            qpos[13] = 0
            qpos[14] = joint_arc[14] / 2
            qpos[15] = joint_arc[14] / 2
        elif joint_arc[14] > joint_arc[15] > joint_arc[16]:
            # print("2，3关节伸直状态1", joint_arc[14],joint_arc[15],joint_arc[16])
            qpos[13] = (joint_arc[14] - 0.7) * 4
            qpos[14] = joint_arc[14] / 2
            qpos[15] = joint_arc[14] / 2
        else:
            if joint_arc[11] < joint_arc[10] < 0.9 and 0.9 > joint_arc[10] > joint_arc[12]:
                # print("2，3关节伸直状态2", joint_arc[14], joint_arc[15], joint_arc[16])
                qpos[13] = (joint_arc[14] - 0.7) * 4
                qpos[14] = joint_arc[14] / 2
                qpos[15] = joint_arc[14] / 2
            else:
                # print("2，3关节弯曲状态", joint_arc[14], joint_arc[15], joint_arc[16])
                qpos[13] = (joint_arc[14] - 0.7) * 4
                qpos[14] = joint_arc[16]
                qpos[15] = joint_arc[16]
        # if joint_arc[16]-0.2 > 0.4:
        #     qpos[14] = joint_arc[16]
        #     qpos[15] = joint_arc[16]
        # else:
        #     qpos[14] = 0
        #     qpos[15] = 0

        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slow_limit = 2
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.8)
            min_vel = int(self.last_jointvelocity[i] * 0.6)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 10 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 10 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
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
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 10 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity [i] = int(target_vel)

            if self.g_jointvelocity [i] > 255:
                self.g_jointvelocity [i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity [i]
            self.last_jointpositions[i] = self.g_jointpositions[i]