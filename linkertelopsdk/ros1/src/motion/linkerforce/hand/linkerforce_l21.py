import numpy as np
from linkerhand.handcore import HandCore


class RightHand():
    def __init__(self, handcore: HandCore, length=25):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.thumb_diff = [0] * 5
        self.index_diff = [0] * 3
        self.middle_diff = [0] * 3
        self.ring_diff = [0] * 3
        self.pinky_diff = [0] * 3
        self.fourroll_diff = [0] * 6

    def get_joint_diff(self,normal,open,close):
        all_diff = np.array(close) - np.array(open)
        self.thumb_diff = all_diff[:5]
        self.index_diff = all_diff[6:9]
        self.middle_diff = all_diff[10:13]
        self.ring_diff = all_diff[14:17]
        self.pinky_diff = all_diff[18:21]
        fourroll_index = np.array([1,2,5,9,13,17])
        fourroll_diff_array = np.array(open) - np.array(normal)
        self.fourroll_diff = fourroll_diff_array[fourroll_index]   

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

        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slow_limit = 1
            fast_limit = 5
            max_vel = int(self.last_jointvelocity[i] * 1.5)
            min_vel = int(self.last_jointvelocity[i] * 0.995)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 6 + 50
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 6 + 100
                    if target_vel > max_vel:
                        target_vel = max_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 6 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 10 + 100
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 6 + 50
                    if target_vel < min_vel:
                        target_vel = min_vel
                else:
                    target_vel = min_vel
                    if min_vel < 150:
                        self.handstate[i] = 1
            self.g_jointvelocity[i] = int(255)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
        self.g_jointvelocity[6] = 255
        self.g_jointvelocity[7] = 255
        self.g_jointvelocity[8] = 255
        self.g_jointvelocity[9] = 255


class LeftHand:
    def __init__(self, handcore: HandCore, length=25):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.isfirst = True
        self.calibrationnormal = None
        self.calibrationopen = None
        self.calibrationclose = None
        self.calibrationindexclose = None
        self.calibrationmiddleclose = None
        self.calibrationringclose = None
        self.calibrationpinkyclose = None
        self.fourroll_index = np.array([1,2,5,9,13,17])     


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
        joint_diff = np.array(joint_arc) - np.array(self.calibrationopen)
        fourroll_diff_array = np.array(joint_arc) - np.array(self.calibrationnormal)
        fourroll_diff = fourroll_diff_array[self.fourroll_index]  
        qpos[0] = fourroll_diff[2] * 1.8
        qpos[4] = fourroll_diff[5] * 0.5
        qpos[8] = fourroll_diff[3]
        qpos[12] = fourroll_diff[4] *0.5

        index_diff = joint_diff[6:9]
        qpos[1] = index_diff[0]
        if index_diff[0] < 0.2 and index_diff[1] < 0.2 and index_diff[2] < 0.2:
            # print("手指伸直状态", joint_arc[6], joint_arc[7], joint_arc[8])
            qpos[2] = 0
            qpos[3] = 0
        elif index_diff[0] > index_diff[2]:
            # print("2，3关节伸直状态", joint_arc[6],joint_arc[7],joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[0] = qpos[0] / 2
            qpos[2] = index_diff[2] / 4 
            qpos[3] = index_diff[2] / 4 
        else:
            # print("2，3关节弯曲状态", joint_arc[6], joint_arc[7], joint_arc[8],(joint_arc[6] - 0.6) * 4)
            qpos[0] = qpos[0] / 4
            qpos[2] = index_diff[2]
            qpos[3] = index_diff[2]

        pinky_diff = joint_diff[10:13]
        qpos[5] = pinky_diff[0]
        if pinky_diff[0] < 0.2 and pinky_diff[1] < 0.2 and pinky_diff[2] < 0.2:
            qpos[6] = 0
            qpos[7] = 0
        elif pinky_diff[0] > pinky_diff[2]:
            qpos[4] = qpos[4] / 2
            qpos[6] = pinky_diff[2] / 4
            qpos[7] = pinky_diff[2] / 4
        else:
            qpos[4] = qpos[4] / 4
            qpos[6] = pinky_diff[2]
            qpos[7] = pinky_diff[2]

        middle_diff = joint_diff[10:13]
        qpos[9] = middle_diff[0]
        if middle_diff[0] < 0.2 and middle_diff[1] < 0.2 and middle_diff[2] < 0.2:
            qpos[10] = 0
            qpos[11] = 0
        elif middle_diff[0] > middle_diff[2]:
            qpos[8] = qpos[8] / 2
            qpos[10] = middle_diff[2] / 2
            qpos[11] = middle_diff[2] / 2
        else:
            qpos[8] = qpos[8] / 2
            qpos[10] = middle_diff[2] 
            qpos[11] = middle_diff[2]

        ring_diff = joint_diff[14:17]
        qpos[13] = ring_diff[0]
        if ring_diff[0] < 0.2 and ring_diff[1] < 0.2 and ring_diff[2] < 0.2:
            # print("手指伸直状态", ring_diff)
            qpos[14] = 0
            qpos[15] = 0
        elif ring_diff[0] > ring_diff[2] and ring_diff[0] < 1:
            # print("2，3关节伸直状态", ring_diff)
            qpos[12] = qpos[12] / 2
            qpos[14] = ring_diff[2]
            qpos[15] = ring_diff[2]
        else:
            # print("2，3关节弯曲状态", ring_diff)
            qpos[12] = qpos[12] / 4
            qpos[14] = ring_diff[2] * 1.5
            qpos[15] = ring_diff[2] * 1.5

        thumb_diff = joint_diff[:5]
        # print()
        qpos[16] = (thumb_diff[1] - 0.25) * -1.5
        qpos[17] = (thumb_diff[1] - 0.25) * -2.5
        qpos[19] = 0
        qpos[20] = 0
        if thumb_diff[2] < 0.2 and thumb_diff[3] < 0.2 and thumb_diff[4] < 0.2:
            # print("手指伸直状态", thumb_diff)
            qpos[18] = 0
            qpos[19] = 0
        elif thumb_diff[2] > thumb_diff[4]:
            # print("1关节弯曲状态", thumb_diff)
            qpos[18] = thumb_diff[2] * 0.3
            qpos[19] = thumb_diff[2] * 1
            qpos[20] = thumb_diff[4] * 1.5
        else:
            # print("3关节弯曲状态", thumb_diff)
            qpos[18] = thumb_diff[2] * 0.3
            qpos[19] = thumb_diff[2] * 1
            qpos[20] = thumb_diff[4] * 1.5

        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slow_limit = 1
            fast_limit = 5
            max_vel = int(self.last_jointvelocity[i] * 1.5)
            min_vel = int(self.last_jointvelocity[i] * 0.995)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 3 + 50
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 70
                    if target_vel > max_vel:
                        target_vel = max_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 150
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 3 + 200
                    if target_vel < min_vel:
                        target_vel = min_vel
                else:
                    target_vel = min_vel
                    if min_vel < 150:
                        self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
        self.g_jointvelocity[6] = 255
        self.g_jointvelocity[7] = 255
        self.g_jointvelocity[8] = 255
        self.g_jointvelocity[9] = 255
