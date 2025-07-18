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

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[16] = joint_arc[20] * 1
        qpos[17] = joint_arc[20] * 2.570567
        qpos[18] = joint_arc[1] * -0.76688
        qpos[19] = joint_arc[0] * -1.231859

        qpos[0] = joint_arc[7] * 1
        qpos[1] = joint_arc[6] * -1
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1

        qpos[4] = joint_arc[19] * 1
        qpos[5] = joint_arc[18] * -1
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1

        qpos[8] = joint_arc[11] * -1
        qpos[9] = joint_arc[10] * -1
        qpos[10] = joint_arc[9] * -1
        qpos[11] = joint_arc[8] * -1

        qpos[12] = joint_arc[15] * 1
        qpos[13] = joint_arc[14] * -1
        qpos[14] = joint_arc[13] * -1
        qpos[15] = joint_arc[12] * -1
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            slow_rimit = 2
            fast_rimit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.8)
            min_vel = int(self.last_jointvelocity[i] * 0.6)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 10 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_rimit:
                    target_vel = position_error * 10 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 10 + 10
            else:  # fast
                if position_error >= fast_rimit:
                    target_vel = position_error * 10 + 50
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_rimit < position_error < fast_rimit:
                    target_vel = position_error * 10 + 30
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_rimit:
                    target_vel = position_error * 10 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity [i] = int(target_vel)

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

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[16] = joint_arc[20] * 1
        qpos[17] = joint_arc[20] * 2.570567
        qpos[18] = joint_arc[1] * -0.76688
        qpos[19] = joint_arc[0] * -1.231859

        qpos[0] = joint_arc[7] * 1
        qpos[1] = joint_arc[6] * -1
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1

        qpos[4] = joint_arc[19] * 1
        qpos[5] = joint_arc[18] * -1
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1

        qpos[8] = joint_arc[11] * 1
        qpos[9] = joint_arc[10] * -1
        qpos[10] = joint_arc[9] * -1
        qpos[11] = joint_arc[8] * -1

        qpos[12] = joint_arc[15] * 1
        qpos[13] = joint_arc[14] * -1
        qpos[14] = joint_arc[13] * -1
        qpos[15] = joint_arc[12] * -1

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