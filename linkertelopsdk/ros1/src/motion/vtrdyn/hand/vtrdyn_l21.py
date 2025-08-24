import numpy as np
from linkerhand.handcore import HandCore


class RightHand:
    def __init__(self, handcore: HandCore, length=25):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)

        qpos[16] = joint_arc[0]
        qpos[17] = joint_arc[1] * 0.5
        qpos[18] = joint_arc[2] * 0.5
        qpos[19] = joint_arc[3]
        qpos[20] = joint_arc[4]

        qpos[0] = joint_arc[5] * 1.2
        qpos[1] = joint_arc[7]
        qpos[2] = joint_arc[8]
        qpos[3] = joint_arc[9]

        qpos[4] = joint_arc[20] * -1.2
        qpos[5] = joint_arc[22]
        qpos[6] = joint_arc[23]
        qpos[7] = joint_arc[24]

        qpos[8] = joint_arc[10] 
        qpos[9] = joint_arc[12]
        qpos[10] = joint_arc[13]
        qpos[11] = joint_arc[14]

        qpos[12] = joint_arc[15] * -1.5
        qpos[13] = joint_arc[17]
        qpos[14] = joint_arc[18]
        qpos[15] = joint_arc[19]
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
                    target_vel = position_error * 3 + 50
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 100
                    if target_vel > max_vel:
                        target_vel = max_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 200
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
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[16] = joint_arc[0]
        qpos[17] = joint_arc[1] * 0.6
        qpos[18] = joint_arc[2] * 0.5
        qpos[19] = joint_arc[4]
        qpos[20] = joint_arc[4]

        qpos[0] = joint_arc[5] * 1.2
        qpos[1] = joint_arc[7] *1.1
        qpos[2] = joint_arc[8] *1.1
        qpos[3] = joint_arc[9]

        qpos[4] = joint_arc[20] * -1.2
        qpos[5] = joint_arc[22] *1.1
        qpos[6] = joint_arc[23] *1.1
        qpos[7] = joint_arc[24]

        qpos[8] = joint_arc[10] 
        qpos[9] = joint_arc[12] *1.1
        qpos[10] = joint_arc[13] *1.1
        qpos[11] = joint_arc[14]

        qpos[12] = joint_arc[15] * -1.5
        qpos[13] = joint_arc[17] *1.1
        qpos[14] = joint_arc[18] *1.1
        qpos[15] = joint_arc[19]
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
                    target_vel = position_error * 3 + 100
                    if target_vel > max_vel:
                        target_vel = max_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 200
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
            self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
        self.g_jointvelocity[6] = 255
        self.g_jointvelocity[7] = 255
        self.g_jointvelocity[8] = 255
        self.g_jointvelocity[9] = 255