import numpy as np
from linkerhand.handcore import HandCore


class RightHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[16] = joint_arc[0] * 0.5
        qpos[17] = joint_arc[1] * 0.4
        qpos[18] = joint_arc[2] * 1.5
        qpos[19] = joint_arc[3] 
        qpos[20] = joint_arc[4]
        
        qpos[0] = joint_arc[5] * -1.2
        qpos[1] = joint_arc[7]
        qpos[2] = joint_arc[8]
        qpos[3] = joint_arc[9]

        qpos[4] = joint_arc[20] * 2
        qpos[5] = joint_arc[22]
        qpos[6] = joint_arc[23]
        qpos[7] = joint_arc[24]

        qpos[8] = joint_arc[10]
        qpos[9] = joint_arc[12]
        qpos[10] = joint_arc[13]
        qpos[11] = joint_arc[14]

        qpos[12] = joint_arc[15] * 1.5
        qpos[13] = joint_arc[17]
        qpos[14] = joint_arc[18]
        qpos[15] = joint_arc[19]
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


class LeftHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[16] = joint_arc[0] * 0.3
        qpos[17] = joint_arc[1] * 0.8
        qpos[18] = joint_arc[2] * 0.5
        qpos[19] = joint_arc[4] * 1
        qpos[20] = joint_arc[4]

        qpos[0] = joint_arc[5] * -1.2
        qpos[1] = joint_arc[7] * 0.7
        qpos[2] = joint_arc[8] * 0.6
        qpos[3] = joint_arc[9]

        qpos[4] = joint_arc[20] * 2
        qpos[5] = joint_arc[22] * 0.7
        qpos[6] = joint_arc[23] * 0.6
        qpos[7] = joint_arc[24]

        qpos[8] = joint_arc[10]
        qpos[9] = joint_arc[12] * 0.7
        qpos[10] = joint_arc[13] * 0.6
        qpos[11] = joint_arc[14]

        qpos[12] = joint_arc[15] * 1.5
        qpos[13] = joint_arc[17] * 0.7
        qpos[14] = joint_arc[18] * 0.6
        qpos[15] = joint_arc[19]

        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 4
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 5 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                    target_vel = position_error * 5 + 100
                else:
                    target_vel = position_error * 5 + 100
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 90
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 5 + 60
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 5 + 40
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.g_jointvelocity[i] = 255    
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]