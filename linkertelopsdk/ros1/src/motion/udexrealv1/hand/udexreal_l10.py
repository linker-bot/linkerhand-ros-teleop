import numpy as np
from linkerhand.handcore import HandCore


class RightHand:
    def __init__(self, length=10):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[15] = joint_arc[20] * 1  # 侧摆
        qpos[16] = joint_arc[20] * 2.2144  # 旋转
        qpos[17] = joint_arc[2] * -0.3878  # 根部关节
        qpos[18] = joint_arc[1] * -0.66845 # 中部关节
        qpos[19] = joint_arc[0] * -0.66845 # 远端关节

        qpos[0] = joint_arc[7]
        qpos[1] = joint_arc[6] * -1.0098
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1

        qpos[4] = joint_arc[19]
        qpos[5] = joint_arc[18] * -1.077160
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1

        qpos[20] = joint_arc[11]
        qpos[8] = joint_arc[10] * -1.0098
        qpos[9] = joint_arc[9] * -1
        qpos[10] = joint_arc[8] * -1

        qpos[11] = joint_arc[15]
        qpos[12] = joint_arc[14] * -1.0098
        qpos[13] = joint_arc[13] * -1
        qpos[14] = joint_arc[12] * -1
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 2
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 3 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 5 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 10
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 3 + 20
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 3 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            if i == 0 or i == 1 or i == 9:
                target_vel = 25
            self.g_jointvelocity[i] = int(target_vel)

            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


class LeftHand:
    def __init__(self, length=10):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[15] = joint_arc[20] * 1  # 侧摆
        qpos[16] = joint_arc[20] * 2.2144 # 旋转
        qpos[17] = joint_arc[2] * -0.5878  # 根部关节
        qpos[18] = joint_arc[1] * -0.66845  # 中部关节
        qpos[19] = joint_arc[0] * -0.66845  # 远端关节

        # 食指 index
        qpos[0] = joint_arc[7] * -1
        qpos[1] = joint_arc[6] * -1.0098
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1

        # 小指 little
        qpos[4] = joint_arc[19] * -1
        qpos[5] = joint_arc[18] * -1.077160
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1

        # 中指 middle
        qpos[8] = joint_arc[10] * -1.0098
        qpos[9] = joint_arc[9] * -1
        qpos[10] = joint_arc[8] * -1

        # 无名指 ring
        qpos[11] = joint_arc[15] * -1
        qpos[12] = joint_arc[14] * -1.0098  # gen
        qpos[13] = joint_arc[13] * -1  # zhong
        qpos[14] = joint_arc[12] * -1

        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)

    def speed_update(self):
        for i in range(len(self.g_jointpositions)):
            lastpos = self.last_jointpositions[i]
            position_error = int(abs(self.g_jointpositions[i] - lastpos))
            position_derict = 1 if self.g_jointpositions[i] - lastpos > 0 else -1
            slow_limit = 2
            fast_limit = 10
            max_vel = int(self.last_jointvelocity[i] * 2)
            mid_vel = int(self.last_jointvelocity[i] * 0.7)
            min_vel = int(self.last_jointvelocity[i] * 0.5)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 3 + 5
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 30
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 10
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 3 + 20
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 3 + 10
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            if position_derict == -1:
                target_vel = 2 * target_vel
            if i == 0 or i == 1 or i == 9:
                target_vel = 25
            self.g_jointvelocity[i] = int(target_vel * 1)
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]