import numpy as np


class RightHand:
    def __init__(self, length=25):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length

    @staticmethod
    def joint_update(joint_arc) -> np.ndarray:
        joint_degreeranges = [
                            [0, 40],       # 大拇指侧摆
                            [0, 80],       # 拇指旋转
                            [0, 70],       # 拇指弯曲
                            [0, 60],       # 拇指指尖
                            [-10, 10],     # 食指侧摆
                            [0, 80],       # 食指指根弯曲
                            [0, 80],       # 食指指尖
                            [-10, 10],     # 中指侧摆
                            [0, 80],       # 中指指根
                            [0, 80],       # 中指指尖
                            [-10, 10],     # 无名指侧摆
                            [0, 80],       # 无名指指根
                            [0, 80],       # 无名指指尖
                            [-10, 10],     # 小指侧摆
                            [0, 80],       # 小指指根
                            [0, 80],       # 小指指尖
                            [-50, 50]      # 手腕
                        ]
        degree_array = np.array(joint_degreeranges)
        radian_ranges = np.deg2rad(degree_array)
        print(radian_ranges)


        qpos = np.zeros(17)
        qpos[0] = joint_arc[20]
        qpos[1] = joint_arc[20] * 2
        qpos[2] = joint_arc[2] * -1
        qpos[3] = joint_arc[0] * -1

        qpos[4] = joint_arc[7] * -1
        qpos[5] = joint_arc[6] * -1
        qpos[6] = joint_arc[4] * -1

        qpos[7] = joint_arc[11] * -1
        qpos[8] = joint_arc[10] * -1
        qpos[9] = joint_arc[8] * -1

        qpos[10] = joint_arc[15] * -1
        qpos[11] = joint_arc[14] * -1
        qpos[12] = joint_arc[12] * -1

        qpos[13] = joint_arc[19] * -1
        qpos[14] = joint_arc[18] * -1
        qpos[15] = joint_arc[16] * -1

        qpos[16] = 0
        # qpos
        
        # qpos[15] = joint_arc[3] * -2.5  # 侧摆
        # qpos[16] = joint_arc[20] * -2.6  # 旋转
        # qpos[17] = joint_arc[2] * -0.2  # 根部关节
        # qpos[18] = joint_arc[1] * -1.5  # 中部关节
        # qpos[19] = joint_arc[0] * -1.5  # 远端关节

        # qpos[0] = joint_arc[7]
        # qpos[1] = joint_arc[6] * -1
        # qpos[2] = joint_arc[5] * -1
        # qpos[3] = joint_arc[4] * -1
        # if joint_arc[6] > -80 * 3.14 / 180: qpos[2] = 0
        # if joint_arc[6] > -80 * 3.14 / 180: qpos[3] = 0

        # qpos[4] = joint_arc[19]
        # qpos[5] = joint_arc[18] * -1
        # qpos[6] = joint_arc[17] * -1
        # qpos[7] = joint_arc[16] * -1
        # if joint_arc[18] > -75 * 3.14 / 180: qpos[6] = 0
        # if joint_arc[18] > -75 * 3.14 / 180: qpos[7] = 0

        # qpos[20] = joint_arc[11]
        # qpos[8] = joint_arc[10] * -1
        # qpos[9] = joint_arc[9] * -1
        # qpos[10] = joint_arc[8] * -1
        # if joint_arc[10] > -80 * 3.14 / 180: qpos[9] = 0
        # if joint_arc[10] > -80 * 3.14 / 180: qpos[10] = 0

        # qpos[11] = joint_arc[15]
        # qpos[12] = joint_arc[14] * -1
        # qpos[13] = joint_arc[13] * -1
        # qpos[14] = joint_arc[12] * -1
        # if joint_arc[14] > -80 * 3.14 / 180: qpos[13] = 0
        # if joint_arc[14] > -80 * 3.14 / 180: qpos[14] = 0
        return qpos

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
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
        self.g_jointvelocity[6] = 255
        self.g_jointvelocity[7] = 255
        self.g_jointvelocity[8] = 255
        self.g_jointvelocity[9] = 255


class LeftHand:
    def __init__(self, length=25):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length

    @staticmethod
    def joint_update(joint_arc) -> np.ndarray:
        qpos = np.zeros(25)
        qpos[15] = joint_arc[3] * 2.5  # 侧摆
        qpos[16] = joint_arc[20] * 2.6  # 旋转
        qpos[17] = joint_arc[2] * 0.2  # 根部关节
        qpos[18] = joint_arc[1] * 1.5  # 中部关节
        qpos[19] = joint_arc[0] * 1.5  # 远端关节

        # 食指 index
        qpos[0] = joint_arc[7] * -1
        qpos[1] = joint_arc[6] * -1
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1
        if joint_arc[6] > -80 * 3.14 / 180: qpos[2] = 0
        if joint_arc[6] > -80 * 3.14 / 180: qpos[3] = 0

        # 小指 little
        qpos[4] = joint_arc[19] * -1
        qpos[5] = joint_arc[18] * -1
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1
        if joint_arc[18] > -75 * 3.14 / 180: qpos[6] = 0
        if joint_arc[18] > -75 * 3.14 / 180: qpos[7] = 0

        # 中指 middle
        qpos[8] = joint_arc[10] * -2
        qpos[9] = joint_arc[9] * -1
        qpos[10] = joint_arc[8] * -1
        if joint_arc[10] > -80 * 3.14 / 180: qpos[9] = 0
        if joint_arc[10] > -80 * 3.14 / 180: qpos[10] = 0

        # 无名指 ring
        qpos[11] = joint_arc[15] * -1
        qpos[12] = joint_arc[14] * -1  # gen
        qpos[13] = joint_arc[13] * -1  # zhong
        qpos[14] = joint_arc[12] * -1
        if joint_arc[14] > -80 * 3.14 / 180: qpos[13] = 0
        if joint_arc[14] > -80 * 3.14 / 180: qpos[14] = 0

        return qpos

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
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
        self.g_jointvelocity[6] = 255
        self.g_jointvelocity[7] = 255
        self.g_jointvelocity[8] = 255
        self.g_jointvelocity[9] = 255