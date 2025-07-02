import numpy as np
from linkerhand.handcore import HandCore


class RightHand:
    def __init__(self, handcore: HandCore, length=17):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.handcore = handcore

    def joint_update(self, joint_arc):
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
        self.g_jointpositions = np.rad2deg(qpos)

    def speed_update(self):
        pass


class LeftHand:
    def __init__(self, handcore: HandCore, length=17):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.handcore = handcore

    def joint_update(self, joint_arc):
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
        self.g_jointpositions = np.rad2deg(qpos)

    def speed_update(self):
        pass