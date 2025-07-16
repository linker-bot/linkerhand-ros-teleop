import numpy as np
from linkerhand.handcore import HandCore

class RightHand:
    def __init__(self, handcore: HandCore, length=7):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[15] = joint_arc[20] * 2  # 侧摆
        qpos[16] = joint_arc[20] * 2.2144  # 旋转
        qpos[17] = joint_arc[2]  # 根部关节
        qpos[18] = joint_arc[1] * -10 # 中部关节
        qpos[19] = joint_arc[0] * -0.66845 # 远端关节
        
        qpos[0] = joint_arc[7]
        qpos[1] = joint_arc[6] * -1.0098
        
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1

        qpos[20] = joint_arc[11]
        qpos[8] = joint_arc[10] * -1.0098
        
        qpos[9] = joint_arc[9] * -1
        qpos[10] = joint_arc[8] * -1

        qpos[11] = joint_arc[15]
        qpos[12] = joint_arc[14] * -1.0098
        
        qpos[13] = joint_arc[13] * -1
        qpos[14] = joint_arc[12] * -1
        
        qpos[4] = joint_arc[19]
        qpos[5] = joint_arc[18] * -1.077160
        
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)

        O7_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
        self.glove_torch(O7_joint)

    def glove_torch(self,joint):
        O7_joint = joint
        value = O7_joint[3:]
        print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([O7_joint[3], O7_joint[4], O7_joint[5], O7_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           if O7_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               if O7_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                      print("index_torch")
                      self.g_jointpositions[6] = 92
                      self.g_jointpositions[0] = 142
                      self.g_jointpositions[1] = 92
                      self.g_jointpositions[2] = 167
           
         if min_index == 1:
           print("middle--------")
           if O7_joint[4] < -1.3 * 0.5:
               print("middle_probability_more---------")
               if O7_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("middle_torch")
                      self.g_jointpositions[6] = 62
                      self.g_jointpositions[0] = 129
                      self.g_jointpositions[1] = 85
                      self.g_jointpositions[3] = 159
           
           
         if min_index == 2:
           print("ring----------")
           if O7_joint[5] < -1.3 * 0.5:
               print("ring_probability_more---------")
               if O7_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("ring_torch")
                      self.g_jointpositions[6] = 28
                      self.g_jointpositions[0] = 135
                      self.g_jointpositions[1] = 100
                      self.g_jointpositions[4] = 159
           
         if min_index == 3:
           print("little--------")
           if O7_joint[6] < -1 * 0.5:
               print("little_probability_more---------")
               if O7_joint[0] < 0:
                   print("little_probability_more_more---------")
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                      print("little_torch")
                      self.g_jointpositions[6] = 17
                      self.g_jointpositions[0] = 137
                      self.g_jointpositions[1] = 85
                      self.g_jointpositions[5] = 147       
           
         grape_pose = [-0.2321,0.2024,0.2024,-0.7958,-0.8936,-1.1414,-0.1308]
         weights_grape_pose = [50, 50, 50, 30,30,30,30]
         weighted_diff_grape_pose = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[3], O7_joint[4], O7_joint[5], O7_joint[6]], grape_pose, weights_grape_pose)]
         threshold_grape_pose = np.sqrt(np.sum(np.square(weighted_diff_grape_pose)))
         print("threshold_grape_pose",threshold_grape_pose)
         if  threshold_grape_pose < 20 + 20 * 0.5 and threshold_grape_pose > 20 - 20 * 0.5:
             self.g_jointpositions[6] = 88 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[6] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[0] = 155 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[0] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[1] = 76 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[1] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[2] = 168 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[2] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[3] = 156 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[4] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[4] = 141 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[5] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[5] = 156 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[6] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)        
    
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
                    target_vel = position_error * 14 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 28 + 120
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 28 + 60
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 28 + 120
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 28 + 80
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 28 + 40
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


class LeftHand:
    def __init__(self, handcore: HandCore, length=7):
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
        O7_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
        self.glove_torch(O7_joint)

    def glove_torch(self,joint):
        O7_joint = joint
        value = O7_joint[3:]
        print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([O7_joint[3], O7_joint[4], O7_joint[5], O7_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           if O7_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               if O7_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                      print("index_torch")
                      self.g_jointpositions[6] = 92
                      self.g_jointpositions[0] = 142
                      self.g_jointpositions[1] = 92
                      self.g_jointpositions[2] = 167
           
         if min_index == 1:
           print("middle--------")
           if O7_joint[4] < -1.3 * 0.5:
               print("middle_probability_more---------")
               if O7_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("middle_torch")
                      self.g_jointpositions[6] = 62
                      self.g_jointpositions[0] = 129
                      self.g_jointpositions[1] = 85
                      self.g_jointpositions[3] = 159
           
           
         if min_index == 2:
           print("ring----------")
           if O7_joint[5] < -1.3 * 0.5:
               print("ring_probability_more---------")
               if O7_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("ring_torch")
                      self.g_jointpositions[6] = 28
                      self.g_jointpositions[0] = 135
                      self.g_jointpositions[1] = 100
                      self.g_jointpositions[4] = 159
           
         if min_index == 3:
           print("little--------")
           if O7_joint[6] < -1 * 0.5:
               print("little_probability_more---------")
               if O7_joint[0] < 0:
                   print("little_probability_more_more---------")
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                      print("little_torch")
                      self.g_jointpositions[6] = 17
                      self.g_jointpositions[0] = 137
                      self.g_jointpositions[1] = 85
                      self.g_jointpositions[5] = 147       
           
         grape_pose = [-0.2321,0.2024,0.2024,-0.7958,-0.8936,-1.1414,-0.1308]
         weights_grape_pose = [50, 50, 50, 30,30,30,30]
         weighted_diff_grape_pose = [(j - p) * w for j, p, w in zip([O7_joint[0], O7_joint[1], O7_joint[2], O7_joint[3], O7_joint[4], O7_joint[5], O7_joint[6]], grape_pose, weights_grape_pose)]
         threshold_grape_pose = np.sqrt(np.sum(np.square(weighted_diff_grape_pose)))
         print("threshold_grape_pose",threshold_grape_pose)
         if  threshold_grape_pose < 20 + 20 * 0.5 and threshold_grape_pose > 20 - 20 * 0.5:
             self.g_jointpositions[6] = 88 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[6] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[0] = 155 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[0] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[1] = 76 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[1] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[2] = 168 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[2] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[3] = 156 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[4] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[4] = 141 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[5] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)
             self.g_jointpositions[5] = 156 * (20 - abs(threshold_grape_pose - 20)) / 20 + self.g_jointpositions[6] * (1 - (20 - abs(threshold_grape_pose - 20)) / 20)        

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
                    target_vel = position_error * 14 + 30
                    self.handstate[i] = 1
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 28 + 120
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 28 + 60
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 28 + 120
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 28 + 80
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 28 + 40
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