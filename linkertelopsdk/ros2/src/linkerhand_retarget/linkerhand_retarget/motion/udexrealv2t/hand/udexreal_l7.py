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
        self.l7_joint = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        #print("joint_arc[20]",joint_arc[20])
       
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
        
        print("A-------------")
        print(joint_arc[2])
        print(joint_arc[20])
        print(joint_arc[20])
        print(joint_arc[6])
        print(joint_arc[10])
        print(joint_arc[14])
        print(joint_arc[18])
        print("B-------------")
        
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.l7_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]

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
            # print(max_vel,mid_vel,min_vel)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 3 + 50
                    self.handstate[i] = 1
                elif position_error == 0:
                    target_vel = position_error * 3 + 30
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 30
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 70
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            # if position_derict == -1:
            #     target_vel = 2 * target_vel
            if i == 1 or i == 9:
                self.g_jointvelocity[i] =  int(target_vel)
            else:
                self.g_jointvelocity[i] =  int(target_vel * 2)
           
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
    
            self.last_jointpositions[i] = self.g_jointpositions[i]
            
    def glove_torch(self,msg):
        value = self.l7_joint[3:]
        print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.l7_joint[3], self.l7_joint[4], self.l7_joint[5], self.l7_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           if self.l7_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               if self.l7_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                      print("index_torch")
                      msg.position[6] = 92
                      msg.position[0] = 142
                      msg.position[1] = 92
                      msg.position[2] = 167
           
         if min_index == 1:
           print("middle--------")
           if self.l7_joint[4] < -1.3 * 0.5:
               print("middle_probability_more---------")
               if self.l7_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("middle_torch")
                      msg.position[6] = 62
                      msg.position[0] = 129
                      msg.position[1] = 85
                      msg.position[3] = 159
           
           
         if min_index == 2:
           print("ring----------")
           if self.l7_joint[5] < -1.3 * 0.5:
               print("ring_probability_more---------")
               if self.l7_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("ring_torch")
                      msg.position[6] = 28
                      msg.position[0] = 135
                      msg.position[1] = 100
                      msg.position[4] = 159
           
         if min_index == 3:
           print("little--------")
           if self.l7_joint[6] < -1 * 0.5:
               print("little_probability_more---------")
               if self.l7_joint[0] < 0:
                   print("little_probability_more_more---------")
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                      print("little_torch")
                      msg.position[6] = 17
                      msg.position[0] = 137
                      msg.position[1] = 85
                      msg.position[5] = 147       
           
               
    
             
       


class LeftHand:
    def __init__(self, handcore: HandCore, length=7):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length
        self.l7_joint = [0] * length

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
        self.l7_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]

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
            # print(max_vel,mid_vel,min_vel)
            target_vel = self.last_jointvelocity[i]
            if self.handstate[i] == 0:  # stop
                if 0 < position_error:
                    target_vel = position_error * 3 + 50
                    self.handstate[i] = 1
                elif position_error == 0:
                    target_vel = position_error * 3 + 30
            elif self.handstate[i] == 1:  # slow
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel > mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 2
                elif position_error == 0:
                    self.handstate[i] = 0
                else:
                    target_vel = position_error * 3 + 30
            else:  # fast
                if position_error >= fast_limit:
                    target_vel = position_error * 3 + 70
                    if target_vel > max_vel:
                        target_vel = max_vel
                elif slow_limit < position_error < fast_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel < mid_vel:
                        target_vel = mid_vel
                    self.handstate[i] = 3
                elif 0 < position_error <= slow_limit:
                    target_vel = position_error * 3 + 50
                    if target_vel < min_vel:
                        target_vel = min_vel
                    self.handstate[i] = 1
            # if position_derict == -1:
            #     target_vel = 2 * target_vel
            if i == 1 or i == 9:
                self.g_jointvelocity[i] =  int(target_vel)
            else:
                self.g_jointvelocity[i] =  int(target_vel * 2)
           
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
            
    def glove_torch(self,msg):
        value = self.l7_joint[3:]
        print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.l7_joint[3], self.l7_joint[4], self.l7_joint[5], self.l7_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           if self.l7_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               if self.l7_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                      print("index_torch")
                      msg.position[6] = 122
                      msg.position[0] = 142
                      msg.position[1] = 92
                      msg.position[2] = 167
           
         if min_index == 1:
           print("middle--------")
           if self.l7_joint[4] < -1.3 * 0.5:
               print("middle_probability_more---------")
               if self.l7_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("middle_torch")
                      msg.position[6] = 92
                      msg.position[0] = 129
                      msg.position[1] = 85
                      msg.position[3] = 159
           
           
         if min_index == 2:
           print("ring----------")
           if self.l7_joint[5] < -1.3 * 0.5:
               print("ring_probability_more---------")
               if self.l7_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                      print("ring_torch")
                      msg.position[6] = 58
                      msg.position[0] = 135
                      msg.position[1] = 100
                      msg.position[4] = 159
           
         if min_index == 3:
           print("little--------")
           if self.l7_joint[6] < -1 * 0.5:
               print("little_probability_more---------")
               if self.l7_joint[0] < 0:
                   print("little_probability_more_more---------")
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l7_joint[0], self.l7_joint[1], self.l7_joint[2], self.l7_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                      print("little_torch")
                      msg.position[6] = 47
                      msg.position[0] = 137
                      msg.position[1] = 85
                      msg.position[5] = 147       
           
                
