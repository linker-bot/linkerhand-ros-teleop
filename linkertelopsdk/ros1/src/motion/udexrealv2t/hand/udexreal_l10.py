import numpy as np
from linkerhand.handcore import HandCore


class RightHand:
    def __init__(self, handcore: HandCore, length=10):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.L10_joint = [0] * length
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
        self.L10_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
        '''
        print("A-------------")
        print(joint_arc[2])
        print(joint_arc[20])
        print(joint_arc[20])
        print(joint_arc[6])
        print(joint_arc[10])
        print(joint_arc[14])
        print(joint_arc[18])
        print("B-------------")
        '''
        

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
            
   

    def glove_torch(self,msg):
        distance_index_thumb = abs(self.L10_joint[0] + self.L10_joint[3])
        distance_middle_thumb = abs(self.L10_joint[0] + self.L10_joint[4])
        distance_ring_thumb = abs(self.L10_joint[0] + self.L10_joint[5])
        distance_little_thumb = abs(self.L10_joint[0] + self.L10_joint[6])
        distance = [distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb]
        print("distance_index_thumb_right_l10",abs(self.L10_joint[0] + self.L10_joint[3]))
        print("distance_middle_thumb_right_l10",abs(self.L10_joint[0] + self.L10_joint[4]))
        print("distance_ring_thumb_right_l10",abs(self.L10_joint[0] + self.L10_joint[5]))
        print("distance_little_thumb_right_l10",abs(self.L10_joint[0] + self.L10_joint[6]))
        
        value = self.L10_joint[3:]
        #print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.L10_joint[3], self.L10_joint[4], self.L10_joint[5], self.L10_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        grape_pose = [0.694,0.923,1.38,0.35]
        weights_grape_pose = [30,30,30,10]
        weighted_diff_grape_pose = [(j - p) * w for j, p, w in zip([distance[0], distance[1],distance[2],distance[3]],grape_pose, weights_grape_pose)]
        threshold_grape_pose = np.sqrt(np.sum(np.square(weighted_diff_grape_pose)))
        print("threshold_grape_pose",threshold_grape_pose)
        #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.L10_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 and self.L10_joint[1] > 0.1:
                     #if distance_index_thumb > 1.4:
                      print("index_torch")
                      msg.position[0] = 151
                      msg.position[1] = 179
                      msg.position[2] = 149
                      msg.position[9] = 63
                      
                      #msg = msg
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2] 
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
                      
                      
           if self.L10_joint[3] > -1.3 * 0.5 and self.L10_joint[3] < -0.2:
               print("index_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("index_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   index_torch_thumb_pose_straight = [-0.0837,0.129,0.129,-0.28]
                   weights_index_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[3]], index_torch_thumb_pose_straight, weights_index_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_index_thumb > 0.5:
                      '''
                      msg.position[0] = 60  
                      msg.position[1] = 0
                      msg.position[5] = 190
                      msg.position[6] = 33
                      msg.position[10] = 9
                      msg.position[15] = 255
                      msg.position[16] = 133
                      '''
                      pass
         if min_index == 1:
           print("middle--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.L10_joint[4] < -1.3 * 0.5:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("middle_probability_more---------")
               if self.L10_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    
                      print("middle_torch")
                      msg.position[0] = 147
                      msg.position[1] = 105
                      msg.position[3] = 126
                      msg.position[9] = 66
                      
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3] 
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
           if self.L10_joint[4] > -1.3 * 0.5 and self.L10_joint[4] < -0.2:
               print("middle_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("middle_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose_straight = [-0.261,0.218,0.218,-0.642]
                   weights_middle_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[4]], middle_torch_thumb_pose_straight, weights_middle_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 25:
                    if distance_middle_thumb > 0.6:
                      ''' 
                      msg.position[0] = 54  
                      msg.position[2] = 0
                      msg.position[5] = 141
                      msg.position[7] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[17] = 176
                      '''
                      pass
         if min_index == 2 and self.L10_joint[1] > 0.3:
           print("ring----------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.L10_joint[5] < -1.3 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("ring_probability_more---------")
               if self.L10_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    
                      print("ring_torch")
                      msg.position[0] = 135
                      msg.position[1] = 142
                      msg.position[4] = 138
                      msg.position[9] = 38
                      
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4] 
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
           if self.L10_joint[5] > -1.3 * 0.8 and self.L10_joint[5] < -0.2:
               print("ring_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("ring_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose_straight = [-0.642,0.408,0.408,-0.780]
                   weights_ring_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[5]], ring_torch_thumb_pose_straight, weights_ring_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_ring_thumb > 1:
                      print("ring_torch")
                      '''
                      msg.position[0] = 68  
                      msg.position[3] = 0
                      msg.position[5] = 113
                      msg.position[8] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[18] = 175
                      '''
                      pass
         if min_index == 3 :
           print("little--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.L10_joint[6] < -1 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("little_probability_more---------")
               if self.L10_joint[0] < 0:
                   print("little_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                    
                      print("little_torch")
                      msg.position[0] = 135
                      msg.position[1] = 66
                      msg.position[5] = 128
                      msg.position[9] = 37
                      
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5] 
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
                      
           if self.L10_joint[6] > -1 * 0.8 and self.L10_joint[6] < -0.2:
               print("little_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("little_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose_straight = [-0.99,0.582,0.582,-0.068]
                   weights_little_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[6]], little_torch_thumb_pose_straight, weights_little_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                      print("little_torch")
                      '''
                      msg.position[0] = 100  
                      msg.position[4] = 29
                      msg.position[5] = 79
                      msg.position[9] = 183
                      msg.position[10] = 0
                      msg.position[15] = 193
                      msg.position[19] = 166
                      '''  
                      pass 
        

class LeftHand:
    def __init__(self, handcore: HandCore, length=10):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.L10_joint = [0] * length
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
        self.L10_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
        print("left_l10")
        print("A-------------")
        print(joint_arc[2])
        print(joint_arc[20])
        print(joint_arc[20])
        print(joint_arc[6])
        print(joint_arc[10])
        print(joint_arc[14])
        print(joint_arc[18])
        print("B-------------")
        
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
    
        
            
    def glove_torch(self,msg):
        distance_index_thumb = abs(self.L10_joint[0] + self.L10_joint[3])
        distance_middle_thumb = abs(self.L10_joint[0] + self.L10_joint[4])
        distance_ring_thumb = abs(self.L10_joint[0] + self.L10_joint[5])
        distance_little_thumb = abs(self.L10_joint[0] + self.L10_joint[6])
        distance = [distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb]
        print("distance_index_thumb_left_l10",abs(self.L10_joint[0] + self.L10_joint[3]))
        print("distance_middle_thumb_left_l10",abs(self.L10_joint[0] + self.L10_joint[4]))
        print("distance_ring_thumb_left_l10",abs(self.L10_joint[0] + self.L10_joint[5]))
        print("distance_little_thumb_left_l10",abs(self.L10_joint[0] + self.L10_joint[6]))
        
        value = self.L10_joint[3:]
        #print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.L10_joint[3], self.L10_joint[4], self.L10_joint[5], self.L10_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        grape_pose = [0.694,0.923,1.38,0.35]
        weights_grape_pose = [30,30,30,10]
        weighted_diff_grape_pose = [(j - p) * w for j, p, w in zip([distance[0], distance[1],distance[2],distance[3]],grape_pose, weights_grape_pose)]
        threshold_grape_pose = np.sqrt(np.sum(np.square(weighted_diff_grape_pose)))
        print("threshold_grape_pose",threshold_grape_pose)
        #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.L10_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 and self.L10_joint[1] > 0.1:
                     #if distance_index_thumb > 1.4:
                      print("index_torch")
                      msg.position[0] = 151
                      msg.position[1] = 179
                      msg.position[2] = 149
                      msg.position[9] = 63
                      
                      #msg = msg
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2] 
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9]  
                      
                      
                      pass   
           if self.L10_joint[3] > -1.3 * 0.5 and self.L10_joint[3] < -0.2:
               print("index_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("index_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   index_torch_thumb_pose_straight = [-0.0837,0.129,0.129,-0.28]
                   weights_index_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[3]], index_torch_thumb_pose_straight, weights_index_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_index_thumb > 0.5:
                      '''
                      msg.position[0] = 60  
                      msg.position[1] = 0
                      msg.position[5] = 190
                      msg.position[6] = 33
                      msg.position[10] = 9
                      msg.position[15] = 255
                      msg.position[16] = 133
                      '''
                      pass
         if min_index == 1:
           print("middle--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.L10_joint[4] < -1.3 * 0.5:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("middle_probability_more---------")
               if self.L10_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    
                      print("middle_torch")
                      msg.position[0] = 147
                      msg.position[1] = 105
                      msg.position[3] = 126
                      msg.position[9] = 66
                      
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3] 
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
           if self.L10_joint[4] > -1.3 * 0.5 and self.L10_joint[4] < -0.2:
               print("middle_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("middle_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose_straight = [-0.261,0.218,0.218,-0.642]
                   weights_middle_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[4]], middle_torch_thumb_pose_straight, weights_middle_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 25:
                    if distance_middle_thumb > 0.6:
                      ''' 
                      msg.position[0] = 54  
                      msg.position[2] = 0
                      msg.position[5] = 141
                      msg.position[7] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[17] = 176
                      '''
                      pass
         if min_index == 2 and self.L10_joint[1] > 0.3:
           print("ring----------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
          
           if self.L10_joint[5] < -1.3 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("ring_probability_more---------")
               if self.L10_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    
                      print("ring_torch")
                      msg.position[0] = 135
                      msg.position[1] = 142
                      msg.position[4] = 138
                      msg.position[9] = 38
                      
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
           if self.L10_joint[5] > -1.3 * 0.8 and self.L10_joint[5] < -0.2:
               print("ring_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("ring_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose_straight = [-0.642,0.408,0.408,-0.780]
                   weights_ring_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[5]], ring_torch_thumb_pose_straight, weights_ring_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_ring_thumb > 1:
                      print("ring_torch")
                      '''
                      msg.position[0] = 68  
                      msg.position[3] = 0
                      msg.position[5] = 113
                      msg.position[8] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[18] = 175
                      '''
                      pass
         if min_index == 3 :
           print("little--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
         
           if self.L10_joint[6] < -1 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("little_probability_more---------")
               if self.L10_joint[0] < 0:
                   print("little_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                    
                      print("little_torch")
                      msg.position[0] = 135
                      msg.position[1] = 66
                      msg.position[5] = 128
                      msg.position[9] = 37
                      
                      msg.position[0] = msg.position[0] 
                      msg.position[1] = msg.position[1] 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      msg.position[5] = msg.position[5] 
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[8]
                      msg.position[9] = msg.position[9] 
                      
           if self.L10_joint[6] > -1 * 0.8 and self.L10_joint[6] < -0.2:
               print("little_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.L10_joint[0] <= 0.17:
                   print("little_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose_straight = [-0.99,0.582,0.582,-0.068]
                   weights_little_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.L10_joint[0], self.L10_joint[1], self.L10_joint[2], self.L10_joint[6]], little_torch_thumb_pose_straight, weights_little_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                      print("little_torch")
                      '''
                      msg.position[0] = 100  
                      msg.position[4] = 29
                      msg.position[5] = 79
                      msg.position[9] = 183
                      msg.position[10] = 0
                      msg.position[15] = 193
                      msg.position[19] = 166
                      ''' 
                      pass  
       
     
             
         
