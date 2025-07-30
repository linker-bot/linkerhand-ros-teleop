
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
        self.l20_joint = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[16] = joint_arc[20] * 1
        qpos[17] = joint_arc[20] * 2.570567
        qpos[18] = joint_arc[2] * -2 #thumb_root
        qpos[19] = joint_arc[1] * -1.231859 #thumb_tip
        

        qpos[0] = joint_arc[7] * 1
        qpos[1] = joint_arc[6] * -1
        qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[4] * -1

        qpos[8] = joint_arc[11] * -1
        qpos[9] = joint_arc[10] * -1
        qpos[10] = joint_arc[9] * -1
        qpos[11] = joint_arc[8] * -1

        qpos[12] = joint_arc[15] * 0.5
        qpos[13] = joint_arc[14] * -1
        qpos[14] = joint_arc[13] * -1
        qpos[15] = joint_arc[12] * -1
        
        qpos[4] = joint_arc[19] * 1
        qpos[5] = joint_arc[18] * -1
        qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1

        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.l20_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
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
            self.g_jointvelocity [i] = int(255)

            if self.g_jointvelocity [i] > 255:
                self.g_jointvelocity [i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity [i]
            self.last_jointpositions[i] = self.g_jointpositions[i]
            
    def glove_torch(self,msg):
        distance_index_thumb = abs(self.l20_joint[0] + self.l20_joint[3])
        distance_middle_thumb = abs(self.l20_joint[0] + self.l20_joint[4])
        distance_ring_thumb = abs(self.l20_joint[0] + self.l20_joint[5])
        distance_little_thumb = abs(self.l20_joint[0] + self.l20_joint[6])
        print("distance_index_thumb",abs(self.l20_joint[0] + self.l20_joint[3]))
        print("distance_middle_thumb",abs(self.l20_joint[0] + self.l20_joint[4]))
        print("distance_ring_thumb",abs(self.l20_joint[0] + self.l20_joint[5]))
        print("distance_little_thumb",abs(self.l20_joint[0] + self.l20_joint[6]))
        
        value = self.l20_joint[3:]
        #print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.l20_joint[3], self.l20_joint[4], self.l20_joint[5], self.l20_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           
           if self.l20_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                     if distance_index_thumb > 1.4:
                      print("index_torch")
                      
                      msg.position[0] = 131  
                      msg.position[1] = 97
                      msg.position[5] = 188
                      msg.position[6] = 118
                      msg.position[10] = 0
                      msg.position[15] = 159
                      msg.position[16] = 91
                      
                      msg.position[0] = msg.position[0]  
                      msg.position[1] = msg.position[1]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[10] = msg.position[10]
                      msg.position[15] = msg.position[15]
                      msg.position[16] = msg.position[16]
                      
                      
           if self.l20_joint[3] > -1.3 * 0.5 and self.l20_joint[3] < -0.2:
               print("index_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("index_straight_probability_more_more---------")
                   index_torch_thumb_pose_straight = [-0.0837,0.129,0.129,-0.28]
                   weights_index_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[3]], index_torch_thumb_pose_straight, weights_index_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_index_thumb > 0.5:
                      msg.position[0] = 60  
                      msg.position[1] = 0
                      msg.position[5] = 190
                      msg.position[6] = 33
                      msg.position[10] = 9
                      msg.position[15] = 255
                      msg.position[16] = 133
                   
         if min_index == 1:
           print("middle--------")
           
           if self.l20_joint[4] < -1.3 * 0.8:
               print("middle_probability_more---------")
               if self.l20_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                    if distance_middle_thumb > 1.5:
                      print("middle_torch")
                      msg.position[0] = 108  
                      msg.position[2] = 150
                      msg.position[5] = 129
                      msg.position[7] = 128
                      msg.position[10] = 41
                      msg.position[15] = 167
                      msg.position[17] = 68
           if self.l20_joint[4] > -1.3 * 0.8 and self.l20_joint[4] < -0.2:
               print("middle_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("middle_straight_probability_more_more---------")
                   middle_torch_thumb_pose_straight = [-0.261,0.218,0.218,-0.642]
                   weights_middle_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[4]], middle_torch_thumb_pose_straight, weights_middle_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 25:
                    if distance_middle_thumb > 0.6: 
                      msg.position[0] = 54  
                      msg.position[2] = 0
                      msg.position[5] = 141
                      msg.position[7] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[17] = 176
           
         if min_index == 2 and self.l20_joint[1] > 0.3:
           print("ring----------")
          
           if self.l20_joint[5] < -1.3 * 0.8:
               print("ring_probability_more---------")
               if self.l20_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    if distance_ring_thumb > 1.5:
                      print("ring_torch")
                      msg.position[0] = 205  
                      msg.position[3] = 81
                      msg.position[5] = 91
                      msg.position[8] = 128
                      msg.position[10] = 29
                      msg.position[15] = 133
                      msg.position[18] = 126
           if self.l20_joint[5] > -1.3 * 0.8 and self.l20_joint[5] < -0.2:
               print("ring_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("ring_straight_probability_more_more---------")
                   ring_torch_thumb_pose_straight = [-0.642,0.408,0.408,-0.780]
                   weights_ring_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[5]], ring_torch_thumb_pose_straight, weights_ring_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_ring_thumb > 1:
                      print("ring_torch")
                      msg.position[0] = 68  
                      msg.position[3] = 0
                      msg.position[5] = 113
                      msg.position[8] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[18] = 175
           
         if min_index == 3 :
           print("little--------")
           
           if self.l20_joint[6] < -1 * 0.8:
               print("little_probability_more---------")
               if self.l20_joint[0] < 0:
                   print("little_probability_more_more---------")
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                    if distance_little_thumb > 0.8:
                      print("little_torch")
                      msg.position[0] = 152  
                      msg.position[4] = 118
                      msg.position[5] = 80
                      msg.position[9] = 163
                      msg.position[10] = 5
                      msg.position[15] = 137
                      msg.position[19] = 105   
           ''' 
           if self.l20_joint[6] > -1 * 0.8 and self.l20_joint[6] < -0.2:
               print("little_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("little_straight_probability_more_more---------")
                   little_torch_thumb_pose_straight = [-0.99,0.582,0.582,-0.068]
                   weights_little_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[6]], little_torch_thumb_pose_straight, weights_little_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                      print("little_torch")
                      msg.position[0] = 100  
                      msg.position[4] = 29
                      msg.position[5] = 79
                      msg.position[9] = 183
                      msg.position[10] = 0
                      msg.position[15] = 193
                      msg.position[19] = 166
           '''
         
         
             
                
      


class LeftHand:
    def __init__(self, handcore: HandCore, length=20):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length
        self.l20_joint = [0] * length

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
        self.l20_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
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
            
    def glove_torch(self,msg):
        distance_index_thumb = abs(self.l20_joint[0] + self.l20_joint[3])
        distance_middle_thumb = abs(self.l20_joint[0] + self.l20_joint[4])
        distance_ring_thumb = abs(self.l20_joint[0] + self.l20_joint[5])
        distance_little_thumb = abs(self.l20_joint[0] + self.l20_joint[6])
        print("distance_index_thumb",abs(self.l20_joint[0] + self.l20_joint[3]))
        print("distance_middle_thumb",abs(self.l20_joint[0] + self.l20_joint[4]))
        print("distance_ring_thumb",abs(self.l20_joint[0] + self.l20_joint[5]))
        print("distance_little_thumb",abs(self.l20_joint[0] + self.l20_joint[6]))
        
        value = self.l20_joint[3:]
        #print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.l20_joint[3], self.l20_joint[4], self.l20_joint[5], self.l20_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        
        
        print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           
           if self.l20_joint[3] < -1.3 * 0.5:
               print("index_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                     if distance_index_thumb > 1.4:
                      print("index_torch")
                      
                      msg.position[0] = 131  
                      msg.position[1] = 97
                      msg.position[5] = 188
                      msg.position[6] = 118
                      msg.position[10] = 0
                      msg.position[15] = 159
                      msg.position[16] = 91
                      
                      msg.position[0] = msg.position[0]  
                      msg.position[1] = msg.position[1]
                      msg.position[5] = msg.position[5]
                      msg.position[6] = msg.position[6]
                      msg.position[10] = msg.position[10]
                      msg.position[15] = msg.position[15]
                      msg.position[16] = msg.position[16]
                    
                      
           if self.l20_joint[3] > -1.3 * 0.5 and self.l20_joint[3] < -0.2:
               print("index_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("index_straight_probability_more_more---------")
                   index_torch_thumb_pose_straight = [-0.0837,0.129,0.129,-0.28]
                   weights_index_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[3]], index_torch_thumb_pose_straight, weights_index_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_index_thumb > 0.5:
                      msg.position[0] = 60  
                      msg.position[1] = 0
                      msg.position[5] = 190
                      msg.position[6] = 33
                      msg.position[10] = 9
                      msg.position[15] = 255
                      msg.position[16] = 133
                   
         if min_index == 1:
           print("middle--------")
           
           if self.l20_joint[4] < -1.3 * 0.8:
               print("middle_probability_more---------")
               if self.l20_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 30:
                    if distance_middle_thumb > 1.5:
                      print("middle_torch")
                      msg.position[0] = 108  
                      msg.position[2] = 150
                      msg.position[5] = 129
                      msg.position[7] = 128
                      msg.position[10] = 41
                      msg.position[15] = 167
                      msg.position[17] = 68
           if self.l20_joint[4] > -1.3 * 0.8 and self.l20_joint[4] < -0.2:
               print("middle_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("middle_straight_probability_more_more---------")
                   middle_torch_thumb_pose_straight = [-0.261,0.218,0.218,-0.642]
                   weights_middle_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[4]], middle_torch_thumb_pose_straight, weights_middle_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 25:
                    if distance_middle_thumb > 0.6: 
                      msg.position[0] = 54  
                      msg.position[2] = 0
                      msg.position[5] = 141
                      msg.position[7] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[17] = 176
           
         if min_index == 2 and self.l20_joint[1] > 0.3:
           print("ring----------")
          
           if self.l20_joint[5] < -1.3 * 0.8:
               print("ring_probability_more---------")
               if self.l20_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    if distance_ring_thumb > 1.5:
                      print("ring_torch")
                      msg.position[0] = 205  
                      msg.position[3] = 81
                      msg.position[5] = 91
                      msg.position[8] = 128
                      msg.position[10] = 29
                      msg.position[15] = 133
                      msg.position[18] = 126
           if self.l20_joint[5] > -1.3 * 0.8 and self.l20_joint[5] < -0.2:
               print("ring_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("ring_straight_probability_more_more---------")
                   ring_torch_thumb_pose_straight = [-0.642,0.408,0.408,-0.780]
                   weights_ring_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[5]], ring_torch_thumb_pose_straight, weights_ring_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_ring_thumb > 1:
                      print("ring_torch")
                      msg.position[0] = 68  
                      msg.position[3] = 0
                      msg.position[5] = 113
                      msg.position[8] = 128
                      msg.position[10] = 0
                      msg.position[15] = 255
                      msg.position[18] = 175
           
         if min_index == 3 :
           print("little--------")
           
           if self.l20_joint[6] < -1 * 0.8:
               print("little_probability_more---------")
               if self.l20_joint[0] < 0:
                   print("little_probability_more_more---------")
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                    if distance_little_thumb > 0.8:
                      print("little_torch")
                      msg.position[0] = 152  
                      msg.position[4] = 118
                      msg.position[5] = 80
                      msg.position[9] = 163
                      msg.position[10] = 5
                      msg.position[15] = 137
                      msg.position[19] = 105   
           ''' 
           if self.l20_joint[6] > -1 * 0.8 and self.l20_joint[6] < -0.2:
               print("little_straight_probability_more---------")
               if self.l20_joint[0] <= 0.17:
                   print("little_straight_probability_more_more---------")
                   little_torch_thumb_pose_straight = [-0.99,0.582,0.582,-0.068]
                   weights_little_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l20_joint[0], self.l20_joint[1], self.l20_joint[2], self.l20_joint[6]], little_torch_thumb_pose_straight, weights_little_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                      print("little_torch")
                      msg.position[0] = 100  
                      msg.position[4] = 29
                      msg.position[5] = 79
                      msg.position[9] = 183
                      msg.position[10] = 0
                      msg.position[15] = 193
                      msg.position[19] = 166
           '''
         
         
                     
            
     
