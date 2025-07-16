import numpy as np
from linkerhand.handcore import HandCore

class RightHand:
    def __init__(self, handcore: HandCore, length=25):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.self.l21_joint = [0] * length
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[17] = joint_arc[3]*-3#侧摆2.25
        qpos[16] = joint_arc[20] * 1.6  # 旋转 
        qpos[18] = joint_arc[2] * -0.8  # 根部关节
        qpos[19] = joint_arc[1] * -0.6  # 中部关节
        qpos[20] = joint_arc[0] * -1 # 远端关节
        #print(f"cebai arc3:{joint_arc[3]}   xuanzhuan arc0:{joint_arc[0]} arc20:{qpos[20]} genbu arc2: {joint_arc[2]}  index ce:{joint_arc[7]}") 
        # 食指 index
        qpos[0] = joint_arc[7] * -1*0.9-0.09
        qpos[1] = joint_arc[6] * -1*1.4
        #qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[5] * -1
        #if joint_arc[6] > -75 * 3.14 / 180: qpos[3] = qpos[3]+0.35
        if joint_arc[5] > -80 * 3.14 / 180: qpos[3] = 0

        # 小指 little
        qpos[4] = joint_arc[19] * -1.5
        qpos[5] = joint_arc[18] * -1
        #qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1
        #if joint_arc[18] > -65 * 3.14 / 180: qpos[6] = 0
        if joint_arc[16] > -65 * 3.14 / 180: qpos[7] = 0

        # 中指 middle
        qpos[8] = joint_arc[11] * 0 #cebai
        qpos[9] = joint_arc[10] * -2 #genbu
        #qpos[9] = joint_arc[9] * -1 #zhongbu
        qpos[11] = joint_arc[8] * -1 #yuanduan
        #if joint_arc[10] > -70 * 3.14 / 180: qpos[9] = 0
        if joint_arc[8] > -70 * 3.14 / 180: qpos[11] = 0

        # 无名指 ring
        qpos[12] = joint_arc[15] * -1
        qpos[13] = joint_arc[14] * -1  # gen 
        #qpos[14] = joint_arc[13] * -1  # zhong
        qpos[15] = joint_arc[12] * -1
        #if joint_arc[14] > -70 * 3.14 / 180: qpos[13] = 0
        if joint_arc[12] > -70 * 3.14 / 180: qpos[15] = 0
    
        self.g_jointpositions = self.handcore.trans_to_motor_right(qpos)
        self.l21_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
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
        
   
        
    def glove_torch(self,msg):     
        distance_index_thumb = abs(self.l21_joint[0] + self.l21_joint[3])
        distance_middle_thumb = abs(self.l21_joint[0] + self.l21_joint[4])
        distance_ring_thumb = abs(self.l21_joint[0] + self.l21_joint[5])
        distance_little_thumb = abs(self.l21_joint[0] + self.l21_joint[6])
        distance = [distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb]
        #print("distance_index_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[3]))
        #print("distance_middle_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[4]))
        #print("distance_ring_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[5]))
        #print("distance_little_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[6]))
        
        value = self.l21_joint[3:]
        #print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.l21_joint[3], self.l21_joint[4], self.l21_joint[5], self.l21_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        grape_pose = [0.694,0.923,1.38,0.35]
        weights_grape_pose = [30,30,30,10]
        weighted_diff_grape_pose = [(j - p) * w for j, p, w in zip([distance[0], distance[1],distance[2],distance[3]],grape_pose, weights_grape_pose)]
        threshold_grape_pose = np.sqrt(np.sum(np.square(weighted_diff_grape_pose)))
        #print("threshold_grape_pose",threshold_grape_pose)
        #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
        #print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
          
           if self.l21_joint[3] < -1.3 * 0.5 * (1.6/distance_index_thumb):
               print("index_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold_l21_right",threshold)
                   if threshold < 40 and distance_index_thumb > 0.6:
                      msg.position[0] = 87 
                      msg.position[1] = 112 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      
                      msg.position[5] = 1
                      msg.position[6] = 222
                      msg.position[7] = msg.position[7] 
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      
                      msg.position[10] = 205
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      
                      msg.position[20] = 122 
                      msg.position[21] = 84 
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
                        
           if self.l21_joint[3] > -1.3 * 0.5  and self.l21_joint[3] < -0.2:
               print("index_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("index_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   index_torch_thumb_pose_straight = [-0.0837,0.129,0.129,-0.28]
                   weights_index_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[3]], index_torch_thumb_pose_straight, weights_index_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20 and distance_index_thumb < 0.6:
                     
                      
                      msg.position[0] = 66 
                      msg.position[1] = 28 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      
                      msg.position[5] = 4
                      msg.position[6] = 54
                      msg.position[7] = msg.position[7] 
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      
                      msg.position[10] = 255
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      
                      msg.position[20] = 255 
                      msg.position[21] = 188 
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
         if min_index == 1:
           print("middle--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if  distance_middle_thumb > 1.5:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("middle_probability_more---------")
               if self.l21_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                      #指根
                      msg.position[0] = 58 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = 120 
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 22
                      msg.position[6] = msg.position[6]
                      msg.position[7] = 109
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 130
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[11]
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 185 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = 84 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
           if distance_middle_thumb < 1.5:
               print("middle_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("middle_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose_straight = [-0.261,0.218,0.218,-0.642]
                   weights_middle_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[4]], middle_torch_thumb_pose_straight, weights_middle_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 25:
                    if distance_middle_thumb > 0.6:
                      #指根
                      msg.position[0] = 53 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = 1 
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 20
                      msg.position[6] = msg.position[6]
                      msg.position[7] = 114
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 125
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 255 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = 255 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
         if min_index == 2 and self.l21_joint[1] > 0.3:
           print("ring----------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
          
           if self.l21_joint[5] < -1.3 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("ring_probability_more---------")
               if self.l21_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    
                      print("ring_torch")
                      #指根
                      msg.position[0] = 80 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = 88 
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = 196
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 53
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 164 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[23]
                      msg.position[23] = 112 
                      msg.position[24] = msg.position[24]
           if self.l21_joint[5] > -1.3 * 0.8 and self.l21_joint[5] < -0.2:
               print("ring_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("ring_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose_straight = [-0.642,0.408,0.408,-0.780]
                   weights_ring_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[5]], ring_torch_thumb_pose_straight, weights_ring_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_ring_thumb > 1:
                      print("ring_torch")
                      #指根
                      msg.position[0] = 72 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = 1 
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = 163
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 81
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 255 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[22]
                      msg.position[23] = 255 
                      msg.position[24] = msg.position[24]
         if min_index == 3 :
           print("little--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
          
           if self.l21_joint[6] < -1 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("little_probability_more---------")
               if self.l21_joint[0] < 0:
                   print("little_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                    
                      print("little_torch")
                      #指根
                      msg.position[0] = 79 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = 99 
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[9]
                      msg.position[9] = 225 
                      #横滚
                      msg.position[10] = 1
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 145 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = 120 
                      
                      
           if self.l21_joint[6] > -1 * 0.8 and self.l21_joint[6] < -0.2:
               print("little_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("little_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose_straight = [-0.99,0.582,0.582,-0.068]
                   weights_little_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[6]], little_torch_thumb_pose_straight, weights_little_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                      print("little_torch")
                      #指根
                      msg.position[0] = 59 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = 20 
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[9]
                      msg.position[9] = 221 
                      #横滚
                      msg.position[10] = 13
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 255 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = 255 
        #else:
            #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)  
            
            
            
#1 thumb gen 18
#2 index gen 1 
#3 mid gen 9
#4 ring gen 13
#5 little gen 5
#6 thumb ce 17
#7 index 0
#8 mid 8
#9 ring 12
#10 little4
#11 thumb xuan16
#12 n
#13 n 
#14 n
#15 n
#16 thumb mid19
#17 n
#18 n
#19 n
#20 n
#21 thumb l20
#22 index3
#23 mid11
#24 ring 15
#25 little7
#


class LeftHand:
    def __init__(self, handcore: HandCore, length=25):
        self.handcore = handcore
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.g_jointpositions[6:4] = [128, 128, 128, 128]
        self.handstate = [0] * length

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[17] = joint_arc[20] * 2  # 侧摆2.25
        qpos[16] = joint_arc[20] * 1  # 旋转 
        qpos[18] = joint_arc[2] * -0.4 # 根部关节
        qpos[19] = joint_arc[1] * -0.6  # 中部关节
        qpos[20] = joint_arc[0] * -1 # 远端关节
        #print(f"cebai arc3:{joint_arc[3]}   xuanzhuan arc0:{joint_arc[0]} arc20:{qpos[20]} genbu arc2: {joint_arc[2]}  index ce:{joint_arc[7]}") 
        # 食指 index
        qpos[0] = joint_arc[7] * -1*0.9-0.09
        qpos[1] = joint_arc[6] * -1
        #qpos[2] = joint_arc[5] * -1
        qpos[3] = joint_arc[5] * -1
        #if joint_arc[6] > -75 * 3.14 / 180: qpos[3] = qpos[3]+0.35
        if joint_arc[5] > -80 * 3.14 / 180: qpos[3] = 0

        # 小指 little
        qpos[4] = joint_arc[19] * -1  #19
        qpos[5] = joint_arc[18] * -1
        #qpos[6] = joint_arc[17] * -1
        qpos[7] = joint_arc[16] * -1
        #if joint_arc[18] > -65 * 3.14 / 180: qpos[6] = 0
        if joint_arc[16] > -65 * 3.14 / 180: qpos[7] = 0

        # 中指 middle
        qpos[8] = joint_arc[11] * 1#cebai
        qpos[9] = joint_arc[10] * -1 #genbu
        #qpos[9] = joint_arc[9] * -1 #zhongbu
        qpos[11] = joint_arc[8] * -1 #yuanduan
        #if joint_arc[10] > -70 * 3.14 / 180: qpos[9] = 0
        if joint_arc[8] > -70 * 3.14 / 180: qpos[11] = 0

        # 无名指 ring
        qpos[12] = joint_arc[15] * -1
        qpos[13] = joint_arc[14] * -1  # gen 
        #qpos[14] = joint_arc[13] * -1  # zhong
        qpos[15] = joint_arc[12] * -1
        #if joint_arc[14] > -70 * 3.14 / 180: qpos[13] = 0
        if joint_arc[12] > -70 * 3.14 / 180: qpos[15] = 0

        self.g_jointpositions = self.handcore.trans_to_motor_left(qpos)
        self.l21_joint = [joint_arc[2],joint_arc[20],joint_arc[20],joint_arc[6],joint_arc[10],joint_arc[14],joint_arc[18]]
        
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
               
    def glove_torch(self,msg):
        distance_index_thumb = abs(self.l21_joint[0] + self.l21_joint[3])
        distance_middle_thumb = abs(self.l21_joint[0] + self.l21_joint[4])
        distance_ring_thumb = abs(self.l21_joint[0] + self.l21_joint[5])
        distance_little_thumb = abs(self.l21_joint[0] + self.l21_joint[6])
        distance = [distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb]
        #print("distance_index_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[3]))
        #print("distance_middle_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[4]))
        #print("distance_ring_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[5]))
        #print("distance_little_thumb_left_L21",abs(self.l21_joint[0] + self.l21_joint[6]))
        
        value = self.l21_joint[3:]
        #print("value",value)
        minvalue = min(value)
        min_index = value.index(minvalue)
        
        stone_pose = [-1.39,-1.39,-1.39,-1.39]
        weights_index_stone = [50, 50, 50, 50]
        weighted_diff_stone = [(j - p) * w for j, p, w in zip([self.l21_joint[3], self.l21_joint[4], self.l21_joint[5], self.l21_joint[6]], stone_pose, weights_index_stone)]
        threshold_stone = np.sqrt(np.sum(np.square(weighted_diff_stone)))
        grape_pose = [0.694,0.923,1.38,0.35]
        weights_grape_pose = [30,30,30,10]
        weighted_diff_grape_pose = [(j - p) * w for j, p, w in zip([distance[0], distance[1],distance[2],distance[3]],grape_pose, weights_grape_pose)]
        threshold_grape_pose = np.sqrt(np.sum(np.square(weighted_diff_grape_pose)))
        #print("threshold_grape_pose",threshold_grape_pose)
        #self.origin_hand_data(msg,msg)
        #print("threshold_stone",threshold_stone)
        if threshold_stone > 30:
         if min_index == 0:
           print("index---------")
           #self.origin_hand_data(msg,msg)
          
           if self.l21_joint[3] < -1.3 * 0.5 * (1.6/distance_index_thumb):
               print("index_probability_more---------")
               #self.origin_hand_data(msg,msg)
               if self.l21_joint[0] <= 0.17:
                   print("index_probability_more_more---------")
                   #self.origin_hand_data(msg,msg)
                   index_torch_thumb_pose = [-0.12,0.15,0.15,-1.39]
                   weights_index = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[3]], index_torch_thumb_pose, weights_index)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 and distance_index_thumb > 0.6:
                      msg.position[0] = 87 
                      msg.position[1] = 112 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      
                      msg.position[5] = 1
                      msg.position[6] = 222
                      msg.position[7] = msg.position[7] 
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      
                      msg.position[10] = 205
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      
                      msg.position[20] = 122 
                      msg.position[21] = 84 
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
                        
           if self.l21_joint[3] > -1.3 * 0.5  and self.l21_joint[3] < -0.2:
               print("index_straight_probability_more---------")
               #self.origin_hand_data(msg,msg)
               if self.l21_joint[0] <= 0.17:
                   print("index_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg)
                   index_torch_thumb_pose_straight = [-0.0837,0.129,0.129,-0.28]
                   weights_index_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[3]], index_torch_thumb_pose_straight, weights_index_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight_l21",threshold)
                   if threshold < 20:
                     
                      
                      msg.position[0] = 66 
                      msg.position[1] = 28 
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      
                      msg.position[5] = 4
                      msg.position[6] = 54
                      msg.position[7] = msg.position[7] 
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      
                      msg.position[10] = 255
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      
                      msg.position[20] = 255 
                      msg.position[21] = 188 
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
         if min_index == 1:
           print("middle--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
          
           if  distance_middle_thumb > 1.5:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("middle_probability_more---------")
               if self.l21_joint[0] < 0:
                   print("middle_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose = [-0.44,0.31,0.31,-1.39]
                   weights_middle = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[4]], middle_torch_thumb_pose, weights_middle)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                      #指根
                      msg.position[0] = 58 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = 120 
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 22
                      msg.position[6] = msg.position[6]
                      msg.position[7] = 109
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 130
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[11]
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 185 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = 84 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
           if distance_middle_thumb < 1.5:
               print("middle_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("middle_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   middle_torch_thumb_pose_straight = [-0.261,0.218,0.218,-0.642]
                   weights_middle_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[4]], middle_torch_thumb_pose_straight, weights_middle_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 25:
                    if distance_middle_thumb > 0.6:
                      #指根
                      msg.position[0] = 53 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = 1 
                      msg.position[3] = msg.position[3]
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 20
                      msg.position[6] = msg.position[6]
                      msg.position[7] = 114
                      msg.position[8] = msg.position[8] 
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 125
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 255 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = 255 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = msg.position[24]
         if min_index == 2 and self.l21_joint[1] > 0.3:
           print("ring----------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.l21_joint[5] < -1.3 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("ring_probability_more---------")
               if self.l21_joint[0] < 0:
                   print("ring_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose = [-1.16,0.67,0.67,-0.89]
                   weights_ring = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[5]], ring_torch_thumb_pose, weights_ring)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40:
                    
                      print("ring_torch")
                      #指根
                      msg.position[0] = 80 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = 88 
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = 196
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 53
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 164 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[23]
                      msg.position[23] = 112 
                      msg.position[24] = msg.position[24]
           if self.l21_joint[5] > -1.3 * 0.8 and self.l21_joint[5] < -0.2:
               print("ring_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("ring_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   ring_torch_thumb_pose_straight = [-0.642,0.408,0.408,-0.780]
                   weights_ring_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[5]], ring_torch_thumb_pose_straight, weights_ring_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                     if distance_ring_thumb > 1:
                      print("ring_torch")
                      #指根
                      msg.position[0] = 72 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = 1 
                      msg.position[4] = msg.position[4]
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = 163
                      msg.position[9] = msg.position[9] 
                      #横滚
                      msg.position[10] = 81
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 255 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[22]
                      msg.position[23] = 255 
                      msg.position[24] = msg.position[24]
         if min_index == 3 :
           print("little--------")
           #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
           
           if self.l21_joint[6] < -1 * 0.8:
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               print("little_probability_more---------")
               if self.l21_joint[0] < 0:
                   print("little_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose = [-1.22,0.698,0.698,-1.312]
                   weights_little = [10, 50, 50, 50]
                   weighted_diff = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[6]], little_torch_thumb_pose, weights_little)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff)))
                   print("threshold",threshold)
                   if threshold < 40 :
                    
                      print("little_torch")
                      #指根
                      msg.position[0] = 79 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = 99 
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[9]
                      msg.position[9] = 225 
                      #横滚
                      msg.position[10] = 1
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255 
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 145 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = 120 
                      
                      
           if self.l21_joint[6] > -1 * 0.8 and self.l21_joint[6] < -0.2:
               print("little_straight_probability_more---------")
               #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
               if self.l21_joint[0] <= 0.17:
                   print("little_straight_probability_more_more---------")
                   #self.origin_hand_data(msg,msg,threshold_grape_pose,distance_index_thumb,distance_middle_thumb,distance_ring_thumb,distance_little_thumb)
                   little_torch_thumb_pose_straight = [-0.99,0.582,0.582,-0.068]
                   weights_little_straight = [20, 50, 50, 50]
                   weighted_diff_straight = [(j - p) * w for j, p, w in zip([self.l21_joint[0], self.l21_joint[1], self.l21_joint[2], self.l21_joint[6]], little_torch_thumb_pose_straight, weights_little_straight)]
                   threshold = np.sqrt(np.sum(np.square(weighted_diff_straight)))
                   print("threshold_straight",threshold)
                   if threshold < 20:
                      print("little_torch")
                      #指根
                      msg.position[0] = 59 
                      msg.position[1] = msg.position[1]
                      msg.position[2] = msg.position[2]
                      msg.position[3] = msg.position[3]
                      msg.position[4] = 20 
                      #侧摆
                      msg.position[5] = 1
                      msg.position[6] = msg.position[6]
                      msg.position[7] = msg.position[7]
                      msg.position[8] = msg.position[9]
                      msg.position[9] = 221 
                      #横滚
                      msg.position[10] = 13
                      msg.position[11] = msg.position[11]
                      msg.position[12] = msg.position[12] 
                      msg.position[13] = msg.position[13] 
                      msg.position[14] = msg.position[14] 
                      #中部
                      msg.position[15] = 255
                      msg.position[16] = msg.position[16]
                      msg.position[17] = msg.position[17] 
                      msg.position[18] = msg.position[18] 
                      msg.position[19] = msg.position[19]
                      #指尖
                      msg.position[20] = 255 
                      msg.position[21] = msg.position[21]
                      msg.position[22] = msg.position[22] 
                      msg.position[23] = msg.position[23] 
                      msg.position[24] = 255 
        #else:
            #self.origin_hand_data(msg,msg)  
            
            
            
     
              
     
