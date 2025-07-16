import numpy as np


class RightHand:
    def __init__(self, length=7):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length
        self.thumb_diff = [0] * 5
        self.index_diff = [0] * 3
        self.middle_diff = [0] * 3
        self.ring_diff = [0] * 3
        self.pinky_diff = [0] * 3
        self.fourroll_diff = [0] * 6

    def get_joint_diff(self,normal,open,close):
        all_diff = np.array(close) - np.array(open)
        self.thumb_diff = all_diff[:5]
        self.index_diff = all_diff[6:9]
        self.middle_diff = all_diff[10:13]
        self.ring_diff = all_diff[14:17]
        self.pinky_diff = all_diff[18:21]
        fourroll_index = np.array([1,2,5,9,13,17])
        fourroll_diff_array = np.array(open) - np.array(normal)
        self.fourroll_diff = fourroll_diff_array[fourroll_index]   

    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[0] = joint_arc[5]
        qpos[1] = joint_arc[6] * 0.55
        qpos[2] = joint_arc[7]
        qpos[3] = joint_arc[8]

        qpos[4] = joint_arc[17]
        qpos[5] = joint_arc[18] * 0.7
        qpos[6] = joint_arc[19]
        qpos[7] = joint_arc[20]

        qpos[8] = joint_arc[10] * 0.8
        qpos[9] = joint_arc[11]
        qpos[10] = joint_arc[12]

        qpos[11] = joint_arc[13]
        qpos[12] = joint_arc[14] * 0.7
        qpos[13] = joint_arc[15]
        qpos[14] = joint_arc[16]

        qpos[15] = joint_arc[1] * -0.8
        qpos[16] = joint_arc[1] * -1.8
        # print(qpos[15],qpos[16],joint_arc[0])
        qpos[17] = joint_arc[2] * 2.2
        qpos[18] = joint_arc[2]
        qpos[19] = joint_arc[2]

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
            if i == 0 or i == 1 or i == 9:
                self.g_jointvelocity[i] =  int(target_vel)
            else:
                self.g_jointvelocity[i] =  int(target_vel * 2)
            
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]


class LeftHand:
    def __init__(self, length=7):
        self.g_jointpositions = [255] * length
        self.g_jointvelocity = [255] * length
        self.last_jointpositions = [255] * length
        self.last_jointvelocity = [255] * length
        self.handstate = [0] * length
        self.thumb_diff = [0] * 5
        self.index_diff = [0] * 3
        self.middle_diff = [0] * 3
        self.ring_diff = [0] * 3
        self.pinky_diff = [0] * 3
        self.fourroll_diff = [0] * 6

    def get_joint_diff(self,normal,open,close):
        all_diff = np.array(close) - np.array(open)
        self.thumb_diff = all_diff[:5]
        self.index_diff = all_diff[6:9]
        self.middle_diff = all_diff[10:13]
        self.ring_diff = all_diff[14:17]
        self.pinky_diff = all_diff[18:21]
        fourroll_index = np.array([1,2,5,9,13,17])
        fourroll_diff_array = np.array(open) - np.array(normal)
        self.fourroll_diff = fourroll_diff_array[fourroll_index]   
        
    def joint_update(self, joint_arc):
        qpos = np.zeros(25)
        qpos[0] = joint_arc[5]
        qpos[1] = joint_arc[6] * 0.55
        qpos[2] = joint_arc[7]
        qpos[3] = joint_arc[8]

        qpos[4] = joint_arc[17]
        qpos[5] = joint_arc[18] * 0.7
        qpos[6] = joint_arc[19]
        qpos[7] = joint_arc[20]

        qpos[8] = joint_arc[10] * 0.8
        qpos[9] = joint_arc[11]
        qpos[10] = joint_arc[12]

        qpos[11] = joint_arc[13]
        qpos[12] = joint_arc[14] * 0.7
        qpos[13] = joint_arc[15]
        qpos[14] = joint_arc[16]

        qpos[15] = joint_arc[1] * -0.8
        qpos[16] = joint_arc[1] * -1.8
        # print(qpos[15],qpos[16],joint_arc[0])
        qpos[17] = joint_arc[2] * 2.2
        qpos[18] = joint_arc[2]
        qpos[19] = joint_arc[2]

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
            if i == 0 or i == 1 or i == 9:
                self.g_jointvelocity[i] =  int(target_vel)
            else:
                self.g_jointvelocity[i] =  int(target_vel * 2)
            
            if self.g_jointvelocity[i] > 255:
                self.g_jointvelocity[i] = 255
            self.last_jointvelocity[i] = self.g_jointvelocity[i]
            self.last_jointpositions[i] = self.g_jointpositions[i]