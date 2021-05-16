'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import rightBackToStand
from scipy import interpolate
import numpy as np
from scipy.interpolate import BSpline


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        #__________________________________________________________________
        
        all_names, all_times, all_keys = keyframes
        
        if(self.start_time == -1):
            self.start_time = perception.time
            
        actual_time = perception.time - self.start_time
        
        
        for joint in range(len(all_names)):
            name = all_names[joint]
            
            if name not in self.joint_names:
                continue
                        
            joint_times = all_times[joint]
            joint_keys = all_keys[joint]
           
            
            last_elem = len(joint_times)
            if(actual_time >= (joint_times[0] - 0.1) and actual_time <= joint_times[last_elem-1]):
                f = interpolate.splrep(joint_times, [item[0] for item in joint_keys])
                target_joints[name] = interpolate.splev(actual_time, f)
        
        #__________________________________________________________________
        
        # names, times, keys = keyframes

        # if self.start_time == 0:
        #     self.start_time = perception.time

        # new_time = perception.time - self.start_time

        # for i in range(len(names)):
        #     name = names[i]

        #     if name not in self.joint_names:
        #         continue

        #     time = times[i]

        #     key = keys[i]

        #     for t in range(len(time)):
        #         if new_time > time[-1]:
        #             continue

        #         if new_time < time[t]:

        #             if t == 0:
        #                 T0, P0, P1 = 0, 0, 0
        #                 T3 = time[0]
        #                 P2 = key[0][1][2]
        #                 P3 = key[0][0]
        #                 j = new_time / T3
                    
        #             elif new_time >= time[t - 1]:
        #                 T0 = time[t - 1]
        #                 T3 = time[t]
        #                 P0 = key[t - 1][0]
        #                 P3 = key[t][0]
        #                 P1 = P0 + key[t - 1][2][2]
        #                 P2 = P3 + key[t][1][2]
        #                 j = (new_time - T0)/(T3 - T0)

        #             target_joints[name] = ((1 - j) ** 3) * P0 + 3 * ((1 - j) ** 2) * j * P1 + 3 * ((1 - j) ** 3) * (j **2) * P2 + (j ** 3) * P3

        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
