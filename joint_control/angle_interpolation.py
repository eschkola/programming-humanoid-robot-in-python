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
from keyframes import leftBackToStand
from scipy import interpolate
import numpy as np
from scipy.interpolate import BSpline
import standing_up


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
        
        if(self.start_time < 0):
            self.start_time = perception.time
            
        actual_time = perception.time - self.start_time
        
        for joint in range(len(all_names)):
                
            name = all_names[joint]    
            joint_times = all_times[joint]
            joint_keys = all_keys[joint]
            
            first_items = [item[0] for item in joint_keys]

                        
            if actual_time > joint_times[-1]:
                target_joints[name] = first_items[-1]
                if joint == len(all_names) - 1:
                    self.keyframes = ([], [], [])
                    self.start_times = -1;
                    
            if actual_time <= joint_times[0]:
                target_joints[name] = first_items[0]
                
            

            if(joint_times[0] <= actual_time <= joint_times[-1]):
                
                # original idea: spline with scipy
                f = interpolate.splrep(joint_times, first_items)
                target_joints[name] = interpolate.splev(actual_time, f)
                
                # bezier - should be working but not my original idea
                # frame_n = np.argmax(np.array(joint_times) > actual_time)
                # max_t = joint_times[frame_n]
                # min_t = joint_times[frame_n-1] if frame_n != 0 else 0
                # time = (actual_time - min_t) / (max_t - min_t)
                # target_joints[name] = self.bezier(joint_keys, frame_n, time)
                
                # point = np.argmax(np.array(joint_times) > actual_time)
                
                # P3 = joint_keys[point][0]
                # P0 = joint_keys[point - 1][0]
                # P1 = P0 + joint_keys[point - 1][2][2]
                # P2 = P3 + joint_keys[point][1][2]
                
                # if point != 0:
                #     min = joint_times[point - 1]
                # else:
                #     min = 0
                
                # max = joint_times[point]
                
                # t =  (actual_time - min) / (max - min)
                
                # target_joints[name] = ((1-t) ** 3) * P0 \
                #                         + 3 * (1-t) ** 2 * t * P1 \
                #                         + 3 * (1-t) * (t ** 2) * P2 \
                #                         + t ** 3 * P3
                
            if ("LHipYawPitch" in target_joints):
                target_joints["RHipYawPitch"] = target_joints["LHipYawPitch"]
       
        
        return target_joints
    
    def bezier(self, keys, frame_n, t):
        p0 = keys[frame_n-1][0]
        p3 = keys[frame_n][0]
        p1 = p0 + keys[frame_n-1][2][2]
        p2 = p3 + keys[frame_n][1][2]
        return np.power(1 - t, 3) * p0 + 3 * t * np.power(1 - t, 2) * p1 + 3 * np.power(t, 2) * (
                    1 - t) * p2 + np.power(t, 3) * p3

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
