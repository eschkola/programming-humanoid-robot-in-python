'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from scipy.optimize import fmin
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        angles_before = []
        
        joints = self.chains[effector_name]
        for joint in joints:
            angles_before.append(self.perception.joint[joint])
        
        # YOUR CODE HERE
        error_function = self.error_func(angles_before, effector_name, transform)
        optimization = fmin(error_function, angles_before)
        
        
        joint_angles = dict(zip(self.chains[effector_name], optimization))      
        
        return joint_angles
    
    def error_func(self, angles_before, limb, transform):
        Te = identity(4)
        for joint in self.chains_with[limb]:
            for angle in angles_before:
                Tl = self.local_trans(joint, angle)
                Te = limb @ Tl
                
        e = self.from_trans(transform.T) - self.from_trans(Te)
        return np.linalg.norm(e) 
            
        
        return np.sum(e * e)
    
    def from_trans(self, transform):
        theta_x = np.arctan2(transform[2, 1], transform[2, 2])
        theta_y = np.arctan2(-transform[2,0], transform[np.sqrt((transform[2, 1]) ** 2 + (transform[2, 2]) ** 2 )])
        theta_z = np.arctan2(transform[1, 0], transform[0, 0])
        
        return np.array([transform[0, -1], transform[1, -1], transform[2, -1], theta_x, theta_y, theta_z])
        
        
    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        
        joint_angles = self.inverse_kinematics(effector_name = transform)
        
        joints = self.chains[effector_name]
        times = [[0, 5]]  * len(joints)
        keys = []
        
        for i, name in enumerate(joints):
            keys.insert(i, [[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])
            
        self.keyframes = (joints, times, keys)



if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
