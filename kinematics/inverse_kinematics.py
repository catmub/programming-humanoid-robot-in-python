'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


import numpy as np
from scipy.optimize import fmin
from math import atan2
from numpy.matlib import identity
from forward_kinematics import ForwardKinematicsAgent

class InverseKinematicsAgent(ForwardKinematicsAgent):
        
    def error_func(self, angles, joints, target):
        Te = np.identity(4)
        for joint, angle in zip(joints, angles):
            Te = Te @ self.local_trans(joint, angle)
        current = np.array(self.from_trans(Te))
        return np.linalg.norm(current - np.array(target).flatten())
        
    def from_trans(self,m):
        '''get x, y, theta from transform matrix'''
        return [m[0, -1], m[1, -1], atan2(m[1, 0], m[0, 0])]
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        self.ranges = {'HeadYaw': (-2.0857, 2.0857),
                       'HeadPitch': (-0.6720, 0.5149),
                       'LShoulderPitch': (-2.0857, 2.0857),
                       'RShoulderPitch': (-2.0857, 2.0857),
                       'LShoulderRoll': (-0.3142, 1.3265),
                       'RShoulderRoll': (-0.3142, 1.3265),
                       'LElbowYaw': (-2.0857, 2.0857),
                       'RElbowYaw': (-2.0857, 2.0857),
                       'LElbowRoll': (-1.5446, 0.0349),
                       'RElbowRoll': (-1.5446, 0.0349),
                       'LHipYawPitch': (-1.145303, 0.740810),
                       'RHipYawPitch': (-1.145303, 0.740810),
                       'LHipRoll': (-0.379472, 0.790477),
                       'RHipRoll': (-0.379472, 0.790477),
                       'LHipPitch': (-1.535889, 0.484090),
                       'RHipPitch': (-1.535889, 0.484090),
                       'LKneePitch': (-0.092346, 2.112528),
                       'RKneePitch': (-0.092346, 2.112528),
                       'LAnklePitch': (-1.189516, 0.922747),
                       'RAnklePitch': (-1.189516, 0.922747),
                       'LAnkleRoll': (-0.397880, 0.769001),
                       'RAnkleRoll': (-0.397880, 0.769001),}
        
        #Analytical solution
        #p, R = np.asarray(transform[:3, 3]).flatten(), np.asarray(transform[:3, :3])
        transform = np.array(transform)  # Ensure it's a NumPy array
        p, R = transform[:3, 3], transform[:3, :3]
         
        HipOffsetZ, HipOffsetY = 0.085, 0.050
        ThighLength, TibiaLength, FootHeight = 0.100, 0.1029, 0.04519

      
        hip_pos = np.array([0, HipOffsetY, -HipOffsetZ * 1 if effector_name == 'LLeg' else -1])
        ankle_pos = p.copy()
        ankle_pos[2] += FootHeight
        vec = ankle_pos - hip_pos

  
        hip_yaw_pitch = np.arctan2(vec[1], vec[0])
        vec_sag = np.array([
            [np.cos(-hip_yaw_pitch), -np.sin(-hip_yaw_pitch), 0],
            [np.sin(-hip_yaw_pitch),  np.cos(-hip_yaw_pitch), 0],
            [0, 0, 1]
        ]) @ vec
        hip_roll = np.arctan2(vec_sag[1], vec_sag[2])
        L = np.linalg.norm(vec_sag[[0, 2]])

  

        knee_pitch = np.pi - np.arccos(np.clip((ThighLength**2 + TibiaLength**2 - L**2) / (2 * ThighLength * TibiaLength), -1.0, 1.0))
        alpha = np.arccos(np.clip((ThighLength**2 + L**2 - TibiaLength**2) / (2 * ThighLength * L), -1.0, 1.0))
        hip_pitch = -(np.arctan2(-vec_sag[2], vec_sag[0]) + alpha)
        ankle_pitch = -(hip_pitch + knee_pitch)
        ankle_roll = np.arctan2(R[2, 1], R[2, 2]) - hip_roll

        angles = [hip_yaw_pitch, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll]
        return [
            np.clip(angle, *self.ranges[self.chains[effector_name][i]])
            for i, angle in enumerate(angles)
        ]

    def set_transforms(self, effector_name, transform):
        '''Solve IK and generate joint keyframes.'''
        angles = self.inverse_kinematics(effector_name, transform)
        names  = self.chains[effector_name]
        initial = [self.perception.joint[j] for j in names]
        times = [[0.0, 1.0]] * len(names)
        keys  = [
           [[i, [3,0,0],[3,0,0]], [g, [3,0,0],[3,0,0]]]
           for i, g in zip(initial, angles)
        ]
        self.keyframes = (names, times, keys)
        return True
        
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('RLeg', T)
    agent.run()
