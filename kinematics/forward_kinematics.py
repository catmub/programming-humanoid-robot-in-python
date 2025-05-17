'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent

import numpy as np

class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                       }
        self.link_offsets = {
        # Head
        'HeadYaw': [0, 0, 0.1265],      
        'HeadPitch': [0, 0, 0],        
    
        # Left Arm
        'LShoulderPitch': [0.0, 0.098, 0.100],    
        'LShoulderRoll': [0.0, 0.0, 0.0],    
        'LElbowYaw': [0.105, 0.015, 0.0],         
        'LElbowRoll': [0.0, 0.0, 0.0],        
        'LWristYaw': [0.05595, 0.0, 0.0],        
     
    
        # Right Arm (mirror of Left)
        'RShoulderPitch': [0.0, -0.098, 0.100],
        'RShoulderRoll': [0.0, 0.0, 0.0],
        'RElbowYaw': [0.105, -0.015, 0.0],
        'RElbowRoll': [0.0, 0.0, 0.0],
        'RWristYaw': [0.05595, 0.0, 0.0],
    
        # Left Leg
        'LHipYawPitch': [0.0, 0.050, -0.085],     
        'LHipRoll': [0.0, 0.0, 0.0],
        'LHipPitch': [0.0, 0.0, 0.0],
        'LKneePitch': [0.0, 0.0, -0.100],
        'LAnklePitch': [0.0, 0.0, -0.1029],
        'LAnkleRoll': [0.0, 0.0, 0.0],            
    
        # Right Leg (mirror of Left)
        'RHipYawPitch': [0.0, -0.050, -0.085],
        'RHipRoll': [0.0, 0.0, 0.0],
        'RHipPitch': [0.0, 0.0, 0.0],
        'RKneePitch': [0.0, 0.0, -0.100],
        'RAnklePitch': [0.0, 0.0, -0.1029],
        'RAnkleRoll': [0.0, 0.0, 0.0],
        }

        self.joint_axis= {
        'HeadYaw': [0, 0, 1],
        'HeadPitch': [0, 1, 0],

        'RShoulderPitch': [0, 1, 0],
        'RShoulderRoll': [0, 0, 1],
        'RElbowYaw': [1, 0, 0],
        'RElbowRoll': [0, 0, 1],
        'RWristYaw' : [0, 0, 1], #??? or it might be 1,1,1
            
        'LShoulderPitch': [0, 1, 0],
        'LShoulderRoll': [0, 0, 1],
        'LElbowYaw': [0, 1, 0],
        'LElbowRoll': [0, 0, 1],
        'LWristYaw' : [0, 0, 1], #??? or it might be 1,1,1

        'LHipYawPitch': [0, 0, 1],
        'RHipYawPitch': [0, 0, 1],
        
        'LHipRoll': [1, 0, 0],
        'LHipPitch': [0, 1, 0],
        'LKneePitch': [0, 1, 0],
        'LAnklePitch': [0, 1, 0],
        'LAnkleRoll': [1, 0, 0],
        
        'RHipRoll': [1, 0, 0],
        'RHipPitch': [0, 1, 0],
        'RKneePitch': [0, 1, 0],
        'RAnklePitch': [0, 1, 0],
        'RAnkleRoll': [1, 0, 0],
        }

    #help function
    def rotation_matrix(axis, angle):
    
        axis = tuple(axis)
        c, s = np.cos(angle), np.sin(angle)
        if axis == (1, 0, 0):  # X
            return np.array([
                [1, 0, 0],
                [0, c, -s],
                [0, s, c]
            ])
        elif axis == (0, 1, 0):  # Y
            return np.array([
                [c, 0, s],
                [0, 1, 0],
                [-s, 0, c]
            ])
        elif axis == (0, 0, 1):  # Z
            return np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ])
        else:
            raise ValueError("Unsupported rotation axis")
    #----
    
    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        # 1. Get fixed offset
        offset = self.link_offsets.get(joint_name, [0, 0, 0])
        T_offset = identity(4)
        T_offset[0:3, 3] = offset
    
        # 2. Get rotation
        axis = self.joint_axes.get(joint_name)
        if axis is None:
            raise KeyError(f"Missing axis definition for joint {joint_name}")
        R = rotation_matrix(axis, joint_angle)
        T_rotate = identity(4)
        T_rotate[0:3, 0:3] = R
    
        # 3. Combine: Translate then rotate
        T = T_offset @ T_rotate
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                for joint in chain:
                    T_local = self.local_trans(joint, joint_angles[joint])
                    T = T @ T_local
    
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
