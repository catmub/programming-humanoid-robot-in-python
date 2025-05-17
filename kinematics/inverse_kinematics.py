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


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def error_func(self, angles, joints, target):
        Te = np.identity(4)
        for joint, angle in zip(joints, joint_angles):
            Te = np.dot(Te, self.local_trans(joint_name, angle))
        e = matrix(self.from_transform(Te)).T
        return np.linalg.norm(e)
        
    def from_trans(m):
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
        joint_names = self.chains[effector_name]
        target = np.matrix(self.from_trans(transform))
        #optimized_angles = self._optimize_joint_angles(joint_names, target)
        initial_angles = [self.perception.joint[name] for name in joint_names]
        
        return fmin(lambda t: self._end_effector_error(t, joint_names, target), initial_angles)
        
        
    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        chain = self.chains[effector_name]
        timing = [[0.0, 3.0]] * len(joint_angles) 
        

        joint_animation = []
        for name, angle in joint_angles.items():
            joint_data = [
                [self.perception.joint[name], [3, 0, 0], [3, 0, 0]], 
                [angle, [3, 0, 0], [3, 0, 0]                        
            ]]
            joint_animation.append(joint_data)

        self.keyframes = (chain, timings, joint_animation)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
