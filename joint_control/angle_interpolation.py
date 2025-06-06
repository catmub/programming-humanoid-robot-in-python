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
from keyframes import *


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        
        names, times_list, keys_list = keyframes
        t = perception.time - self.start_time
        for joint_idx, name in enumerate(names):
            times = times_list[joint_idx]
            keys = keys_list[joint_idx]
            #t = perception.time -self.start_time#%max(times)
            for j in range(len(times) - 1):
                #print('time i',times[j])
                #print('cur time', t)
                #print('time i+1',times[j+1])
                if times[j] < t < times[j + 1]:
                    i = (t - times[j]) / (times[j+1] - times[j])
                    
                    P0 = (keys[j][0])
                    P3 = (keys[j+1][0])
                    P1 = (P0+keys[j][1][2])       
                    P2 = (P3+keys[j+1][1][2])

                    Bi = ((1-i)**3)*P0 +   3*((1-i)**2)*i*P1  +  3*(1-i)*(i**2)*P2  +  (i**3)*P3
                    target_joints[name] = Bi
                    break
            if 'LHipYawPitch' in target_joints:
                target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        #print(target_joints)

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
    
