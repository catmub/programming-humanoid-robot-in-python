'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

import xmlrpc.server 
from xmlrpc.server import SimpleXMLRPCServer
import logging
import time


import numpy as np
import threading


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        		
        logging.basicConfig(level=logging.DEBUG)
        s = SimpleXMLRPCServer(('localhost',9999),logRequests=True, allow_none=True)
        #print("XMLRBC works")
        s.register_instance(self)
        s.register_introspection_functions()
        #print("introspection")
        s.register_multicall_functions()
        #print("server init works")
        thread = threading.Thread(target=s.serve_forever)
        thread.start()
        #print("thread works")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle
        return True

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        names, times_list, _ = keyframes
        duration = max(max(times) for times in times_list)  # Total time for the keyframes
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            self.perception.time = elapsed + self.start_time  # Simulate time passage
            self.target_joints = self.angle_interpolation(self.keyframes, self.perception)

            if elapsed >= duration:
                break

            #time.sleep(0.01)  # Sleep briefly to simulate real-time motion steps

        return True
        
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        transform = np.array(transform) 
        self.set_transforms(effector_name, transform)
        print(self.set_transforms(effector_name, transform))
        print('transform', transform)
        print('eff', effector_name)
        return True

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

