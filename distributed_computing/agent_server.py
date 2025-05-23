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

from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import threading

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service'''
    
    def __init__(self, host="localhost", port=8000):
        super().__init__()
        self.server = SimpleXMLRPCServer((host, port), allow_none=True, logRequests=False)
        self.server.register_instance(self)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        print(f"[RPC Server] Running on {host}:{port}")
    
    def run(self):
        self.server_thread.start()
        self.server_thread.join()

    def get_angle(self, joint_name):
        '''Get sensor value of given joint'''
        print(f"[RPC] get_angle({joint_name})")
        return self.perception.joint[joint_name]

    def set_angle(self, joint_name, angle):
        '''Set target angle of joint for PID controller'''
        print(f"[RPC] set_angle({joint_name}, {angle})")
        self.perception.joint[joint_name] = angle
        return self.perception.joint[joint_name]

    def get_posture(self):
        '''Return current posture of robot'''
        print("[RPC] get_posture()")
        return self.recognize_posture()
    
    def execute_keyframes(self, keyframes):
        '''Execute keyframes (blocking)'''
        print(f"[RPC] execute_keyframes({keyframes})")
        self.keyframes = keyframes
        return True

    def get_transform(self, name):
        '''Get transform with given name'''
        print(f"[RPC] get_transform({name})")
        transform = self.transforms.get(name)
        return transform

    def set_transform(self, effector_name, transform):
        '''Set a transform via inverse kinematics'''
        print(f"[RPC] set_transform({effector_name}, {transform})")
        self.set_transforms(effector_name, transform)
        return True


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()