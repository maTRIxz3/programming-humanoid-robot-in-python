'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import threading
import weakref
from xmlrpc.client import ServerProxy
import hello
from numpy.matlib import identity


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,)).start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform)).start()



class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self, host="localhost", port=8000):
        self.server = ServerProxy(f'http://{host}:{port}', allow_none=True)
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.server.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        return self.server.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.server.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.server.set_transform(effector_name, transform)


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    print("\n=== RPC Client Test ===")
    print("→ get_angle('left_elbow'):", agent.get_angle("HeadYaw"))
    print("→ set_angle('left_elbow', 1.57):", agent.set_angle("LShoulderPitch", 0))
    print("→ get_posture():", agent.get_posture())
    
    # Simuliere keyframe-Test
    fake_keyframes = hello()
    print("→ execute_keyframes(...):", agent.execute_keyframes(fake_keyframes))
    
    # Transform Test
    print("→ get_transform:", agent.get_transform())
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    print("→ set_transform('LLeg', [0.1, 0.2, 0.3]):", agent.set_transform("LLeg", T))

    # Async test
    print("→ post.set_transform('LLeg', [0.2, 0.0, 0.0])")
    agent.post.set_transform("LLeg", T)

    print("→ post.execute_keyframes([...])")
    agent.post.execute_keyframes(hello())

