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
from numpy import sin, cos, pi
from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent


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
                        'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                        'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                        'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }

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

        # Definition der Rotationsachsen laut NAO-Doku:
        if joint_name in ['HeadYaw', 'LShoulderYaw', 'RShoulderYaw',
                          'LHipYawPitch', 'RHipYawPitch','LShoulderRoll','RShoulderRoll']:
            axis = 'z'
        elif joint_name in ['HeadPitch', 'LElbowYaw', 'RElbowYaw',
                            'LShoulderPitch', 'RShoulderPitch',
                            'LHipPitch', 'RHipPitch',
                            'LKneePitch', 'RKneePitch',
                            'LAnklePitch', 'RAnklePitch']:
            axis = 'y'
        elif joint_name in ['LElbowRoll', 'RElbowRoll',
                            'LHipRoll', 'RHipRoll',
                            'LAnkleRoll', 'RAnkleRoll']:
            axis = 'x'
        else:
            raise ValueError(f"Unbekanntes Gelenk: {joint_name}")


        if axis == 'x':
            T[1,1] = cos(joint_angle)
            T[1,2] = -sin(joint_angle)
            T[2,1] = sin(joint_angle)
            T[2,2] = cos(joint_angle)
        elif axis == 'y':
            T[0,0] = cos(joint_angle)
            T[0,2] = sin(joint_angle)
            T[2,0] = -sin(joint_angle)
            T[2,2] = cos(joint_angle)
        elif axis == 'z':
            T[0,0] = cos(joint_angle)
            T[0,1] = -sin(joint_angle)
            T[1,0] = sin(joint_angle)
            T[1,1] = cos(joint_angle)
        
        Tx = 0.0
        Ty = 0.0
        Tz = 0.0

        if joint_name == 'HeadYaw':
            Tz = 0.1265 
        elif joint_name in ['LShoulderPitch','RShoulderPitch']:
            Tz = 0.1
            Ty = 0.098
        elif joint_name in ['LElbowYaw','RElbowYaw']:
            Ty = 0.015
            Tx = 0.105
        elif joint_name in ['LWristYaw','RWristYaw']:
            Tx = 0.05595
        elif joint_name in ['LElbowYaw','RElbowYaw']:
            Ty = 0.015
            Tx = 0.105
        elif joint_name in ['LHipYawPitch','RYawPitch']:
            Tz = -0.085
            Ty = 0.05
        elif joint_name in ['LKneePitch','RKneePitch']:
            Tz = -0.1
        elif joint_name in ['LAnklePitch','RAnklePitch']:
            Tz = -0.1029
        elif joint_name in ['LHipYawPitch','RHipYawPitch']:
            Tz = -0.085


        # Trage Translation in T ein
        T[0,3] = Tx
        T[1,3] = Ty
        T[2,3] = Tz

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

                T = T @ Tl  # Transformation aufbauen

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
