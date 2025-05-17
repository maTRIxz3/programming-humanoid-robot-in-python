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
import numpy as np



class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        joint_angles = [0] * 6  # 6 Gelenke: HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll

        # Segmentlängen
        L1 = 0.100     # Hüfte -> Knie
        L2 = 0.1029    # Knie -> Knöchel
        L3 = 0.04519   # Knöchel -> Boden (Fußhöhe)

        # Zielposition extrahieren (Fuß relativ zur Hüfte)
        target_pos = transform[:3, 3]
        x, y, z = target_pos[0], target_pos[1], target_pos[2] + L3  # Fußhöhe dazurechnen

        # Berechne Distanz Hüfte -> Fuß
        d = np.sqrt(x**2 + y**2 + z**2)

        # Kniewinkel mit Kosinussatz (Gelenk 3: KneePitch)
        try:
            knee_angle = np.arccos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))
        except ValueError:
            # Ziel ist unerreichbar -> Knie maximal strecken
            knee_angle = 0.0

        joint_angles[3] = -knee_angle  # LKneePitch ist negativ in NAO-Koord.

        # Beinlänge = L1 + L2 => berechne Hüftwinkel
        a = np.arctan2(-z, np.sqrt(x**2 + y**2))  # Richtung nach unten
        b = np.arccos((L1**2 + d**2 - L2**2) / (2 * L1 * d))  # Winkel im Oberschenkel

        hip_pitch = a + b  # Gelenk 2: LHipPitch
        joint_angles[2] = hip_pitch

        # Fuß muss Boden parallel sein → AnklePitch = - (HipPitch + KneePitch)
        joint_angles[4] = -(joint_angles[2] + joint_angles[3])

        # HipYawPitch, HipRoll, AnkleRoll = 0 (wir ignorieren Rotation um Yaw & Roll)
        joint_angles[0] = 0.0  # LHipYawPitch
        joint_angles[1] = np.arctan2(y, x)  # einfache Roll-Kompensation
        joint_angles[5] = -joint_angles[1]  # LAnkleRoll

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        # 1. Inverse Kinematik berechnen
        joint_angles = self.inverse_kinematics(effector_name, transform)

        # 2. Gelenknamen je nach Bein wählen
        if effector_name == 'LLeg':
            joint_names = [
                'LHipYawPitch',
                'LHipRoll',
                'LHipPitch',
                'LKneePitch',
                'LAnklePitch',
                'LAnkleRoll'
            ]
        elif effector_name == 'RLeg':
            joint_names = [
                'RHipYawPitch',
                'RHipRoll',
                'RHipPitch',
                'RKneePitch',
                'RAnklePitch',
                'RAnkleRoll'
            ]
        else:
            raise ValueError(f"Unbekannter Effektorname: {effector_name}")

        # 3. Zeit- und Winkelwerte aufbauen
        timestamps = [[0.0] for _ in joint_names]
        angle_values = [[float(angle)] for angle in joint_angles]

        # 4. Als Tupel speichern (damit angle_interpolation es versteht)
        self.keyframes = (joint_names, timestamps, angle_values)
        #self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
