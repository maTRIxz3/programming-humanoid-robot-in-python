'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import leftBackToStand
import pickle
import logging
import os

#logging.basicConfig(filename="agent_log.txt", level=logging.DEBUG) #for debugging


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        model_path = os.path.join(os.path.dirname(__file__), 'robot_pose.pkl')
        self.posture_classifier = pickle.load(open(model_path, 'rb'))
        #self.posture_classifier = pickle.load(open('robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):

        self.posture = self.recognize_posture(perception)
        #logging.debug(self.posture) # debugging
        #print(self.posture)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        features = [
            perception.joint['LHipYawPitch'],
            perception.joint['LHipRoll'],
            perception.joint['LHipPitch'],
            perception.joint['LKneePitch'],
            perception.joint['RHipYawPitch'],
            perception.joint['RHipRoll'],
            perception.joint['RHipPitch'],
            perception.joint['RKneePitch'],
            perception.imu[0],  # AngleX
            perception.imu[1]   # AngleY
        ]

        # predict posture
        posture = self.posture_classifier.predict([features])[0]

        return posture # returns a Number matching the indices of the posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
