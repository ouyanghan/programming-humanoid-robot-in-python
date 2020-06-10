'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''

import pickle
import numpy as np

from angle_interpolation import AngleInterpolationAgent
from keyframes import *


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('robot_pose.pkl', 'rb'))
        self.posture_class_joints = [
            'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
            'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch'
        ]

        self.known_postures = [
            'Sit', 'Knee', 'HeadBack', 'StandInit', 'Stand', 'Frog', 'Right', 'Belly', 'Crouch', 'Left', 'Back'
        ]

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        data = np.array([[perception.joint[j] for j in self.posture_class_joints] + perception.imu])
        posture = self.known_postures[int(self.posture_classifier.predict(data))]

        return posture


if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
