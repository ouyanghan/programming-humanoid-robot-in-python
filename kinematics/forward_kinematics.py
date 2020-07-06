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
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
from numpy import cos, sin

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        # http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
        # note: Simulation does not seem to have the WristYaw and Hand joints!
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LKneePitch', 'LAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RKneePitch', 'RAnkleRoll']
        }
        
        # http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
        # y values are mirrored for right side
        self.jointLengths = {
            'HeadYaw': [0., 0., 126.5],
            'HeadPitch': [0., 0., 0.],
            'LShoulderPitch': [0., 98., 100.],
            'LShoulderRoll': [0., 0., 0.],
            'LElbowYaw': [105., 15., 0.],
            'LElbowRoll': [0., 0., 0.],
            'LHipYawPitch': [0., 50., -85.],
            'LHipRoll': [0., 0., 0.],
            'LHipPitch': [0., 0., 0.],
            'LKneePitch': [0., 0., -100.],
            'LAnklePitch': [0., 0., -102.9],
            'LAnkleRoll': [0., 0., 0.],
            'RShoulderPitch': [0., -98., 100.],
            'RShoulderRoll': [0., 0., 0.],
            'RElbowYaw': [105., -15., 0.],
            'RElbowRoll': [0., 0., 0.],
            'RHipYawPitch': [0., -50., -85.],
            'RHipRoll': [0., 0., 0.],
            'RHipPitch': [0., 0., 0.],
            'RKneePitch': [0., 0., -100.],
            'RAnklePitch': [0., 0., -102.9],
            'RAnkleRoll': [0., 0., 0.]
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
        # YOUR CODE HERE
        
        # 2D rotation matrix
        c, s = cos(joint_angle), sin(joint_angle)
        R = matrix([[c, -s], [s, c]])
        
        # Place according to rotation axis
        if joint_name.endswith('Roll'):
            T[1:3, 1:3] = R
        elif joint_name.endswith('Pitch'):
            T[0:3:2, 0:3:2] = R.T
        elif joint_name.endswith('Yaw'):
            T[0:2, 0:2] = R.T
            
        for i in range(3):
            T[i, 3] = self.jointLengths[joint_name][i]
            
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

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
