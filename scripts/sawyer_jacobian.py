#!/usr/bin/env python
import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser


class SawyerRobot:
    def _setup(self):
        # load in ros parameters
        self.baselink = rospy.get_param("sawyer/baselink")
        self.endlink = rospy.get_param("sawyer/endlink")
        flag, self.tree = kdl_parser.treeFromParam("/robot_description")

        self.joint_names = rospy.get_param("named_poses/right/joint_names")
        print(self.joint_names)

        # build kinematic chain and fk and jacobian solvers
        chain_ee = self.tree.getChain(self.baselink, self.endlink)
        self.fk_ee = kdl.ChainFkSolverPos_recursive(chain_ee)
        self.jac_ee = kdl.ChainJntToJacSolver(chain_ee)

        # building robot joint state
        self.num_joints = chain_ee.getNrOfJoints()
        self.joints = kdl.JntArray(self.num_joints)



    def update_joints(self, joint_msg):
        for i, n in enumerate(self.joint_names):
            index = joint_msg.name.index(n)
            self.joints[i] = joint_msg.position[index]



        if self.debug:
            self.debug_count += 1
            if (self.debug_count % 10) == 0:
                frame = kdl.Frame()
                self.fk_ee.JntToCart(self.joints, frame)
                print("frame translation:")
                print(frame.p)
                print("frame rotation:")
                print(frame.M)

                jacobian = kdl.Jacobian(self.num_joints)
                self.jac_ee.JntToJac(self.joints, jacobian)
                print("jacobian")
                print(jacobian)

    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            self.debug_count = 0

        rospy.init_node("sawyer_jacobian")
        rospy.Subscriber("/joint_states", JointState, self.update_joints)
        self._setup()

def main():
    sr = SawyerRobot(debug=True)
    rospy.spin()

if __name__ == "__main__":
    main()
