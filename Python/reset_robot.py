import sys
import numpy as np
import math

import rtde_receive
import rtde_control

sys.path.append('libraries/')


class Reset_Robot():
    """
    This class contains the functions that communicate with the robotic arm.
    """
    def __init__(self) -> None:
        self.IP_robot = "192.168.1.22"
        self.speed_J = 0.3
        self.acc_J = 1
        # self.speed_L = 0.01
        # self.acc_L = 0.7

        self.rtde_c = rtde_control.RTDEControlInterface(self.IP_robot)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.IP_robot)

    def move_arm_to_position(self, pos, speed_L = 0.01, acc_L = 0.7,  flag=False):
        """
        Move end-effector to a chosen Cartesian position.
        """
        self.rtde_c.moveL(pos, speed_L, acc_L, flag)

    def move_arm_joints(self, pos, flag=False):
        """
        Move arm joints to a chosen position.
        """
        self.rtde_c.moveJ(pos, self.speed_J, self.acc_J, flag)

    def change_arm(self, pull_vec, vel_or_pos):
        """
        Move arm by a chosen position displacement OR move at a chosen speed.
        """
        if vel_or_pos == 0:
            pos = self.get_arm_speed()
            pos[0] = pull_vec[0]
            pos[1] = pull_vec[1]
            pos[2] = pull_vec[2]
        else:
            pos = self.get_arm_pose()
        pos[0] += pull_vec[0]
        pos[1] += pull_vec[1]
        pos[2] += pull_vec[2]
        return pos

    def move_arm(self, cartesian_speed, acc=0.25):
        """
        Move arm at a chosen speed (Cartesian).
        """
        self.rtde_c.speedL(cartesian_speed, acc)

    def stop_arm(self, acc=10):
        """
        Stop arm in current position.
        """
        self.rtde_c.speedStop(acc)

    def get_arm_speed(self):
        """
        Get the speed of the end-effector (Cartesian).
        """
        return self.rtde_r.getActualTCPSpeed()

    def get_arm_pose(self):
        """
        Get the end-effector's pose (Cartesian).
        """
        return self.rtde_r.getActualTCPPose()

    def get_joints(self):
        """
        Get current joint angles.
        """
        return self.rtde_r.getActualQ()

    def DH_matrix(self, robot):
        """
        Denavit-Hartenberg matrixes for the UR5 and UR3.
        """
        [q1, q2, q3, q4, q5, q6] = self.get_joints()
        if robot == 'UR3':
                             #   d|theta|a|alpha|
            DH = np.array([[0.15185, q1, 0, math.pi/2],
                           [0, q2, -0.24355, 0],
                           [0, q3, -0.2132, 0],
                           [0.13105, q4, 0, math.pi/2],
                           [0.08535, q5, 0, -math.pi/2],
                           [0.0921, q6, 0, 0]])
        elif robot == 'UR5':
                             #   d|theta|a|alpha|
            DH = np.array([[0.1625, q1, 0, math.pi/2],
                           [0, q2, -0.425, 0],
                           [0, q3, -0.3922, 0],
                           [0.1333, q4, 0, math.pi/2],
                           [0.0997, q5, 0, -math.pi/2],
                           [0.0996, q6, 0, 0]])
        return DH

    def DHTranf(self, p):
        """
        Obtain trasnformation matrix.
        """
        # DHTransf Returns the symbolic D-H joint transformation matrix
        # p = [d v a alpha]
        return np.array([[np.cos(p[1]), -np.sin(p[1])*np.cos(p[3]),  np.sin(p[1])*np.sin(p[3]), p[2]*np.cos(p[1])],
                         [np.sin(p[1]),  np.cos(p[1])*np.cos(p[3]), -np.cos(p[1])*np.sin(p[3]), p[2]*np.sin(p[1])],
                         [           0,               np.sin(p[3]),               np.cos(p[3]),              p[0]],
                         [           0,                          0,                          0,                 1]])

    def get_R06(self, robot):
        """
        Obtain rotation matrix.
        """
        DH = self.DH_matrix(robot)
        A01 = self.DHTranf(DH[0][:])
        A12 = self.DHTranf(DH[1][:])
        A23 = self.DHTranf(DH[2][:])
        A34 = self.DHTranf(DH[3][:])
        A45 = self.DHTranf(DH[4][:])
        A56 = self.DHTranf(DH[5][:])
        A06 = A01@A12@A23@A34@A45@A56
        return A06[0:3,0:3]
