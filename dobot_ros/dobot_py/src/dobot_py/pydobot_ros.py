#!/usr/bin/env python
import rospy

from dobot_msgs.srv import *
from .ptpMode import PTPMode

class Dobot:

    def __init__(self, namespace = None):
        self.namespace = namespace

    def move_to(self, x, y, z, r):
        self._set_ptp_cmd(PTPMode.MOVL_XYZ.value, x, y, z, r)

    def suck(self, enable):
        self._set_end_effector_suction_cup(1, enable, 1)

    def grip(self, enable):
        self._set_end_effector_gripper(1, enable, 1)

    def speed(self, Velocity, Acceration, velocityRatio=100., accerationRatio=100.,):
        self._set_ptp_common_params(velocityRatio, accerationRatio, 1)
        self._set_ptp_coordinate_params(Velocity, Velocity, Acceration, Acceration, 1)

    def wait(self, ms):
        self._set_wait_cmd(ms)

    def pose(self):
        return self._get_pose()
    
    def home(self):
        self._set_home_cmd()

    def _set_ptp_cmd(self, ptpMode, x, y, z, r):
        """
        Args:
            uint8 ptpMode
            float32 x
            float32 y
            float32 z
            float32 r

        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return self._run(SetPTPCmd, ptpMode, x, y, z, r)

    def _set_end_effector_gripper(self, enableCtrl, grip, isQueued):
        """
        Args:
            uint8 enableCtrl
            uint8 grip
            bool isQueued
        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return self._run(SetEndEffectorGripper, enableCtrl, grip, isQueued)

    def _set_end_effector_suction_cup(self, enableCtrl, suck, isQueued):
        """
        Args:
            uint8 enableCtrl
            uint8 suck
            bool isQueued

        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return self._run(SetEndEffectorSuctionCup, enableCtrl, suck, isQueued)


    def _set_wait_cmd(self, timeout):
        """
        Args:
            uint32 timeout

        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return self._run(SetWAITCmd, timeout)

    def _get_pose(self):
        """
        Returns:
            int32 result
            float32 x
            float32 y
            float32 z
            float32 r
            float32[] jointAngle
        """
        return self._run(GetPose)

    def _set_ptp_common_params(self, velocityRatio, accerationRatio, isQueued):
        """
        Args:
            float32 velocityRatio
            float32 accelerationRatio
            bool isQueued

        Returns:
            int32 result
            uint64 queuedCmdIndex

        """
        return self._run(SetPTPCommonParams, velocityRatio, accerationRatio, isQueued)

    def _set_ptp_coordinate_params(self, xyzVelocity, rVelocity, xyzAcceration, rAcceration, isQueued):
        """
        Args:
            float32 xyzVelocity
            float32 rVelocity
            float32 xyzAcceleration
            float32 rAcceleration
            bool isQueued

        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return self._run(SetPTPCoordinateParams, xyzVelocity, rVelocity, xyzAcceration, rAcceration, isQueued)

    def _set_home_cmd(self):
        """
        Returns:
            int32 result
            uint64 queuedCmdIndex
        """
        return self._run(SetHOMECmd)

    def _run(self, command, *args, **kwargs):
        if self.namespace == None:
            rospy.wait_for_service('{}'.format(command.__name__))
            cmd = rospy.ServiceProxy('{}'.format(command.__name__), command)
        else:
            rospy.wait_for_service('/{}/{}'.format(self.namespace, command.__name__))
            cmd = rospy.ServiceProxy('/{}/{}'.format(self.namespace, command.__name__), command)
        response = cmd(*args, **kwargs)
        return response