#!/usr/bin/env python
import rospy

from dobot_msgs.srv import *


def run(command, *args, **kwargs):
    rospy.wait_for_service('{}'.format(command.__name__))
    cmd = rospy.ServiceProxy('{}'.format(command.__name__), command)
    response = cmd(*args, **kwargs)
    return response

def clear_all_alarm_state():
    """
    Returns:
        int32 result
    """

    return run(ClearAllAlarmsState)

def get_alarm_state():
    """
    Returns:
        int32 result
        uint8[] alarmsState
    """
    return run(GetAlarmsState)

def get_arc_params():
    """
    Returns:
        int32 result
        float32 xyzVelocity
        float32 rVelocity
        float32 xyzAcceleration
        float32 rAcceleration
    """
    return run(GetARCParams)

def get_color_sensor():
    """
    Returns:
        int32 result
        uint8 r
        uint8 g
        uint8 b

    """
    return run(GetColorSensor)

def get_cp_cmd():
    """
    Returns:
        int32 result
        uint8 cpMode
        float32 x
        float32 y
        float32 z
        float32 velocity
    """
    return run(GetCPParams)

def get_cp_params():
    """
    Returns:
        int32 result
        float32 planAcc
        float32 junctionVel
        float32 acc
        uint8 realTimeTrack

    """
    return run(GetCPParams)

def get_device_name():
    """
    Returns:
        int32 result
        std_msgs/String deviceName
    """
    return run(GetDeviceName)

def get_device_sn():
    """
        int32 result
        std_msgs/String deviceSN

    """
    return run(GetDeviceSN)

def get_device_version():
    """
    Returns:
        int32 result
        uint8 majorVersion
        uint8 minorVersion
        uint8 revision

    """
    return run(GetDeviceVersion)

def get_end_effector_gripper():
    """
    Returns:
        int32 result
        uint8 enableCtrl
        uint8 grip
    """
    return run(GetEndEffectorGripper)

def get_end_effector_laser():
    """
    Returns:
        int32 result
        uint8 enableCtrl
        uint8 on
    """
    return run(GetEndEffectorLaser)

def get_end_effector_params():
    """
    Returns:
        int32 result
        float32 xBias
        float32 yBias
        float32 zBias
    """

    return run(GetEndEffectorParams)

def get_end_effector_suctioncup():
    """
    Returns:
        int32 result
        uint8 enableCtrl
        uint8 suck
    """
    return run(GetEndEffectorSuctionCup)

def get_home_params():
    """
    Returns:
        int32 result
        float32 x
        float32 y
        float32 z
        float32 r

    """
    return run(GetHOMEParams)

def get_infrared_sensor(infraredPort):
    """
    Args:
        int32 infraredPort
    Returns:
        int32 result
        uint8 value
    """
    return run(GetInfraredSensor, infraredPort)

def get_ioadc(address):
    """
    Args:
        uint8 address
    Returns:
        int32 result
        uint16 value
    """
    return run(GetIOADC, address)

def get_iodi(address):
    """
    Args:
        uint8 address
    Returns:
        int32 result
        uint8 level
    """
    return run(GetIODI, address)

def get_iodo(address):
    """
    Args:
        uint8 address
    Returns:
        int32 result
        uint8 level
    """
    return run(GetIODO, address)

def get_iomultiplexing(address):
    """
    Args:
        uint8 address
    Returns:
        int32 result
        uint8 multiplex
    """
    return run(GetIOMultiplexing, address)

def get_iopwm(address):
    """
    Args:
        uint8 address
    Returns:
        int32 result
        float32 frequency
        float32 dutyCycle
    """
    return run(GetIOPWM, address)

def get_jog_common_params():
    """
    Returns:
        int32 result
        float32 velocityRatio
        float32 accelerationRatio
    """
    return run(GetJOGCommonParams)

def get_jog_coordinate_params():
    """
    Returns:
        int32 result
        float32[] velocity
        float32[] acceleration
    """
    return run(GetJOGCoordinateParams)

def get_jog_joint_params():
    """
    Returns:
        int32 result
        float32[] velocity
        float32[] acceleration
    """
    return run(GetJOGJointParams)

def get_pose():
    """
    Returns:
        int32 result
        float32 x
        float32 y
        float32 z
        float32 r
        float32[] jointAngle
    """
    return run(GetPose)

def get_ptp_common_params():
    """
    Returns:
        int32 result
        float32 velocityRatio
        float32 accelerationRatio

    """
    return run(GetPTPCommonParams)

def get_ptp_coordinate_params():
    """
    Returns:
        int32 result
        float32 xyzVelocity
        float32 rVelocity
        float32 xyzAcceleration
        float32 rAcceleration
    """
    return run(GetPTPCoordinateParams)

def get_ptp_joint_params():
    """
    Returns:
        int32 result
        float32[] velocity
        float32[] acceleration
    """
    return run(GetPTPJointParams)

def get_ptp_jump_params():
    """
    Returns:
        int32 result
        float32 jumpHeight
        float32 zLimit
    """
    return run(GetPTPJumpParams)

def set_arc_cmd(x1, y1, z1, r1, x2, y2, z2, r2):
    """

    Args:
        float32 x1
        float32 y1
        float32 z1
        float32 r1
        float32 x2
        float32 y2
        float32 z2
        float32 r2

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetARCCmd, x1, y1, z1, r1, x2, y2, z2, r2)

def set_arc_params(xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued):
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
    return run(SetARCParams, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued)

def set_cmd_timeoout(timeout):
    """
    Args:
        uint32 timeout

    Returns:
        int32 result
    """
    return run(SetCmdTimeout, timeout)

def set_color_sensor(enableCtrl, colorPort):
    """
    Args:
        bool enableCtrl
        int32 colorPort
    Returns:
        int32 result
    """
    return run(SetColorSensor, enableCtrl, colorPort)

def set_cp_cmd(cpMode, x, y, z, velocity):
    """
    Args:
        uint8 cpMode
        float32 x
        float32 y
        float32 z
        float32 velocity

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetCPCmd, cpMode, x, y, z, velocity)

def set_cp_params(planAcc, junctionVel, acc, realTimeTrack, isQueued):
    """
    Args:
        float32 planAcc
        float32 junctionVel
        float32 acc
        uint8 realTimeTrack
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetCPParams, planAcc, junctionVel, acc, realTimeTrack, isQueued)

def set_device_name(deviceName):
    """
    Args:
        std_msgs/String deviceName

    Returns:
        int32 result
    """
    return run(SetDeviceName, deviceName)

def set_emotor(index, isEnabled, speed, isQueued):
    """
    Args:
        uint8 index
        uint8 isEnabled
        float32 speed
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetEMotor, index, isEnabled, speed, isQueued)

def set_end_effector_gripper(enableCtrl, grip, isQueued):
    """
    Args:
        uint8 enableCtrl
        uint8 grip
        bool isQueued
    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetEndEffectorGripper, enableCtrl, grip, isQueued)

def set_end_effector_laser(enableCtrl, on, isQueued):
    """
    Args:
        uint8 enableCtrl
        uint8 on
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetEndEffectorLaser, enableCtrl, on, isQueued)

def set_end_effector_params(xBias, yBias, zBias, isQueued):
    """
    Args:
        float32 xBias
        float32 yBias
        float32 zBias
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetEndEffectorParams, xBias, yBias, zBias, isQueued)

def set_end_effector_suctioncup(enableCtrl, suck, isQueued):
    """
    Args:
        uint8 enableCtrl
        uint8 suck
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetEndEffectorSuctionCup, enableCtrl, suck, isQueued)

def set_home_cmd():
    """
    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetHOMECmd)

def set_home_params(x, y, z, r, isQueued):
    """
    Args:
        float32 x
        float32 y
        float32 z
        float32 r
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetHOMEParams, x, y, z, r, isQueued)

def set_inflared_sensor(enableCtrl, infraredPort):
    """
    Args:
        bool enableCtrl
        int32 infraredPort

    Returns:
        int32 result
    """
    return run(SetInfraredSensor, enableCtrl, infraredPort)

def set_iodo(address, level, isQueued):
    """
    Args:
        uint8 address
        uint8 level
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex

    """
    return run(SetIODO, address, level, isQueued)


def set_iomultiplexing(address, multiplex, isQueued):
    """
    Args:
        uint8 address
        uint8 multiplex
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetIOMultiplexing, address, multiplex, isQueued)


def set_iopwm(address, frequency, dutyCycle, isQueued):
    """
    Args:
        uint8 address
        float32 frequency
        float32 dutyCycle
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex

    """
    return run(SetIOPWM, address, frequency, dutyCycle, isQueued)


def set_jog_cmd(isJoint, cmd):
    """
    Args:
        uint8 isJoint
        uint8 cmd

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetJOGCmd, isJoint, cmd)

def set_jog_common_params(velocityRatio, accerationRatio, isQueued):
    """
    Args:
        float32 velocityRatio
        float32 accelerationRatio
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetJOGCommonParams, velocityRatio, accerationRatio, isQueued)

def set_jog_coordinate_params(velocity, acceration, isQueued):
    """
    Args:
        float32[] velocity
        float32[] acceleration
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetJOGCoordinateParams, velocity, acceration, isQueued)

def set_jog_joint_params(velocity, acceration, isQueued):
    """
    Args:
        float32[] velocity
        float32[] acceleration
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetJOGJointParams, velocity, acceration, isQueued)

def set_ptp_cmd(ptpMode, x, y, z, r):
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
    return run(SetPTPCmd, ptpMode, x, y, z, r)

def set_ptp_common_params(velocityRatio, accerationRatio, isQueued):
    """
    Args:
        float32 velocityRatio
        float32 accelerationRatio
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex

    """
    return run(SetPTPCommonParams, velocityRatio, accerationRatio, isQueued)

def set_ptp_coordinate_params(xyzVelocity, rVelocity, xyzAcceration, rAcceration, isQueued):
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
    return run(SetPTPCoordinateParams, xyzVelocity, rVelocity, xyzAcceration, rAcceration, isQueued)

def set_ptp_joint_params(velocity, acceration, isQueued):
    """
    Args:
        float32[] velocity
        float32[] acceleration
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetPTPJointParams, velocity, acceration, isQueued)

def set_ptp_jump_params(jumpHeight, zLimit, isQueued):
    """
    Args:
        float32 jumpHeight
        float32 zLimit
        bool isQueued

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetPTPJumpParams, jumpHeight, zLimit, isQueued)

def set_queued_cmd_clear():
    """
    Returns:
        int32 result
    """
    return run(SetQueuedCmdClear)

def set_queued_cmd_force_stop_exec():
    """
    Returns:
        int32 result
    """
    return run(SetQueuedCmdForceStopExec)

def set_queued_cmd_start_exec():
    """
    Returns:
        int32 result
    """
    return run(SetQueuedCmdStartExec)

def set_queued_cmd_stop_exec():
    """
    Returns:
        int32 result
    """
    return run(SetQueuedCmdStopExec)

def set_trig_cmd(address, mode, condition, threhold):
    """
    Args:
        uint8 address
        uint8 mode
        uint8 condition
        uint16 threshold

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetTRIGCmd, address, mode, condition, threhold)

def set_wait_cmd(timeout):
    """
    Args:
        uint32 timeout

    Returns:
        int32 result
        uint64 queuedCmdIndex
    """
    return run(SetWAITCmd, timeout)


