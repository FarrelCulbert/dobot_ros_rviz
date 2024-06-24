#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "dobot_api/DobotDll.h"

/*
 * Cmd timeout
 */
#include "dobot_msgs/SetCmdTimeout.h"

bool SetCmdTimeoutService(dobot_msgs::SetCmdTimeout::Request &req, dobot_msgs::SetCmdTimeout::Response &res)
{
    res.result = SetCmdTimeout(req.timeout);
    return true;
}

void InitCmdTimeoutServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;
    server = n.advertiseService("SetCmdTimeout", SetCmdTimeoutService);
    serverVec.push_back(server);
}

/*
 * Device information
 */
#include "dobot_msgs/GetDeviceSN.h"
#include "dobot_msgs/SetDeviceName.h"
#include "dobot_msgs/GetDeviceName.h"
#include "dobot_msgs/GetDeviceVersion.h"

bool GetDeviceSNService(dobot_msgs::GetDeviceSN::Request &req, dobot_msgs::GetDeviceSN::Response &res)
{
    char deviceSN[256];

    res.result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res.deviceSN.data = ss.str();
    }

    return true;
}

bool SetDeviceNameService(dobot_msgs::SetDeviceName::Request &req, dobot_msgs::SetDeviceName::Response &res)
{
    res.result = SetDeviceName(req.deviceName.data.c_str());

    return true;
}

bool GetDeviceNameService(dobot_msgs::GetDeviceName::Request &req, dobot_msgs::GetDeviceName::Response &res)
{
    char deviceName[256];

    res.result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName;
        res.deviceName.data = ss.str();
    }

    return true;
}

bool GetDeviceVersionService(dobot_msgs::GetDeviceVersion::Request &req, dobot_msgs::GetDeviceVersion::Response &res)
{
    uint8_t majorVersion, minorVersion, revision;

    res.result = GetDeviceVersion(&majorVersion, &minorVersion, &revision);
    if (res.result == DobotCommunicate_NoError) {
        res.majorVersion = majorVersion;
        res.minorVersion = minorVersion;
        res.revision = revision;
    }

    return true;
}

void InitDeviceInfoServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("GetDeviceSN", GetDeviceSNService);
    serverVec.push_back(server);
    server = n.advertiseService("SetDeviceName", SetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService("GetDeviceName", GetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService("GetDeviceVersion", GetDeviceVersionService);
    serverVec.push_back(server);
}

/*
 * Pose
 */
#include "dobot_msgs/GetPose.h"

bool GetPoseService(dobot_msgs::GetPose::Request &req, dobot_msgs::GetPose::Response &res)
{
    Pose pose;

    res.result = GetPose(&pose);
    if (res.result == DobotCommunicate_NoError) {
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.r = pose.r;
        for (int i = 0; i < 4; i++) {
            res.jointAngle.push_back(pose.jointAngle[i]);
        }
    }

    return true;
}

void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("GetPose", GetPoseService);
    serverVec.push_back(server);
}

/*
 * Alarms
 */
#include "dobot_msgs/GetAlarmsState.h"
#include "dobot_msgs/ClearAllAlarmsState.h"

bool GetAlarmsStateService(dobot_msgs::GetAlarmsState::Request &req, dobot_msgs::GetAlarmsState::Response &res)
{
    uint8_t alarmsState[128];
    uint32_t len;

    res.result = GetAlarmsState(alarmsState, &len, sizeof(alarmsState));
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < len; i++) {
            res.alarmsState.push_back(alarmsState[i]);
        }
    }

    return true;
}

bool ClearAllAlarmsStateService(dobot_msgs::ClearAllAlarmsState::Request &req, dobot_msgs::ClearAllAlarmsState::Response &res)
{
    res.result = ClearAllAlarmsState();

    return true;
}

void InitAlarmsServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("GetAlarmsState", GetAlarmsStateService);
    serverVec.push_back(server);
    server = n.advertiseService("ClearAllAlarmsState", ClearAllAlarmsStateService);
    serverVec.push_back(server);
}

/*
 * HOME
 */
#include "dobot_msgs/SetHOMEParams.h"
#include "dobot_msgs/GetHOMEParams.h"
#include "dobot_msgs/SetHOMECmd.h"

bool SetHOMEParamsService(dobot_msgs::SetHOMEParams::Request &req, dobot_msgs::SetHOMEParams::Response &res)
{
    HOMEParams params;
    uint64_t queuedCmdIndex;

    params.x = req.x;
    params.y = req.y;
    params.z = req.z;
    params.r = req.r;

    res.result = SetHOMEParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetHOMEParamsService(dobot_msgs::GetHOMEParams::Request &req, dobot_msgs::GetHOMEParams::Response &res)
{
    HOMEParams params;

    res.result = GetHOMEParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.x = params.x;
        res.y = params.y;
        res.z = params.z;
        res.r = params.r;
    }

    return true;
}

bool SetHOMECmdService(dobot_msgs::SetHOMECmd::Request &req, dobot_msgs::SetHOMECmd::Response &res)
{
    HOMECmd cmd;
    uint64_t queuedCmdIndex;

    res.result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("SetHOMEParams", SetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("GetHOMEParams", GetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("SetHOMECmd", SetHOMECmdService);
    serverVec.push_back(server);
}

/*
 * End effector
 */
#include "dobot_msgs/SetEndEffectorParams.h"
#include "dobot_msgs/GetEndEffectorParams.h"
#include "dobot_msgs/SetEndEffectorLaser.h"
#include "dobot_msgs/GetEndEffectorLaser.h"
#include "dobot_msgs/SetEndEffectorSuctionCup.h"
#include "dobot_msgs/GetEndEffectorSuctionCup.h"
#include "dobot_msgs/SetEndEffectorGripper.h"
#include "dobot_msgs/GetEndEffectorGripper.h"

bool SetEndEffectorParamsService(dobot_msgs::SetEndEffectorParams::Request &req, dobot_msgs::SetEndEffectorParams::Response &res)
{
    EndEffectorParams params;
    uint64_t queuedCmdIndex;

    params.xBias = req.xBias;
    params.yBias = req.yBias;
    params.zBias = req.zBias;

    res.result = SetEndEffectorParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorParamsService(dobot_msgs::GetEndEffectorParams::Request &req, dobot_msgs::GetEndEffectorParams::Response &res)
{
    EndEffectorParams params;

    res.result = GetEndEffectorParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xBias = params.xBias;
        res.yBias = params.yBias;
        res.zBias = params.zBias;
    }

    return true;
}

bool SetEndEffectorLaserService(dobot_msgs::SetEndEffectorLaser::Request &req, dobot_msgs::SetEndEffectorLaser::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorLaser(req.enableCtrl, req.on, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorLaserService(dobot_msgs::GetEndEffectorLaser::Request &req, dobot_msgs::GetEndEffectorLaser::Response &res)
{
    bool enableCtrl, on;

    res.result = GetEndEffectorLaser(&enableCtrl, &on);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.on = on;
    }

    return true;
}

bool SetEndEffectorSuctionCupService(dobot_msgs::SetEndEffectorSuctionCup::Request &req, dobot_msgs::SetEndEffectorSuctionCup::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorSuctionCup(req.enableCtrl, req.suck, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorSuctionCupService(dobot_msgs::GetEndEffectorSuctionCup::Request &req, dobot_msgs::GetEndEffectorSuctionCup::Response &res)
{
    bool enableCtrl, suck;

    res.result = GetEndEffectorLaser(&enableCtrl, &suck);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.suck = suck;
    }

    return true;
}

bool SetEndEffectorGripperService(dobot_msgs::SetEndEffectorGripper::Request &req, dobot_msgs::SetEndEffectorGripper::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorGripper(req.enableCtrl, req.grip, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorGripperService(dobot_msgs::GetEndEffectorGripper::Request &req, dobot_msgs::GetEndEffectorGripper::Response &res)
{
    bool enableCtrl, grip;

    res.result = GetEndEffectorLaser(&enableCtrl, &grip);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.grip = grip;
    }

    return true;
}

void InitEndEffectorServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("SetEndEffectorParams", SetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("GetEndEffectorParams", GetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("SetEndEffectorLaser", SetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService("GetEndEffectorLaser", GetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService("SetEndEffectorSuctionCup", SetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService("GetEndEffectorSuctionCup", GetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService("SetEndEffectorGripper", SetEndEffectorGripperService);
    serverVec.push_back(server);
    server = n.advertiseService("GetEndEffectorGripper", GetEndEffectorGripperService);
    serverVec.push_back(server);
}

/*
 * JOG
 */
#include "dobot_msgs/SetJOGJointParams.h"
#include "dobot_msgs/GetJOGJointParams.h"
#include "dobot_msgs/SetJOGCoordinateParams.h"
#include "dobot_msgs/GetJOGCoordinateParams.h"
#include "dobot_msgs/SetJOGCommonParams.h"
#include "dobot_msgs/GetJOGCommonParams.h"
#include "dobot_msgs/SetJOGCmd.h"

bool SetJOGJointParamsService(dobot_msgs::SetJOGJointParams::Request &req, dobot_msgs::SetJOGJointParams::Response &res)
{
    JOGJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGJointParamsService(dobot_msgs::GetJOGJointParams::Request &req, dobot_msgs::GetJOGJointParams::Response &res)
{
    JOGJointParams params;

    res.result = GetJOGJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCoordinateParamsService(dobot_msgs::SetJOGCoordinateParams::Request &req, dobot_msgs::SetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCoordinateParamsService(dobot_msgs::GetJOGCoordinateParams::Request &req, dobot_msgs::GetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;

    res.result = GetJOGCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCommonParamsService(dobot_msgs::SetJOGCommonParams::Request &req, dobot_msgs::SetJOGCommonParams::Response &res)
{
    JOGCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetJOGCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCommonParamsService(dobot_msgs::GetJOGCommonParams::Request &req, dobot_msgs::GetJOGCommonParams::Response &res)
{
    JOGCommonParams params;

    res.result = GetJOGCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetJOGCmdService(dobot_msgs::SetJOGCmd::Request &req, dobot_msgs::SetJOGCmd::Response &res)
{
    JOGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.isJoint = req.isJoint;
    cmd.cmd = req.cmd;
    res.result = SetJOGCmd(&cmd, false, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitJOGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("SetJOGJointParams", SetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("GetJOGJointParams", GetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("SetJOGCoordinateParams", SetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("GetJOGCoordinateParams", GetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("SetJOGCommonParams", SetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("GetJOGCommonParams", GetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("SetJOGCmd", SetJOGCmdService);
    serverVec.push_back(server);
}

/*
 * PTP
 */
#include "dobot_msgs/SetPTPJointParams.h"
#include "dobot_msgs/GetPTPJointParams.h"
#include "dobot_msgs/SetPTPCoordinateParams.h"
#include "dobot_msgs/GetPTPCoordinateParams.h"
#include "dobot_msgs/SetPTPJumpParams.h"
#include "dobot_msgs/GetPTPJumpParams.h"
#include "dobot_msgs/SetPTPCommonParams.h"
#include "dobot_msgs/GetPTPCommonParams.h"
#include "dobot_msgs/SetPTPCmd.h"

bool SetPTPCmdService(dobot_msgs::SetPTPCmd::Request &req, dobot_msgs::SetPTPCmd::Response &res)
{
    PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    res.result = SetPTPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitPTPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;
    server = n.advertiseService("SetPTPCmd", SetPTPCmdService);
    serverVec.push_back(server);
}

/*
 * Subscriber to /joint_states
 */
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() < 4) {
        ROS_WARN("Received joint state does not have enough positions.");
        return;
    }

    PTPCmd cmd;
    cmd.ptpMode = PTPMOVLXYZMode;
    cmd.x = msg->position[0]; // Assuming joint 0 corresponds to X
    cmd.y = msg->position[1]; // Assuming joint 1 corresponds to Y
    cmd.z = msg->position[2]; // Assuming joint 2 corresponds to Z
    cmd.r = msg->position[3]; // Assuming joint 3 corresponds to R

    uint64_t queuedCmdIndex;
    SetPTPCmd(&cmd, true, &queuedCmdIndex);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }

    // Connect Dobot before starting the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
            break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            return -3;
        default:
            return -4;
    }

    ros::init(argc, argv, "DobotServer");
    ros::NodeHandle n;

    std::vector<ros::ServiceServer> serverVec;

    InitCmdTimeoutServices(n, serverVec);
    InitDeviceInfoServices(n, serverVec);
    InitPoseServices(n, serverVec);
    InitAlarmsServices(n, serverVec);
    InitHOMEServices(n, serverVec);
    InitEndEffectorServices(n, serverVec);
    InitJOGServices(n, serverVec);
    InitPTPServices(n, serverVec);
    InitCPServices(n, serverVec);
    InitARCServices(n, serverVec);
    InitWAITServices(n, serverVec);
    InitTRIGServices(n, serverVec);
    InitEIOServices(n, serverVec);
    InitQueuedCmdServices(n, serverVec);

    // Subscribe to /joint_states topic
    ros::Subscriber sub = n.subscribe("/joint_states", 10, jointStateCallback);

    ROS_INFO("Dobot service running...");
    ros::spin();
    ROS_INFO("Dobot service exiting...");

    // Disconnect Dobot
    DisconnectDobot();

    return 0;
}
