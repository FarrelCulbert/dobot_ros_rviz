#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# ROS publishers for each joint command and joint states
pub_joint1 = rospy.Publisher('/dobot/joint_1_position_controller/command', Float64, queue_size=10)
pub_joint2 = rospy.Publisher('/dobot/joint_2_position_controller/command', Float64, queue_size=10)
pub_joint3 = rospy.Publisher('/dobot/joint_3_position_controller/command', Float64, queue_size=10)
pub_joint4 = rospy.Publisher('/dobot/joint_4_position_controller/command', Float64, queue_size=10)
pub_joint4 = rospy.Publisher('/dobot/joint_5_position_controller/command', Float64, queue_size=10)
pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=10)

# Initial positions of joints
joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]

def cmd_vel_callback(msg):
    # Example conversion logic (adjust according to actual kinematics and your robot's configuration)
    # Here just directly mapping linear.x to joint1 for demonstration:
    joint_positions[0] = msg.linear.x
    joint_positions[1] = msg.angular.z

    # Publish individual joint commands
    pub_joint1.publish(Float64(joint_positions[0]))
    pub_joint2.publish(Float64(joint_positions[1]))

    # Publish joint states
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4', 'magician_joint_5']
    joint_state.position = joint_positions
    joint_state.velocity = [0, 0, 0, 0]  # Assuming static velocities for simplicity
    joint_state.effort = []
    pub_joint_states.publish(joint_state)

def main():
    rospy.init_node('cmd_vel_to_joint_state')
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
