#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

class JointStateMonitor:
    def __init__(self):
        self.joint_names = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4', 'magician_joint_5']
        self.last_positions = {name: None for name in self.joint_names}
        rospy.init_node('joint_state_listener', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

    def joint_states_callback(self, data):
        # This function checks if there are changes in the joint positions
        for name, position in zip(data.name, data.position):
            if name in self.last_positions:
                # Only print and update if the position has changed
                if self.last_positions[name] is None or not self.are_positions_equal(self.last_positions[name], position):
                    rospy.loginfo(f"Position of {name} changed to: {position}")
                    self.last_positions[name] = position

    @staticmethod
    def are_positions_equal(pos1, pos2, tolerance=0.01):
        # Check if positions are the same within a tolerance
        return abs(pos1 - pos2) < tolerance

    def run(self):
        rospy.spin()  # Keep the node running


if __name__ == '__main__':
    monitor = JointStateMonitor()
    monitor.run()
