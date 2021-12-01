#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# ros2 action list -t
# ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FollowJointTrajectory

class HomePoseClient(Node):

    def __init__(self):
        super().__init__('wheel_steer_actionclient')
        self._action_client = ActionClient(self, FollowJointTrajectory,
                                           '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, pose):
        goal_msg = FollowJointTrajectory.Goal()
        # Fill in data for trajectory
        joint_names = ["joint1",
                       "joint2",
                       "joint3",
                       "joint4",
                       "gripper"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(seconds=3, nanoseconds=0).to_msg()
        point1.positions = [float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4])]
        print([float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3]), float(pose[4])])
        points.append(point1)

        goal_msg.goal_time_tolerance = Duration(seconds=2, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedback:'+str(feedback))


def main(args=None):
    rclpy.init()

    action_client = HomePoseClient()

    # pose per joint should be inputted as separate arguments
    # for example: $ros2 run agrobot_control arm_custom_pose.py 0.0 0.0 0.0 0.0 0.0
    input = sys.argv[1:]
    pose = [float(item) for item in input]
    future = action_client.send_goal(pose)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()