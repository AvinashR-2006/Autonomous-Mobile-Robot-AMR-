#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalRelay(Node):
    """
    Relays Foxglove Studio's default goal topics (/move_base_simple/goal and /goal)
    to Nav2's standard input topic (/goal_pose).
    """
    def __init__(self):
        super().__init__('foxglove_goal_relay')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.sub1 = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.cb1, 10)
        self.sub2 = self.create_subscription(PoseStamped, '/goal', self.cb2, 10)
        self.get_logger().info("Foxglove Goal Relay active: forwarding /move_base_simple/goal and /goal -> /goal_pose")

    def cb1(self, msg):
        self.get_logger().info("Received goal from /move_base_simple/goal -> Forwarding to Nav2 /goal_pose")
        self.pub.publish(msg)

    def cb2(self, msg):
        self.get_logger().info("Received goal from /goal -> Forwarding to Nav2 /goal_pose")
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
