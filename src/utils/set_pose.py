#!/usr/bin/env python3
import sys, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose(x=0.0, y=0.0, yaw_deg=0.0):
    """
    Publishes an initial pose directly to /initialpose for AMCL localization.
    Useful for testing or resetting pose from the terminal.
    """
    rclpy.init()
    node = Node('set_initial_pose_cli')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
    # Wait for publisher connection
    rclpy.spin_once(node, timeout_sec=0.5)
    
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    
    yaw_rad = math.radians(float(yaw_deg))
    msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    
    cov = [0.0] * 36
    cov[0] = 0.25   # x variance
    cov[7] = 0.25   # y variance
    cov[35] = 0.10  # yaw variance (~18 deg)
    msg.pose.covariance = cov
    
    for _ in range(3):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print(f"\n[OK] Sent initial pose to AMCL: x={x}m, y={y}m, yaw={yaw_deg}° in frame 'map'")
    print("Check Foxglove: The robot should now be at this position!\n")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.7
    y = float(sys.argv[2]) if len(sys.argv) > 2 else -0.2
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    set_initial_pose(x, y, yaw)
