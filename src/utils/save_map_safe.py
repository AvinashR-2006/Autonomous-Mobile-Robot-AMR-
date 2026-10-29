#!/usr/bin/env python3
import sys, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

class SafeMapSaver(Node):
    """
    Fail-safe map saver node for ROS 2 Jazzy.
    Solves map_saver_cli timeout issues by subscribing to /map with both
    Transient Local and Volatile QoS profiles, then directly converts the
    OccupancyGrid into standard Nav2 P5 binary PGM and YAML metadata.
    """
    def __init__(self, output_prefix="/home/robopi/my_room_map"):
        super().__init__('safe_map_saver')
        self.output_prefix = output_prefix
        self.map_received = False
        
        qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.sub_tl = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_transient)
        self.sub_vol = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_volatile)
        self.get_logger().info("Listening on /map topic (Transient Local and Volatile QoS)...")

    def map_cb(self, msg: OccupancyGrid):
        if self.map_received:
            return
        self.map_received = True
        self.get_logger().info(f"Received /map! Dimensions: {msg.info.width} x {msg.info.height}, resolution: {msg.info.resolution} m/cell")
        self.save_pgm_yaml(msg)
        
    def save_pgm_yaml(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        oz = msg.info.origin.position.z
        
        pgm_path = f"{self.output_prefix}.pgm"
        yaml_path = f"{self.output_prefix}.yaml"
        
        # Convert occupancy values to PGM bytes
        # OccupancyGrid: row 0 is bottom (y=0), row h-1 is top (y=max)
        # PGM image: row 0 is top -> invert rows
        pixels = bytearray(w * h)
        data = msg.data
        occ_thresh = 65
        free_thresh = 25
        
        for y in range(h):
            src_row = (h - 1 - y) * w
            dst_row = y * w
            for x in range(w):
                val = data[src_row + x]
                if val < 0:
                    pixels[dst_row + x] = 205 # unknown (gray)
                elif val >= occ_thresh:
                    pixels[dst_row + x] = 0   # occupied (black)
                elif val <= free_thresh:
                    pixels[dst_row + x] = 254 # free (white)
                else:
                    pixels[dst_row + x] = int(255 - (val * 255 // 100))
        
        # Write P5 binary PGM
        header = f"P5\n{w} {h}\n255\n".encode('ascii')
        with open(pgm_path, 'wb') as f:
            f.write(header)
            f.write(pixels)
            
        # Write YAML metadata
        pgm_basename = os.path.basename(pgm_path)
        yaml_content = f"""image: {pgm_basename}
mode: trinary
resolution: {res:.6f}
origin: [{ox:.6f}, {oy:.6f}, {oz:.6f}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)
            
        self.get_logger().info(f"SUCCESSFULLY SAVED MAP:")
        self.get_logger().info(f"  -> {pgm_path} ({os.path.getsize(pgm_path)} bytes)")
        self.get_logger().info(f"  -> {yaml_path}")
        print("\n" + "="*50)
        print(f" MAP SAVED SUCCESSFULLY TO {pgm_path} !")
        print("="*50 + "\n")

def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "/home/robopi/my_room_map"
    rclpy.init()
    node = SafeMapSaver(output_prefix=target)
    start_time = time.time()
    while rclpy.ok() and not node.map_received:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - start_time > 15.0:
            node.get_logger().error("Timeout after 15s waiting for /map message.")
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
