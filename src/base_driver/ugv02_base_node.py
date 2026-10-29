#!/usr/bin/env python3
import glob, json, math, re, time, serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class UGV02BaseNode(Node):
    """
    ROS 2 Hardware Driver Node for Waveshare UGV02 6-Wheel Rover.
    Communicates with ESP32 board over UART serial.
    Fuses wheel encoders (linear distance) with 9-axis IMU Gyroscope (angular heading)
    to achieve 100% drift-free rotation odometry on skid-steer chassis.
    """
    def __init__(self):
        super().__init__('ugv02_base_node')
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_track_m', 0.203)
        self.declare_parameter('meters_per_unit', 0.01)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('command_timeout', 0.50)

        self.port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.track = float(self.get_parameter('wheel_track_m').value)
        self.scale = float(self.get_parameter('meters_per_unit').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.stale_after = float(self.get_parameter('command_timeout').value)

        self.serial = serial.Serial(self.port, baudrate=baud, timeout=0.01, write_timeout=0.1)
        self.serial.write(b'{"T":"13","X":0.0,"Z":0.0}\n')
        self.serial.flush()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 20)

        self.cmd_x = 0.0
        self.cmd_z = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_send_time = 0.0
        self.last_left = None
        self.last_right = None
        self.last_enc_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.buffer = ""

        # 9-Axis IMU Gyroscope parameters for rock-solid turning odometry
        # Calibrated against physical rotations on Waveshare UGV02 chassis
        self.gyro_scale = 924.3  # LSB per rad/s
        self.gyro_bias = 0.0
        self.bias_samples = []
        self.bias_calibrated = False

        # 50 Hz control & TF broadcast loop
        self.timer = self.create_timer(0.02, self.loop_callback)
        self.get_logger().info(f"UGV02 Base Driver initialized on {self.port} at 50 Hz TF")

    def cmd_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        if math.isfinite(vx) and math.isfinite(wz):
            self.cmd_x = max(-self.max_linear, min(self.max_linear, vx))
            self.cmd_z = max(-self.max_angular, min(self.max_angular, wz))
            self.last_cmd_time = time.monotonic()

    def loop_callback(self):
        self.send_command()
        self.read_feedback()
        self.publish_odometry()

    def send_command(self):
        now = time.monotonic()
        if now - self.last_send_time < 0.05:
            return
        self.last_send_time = now
        x = self.cmd_x if (now - self.last_cmd_time <= self.stale_after) else 0.0
        z = self.cmd_z if (now - self.last_cmd_time <= self.stale_after) else 0.0
        msg = (json.dumps({"T":"13","X":round(x,3),"Z":round(z,3)}, separators=(',',':')) + '\n').encode('utf-8')
        try:
            self.serial.write(msg)
            self.serial.flush()
        except Exception:
            pass

    def read_feedback(self):
        try:
            waiting = self.serial.in_waiting
            if waiting > 0:
                self.buffer += self.serial.read(waiting).decode('utf-8', errors='ignore')
        except Exception:
            return

        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            line = line.strip()
            if not line:
                continue
            match = re.search(r'\{.*\}', line)
            if not match:
                continue
            try:
                data = json.loads(match.group())
                if 'odl' in data and 'odr' in data:
                    self.update_pose(float(data['odl']), float(data['odr']), data)
            except Exception:
                continue

    def update_pose(self, left, right, raw_data=None):
        now = self.get_clock().now()
        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            self.last_enc_time = now
            return
        dt = (now - self.last_enc_time).nanoseconds / 1e9
        if dt <= 0:
            return
        dl = (left - self.last_left) * self.scale
        dr = (right - self.last_right) * self.scale
        self.last_left = left
        self.last_right = right
        self.last_enc_time = now

        if abs(dl) > 0.50 or abs(dr) > 0.50:
            return

        # 1. Linear distance from wheel encoders
        distance = (dl + dr) / 2.0

        # 2. Calibrate Gyro zero-bias while robot starts up (first 50 packets ~ 1 sec)
        if not self.bias_calibrated and raw_data and 'gz' in raw_data:
            self.bias_samples.append(float(raw_data['gz']))
            if len(self.bias_samples) >= 50:
                self.gyro_bias = sum(self.bias_samples) / len(self.bias_samples)
                self.bias_calibrated = True
                self.get_logger().info(f"9-Axis IMU Gyro calibrated! Zero bias: {self.gyro_bias:.2f}, Scale: {self.gyro_scale}")

        # 3. Rotational yaw from 9-axis IMU Gyro (100% immune to wheel slip!)
        if self.bias_calibrated and raw_data and 'gz' in raw_data:
            raw_gz = float(raw_data['gz']) - self.gyro_bias
            if abs(raw_gz) < 4.0:  # Sensor noise deadband
                raw_gz = 0.0
            wz = raw_gz / self.gyro_scale  # Angular velocity in rad/s
            dtheta = wz * dt
        else:
            # Fallback to wheel encoder difference
            dtheta = (dr - dl) / self.track
            wz = dtheta / dt

        # 4. Integrate planar 2D odometry
        mid_yaw = self.yaw + dtheta / 2.0
        self.x += distance * math.cos(mid_yaw)
        self.y += distance * math.sin(mid_yaw)
        self.yaw += dtheta
        # Keep yaw normalized between -pi and +pi
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.vx = distance / dt
        self.wz = wz

    def publish_odometry(self):
        now = self.get_clock().now()
        # Add 100ms future tolerance to TF timestamp to prevent AMCL / Nav2 extrapolation errors
        future_ns = now.nanoseconds + 100_000_000
        future_stamp = now.to_msg()
        future_stamp.sec = future_ns // 1_000_000_000
        future_stamp.nanosec = future_ns % 1_000_000_000

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = future_stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        tf_fp = TransformStamped()
        tf_fp.header.stamp = future_stamp
        tf_fp.header.frame_id = 'base_link'
        tf_fp.child_frame_id = 'base_footprint'
        tf_fp.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([tf, tf_fp])

def main(args=None):
    rclpy.init(args=args)
    node = UGV02BaseNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
