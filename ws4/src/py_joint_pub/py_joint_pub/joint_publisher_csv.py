import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool          # ← for LED on/off
import pkg_resources
import numpy as np


class JointPublisherCSV(Node):

    def __init__(self):
        super().__init__('joint_publisher_csv')

        # Joint state publisher
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # LED publisher  (True = ON, False = OFF)
        self.led_pub_ = self.create_publisher(Bool, 'led_state', 10)

        timer_period = 0.002   # 500 Hz = 1/500 seconds  (matches dt=0.002 in CSV)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # ── FIX 1: change filename here ──────────────────────
        # filename = 'tyagi55_bonus_heart.csv'   # ← your bonus CSV
        filename = 'tyagi55_final.csv'       # ← swap to this for the regular Lissajous

        csv_file = pkg_resources.resource_filename(
            'py_joint_pub', f'../resource/{filename}')
        self.get_logger().info(f"Loading CSV: {csv_file}")

        # ── FIX 2: skip_header=0 because your CSV has NO header ──
        self.csv_data = np.genfromtxt(csv_file, delimiter=',', skip_header=0)

        self.data_length = len(self.csv_data)
        self.get_logger().info(
            f"Loaded {self.data_length} rows × "
            f"{self.csv_data.shape[1]} columns from {filename}")

        # Confirm columns
        # col 0 = time
        # col 1-6 = joint angles (rad)
        # col 7 = LED  (1.0 = ON,  0.0 = OFF)
        self.get_logger().info(
            f"First row: time={self.csv_data[0,0]:.4f}  "
            f"j1={self.csv_data[0,1]:.4f}  "
            f"led={int(self.csv_data[0,7])}")

    def timer_callback(self):

        row = self.csv_data[self.i]

        # ── Publish joint angles ──────────────────────────────
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        msg.position = row[1:7].tolist()   # columns 1-6
        msg.velocity = []
        msg.effort   = []
        self.publisher_.publish(msg)

        # ── FIX 3: Publish LED state from column 7 ───────────
        led_msg = Bool()
        led_msg.data = bool(row[7] == 1.0)   # True = LED ON, False = LED OFF
        self.led_pub_.publish(led_msg)

        # Log LED state changes (only when it switches)
        if self.i > 0:
            prev_led = self.csv_data[self.i - 1, 7]
            curr_led = row[7]
            if prev_led != curr_led:
                state = "ON" if curr_led == 1.0 else "OFF"
                self.get_logger().info(
                    f"LED turned {state} at t={row[0]:.3f}s  (row {self.i})")

        # Advance index, loop back to start when done
        self.i += 1
        self.i %= self.data_length


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisherCSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
