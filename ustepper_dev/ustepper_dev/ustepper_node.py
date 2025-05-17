import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import os
import yaml

from ament_index_python.packages import get_package_share_directory

# Constants
CMD_TERM = '\t'
FW_PROMPT = '>>> '

def load_rob_config():
    yaml_path = os.getenv("ROB_CENTRAL_CONFIG_PATH")
    if not yaml_path:  # fallback to default package share directory
        yaml_path = os.path.join(get_package_share_directory('rob_central'), 'config', 'rob_config.yaml')
    try:
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f"Failed to load config file: {yaml_path}\n{e}")

class SubControl(Node):
    def __init__(self):
        super().__init__('ustepper_controller_node')

        config = load_rob_config()
        port = config['ustepper_dev']['port']
        baudrate = config['ustepper_dev']['baudrate']
        timeout = config['ustepper_dev']['timeout']

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.ser.read_until(b"begin.")
            self.ser.write(b'b')  # send begin signal (hardware init)
            self.ser.read_until(FW_PROMPT.encode())
            self.get_logger().info(f"UstepperS32 connected on port {port} and ready for commands.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise RuntimeError("Serial port initialization failed") from e

        self.subscription = self.create_subscription(
            Float64,
            '/ustepper_control',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        cmd = f"{msg.data}{CMD_TERM}"
        if self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
                self.get_logger().debug(f"Sent command: {cmd.strip()}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write error: {e}")
        else:
            self.get_logger().warn("Serial port is not open")

    def close_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial port closed successfully.")
            except Exception as e:
                self.get_logger().warn(f"Exception on closing serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SubControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
