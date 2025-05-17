import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Vector3

from omni_msgs.msg import OmniButtonEvent, OmniState, OmniFeedback
from dynamixel_sdk_custom_interfaces.msg import SetPosition, GetMotorState
from dynamixel_sdk_custom_interfaces.srv import GetCurrent

import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_rob_config():
    yaml_path = os.getenv("ROB_CENTRAL_CONFIG_PATH")
    if not yaml_path: # without the path in ENV
        yaml_path = os.path.join(get_package_share_directory('rob_central'), 'config', 'rob_config.yaml')
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

config = load_rob_config()

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min


class ReadWriteNode(Node):

    def __init__(self):
        super().__init__('rob_central_node')
        self.get_logger().warn("Hold the Geomagic Device to the center/comfortable posture, then press the grey button on haptic device to continue.")

        self._initialize_state()
        self._configure_motors()
        self._create_publishers()
        self._create_subscribers()
        self._create_clients()

        self._create_timers()
        
    def _initialize_state(self):
        self._get_init_haptic_dev_posture = False
        self._get_init_haptic_dev_joint = False

        self._enable_rotate_dynamixel_motor = False

        self.prev_time = None
        self.prev_positions = None

        self._haptic_dev_init_pos = np.zeros(3)
        self._haptic_dev_init_quat = np.zeros(4)
        self._haptic_dev_init_euler = np.zeros(3)
        self._haptic_dev_init_joint = np.zeros(6)

        self._absolute_joints = np.zeros(6)
        self._absolute_pos = np.zeros(3)
        self._haptic_vel_joints = np.zeros(6)
        self.virtual_joints_torque = np.zeros(6)

        self.dynamixel_current_reader = np.zeros(2) # for read current and update to motor list
        self._force_haptic_dev_joint = np.zeros(3)

        self.dynamixel_motor1_current = 0
        self.dynamixel_motor1_position = 0
        self.dynamixel_motor1_vel = 0

        self.tele_mode = config['rob_central']['tele_mode'] 
        # for controlling the continuum robot with different modes, details in config.yaml
        self.bend_motor_residual_current_range = config['rob_central']['dynamixel_residual_current_range']

    def _configure_motors(self):
        self.dynamixel_ids = config['rob_central']['dynmaixel_motors_id'] # 2 dynamixel motors
        self.dynamixel_motors = np.array([[self.dynamixel_ids[0], 0], [self.dynamixel_ids[1], 0]], dtype=float)
        self.dynamixel_motors_current = np.array([[self.dynamixel_ids[0], 0], [self.dynamixel_ids[1], 0]], dtype=float)

        self.dev_joint_threshold = np.array(config['rob_central']['joint_threshold'])
        self.dev_pos_threshold = np.array(config['rob_central']['position_threshold'])
        self.dev_rotation_map_dynamixel_range = np.array(config['rob_central']['rotation_map_dynamixel_range'])
        self.dev_bending_map_dynamixel_range = np.array(config['rob_central']['bending_map_dynamixel_range'])

        self.bend_dynamixel_range = np.array(config['rob_central']['bend_dynamixel_range'])
        self.rotate_dynamixel_range = np.array(config['rob_central']['rotate_dynamixel_range'])
        
        self.dynamixel_motors[1, 1] = np.mean(self.rotate_dynamixel_range)

        self.dev_pos_map_ustepper_range = np.array(config['rob_central']['pos_map_ustepper_range'])
        self.ustepper_vel_range = np.array(config['rob_central']['ustepper_vel_range'])
        self.ustepper_motors = np.mean(self.ustepper_vel_range)

        self.dynamixel_current_range = np.array(config['rob_central']['dynamixel_current_range'])
        self.haptic_dev_joint_force_range = np.array(config['rob_central']['haptic_dev_joint_force_range'])
        self.residual_current_map_range = np.array(config['rob_central']['residual_current_map_range'])

        self.haptic_joint_spring = np.array(config['geomagic_touch_dev']['spring'])
        self.haptic_joint_damping = np.array(config['geomagic_touch_dev']['damping'])

        self.dynamixel_max_torque = np.array(config['dynamixel_io']['max_torque'])
        self.dynamixel_max_current = np.array(config['dynamixel_io']['max_current'])
        self.dynamixel_max_pwm = np.array(config['dynamixel_io']['max_pwm'])
        self.dynamixel_K_t = self.dynamixel_max_torque / self.dynamixel_max_pwm # for calculate torque
        self.dynamixel_current_control_range = np.array(config['rob_central']['dynamixel_current_control_range'])
        self.damping_power = np.array(config['geomagic_touch_dev']['damping_power'])

        if self.tele_mode == 1 or self.tele_mode == 2:
            self.dynamixel_motors[0, 1] = np.mean(self.bend_dynamixel_range)
        elif self.tele_mode == 3:
            self.dynamixel_motors[0, 1] = np.mean(self.dynamixel_current_control_range)

    def _create_publishers(self):
        self.publisher_control_dynamixel_motors = self.create_publisher(SetPosition, "/dynamixel/set_position", 10)
        self.publisher_control_ustepper_motors = self.create_publisher(Float64, "ustepper_control", 10)
        self.publisher_feedbcak_force_to_haptic_dev = self.create_publisher(OmniFeedback, '/phantom/force_feedback', 10)

        self.publisher_debug = self.create_publisher(JointState, '/central_control/debug', 10)


    def _create_timers(self):
        timer_period = 0.001
        self.start_time = time.time()
        self.create_timer(timer_period, self._publish_dynamixel_rotation_motor)
        self.create_timer(timer_period, self._publish_dynamixel_bending_motor)
        self.create_timer(timer_period, self._publish_ustepper_motor)
        # self.create_timer(timer_period, self._call_dynamixel_current_service)
        if self.tele_mode == 1: # free mode / mode 1
            pass
        elif self.tele_mode == 2: # control loop / mode 2
            self.create_timer(timer_period, self._control_haptic_force_from_dynamixel_current) 
        elif self.tele_mode == 3: # control loop / mode 3
            self.create_timer(timer_period, self._control_haptic_force_from_residual_current) 

        self.create_timer(timer_period, self._publish_debug)

    def _create_clients(self):
        self.dynamixel_current_client = self.create_client(GetCurrent, '/dynamixel/get_current')
        while not self.dynamixel_current_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dynamixel/get_current...')

    def _create_subscribers(self):
        self.create_subscription(OmniButtonEvent, '/phantom/button', self._on_button_event, 10)
        self.create_subscription(OmniState, '/phantom/state', self._on_state_event, 10)
        self.create_subscription(JointState, '/phantom/joint_states', self._on_joint_state_event, 10)
        self.create_subscription(GetMotorState, '/dynamixel/id_1_motor_state', self._on_dynamixel_motor1_state, 10)

    def _on_button_event(self, msg):
        if msg.grey_button == 1: # pushed
            self._get_init_haptic_dev_joint = True
            self._get_init_haptic_dev_posture = True
        self._enable_rotate_dynamixel_motor = True if msg.white_button == 1 else False

    def _on_joint_state_event(self, msg):
        if not self._get_init_haptic_dev_joint:
            self._haptic_dev_init_joint[:] = msg.position[:6]
        else:
            self._absolute_joints[:] = self._haptic_dev_init_joint - msg.position[:6]
            self._absolute_joints[0] = -self._absolute_joints[0] # for intuitive control
            current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 # convert ros time to sec
            positions = msg.position
            positions_np = np.array(positions)
            if self.prev_positions is None:
                self.prev_positions = positions_np
                self.prev_time = current_time_sec
                return
            dt = current_time_sec - self.prev_time
            if dt <= 0.0:
                return
            if len(positions) != len(self.prev_positions):
                self.get_logger().warn("Joint count mismatch, skipping velocity calculation.")
                return
            self._haptic_vel_joints = (positions_np - self.prev_positions) / dt
            self.prev_positions = positions_np
            self.prev_time = current_time_sec

            # self.virtual_joints_torque = -self.haptic_joint_spring * self._absolute_joints \
            #                             -self.haptic_joint_damping * self._haptic_vel_joints
            
            self.virtual_joints_torque = -self.haptic_joint_spring * self._absolute_joints

            # self.virtual_joints_torque = -self.haptic_joint_spring * self._absolute_joints \
            #                             -self.haptic_joint_damping * np.sign(self._haptic_vel_joints) * abs(self._haptic_vel_joints) ** self.damping_power
            
            # self.get_logger().info(f'calculated virtual torque for Motor1 is {-self.virtual_joints_torque[0]} N.m')
            
            # test, calculate current
            # current_test = int(self.virtual_joints_torque[0] / self.dynamixel_K_t)
            # self.get_logger().info(f'calculated target current cmd for Motor1 is {current_test}')

    def _on_state_event(self, msg):
        if not self._get_init_haptic_dev_posture:
            self._haptic_dev_init_pos[:] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self._haptic_dev_init_quat[:] = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            self._haptic_dev_init_euler = R.from_quat(self._haptic_dev_init_quat).as_euler('zyx')
        else:
            self._absolute_pos[:] = self._haptic_dev_init_pos - np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            self._update_motor_targets()

    def _on_dynamixel_motor1_state(self, msg):
        self.dynamixel_motor1_current = msg.current * 2.69 * 1.1 # to mNm
        # self.dynamixel_motor1_current = msg.current
        # self.dynamixel_motor1_position = msg.position * (360 / 4096) # to degree
        self.dynamixel_motor1_position = msg.position # rad
        self.dynamixel_motor1_vel = msg.velocity * 0.229 # to rpm

    def _update_motor_targets(self):
        # Rotation Motor
        self.dynamixel_motors[1, 1] = self._map_motor_value(
            self._absolute_joints[-1],
            self.dev_joint_threshold,
            self.dev_rotation_map_dynamixel_range,
            self.rotate_dynamixel_range
        )

        # Bending Motor
        if self.tele_mode == 1 or self.tele_mode == 2: # position control
            self.dynamixel_motors[0, 1] = self._map_motor_value(
                self._absolute_joints[0],
                self.dev_joint_threshold,
                self.dev_bending_map_dynamixel_range,
                self.bend_dynamixel_range
            )
        elif self.tele_mode == 3:
            self.dynamixel_motors[0, 1] = int(-self.virtual_joints_torque[0] / self.dynamixel_K_t)
            self.dynamixel_motors[0, 1] = np.clip(self.dynamixel_motors[0, 1], *self.dynamixel_current_control_range)

        # uStepper Motor
        self.ustepper_motors = self._map_motor_value(
            self._absolute_pos[1],
            self.dev_pos_threshold,
            self.dev_pos_map_ustepper_range,
            self.ustepper_vel_range
        )

    def _map_motor_value(self, val, threshold, input_range, output_range):
        """
        mapping function 

        params:
        - val: current joint/position
        - threshold: block threshold [min, max]
        - input_range: control range
        - output_range: motor range

        return:
        - mapping value for motors
        """
        if threshold[0] < val < threshold[1]:
            return np.mean(output_range)

        val = np.clip(val, *input_range)

        if val > 0:
            return map_range(val, threshold[1], input_range[1], np.mean(output_range), output_range[1])
        else:
            return map_range(val, input_range[0], threshold[0], output_range[0], np.mean(output_range))

    def _publish_dynamixel_rotation_motor(self):
        if self._enable_rotate_dynamixel_motor is True:
            msg = SetPosition()
            msg.id = int(self.dynamixel_motors[1, 0])
            msg.position = int(self.dynamixel_motors[1, 1])
            self.publisher_control_dynamixel_motors.publish(msg)
        else:
            pass

    def _publish_dynamixel_bending_motor(self):
        msg = SetPosition()
        msg.id = int(self.dynamixel_motors[0, 0])
        msg.position = int(self.dynamixel_motors[0, 1])
        self.publisher_control_dynamixel_motors.publish(msg)
    
    def _publish_force_pos_to_haptic_dev(self, _force):
        msg = OmniFeedback()
        msg.position.x = float(0)
        msg.position.y = float(0)
        msg.position.z = float(0)
        msg.force.x = float(_force[0])
        msg.force.y = float(_force[1])
        msg.force.z = float(_force[2])
        self.publisher_feedbcak_force_to_haptic_dev.publish(msg)

    def _publish_ustepper_motor(self):
        msg = Float64()
        msg.data = float(self.ustepper_motors)
        self.publisher_control_ustepper_motors.publish(msg)

    def _publish_debug(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['virtual_joint1_torque', 'motor1_current', 'force_feedback_to_haptic_dev']
        t = time.time() - self.start_time
        msg.position = [
            float(self.virtual_joints_torque[0] / self.dynamixel_K_t), 
            float(-self.dynamixel_motor1_current), 
            float(self._force_haptic_dev_joint[0])
            ]
        msg.velocity = []
        msg.effort = []
        self.publisher_debug.publish(msg)

    def _call_dynamixel_current_service(self, motor_id):
        req = GetCurrent.Request()
        req.id = motor_id
        self.dynamixel_current_reader[0] = motor_id
        future = self.dynamixel_current_client.call_async(req)
        future.add_done_callback(self._handle_current_response)
    
    def _handle_current_response(self, future):
        try:
            result = future.result()
            # self.get_logger().info(f"[Current] Current: {result.current}")
            self.dynamixel_current_reader[1] = result.current
            idx = np.where(self.dynamixel_motors_current[:, 0] == self.dynamixel_current_reader[0])[0]
            if idx.size > 0:
                self.dynamixel_motors_current[idx[0], 1] = self.dynamixel_current_reader[1]

        except Exception as e:
            self.get_logger().error(f"Failed to call get_current: {e}")

    def _control_haptic_force_from_dynamixel_current(self):
        id = 1 # hard code, revised in future
        # !client current
        # self._call_dynamixel_current_service(motor_id=id)
        # id_current = self.dynamixel_motors_current[self.dynamixel_motors_current[:, 0] == id, 1]

        # self.get_logger().info(f"ID: {id}, Current: {id_current}")

        # ! joint force/torque control
        if id == 1: # rotate motor -> haptic dev's root joint
            # !sub current
            # root_joint_torque = map_range(self.dynamixel_motor1_current, self.dynamixel_current_range[0], self.dynamixel_current_range[1], self.haptic_dev_joint_force_range[0], self.haptic_dev_joint_force_range[1])
            root_joint_torque = self.dynamixel_K_t * self.dynamixel_motor1_current
            # !client current
            # root_joint_torque = map_range(id_current, self.dynamixel_current_range[0], self.dynamixel_current_range[1], self.haptic_dev_joint_force_range[0], self.haptic_dev_joint_force_range[1])
            self._force_haptic_dev_joint[0] = root_joint_torque * 1000 # to N/m
            self._publish_force_pos_to_haptic_dev(self._force_haptic_dev_joint)
    
    def _control_haptic_force_from_residual_current(self):
        id = 1 # hard code, revised in future
        ratio = 1.25
        # self.get_logger().info(f'current feedback is {self.dynamixel_motor1_current}')
        if id == 1: # rotate motor -> haptic dev's root joint
            feedback_joint_torque = self.dynamixel_K_t * (-self.dynamixel_motor1_current * ratio - (self.virtual_joints_torque[0] / self.dynamixel_K_t))
            self._force_haptic_dev_joint[0] = -self.virtual_joints_torque[0] * 1000  # to N/m
            self._publish_force_pos_to_haptic_dev(self._force_haptic_dev_joint)
        
def main(args=None):
    rclpy.init(args=args)
    node = ReadWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

