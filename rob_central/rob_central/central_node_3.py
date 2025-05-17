import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

from omni_msgs.msg import OmniButtonEvent, OmniFeedback, OmniState
from sensor_msgs.msg import JointState

from dynamixel_sdk_custom_interfaces.msg import SetPosition

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

class ReadWriteNode(Node):

    def __init__(self):
        super().__init__('rob_central_node')

        self.get_logger().warn("Hold the Geomagic Device to the center/comfortable posture, then press any key to continue.")
        self._get_init_haptic_dev_posture = False
        self._get_init_haptic_dev_joint = False
        self._haptic_dev_init_pos = np.zeros(3)
        self._haptic_dev_init_quat = np.zeros(4)
        self._haptic_dev_init_euler = np.zeros(3)
        self._haptic_dev_init_joint = np.zeros(6)
        input()
        self._absolute_joints = np.zeros(6)
        self._absolute_pos = np.zeros(3)

        # data defined for controlling motors
        self.ustepper_motors = np.zeros(1) # hard code, the number of motors
        self.dynamixel_motors = np.zeros((2, 2)) # hard code, the number of motors, id and degree
        self.dynamixel_motors[0, 0] = 1
        self.dynamixel_motors[1, 0] = 2
        # configuration for rotation and bending
        self.dev_joint_threshold = np.array([-0.1, 0.1]) # hard code
        self.dev_pos_threshold = np.array([-30, 30]) # hard code
        self.dev_rotation_map_dynamixel_range = np.array([-1.5, 1.5]) # hard code
        self.dev_bending_map_dynamixel_range = np.array([-0.8, 0.8])
        
        self.bend_dynamixel_range = np.array([1750, 2450]) # hard code, range of bending motor
        self.rotate_dynamixel_range = np.array([910, 2910]) # hard code, range of rotation motor
        self.dynamixel_motors[0, 1] = np.mean(self.bend_dynamixel_range)
        self.dynamixel_motors[1, 1] = np.mean(self.rotate_dynamixel_range)
        # ustepper configuration
        self.dev_pos_map_ustepper_range = np.array([-60, 60])
        self.ustepper_vel_range = np.array([-30, 30])
        self.ustepper_motors = np.mean(self.ustepper_vel_range)

        # publisher port for controller motors
        self.publisher_control_dynamixel_motors = self.create_publisher(
            SetPosition, 
            "/dynamixel/set_position", 
            100
        )
        # self.dynamixel_motors_control_msg = SetPosition()
        timer_period = 0.01  # seconds
        self.timer_dynamixel_rotation_motor = self.create_timer(timer_period, self.timer_dynamixel_rotation_motor_callback)
        self.timer_dynamixel_bending_motor = self.create_timer(timer_period, self.timer_dynamixel_bending_motor_callback)

        # publisher port for controller motors
        self.publisher_control_ustepper_motors = self.create_publisher(
            Float64, 
            "ustepper_control", 
            100
        )
        self.timer_ustepper_motor = self.create_timer(timer_period, self.timer_ustepper_motor_callback)

        # subscriber port for getting info from Geomagic Device 
        self.subscription_haptic_dev_button = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.haptic_dev_button_callback,
            500
        )
        self.subscription_haptic_dev_button  # prevent unused variable warning

        self.subscription_haptic_dev_state = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.haptic_dev_state_callback,
            500
        )
        self.subscription_haptic_dev_state  # prevent unused variable warning

        self.subscription_haptic_dev_joint_state = self.create_subscription(
            JointState,
            '/phantom/joint_states',
            self.haptic_dev_joint_state_callback,
            500
        )
        self.subscription_haptic_dev_joint_state  # prevent unused variable warning


    def haptic_dev_button_callback(self, msg):
        print(msg.grey_button)

    def haptic_dev_joint_state_callback(self, msg):
        # print('position:', msg.pose.position.x, msg.pose.orientation.x)
        if self._get_init_haptic_dev_joint is False:
            for i in range(6):
                self._haptic_dev_init_joint[i] = msg.position[i]
            self._get_init_haptic_dev_joint = True
        
        for i in range(6):
            self._absolute_joints[i] = self._haptic_dev_init_joint[i] - msg.position[i]

    def haptic_dev_state_callback(self, msg):
        # print('position:', msg.pose.position.x, msg.pose.orientation.x)
        if self._get_init_haptic_dev_posture is False:
            self._haptic_dev_init_pos[0] = msg.pose.position.x
            self._haptic_dev_init_pos[1] = msg.pose.position.y
            self._haptic_dev_init_pos[2] = msg.pose.position.z

            self._haptic_dev_init_quat[0] = msg.pose.orientation.x
            self._haptic_dev_init_quat[1] = msg.pose.orientation.y
            self._haptic_dev_init_quat[2] = msg.pose.orientation.z
            self._haptic_dev_init_quat[3] = msg.pose.orientation.w

            self._haptic_dev_init_euler = R.from_quat(self._haptic_dev_init_quat).as_euler('zyx')

            self._get_init_haptic_dev_posture = True
        
        self._absolute_pos[0] = self._haptic_dev_init_pos[0] - msg.pose.position.x
        self._absolute_pos[1] = self._haptic_dev_init_pos[1] - msg.pose.position.y
        self._absolute_pos[2] = self._haptic_dev_init_pos[2] - msg.pose.position.z

        # print(self._haptic_dev_init_pos, self._haptic_dev_init_quat)
        # print(self._absolute_joints[0], self._absolute_joints[-1], self._absolute_pos[1])

        # # testing quaternion to euler -----------
        # self._haptic_dev_init_quat[0] = msg.pose.orientation.x
        # self._haptic_dev_init_quat[1] = msg.pose.orientation.y
        # self._haptic_dev_init_quat[2] = msg.pose.orientation.z
        # self._haptic_dev_init_quat[3] = msg.pose.orientation.w
        # euler_angle = R.from_quat(self._haptic_dev_init_quat).as_euler('zyx')
        # print(euler_angle)
        # #  ---------------------

        # dynamixel control, we have two dynamixel motors right now
        ## id = 1, 2
        ## range (raw data): straight motor id1->[1750-2100-2450]
        ##                   rotate motor id2->[910-1910-2910] 
        ## haptic mapping: rotation -> haptic dev (abs joint -1) -> [-1.5, 1.5], threshold: [-0.1, 0.1]
        ##                 bending -> haptic dev (abs joint 0) -> [-0.8, 0.8], threshold: [-0.1, 0.1]
        # map rotation motor
        if self.dev_joint_threshold[0] < self._absolute_joints[-1] < self.dev_joint_threshold[1]:
            self.dynamixel_motors[1, 1] = np.mean(self.rotate_dynamixel_range)
        else:
            clipped_abs_rotate_joint = np.clip(self._absolute_joints[-1], self.dev_rotation_map_dynamixel_range[0], self.dev_rotation_map_dynamixel_range[1])
            # print(clipped_abs_rotate_joint)
            if clipped_abs_rotate_joint > 0:
                self.dynamixel_motors[1, 1] = map_range(
                    clipped_abs_rotate_joint,
                    in_min=self.dev_joint_threshold[1],
                    in_max=self.dev_rotation_map_dynamixel_range[1],
                    out_min=np.mean(self.rotate_dynamixel_range),
                    out_max=self.rotate_dynamixel_range[1]
                )
            else:
                self.dynamixel_motors[1, 1] = map_range(
                    clipped_abs_rotate_joint,
                    in_min=self.dev_rotation_map_dynamixel_range[0],
                    in_max=self.dev_joint_threshold[0],
                    out_min=self.rotate_dynamixel_range[0],
                    out_max=np.mean(self.rotate_dynamixel_range)
                )
        # map bending motor
        if self.dev_joint_threshold[0] < self._absolute_joints[0] < self.dev_joint_threshold[1]:
            self.dynamixel_motors[0, 1] = np.mean(self.bend_dynamixel_range)
        else:
            clipped_abs_bending_joint = np.clip(self._absolute_joints[0], self.dev_bending_map_dynamixel_range[0], self.dev_bending_map_dynamixel_range[1])
            if clipped_abs_bending_joint > 0:
                self.dynamixel_motors[0, 1] = map_range(
                    clipped_abs_bending_joint,
                    in_min=self.dev_joint_threshold[1],
                    in_max=self.dev_bending_map_dynamixel_range[1],
                    out_min=np.mean(self.bend_dynamixel_range),
                    out_max=self.bend_dynamixel_range[1]
                )
            else:
                self.dynamixel_motors[0, 1] = map_range(
                    clipped_abs_bending_joint,
                    in_min=self.dev_bending_map_dynamixel_range[0],
                    in_max=self.dev_joint_threshold[0],
                    out_min=self.bend_dynamixel_range[0],
                    out_max=np.mean(self.bend_dynamixel_range)
                )
        # print(self.dynamixel_motors[0, 1], self.dynamixel_motors[1, 1])

        # map pos to ustepper motor
        if self.dev_pos_threshold[0] < self._absolute_pos[1] < self.dev_pos_threshold[1]:
            self.ustepper_motors = np.mean(self.ustepper_vel_range)
        else:
            clipped_abs_dev_pos = np.clip(self._absolute_pos[1], self.dev_pos_map_ustepper_range[0], self.dev_pos_map_ustepper_range[1])
            # print(clipped_abs_rotate_joint)
            if clipped_abs_dev_pos > 0:
                self.ustepper_motors = map_range(
                    clipped_abs_dev_pos,
                    in_min=self.dev_pos_threshold[1],
                    in_max=self.dev_pos_map_ustepper_range[1],
                    out_min=np.mean(self.ustepper_vel_range),
                    out_max=self.ustepper_vel_range[1]
                )
            else:
                self.ustepper_motors = map_range(
                    clipped_abs_dev_pos,
                    in_min=self.dev_pos_map_ustepper_range[0],
                    in_max=self.dev_pos_threshold[0],
                    out_min=self.ustepper_vel_range[0],
                    out_max=np.mean(self.ustepper_vel_range),
                )

        
    def timer_dynamixel_rotation_motor_callback(self):
        dynamixel_motors_control_msg = SetPosition()
        dynamixel_motors_control_msg.id = int(self.dynamixel_motors[0, 0])
        dynamixel_motors_control_msg.position = int(self.dynamixel_motors[0, 1])
        self.publisher_control_dynamixel_motors.publish(dynamixel_motors_control_msg)

    def timer_dynamixel_bending_motor_callback(self):
        dynamixel_motors_control_msg = SetPosition()
        dynamixel_motors_control_msg.id = int(self.dynamixel_motors[1, 0])
        dynamixel_motors_control_msg.position = int(self.dynamixel_motors[1, 1])
        self.publisher_control_dynamixel_motors.publish(dynamixel_motors_control_msg)

    def timer_ustepper_motor_callback(self):
        ustepper_motor_control_msg = Float64()
        ustepper_motor_control_msg.data = self.ustepper_motors.astype(np.float64)
        self.publisher_control_ustepper_motors.publish(ustepper_motor_control_msg)


def main(args=None):
    
    rclpy.init(args=args)
    node = ReadWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # print('success to use central node.')

if __name__ == '__main__':
    main()
