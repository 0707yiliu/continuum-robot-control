import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
import pandas as pd
import matplotlib.pyplot as plt

from collections import defaultdict

import numpy as np

bag_path = '/home/yi/rosbags/continuum_demonstration/rosbag_all_topic_20250519_194340'  # modify to target bag
target_topic = '/central_control/debug'       # topic want to plot

# init Reader
reader = SequentialReader()
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage_options, converter_options)

# get topic type
topic_types = reader.get_all_topics_and_types()
type_dict = {t.name: t.type for t in topic_types}

# loading
from rosidl_runtime_py.utilities import get_message
msg_type = get_message(type_dict[target_topic])

# get data list
timestamps = []
data_values = []
data_dict = defaultdict(list) 

while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == target_topic:
        if topic == '/phantom/joint_states' or topic == '/central_control/debug':
            msg = deserialize_message(data, JointState)
        else:
            msg = msg_type.deserialize(data)
        timestamps.append(t * 1e-9)  # to second
        for i, joint_name in enumerate(msg.name):
            data_dict[joint_name].append(msg.position[i])  # get each jointstate

# np.save(bag_path + '/NNinput_abs_joint0.npy', np.array(data_dict['abs_joint0']))
# np.save(bag_path + '/NNinput_joint0_vel.npy', np.array(data_dict['joint1_vel']))
# np.save(bag_path + '/NNoutput.npy', np.array(data_dict['motor1_current_to_torque']))

# plot
for joint_name, values in data_dict.items():
    plt.plot(timestamps[:len(values)], values, label=joint_name)  # each curve / label

# plt.plot(timestamps[:len(values)], np.array(data_dict['virtual_joint1_torque']) - np.array(data_dict['motor1_current_to_torque']))

plt.xlabel("Time [s]")
plt.ylabel("N.m") 
plt.title("Mapping Debug")
plt.legend()
plt.grid()
plt.show()