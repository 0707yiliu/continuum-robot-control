#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/get_motor_state.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_current.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126
#define ADDR_PRESENT_VELOCITY 128 
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_VELOCITY 104
#define ADDR_GOAL_PWM 100


// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
// #define DEVICE_NAME "/dev/ttyACM1"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

const char* env_path = std::getenv("ROB_CENTRAL_CONFIG_PATH");

struct Config{
  std::string port;
  int control_mode;
};

Config load_config()
{
    Config config;
    std::string yaml_path;
    if (env_path) {
      yaml_path = std::string(env_path);
    } else {
      yaml_path = ament_index_cpp::get_package_share_directory("rob_central") + "/config/rob_config.yaml";

    }
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "the ymal path for dynamixel motor is %s", yaml_path.c_str());
    try {
        YAML::Node config_node = YAML::LoadFile(yaml_path);
        config.port = config_node["dynamixel_io"]["port"].as<std::string>();
        config.control_mode = config_node["rob_central"]["tele_mode"].as<int>();
        
    } catch (const std::exception &e) {
        std::cerr << "Failed to load config for dynamixel io: " << e.what() << std::endl;
    }
    return config;
}

ReadWriteNode::ReadWriteNode(int control_mode)
: Node("read_write_node"), control_mode_(control_mode)
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
  this->create_subscription<SetPosition>(
    "/dynamixel/set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;
      int dxl_comm_result = COMM_TX_FAIL;

      // switch control mode for ID 1 motor
      if (msg->id == 1) {
        switch (control_mode_) {
          case 1:  // position control
          case 2:
          {
            uint32_t goal_position = static_cast<uint32_t>(msg->position);
            dxl_comm_result = packetHandler->write4ByteTxRx(
              portHandler,
              msg->id,
              ADDR_GOAL_POSITION,
              goal_position,
              &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            else if (dxl_error != 0)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            else
              RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
            break;
          }

          // case 3:  // current control
          case 4:
          {
            int16_t desired_current = static_cast<int16_t>(msg->position);
            if (desired_current < -1193) desired_current = -1193;
            if (desired_current >  1193) desired_current = 1193;

            dxl_comm_result = packetHandler->write2ByteTxRx(
                portHandler,
                msg->id,
                ADDR_GOAL_CURRENT,
                desired_current,
                &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            else if (dxl_error != 0)
              RCLCPP_WARN(this->get_logger(), "%s, %d", packetHandler->getRxPacketError(dxl_error));
            else
              RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Current: %d]", msg->id, desired_current);
            break;
          }

          case 5:  // velocity control
          {
            int32_t goal_velocity = static_cast<int32_t>(msg->position);
            dxl_comm_result = packetHandler->write4ByteTxRx(
              portHandler,
              msg->id,
              ADDR_GOAL_VELOCITY,
              goal_velocity,
              &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            else if (dxl_error != 0)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            else
              RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", msg->id, goal_velocity);
            break;
          }

          case 3:  // current control
          case 6:  // PMW control
          {
            int16_t goal_pwm = static_cast<int16_t>(msg->position);
            dxl_comm_result = packetHandler->write2ByteTxRx(
              portHandler,
              msg->id,
              ADDR_GOAL_PWM, 
              goal_pwm,
              &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            else if (dxl_error != 0)
              RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            else
              RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal PWM: %d]", msg->id, goal_pwm);
            break;
          }

          default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported control_mode: %d", control_mode_);
        }
      }
      // position control for motor ID != 1
      else {
        uint32_t goal_position = static_cast<uint32_t>(msg->position);
        dxl_comm_result = packetHandler->write4ByteTxRx(
          portHandler,
          msg->id,
          ADDR_GOAL_POSITION,
          goal_position,
          &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
          RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
        else if (dxl_error != 0)
          RCLCPP_WARN(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
        else
          RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      }
    }
  );

  // uint8_t dxl_id =1;
  rotator_motor_state_publisher = this->create_publisher<GetMotorState>(
    "/dynamixel/id_1_motor_state", QOS_RKL10V);
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ReadWriteNode::pub_motor1_callback, this));

  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("/dynamixel/get_position", get_present_position);

  auto get_present_current =
    [this](
    const std::shared_ptr<GetCurrent::Request> request,
    std::shared_ptr<GetCurrent::Response> response) -> void
    {
      dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_CURRENT,
        reinterpret_cast<uint16_t *>(&present_current),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Current: %d]",
        request->id,
        present_current
      );

      response->current = present_current;
    };

  get_current_server_ = create_service<GetCurrent>("/dynamixel/get_current", get_present_current);
}

ReadWriteNode::~ReadWriteNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down read_write_node");
}

void ReadWriteNode::pub_motor1_callback()
{
  dxl_comm_result = packetHandler->read2ByteTxRx(
    portHandler, 
    1, 
    ADDR_PRESENT_CURRENT,
    (uint16_t*)&present_current, &dxl_error);
  
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, 
    1, 
    ADDR_PRESENT_POSITION,
    (uint32_t*)&present_position, &dxl_error);
  
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, 
    1, 
    ADDR_PRESENT_VELOCITY,
    (uint32_t*)&present_velocity, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Failed to read current: %s",
      packetHandler->getTxRxResult(dxl_comm_result));
    return;
  }

  GetMotorState msg;
  msg.current = static_cast<int16_t>(present_current);
  msg.position = static_cast<int32_t>(present_position);
  msg.velocity = static_cast<int32_t>(present_velocity);
  rotator_motor_state_publisher->publish(msg);
}

void setupDynamixel(uint8_t dxl_id, int mode)
{
  uint8_t operating_mode = 3;  // Default Position Control Mode
  bool is_mode_valid = true;

  if (dxl_id == 1) {
    switch (mode) {
      case 1:
      case 2:
        operating_mode = 3; // Position Control Mode
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Setting to Position Control Mode.", dxl_id);
        break;
      
      case 4:
        operating_mode = 0; // Current Control Mode
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Setting to Current Control Mode.", dxl_id);
        break;
      case 5:
        operating_mode = 1; // Velocity Control Mode
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Setting to Velocity Control Mode.", dxl_id);
        break;
      case 3:
      case 6:
        operating_mode = 16; // PWM Control Mode
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Setting to PWM Control Mode.", dxl_id);
        break;
      default:
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "ID %d: Invalid mode %d. Defaulting to Position Control.", dxl_id, mode);
        is_mode_valid = false;
        operating_mode = 3;
        break;
    }
  } else {
    // Position Control Mode
    operating_mode = 3;
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Forcing to Position Control Mode.", dxl_id);
  }

  // shutdown the torque mode for change control mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    0, // disable torque first
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS)
    RCLCPP_WARN(rclcpp::get_logger("read_write_node"), "ID %d: Failed to disable torque.", dxl_id);

  // set operate mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    operating_mode,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "ID %d: Failed to set Operating Mode.", dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Operating Mode set to %d.", dxl_id, operating_mode);
  }

  // enable torque
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1, // enable torque
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "ID %d: Failed to enable torque.", dxl_id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "ID %d: Torque enabled.", dxl_id);
  }
}

int main(int argc, char * argv[])
{
  Config config = load_config();
  portHandler = dynamixel::PortHandler::getPortHandler(config.port.c_str());
  RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "the port is %s", config.port.c_str());
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(1, config.control_mode);
  setupDynamixel(2, config.control_mode);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>(config.control_mode);
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
