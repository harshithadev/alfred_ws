#include "alfred_firmware/alfred_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace alfred_firmware
{

// empty constructor 
AlfredInterface::AlfredInterface()
{
}


// destructor function : close the serial connection. 
AlfredInterface::~AlfredInterface()
{
  if (teensy_.IsOpen())
  {
    try
    {
      teensy_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlfredInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


// checking for serial connection, else throwing error. 
// also when connection established, initialising vectors. 
CallbackReturn AlfredInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("AlfredInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


// exporting state interfaces 
std::vector<hardware_interface::StateInterface> AlfredInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

//exporting command interfaces
std::vector<hardware_interface::CommandInterface> AlfredInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn AlfredInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AlfredInterface"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };

  try
  {
    teensy_.Open(port_);
    teensy_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlfredInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfredInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn AlfredInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AlfredInterface"), "Stopping robot hardware ...");

  if (teensy_.IsOpen())
  {
    try
    {
      teensy_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlfredInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AlfredInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


// reading the current velocity of the motors and setting it to the variables. 
hardware_interface::return_type AlfredInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Interpret the string
  if(teensy_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    teensy_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    while(std::getline(ss, res, ','))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlfredInterface"),
      "FALSE ALARM : rec message "
          << res << " to the port " << port_);

      if(res.at(0) == 'r')
      {
        velocity_states_.at(0) = std::stod(res.substr(1, res.size()));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      }
      else if(res.at(0) == 'l')
      {
        velocity_states_.at(1) = std::stod(res.substr(1, res.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("AlfredInterface"), "reading position and velocity");
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AlfredInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Implement communication protocol with the Arduino
  std::stringstream message_stream;
  std::string compensate_zeros_right = "";
  std::string compensate_zeros_left = "";
  
  message_stream << std::fixed << std::setprecision(2) << "JOINT_VELOCITIES " <<
    "r"  << velocity_commands_.at(0) << 
    ",l" << velocity_commands_.at(1) << "\n";

  
  try
  {
    teensy_.Write(message_stream.str());
    // RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlfredInterface"),
    //                     "FALSE ALARM : sending message "
    //                         << message_stream.str() << " to the port " << port_);
  
   }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlfredInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace alfred_firmware

PLUGINLIB_EXPORT_CLASS(alfred_firmware::AlfredInterface, hardware_interface::SystemInterface)