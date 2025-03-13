#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>
#include <iomanip>
#include <sstream>

using namespace LibSerial;

class SerialPublisher : public rclcpp::Node {
public:
    SerialPublisher() : Node("serial_publisher") {
        // Subscribe to /cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SerialPublisher::cmdVelCallback, this, std::placeholders::_1));

        // Open Serial Port
        try {
            serial_port.Open("/dev/ttyACM0");
            serial_port.SetBaudRate(BaudRate::BAUD_115200);
            serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port.SetStopBits(StopBits::STOP_BITS_1);
            serial_port.SetParity(Parity::PARITY_NONE);
            RCLCPP_INFO(this->get_logger(), "Serial Port /dev/ttyACM0 opened successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        }
    }

    ~SerialPublisher() {
        if (serial_port.IsOpen()) {
            serial_port.Close();
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (serial_port.IsOpen()) {
            // Assuming a simple differential drive model
            double wheel_base = 0.600; // Distance between wheels
            double wheel_radius = 0.075; // Radius of each wheel

            // Compute left and right wheel velocities
            double v_right = (msg->linear.x + (msg->angular.z * wheel_base / 2)) / wheel_radius;
            double v_left = (msg->linear.x - (msg->angular.z * wheel_base / 2)) / wheel_radius;

            // Format the string properly
            std::ostringstream serial_data;
            serial_data << "JOINT_VELOCITIES r" << std::fixed << std::setprecision(1) << v_right
                        << ",l" << std::fixed << std::setprecision(1) << v_left << "\n";

            try {
                serial_port.Write(serial_data.str());
                RCLCPP_INFO(this->get_logger(), "Sent: %s", serial_data.str().c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    SerialPort serial_port;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialPublisher>());
    rclcpp::shutdown();
    return 0;
}
