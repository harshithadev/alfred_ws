#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>

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
            // Format data to send over serial
            std::string serial_data = "Vx:" + std::to_string(msg->linear.x) +
                                      " Vy:" + std::to_string(msg->linear.y) +
                                      " Wz:" + std::to_string(msg->angular.z) + "\n";

            try {
                serial_port.Write(serial_data);
                RCLCPP_INFO(this->get_logger(), "Sent: %s", serial_data.c_str());
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
