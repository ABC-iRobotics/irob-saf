#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * Minimal serial test placeholder without external dependencies.
 * It just logs the configured port/baudrate.
 * If you want real serial I/O, we can add <serial/serial.h> later.
 */
class SerialTest : public rclcpp::Node {
public:
    SerialTest()
    : rclcpp::Node("serial_test")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);

        RCLCPP_INFO(get_logger(), "serial_test: port=%s baud=%d", port_.c_str(), baudrate_);
        RCLCPP_INFO(get_logger(), "(placeholder) no real serial I/O is performed.");
    }
private:
    std::string port_;
    int baudrate_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialTest>());
    rclcpp::shutdown();
    return 0;
}
