#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/asio.hpp>
#include <sstream>
#include <string>

class serial_joint_state_publisher : public rclcpp::Node
{
public:
    serial_joint_state_publisher()
    : Node("serial_joint_state_publisher"),
      io(),
      serial(io, "/dev/pts/7")
    {
        // Set the serial port settings
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));

        // Subscribe to JointState topic
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->joint_state_callback(msg);
            });
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::ostringstream oss;
        for (const auto & position : msg->position) {
            oss << position << ","; // Comma-separated joint positions
        }
        std::string data = oss.str();
        data.pop_back(); // Remove the trailing comma

        RCLCPP_INFO(this->get_logger(), "Sending: '%s'", data.c_str());
        
        // Write data to serial port
        boost::asio::write(serial, boost::asio::buffer(data + "\n")); // Add newline for separation
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    boost::asio::io_service io;
    boost::asio::serial_port serial;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_joint_state_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
