#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <vector>
#include <cstdint>

extern "C" {
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
}

class MotorNode : public rclcpp::Node
{
public:
  MotorNode() : Node("motor_node"), fd_(-1)
  {
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("protocol", "mavlink");
    this->declare_parameter("max_linear", 0.5);
    this->declare_parameter("max_angular", 0.8);

    port_ = this->get_parameter("serial_port").as_string();
    baud_ = this->get_parameter("baudrate").as_int();
    protocol_ = this->get_parameter("protocol").as_string();
    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();

    fd_ = open_serial(port_, baud_);
    if (fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "Serial not available (%s), running in dry mode", port_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Connected to %s @ %d baud", port_.c_str(), baud_);
    }

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/robot/cmd_vel", 10, std::bind(&MotorNode::cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "motor_node started (protocol=%s)", protocol_.c_str());
  }

  ~MotorNode()
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  int open_serial(const std::string & port, int baud)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    cfsetospeed(&tty, speed_t(baud_to_speed(baud)));
    cfsetispeed(&tty, speed_t(baud_to_speed(baud)));

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
  }

  int baud_to_speed(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return B115200;
    }
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (fd_ < 0) return;

    double linear = std::max(-max_linear_, std::min(max_linear_, msg->linear.x));
    double angular = std::max(-max_angular_, std::min(max_angular_, msg->angular.z));

    if (protocol_ == "msp") {
      send_msp(linear, angular);
    } else if (protocol_ == "raw") {
      send_raw(linear, angular);
    } else {
      RCLCPP_WARN_ONCE(this->get_logger(), "MAVLink not implemented in C++; falling back to raw");
      send_raw(linear, angular);
    }
  }

  void send_raw(double linear, double angular)
  {
    double left = linear - angular;
    double right = linear + angular;
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%.2f,%.2f\n", left, right);
    if (write(fd_, buf, n) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial write error");
    }
  }

  void send_msp(double linear, double angular)
  {
    int left = std::clamp(static_cast<int>(1500 + (linear - angular) * 500), 1000, 2000);
    int right = std::clamp(static_cast<int>(1500 + (linear + angular) * 500), 1000, 2000);

    std::vector<uint8_t> buf;
    buf.push_back(36);  // $
    buf.push_back(77);  // M
    buf.push_back(60);  // <
    buf.push_back(0);   // flag
    buf.push_back(8);   // size (8 bytes for 4 channels)
    buf.push_back(left & 0xFF);
    buf.push_back((left >> 8) & 0xFF);
    buf.push_back(right & 0xFF);
    buf.push_back((right >> 8) & 0xFF);
    buf.push_back(1500 & 0xFF);
    buf.push_back((1500 >> 8) & 0xFF);
    buf.push_back(1500 & 0xFF);
    buf.push_back((1500 >> 8) & 0xFF);

    uint8_t checksum = 0;
    for (size_t i = 3; i < buf.size(); ++i) checksum ^= buf[i];
    buf.push_back(checksum);

    if (write(fd_, buf.data(), buf.size()) < 0) {
      RCLCPP_ERROR(this->get_logger(), "MSP write error");
    }
  }

  int fd_;
  std::string port_;
  int baud_;
  std::string protocol_;
  double max_linear_;
  double max_angular_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
