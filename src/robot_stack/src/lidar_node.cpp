#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class LidarNode : public rclcpp::Node
{
public:
  LidarNode() : Node("lidar_node")
  {
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("num_sectors", 4);
    this->declare_parameter("obstacle_distance", 0.5);

    scan_topic_ = this->get_parameter("scan_topic").as_string();
    num_sectors_ = this->get_parameter("num_sectors").as_int();
    obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/obstacle", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&LidarNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "lidar_node started");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    size_t n = msg->ranges.size();
    if (n == 0) return;

    size_t sector_size = n / num_sectors_;
    std::vector<float> min_distances(num_sectors_, msg->range_max);

    for (int i = 0; i < num_sectors_; ++i) {
      size_t start = i * sector_size;
      size_t end = (i == num_sectors_ - 1) ? n : start + sector_size;
      for (size_t j = start; j < end; ++j) {
        float r = msg->ranges[j];
        if (r > msg->range_min && r < msg->range_max && r < min_distances[i]) {
          min_distances[i] = r;
        }
      }
    }

    float front = min_distances[0];
    float right = min_distances[1];
    float back = min_distances[2];
    float left = min_distances[3];

    auto obstacle_msg = std_msgs::msg::Float32MultiArray();
    obstacle_msg.data = {front, right, back, left};
    pub_->publish(obstacle_msg);

    if (front < obstacle_distance_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle front: %.2f m", front);
    }
  }

  std::string scan_topic_;
  int num_sectors_;
  double obstacle_distance_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
