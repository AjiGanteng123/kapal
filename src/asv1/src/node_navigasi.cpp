#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>

class NodeNavigasi : public rclcpp::Node
{
public:
  NodeNavigasi() : Node("node_navigasi")
  {
    declare_parameter("waypoint_lat", -7.0526);
    declare_parameter("waypoint_lon", 110.4348);
    declare_parameter("cruise_speed", 0.5);
    declare_parameter("approach_speed", 0.35);
    declare_parameter("visual_p_gain", 0.002);
    declare_parameter("visual_i_gain", 0.0001);
    declare_parameter("visual_d_gain", 0.0005);
    declare_parameter("lpf_alpha", 0.3);
    declare_parameter("gps_p_gain", 0.05);
    declare_parameter("max_yaw_rate", 0.5);
    declare_parameter("obstacle_stop_dist", 0.4);
    declare_parameter("obstacle_avoid_speed", 0.3);
    declare_parameter("fusion_dist_min", 0.3);
    declare_parameter("fusion_dist_max", 3.0);
    declare_parameter("fusion_speed_min", 0.15);
    declare_parameter("fusion_speed_max", 0.6);
    declare_parameter("scan_speed", 0.1);
    declare_parameter("scan_yaw_rate", 0.25);
    declare_parameter("scan_cycles", 40);
    declare_parameter("log_path", "/tmp/navigasi_log.csv");

    wp_lat_ = get_parameter("waypoint_lat").as_double();
    wp_lon_ = get_parameter("waypoint_lon").as_double();
    cruise_speed_ = get_parameter("cruise_speed").as_double();
    approach_speed_ = get_parameter("approach_speed").as_double();
    visual_p_gain_ = get_parameter("visual_p_gain").as_double();
    visual_i_gain_ = get_parameter("visual_i_gain").as_double();
    visual_d_gain_ = get_parameter("visual_d_gain").as_double();
    lpf_alpha_ = get_parameter("lpf_alpha").as_double();
    gps_p_gain_ = get_parameter("gps_p_gain").as_double();
    max_yaw_rate_ = get_parameter("max_yaw_rate").as_double();
    obstacle_stop_dist_ = get_parameter("obstacle_stop_dist").as_double();
    obstacle_avoid_speed_ = get_parameter("obstacle_avoid_speed").as_double();
    fusion_dist_min_ = get_parameter("fusion_dist_min").as_double();
    fusion_dist_max_ = get_parameter("fusion_dist_max").as_double();
    fusion_speed_min_ = get_parameter("fusion_speed_min").as_double();
    fusion_speed_max_ = get_parameter("fusion_speed_max").as_double();
    scan_speed_ = get_parameter("scan_speed").as_double();
    scan_yaw_rate_ = get_parameter("scan_yaw_rate").as_double();
    scan_cycles_max_ = get_parameter("scan_cycles").as_int();
    log_path_ = get_parameter("log_path").as_string();

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/asv/cmd_vel", 10);

    sub_tracking_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/asv/tracking", 10,
      std::bind(&NodeNavigasi::cb_tracking, this, std::placeholders::_1));

    sub_telemetri_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/asv/telemetri", 10,
      std::bind(&NodeNavigasi::cb_telemetri, this, std::placeholders::_1));

    sub_obstacle_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/asv/obstacle", 10,
      std::bind(&NodeNavigasi::cb_obstacle, this, std::placeholders::_1));

    sub_fc_mode_ = create_subscription<std_msgs::msg::Int32>(
      "/asv/fc_mode", 10,
      std::bind(&NodeNavigasi::cb_fc_mode, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&NodeNavigasi::control_loop, this));

    csv_file_.open(log_path_, std::ios::out);
    if (csv_file_.is_open())
      csv_file_ << "time_s,mode,offset_px,linear_x,angular_z,error_p,error_i,error_d\n";
    else
      RCLCPP_WARN(get_logger(), "CSV log failed: %s", log_path_.c_str());

    RCLCPP_INFO(get_logger(), "=== NAVIGASI NODE START ===");
    RCLCPP_INFO(get_logger(), "Priority: 1=Obstacle, 2=Fusion, 3=Visual, 4=Scan, 5=GPS");
    RCLCPP_INFO(get_logger(), "PID: P=%.4f I=%.4f D=%.4f LPF=%.2f",
                visual_p_gain_, visual_i_gain_, visual_d_gain_, lpf_alpha_);
    RCLCPP_INFO(get_logger(), "Scan: speed=%.2f yaw=%.2f cycle=%d",
                scan_speed_, scan_yaw_rate_, scan_cycles_max_);
  }

  ~NodeNavigasi()
  {
    if (csv_file_.is_open())
    {
      csv_file_ << "# END\n";
      csv_file_.close();
      RCLCPP_INFO(get_logger(), "CSV log saved: %s", log_path_.c_str());
    }
  }

private:
  static constexpr int ARRIVAL_THRESHOLD_PX = 20;
  static constexpr int ARRIVAL_CYCLES_REQ = 100;
  static constexpr int STOP_CYCLES = 40;
  static constexpr int RETURN_CYCLES = 200;
  static constexpr int SCAN_TIMEOUT_CYCLES = 600;

  double wp_lat_, wp_lon_;
  double cruise_speed_, approach_speed_;
  double visual_p_gain_, visual_i_gain_, visual_d_gain_;
  double lpf_alpha_;
  double gps_p_gain_;
  double max_yaw_rate_;
  double obstacle_stop_dist_, obstacle_avoid_speed_;
  double fusion_dist_min_, fusion_dist_max_;
  double fusion_speed_min_, fusion_speed_max_;
  double scan_speed_, scan_yaw_rate_;
  int scan_cycles_max_;
  std::string log_path_;

  int tracking_status_ = 0;
  int tracking_offset_ = 0;
  bool has_midpoint_ = false;
  double tracking_max_area_ = 0.0;
  double curr_lat_ = 0.0, curr_lon_ = 0.0;
  double curr_heading_ = 0.0;
  double obstacle_front_ = 99.0;
  double obstacle_right_ = 99.0;
  double obstacle_back_ = 99.0;
  double obstacle_left_ = 99.0;

  int last_mode_ = -1;

  int visual_hold_count_ = 0;
  const int VISUAL_HOLD_MAX = 50;
  double last_visual_linear_ = 0.0;
  double last_visual_angular_ = 0.0;

  double integral_error_ = 0.0;
  double prev_error_ = 0.0;
  double prev_angular_z_ = 0.0;

  int mission_state_ = 0;
  int mission_timer_ = 0;
  int arrival_cycles_ = 0;

  int scan_cycles_ = 0;
  int scan_dir_ = 1;
  int scan_no_detect_cycles_ = 0;
  
  int fc_mode_ = 0;  // NEW: FC mode (0=MANUAL, 4=GUIDED, 10=RTL, etc)
  bool autonomous_enabled_ = true;  // NEW: Allow disabling autonomous

  std::ofstream csv_file_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_tracking_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_telemetri_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_obstacle_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_fc_mode_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cb_tracking(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 12) return;
    tracking_status_ = static_cast<int>(msg->data[0]);
    tracking_offset_ = static_cast<int>(msg->data[1]);
    has_midpoint_ = (msg->data[8] > 0.5);
    tracking_max_area_ = msg->data[11];
  }

  void cb_telemetri(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) return;
    curr_lat_ = msg->data[0];
    curr_lon_ = msg->data[1];
    curr_heading_ = msg->data[2];
  }

  void cb_obstacle(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;
    obstacle_front_ = msg->data[0];
    obstacle_right_ = msg->data[1];
    obstacle_back_ = msg->data[2];
    obstacle_left_ = msg->data[3];
  }

  void cb_fc_mode(const std_msgs::msg::Int32::SharedPtr msg)
  {
    fc_mode_ = msg->data;
    // FC mode: 0=MANUAL, 4=GUIDED, 10=RTL, 11=LOITER, etc
    // Only GUIDED (4) allows autonomous control
    bool was_enabled = autonomous_enabled_;
    autonomous_enabled_ = (fc_mode_ == 4);  // 4 = GUIDED mode
    
    if (was_enabled && !autonomous_enabled_)
      RCLCPP_WARN(get_logger(), "FC mode changed to %d — AUTONOMOUS DISABLED!", fc_mode_);
    else if (!was_enabled && autonomous_enabled_)
      RCLCPP_INFO(get_logger(), "FC mode changed to GUIDED — AUTONOMOUS ENABLED");
  }

  double deg2rad(double deg) { return deg * M_PI / 180.0; }
  double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  double bearing_to_target(double lat1, double lon1, double lat2, double lon2)
  {
    double dlon = deg2rad(lon2 - lon1);
    double rlat1 = deg2rad(lat1);
    double rlat2 = deg2rad(lat2);
    double y = std::sin(dlon) * std::cos(rlat2);
    double x = std::cos(rlat1) * std::sin(rlat2) -
               std::sin(rlat1) * std::cos(rlat2) * std::cos(dlon);
    return std::fmod(rad2deg(std::atan2(y, x)) + 360.0, 360.0);
  }

  void control_loop()
  {
    auto cmd = geometry_msgs::msg::Twist();
    int active_mode = 0;
    double p_term = 0, i_term = 0, d_term = 0;

    if (mission_state_ == 1)
    {
      mission_timer_++;
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      active_mode = 4;
      if (mission_timer_ >= STOP_CYCLES)
      {
        mission_state_ = 2;
        mission_timer_ = 0;
        RCLCPP_INFO(get_logger(), "MISSION: stopping done, now reversing");
      }
    }
    else if (mission_state_ == 2)
    {
      mission_timer_++;
      cmd.linear.x = -approach_speed_;
      cmd.angular.z = 0.0;
      active_mode = 4;
      if (mission_timer_ >= RETURN_CYCLES)
      {
        mission_state_ = 0;
        mission_timer_ = 0;
        RCLCPP_INFO(get_logger(), "MISSION: return complete, back to normal");
      }
    }
    else
    {
      if (has_midpoint_ && tracking_status_ == 1)
        scan_no_detect_cycles_ = 0;
      else
        scan_no_detect_cycles_++;

      if (obstacle_front_ < obstacle_stop_dist_)
      {
        if (tracking_status_ == 1 && has_midpoint_ && std::abs(tracking_offset_) < 100)
        {
          double dist = std::clamp(obstacle_front_, fusion_dist_min_, fusion_dist_max_);
          double speed_ratio = (dist - fusion_dist_min_) / (fusion_dist_max_ - fusion_dist_min_);
          cmd.linear.x = fusion_speed_min_ + speed_ratio * (fusion_speed_max_ - fusion_speed_min_);
          cmd.linear.x = std::clamp(cmd.linear.x, fusion_speed_min_, fusion_speed_max_);

          double error = tracking_offset_;
          integral_error_ += error * 0.05;
          integral_error_ = std::clamp(integral_error_, -500.0, 500.0);
          double derivative = (error - prev_error_) / 0.05;
          double yaw_rate = error * visual_p_gain_
                          + integral_error_ * visual_i_gain_
                          + derivative * visual_d_gain_;
          yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
          prev_error_ = error;
          cmd.angular.z = lpf_alpha_ * yaw_rate + (1.0 - lpf_alpha_) * prev_angular_z_;
          prev_angular_z_ = cmd.angular.z;

          active_mode = 5;

          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "FUSION: ball at %.2fm front, speed=%.2f", dist, cmd.linear.x);
        }
        else
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                               "OBSTACLE: %.2fm di depan!", obstacle_front_);
          if (obstacle_left_ > obstacle_right_)
            cmd.angular.z = max_yaw_rate_;
          else
            cmd.angular.z = -max_yaw_rate_;
          cmd.linear.x = obstacle_avoid_speed_;
          active_mode = 1;
        }
      }
      else if (tracking_status_ == 1 && has_midpoint_)
      {
        double error = tracking_offset_;
        integral_error_ += error * 0.05;
        integral_error_ = std::clamp(integral_error_, -500.0, 500.0);
        double derivative = (error - prev_error_) / 0.05;
        double yaw_rate = error * visual_p_gain_
                        + integral_error_ * visual_i_gain_
                        + derivative * visual_d_gain_;
        yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
        prev_error_ = error;

        cmd.angular.z = lpf_alpha_ * yaw_rate + (1.0 - lpf_alpha_) * prev_angular_z_;
        prev_angular_z_ = cmd.angular.z;

        double fusion_speed = cruise_speed_;
        if (obstacle_front_ < fusion_dist_max_)
        {
          double dist = std::clamp(obstacle_front_, fusion_dist_min_, fusion_dist_max_);
          double speed_ratio = (dist - fusion_dist_min_) / (fusion_dist_max_ - fusion_dist_min_);
          fusion_speed = fusion_speed_min_ + speed_ratio * (fusion_speed_max_ - fusion_speed_min_);
        }
        else
        {
          const double AREA_FAR = 500.0;
          const double AREA_CLOSE = 30000.0;
          double area_ratio = std::clamp(
            (tracking_max_area_ - AREA_FAR) / (AREA_CLOSE - AREA_FAR), 0.0, 1.0);
          fusion_speed = cruise_speed_ * (1.0 - area_ratio) + approach_speed_ * area_ratio;
        }
        cmd.linear.x = std::clamp(fusion_speed, 0.15, cruise_speed_);

        active_mode = 2;
        visual_hold_count_ = 0;
        last_visual_linear_ = cmd.linear.x;
        last_visual_angular_ = cmd.angular.z;

        if (std::abs(error) < ARRIVAL_THRESHOLD_PX)
          arrival_cycles_++;
        else
          arrival_cycles_ = 0;

        if (arrival_cycles_ >= ARRIVAL_CYCLES_REQ)
        {
          RCLCPP_INFO(get_logger(), "MISSION: ARRIVED at offset=%dpx! Stop then return.", tracking_offset_);
          mission_state_ = 1;
          mission_timer_ = 0;
          arrival_cycles_ = 0;
        }

        p_term = error * visual_p_gain_;
        i_term = integral_error_ * visual_i_gain_;
        d_term = derivative * visual_d_gain_;
      }
      else if ((last_mode_ == 2 || last_mode_ == 5) && visual_hold_count_ < VISUAL_HOLD_MAX)
      {
        visual_hold_count_++;
        cmd.linear.x = last_visual_linear_;
        cmd.angular.z = last_visual_angular_;
        active_mode = 2;
      }
      else if (scan_no_detect_cycles_ < SCAN_TIMEOUT_CYCLES)
      {
        scan_cycles_++;
        if (scan_cycles_ >= scan_cycles_max_)
        {
          scan_dir_ *= -1;
          scan_cycles_ = 0;
        }
        cmd.angular.z = scan_dir_ * scan_yaw_rate_;
        cmd.linear.x = scan_speed_;
        active_mode = 6;
      }
      else
      {
        if (obstacle_front_ < fusion_dist_max_ && obstacle_front_ < cruise_speed_ * 2.0)
        {
          cmd.linear.x = obstacle_avoid_speed_;
          cmd.angular.z = 0.0;
          active_mode = 3;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "CRUISE: LiDAR sees %.2fm ahead, slowing", obstacle_front_);
        }
        else
        {
          cmd.linear.x = approach_speed_;
          cmd.angular.z = 0.0;
          active_mode = 3;
        }
      }

      if (active_mode != 2 && active_mode != 5)
      {
        integral_error_ = 0.0;
        prev_error_ = 0.0;
        prev_angular_z_ = 0.0;
        arrival_cycles_ = 0;
      }
    }

    if (csv_file_.is_open())
    {
      csv_file_ << get_clock()->now().seconds() << ","
                << active_mode << ","
                << tracking_offset_ << ","
                << cmd.linear.x << ","
                << cmd.angular.z << ","
                << p_term << ","
                << i_term << ","
                << d_term << "\n";
    }

    if (active_mode != last_mode_)
    {
      const char* mode_names[] = {"IDLE", "OBSTACLE", "VISUAL", "CRUISE", "MISSION", "FUSION", "SCAN"};
      RCLCPP_INFO(get_logger(), "MODE: %s -> linear=%.2f, angular=%.2f",
                  mode_names[active_mode], cmd.linear.x, cmd.angular.z);
      last_mode_ = active_mode;
    }

    // NEW: Safety check — stop autonomous if FC not in GUIDED mode
    if (!autonomous_enabled_)
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    pub_cmd_->publish(cmd);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeNavigasi>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
