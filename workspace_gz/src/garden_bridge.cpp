#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

namespace
{
constexpr const char * kClockGz = "/world/default/clock";
constexpr const char * kImuGz =
  "/world/default/model/roboboat/link/base_link/sensor/sensor_imu/imu";
constexpr const char * kGpsGz =
  "/world/default/model/roboboat/link/base_link/sensor/sensor_gps/navsat";
constexpr const char * kScanGz =
  "/world/default/model/roboboat/link/base_link/sensor/sensor_lidar/scan";
constexpr const char * kOdomGz = "/model/roboboat/odometry";
constexpr const char * kPoseGz = "/world/default/pose/info";
constexpr const char * kLeftThrustGz =
  "/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust";
constexpr const char * kRightThrustGz =
  "/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust";

builtin_interfaces::msg::Time toRosTime(const gz::msgs::Time & time)
{
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(time.sec());
  ros_time.nanosec = static_cast<uint32_t>(time.nsec());
  return ros_time;
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double yawFromQuaternion(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}
}  // namespace

class GardenBridge : public rclcpp::Node
{
public:
  GardenBridge()
  : Node("garden_bridge")
  {
    lidar_yaw_correction_ = declare_parameter<double>("lidar_yaw_correction", 0.0);
    lidar_output_min_angle_ = declare_parameter<double>("lidar_output_min_angle", -2.35619);
    lidar_output_max_angle_ = declare_parameter<double>("lidar_output_max_angle", 2.35619);
    lidar_output_max_range_ = declare_parameter<double>("lidar_output_max_range", 15.0);

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/roboboat/sensors/imu/imu", 10);
    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/roboboat/sensors/gps/navsat", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/roboboat/sensors/lidar/scan", rclcpp::SensorDataQoS());
    sim_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/odometry", 10);

    left_thrust_pub_ = gz_node_.Advertise<gz::msgs::Double>(kLeftThrustGz);
    right_thrust_pub_ = gz_node_.Advertise<gz::msgs::Double>(kRightThrustGz);

    left_thrust_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/roboboat/thrusters/left/thrust", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        gz::msgs::Double gz_msg;
        gz_msg.set_data(msg->data);
        left_thrust_pub_.Publish(gz_msg);
      });

    right_thrust_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/roboboat/thrusters/right/thrust", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        gz::msgs::Double gz_msg;
        gz_msg.set_data(msg->data);
        right_thrust_pub_.Publish(gz_msg);
      });

    subscribe(kClockGz, &GardenBridge::onClock);
    subscribe(kImuGz, &GardenBridge::onImu);
    subscribe(kGpsGz, &GardenBridge::onGps);
    subscribe(kScanGz, &GardenBridge::onScan);
    subscribe(kOdomGz, &GardenBridge::onSimOdom);
    subscribe(kPoseGz, &GardenBridge::onPoseInfo);
  }

private:
  template<typename CallbackT>
  void subscribe(const std::string & topic, CallbackT callback)
  {
    if (!gz_node_.Subscribe(topic, callback, this)) {
      RCLCPP_ERROR(get_logger(), "Failed to subscribe to Gazebo topic: %s", topic.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Subscribed to Gazebo topic: %s", topic.c_str());
    }
  }

  void onClock(const gz::msgs::Clock & msg)
  {
    rosgraph_msgs::msg::Clock out;
    out.clock = toRosTime(msg.sim());
    if (has_clock_ && stampSeconds(out.clock) < stampSeconds(last_clock_)) {
      has_last_pose_ = false;
      RCLCPP_WARN(get_logger(), "Simulation clock jumped backwards; reset bridge pose history.");
    }
    last_clock_ = out.clock;
    has_clock_ = true;
    clock_pub_->publish(out);
  }

  builtin_interfaces::msg::Time stampFromHeaderOrClock(const gz::msgs::Header & header) const
  {
    if (header.has_stamp()) {
      return toRosTime(header.stamp());
    }
    if (has_clock_) {
      return last_clock_;
    }
    builtin_interfaces::msg::Time zero;
    return zero;
  }

  void onImu(const gz::msgs::IMU & msg)
  {
    sensor_msgs::msg::Imu out;
    out.header.stamp = stampFromHeaderOrClock(msg.header());
    out.header.frame_id = "imu_link";

    out.orientation.x = msg.orientation().x();
    out.orientation.y = msg.orientation().y();
    out.orientation.z = msg.orientation().z();
    out.orientation.w = msg.orientation().w();

    out.angular_velocity.x = msg.angular_velocity().x();
    out.angular_velocity.y = msg.angular_velocity().y();
    out.angular_velocity.z = msg.angular_velocity().z();

    out.linear_acceleration.x = msg.linear_acceleration().x();
    out.linear_acceleration.y = msg.linear_acceleration().y();
    out.linear_acceleration.z = msg.linear_acceleration().z();

    imu_pub_->publish(out);
  }

  void onGps(const gz::msgs::NavSat & msg)
  {
    sensor_msgs::msg::NavSatFix out;
    out.header.stamp = stampFromHeaderOrClock(msg.header());
    out.header.frame_id = "gps_link";
    out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    out.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    out.latitude = msg.latitude_deg();
    out.longitude = msg.longitude_deg();
    out.altitude = msg.altitude();
    gps_pub_->publish(out);
  }

  void onScan(const gz::msgs::LaserScan & msg)
  {
    sensor_msgs::msg::LaserScan out;
    out.header.stamp = stampFromHeaderOrClock(msg.header());
    out.header.frame_id = "lidar_link";
    out.angle_min = static_cast<float>(msg.angle_min());
    out.angle_max = static_cast<float>(msg.angle_max());
    out.angle_increment = static_cast<float>(msg.angle_step());
    out.range_min = static_cast<float>(msg.range_min());
    out.range_max = static_cast<float>(msg.range_max());
    out.scan_time = 0.0F;
    out.time_increment = 0.0F;

    out.ranges.resize(static_cast<size_t>(msg.ranges_size()));
    std::transform(
      msg.ranges().begin(), msg.ranges().end(), out.ranges.begin(),
      [](double value) { return static_cast<float>(value); });
    rotateLaserArray(out.ranges, lidar_yaw_correction_, out.angle_increment);

    out.intensities.resize(static_cast<size_t>(msg.intensities_size()));
    std::transform(
      msg.intensities().begin(), msg.intensities().end(), out.intensities.begin(),
      [](double value) { return static_cast<float>(value); });
    rotateLaserArray(out.intensities, lidar_yaw_correction_, out.angle_increment);
    limitLaserRange(out);
    trimLaserScan(out);

    scan_pub_->publish(out);
  }

  void rotateLaserArray(std::vector<float> & values, double yaw_correction, double angle_increment) const
  {
    if (values.empty() || std::abs(angle_increment) < 1.0e-9 || std::abs(yaw_correction) < 1.0e-9) {
      return;
    }

    const int size = static_cast<int>(values.size());
    int shift = static_cast<int>(std::lround(yaw_correction / angle_increment));
    shift %= size;
    if (shift < 0) {
      shift += size;
    }
    if (shift == 0) {
      return;
    }

    std::vector<float> rotated(values.size());
    for (int i = 0; i < size; ++i) {
      rotated[static_cast<size_t>((i + shift) % size)] = values[static_cast<size_t>(i)];
    }
    values = std::move(rotated);
  }

  void limitLaserRange(sensor_msgs::msg::LaserScan & scan) const
  {
    if (lidar_output_max_range_ <= 0.0) {
      return;
    }

    const float max_range = static_cast<float>(
      std::min(static_cast<double>(scan.range_max), lidar_output_max_range_));
    for (auto & range : scan.ranges) {
      if (std::isfinite(range) && range > max_range) {
        range = std::numeric_limits<float>::infinity();
      }
    }
    scan.range_max = max_range;
  }

  void trimLaserScan(sensor_msgs::msg::LaserScan & scan) const
  {
    if (scan.ranges.empty() || std::abs(scan.angle_increment) < 1.0e-9) {
      return;
    }

    const double min_angle = std::max(static_cast<double>(scan.angle_min), lidar_output_min_angle_);
    const double max_angle = std::min(static_cast<double>(scan.angle_max), lidar_output_max_angle_);
    if (min_angle >= max_angle) {
      return;
    }

    const int size = static_cast<int>(scan.ranges.size());
    int first = static_cast<int>(std::ceil((min_angle - scan.angle_min) / scan.angle_increment));
    int last = static_cast<int>(std::floor((max_angle - scan.angle_min) / scan.angle_increment));
    first = std::max(0, std::min(size - 1, first));
    last = std::max(0, std::min(size - 1, last));
    if (first == 0 && last == size - 1) {
      return;
    }
    if (first > last) {
      return;
    }

    scan.ranges = std::vector<float>(
      scan.ranges.begin() + first,
      scan.ranges.begin() + last + 1);
    if (scan.intensities.size() == static_cast<size_t>(size)) {
      scan.intensities = std::vector<float>(
        scan.intensities.begin() + first,
        scan.intensities.begin() + last + 1);
    }
    scan.angle_min = scan.angle_min + static_cast<float>(first) * scan.angle_increment;
    scan.angle_max = scan.angle_min + static_cast<float>(scan.ranges.size() - 1) * scan.angle_increment;
  }

  void setLowCovariance(nav_msgs::msg::Odometry & out) const
  {
    for (size_t idx = 0; idx < out.pose.covariance.size(); ++idx) {
      out.pose.covariance[idx] = 0.0;
      out.twist.covariance[idx] = 0.0;
    }
    out.pose.covariance[0] = 1.0e-4;
    out.pose.covariance[7] = 1.0e-4;
    out.pose.covariance[35] = 1.0e-4;
    out.twist.covariance[0] = 1.0e-3;
    out.twist.covariance[7] = 1.0e-3;
    out.twist.covariance[35] = 1.0e-3;
  }

  void onSimOdom(const gz::msgs::Odometry & msg)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = stampFromHeaderOrClock(msg.header());
    out.header.frame_id = "odom";
    out.child_frame_id = "base_link";
    out.pose.pose.position.x = msg.pose().position().x();
    out.pose.pose.position.y = msg.pose().position().y();
    out.pose.pose.position.z = msg.pose().position().z();
    out.pose.pose.orientation.x = msg.pose().orientation().x();
    out.pose.pose.orientation.y = msg.pose().orientation().y();
    out.pose.pose.orientation.z = msg.pose().orientation().z();
    out.pose.pose.orientation.w = msg.pose().orientation().w();
    out.twist.twist.linear.x = msg.twist().linear().x();
    out.twist.twist.linear.y = msg.twist().linear().y();
    out.twist.twist.linear.z = msg.twist().linear().z();
    out.twist.twist.angular.x = msg.twist().angular().x();
    out.twist.twist.angular.y = msg.twist().angular().y();
    out.twist.twist.angular.z = msg.twist().angular().z();
    setLowCovariance(out);
    sim_odom_pub_->publish(out);
  }

  void onPoseInfo(const gz::msgs::Pose_V & msg)
  {
    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto & pose = msg.pose(i);
      if (pose.name() != "roboboat") {
        continue;
      }

      nav_msgs::msg::Odometry out;
      out.header.stamp = stampFromHeaderOrClock(msg.header());
      out.header.frame_id = "odom";
      out.child_frame_id = "base_link";
      out.pose.pose.position.x = pose.position().x();
      out.pose.pose.position.y = pose.position().y();
      out.pose.pose.position.z = pose.position().z();
      out.pose.pose.orientation.x = pose.orientation().x();
      out.pose.pose.orientation.y = pose.orientation().y();
      out.pose.pose.orientation.z = pose.orientation().z();
      out.pose.pose.orientation.w = pose.orientation().w();

      const double now = stampSeconds(out.header.stamp);
      const double yaw = yawFromQuaternion(
        out.pose.pose.orientation.x,
        out.pose.pose.orientation.y,
        out.pose.pose.orientation.z,
        out.pose.pose.orientation.w);
      if (has_last_pose_ && now > last_pose_time_) {
        const double dt = now - last_pose_time_;
        out.twist.twist.linear.x = (out.pose.pose.position.x - last_x_) / dt;
        out.twist.twist.linear.y = (out.pose.pose.position.y - last_y_) / dt;
        out.twist.twist.linear.z = (out.pose.pose.position.z - last_z_) / dt;
        out.twist.twist.angular.z = normalizeAngle(yaw - last_yaw_) / dt;
      }

      setLowCovariance(out);

      sim_odom_pub_->publish(out);
      last_x_ = out.pose.pose.position.x;
      last_y_ = out.pose.pose.position.y;
      last_z_ = out.pose.pose.position.z;
      last_yaw_ = yaw;
      last_pose_time_ = now;
      has_last_pose_ = true;
      return;
    }
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher left_thrust_pub_;
  gz::transport::Node::Publisher right_thrust_pub_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sim_odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_thrust_sub_;
  bool has_clock_{false};
  builtin_interfaces::msg::Time last_clock_;
  double lidar_yaw_correction_{0.0};
  double lidar_output_min_angle_{-2.35619};
  double lidar_output_max_angle_{2.35619};
  double lidar_output_max_range_{15.0};
  bool has_last_pose_{false};
  double last_pose_time_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_z_{0.0};
  double last_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GardenBridge>());
  rclcpp::shutdown();
  return 0;
}
