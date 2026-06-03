#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <gz/msgs/clock.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/transport/Node.hh>

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

builtin_interfaces::msg::Time stampOrZero(const gz::msgs::Header & header)
{
  if (header.has_stamp()) {
    return toRosTime(header.stamp());
  }
  builtin_interfaces::msg::Time zero;
  return zero;
}
}  // namespace

class GardenBridge : public rclcpp::Node
{
public:
  GardenBridge()
  : Node("garden_bridge")
  {
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/roboboat/sensors/imu/imu", 10);
    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/roboboat/sensors/gps/navsat", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/roboboat/sensors/lidar/scan", rclcpp::SensorDataQoS());

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
    clock_pub_->publish(out);
  }

  void onImu(const gz::msgs::IMU & msg)
  {
    sensor_msgs::msg::Imu out;
    out.header.stamp = stampOrZero(msg.header());
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
    out.header.stamp = stampOrZero(msg.header());
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
    out.header.stamp = stampOrZero(msg.header());
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

    out.intensities.resize(static_cast<size_t>(msg.intensities_size()));
    std::transform(
      msg.intensities().begin(), msg.intensities().end(), out.intensities.begin(),
      [](double value) { return static_cast<float>(value); });

    scan_pub_->publish(out);
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher left_thrust_pub_;
  gz::transport::Node::Publisher right_thrust_pub_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_thrust_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GardenBridge>());
  rclcpp::shutdown();
  return 0;
}
