#include <cmath>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class PX4OdomRepublisher : public rclcpp::Node {
public:
  PX4OdomRepublisher() : Node("px4_odom_republisher") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", qos,
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) { callback(msg); });

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/px4/odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/px4/path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = "odom";
  }

private:
  void callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(msg->timestamp / 1000000ULL);
    stamp.nanosec = static_cast<uint32_t>((msg->timestamp % 1000000ULL) * 1000);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // NED -> ENU position
    odom.pose.pose.position.x = msg->position[1];
    odom.pose.pose.position.y = msg->position[0];
    odom.pose.pose.position.z = -msg->position[2];

    // NED/FRD -> ENU/FLU quaternion: q_ned2enu * q_px4 * q_flu2frd
    const double w = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];
    constexpr double s = 0.7071067811865476;  // 1/sqrt(2)
    odom.pose.pose.orientation.w = s * (w + z);
    odom.pose.pose.orientation.x = s * (x + y);
    odom.pose.pose.orientation.y = s * (x - y);
    odom.pose.pose.orientation.z = s * (w - z);

    // NED -> ENU velocity
    odom.twist.twist.linear.x = msg->velocity[1];
    odom.twist.twist.linear.y = msg->velocity[0];
    odom.twist.twist.linear.z = -msg->velocity[2];

    // FRD -> FLU angular velocity
    odom.twist.twist.angular.x = msg->angular_velocity[0];
    odom.twist.twist.angular.y = -msg->angular_velocity[1];
    odom.twist.twist.angular.z = -msg->angular_velocity[2];

    pub_odom_->publish(odom);

    // Path
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    poses_.push_back(ps);
    if (poses_.size() > 50000) poses_.pop_front(); // keep the last 50000 poses

    path_.header.stamp = stamp;
    path_.poses.assign(poses_.begin(), poses_.end());
    pub_path_->publish(path_);

    // TF
    geometry_msgs::msg::TransformStamped t;
    t.header = odom.header;
    t.child_frame_id = "base_link";
    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = odom.pose.pose.position.z;
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_;
  std::deque<geometry_msgs::msg::PoseStamped> poses_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4OdomRepublisher>());
  rclcpp::shutdown();
  return 0;
}
