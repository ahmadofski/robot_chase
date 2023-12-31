#include <string>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class RobotChaser : public rclcpp::Node {
public:
  RobotChaser(std::string chaser, std::string target, float kp_distance,
              float kp_yaw)
      : Node("robot_chaser") {
    this->kp_distance = kp_distance;
    this->kp_yaw = kp_yaw;
    this->chaser_frame = chaser + "/base_link";
    this->target_frame = target + "/base_link";

    steady_clock = rclcpp::Clock();

    tf_lstn_cbg = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    pub_cbg = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        chaser + "/cmd_vel", 100);
    tf_lstn_timer_ = this->create_wall_timer(
        20ms, std::bind(&RobotChaser::tf_lstn_callback, this),
        tf_lstn_cbg); //!< 50hz timer for tf transform
    pub_timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotChaser::pub_timer_callback, this),
        pub_cbg); //!< 10hz timer for publisher
  }

private:
  rclcpp::CallbackGroup::SharedPtr tf_lstn_cbg, pub_cbg;
  rclcpp::TimerBase::SharedPtr tf_lstn_timer_, pub_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  float error_distance, error_yaw, kp_distance, kp_yaw;
  std::string chaser_frame, target_frame;
  rclcpp::Clock steady_clock;
  bool broadcasterReady;

  void tf_lstn_callback() {

    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer_->lookupTransform(this->chaser_frame, this->target_frame,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  this->target_frame.c_str(), this->chaser_frame.c_str(),
                  ex.what());
      return;
    }
    this->error_yaw =
        std::atan2(t.transform.translation.y, t.transform.translation.x);
    this->error_distance = std::sqrt(std::pow(t.transform.translation.x, 2) +
                                     std::pow(t.transform.translation.y, 2));
  }

  void pub_timer_callback() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = this->error_distance > 0.75
                       ? this->error_distance * this->kp_distance
                       : 0.0;
    msg.angular.z = this->error_yaw * this->kp_yaw;
    this->publisher_->publish(msg);
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), this->steady_clock, 200,
                          "TF Distance %2f Yaw %2f", this->error_distance,
                          this->error_yaw);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<RobotChaser> node =
      std::make_shared<RobotChaser>("rick", "morty", 0.4, 0.4);
  rcutils_logging_set_logger_level(node->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_DEBUG);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
