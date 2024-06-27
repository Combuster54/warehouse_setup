#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/time.hpp"
#include "rmw/types.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/int32.hpp"

#include "std_srvs/srv/empty.hpp"

#include <geometry_msgs/msg/transform_stamped.h>
#include <unistd.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

using EmptySrv = std_srvs::srv::Empty;

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node {
public:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped static_transform_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  bool detect = false;

  float dist = 0;
  float x_frame = 0.0;
  float y_frame = 0.0;
  bool on_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  int count = 0;
  std::vector<float> x_list;
  std::vector<float> y_list;
  float prom_x = 0;
  float prom_y = 0;
  geometry_msgs::msg::Point position;

  FramePublisher()
      : Node("FramePublisher_Node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    count = 0;
    x_list.clear();
    y_list.clear();

    // cart_frame
    static_transform_.header.frame_id = "robot_cart_laser";
    static_transform_.child_frame_id = "approach_point";

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

  void timer_on() {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FramePublisher::publish_frame, this));
  }

  void timer_off() { timer_->cancel(); }

  void publish_frame() {

    try {

      rclcpp::Time now = rclcpp::Clock().now();

      rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(100000000);

      if (tf_buffer_.canTransform("robot_cart_laser", "map", now, timeout)) {

        auto transform = tf_buffer_.lookupTransform(
            "map", "robot_cart_laser", tf2::TimePoint(),
            tf2::Duration(static_cast<long int>(0.1 * 1e9)));

        if (transform.transform.translation.y > 0.2) {
          // Shelf in left side

          static_transform_.header.stamp = this->now();
          static_transform_.transform.translation.x = +0.7;
          static_transform_.transform.translation.y = 0.0;
          static_transform_.transform.translation.z = 0.0;

          static_transform_.transform.rotation.x = 0.0;

          static_transform_.transform.rotation.y = 0.0;

          static_transform_.transform.rotation.z = 0.0;

          static_transform_.transform.rotation.w = 1.0;

          broadcaster_->sendTransform(static_transform_);
          RCLCPP_INFO_ONCE(this->get_logger(), "canTransform: left");
          detect = true;
        }

        if (transform.transform.translation.y <= -0.3) {

          // Shelf in right side

          static_transform_.header.stamp = this->now();
          static_transform_.transform.translation.x = -0.7;
          static_transform_.transform.translation.y = 0.0;
          static_transform_.transform.translation.z = 0.0;

          static_transform_.transform.rotation.x = 0.0;

          static_transform_.transform.rotation.y = 0.0;

          static_transform_.transform.rotation.z = 0.0;

          static_transform_.transform.rotation.w = 1.0;

          broadcaster_->sendTransform(static_transform_);
          RCLCPP_INFO_ONCE(this->get_logger(), "canTransform: right");
          detect = true;
        }

      } else {
        RCLCPP_INFO(this->get_logger(), "canTransform: FALSE");
      }

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
  }
};

// START HERE
class GenericServer : public rclcpp::Node {
public:
  std::shared_ptr<FramePublisher> framePublisher;

  GenericServer() : Node("server_pub_frame") {

    // Initialize FrameFollower and FramePublisher
    framePublisher = std::make_shared<FramePublisher>();

    // Create Server
    srv_ = this->create_service<EmptySrv>(
        "approach_shelf",
        std::bind(&GenericServer::service_callback, this, std::placeholders::_1,
                  std::placeholders::_2),
        ::rmw_qos_profile_default);

    RCLCPP_INFO(this->get_logger(), "Server Pub Frame is READY!");
  }

private:
  rclcpp::Service<EmptySrv>::SharedPtr srv_;

  void service_callback(const std::shared_ptr<EmptySrv::Request> request,
                        const std::shared_ptr<EmptySrv::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Server has been called!");

    // Start publisher timer
    framePublisher->timer_on();

    int count = 0;
    while (count <= 3) {

      sleep(1);
      count += 1;
    }

    if (framePublisher->detect) {

      RCLCPP_INFO(this->get_logger(), "Frame Detected!");
      framePublisher->timer_off();
    } else {

      RCLCPP_INFO(this->get_logger(), "There's any shelf here");
    }
  }
}; // GenericServer

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto genericServer = std::make_shared<GenericServer>();
  auto framePublisher = genericServer->framePublisher;

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(genericServer);
  executor.add_node(framePublisher);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}