#pragma once
#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "vanjee_lidar_msg/msg/device_ctrl.hpp"

namespace vanjee {
namespace lidar {
template <typename T_RosMsg>
class VanjeeLidarSdkPublishRosMsg {
 public:
  using NodePtr = std::shared_ptr<rclcpp::Node>;
  explicit VanjeeLidarSdkPublishRosMsg(NodePtr node_ptr, std::string topic_name) {
    pub_ = node_ptr->create_publisher<T_RosMsg>(topic_name, 10);
  }
  void PublishMsg(T_RosMsg& msg) {
    pub_->publish(msg);
  }

 private:
  typename rclcpp::Publisher<T_RosMsg>::SharedPtr pub_;
};

template <typename T_RosMsg, typename T_SdkMsg>
class VanjeeLidarSdkSubscribeRosMsg {
 public:
  using NodePtr = std::shared_ptr<rclcpp::Node>;
  explicit VanjeeLidarSdkSubscribeRosMsg(const NodePtr node_ptr, const std::string& topic_name) : node_ptr_(node_ptr), topic_name_(topic_name) {
  }

  void SubscribeTopic(std::function<void(const std::shared_ptr<T_RosMsg>)> subscribe_topic_callback) {
    sub_ = node_ptr_->create_subscription<T_RosMsg>(topic_name_, 10, subscribe_topic_callback);
  }

 private:
  typename rclcpp::Subscription<T_RosMsg>::SharedPtr sub_;
  NodePtr node_ptr_;
  std::string topic_name_;
};

class VanjeeLidarSdkNode : public rclcpp::Node {
 public:
  static std::shared_ptr<VanjeeLidarSdkNode> CreateInstance() {
    static std::shared_ptr<VanjeeLidarSdkNode> vanjee_lidar_sdk_node_ptr(new VanjeeLidarSdkNode());
    return vanjee_lidar_sdk_node_ptr;
  }

  VanjeeLidarSdkNode(const VanjeeLidarSdkNode&) = delete;
  VanjeeLidarSdkNode& operator=(const VanjeeLidarSdkNode&) = delete;

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::PointCloud2>> GetPointCloud2MsgPublisher(std::string topic_name) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::PointCloud2>>(CreateInstance(), topic_name);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::LaserScan>> GetLaserScanMsgPublisher(std::string topic_name) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::LaserScan>>(CreateInstance(), topic_name);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::Imu>> GetImuMsgPublisher(std::string topic_name) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::Imu>>(CreateInstance(), topic_name);
  }

  std::shared_ptr<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_msg::msg::DeviceCtrl>> GetDeviceCtrlMsgPublisher(std::string topic_name) {
    return std::make_shared<VanjeeLidarSdkPublishRosMsg<vanjee_lidar_msg::msg::DeviceCtrl>>(CreateInstance(), topic_name);
  }

  std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_msg::msg::DeviceCtrl, vanjee::lidar::DeviceCtrl>> GetDeviceCtrlMsgSubscriber(
      std::string topic_name) {
    return std::make_shared<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_msg::msg::DeviceCtrl, vanjee::lidar::DeviceCtrl>>(CreateInstance(),
                                                                                                                         topic_name);
  }

 private:
  VanjeeLidarSdkNode() : Node("vanjee_lidar_sdk_node") {
  }
};
}  // namespace lidar
}  // namespace vanjee

#endif