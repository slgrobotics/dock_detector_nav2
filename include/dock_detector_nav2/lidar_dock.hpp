#ifndef DOCK_DETECTOR_NAV2__LIDAR_DOCK_HPP_
#define DOCK_DETECTOR_NAV2__LIDAR_DOCK_HPP_
#include <chrono>
#include <future>
#include <mutex>
#include "opennav_docking/simple_non_charging_dock.hpp"
#include "dock_detector_interfaces/msg/dock_detection.hpp"
#include "dock_detector_interfaces/srv/select_dock.hpp"

namespace dock_detector_nav2
{
class LidarDock : public opennav_docking::SimpleNonChargingDock
{
public:
  void configure(const nav2::LifecycleNode::WeakPtr &, const std::string &,
    nav2::TransformBuffer::SharedPtr) override;
  bool startDetectionProcess() override;
  bool stopDetectionProcess() override;
  bool getRefinedPose(geometry_msgs::msg::PoseStamped &, std::string) override;
  bool isDocked() override;
  void deactivate() override;
  void cleanup() override;
private:
  using Select = dock_detector_interfaces::srv::SelectDock;
  using Detection = dock_detector_interfaces::msg::DockDetection;
  using Steady = std::chrono::steady_clock;
  void receive(Detection::ConstSharedPtr);
  void reset();
  bool fresh() const;
  std::mutex mutex_;
  rclcpp::Client<Select>::SharedPtr client_;
  nav2::Subscription<Detection>::SharedPtr subscription_;
  std::shared_future<Select::Response::SharedPtr> future_;
  int64_t pending_id_{0};
  std::string id_, token_;
  Detection latest_;
  bool active_{false}, acknowledged_{false}, received_{false}, refined_{false};
  Steady::time_point requested_, receipt_;
  rclcpp::Time selected_stamp_{0, 0, RCL_ROS_TIME};
  int64_t filtered_stamp_{0};
  double selection_timeout_{3.0}, filter_coefficient_{0.1};
};
}
#endif
