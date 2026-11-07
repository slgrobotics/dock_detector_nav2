#include "dock_detector_nav2/lidar_dock.hpp"
#include <cmath>
#include <random>
#include <sstream>
#include "pluginlib/class_list_macros.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dock_detector_nav2
{
void LidarDock::configure(const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & name, nav2::TransformBuffer::SharedPtr tf)
{
  SimpleNonChargingDock::configure(parent, name, tf);
  // Never consume anonymous PoseStamped observations or Trigger-based activation.
  use_external_detection_pose_ = true;
  dock_pose_sub_.reset();
  detector_client_.reset();
  auto service = node_->declare_or_get_parameter(name + ".selection_service",
    std::string("dock_detector/select_dock"));
  auto topic = node_->declare_or_get_parameter(name + ".detection_topic",
    std::string("dock_detector/detection"));
  selection_timeout_ = node_->declare_or_get_parameter(name + ".selection_timeout", 3.0);
  filter_coefficient_ = node_->declare_or_get_parameter(name + ".filter_coef", 0.1);
  if (!std::isfinite(selection_timeout_) || selection_timeout_ <= 0 ||
    !std::isfinite(external_detection_timeout_) || external_detection_timeout_ <= 0 ||
    !std::isfinite(filter_coefficient_) || filter_coefficient_ < 0 || filter_coefficient_ > 1)
  {
    throw std::invalid_argument("Invalid LiDAR docking timeouts or filter coefficient");
  }
  // This detector already publishes the Nav2 parking pose; transforms must be identity.
  if (std::abs(external_detection_translation_x_) > 1e-9 ||
    std::abs(external_detection_translation_y_) > 1e-9 ||
    external_detection_rotation_.angleShortestPath(tf2::Quaternion(0, 0, 0, 1)) > 1e-6)
  {
    throw std::invalid_argument("LidarDock requires identity external detection transforms");
  }
  if (shouldRotateToDock()) {
    throw std::invalid_argument("LidarDock requires rotate_to_dock=false for continuous detection");
  }
  client_ = rclcpp::create_client<Select>(node_->get_node_base_interface(),
    node_->get_node_graph_interface(), node_->get_node_services_interface(), service);
  subscription_ = node_->create_subscription<Detection>(topic,
    [this](Detection::ConstSharedPtr msg) {receive(msg);}, nav2::qos::StandardTopicQoS());
}

void LidarDock::reset()
{
  if (future_.valid() && client_) {client_->remove_pending_request(pending_id_);}
  future_ = {};
  id_.clear(); token_.clear();
  acknowledged_ = received_ = refined_ = false;
  filtered_stamp_ = 0;
  latest_ = Detection();
  dock_pose_ = geometry_msgs::msg::PoseStamped();
  filter_ = std::make_unique<opennav_docking::PoseFilter>(
    filter_coefficient_, external_detection_timeout_);
}

bool LidarDock::startDetectionProcess()
{
  stopDetectionProcess();
  std::lock_guard<std::mutex> guard(mutex_);
  active_ = true;
  return true;
}

bool LidarDock::stopDetectionProcess()
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (client_ && !token_.empty()) {
    auto request = std::make_shared<Select::Request>();
    request->activate = false; request->dock_id = id_; request->acquisition_id = token_;
    // Best-effort release. Token matching prevents a late release cancelling a new session.
    auto future = client_->async_send_request(request);
    client_->remove_pending_request(future.request_id);
  }
  active_ = false;
  reset();
  return true;
}

void LidarDock::receive(Detection::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!active_ || !acknowledged_ || msg->dock_id != id_ || msg->acquisition_id != token_) {
    return;
  }
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (msg->pose.header.frame_id.empty() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(p.z) || !std::isfinite(norm) || std::abs(norm - 1.) > .01)
  {return;}
  rclcpp::Time stamp(msg->pose.header.stamp, node_->get_clock()->get_clock_type());
  double age = (node_->now() - stamp).seconds();
  if (stamp.nanoseconds() <= 0 || stamp <= selected_stamp_ || age < -.1 ||
    age > external_detection_timeout_ ||
    (received_ && stamp <= rclcpp::Time(latest_.pose.header.stamp, stamp.get_clock_type())))
  {return;}
  latest_ = *msg;
  receipt_ = Steady::now();
  received_ = true;
}

bool LidarDock::fresh() const
{
  if (!active_ || !acknowledged_ || !received_) {return false;}
  auto stamp = rclcpp::Time(latest_.pose.header.stamp, node_->get_clock()->get_clock_type());
  double age = (node_->now() - stamp).seconds();
  return age >= -.1 && age <= external_detection_timeout_ &&
    std::chrono::duration<double>(Steady::now() - receipt_).count() <= external_detection_timeout_;
}

bool LidarDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!active_) {return false;}
  if (id.empty()) {
    throw opennav_docking_core::FailedToDetectDock("LidarDock requires a named dock_id");
  }
  if (id_ != id) {
    reset();
    id_ = id;
    std::random_device random;
    std::ostringstream token;
    for (int i = 0; i < 4; ++i) {token << std::hex << random() << '-';}
    token_ = token.str();
    selected_stamp_ = node_->now();
    requested_ = Steady::now();
  }
  if (!acknowledged_) {
    if (std::chrono::duration<double>(Steady::now() - requested_).count() > selection_timeout_) {
      throw opennav_docking_core::FailedToDetectDock("Detector selection timeout");
    }
    if (!future_.valid()) {
      if (!client_->service_is_ready()) {return false;}
      auto request = std::make_shared<Select::Request>();
      request->activate = true; request->dock_id = id_; request->acquisition_id = token_;
      auto pending = client_->async_send_request(request);
      pending_id_ = pending.request_id;
      future_ = pending.future.share();
      return false;
    }
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {return false;}
    auto reply = future_.get();
    future_ = {};
    if (!reply->success || reply->dock_id != id_ || reply->acquisition_id != token_) {
      throw opennav_docking_core::FailedToDetectDock("Detector refused dock selection: " + reply->message);
    }
    acknowledged_ = true;
    return false;  // Wait for a fresh, identified measurement AFTER acknowledgement.
  }
  if (!fresh()) {return false;}
  auto detected = latest_.pose;
  try {
    if (detected.header.frame_id != pose.header.frame_id) {
      tf2_buffer_->transform(detected, detected, pose.header.frame_id);
    }
  } catch (const tf2::TransformException &) {return false;}
  int64_t stamp = rclcpp::Time(detected.header.stamp).nanoseconds();
  if (stamp != filtered_stamp_) {
    detected = filter_->update(detected);
    dock_pose_ = detected;
    dock_pose_.pose.position.z = 0.;
    filtered_stamp_ = stamp;
    refined_ = true;
    filtered_dock_pose_pub_->publish(dock_pose_);
    dock_pose_pub_->publish(dock_pose_);
  }
  pose = dock_pose_;
  return true;
}

bool LidarDock::isDocked()
{
  std::lock_guard<std::mutex> guard(mutex_);
  return refined_ && fresh() &&
    rclcpp::Time(latest_.pose.header.stamp).nanoseconds() == filtered_stamp_ &&
    SimpleNonChargingDock::isDocked();
}

void LidarDock::deactivate()
{
  stopDetectionProcess();
  SimpleNonChargingDock::deactivate();
}

void LidarDock::cleanup()
{
  stopDetectionProcess();
  subscription_.reset(); client_.reset();
  SimpleNonChargingDock::cleanup();
}
}
PLUGINLIB_EXPORT_CLASS(dock_detector_nav2::LidarDock, opennav_docking_core::ChargingDock)
