#include "dock_detector_nav2/lidar_dock.hpp"
#include <cmath>
#include <random>
#include <sstream>
#include "pluginlib/class_list_macros.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dock_detector_nav2
{
namespace
{
double yawError(const geometry_msgs::msg::Pose & robot,
  const geometry_msgs::msg::Pose & dock, double heading_offset)
{
  return std::remainder(
    tf2::getYaw(robot.orientation) - tf2::getYaw(dock.orientation) - heading_offset,
    2.0 * M_PI);
}
}  // namespace

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
  const double docking_threshold_yaw_deg = node_->declare_or_get_parameter(
    name + ".docking_threshold_yaw", 5.0);
  if (!std::isfinite(docking_threshold_yaw_deg) ||
    docking_threshold_yaw_deg <= 0.0 || docking_threshold_yaw_deg > 180.0)
  {
    throw std::invalid_argument("docking_threshold_yaw must be in (0, 180] degrees");
  }
  docking_threshold_yaw_rad_ = docking_threshold_yaw_deg * M_PI / 180.0;
  trace_enabled_ = node_->declare_or_get_parameter(name + ".trace_enabled", false);
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
  last_trace_ = {};
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
  if (!refined_ || !fresh() ||
    rclcpp::Time(latest_.pose.header.stamp).nanoseconds() != filtered_stamp_)
  {
    return false;
  }
  const bool contact = SimpleNonChargingDock::isDocked();
  const auto now = Steady::now();
  const bool trace_due = trace_enabled_ && now - last_trace_ >= std::chrono::milliseconds(500);
  if (!contact && !trace_due) {
    return false;
  }

  geometry_msgs::msg::PoseStamped base;
  base.header.frame_id = base_frame_id_;
  base.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(base, base, dock_pose_.header.frame_id);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double heading_offset =
    getDockDirection() == opennav_docking_core::DockDirection::BACKWARD ? M_PI : 0.0;
  const double filtered_yaw_error = yawError(base.pose, dock_pose_.pose, heading_offset);
  const bool arrived = contact && std::abs(filtered_yaw_error) <= docking_threshold_yaw_rad_;
  if (trace_enabled_ && (arrived || trace_due)) {
    last_trace_ = now;
    traceApproach(arrived, base, heading_offset, filtered_yaw_error);
  }
  return arrived;
}

void LidarDock::traceApproach(bool arrived, const geometry_msgs::msg::PoseStamped & base,
  double heading_offset, double filtered_yaw_error)
{
  auto raw = latest_.pose;
  const auto & frame = dock_pose_.header.frame_id;
  const char * phase = arrived ? "arrival" : "approach";
  try {
    if (raw.header.frame_id != frame) {
      tf2_buffer_->transform(raw, raw, frame);
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN(node_->get_logger(), "LiDAR dock %s id=%s trace TF unavailable: %s",
      phase, id_.c_str(), error.what());
    return;
  }

  const auto & robot = base.pose.position;
  const auto & measured = raw.pose.position;
  const auto & filtered = dock_pose_.pose.position;
  const double dx = robot.x - measured.x;
  const double dy = robot.y - measured.y;
  const rclcpp::Time scan_stamp(latest_.pose.header.stamp, node_->get_clock()->get_clock_type());
  const double scan_age_ms = (node_->now() - scan_stamp).seconds() * 1000.0;
  RCLCPP_INFO(node_->get_logger(),
    "DOCK: %s id=%s frame=%s scan_age_ms=%.1f robot=(%.3f,%.3f) "
    "detected=(%.3f,%.3f) filtered=(%.3f,%.3f) threshold_cm=%.2f threshold_yaw_deg=%.2f",
    phase, id_.c_str(), frame.c_str(), scan_age_ms,
    robot.x, robot.y, measured.x, measured.y, filtered.x, filtered.y, 100.0 * docking_threshold_,
    docking_threshold_yaw_rad_ * 180.0 / M_PI
  );
  RCLCPP_INFO(node_->get_logger(),
    "      xy_cm=%.2f yaw_deg=%+.2f filtered_xy_cm=%.2f dx_cm=%+.2f dy_cm=%+.2f "
    "filtered_yaw_deg=%+.2f",
    100.0 * std::hypot(dx, dy),
    yawError(base.pose, raw.pose, heading_offset) * 180.0 / M_PI,
    100.0 * std::hypot(robot.x - filtered.x, robot.y - filtered.y),
    100.0 * dx, 100.0 * dy,
    filtered_yaw_error * 180.0 / M_PI
  );
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
