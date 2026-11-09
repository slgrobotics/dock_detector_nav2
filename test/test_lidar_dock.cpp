#include <gtest/gtest.h>
#include <thread>
#include "dock_detector_nav2/lidar_dock.hpp"
#include "pluginlib/class_loader.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
using namespace std::chrono_literals;
using Select = dock_detector_interfaces::srv::SelectDock;
using Detection = dock_detector_interfaces::msg::DockDetection;

class PluginTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2::LifecycleNode>("plugin_test");
    node->declare_parameter("base_frame", std::string("base_link"));
    node->declare_parameter("lidar.external_detection_translation_x", 0.);
    node->declare_parameter("lidar.external_detection_translation_y", 0.);
    node->declare_parameter("lidar.external_detection_rotation_roll", 0.);
    node->declare_parameter("lidar.external_detection_rotation_pitch", 0.);
    node->declare_parameter("lidar.external_detection_timeout", .15);
    node->declare_parameter("lidar.selection_timeout", .2);
    node->declare_parameter("lidar.dock_direction", std::string("backward"));
    node->declare_parameter("lidar.docking_threshold_yaw", 3.0);
    node->declare_parameter("lidar.staging_x_offset", -.8);
    node->declare_parameter("lidar.staging_yaw_offset", M_PI);
    server = std::make_shared<rclcpp::Node>("detector_stub");
    service = server->create_service<Select>("dock_detector/select_dock",
      [this](Select::Request::SharedPtr req, Select::Response::SharedPtr res) {
        requests.push_back(*req);
        res->success = req->dock_id != "unknown";
        res->dock_id = req->dock_id; res->acquisition_id = req->acquisition_id;
        if (req->activate) {selected = *req;}
      });
    publisher = server->create_publisher<Detection>("dock_detector/detection", 10);
    loader = std::make_unique<pluginlib::ClassLoader<opennav_docking_core::ChargingDock>>(
      "opennav_docking_core", "opennav_docking_core::ChargingDock");
    plugin = loader->createSharedInstance("dock_detector_nav2::LidarDock");
    tf = std::make_shared<nav2::TransformBuffer>(node->get_clock());
    plugin->configure(node, "lidar", tf);
    plugin->activate();
    executor.add_node(node->get_node_base_interface()); executor.add_node(server);
    ASSERT_TRUE(plugin->startDetectionProcess());
    pose.header.frame_id = "odom";
  }
  void TearDown() override
  {
    plugin->deactivate(); plugin->cleanup(); plugin.reset();
    executor.remove_node(node->get_node_base_interface()); executor.remove_node(server);
    node->shutdown();
  }
  void spin(int milliseconds = 30)
  {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some(); std::this_thread::sleep_for(2ms);
    }
  }
  void select(const std::string & id)
  {
    for (int i = 0; i < 30; ++i) {
      EXPECT_FALSE(plugin->getRefinedPose(pose, id)); spin(5);
      if (selected.dock_id == id) {
        // Consume service acknowledgement before any measurement is published.
        EXPECT_FALSE(plugin->getRefinedPose(pose, id)); spin(5); return;
      }
    }
    FAIL() << "Selection service not reached";
  }
  Detection measurement()
  {
    Detection msg;
    msg.dock_id = selected.dock_id; msg.acquisition_id = selected.acquisition_id;
    msg.pose.header.frame_id = "odom"; msg.pose.header.stamp = node->now();
    msg.pose.pose.position.x = 2.; msg.pose.pose.orientation.w = 1.;
    return msg;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  nav2::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr server;
  rclcpp::Service<Select>::SharedPtr service;
  rclcpp::Publisher<Detection>::SharedPtr publisher;
  std::unique_ptr<pluginlib::ClassLoader<opennav_docking_core::ChargingDock>> loader;
  opennav_docking_core::ChargingDock::Ptr plugin;
  nav2::TransformBuffer::SharedPtr tf;
  Select::Request selected;
  std::vector<Select::Request> requests;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(PluginTest, IdentifiedAcquisitionSwitchAndFreshness)
{
  select("a");
  auto first = measurement();
  auto wrong = first; wrong.dock_id = "b";
  publisher->publish(wrong); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "a"));
  wrong = first; wrong.acquisition_id = "old";
  publisher->publish(wrong); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "a"));
  publisher->publish(first); spin(); ASSERT_TRUE(plugin->getRefinedPose(pose, "a"));
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 2.);
  // Duplicate timestamps cannot refresh the steady-clock watchdog.
  spin(100); publisher->publish(first); spin(40);
  EXPECT_FALSE(plugin->getRefinedPose(pose, "a")); EXPECT_FALSE(plugin->isDocked());
  select("b");
  publisher->publish(first); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "b"));
  auto current = measurement(); publisher->publish(current); spin();
  ASSERT_TRUE(plugin->getRefinedPose(pose, "b"));
  plugin->stopDetectionProcess();
  EXPECT_FALSE(plugin->getRefinedPose(pose, "b"));
  ASSERT_TRUE(plugin->startDetectionProcess()); select("b");
  EXPECT_NE(selected.acquisition_id, current.acquisition_id);
  publisher->publish(current); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "b"));
}

TEST_F(PluginTest, ArrivalRequiresPositionAndBackwardHeading)
{
  select("a");
  auto msg = measurement();
  publisher->publish(msg);
  spin();
  ASSERT_TRUE(plugin->getRefinedPose(pose, "a"));

  geometry_msgs::msg::TransformStamped base;
  base.header.frame_id = "odom";
  base.child_frame_id = "base_link";
  base.transform.translation.x = msg.pose.pose.position.x;
  base.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(base, "test", true));
  EXPECT_FALSE(plugin->isDocked());

  tf2::Quaternion heading;
  // This heading is just over the configured 3-degree limit across the wrap.
  heading.setRPY(0.0, 0.0, -M_PI + 0.06);
  base.transform.rotation = tf2::toMsg(heading);
  ASSERT_TRUE(tf->setTransform(base, "test", true));
  EXPECT_FALSE(plugin->isDocked());

  // A smaller yaw on the negative side of the wrap is within the limit.
  heading.setRPY(0.0, 0.0, -M_PI + 0.04);
  base.transform.rotation = tf2::toMsg(heading);
  ASSERT_TRUE(tf->setTransform(base, "test", true));
  EXPECT_TRUE(plugin->isDocked());

  base.transform.translation.x += 0.1;
  ASSERT_TRUE(tf->setTransform(base, "test", true));
  EXPECT_FALSE(plugin->isDocked());
}

TEST_F(PluginTest, UnknownDockAndUnnamedGoalFail)
{
  EXPECT_THROW(plugin->getRefinedPose(pose, ""), opennav_docking_core::FailedToDetectDock);
  EXPECT_FALSE(plugin->getRefinedPose(pose, "unknown"));
  spin(100);
  EXPECT_THROW(plugin->getRefinedPose(pose, "unknown"), opennav_docking_core::FailedToDetectDock);
}

TEST_F(PluginTest, StagingUsesNav2InwardHeading)
{
  geometry_msgs::msg::Pose dock;
  dock.position.x = -1.; dock.position.y = 3.; dock.orientation.z = 1.; dock.orientation.w = 0.;
  auto staging = plugin->getStagingPose(dock, "map");
  EXPECT_NEAR(staging.pose.position.x, -.2, 1e-8);
  EXPECT_NEAR(staging.pose.position.y, 3., 1e-8);
  EXPECT_NEAR(std::abs(staging.pose.orientation.w), 1., 1e-8);
}

TEST_F(PluginTest, SelectionDeadlineUsesWallTime)
{
  service.reset();
  EXPECT_FALSE(plugin->getRefinedPose(pose, "a"));
  spin(250);
  EXPECT_THROW(plugin->getRefinedPose(pose, "a"), opennav_docking_core::FailedToDetectDock);
}

TEST_F(PluginTest, InvalidAndFutureMeasurementsAreRejected)
{
  select("a");
  auto msg = measurement(); msg.pose.pose.orientation.w = 0.;
  publisher->publish(msg); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "a"));
  msg = measurement(); msg.pose.header.stamp = node->now() + rclcpp::Duration::from_seconds(1.);
  publisher->publish(msg); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "a"));
  msg = measurement(); msg.pose.header.stamp = node->now() - rclcpp::Duration::from_seconds(1.);
  publisher->publish(msg); spin(); EXPECT_FALSE(plugin->getRefinedPose(pose, "a"));
}

TEST_F(PluginTest, Nav2DatabaseIdentityIsForwardedToSelection)
{
  node->declare_parameter("bay.type", std::string("lidar"));
  node->declare_parameter("bay.pose", std::vector<double>{-1., 3., M_PI});
  node->declare_parameter("bay.id", std::string("catalogue_key"));
  DockMap database;
  ASSERT_TRUE(utils::parseDockParams({"bay"}, node, database));
  ASSERT_EQ(database.at("bay").id, "catalogue_key");
  select(database.at("bay").id);
  EXPECT_EQ(selected.dock_id, "catalogue_key");
}
