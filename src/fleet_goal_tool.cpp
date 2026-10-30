#include "fleet_selection.hpp"
#include <algorithm>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QUuid>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace fire_observation_demo
{
FleetSelection & fleet_selection()
{
  static FleetSelection selection;
  return selection;
}

class FleetGoalTool : public rviz_default_plugins::tools::PoseTool
{
public:
  FleetGoalTool() { shortcut_key_ = 'g'; setName("2D Fleet Goal"); }
  void onInitialize() override
  {
    PoseTool::onInitialize();
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = node->create_publisher<std_msgs::msg::String>("/fire_fleet/map_goal", 10);
    route_publisher_ = node->create_publisher<std_msgs::msg::String>("/fire_fleet/mission/config", 10);
    setName("2D Fleet Goal");
  }
protected:
  void onPoseSet(double x, double y, double heading) override
  {
    const auto selection = fleet_selection();
    QJsonObject object;
    object["uav"] = QString::fromStdString(selection.uav);
    if (selection.edit_route) {
      object["op"] = "append_waypoint";
      object["request_id"] = QUuid::createUuid().toString();
      object["frame"] = context_->getFixedFrame();
      object["waypoint"] = QJsonObject{{"position", QJsonArray{x, y, selection.altitude}}, {"heading", heading}};
      std_msgs::msg::String message;
      message.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
      route_publisher_->publish(message);
      return;
    }
    object["altitude_m"] = selection.altitude;
    object["enabled"] = selection.enabled;
    object["frame"] = context_->getFixedFrame();
    object["x"] = x;
    object["y"] = y;
    object["heading"] = heading;
    std_msgs::msg::String message;
    message.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
    publisher_->publish(message);
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_publisher_;
};

class SurveyRegionTool : public rviz_common::Tool
{
public:
  SurveyRegionTool() { shortcut_key_ = 'r'; setName("Survey Region"); }
  void onInitialize() override
  {
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = node->create_publisher<std_msgs::msg::String>("/fire_fleet/survey/selection", 10);
    setName("Survey Region");
  }
  void activate() override { dragging_ = false; setStatus("Drag opposite corners of the shared ground rectangle. This edits bounds only."); }
  void deactivate() override { dragging_ = false; }
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override
  {
    auto projection = projection_.getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(), event.x, event.y);
    if (!projection.first) {
      return 0;
    }
    const auto point = projection.second;
    if (event.leftDown()) { start_ = point; dragging_ = true; }
    if (event.leftUp() && dragging_) {
      dragging_ = false;
      QJsonObject data{{"frame", context_->getFixedFrame()},
        {"bounds", QJsonArray{std::min(double(start_.x), double(point.x)), std::min(double(start_.y), double(point.y)),
                              std::max(double(start_.x), double(point.x)), std::max(double(start_.y), double(point.y))}}};
      std_msgs::msg::String msg;
      msg.data = QJsonDocument(data).toJson(QJsonDocument::Compact).toStdString();
      publisher_->publish(msg);
      return Render | Finished;
    }
    return Render;
  }
private:
  bool dragging_ = false;
  rviz_rendering::ViewportProjectionFinder projection_;
  Ogre::Vector3 start_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}
PLUGINLIB_EXPORT_CLASS(fire_observation_demo::FleetGoalTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(fire_observation_demo::SurveyRegionTool, rviz_common::Tool)
