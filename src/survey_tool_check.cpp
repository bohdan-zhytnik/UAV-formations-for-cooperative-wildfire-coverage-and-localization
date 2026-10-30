// Exercise the actual RViz plugin with mouse events on its own render panel.
#include <cmath>
#include <iostream>
#include <vector>
#include <QApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/visualizer_app.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

int main(int argc, char ** argv)
{
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qt_args;
  for (auto & arg : args) { qt_args.push_back(arg.data()); }
  int qt_argc = static_cast<int>(qt_args.size());
  QApplication app(qt_argc, qt_args.data());
  rviz_common::VisualizerApp rviz(std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  rviz.setApp(&app);
  if (!rviz.init(argc, argv)) { return 1; }
  auto node = std::make_shared<rclcpp::Node>("survey_tool_interaction_check");
  bool sent = false;
  int result = 1;
  auto restore = node->create_publisher<std_msgs::msg::String>("/fire_fleet/survey/selection", 10);
  auto subscription = node->create_subscription<std_msgs::msg::String>("/fire_fleet/survey/selection", 10,
    [&](std_msgs::msg::String::ConstSharedPtr msg) {
      if (!sent || result == 0) { return; }
      auto data = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
      auto bounds = data["bounds"].toArray();
      if (bounds.size() != 4 || data["frame"].toString() != "common_origin") { return; }
      if (!(bounds[0].toDouble() < bounds[2].toDouble() && bounds[1].toDouble() < bounds[3].toDouble())) { return; }
      result = 0;
      std::cout << "PASS: real SurveyRegion tool projected a drag to ordered ground bounds: " << msg->data << std::endl;
      std_msgs::msg::String original;
      original.data = R"({"frame":"common_origin","bounds":[5,-5,17,9]})";
      restore->publish(original);
      QTimer::singleShot(300, &app, &QApplication::quit);
    });
  QTimer spin;
  QObject::connect(&spin, &QTimer::timeout, &app, [&]() { rclcpp::spin_some(node); });
  spin.start(50);
  QTimer::singleShot(3000, [&]() {
    for (auto * widget : app.topLevelWidgets()) {
      auto * frame = qobject_cast<rviz_common::VisualizationFrame *>(widget);
      if (!frame) { continue; }
      auto * manager = frame->getManager();
      auto * tools = manager->getToolManager();
      for (int i = 0; i < tools->numTools(); ++i) {
        auto * tool = tools->getTool(i);
        if (tool->getClassId() != "fire_observation_demo/SurveyRegion") { continue; }
        auto * panel = manager->getRenderPanel();
        const QPointF start(panel->width()*.4, panel->height()*.45);
        const QPointF end(panel->width()*.7, panel->height()*.7);
        tool->activate();
        QMouseEvent press(QEvent::MouseButtonPress, start, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
        QMouseEvent release(QEvent::MouseButtonRelease, end, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
        rviz_common::ViewportMouseEvent first(panel, &press, start.x(), start.y());
        rviz_common::ViewportMouseEvent last(panel, &release, start.x(), start.y());
        sent = true;
        tool->processMouseEvent(first);
        if (!(tool->processMouseEvent(last) & rviz_common::Tool::Finished)) {
          std::cerr << "FAIL: drag did not finish" << std::endl;
        }
        tool->deactivate();
      }
    }
  });
  QTimer::singleShot(12000, &app, &QApplication::quit);
  app.exec();
  if (rclcpp::ok()) { rclcpp::shutdown(); }
  return result;
}
