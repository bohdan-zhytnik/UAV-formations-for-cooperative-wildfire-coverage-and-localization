// Start the standard RViz application, then enable this scenario's image docks
// after Qt has restored their positions. Restoring docks can temporarily emit
// visibilityChanged(false), which disables Image displays during startup.
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <QApplication>
#include <QDockWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/visualizer_app.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

// A scenario-specific Qt dock. MRS still executes every flight command through
// its normal reference service; this panel only selects the goal settings.
class DemoControls : public QDockWidget
{
public:
  explicit DemoControls(QWidget * parent) : QDockWidget("Fire demo controls", parent)
  {
    setObjectName("FireDemoControls");
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    node_ = std::make_shared<rclcpp::Node>("fire_demo_controls", options);
    auto durable = rclcpp::QoS(1).transient_local();
    settings_ = node_->create_publisher<std_msgs::msg::String>("/fire_demo/goal_settings", durable);
    actions_ = node_->create_publisher<std_msgs::msg::String>("/fire_demo/action", 10);
    auto * content = new QWidget(this);
    auto * layout = new QVBoxLayout(content);
    auto * form = new QFormLayout();
    vehicle_ = new QComboBox(content);
    vehicle_->addItem("uav1 (blue)", "uav1");
    altitude_ = new QDoubleSpinBox(content);
    altitude_->setRange(1.0, 9.0);
    altitude_->setSingleStep(0.5);
    altitude_->setSuffix(" m");
    altitude_->setValue(5.0);
    form->addRow("Drone", vehicle_);
    form->addRow("Flight altitude", altitude_);
    layout->addLayout(form);
    enabled_ = new QCheckBox("Enable map commands", content);
    layout->addWidget(enabled_);
    auto * instructions = new QLabel(
      "1. Take off: ./takeoff.sh\n2. Set altitude and enable commands.\n"
      "3. Use 2D Goal Pose: click and drag.\nAltitude changes apply to the next goal.", content);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);
    auto * hover = new QPushButton("Hover / cancel current target", content);
    auto * reset = new QPushButton("Clear fire estimate and flight trail", content);
    layout->addWidget(hover);
    layout->addWidget(reset);
    command_status_ = new QLabel("Waiting for command node...", content);
    command_status_->setWordWrap(true);
    command_status_->setMaximumWidth(300);
    layout->addWidget(command_status_);
    fire_status_ = new QLabel("Waiting for fire detector...", content);
    fire_status_->setWordWrap(true);
    fire_status_->setMaximumWidth(300);
    layout->addWidget(fire_status_);
    layout->addStretch();
    setWidget(content);
    setMinimumWidth(280);
    connect(enabled_, &QCheckBox::toggled, this, [this]() { publishSettings(); });
    connect(altitude_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      [this]() { publishSettings(); });
    connect(vehicle_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
      [this]() { publishSettings(); });
    connect(hover, &QPushButton::clicked, this, [this]() { action("hover"); });
    connect(reset, &QPushButton::clicked, this, [this]() { action("reset_map"); });
    command_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/fire_demo/command_status", durable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        command_status_->setText(QString::fromStdString(msg->data));
      });
    fire_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/fire_demo/fire_status", durable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        auto object = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
        auto centre = object["centre_m"].toArray();
        QString position = centre.size() == 2 ?
          QString("Fire 1: (%1, %2) m\n").arg(centre[0].toDouble(), 0, 'f', 2).arg(centre[1].toDouble(), 0, 'f', 2) : "";
        fire_status_->setText(position + object["note"].toString() +
          "\nYellow ellipse: illustrative 2-sigma repeatability; 3 cm floor.");
      });
    auto * spin = new QTimer(this);
    connect(spin, &QTimer::timeout, this, [this]() {
      if (rclcpp::ok()) { rclcpp::spin_some(node_); }
    });
    spin->start(50);
    publishSettings();
  }

private:
  void publishSettings()
  {
    QJsonObject object;
    object["uav"] = vehicle_->currentData().toString();
    object["altitude_m"] = altitude_->value();
    object["enabled"] = enabled_->isChecked();
    std_msgs::msg::String message;
    message.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
    settings_->publish(message);
  }
  void action(const std::string & name)
  {
    std_msgs::msg::String message;
    message.data = name;
    actions_->publish(message);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr settings_, actions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_, fire_subscription_;
  QComboBox * vehicle_;
  QDoubleSpinBox * altitude_;
  QCheckBox * enabled_;
  QLabel * command_status_;
  QLabel * fire_status_;
};

int main(int argc, char ** argv)
{
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qt_args;
  for (auto & arg : args) { qt_args.push_back(arg.data()); }
  int qt_argc = static_cast<int>(qt_args.size());
  QApplication app(qt_argc, qt_args.data());
  rviz_common::VisualizerApp rviz(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  rviz.setApp(&app);
  if (!rviz.init(argc, argv)) { return 1; }
  QTimer::singleShot(1500, [&app]() {
    for (auto * widget : app.topLevelWidgets()) {
      auto * frame = qobject_cast<rviz_common::VisualizationFrame *>(widget);
      if (!frame) { continue; }
      auto * controls = new DemoControls(frame);
      frame->addDockWidget(Qt::LeftDockWidgetArea, controls);
      if (auto * displays = frame->findChild<QDockWidget *>("Displays")) {
        frame->splitDockWidget(displays, controls, Qt::Vertical);
      }
      controls->show();
      auto * root = frame->getManager()->getRootDisplayGroup();
      for (int i = 0; i < root->numDisplays(); ++i) {
        auto * display = root->getDisplayAt(i);
        if (display->getClassId() == "rviz_default_plugins/Image") {
          if (auto * dock = display->getAssociatedWidgetPanel()) {
            dock->overrideVisibility(false);
            dock->setVisible(true);
          }
          display->setEnabled(true);
          std::cout << "Enabled camera display: " << display->getName().toStdString() << std::endl;
        }
      }
    }
  });
  QTimer shutdown_monitor;
  QObject::connect(&shutdown_monitor, &QTimer::timeout, &app, [&app]() {
    if (!rclcpp::ok()) { app.quit(); }
  });
  shutdown_monitor.start(100);
  QObject::connect(&app, &QApplication::aboutToQuit, []() {
    if (rclcpp::ok()) { rclcpp::shutdown(); }
  });
  return app.exec();
}
