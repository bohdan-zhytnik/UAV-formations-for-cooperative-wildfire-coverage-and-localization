#include "fleet_selection.hpp"
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QTableWidget>
#include <QHeaderView>
#include <QTabWidget>
#include <QScrollArea>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QUuid>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/visualizer_app.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

using fire_observation_demo::FleetSelection;

class FleetControls : public QDockWidget
{
public:
  explicit FleetControls(rviz_common::VisualizationFrame * frame)
  : QDockWidget("Two-drone controls", frame), frame_(frame)
  {
    setObjectName("FireFleetControls");
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    node_ = std::make_shared<rclcpp::Node>("fire_fleet_controls", options);
    auto durable = rclcpp::QoS(1).transient_local();
    settings_ = node_->create_publisher<std_msgs::msg::String>("/fire_fleet/settings", durable);
    actions_ = node_->create_publisher<std_msgs::msg::String>("/fire_fleet/action", 10);
    mission_config_ = node_->create_publisher<std_msgs::msg::String>("/fire_fleet/mission/config", 10);
    states_["uav1"] = {"uav1", 5.0, false};
    states_["uav2"] = {"uav2", 5.0, false};
    auto * content = new QWidget(this);
    auto * layout = new QVBoxLayout(content);
    auto * form = new QFormLayout();
    vehicle_ = new QComboBox(content);
    vehicle_->setObjectName("FleetVehicleSelector");
    vehicle_->addItem("uav1 (blue)", "uav1");
    vehicle_->addItem("uav2 (green)", "uav2");
    altitude_ = new QDoubleSpinBox(content);
    altitude_->setRange(1.0, 6.0);
    altitude_->setSingleStep(0.5);
    altitude_->setSuffix(" m");
    altitude_->setValue(5.0);
    form->addRow("Selected drone", vehicle_);
    form->addRow("Next goal altitude", altitude_);
    layout->addLayout(form);
    enabled_ = new QCheckBox("Enable map goals for selected drone", content);
    layout->addWidget(enabled_);
    auto * tabs = new QTabWidget(content);
    layout->addWidget(tabs);
    auto * manual = new QWidget(tabs);
    tabs->addTab(manual, "Manual");
    layout = new QVBoxLayout(manual);
    auto * instructions = new QLabel(
      "Take off and wait for the climb.\nEnable map goals, then use 2D Fleet Goal (g).\n"
      "Click and drag to choose position and heading.\nSelection changes the cameras; it sends no flight command.", content);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);
    button(layout, content, "Take off selected", "takeoff");
    button(layout, content, "Hover selected", "hover");
    button(layout, content, "Land selected", "land");
    button(layout, content, "Hover both drones", "hover_all");
    command_ = new QLabel("Waiting for the fleet interface...", content);
    command_->setWordWrap(true);
    command_->setMaximumWidth(310);
    layout->addWidget(command_);
    fleet_ = new QLabel("Waiting for both vehicles...", content);
    fleet_->setWordWrap(true);
    layout->addWidget(fleet_);
    fire_ = new QLabel("Selected camera: waiting for detections", content);
    fire_->setWordWrap(true);
    fire_->setMaximumWidth(310);
    layout->addWidget(fire_);
    layout->addStretch();
    createMissionTab(tabs);
    createSurveyTab(tabs);
    connect(tabs, &QTabWidget::currentChanged, this, [this](int index) {
      if (index == 0) { edit_route_->setChecked(false); }
    });
    setWidget(content);
    setMinimumWidth(300);
    connect(vehicle_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() { select(); });
    connect(altitude_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this]() { settings(); });
    connect(enabled_, &QCheckBox::toggled, this, [this]() { settings(); });
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/mission/status", durable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        missions_ = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
        updateMission();
      }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/mission/config_ack", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        auto ack = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
        if (ack["uav"].toString() == "all" || ack["uav"].toString().toStdString() == current_) {
          mission_notice_->setText((ack["success"].toBool() ? "" : "Rejected: ")+ack["message"].toString());
          survey_notice_->setText((ack["success"].toBool() ? "" : "Rejected: ")+ack["message"].toString());
        }
      }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/survey/selection", 10, [this, tabs](std_msgs::msg::String::ConstSharedPtr msg) {
        auto data = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
        if (data["frame"].toString() != "common_origin") {
          survey_notice_->setText("Set RViz Fixed Frame to common_origin before drawing"); return;
        }
        auto bounds = data["bounds"].toArray();
        if (bounds.size() != 4) { return; }
        for (int i = 0; i < 4; ++i) { survey_bounds_[i]->setValue(bounds[i].toDouble()); }
        survey_notice_->setText("Rectangle selected. Pause both missions, then Apply region and split.");
        tabs->setCurrentIndex(2);
      }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/"+std::string("uav1")+"/action_ack", durable,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { missionActionAck(msg); }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/"+std::string("uav2")+"/action_ack", durable,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { missionActionAck(msg); }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/select_uav", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        int index = vehicle_->findData(QString::fromStdString(msg->data));
        if (index >= 0) {
          vehicle_->setCurrentIndex(index);
          settings();
        }
      }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/status", durable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        auto object = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
        auto vehicles = object["uavs"].toObject();
        QStringList lines;
        for (const auto & name : {QString("uav1"), QString("uav2")}) {
          auto state = vehicles[name].toObject();
          auto position = state["position_m"].toArray();
          QString height = position.size() == 3 ? QString::number(position[2].toDouble(), 'f', 1)+" m" : "no TF";
          lines << name+": "+(state["connected"].toBool() ?
            (state["armed"].toBool() ? "ARMED" : "DISARMED") : "NO CONNECTION")+" | "+height;
          if (name.toStdString() == current_) { command_->setText(state["message"].toString()); }
        }
        fleet_->setText(lines.join("\n"));
      }));
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/notice", durable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        command_->setText(QString::fromStdString(msg->data));
      }));
    auto * map_dock = new QDockWidget("Shared fire map", frame_);
    map_dock->setObjectName("FireFleetSharedMap");
    auto * map_content = new QWidget(map_dock);
    auto * map_layout = new QVBoxLayout(map_content);
    map_table_ = new QTableWidget(0, 7, map_content);
    map_table_->setHorizontalHeaderLabels({"Fire ID", "Centre x, y [m]", "Area [m^2]",
      "Mean / max [K]", "Last seen [sim s]", "Contributing drones", "Visible from"});
    map_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    map_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    map_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    map_table_->horizontalHeader()->setStretchLastSection(true);
    map_layout->addWidget(map_table_);
    map_layout->addWidget(new QLabel("Ellipses show illustrative 2-sigma repeatability; they are not calibrated accuracy. "
      "Raw per-drone observations can be enabled in Displays.", map_content));
    map_dock->setWidget(map_content);
    map_dock->setMinimumHeight(135);
    map_dock->setMaximumHeight(190);
    frame_->addDockWidget(Qt::BottomDockWidgetArea, map_dock);
    subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/fire_fleet/fire_map/status", durable, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        auto object = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
        auto fires = object["fires"].toArray();
        map_table_->setRowCount(fires.size());
        int row = 0;
        for (const auto & value : fires) {
          auto fire = value.toObject();
          auto centre = fire["centre_m"].toArray();
          QStringList sources, visible;
          for (const auto & source : fire["contributing_uavs"].toArray()) { sources << source.toString(); }
          for (const auto & source : fire["visible_from"].toArray()) { visible << source.toString(); }
          QStringList cells{QString("Fire %1").arg(fire["id"].toInt()),
            QString("%1, %2").arg(centre[0].toDouble(), 0, 'f', 2).arg(centre[1].toDouble(), 0, 'f', 2),
            QString::number(fire["area_m2"].toDouble(), 'f', 2),
            QString("%1 / %2").arg(fire["mean_kelvin"].toDouble(), 0, 'f', 1).arg(fire["max_kelvin"].toDouble(), 0, 'f', 1),
            QString::number(fire["last_seen_age_s"].toDouble(), 'f', 1), sources.join(", "),
            visible.isEmpty() ? "out of view" : visible.join(", ")};
          for (int column = 0; column < cells.size(); ++column) {
            map_table_->setItem(row, column, new QTableWidgetItem(cells[column]));
          }
          ++row;
        }
      }));
    for (const auto & name : {std::string("uav1"), std::string("uav2")}) {
      subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
        "/fire_fleet/"+name+"/fire_status", durable,
        [this, name](std_msgs::msg::String::ConstSharedPtr msg) {
          fires_[name] = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
          if (name == current_) { updateFire(); }
        }));
    }
    auto * spin = new QTimer(this);
    connect(spin, &QTimer::timeout, this, [this]() { if (rclcpp::ok()) { rclcpp::spin_some(node_); } });
    spin->start(50);
    settings();
    prepareImages();
  }

private:
  void createMissionTab(QTabWidget * tabs)
  {
    auto * scroll = new QScrollArea(tabs);
    scroll->setWidgetResizable(true);
    auto * page = new QWidget(scroll);
    auto * layout = new QVBoxLayout(page);
    auto * hint = new QLabel("Uses the selected drone and altitude above.\nTake off manually before starting a mission.\nManual goals, Hover and Land pause the mission.", page);
    hint->setWordWrap(true);
    layout->addWidget(hint);
    edit_route_ = new QCheckBox("Add route points with 2D Fleet Goal (g)", page);
    edit_route_->setToolTip("Editing adds points only. Uncheck to send manual flight goals. Pause before editing.");
    layout->addWidget(edit_route_);
    connect(edit_route_, &QCheckBox::toggled, this, [this]() { settings(); });
    route_table_ = new QTableWidget(0, 4, page);
    layout->addWidget(new QLabel("Waypoint route only (cooperative search uses Survey)", page));
    route_table_->setHorizontalHeaderLabels({"#", "x", "y", "z"});
    route_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    route_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    route_table_->setMaximumHeight(135);
    layout->addWidget(route_table_);
    auto * row = new QHBoxLayout();
    for (const auto & pair : {std::pair<QString, QString>{"Undo", "pop_waypoint"},
        {"Clear", "clear_route"}, {"Preset", "load_preset"}}) {
      auto * b = new QPushButton(pair.first, page);
      row->addWidget(b);
      connect(b, &QPushButton::clicked, this, [this, pair]() {
        if (pair.second == "load_preset") { mode_->setCurrentIndex(mode_->findData("route")); }
        configure({{"op", pair.second}});
      });
    }
    layout->addLayout(row);
    auto * form = new QFormLayout();
    loop_ = new QCheckBox("Repeat route", page);
    loop_->setChecked(true);
    dwell_ = new QDoubleSpinBox(page);
    dwell_->setRange(1., 60.);
    dwell_->setValue(3.);
    dwell_->setSuffix(" sim s");
    form->addRow(loop_);
    form->addRow("Inspect each point", dwell_);
    mode_ = new QComboBox(page);
    mode_->addItem("Waypoint route", "route");
    mode_->addItem("Cooperative search / revisit", "cooperative");
    form->addRow("Start mode", mode_);
    layout->addLayout(form);
    auto * apply = new QPushButton("Apply repeat / dwell settings", page);
    layout->addWidget(apply);
    connect(apply, &QPushButton::clicked, this, [this]() {
      configure({{"op", "set_options"}, {"loop", loop_->isChecked()}, {"dwell_sim_s", dwell_->value()}});
    });
    for (const auto & pair : {std::pair<QString, std::string>{"Start selected", "mission_start"},
        {"Pause selected and hover", "mission_pause"}, {"Resume selected", "mission_resume"},
        {"Start both: cooperative", "mission_start_all"}}) {
      button(layout, page, pair.first, pair.second);
    }
    assigned_ = new QLineEdit(page);
    assigned_->setPlaceholderText("Fire IDs, e.g. 1,2 (empty = clear)");
    layout->addWidget(assigned_);
    auto * assign = new QPushButton("Assign these fires to selected drone", page);
    layout->addWidget(assign);
    connect(assign, &QPushButton::clicked, this, [this]() {
      QJsonArray ids;
      for (const auto & text : assigned_->text().split(',', Qt::SkipEmptyParts)) {
        bool ok;
        int id = text.trimmed().toInt(&ok);
        if (!ok || id < 1) { mission_notice_->setText("Enter positive fire IDs separated by commas"); return; }
        ids.append(id);
      }
      configure({{"op", "assign_fires"}, {"fire_ids", ids}});
    });
    mission_state_ = new QLabel("Waiting for mission coordinator...", page);
    mission_state_->setWordWrap(true);
    mission_state_->setMaximumWidth(310);
    layout->addWidget(mission_state_);
    mission_notice_ = new QLabel("Area and revisit interval: use missionctl.sh in tmux.\nEditing a route does not start flight.", page);
    mission_notice_->setWordWrap(true);
    mission_notice_->setMaximumWidth(310);
    layout->addWidget(mission_notice_);
    layout->addStretch();
    scroll->setWidget(page);
    tabs->addTab(scroll, "Missions");
  }
  void createSurveyTab(QTabWidget * tabs)
  {
    auto * scroll = new QScrollArea(tabs);
    scroll->setWidgetResizable(true);
    auto * page = new QWidget(scroll);
    auto * layout = new QVBoxLayout(page);
    auto * help = new QLabel("Shared survey for both drones.\nPress r (Survey Region), then drag opposite map corners, or enter bounds below.\nApply divides the rectangle equally; it sends no flight command.", page);
    help->setWordWrap(true);
    layout->addWidget(help);
    auto * form = new QFormLayout();
    const std::array<QString, 4> names{"Minimum x", "Minimum y", "Maximum x", "Maximum y"};
    const std::array<double, 4> defaults{5., -5., 17., 9.};
    for (int i = 0; i < 4; ++i) {
      survey_bounds_[i] = new QDoubleSpinBox(page);
      survey_bounds_[i]->setRange(-20., 20.); survey_bounds_[i]->setSingleStep(.5);
      survey_bounds_[i]->setValue(defaults[i]);
      form->addRow(names[i], survey_bounds_[i]);
    }
    survey_axis_ = new QComboBox(page);
    survey_axis_->addItem("Automatic (longer side)", "auto");
    survey_axis_->addItem("x: west / east", "x"); survey_axis_->addItem("y: south / north", "y");
    form->addRow("Split direction", survey_axis_);
    survey_altitude_ = new QDoubleSpinBox(page);
    survey_altitude_->setRange(1., 6.); survey_altitude_->setValue(5.); survey_altitude_->setSuffix(" m");
    form->addRow("Survey altitude", survey_altitude_);
    survey_spacing_ = new QDoubleSpinBox(page);
    survey_spacing_->setRange(.5, 3.); survey_spacing_->setValue(3.); survey_spacing_->setSuffix(" m");
    form->addRow("Coverage spacing", survey_spacing_);
    survey_revisit_ = new QDoubleSpinBox(page);
    survey_revisit_->setRange(3., 600.); survey_revisit_->setValue(20.); survey_revisit_->setSuffix(" sim s");
    form->addRow("Fire revisit target", survey_revisit_);
    layout->addLayout(form);
    auto * apply = new QPushButton("Apply region and split (both paused)", page);
    layout->addWidget(apply);
    connect(apply, &QPushButton::clicked, this, [this]() {
      QJsonArray bounds;
      for (auto * field : survey_bounds_) { bounds.append(field->value()); }
      configure({{"uav", "all"}, {"op", "set_survey"}, {"frame", "common_origin"},
        {"survey", QJsonObject{{"bounds", bounds}, {"split_axis", survey_axis_->currentData().toString()},
        {"altitude_m", survey_altitude_->value()}, {"spacing_m", survey_spacing_->value()},
        {"revisit_sim_s", survey_revisit_->value()}}}});
    });
    button(layout, page, "Start both: cooperative survey", "mission_start_all");
    button(layout, page, "Pause both and hover", "hover_all");
    survey_state_ = new QLabel("Waiting for survey status...", page);
    survey_state_->setWordWrap(true); survey_state_->setMaximumWidth(310);
    layout->addWidget(survey_state_);
    ownership_table_ = new QTableWidget(0, 4, page);
    ownership_table_->setHorizontalHeaderLabels({"Fire", "Owner", "Age s", "Due in s"});
    ownership_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ownership_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ownership_table_->setMinimumHeight(130);
    layout->addWidget(ownership_table_);
    survey_notice_ = new QLabel("First complete camera detection claims a fire.\nRed/negative due times mean overdue. Manual goals, Hover and Land pause that drone.", page);
    survey_notice_->setWordWrap(true); survey_notice_->setMaximumWidth(310);
    layout->addWidget(survey_notice_);
    layout->addStretch(); scroll->setWidget(page); tabs->addTab(scroll, "Survey");
  }
  void configure(QJsonObject object)
  {
    if (!object.contains("uav")) { object["uav"] = QString::fromStdString(current_); }
    object["request_id"] = QUuid::createUuid().toString();
    std_msgs::msg::String msg;
    msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
    mission_config_->publish(msg);
  }
  void missionActionAck(std_msgs::msg::String::ConstSharedPtr msg)
  {
    auto ack = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data)).object();
    if (ack["uav"].toString().toStdString() == current_ && ack["action"].toString().startsWith("mission_")) {
      mission_notice_->setText((ack["success"].toBool() ? "" : "Rejected: ")+ack["message"].toString());
    }
  }
  void updateMission()
  {
    auto survey = missions_["survey"].toObject();
    if (!survey_fields_ready_ && !survey.isEmpty()) {
      auto bounds = survey["bounds"].toArray();
      for (int i = 0; i < 4; ++i) { survey_bounds_[i]->setValue(bounds[i].toDouble()); }
      survey_altitude_->setValue(survey["altitude_m"].toDouble());
      survey_spacing_->setValue(survey["spacing_m"].toDouble());
      survey_revisit_->setValue(survey["revisit_sim_s"].toDouble());
      survey_fields_ready_ = true;
    }
    QStringList lines;
    lines << (survey.isEmpty() ? "Shared survey disabled; using per-drone areas" : "Shared rectangle split along "+survey["split_axis"].toString());
    for (const auto & name : {QString("uav1"), QString("uav2")}) {
      auto coverage = survey["coverage"].toObject()[name].toObject();
      auto state = missions_["uavs"].toObject()[name].toObject();
      lines << QString("%1: %2 | coverage %3/%4").arg(name).arg(state["phase"].toString())
        .arg(coverage["visited_cells"].toInt()).arg(coverage["total_cells"].toInt());
      if (state["phase"].toString() == "waiting_reservation") { lines << state["reason"].toString(); }
    }
    survey_state_->setText(lines.join("\n"));
    auto checks = missions_["fire_checks"].toArray();
    ownership_table_->setRowCount(checks.size());
    for (int row = 0; row < checks.size(); ++row) {
      auto fire = checks[row].toObject();
      QStringList values{QString::number(fire["id"].toInt()), fire["owner"].toString(),
        QString::number(fire["age_sim_s"].toDouble(), 'f', 1), QString::number(fire["due_in_sim_s"].toDouble(), 'f', 1)};
      for (int col = 0; col < values.size(); ++col) {
        auto * item = new QTableWidgetItem(values[col]);
        if (fire["overdue"].toBool()) { item->setForeground(Qt::red); }
        ownership_table_->setItem(row, col, item);
      }
    }
    auto state = missions_["uavs"].toObject()[QString::fromStdString(current_)].toObject();
    auto route = state["route"].toArray();
    route_table_->setRowCount(route.size());
    for (int row = 0; row < route.size(); ++row) {
      auto p = route[row].toObject()["position"].toArray();
      route_table_->setItem(row, 0, new QTableWidgetItem(QString::number(row+1)));
      for (int col = 0; col < 3; ++col) {
        route_table_->setItem(row, col+1, new QTableWidgetItem(QString::number(p[col].toDouble(), 'f', 1)));
      }
    }
    auto area = missions_["areas"].toObject()[QString::fromStdString(current_)].toObject()["bounds"].toArray();
    QString area_text;
    for (const auto & v : area) { area_text += QString::number(v.toDouble(), 'f', 1)+" "; }
    mission_state_->setText(QString::fromStdString(current_)+": "+state["phase"].toString()+"\n"+
      state["reason"].toString()+QString("\nVisits: %1 | loops: %2 | next WP: %3\nArea: %4\nStale after %5 sim s")
      .arg(state["completed_visits"].toInt()).arg(state["cycles"].toInt())
      .arg(state["route_index"].toInt()+1).arg(area_text)
      .arg(missions_["revisit_sim_s"].toObject()[QString::fromStdString(current_)].toDouble()));
  }
  void button(QVBoxLayout * layout, QWidget * parent, const QString & text, const std::string & action)
  {
    auto * widget = new QPushButton(text, parent);
    layout->addWidget(widget);
    connect(widget, &QPushButton::clicked, this, [this, action]() {
      QJsonObject object;
      object["uav"] = QString::fromStdString(current_);
      object["action"] = QString::fromStdString(action);
      object["request_id"] = QUuid::createUuid().toString();
      if (action == "mission_start" || action == "mission_resume" || action == "mission_start_all") {
        edit_route_->setChecked(false);
        object["mode"] = mode_->currentData().toString();
      }
      std_msgs::msg::String msg;
      msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
      actions_->publish(msg);
    });
  }
  void select()
  {
    auto name = vehicle_->currentData().toString().toStdString();
    current_ = name;
    const QSignalBlocker edit_block(edit_route_);
    edit_route_->setChecked(false);
    const QSignalBlocker altitude_block(altitude_), enabled_block(enabled_);
    altitude_->setValue(states_[name].altitude);
    enabled_->setChecked(states_[name].enabled);
    auto mission = missions_["uavs"].toObject()[QString::fromStdString(name)].toObject();
    if (mission.contains("loop")) { loop_->setChecked(mission["loop"].toBool()); }
    if (mission.contains("dwell_sim_s")) { dwell_->setValue(mission["dwell_sim_s"].toDouble()); }
    settings();
    updateImages();
    updateFire();
    updateMission();
  }
  void settings()
  {
    FleetSelection state{current_, altitude_->value(), enabled_->isChecked(), edit_route_ && edit_route_->isChecked()};
    states_[current_] = state;
    fire_observation_demo::fleet_selection() = state;
    QJsonObject object;
    object["uav"] = QString::fromStdString(current_);
    object["altitude_m"] = state.altitude;
    object["enabled"] = state.enabled;
    std_msgs::msg::String msg;
    msg.data = QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
    settings_->publish(msg);
  }
  void prepareImages()
  {
    auto * root = frame_->getManager()->getRootDisplayGroup();
    for (int i = 0; i < root->numDisplays(); ++i) {
      auto * display = root->getDisplayAt(i);
      if (display->getClassId() != "fire_observation_demo/FleetImage") { continue; }
      bool rgb = display->getName().startsWith("RGB");
      std::string name = display->getName().contains("uav2") ? "uav2" : "uav1";
      // Keep each Image display subscribed to one fixed topic. Reconnecting an
      // active Jazzy Image display can throw in message_filters::removeCallback.
      // Selection only changes dock visibility; no subscriptions are destroyed.
      display->setEnabled(true);
      if (auto * dock = display->getAssociatedWidgetPanel()) {
        dock->overrideVisibility(false);
        dock->setVisible(true);
        dock->setWindowTitle((rgb ? "RGB | " : "Thermal | ")+QString::fromStdString(name));
        image_docks_[name][rgb ? 0 : 1] = dock;
      }
    }
    for (size_t camera = 0; camera < 2; ++camera) {
      auto * first = image_docks_["uav1"][camera];
      auto * second = image_docks_["uav2"][camera];
      if (first && second) { frame_->tabifyDockWidget(first, second); }
    }
    updateImages();
  }
  void updateImages()
  {
    for (const auto & entry : image_docks_) {
      for (auto * dock : entry.second) {
        if (!dock) { continue; }
        dock->setVisible(entry.first == current_);
        if (entry.first == current_) { dock->raise(); }
      }
    }
    const auto & selected = image_docks_[current_];
    if (selected[0] && selected[1]) {
      frame_->resizeDocks({selected[0], selected[1]}, {300, 300}, Qt::Vertical);
    }
  }
  void updateFire()
  {
    auto object = fires_[current_];
    fire_->setText(QString::fromStdString(current_)+QString(" camera: %1 complete, %2 partial\n")
      .arg(object["complete_count"].toInt()).arg(object["partial_count"].toInt())+object["note"].toString());
  }
  rviz_common::VisualizationFrame * frame_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr settings_, actions_, mission_config_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  QComboBox * vehicle_;
  QDoubleSpinBox * altitude_;
  QCheckBox * enabled_;
  QLabel * command_;
  QLabel * fleet_;
  QLabel * fire_;
  QTableWidget * map_table_;
  QTableWidget * route_table_;
  QCheckBox * edit_route_ = nullptr;
  QCheckBox * loop_;
  QDoubleSpinBox * dwell_;
  QComboBox * mode_;
  QLineEdit * assigned_;
  QLabel * mission_state_;
  QLabel * mission_notice_;
  std::array<QDoubleSpinBox *, 4> survey_bounds_;
  QComboBox * survey_axis_;
  QDoubleSpinBox * survey_altitude_;
  QDoubleSpinBox * survey_spacing_;
  QDoubleSpinBox * survey_revisit_;
  QLabel * survey_state_;
  QLabel * survey_notice_;
  QTableWidget * ownership_table_;
  bool survey_fields_ready_ = false;
  QJsonObject missions_;
  std::string current_ = "uav1";
  std::map<std::string, FleetSelection> states_;
  std::map<std::string, QJsonObject> fires_;
  std::map<std::string, std::array<rviz_common::PanelDockWidget *, 2>> image_docks_;
};

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
  QTimer::singleShot(1500, [&app]() {
    for (auto * widget : app.topLevelWidgets()) {
      if (auto * frame = qobject_cast<rviz_common::VisualizationFrame *>(widget)) {
        auto * controls = new FleetControls(frame);
        frame->addDockWidget(Qt::LeftDockWidgetArea, controls);
        if (auto * displays = frame->findChild<QDockWidget *>("Displays")) {
          frame->splitDockWidget(displays, controls, Qt::Vertical);
        }
        controls->show();
      }
    }
  });
  QTimer shutdown;
  QObject::connect(&shutdown, &QTimer::timeout, &app, [&app]() { if (!rclcpp::ok()) { app.quit(); } });
  shutdown.start(100);
  QObject::connect(&app, &QApplication::aboutToQuit, []() { if (rclcpp::ok()) { rclcpp::shutdown(); } });
  return app.exec();
}
