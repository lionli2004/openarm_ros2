// Copyright 2026 lionli
// Licensed under the Apache License, Version 2.0

#include "openarm_demo/demo_panel.hpp"

#include <memory>
#include <string>

namespace openarm_demo {

DemoPanel::DemoPanel(QWidget* parent) : rviz_common::Panel(parent) {}

DemoPanel::~DemoPanel() {
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void DemoPanel::onInitialize() {
  // One rclcpp node per panel instance (standard RViz panel pattern).
  node_ = std::make_shared<rclcpp::Node>("openarm_demo_panel");
  // RViz does not spin panel-created nodes: run our own executor thread so
  // service responses and the status subscription actually get processed.
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
  const std::string prefix = "/openarm_demo_controller";
  set_mode_client_ = node_->create_client<std_srvs::srv::SetBool>(
      prefix + "/set_mode");
  estop_client_ = node_->create_client<std_srvs::srv::Trigger>(
      prefix + "/estop");
  record_start_client_ = node_->create_client<std_srvs::srv::Trigger>(
      prefix + "/record_start");
  record_stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
      prefix + "/record_stop");
  replay_stop_client_ = node_->create_client<std_srvs::srv::Trigger>(
      prefix + "/replay_stop");
  replay_client_ = node_->create_client<openarm_demo::srv::Replay>(
      prefix + "/replay");
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      prefix + "/status_text", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        status_label_->setText(QString::fromStdString(msg->data));
      });

  auto* layout = new QVBoxLayout;

  auto* title = new QLabel("OpenArm Drag-Teaching Demo", this);
  title->setStyleSheet("font-weight: bold; font-size: 13pt;");
  layout->addWidget(title);

  auto* mode_row = new QHBoxLayout;
  teaching_btn_ = new QPushButton("拖动模式 (Teaching)", this);
  normal_btn_ = new QPushButton("回放模式 (Normal)", this);
  mode_row->addWidget(teaching_btn_);
  mode_row->addWidget(normal_btn_);
  layout->addLayout(mode_row);

  auto* rec_row = new QHBoxLayout;
  auto* rec_start_btn = new QPushButton("开始记录", this);
  auto* rec_stop_btn = new QPushButton("停止记录", this);
  rec_row->addWidget(rec_start_btn);
  rec_row->addWidget(rec_stop_btn);
  layout->addLayout(rec_row);

  auto* replay_row = new QHBoxLayout;
  auto* replay_btn = new QPushButton("▶ 回放", this);
  auto* replay_stop_btn = new QPushButton("■ 停止回放", this);
  speed_combo_ = new QComboBox(this);
  speed_combo_->addItem("0.5×", 0.5);
  speed_combo_->addItem("1×", 1.0);
  speed_combo_->addItem("2×", 2.0);
  replay_row->addWidget(replay_btn);
  replay_row->addWidget(replay_stop_btn);
  replay_row->addWidget(speed_combo_);
  layout->addLayout(replay_row);

  estop_btn_ = new QPushButton("紧急停止", this);
  estop_btn_->setStyleSheet(
      "background-color: #c0392b; color: white; font-weight: bold;"
      "font-size: 14pt; padding: 8px;");
  layout->addWidget(estop_btn_);

  status_label_ = new QLabel("demo controller not connected", this);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  setLayout(layout);

  connect(teaching_btn_, &QPushButton::clicked, this,
          [this]() { onSetTeaching(true); });
  connect(normal_btn_, &QPushButton::clicked, this,
          [this]() { onSetTeaching(false); });
  connect(rec_start_btn, &QPushButton::clicked, this,
          &DemoPanel::onRecordStart);
  connect(rec_stop_btn, &QPushButton::clicked, this,
          &DemoPanel::onRecordStop);
  connect(replay_btn, &QPushButton::clicked, this, &DemoPanel::onReplay);
  connect(replay_stop_btn, &QPushButton::clicked, this,
          &DemoPanel::onReplayStop);
  connect(estop_btn_, &QPushButton::clicked, this, &DemoPanel::onEstop);
}

void DemoPanel::onSetTeaching(bool teaching) {
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = teaching;
  set_mode_client_->async_send_request(req);
}

void DemoPanel::onRecordStart() {
  record_start_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
}

void DemoPanel::onRecordStop() {
  record_stop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
}

void DemoPanel::onReplay() {
  auto req = std::make_shared<openarm_demo::srv::Replay::Request>();
  req->path = "";  // last recorded trajectory
  req->speed = static_cast<float>(speed_combo_->currentData().toDouble());
  replay_client_->async_send_request(req);
}

void DemoPanel::onReplayStop() {
  replay_stop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
}

void DemoPanel::onEstop() {
  estop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
}

void DemoPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
}

void DemoPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
}

}  // namespace openarm_demo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(openarm_demo::DemoPanel, rviz_common::Panel)
