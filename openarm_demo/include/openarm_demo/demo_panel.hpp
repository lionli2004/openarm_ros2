// Copyright 2026 lionli
// Licensed under the Apache License, Version 2.0

#pragma once

#include <thread>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "openarm_demo/srv/replay.hpp"
#include "openarm_demo/srv/status.hpp"

namespace openarm_demo {

/// RViz panel front-end for the drag-teaching demo.
/// Talks to the openarm_demo_controller node via services.
///
/// NOTE: a panel-created rclcpp::Node is NOT spun by RViz — without a
/// dedicated executor thread, service responses and subscription callbacks
/// never run (requests still go out, which makes buttons "work" while the
/// status never updates). We therefore spin our own SingleThreadedExecutor.
class DemoPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  DemoPanel(QWidget* parent = nullptr);
  ~DemoPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void onSetTeaching(bool teaching);
  void onRecordStart();
  void onRecordStop();
  void onReplay();
  void onReplayStop();
  void onEstop();

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_mode_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr record_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr replay_stop_client_;
  rclcpp::Client<openarm_demo::srv::Replay>::SharedPtr replay_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  QLabel* status_label_;
  QComboBox* speed_combo_;
  QPushButton* teaching_btn_;
  QPushButton* normal_btn_;
  QPushButton* estop_btn_;
};

}  // namespace openarm_demo
