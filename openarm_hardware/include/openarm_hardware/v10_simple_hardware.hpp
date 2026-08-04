// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArm_v10HW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default motor configuration for V10
  const std::vector<openarm::damiao_motor::MotorType> DEFAULT_MOTOR_TYPES = {
      openarm::damiao_motor::MotorType::DM8009,  // Joint 1
      openarm::damiao_motor::MotorType::DM8009,  // Joint 2
      openarm::damiao_motor::MotorType::DM4340,  // Joint 3
      openarm::damiao_motor::MotorType::DM4340,  // Joint 4
      openarm::damiao_motor::MotorType::DM4310,  // Joint 5
      openarm::damiao_motor::MotorType::DM4310,  // Joint 6
      openarm::damiao_motor::MotorType::DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

  const openarm::damiao_motor::MotorType DEFAULT_GRIPPER_MOTOR_TYPE =
      openarm::damiao_motor::MotorType::DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

  // Gains (reference: OpenArm doc Kp=200 Nm/rad, Kd=5 Nms/rad,
  // scaled by motor torque capacity for smaller joints)
  // Damiao MIT mode: Kp range 0-500, Kd range 0-5
  // Overridable via hardware parameters kp1..kp7 / kd1..kd7 (official format)
  std::vector<double> kp_ = {200.0, 200.0, 100.0, 100.0,
                             40.0,  40.0,  40.0,  5.0};
  std::vector<double> kd_ = {5.0,  5.0,  2.5,  2.5,
                             1.0,  1.0,  1.0,  0.1};

  // NOTE: gear ratios (J1/J2 9:1, J3/J4 40:1, J5-J7 10:1) are intentionally
  // NOT applied in the control path anymore. Live experiment (2026-08-03):
  // Damiao MIT-frame q/dq/tau are all output-side units — a +g feedforward of
  // 6.93 Nm held J2 at 45° with -0.2° drift/5s, while g/9 drifted -41°. The
  // old /GEAR_RATIOS division weakened gravity compensation 9-40x and caused
  // the post-trajectory sag ("回掉").

  // Pinocchio dynamics model for gravity compensation
  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;
  std::vector<double> gravity_torques_;
  bool gravity_compensation_enabled_;
  int nv_offset_;  // index offset into data.g for this arm (0=left, 9=right)

  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_DEFAULT_KP = 5.0;
  const double GRIPPER_DEFAULT_KD = 0.1;
  // Overridable via hardware parameters kp_hand / kd_hand
  double gripper_kp_ = GRIPPER_DEFAULT_KP;
  double gripper_kd_ = GRIPPER_DEFAULT_KD;

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  std::string urdf_path_;
  bool hand_;
  bool can_fd_;

  // Teaching (drag) mode: low gain + q_des tracks feedback + gravity τ_ff.
  // Arm can be manually dragged and hovers where released.
  bool teaching_mode_ = false;
  // Scale factor applied to kp_/kd_ while teaching (default 0.1 → Kp 20/10/4)
  double teaching_gain_scale_ = 0.1;
  // Latched by the runtime estop switch; write() then returns ERROR and
  // never sends frames again (motors stay disabled).
  bool estop_triggered_ = false;
  // Effective teaching state for the current write() cycle (runtime switch
  // or startup parameter) — member so compute_friction_ff() can see it.
  bool teaching_now_ = false;

  // Per-joint calibration / compensation parameters (output-side units,
  // consistent with the MIT frame). Loaded from a text file via the
  // `calib_file` hardware parameter; defaults reproduce the uncalibrated
  // behaviour (grav_k=1, no friction feedforward).
  struct JointCalibParams {
    double grav_k = 1.0;        // gravity model scale
    double tau_c_plus = 0.0;    // Nm, Coulomb (dq >= 0)
    double tau_c_minus = 0.0;   // Nm, Coulomb (dq < 0)
    double b_plus = 0.0;        // Nm/(rad/s), viscous (dq >= 0)
    double b_minus = 0.0;       // Nm/(rad/s), viscous (dq < 0)
    double eps = 0.02;          // rad/s, tanh smoothing
  };
  struct CalibParams {
    double fric_scale = 0.8;    // eta, normal mode (under-compensation)
    double teach_scale = 0.6;   // eta, teaching mode (conservative)
    double teach_deadzone = 0.03;  // rad/s, friction FF zero zone in teaching
    std::array<JointCalibParams, ARM_DOF> joints{};
  };
  std::string calib_file_;
  CalibParams calib_;
  bool friction_comp_enabled_ = false;
  std::vector<double> friction_torques_;
  // Per-joint output-side torque caps for τ_ff clamping (matches
  // MOTOR_LIMIT_PARAMS tMax in openarm_can)
  const std::array<double, ARM_DOF> kTMax_ = {54.0, 54.0, 28.0, 28.0,
                                              10.0, 10.0, 10.0};
  // URDF position limits (per joint, already including bimanual offsets).
  // Used as soft limits in teaching mode: in teaching there is no trajectory
  // controller constraining motion, so the arm must not be dragged past them.
  std::vector<double> pos_lower_;
  std::vector<double> pos_upper_;

  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();
  bool init_pinocchio_model();
  void compute_gravity_torques();
  bool parse_calib_file(const std::string& path, CalibParams& out);
  void compute_friction_ff();
  bool load_joint_limits();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace openarm_hardware
