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

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

#include <urdf/model.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace {

// Parse optional double parameter, fall back to default on missing/invalid
double parse_double_param(const hardware_interface::HardwareInfo& info,
                          const std::string& name, double default_value,
                          double min_value, double max_value) {
  auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) return default_value;
  try {
    double v = std::stod(it->second);
    if (v < min_value || v > max_value) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Parameter %s = %f out of range [%f, %f], using default %f",
                  name.c_str(), v, min_value, max_value, default_value);
      return default_value;
    }
    return v;
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Parameter %s invalid ('%s'), using default %f", name.c_str(),
                it->second.c_str(), default_value);
    return default_value;
  }
}

// Parse optional bool parameter, fall back to default
bool parse_bool_param(const hardware_interface::HardwareInfo& info,
                      const std::string& name, bool default_value) {
  auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) return default_value;
  std::string value = it->second;
  std::transform(value.begin(), value.end(), value.begin(), ::tolower);
  return (value == "true");
}

}  // namespace

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW()
    : gravity_compensation_enabled_(false), nv_offset_(0) {}

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  hand_ = parse_bool_param(info, "hand", true);

  // Parse CAN-FD enable (default: true for V10)
  can_fd_ = parse_bool_param(info, "can_fd", true);

  // Parse control gains (official kp1..kp7 / kd1..kd7 / kp_hand / kd_hand
  // format; optional overrides, defaults are the member initializers)
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    kp_[i - 1] = parse_double_param(info, "kp" + std::to_string(i), kp_[i - 1],
                                    0.0, 500.0);  // MIT Kp range
    kd_[i - 1] = parse_double_param(info, "kd" + std::to_string(i), kd_[i - 1],
                                    0.0, 5.0);  // MIT Kd range
  }
  if (hand_) {
    gripper_kp_ = parse_double_param(info, "kp_hand", gripper_kp_, 0.0, 500.0);
    gripper_kd_ = parse_double_param(info, "kd_hand", gripper_kd_, 0.0, 5.0);
  }

  // Parse teaching (drag) mode parameters
  teaching_mode_ = parse_bool_param(info, "teaching_mode", false);
  teaching_gain_scale_ =
      parse_double_param(info, "teaching_gain_scale", 0.1, 0.0, 1.0);

  // Calibration file (per-joint gravity scale + friction feedforward).
  // Loaded during on_configure(); empty → uncalibrated defaults.
  it = info.hardware_parameters.find("calib_file");
  calib_file_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse robot description path for gravity compensation (optional)
  it = info.hardware_parameters.find("robot_description_path");
  urdf_path_ = (it != info.hardware_parameters.end()) ? it->second : "";

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, "
              "teaching_mode=%s, teaching_gain_scale=%.2f, "
              "gravity_compensation=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              teaching_mode_ ? "enabled" : "disabled", teaching_gain_scale_,
              urdf_path_.empty() ? "disabled (no urdf)" : "pending");
  return true;
}

void OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Position limits are loaded in on_configure() from the same URDF that
  // Pinocchio uses (so they always match what RViz shows); default to motor
  // PMAX until then.
  pos_lower_.resize(joint_names_.size(), -12.5);
  pos_upper_.resize(joint_names_.size(), 12.5);

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Clear a latched estop on re-configuration (recovery path)
  estop_triggered_ = false;

  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Initialize Pinocchio model for gravity compensation
  if (!init_pinocchio_model()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Gravity compensation disabled — URDF not available");
  }

  // Load position limits from the same URDF (matches RViz display)
  load_joint_limits();

  // Load per-joint calibration (gravity scale + friction feedforward).
  // Uncalibrated defaults (grav_k=1, no friction) reproduce current behaviour.
  friction_comp_enabled_ = false;
  if (!calib_file_.empty()) {
    CalibParams loaded;
    if (parse_calib_file(calib_file_, loaded)) {
      calib_ = loaded;
      friction_comp_enabled_ = (calib_.fric_scale > 0.0);
      friction_torques_.resize(ARM_DOF, 0.0);
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "Calibration loaded: %s (friction FF %s)", calib_file_.c_str(),
                  friction_comp_enabled_ ? "enabled" : "disabled");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Failed to parse calibration file %s — using defaults",
                  calib_file_.c_str());
    }
  }

  return CallbackReturn::SUCCESS;
}

bool OpenArm_v10HW::load_joint_limits() {
  std::string urdf_path = urdf_path_;
  if (urdf_path.empty()) {
    try {
      std::string share_dir =
          ament_index_cpp::get_package_share_directory("openarm_hardware");
      urdf_path = share_dir + "/openarm.urdf";
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Cannot find openarm_hardware share directory for limits: %s",
                  e.what());
      return false;
    }
  }

  urdf::Model model;
  if (!model.initFile(urdf_path)) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "Failed to parse URDF for joint limits: %s", urdf_path.c_str());
    return false;
  }

  bool any_found = false;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto joint = model.getJoint(joint_names_[i]);
    if (joint && joint->type == urdf::Joint::REVOLUTE && joint->limits) {
      pos_lower_[i] = joint->limits->lower;
      pos_upper_[i] = joint->limits->upper;
      any_found = true;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Joint %s limits: [%.3f, %.3f] rad", joint_names_[i].c_str(),
                pos_lower_[i], pos_upper_[i]);
  }
  return any_found;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Return to zero unless teaching mode: in teaching the arm must keep its
  // current pose so it can be dragged from wherever it is.
  if (!teaching_mode_) {
    return_to_zero();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 activated (teaching_mode=%s, gain_scale=%.2f)",
              teaching_mode_ ? "true" : "false", teaching_gain_scale_);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");

  // Disable all motors; repeat to make sure the commands reach the bus
  // (official behavior)
  for (int i = 0; i < 3; ++i) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Latched emergency stop: once triggered, never send frames again
  // (motors stay disabled until re-activation).
  if (estop_triggered_) {
    return hardware_interface::return_type::ERROR;
  }

  // Runtime mode switch via the (otherwise unused) gripper effort command
  // interface, written by a forward_command_controller from the demo
  // front-end:  >0.5 → teaching (drag), <-0.5 → emergency disable,
  //              otherwise → startup parameter teaching_mode_.
  teaching_now_ = teaching_mode_;
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const double sw = tau_commands_[ARM_DOF];
    if (sw > 0.5) {
      teaching_now_ = true;
    } else if (sw < -0.5) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "ESTOP via gripper-effort switch — disabling motors");
      estop_triggered_ = true;
      openarm_->disable_all();
      return hardware_interface::return_type::ERROR;
    }
  }

  // Compute gravity compensation torques if enabled
  if (gravity_compensation_enabled_) {
    compute_gravity_torques();
  }
  // Compute friction feedforward if calibrated (uses teaching_now_)
  if (friction_comp_enabled_) {
    compute_friction_ff();
  }

  // Teaching mode has no trajectory controller constraining motion: enforce
  // soft position limits here and disable the motors if violated (e.g. the
  // operator drags the arm into a limit).
  // A small deadband absorbs feedback quantization noise. Observed live:
  //   - right J4 at q=-0.000 tripped the lower limit 0.0 (zero pose == lower
  //     limit; a -1e-4 reading noise spike disabled the arm)
  //   - left J3 at q=1.573 tripped the upper limit 1.571 (0.1° overshoot)
  // 0.01 rad (0.57°) is well above the noise (~0.001) and still far from
  // any mechanical stop; the URDF limits themselves stay authoritative.
  if (teaching_now_) {
    static constexpr double kLimitDeadband = 0.01;
    for (size_t i = 0; i < ARM_DOF && i < pos_lower_.size(); ++i) {
      if (pos_states_[i] < pos_lower_[i] - kLimitDeadband ||
          pos_states_[i] > pos_upper_[i] + kLimitDeadband) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                     "TEACHING LIMIT VIOLATION %s: q=%.3f outside [%.3f, %.3f]"
                     " (deadband %.3f) — disabling motors",
                     joint_names_[i].c_str(), pos_states_[i], pos_lower_[i],
                     pos_upper_[i], kLimitDeadband);
        openarm_->disable_all();
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    double gain_scale = teaching_now_ ? teaching_gain_scale_ : 1.0;
    // In teaching mode, track the measured position so the position loop
    // never fights the operator; the velocity command is zeroed likewise.
    double q_des = teaching_now_ ? pos_states_[i] : pos_commands_[i];
    double dq_des = teaching_now_ ? 0.0 : vel_commands_[i];
    // τ_ff = scaled gravity + friction feedforward + external effort
    // injection (calibration ramps / future force control; 0 in production).
    // Clamped to ±0.5·tMax so no single feedforward term can overpower the
    // motor.
    double tau_ff = 0.0;
    if (gravity_compensation_enabled_) {
      tau_ff += calib_.joints[i].grav_k * gravity_torques_[i];
    }
    if (friction_comp_enabled_) {
      tau_ff += friction_torques_[i];
    }
    tau_ff += tau_commands_[i];
    tau_ff = std::clamp(tau_ff, -0.5 * kTMax_[i], 0.5 * kTMax_[i]);
    arm_params.push_back({kp_[i] * gain_scale, kd_[i] * gain_scale, q_des,
                          dq_des, tau_ff});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, motor_command, 0, 0}});
  }
  openarm_->recv_all(100);
  return hardware_interface::return_type::OK;
}

bool OpenArm_v10HW::init_pinocchio_model() {
  std::string urdf_path = urdf_path_;

  // Fallback: find URDF via ament_index if not explicitly configured
  if (urdf_path.empty()) {
    try {
      std::string share_dir =
          ament_index_cpp::get_package_share_directory("openarm_hardware");
      urdf_path = share_dir + "/openarm.urdf";
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "Cannot find openarm_hardware share directory: %s", e.what());
      return false;
    }
  }

  // Check if URDF file exists
  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.good()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                "URDF file not found: %s", urdf_path.c_str());
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Loading URDF for gravity compensation: %s", urdf_path.c_str());

  try {
    pinocchio::urdf::buildModel(urdf_path, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    gravity_torques_.resize(ARM_DOF, 0.0);

    // Determine gravity index offset from arm prefix
    // Bimanual URDF layout: left arm at nv 0..8, right arm at nv 9..17
    if (arm_prefix_ == "right_") {
      nv_offset_ = 9;
    } else {
      nv_offset_ = 0;  // left_ or empty (single-arm fallback)
    }

    // Validate: bimanual URDF expects 18 DOF (or 16 without fingers)
    int expected_dof = nv_offset_ + static_cast<int>(ARM_DOF);
    if (pinocchio_model_.nv < expected_dof) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "URDF has %d DOF, need at least %d for arm_prefix='%s'. "
                  "Trying single-arm mode.",
                  pinocchio_model_.nv, expected_dof, arm_prefix_.c_str());
      nv_offset_ = 0;  // fallback to single-arm URDF
    }

    gravity_compensation_enabled_ = true;

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Pinocchio model loaded: %d DOF, nv_offset=%d, "
                "gravity compensation ENABLED",
                pinocchio_model_.nv, nv_offset_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to build Pinocchio model: %s", e.what());
    gravity_compensation_enabled_ = false;
    return false;
  }

  return true;
}

void OpenArm_v10HW::compute_gravity_torques() {
  // Build q vector using the full model DOF (e.g. 18 for bimanual, 9 for single-arm)
  const int nv = pinocchio_model_.nv;
  Eigen::VectorXd q(nv);
  q.setZero();

  // Fill arm joint positions at the correct offset for this arm
  for (size_t i = 0; i < ARM_DOF; ++i) {
    q[nv_offset_ + i] = pos_states_[i];
  }

  // Fill finger joints if present (finger joints follow arm joints in the model)
  if (hand_ && joint_names_.size() > ARM_DOF) {
    double finger1_rad = pos_states_[ARM_DOF] *
        (GRIPPER_MOTOR_1_RADIANS / GRIPPER_JOINT_0_POSITION);
    int finger1_idx = nv_offset_ + static_cast<int>(ARM_DOF);
    if (finger1_idx < nv) {
      q[finger1_idx] = finger1_rad;
    }
    int finger2_idx = finger1_idx + 1;
    if (finger2_idx < nv) {
      q[finger2_idx] = -finger1_rad;  // mimic joint
    }
  }

  // Compute gravity torques (full model)
  pinocchio::computeGeneralizedGravity(pinocchio_model_, pinocchio_data_, q);

  // Extract this arm's gravity torques.
  // NOTE (2026-08-03, live experiment): the Damiao MIT-frame q/dq/tau fields
  // are ALL in output-side units (motor TMAX values are output-side Nm, and a
  // +g feedforward of 6.93 Nm held J2 at 45° with only -0.2° drift in 5 s,
  // while g/9 drifted -41°). Dividing by GEAR_RATIOS weakened the gravity
  // compensation 9-40x — that was the root cause of the post-trajectory sag
  // ("回掉"). Feedforward must NOT be divided.
  for (size_t i = 0; i < ARM_DOF; ++i) {
    gravity_torques_[i] = pinocchio_data_.g[nv_offset_ + i];
  }
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position (interpolated)...");

  // Send an immediate zero-position command and read the actual starting
  // position, then ramp linearly so the arm moves smoothly from wherever it
  // currently is (official behavior: 200 steps x 10 ms, ~2 s total).
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  std::vector<double> start(ARM_DOF, 0.0);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    start[i] = arm_motors[i].get_position();
  }

  constexpr int kNumSteps = 200;
  constexpr int kStepMs = 10;
  for (int step = 1; step <= kNumSteps; ++step) {
    double t = static_cast<double>(step) / kNumSteps;
    for (size_t i = 0; i < ARM_DOF; ++i) {
      double target = start[i] + t * (0.0 - start[i]);
      arm_params[i] = {kp_[i], kd_[i], target, 0.0, 0.0};
    }
    openarm_->get_arm().mit_control_all(arm_params);
    std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));
    openarm_->recv_all();
  }

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    openarm_->recv_all();
  }
}

void OpenArm_v10HW::compute_friction_ff() {
  // Coulomb + viscous feedforward, direction-asymmetric:
  //   τ_ff = η · (τ_c± · tanh(dq/ε) + B± · dq)
  // η < 1 guarantees the feedforward never exceeds the actual friction
  // (tanh ≤ 1 ⇒ η·τ_c < τ_c), so no self-excited motion can build up.
  // Teaching mode: dead zone below teach_deadzone (operator must overcome
  // the full static threshold — physical limit) with a linear blend out of
  // the zone; hover (dq=0) is completely unaffected.
  for (size_t i = 0; i < ARM_DOF; ++i) {
    const double dq = vel_states_[i];
    const auto& p = calib_.joints[i];
    if (teaching_now_ && std::abs(dq) < calib_.teach_deadzone) {
      friction_torques_[i] = 0.0;
      continue;
    }
    const double tc = (dq >= 0) ? p.tau_c_plus : p.tau_c_minus;
    const double b = (dq >= 0) ? p.b_plus : p.b_minus;
    double ff = tc * std::tanh(dq / p.eps) + b * dq;
    double scale = teaching_now_ ? calib_.teach_scale : calib_.fric_scale;
    if (teaching_now_) {
      // linear blend from dead zone edge to full value over [dz, 2·dz]
      const double a = std::abs(dq);
      if (a < 2.0 * calib_.teach_deadzone) {
        ff *= (a - calib_.teach_deadzone) / calib_.teach_deadzone;
      }
    }
    friction_torques_[i] = scale * ff;
  }
}

bool OpenArm_v10HW::parse_calib_file(const std::string& path,
                                     CalibParams& out) {
  std::ifstream file(path);
  if (!file.good()) {
    return false;
  }
  std::string line;
  bool ok = true;
  while (std::getline(file, line)) {
    // strip comments and trim
    auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    std::istringstream iss(line);
    std::string key;
    if (!(iss >> key) || key.empty()) continue;
    std::vector<double> vals;
    double v;
    while (iss >> v) vals.push_back(v);

    if (key == "grav_k" && vals.size() == ARM_DOF) {
      for (size_t i = 0; i < ARM_DOF; ++i) out.joints[i].grav_k = vals[i];
    } else if (key == "tau_c_plus" && vals.size() == ARM_DOF) {
      for (size_t i = 0; i < ARM_DOF; ++i) out.joints[i].tau_c_plus = vals[i];
    } else if (key == "tau_c_minus" && vals.size() == ARM_DOF) {
      for (size_t i = 0; i < ARM_DOF; ++i) out.joints[i].tau_c_minus = vals[i];
    } else if (key == "b_plus" && vals.size() == ARM_DOF) {
      for (size_t i = 0; i < ARM_DOF; ++i) out.joints[i].b_plus = vals[i];
    } else if (key == "b_minus" && vals.size() == ARM_DOF) {
      for (size_t i = 0; i < ARM_DOF; ++i) out.joints[i].b_minus = vals[i];
    } else if (key == "eps" && vals.size() == ARM_DOF) {
      for (size_t i = 0; i < ARM_DOF; ++i) out.joints[i].eps = vals[i];
    } else if (key == "fric_scale" && vals.size() == 1) {
      out.fric_scale = vals[0];
    } else if (key == "teach_scale" && vals.size() == 1) {
      out.teach_scale = vals[0];
    } else if (key == "teach_deadzone" && vals.size() == 1) {
      out.teach_deadzone = vals[0];
    } else if (!key.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                  "calib file: ignoring unknown/malformed line: %s",
                  line.c_str());
      ok = false;
    }
  }
  return ok;
}

// Gripper mapping helper functions
double OpenArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
