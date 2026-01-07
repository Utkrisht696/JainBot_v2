#pragma once
#include <array>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// libgpiod (C API)
#include <gpiod.h>

namespace jainbot {

class JainbotSystem : public hardware_interface::SystemInterface {
public:
  // init (reads URDF <ros2_control> info)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // expose state & command interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // lifecycle
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & prev_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & prev_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev_state) override;

  // periodic I/O
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Two wheel joints: index 0 = left, 1 = right
  std::array<double,2> pos_{0.0, 0.0};   // rad
  std::array<double,2> vel_{0.0, 0.0};   // rad/s
  std::array<double,2> cmd_{0.0, 0.0};   // desired rad/s

  std::string left_joint_  = "left_wheel_joint";
  std::string right_joint_ = "right_wheel_joint";

  // --- Encoder params ---
  std::string enc_gpiochip_name_ = "gpiochip4";
  int l_a_{16}, l_b_{26}, r_a_{17}, r_b_{27};
  double cpr_wheel_{1994.0}; // counts-per-rev @ wheel (after gearbox, x4)

  // Encoder handles
  gpiod_chip *enc_chip_{nullptr};
  gpiod_line *l_a_line_{nullptr}, *l_b_line_{nullptr};
  gpiod_line *r_a_line_{nullptr}, *r_b_line_{nullptr};

  // Quadrature state & counts
  std::array<int,2> prev_state_{0,0};     // per wheel: (A<<1)|B
  std::array<long long,2> count_{0,0};    // cumulative ticks

  // --- Motor params ---
  std::string gpiochip_name_ = "gpiochip4";
  int m1_en_{22}, m2_en_{23}, m1_dir_{24}, m2_dir_{25};
  std::string pwmchip_name_ = "pwmchip0";
  int pwm0_channel_{0}, pwm1_channel_{1};
  int pwm_period_ns_{50000}; // 20 kHz
  double max_wheel_speed_radps_{20.0};
  bool invert_left_{false}, invert_right_{false};

  // GPIO handles
  gpiod_chip *chip_{nullptr};
  gpiod_line *line_en1_{nullptr}, *line_en2_{nullptr}, *line_dir1_{nullptr}, *line_dir2_{nullptr};

  // PWM paths
  std::string pwm0_path_; // /sys/class/pwm/pwmchipX/pwm0
  std::string pwm1_path_;

  // --- PID & duty shaping ---
  double pid_kp_{0.3}, pid_ki_{0.0}, pid_kd_{0.0};
  double pid_i_max_{0.6};          // clamp on integrator (duty units)
  double duty_ff_scale_{1.0};      // feedforward: |cmd|/max_w -> duty*scale
  double duty_min_{0.05};          // minimum duty when non-zero
  double slew_duty_per_s_{2.0};    // duty rate limit (0=off)

  std::array<double,2> i_term_{0.0, 0.0};
  std::array<double,2> last_err_{0.0, 0.0};
  std::array<double,2> last_duty_{0.0, 0.0};

  // --- Encoder counting thread ---
  std::thread enc_thread_;
  std::atomic<bool> enc_run_{false};
  std::mutex enc_mtx_;  // protects prev_state_ and count_ during updates
  void encoder_thread_();  // worker that drains events continuously

  // helpers
  bool pwm_export_and_configure_(int channel, std::string &out_path);
  bool pwm_set_(const std::string &path, int duty_ns, bool enable);
  static bool write_text_(const std::string &path, const std::string &text);
  static bool exists_(const std::string &path);
};

} // namespace jainbot
