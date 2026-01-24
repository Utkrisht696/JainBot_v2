#include "jainbot/jainbot_hw.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <cmath>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <poll.h>
#include <thread>        // std::thread
#include <atomic>        // std::atomic
#include <mutex>         // std::mutex


namespace fs = std::filesystem;
using std::chrono::milliseconds;

namespace jainbot {

static inline int clamp_int(int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }
static inline double clamp01(double x){ return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
static constexpr double TWO_PI = 6.2831853071795864769;

bool JainbotSystem::exists_(const std::string &p) {
  std::error_code ec; return fs::exists(p, ec);
}

bool JainbotSystem::write_text_(const std::string &path, const std::string &text) {
  std::ofstream f(path);
  if (!f.good()) return false;
  f << text;
  return f.good();
}

bool JainbotSystem::pwm_set_(const std::string &path, int duty_ns, bool enable) {
  // Order matters: duty, then enable
  if (!write_text_(path + "/duty_cycle", std::to_string(duty_ns) + "\n")) return false;
  if (!write_text_(path + "/enable", enable ? "1\n" : "0\n")) return false;
  return true;
}

bool JainbotSystem::pwm_export_and_configure_(int channel, std::string &out_path) {
  const std::string chip = "/sys/class/pwm/" + pwmchip_name_;
  const std::string export_path = chip + "/export";
  const std::string pwm_path = chip + "/pwm" + std::to_string(channel);

  if (!exists_(pwm_path)) {
    if (!write_text_(export_path, std::to_string(channel) + "\n")) {
      RCLCPP_ERROR(rclcpp::get_logger("jainbot"),
                   "PWM export failed for %s ch%d", pwmchip_name_.c_str(), channel);
      return false;
    }
    // Wait for sysfs to create the node
    for (int i=0;i<50 && !exists_(pwm_path);++i) std::this_thread::sleep_for(milliseconds(10));
  }

  // Set period, safe duty=0, enable=0
  if (!write_text_(pwm_path + "/period", std::to_string(pwm_period_ns_) + "\n")) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"),
                 "PWM set period failed at %s", (pwm_path + "/period").c_str());
    return false;
  }
  if (!write_text_(pwm_path + "/duty_cycle", "0\n")) return false;
  if (!write_text_(pwm_path + "/enable", "0\n")) return false;

  out_path = pwm_path;
  return true;
}

hardware_interface::CallbackReturn
JainbotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Expected exactly 2 joints, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_joint_  = info.joints[0].name;
  right_joint_ = info.joints[1].name;

  // Pull params from <hardware>
  auto p = [&info](const std::string &name, std::string &dst){
    auto it = info.hardware_parameters.find(name);
    if (it != info.hardware_parameters.end()) dst = it->second;
  };
  auto pi = [&info](const std::string &name, int &dst){
    auto it = info.hardware_parameters.find(name);
    if (it != info.hardware_parameters.end()) dst = std::stoi(it->second);
  };
  auto pd = [&info](const std::string &name, double &dst){
    auto it = info.hardware_parameters.find(name);
    if (it != info.hardware_parameters.end()) dst = std::stod(it->second);
  };
  auto pb = [&info](const std::string &name, bool &dst){
    auto it = info.hardware_parameters.find(name);
    if (it != info.hardware_parameters.end()) {
      const auto &v = it->second;
      dst = (v=="1"||v=="true"||v=="True"||v=="TRUE");
    }
  };

  // Motor GPIO/PWM params
  p("gpiochip", gpiochip_name_);
  pi("m1_en", m1_en_);  pi("m2_en", m2_en_);
  pi("m1_dir", m1_dir_); pi("m2_dir", m2_dir_);
  p("pwmchip", pwmchip_name_);
  pi("pwm0_channel", pwm0_channel_); pi("pwm1_channel", pwm1_channel_);
  pi("pwm_period_ns", pwm_period_ns_);
  pd("max_wheel_speed_radps", max_wheel_speed_radps_);
  pb("invert_left_dir", invert_left_);
  pb("invert_right_dir", invert_right_);

  // Encoders
  p("enc_gpiochip", enc_gpiochip_name_);
  pi("left_enc_a", l_a_);  pi("left_enc_b", l_b_);
  pi("right_enc_a", r_a_); pi("right_enc_b", r_b_);
  pd("cpr_wheel", cpr_wheel_);
  pb("invert_left_enc", invert_left_enc_);
  pb("invert_right_enc", invert_right_enc_);

  // PID & shaping
  pd("pid_kp", pid_kp_);
  pd("pid_ki", pid_ki_);
  pd("pid_kd", pid_kd_);
  pd("pid_i_max", pid_i_max_);
  pd("duty_ff_scale", duty_ff_scale_);
  pd("duty_min", duty_min_);
  pd("slew_duty_per_s", slew_duty_per_s_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JainbotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(hardware_interface::StateInterface(left_joint_,  hardware_interface::HW_IF_POSITION, &pos_[0]));
  si.emplace_back(hardware_interface::StateInterface(left_joint_,  hardware_interface::HW_IF_VELOCITY, &vel_[0]));
  si.emplace_back(hardware_interface::StateInterface(right_joint_, hardware_interface::HW_IF_POSITION, &pos_[1]));
  si.emplace_back(hardware_interface::StateInterface(right_joint_, hardware_interface::HW_IF_VELOCITY, &vel_[1]));
  return si;
}

std::vector<hardware_interface::CommandInterface>
JainbotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(hardware_interface::CommandInterface(left_joint_,  hardware_interface::HW_IF_VELOCITY, &cmd_[0]));
  ci.emplace_back(hardware_interface::CommandInterface(right_joint_, hardware_interface::HW_IF_VELOCITY, &cmd_[1]));
  return ci;
}

hardware_interface::CallbackReturn
JainbotSystem::on_configure(const rclcpp_lifecycle::State &)
{
  pos_ = {0.0, 0.0};
  vel_ = {0.0, 0.0};
  cmd_ = {0.0, 0.0};
  count_ = {0,0};

  // Motor GPIO
  chip_ = gpiod_chip_open_by_name(gpiochip_name_.c_str());
  if (!chip_) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to open %s", gpiochip_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  line_en1_  = gpiod_chip_get_line(chip_, m1_en_);
  line_en2_  = gpiod_chip_get_line(chip_, m2_en_);
  line_dir1_ = gpiod_chip_get_line(chip_, m1_dir_);
  line_dir2_ = gpiod_chip_get_line(chip_, m2_dir_);
  if (!line_en1_ || !line_en2_ || !line_dir1_ || !line_dir2_) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to get GPIO lines (EN/DIR)");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (gpiod_line_request_output(line_en1_,  "jainbot", 0) ||
      gpiod_line_request_output(line_en2_,  "jainbot", 0) ||
      gpiod_line_request_output(line_dir1_, "jainbot", 0) ||
      gpiod_line_request_output(line_dir2_, "jainbot", 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to request GPIO outputs");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // PWM
  if (!pwm_export_and_configure_(pwm0_channel_, pwm0_path_)) return hardware_interface::CallbackReturn::ERROR;
  if (!pwm_export_and_configure_(pwm1_channel_, pwm1_path_)) return hardware_interface::CallbackReturn::ERROR;

  // Encoders (edge events)
  enc_chip_ = gpiod_chip_open_by_name(enc_gpiochip_name_.c_str());
  if (!enc_chip_) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to open %s for encoders", enc_gpiochip_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  l_a_line_ = gpiod_chip_get_line(enc_chip_, l_a_);
  l_b_line_ = gpiod_chip_get_line(enc_chip_, l_b_);
  r_a_line_ = gpiod_chip_get_line(enc_chip_, r_a_);
  r_b_line_ = gpiod_chip_get_line(enc_chip_, r_b_);
  if (!l_a_line_ || !l_b_line_ || !r_a_line_ || !r_b_line_) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to get encoder lines");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Prefer events with bias pull-ups if the function exists (libgpiod v1.6+).
  bool events_ok = false;
#if defined(GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP)
  // Try with pull-ups first
  if (gpiod_line_request_both_edges_events_flags(l_a_line_, "jainbot", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) == 0 &&
      gpiod_line_request_both_edges_events_flags(l_b_line_, "jainbot", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) == 0 &&
      gpiod_line_request_both_edges_events_flags(r_a_line_, "jainbot", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) == 0 &&
      gpiod_line_request_both_edges_events_flags(r_b_line_, "jainbot", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) == 0) {
    events_ok = true;
    RCLCPP_INFO(rclcpp::get_logger("jainbot"), "Encoder events requested with BIAS_PULL_UP.");
  } else {
    // Fall back to plain both-edges events
    gpiod_line_release(l_a_line_); gpiod_line_release(l_b_line_);
    gpiod_line_release(r_a_line_); gpiod_line_release(r_b_line_);
  }
#endif

  if (!events_ok) {
    if (gpiod_line_request_both_edges_events(l_a_line_, "jainbot") ||
        gpiod_line_request_both_edges_events(l_b_line_, "jainbot") ||
        gpiod_line_request_both_edges_events(r_a_line_, "jainbot") ||
        gpiod_line_request_both_edges_events(r_b_line_, "jainbot")) {
      RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to request encoder edge events");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("jainbot"), "Encoder events requested (no bias flag).");
  }

  // Initialize prev_state from current pin levels
  auto rd = [](gpiod_line* a, gpiod_line* b){
    int av = gpiod_line_get_value(a);
    int bv = gpiod_line_get_value(b);
    if (av < 0 || bv < 0) return 0;
    return (av<<1) | bv;
  };
  prev_state_[0] = rd(l_a_line_, l_b_line_);   // left
  prev_state_[1] = rd(r_a_line_, r_b_line_);   // right

  // Sanity: log initial levels
  RCLCPP_INFO(rclcpp::get_logger("jainbot"),
    "Enc init levels L:A=%d B=%d  R:A=%d B=%d",
    gpiod_line_get_value(l_a_line_), gpiod_line_get_value(l_b_line_),
    gpiod_line_get_value(r_a_line_), gpiod_line_get_value(r_b_line_));

  RCLCPP_INFO(rclcpp::get_logger("jainbot"),
              "Configured HW: gpio=%s, pwm=%s ch%d/ch%d @%d ns, enc_gpio=%s, CPR=%.1f",
              gpiochip_name_.c_str(), pwmchip_name_.c_str(), pwm0_channel_, pwm1_channel_,
              pwm_period_ns_, enc_gpiochip_name_.c_str(), cpr_wheel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn
JainbotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Enable motor drivers, enable PWMs (0 duty)
  gpiod_line_set_value(line_en1_, 1);
  gpiod_line_set_value(line_en2_, 1);
  pwm_set_(pwm0_path_, 0, true);
  pwm_set_(pwm1_path_, 0, true);

  // Give encoder inputs a moment to settle after power-up, then re-sync levels.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Reset PID state
  i_term_ = {0.0, 0.0};
  last_err_ = {0.0, 0.0};
  last_duty_ = {0.0, 0.0};

  // Reset counters & start encoder thread
  {
    std::scoped_lock lk(enc_mtx_);
    count_ = {0,0};
    auto rd = [](gpiod_line* a, gpiod_line* b){
      int av = gpiod_line_get_value(a);
      int bv = gpiod_line_get_value(b);
      if (av < 0 || bv < 0) return 0;
      return (av<<1) | bv;
    };
    prev_state_[0] = rd(l_a_line_, l_b_line_);   // left
    prev_state_[1] = rd(r_a_line_, r_b_line_);   // right
  }

  // Drain any stale edge events before starting the thread.
  gpiod_line_event ev;
  while (gpiod_line_event_read(l_a_line_, &ev) == 0) {}
  while (gpiod_line_event_read(l_b_line_, &ev) == 0) {}
  while (gpiod_line_event_read(r_a_line_, &ev) == 0) {}
  while (gpiod_line_event_read(r_b_line_, &ev) == 0) {}

  enc_run_ = true;
  enc_thread_ = std::thread(&JainbotSystem::encoder_thread_, this);

  RCLCPP_INFO(rclcpp::get_logger("jainbot"), "Activated: EN=1, PWM enabled, encoder thread started.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
JainbotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // stop encoder thread first
  enc_run_ = false;
  if (enc_thread_.joinable()) enc_thread_.join();

  pwm_set_(pwm0_path_, 0, false);
  pwm_set_(pwm1_path_, 0, false);
  gpiod_line_set_value(line_en1_, 0);
  gpiod_line_set_value(line_en2_, 0);
  RCLCPP_INFO(rclcpp::get_logger("jainbot"), "Deactivated: motors off, encoder thread stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void jainbot::JainbotSystem::encoder_thread_()
{
  auto step = [](int prev, int curr)->int {
    if ((prev==0 && curr==1) || (prev==1 && curr==3) ||
        (prev==3 && curr==2) || (prev==2 && curr==0)) return +1;
    if ((prev==1 && curr==0) || (prev==3 && curr==1) ||
        (prev==2 && curr==3) || (prev==0 && curr==2)) return -1;
    return 0;
  };

  const int fd_la = gpiod_line_event_get_fd(l_a_line_);
  const int fd_lb = gpiod_line_event_get_fd(l_b_line_);
  const int fd_ra = gpiod_line_event_get_fd(r_a_line_);
  const int fd_rb = gpiod_line_event_get_fd(r_b_line_);
  if (fd_la < 0 || fd_lb < 0 || fd_ra < 0 || fd_rb < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("jainbot"), "Failed to get encoder event FDs");
    return;
  }

  const short pevents = POLLIN | POLLPRI;
  struct pollfd pfds[4];
  pfds[0] = { fd_la, pevents, 0 };
  pfds[1] = { fd_lb, pevents, 0 };
  pfds[2] = { fd_ra, pevents, 0 };
  pfds[3] = { fd_rb, pevents, 0 };

  RCLCPP_INFO(rclcpp::get_logger("jainbot"),
              "Encoder FDs: LA=%d LB=%d RA=%d RB=%d",
              fd_la, fd_lb, fd_ra, fd_rb);

  gpiod_line_event ev;
  const int timeout_ms = 5;

  // watchdog to detect "no edges" and fall back
  long long last_progress_counts[2] = {0,0};
  auto last_progress_time = std::chrono::steady_clock::now();
  bool sampling_mode = false;

  auto handle_edge = [&](gpiod_line* a, gpiod_line* b, int idx){
    int n = 0;
    while (gpiod_line_event_read(a, &ev) == 0) {
      int av = gpiod_line_get_value(a); if (av < 0) av = 0;
      int bv = gpiod_line_get_value(b); if (bv < 0) bv = 0;
      int curr = (av<<1) | bv;
      std::scoped_lock lk(enc_mtx_);
      count_[idx] += step(prev_state_[idx], curr);
      prev_state_[idx] = curr;
      ++n;
    }
    return n;
  };

  while (enc_run_) {
    if (!sampling_mode) {
      // ----- EVENT MODE -----
      int ret = ::poll(pfds, 4, timeout_ms);
      if (ret < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }

      int handled = 0;
      if (ret > 0) {
        if (pfds[0].revents & (POLLIN|POLLPRI)) handled += handle_edge(l_a_line_, l_b_line_, 0);
        if (pfds[1].revents & (POLLIN|POLLPRI)) handled += handle_edge(l_b_line_, l_a_line_, 0);
        if (pfds[2].revents & (POLLIN|POLLPRI)) handled += handle_edge(r_a_line_, r_b_line_, 1);
        if (pfds[3].revents & (POLLIN|POLLPRI)) handled += handle_edge(r_b_line_, r_a_line_, 1);

        // Extra quick drain pass for cross-line ordering
        if (pfds[0].revents) handled += handle_edge(l_a_line_, l_b_line_, 0);
        if (pfds[1].revents) handled += handle_edge(l_b_line_, l_a_line_, 0);
        if (pfds[2].revents) handled += handle_edge(r_a_line_, r_b_line_, 1);
        if (pfds[3].revents) handled += handle_edge(r_b_line_, r_a_line_, 1);

        pfds[0].revents = pfds[1].revents = pfds[2].revents = pfds[3].revents = 0;
      }

      // progress check
      long long snapL, snapR;
      { std::scoped_lock lk(enc_mtx_); snapL = count_[0]; snapR = count_[1]; }
      bool progressed = (snapL != last_progress_counts[0]) || (snapR != last_progress_counts[1]);
      auto now = std::chrono::steady_clock::now();
      if (progressed) {
        last_progress_counts[0] = snapL; last_progress_counts[1] = snapR;
        last_progress_time = now;
      } else {
        // no progress for > 300 ms -> fall back to sampling
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_progress_time).count() > 300) {
          sampling_mode = true;
          RCLCPP_WARN(rclcpp::get_logger("jainbot"),
            "No encoder events detected for 300 ms. Falling back to level-sampling mode (~2 kHz).");
        }
      }
    } else {
      // ----- LEVEL-SAMPLING MODE (robust, low CPU) -----
      // Sample both wheels at ~2 kHz and apply the same quadrature table
      int la = gpiod_line_get_value(l_a_line_); if (la < 0) la = 0;
      int lb = gpiod_line_get_value(l_b_line_); if (lb < 0) lb = 0;
      int ra = gpiod_line_get_value(r_a_line_); if (ra < 0) ra = 0;
      int rb = gpiod_line_get_value(r_b_line_); if (rb < 0) rb = 0;

      int currL = (la<<1) | lb;
      int currR = (ra<<1) | rb;

      {
        std::scoped_lock lk(enc_mtx_);
        count_[0] += step(prev_state_[0], currL);
        prev_state_[0] = currL;

        count_[1] += step(prev_state_[1], currR);
        prev_state_[1] = currR;
      }

      // ~2 kHz sampling
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
}



hardware_interface::return_type
JainbotSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // snapshot counters safely
  long long snap_count[2];
  {
    std::scoped_lock lk(enc_mtx_);
    snap_count[0] = count_[0];
    snap_count[1] = count_[1];
  }
  
  static rclcpp::Time last_log_time;
  auto now = rclcpp::Clock().now();
  if (!last_log_time.nanoseconds() || (now - last_log_time).seconds() > 1.0) {
    RCLCPP_INFO(rclcpp::get_logger("jainbot"),
            "ENC counts L=%lld R=%lld",
                (long long)snap_count[0], (long long)snap_count[1]);
      last_log_time = now;
  }


  static long long last_count[2] = {0,0};
  const double dt = std::max(1e-6, period.seconds());

  for (int i=0;i<2;++i) {
    long long dcounts = snap_count[i] - last_count[i];
    last_count[i] = snap_count[i];

    if ((i==0 && invert_left_enc_) || (i==1 && invert_right_enc_)) {
      dcounts = -dcounts;
    }

    const double dtheta = (static_cast<double>(dcounts) / cpr_wheel_) * TWO_PI;
    pos_[i] += dtheta;
    vel_[i]  = dtheta / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
JainbotSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = std::max(1e-6, period.seconds());
  const double max_w = (max_wheel_speed_radps_ > 1e-6) ? max_wheel_speed_radps_ : 1.0;

  auto per_wheel = [&](int idx, const std::string &pwm_path, gpiod_line *dir_line){
    const double w_cmd = cmd_[idx];
    const double w_meas = vel_[idx];  // from encoders in read()
    const bool fwd = (idx==0 ? (invert_left_ ? (w_cmd<0.0)  : (w_cmd>=0.0))
                             : (invert_right_? (w_cmd<0.0)  : (w_cmd>=0.0)));

    // speed error (use magnitude for control; direction handled by DIR)
    const double e = std::abs(w_cmd) - std::abs(w_meas);

    // PID terms
    const double kp = pid_kp_;
    const double ki = pid_ki_;
    const double kd = pid_kd_;
    const double de = (e - last_err_[idx]) / dt;
    last_err_[idx] = e;

    // feedforward: map command to duty
    double duty_ff = duty_ff_scale_ * (std::abs(w_cmd) / max_w);

    // PID output in duty units
    double duty_pid = kp*e + i_term_[idx] + kd*de;
    double duty = duty_ff + duty_pid;

    // clamp & anti-windup
    duty = std::clamp(duty, 0.0, 1.0);
    const bool at_hi = (duty >= 1.0 - 1e-6);
    const bool at_lo = (duty <= 0.0 + 1e-6);
    double i_new = i_term_[idx] + ki * e * dt;
    if ((!at_hi && !at_lo) ||
        (at_hi && i_new < i_term_[idx]) ||
        (at_lo && i_new > i_term_[idx])) {
      i_term_[idx] = std::clamp(i_new, -pid_i_max_, pid_i_max_);
    }

    // minimum duty to overcome stiction
    if (duty > 1e-6) duty = std::max(duty, duty_min_);

    // duty slew-rate limit
    if (slew_duty_per_s_ > 1e-9) {
      const double max_delta = slew_duty_per_s_ * dt;
      const double delta = std::clamp(duty - last_duty_[idx], -max_delta, max_delta);
      duty = last_duty_[idx] + delta;
    }
    last_duty_[idx] = duty;

    // Apply outputs
    gpiod_line_set_value(dir_line, fwd ? 0 : 1);
    const int duty_ns = static_cast<int>(duty * pwm_period_ns_);
    pwm_set_(pwm_path, clamp_int(duty_ns, 0, pwm_period_ns_-1), true);  // keep PWM enabled
  };

  // Left = idx0, Right = idx1
  per_wheel(0, pwm0_path_, line_dir1_);
  per_wheel(1, pwm1_path_, line_dir2_);

  return hardware_interface::return_type::OK;
}

} // namespace jainbot

PLUGINLIB_EXPORT_CLASS(jainbot::JainbotSystem, hardware_interface::SystemInterface)
