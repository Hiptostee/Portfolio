#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slambot_interfaces/msg/orchestration_state.hpp"
#include "slambot_interfaces/srv/navigate_to.hpp"
#include "slambot_interfaces/srv/save_map.hpp"
#include "slambot_interfaces/srv/set_mode.hpp"
#include "slambot_interfaces/srv/teleop_command.hpp"
#include "slambot_navigation/action/a_star.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using namespace std::chrono_literals;

struct LaunchProcess
{
  std::string mode;
  pid_t pid{-1};
};

class OrchestrationError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class SlambotOrchestrationNode : public rclcpp::Node
{
public:
  using NavigateTo = slambot_interfaces::srv::NavigateTo;
  using SaveMap = slambot_interfaces::srv::SaveMap;
  using SetMode = slambot_interfaces::srv::SetMode;
  using TeleopCommand = slambot_interfaces::srv::TeleopCommand;
  using Trigger = std_srvs::srv::Trigger;
  using OrchestrationState = slambot_interfaces::msg::OrchestrationState;
  using AStar = slambot_navigation::action::AStar;
  using GoalHandleAStar = rclcpp_action::ClientGoalHandle<AStar>;

  SlambotOrchestrationNode()
  : rclcpp::Node("slambot_orchestration")
  {
    sim_ = declare_parameter<bool>("sim", false);
    startup_mode_ = declare_parameter<std::string>("startup_mode", "mapping");
    bringup_package_ = declare_parameter<std::string>("bringup_package", "slambot_bringup");
    bringup_launch_ = declare_parameter<std::string>("bringup_launch", "slambot_bringup.launch.py");
    default_map_yaml_ = declare_parameter<std::string>("default_map_yaml", defaultMapYamlPath());
    map_save_prefix_ = declare_parameter<std::string>("map_save_prefix", defaultMapSavePrefix());
    a_star_action_name_ = declare_parameter<std::string>("a_star_action_name", "/a_star");
    stop_service_name_ = declare_parameter<std::string>("stop_service_name", "/lqr/stop");
    teleop_timeout_ms_ = declare_parameter<int>("teleop_timeout_ms", 250);
    navigation_state_stale_timeout_sec_ = declare_parameter<double>(
      "navigation_state_stale_timeout_sec", 0.3);
    launch_startup_delay_sec_ = declare_parameter<double>("launch_startup_delay_sec", 1.0);
    launch_stop_timeout_sec_ = declare_parameter<double>("launch_stop_timeout_sec", 10.0);

    const auto path_qos = rclcpp::QoS(1).transient_local().reliable();

    state_publisher_ = create_publisher<OrchestrationState>("/orchestration/state", path_qos);
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_resume_publisher_ = create_publisher<nav_msgs::msg::Path>("/path", path_qos);

    stop_client_ = create_client<Trigger>(stop_service_name_);
    a_star_client_ = rclcpp_action::create_client<AStar>(this, a_star_action_name_);

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/path", path_qos, std::bind(&SlambotOrchestrationNode::handlePath, this, std::placeholders::_1));
    navigating_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/is_navigating", 10, std::bind(&SlambotOrchestrationNode::handleNavigating, this, std::placeholders::_1));

    set_mode_service_ = create_service<SetMode>(
      "/orchestration/set_mode",
      std::bind(&SlambotOrchestrationNode::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));
    save_map_service_ = create_service<SaveMap>(
      "/orchestration/save_map",
      std::bind(&SlambotOrchestrationNode::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));
    navigate_to_service_ = create_service<NavigateTo>(
      "/orchestration/navigate_to",
      std::bind(&SlambotOrchestrationNode::handleNavigateTo, this, std::placeholders::_1, std::placeholders::_2));
    pause_navigation_service_ = create_service<Trigger>(
      "/orchestration/pause_navigation",
      std::bind(&SlambotOrchestrationNode::handlePauseNavigation, this, std::placeholders::_1, std::placeholders::_2));
    resume_navigation_service_ = create_service<Trigger>(
      "/orchestration/resume_navigation",
      std::bind(&SlambotOrchestrationNode::handleResumeNavigation, this, std::placeholders::_1, std::placeholders::_2));
    cancel_navigation_service_ = create_service<Trigger>(
      "/orchestration/cancel_navigation",
      std::bind(&SlambotOrchestrationNode::handleCancelNavigation, this, std::placeholders::_1, std::placeholders::_2));
    stop_motion_service_ = create_service<Trigger>(
      "/orchestration/stop_motion",
      std::bind(&SlambotOrchestrationNode::handleStopMotion, this, std::placeholders::_1, std::placeholders::_2));
    teleop_service_ = create_service<TeleopCommand>(
      "/orchestration/teleop",
      std::bind(&SlambotOrchestrationNode::handleTeleop, this, std::placeholders::_1, std::placeholders::_2));

    teleop_timer_ = create_wall_timer(50ms, std::bind(&SlambotOrchestrationNode::checkTeleopTimeout, this));
    navigation_state_timer_ = create_wall_timer(
      100ms, std::bind(&SlambotOrchestrationNode::checkNavigationStateTimeout, this));
    launch_process_timer_ = create_wall_timer(
      500ms, std::bind(&SlambotOrchestrationNode::checkLaunchProcess, this));

    if (startup_mode_ == "mapping" || startup_mode_ == "localization") {
      try {
        setMode(startup_mode_);
      } catch (const std::exception & exc) {
        setError(std::string("Failed to start in ") + startup_mode_ + ": " + exc.what());
      }
    } else {
      publishState();
    }
  }

  ~SlambotOrchestrationNode() override
  {
    try {
      stopLaunch();
    } catch (...) {
    }
  }

private:
  std::string defaultMapYamlPath() const
  {
    return sim_ ? "/home/slambot/ros2_ws/maps/my_map.yaml" :
      "/home/slambot/ros2_ws_pi/maps/my_map_real.yaml";
  }

  std::string defaultMapSavePrefix() const
  {
    return sim_ ? "/home/slambot/ros2_ws/maps/my_map" :
      "/home/slambot/ros2_ws_pi/maps/my_map_real";
  }

  void setMode(const std::string & mode)
  {
    const auto normalized_mode = normalizeMode(mode);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (normalized_mode == current_mode_ && launchRunningLocked()) {
        return;
      }
    }

    stopLaunch();
    startLaunch(normalized_mode);
  }

  std::string saveMap()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_mode_ != "mapping") {
        throw OrchestrationError("Map saving is only available in mapping mode.");
      }
    }

    std::filesystem::create_directories(std::filesystem::path(map_save_prefix_).parent_path());

    const auto exit_code = runCommand({
      "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_save_prefix_
    }, 60.0);

    if (exit_code != 0) {
      throw OrchestrationError("map_saver_cli failed.");
    }

    const auto yaml_path = map_save_prefix_ + ".yaml";
    if (!std::filesystem::exists(yaml_path)) {
      throw OrchestrationError("Map saver completed but yaml was not created.");
    }

    publishState();
    return yaml_path;
  }

  void stopMotion()
  {
    std::string current_mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_mode = current_mode_;
    }

    clearTeleop();
    publishZeroVelocity();
    if (current_mode == "localization") {
      callLqrStop();
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_navigating_ = false;
      is_paused_ = false;
      pause_requested_ = false;
      current_path_msg_.reset();
      last_planned_path_msg_.reset();
      paused_path_msg_.reset();
      last_navigation_goal_.reset();
    }
    publishState();
  }

  std::size_t sendNavigationGoal(double x_m, double y_m)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_mode_ != "localization") {
        throw OrchestrationError("Point goals are only available in localization mode.");
      }
    }

    if (!a_star_client_->wait_for_action_server(2s)) {
      throw OrchestrationError("A* action server is not available.");
    }

    AStar::Goal goal_msg;
    goal_msg.goal.header.frame_id = "map";
    goal_msg.goal.header.stamp = now();
    goal_msg.goal.pose.position.x = x_m;
    goal_msg.goal.pose.position.y = y_m;
    goal_msg.goal.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<AStar>::SendGoalOptions();
    auto goal_future = a_star_client_->async_send_goal(goal_msg, send_goal_options);
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      throw OrchestrationError("Timed out sending navigation goal.");
    }

    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      throw OrchestrationError("Path rejected.");
    }

    auto result_future = a_star_client_->async_get_result(goal_handle);
    if (result_future.wait_for(10s) != std::future_status::ready) {
      throw OrchestrationError("Timed out waiting for navigation result.");
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result->success) {
      throw OrchestrationError("Whoops! I cannot navigate there.");
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_navigation_goal_ = std::make_pair(x_m, y_m);
      last_planned_path_msg_ = wrapped_result.result->path;
      current_path_msg_ = wrapped_result.result->path;
      paused_path_msg_.reset();
      is_paused_ = false;
      is_navigating_ = true;
    }
    publishState();
    return wrapped_result.result->path.poses.size();
  }

  void pauseNavigation()
  {
    std::optional<nav_msgs::msg::Path> path_msg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_mode_ != "localization") {
        throw OrchestrationError("Pause is only available in localization mode.");
      }
      if (current_path_msg_) {
        path_msg = *current_path_msg_;
      } else if (last_planned_path_msg_) {
        path_msg = *last_planned_path_msg_;
      } else {
        throw OrchestrationError("There is no active path to pause.");
      }
      pause_requested_ = true;
    }
    publishState();

    try {
      callLqrStop();
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pause_requested_ = false;
      }
      publishState();
      throw;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pause_requested_ = false;
      paused_path_msg_ = *path_msg;
      is_paused_ = true;
      is_navigating_ = false;
    }
    publishState();
  }

  void resumeNavigation()
  {
    std::optional<nav_msgs::msg::Path> path_msg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_mode_ != "localization") {
        throw OrchestrationError("Resume is only available in localization mode.");
      }
      if (!paused_path_msg_) {
        throw OrchestrationError("There is no paused path to resume.");
      }
      path_msg = *paused_path_msg_;
    }

    path_resume_publisher_->publish(*path_msg);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_path_msg_ = *path_msg;
      last_planned_path_msg_ = *path_msg;
      is_paused_ = false;
      pause_requested_ = false;
      is_navigating_ = true;
    }
    publishState();
  }

  void cancelNavigation()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_mode_ != "localization") {
        throw OrchestrationError("Cancel is only available in localization mode.");
      }
    }

    callLqrStop();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_path_msg_.reset();
      last_planned_path_msg_.reset();
      paused_path_msg_.reset();
      last_navigation_goal_.reset();
      is_paused_ = false;
      pause_requested_ = false;
      is_navigating_ = false;
    }
    publishState();
  }

  void handleTeleopCommand(
    const std::string & client_id,
    bool deadman,
    double linear_x,
    double linear_y,
    double angular_z)
  {
    checkNavigationStateTimeout();

    std::string current_mode;
    bool is_navigating = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_mode = current_mode_;
      is_navigating = is_navigating_;
    }

    if (current_mode == "mapping") {
    } else if (current_mode == "localization") {
      if (is_navigating) {
        throw OrchestrationError("Teleop is disabled while LQR is actively navigating.");
      }
    } else {
      throw OrchestrationError("Teleop is only available in mapping or idle-localization control.");
    }

    if (!deadman) {
      bool should_clear = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        should_clear = teleop_owner_id_.has_value() && teleop_owner_id_.value() == client_id;
      }
      if (should_clear) {
        clearTeleop();
        publishZeroVelocity();
        publishState();
      }
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = linear_y;
    cmd.angular.z = angular_z;
    cmd_vel_publisher_->publish(cmd);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      teleop_owner_id_ = client_id;
      teleop_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(teleop_timeout_ms_);
    }
    publishState();
  }

  void startLaunch(const std::string & mode)
  {
    std::vector<std::string> args = {
      "ros2",
      "launch",
      bringup_package_,
      bringup_launch_,
      std::string("sim:=") + (sim_ ? "true" : "false"),
      std::string("localization_mode:=") + (mode == "localization" ? "true" : "false"),
      std::string("map_yaml:=") + default_map_yaml_
    };

    const auto pid = spawnProcess(args);
    std::this_thread::sleep_for(std::chrono::duration<double>(launch_startup_delay_sec_));

    int status = 0;
    const auto wait_result = ::waitpid(pid, &status, WNOHANG);
    if (wait_result == pid) {
      throw OrchestrationError(mode + " launch exited immediately.");
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_launch_ = LaunchProcess{mode, pid};
      current_mode_ = mode;
      is_paused_ = false;
      pause_requested_ = false;
      is_navigating_ = false;
      last_error_.clear();
      if (mode == "mapping") {
        current_path_msg_.reset();
        last_planned_path_msg_.reset();
        paused_path_msg_.reset();
        last_navigation_goal_.reset();
      }
    }

    RCLCPP_INFO(get_logger(), "Started %s launch with PID %d", mode.c_str(), static_cast<int>(pid));
    publishState();
  }

  void stopLaunch()
  {
    std::optional<LaunchProcess> launch;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_launch_) {
        launch = current_launch_;
      }
    }

    clearTeleop();
    publishZeroVelocity();

    if (launch) {
      terminateProcessGroup(launch->pid);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_launch_.reset();
      current_mode_ = "idle";
      is_navigating_ = false;
      is_paused_ = false;
      pause_requested_ = false;
      current_path_msg_.reset();
      last_planned_path_msg_.reset();
      paused_path_msg_.reset();
      last_navigation_goal_.reset();
    }
    publishState();
  }

  void callLqrStop()
  {
    if (!stop_client_->wait_for_service(2s)) {
      throw OrchestrationError("LQR stop service is not available.");
    }
    auto request = std::make_shared<Trigger::Request>();
    auto future = stop_client_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
      throw OrchestrationError("LQR stop timed out.");
    }
    const auto response = future.get();
    if (!response->success) {
      throw OrchestrationError(std::string("LQR stop failed: ") + response->message);
    }
  }

  void checkTeleopTimeout()
  {
    bool expired = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      expired = teleop_owner_id_.has_value() &&
        std::chrono::steady_clock::now() >= teleop_deadline_;
    }

    if (expired) {
      RCLCPP_WARN(get_logger(), "Teleop timeout exceeded; publishing zero velocity.");
      clearTeleop();
      publishZeroVelocity();
      publishState();
    }
  }

  void checkNavigationStateTimeout()
  {
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stale = is_navigating_ &&
        last_navigation_state_update_monotonic_.has_value() &&
        (now().seconds() - *last_navigation_state_update_monotonic_) >= navigation_state_stale_timeout_sec_;

      if (!stale) {
        return;
      }

      is_navigating_ = false;
      if (!is_paused_ && !pause_requested_) {
        current_path_msg_.reset();
        last_planned_path_msg_.reset();
        last_navigation_goal_.reset();
      }
    }

    if (stale) {
      RCLCPP_WARN(get_logger(), "Navigation state heartbeat timed out; clearing stale navigating flag.");
      publishState();
    }
  }

  void checkLaunchProcess()
  {
    std::optional<LaunchProcess> launch;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (current_launch_) {
        launch = current_launch_;
      } else {
        return;
      }
    }

    int status = 0;
    const auto result = ::waitpid(launch->pid, &status, WNOHANG);
    if (result == 0) {
      return;
    }
    if (result < 0 && errno == ECHILD) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_launch_.reset();
      current_mode_ = "idle";
      is_navigating_ = false;
      is_paused_ = false;
      pause_requested_ = false;
      last_error_ = launch->mode + " launch exited.";
    }
    publishState();
  }

  void handlePath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!msg->poses.empty()) {
      current_path_msg_ = *msg;
      last_planned_path_msg_ = *msg;
    } else {
      current_path_msg_.reset();
    }
    publishStateLocked();
  }

  void handleNavigating(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_navigation_state_update_monotonic_ = now().seconds();
    const bool new_navigating = msg->data;

    if (is_navigating_ && !new_navigating) {
      is_navigating_ = false;
      is_paused_ = false;
      pause_requested_ = false;
      current_path_msg_.reset();
      last_planned_path_msg_.reset();
      last_navigation_goal_.reset();
    } else {
      is_navigating_ = new_navigating;
      if (new_navigating) {
        is_paused_ = false;
        pause_requested_ = false;
      }
    }
    publishStateLocked();
  }

  void publishZeroVelocity()
  {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
  }

  void clearTeleop()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    teleop_owner_id_.reset();
    teleop_deadline_ = std::chrono::steady_clock::time_point{};
  }

  void publishState()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    publishStateLocked();
  }

  void publishStateLocked()
  {
    OrchestrationState msg;
    msg.stamp = now();
    msg.current_mode = current_mode_;
    msg.launch_running = launchRunningLocked();
    msg.is_navigating = is_navigating_;
    msg.is_paused = is_paused_;
    msg.pause_requested = pause_requested_;
    msg.has_navigation_goal = last_navigation_goal_.has_value();
    if (last_navigation_goal_) {
      msg.navigation_goal_x = last_navigation_goal_->first;
      msg.navigation_goal_y = last_navigation_goal_->second;
    }
    msg.last_error = last_error_;
    msg.map_yaml = default_map_yaml_;
    msg.map_save_prefix = map_save_prefix_;
    state_publisher_->publish(msg);
  }

  bool launchRunningLocked() const
  {
    return current_launch_.has_value() && current_launch_->pid > 0;
  }

  void setError(const std::string & message)
  {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = message;
    }
    publishState();
  }

  std::string normalizeMode(const std::string & mode) const
  {
    if (mode == "mapping" || mode == "localization") {
      return mode;
    }
    throw OrchestrationError(std::string("Unsupported mode: ") + mode);
  }

  pid_t spawnProcess(const std::vector<std::string> & args)
  {
    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (const auto & arg : args) {
      argv.push_back(const_cast<char *>(arg.c_str()));
    }
    argv.push_back(nullptr);

    const auto pid = ::fork();
    if (pid < 0) {
      throw OrchestrationError(std::string("fork failed: ") + std::strerror(errno));
    }

    if (pid == 0) {
      ::setsid();
      ::execvp(argv[0], argv.data());
      std::perror("execvp");
      _exit(127);
    }

    return pid;
  }

  int runCommand(const std::vector<std::string> & args, double timeout_sec)
  {
    const auto pid = spawnProcess(args);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);

    int status = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto result = ::waitpid(pid, &status, WNOHANG);
      if (result == pid) {
        if (WIFEXITED(status)) {
          return WEXITSTATUS(status);
        }
        return -1;
      }
      std::this_thread::sleep_for(100ms);
    }

    terminateProcessGroup(pid);
    return -1;
  }

  void terminateProcessGroup(pid_t pid)
  {
    if (pid <= 0) {
      return;
    }

    int status = 0;
    if (::waitpid(pid, &status, WNOHANG) == pid) {
      return;
    }

    ::killpg(pid, SIGINT);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(launch_stop_timeout_sec_);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto result = ::waitpid(pid, &status, WNOHANG);
      if (result == pid) {
        return;
      }
      std::this_thread::sleep_for(100ms);
    }

    ::killpg(pid, SIGTERM);
    std::this_thread::sleep_for(500ms);
    if (::waitpid(pid, &status, WNOHANG) == pid) {
      return;
    }

    ::killpg(pid, SIGKILL);
    ::waitpid(pid, &status, 0);
  }

  void handleSetMode(
    const std::shared_ptr<SetMode::Request> request,
    std::shared_ptr<SetMode::Response> response)
  {
    try {
      setMode(request->mode);
      response->success = true;
      response->message = "Mode updated.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  void handleSaveMap(
    const std::shared_ptr<SaveMap::Request> /*request*/,
    std::shared_ptr<SaveMap::Response> response)
  {
    try {
      response->map_yaml = saveMap();
      response->success = true;
      response->message = "Map saved.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
      response->map_yaml.clear();
    }
  }

  void handleNavigateTo(
    const std::shared_ptr<NavigateTo::Request> request,
    std::shared_ptr<NavigateTo::Response> response)
  {
    try {
      response->path_points = static_cast<uint32_t>(sendNavigationGoal(request->x_m, request->y_m));
      response->success = true;
      response->message = "Navigation goal accepted.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
      response->path_points = 0;
    }
  }

  void handlePauseNavigation(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    try {
      pauseNavigation();
      response->success = true;
      response->message = "Navigation paused.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  void handleResumeNavigation(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    try {
      resumeNavigation();
      response->success = true;
      response->message = "Navigation resumed.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  void handleCancelNavigation(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    try {
      cancelNavigation();
      response->success = true;
      response->message = "Navigation canceled.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  void handleStopMotion(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    try {
      stopMotion();
      response->success = true;
      response->message = "Motion stopped.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  void handleTeleop(
    const std::shared_ptr<TeleopCommand::Request> request,
    std::shared_ptr<TeleopCommand::Response> response)
  {
    try {
      handleTeleopCommand(
        request->client_id,
        request->deadman,
        request->linear_x,
        request->linear_y,
        request->angular_z);
      response->success = true;
      response->message = "Teleop command applied.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
    }
  }

  bool sim_{false};
  std::string startup_mode_;
  std::string bringup_package_;
  std::string bringup_launch_;
  std::string default_map_yaml_;
  std::string map_save_prefix_;
  std::string a_star_action_name_;
  std::string stop_service_name_;
  int teleop_timeout_ms_{250};
  double navigation_state_stale_timeout_sec_{0.3};
  double launch_startup_delay_sec_{1.0};
  double launch_stop_timeout_sec_{10.0};

  std::mutex state_mutex_;
  std::string current_mode_{"idle"};
  std::optional<LaunchProcess> current_launch_;
  std::optional<nav_msgs::msg::Path> current_path_msg_;
  std::optional<nav_msgs::msg::Path> last_planned_path_msg_;
  std::optional<nav_msgs::msg::Path> paused_path_msg_;
  std::optional<std::pair<double, double>> last_navigation_goal_;
  bool is_navigating_{false};
  bool is_paused_{false};
  bool pause_requested_{false};
  std::optional<double> last_navigation_state_update_monotonic_;
  std::string last_error_;
  std::optional<std::string> teleop_owner_id_;
  std::chrono::steady_clock::time_point teleop_deadline_{};

  rclcpp::Publisher<OrchestrationState>::SharedPtr state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_resume_publisher_;
  rclcpp::Client<Trigger>::SharedPtr stop_client_;
  rclcpp_action::Client<AStar>::SharedPtr a_star_client_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigating_subscription_;

  rclcpp::Service<SetMode>::SharedPtr set_mode_service_;
  rclcpp::Service<SaveMap>::SharedPtr save_map_service_;
  rclcpp::Service<NavigateTo>::SharedPtr navigate_to_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_motion_service_;
  rclcpp::Service<TeleopCommand>::SharedPtr teleop_service_;

  rclcpp::TimerBase::SharedPtr teleop_timer_;
  rclcpp::TimerBase::SharedPtr navigation_state_timer_;
  rclcpp::TimerBase::SharedPtr launch_process_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlambotOrchestrationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
