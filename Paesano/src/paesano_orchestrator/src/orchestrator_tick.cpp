#include "paesano_orchestrator/orchestrator.hpp"

#include <cmath>
#include <memory>

namespace paesano_orchestrator
{

// This is the complete navigation policy. All ROS I/O stays in orchestrator.cpp.
void Orchestrator::tick()
{
  if (state_ == State::IDLE && have_goal_) {
    state_ = State::PLANNING;
    publishState();
  }

  if (state_ == State::PLANNING) {
    requestPlan();
    return;
  }

  if (state_ == State::NAVIGATING && blocked_) {
    blocked_since_ = now();
    state_ = State::WAITING;
    publishState();
    return;
  }

  if (state_ == State::WAITING && !blocked_) {
    state_ = State::NAVIGATING;
    publishState();
    return;
  }

  const bool blocked_long_enough =
    state_ == State::WAITING && blocked_ &&
    (now() - blocked_since_).seconds() >= delay_;

  if (blocked_long_enough) {
    if (stop_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      stop_client_->async_send_request(request);
    }

    const auto current_time = now();
    blocked_recovery_active_ = true;
    blocked_recovery_started_ = current_time;
    next_replan_attempt_ =
      current_time + rclcpp::Duration::from_seconds(replan_retry_interval_);
    state_ = State::REPLANNING;
    publishState();
    requestPlan();
    return;
  }

  if (state_ == State::REPLANNING && blocked_recovery_active_) {
    const auto current_time = now();
    const bool recovery_timed_out =
      (current_time - blocked_recovery_started_).seconds() >= replan_timeout_;

    if (recovery_timed_out && !plan_in_flight_) {
      have_goal_ = false;
      blocked_recovery_active_ = false;
      state_ = State::IDLE;
      publishState();
      publishResult("PLANNING_FAILED");
      return;
    }

    if (!plan_in_flight_ && current_time >= next_replan_attempt_) {
      next_replan_attempt_ =
        current_time + rclcpp::Duration::from_seconds(replan_retry_interval_);
      requestPlan();
    }
    return;
  }

  const bool goal_reached =
    state_ == State::NAVIGATING &&
    std::hypot(
      pose_.pose.position.x - goal_.pose.position.x,
      pose_.pose.position.y - goal_.pose.position.y) < goal_tolerance_;

  if (goal_reached) {
    have_goal_ = false;
    blocked_recovery_active_ = false;
    state_ = State::IDLE;
    publishState();
    publishResult("SUCCEEDED");
  }
}

}  // namespace paesano_orchestrator
