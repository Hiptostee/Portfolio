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

    state_ = State::REPLANNING;
    publishState();
    requestPlan();
    return;
  }

  const bool goal_reached =
    state_ == State::NAVIGATING &&
    std::hypot(
      pose_.pose.position.x - goal_.pose.position.x,
      pose_.pose.position.y - goal_.pose.position.y) < goal_tolerance_;
  if (goal_reached) {
    have_goal_ = false;
    state_ = State::IDLE;
    publishState();
  }
}

}  // namespace paesano_orchestrator
