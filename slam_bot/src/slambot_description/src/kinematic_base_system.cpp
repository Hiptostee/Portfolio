// Kinematic base controller for Gazebo Sim (gz-sim)
// Applies /cmd_vel directly to the model's linear and angular velocity.

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Element.hh>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace gz;
using namespace gz::sim;

namespace slambot
{
class KinematicBaseSystem : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  KinematicBaseSystem() = default;
  ~KinematicBaseSystem() override
  {
    this->running_.store(false);
    if (spin_thread_.joinable())
      spin_thread_.join();
  }

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager & /*_eventMgr*/) override
  {
    this->model_ = Model(_entity);
    if (!this->model_.Valid(_ecm))
    {
      gzerr << "KinematicBaseSystem: Invalid model entity" << std::endl;
      return;
    }

    // Parameters from SDF
    if (_sdf && _sdf->HasElement("topic"))
      this->topic_ = _sdf->Get<std::string>("topic");
    if (_sdf && _sdf->HasElement("max_linear"))
      this->max_linear_ = std::max(0.0, _sdf->Get<double>("max_linear"));
    if (_sdf && _sdf->HasElement("max_angular"))
      this->max_angular_ = std::max(0.0, _sdf->Get<double>("max_angular"));

    // ROS 2 node for cmd_vel
    rclcpp::InitOptions opts;
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr, opts);

    node_ = std::make_shared<rclcpp::Node>("slambot_kinematic_base");
    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist &msg)
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->cmd_lin_.X() = msg.linear.x;
        this->cmd_lin_.Y() = msg.linear.y;
        this->cmd_lin_.Z() = 0.0;
        this->cmd_ang_.Z() = msg.angular.z;
        this->cmd_ang_.X() = 0.0;
        this->cmd_ang_.Y() = 0.0;
      }
    );

    running_.store(true);
    spin_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(this->node_);
      while (rclcpp::ok() && this->running_.load())
      {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });

    gzdbg << "KinematicBaseSystem configured for model '" << this->model_.Name(_ecm)
          << "' listening on topic '" << this->topic_ << "'" << std::endl;
  }

  void PreUpdate(const UpdateInfo & /*_info*/, EntityComponentManager &_ecm) override
  {
    if (!this->model_.Valid(_ecm))
      return;

    // Read latest command under lock
    math::Vector3d lin_cmd;
    math::Vector3d ang_cmd;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      lin_cmd = this->cmd_lin_;
      ang_cmd = this->cmd_ang_;
    }

    // Clamp
    lin_cmd.X() = std::clamp(lin_cmd.X(), -max_linear_, max_linear_);
    lin_cmd.Y() = std::clamp(lin_cmd.Y(), -max_linear_, max_linear_);
    lin_cmd.Z() = 0.0;
    ang_cmd.X() = 0.0;
    ang_cmd.Y() = 0.0;
    ang_cmd.Z() = std::clamp(ang_cmd.Z(), -max_angular_, max_angular_);

    // Write command components on the model entity
    auto ent = this->model_.Entity();
    auto linCmdComp = _ecm.Component<components::LinearVelocityCmd>(ent);
    // LinearVelocityCmd is interpreted in the model (body) frame by Gazebo Sim.
    // So we do NOT rotate to world; we pass body-frame velocities directly.
    if (!linCmdComp)
      _ecm.CreateComponent(ent, components::LinearVelocityCmd(lin_cmd));
    else
      linCmdComp->Data() = lin_cmd;

    auto angCmdComp = _ecm.Component<components::AngularVelocityCmd>(ent);
    if (!angCmdComp)
      _ecm.CreateComponent(ent, components::AngularVelocityCmd(ang_cmd));
    else
      angCmdComp->Data() = ang_cmd;
  }

private:
  Model model_{kNullEntity};
  std::string topic_{"/cmd_vel"};
  double max_linear_{2.0};     // m/s
  double max_angular_{3.0};    // rad/s

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  math::Vector3d cmd_lin_{0, 0, 0};
  math::Vector3d cmd_ang_{0, 0, 0};
};
}  // namespace slambot

GZ_ADD_PLUGIN(
  slambot::KinematicBaseSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(slambot::KinematicBaseSystem, "slambot::KinematicBaseSystem")
GZ_ADD_PLUGIN_ALIAS(slambot::KinematicBaseSystem, "slambot_kinematic_base")
