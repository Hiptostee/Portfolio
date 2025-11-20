// Kinematic base controller for Gazebo Sim (gz-sim)
// Applies /cmd_vel directly to the model's linear and angular velocity.

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <cmath>
#include <sstream>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Element.hh>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

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
    if (_sdf && _sdf->HasElement("publish_rate"))
    {
      auto hz = std::max(1.0, _sdf->Get<double>("publish_rate"));
      this->publish_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / hz));
    }
    if (_sdf && _sdf->HasElement("publish_odom"))
      this->publish_odom_ = _sdf->Get<bool>("publish_odom");
    if (_sdf && _sdf->HasElement("odom_topic"))
      this->odom_topic_ = _sdf->Get<std::string>("odom_topic");
    if (_sdf && _sdf->HasElement("odom_frame"))
      this->odom_frame_ = _sdf->Get<std::string>("odom_frame");
    if (_sdf && _sdf->HasElement("base_frame"))
      this->base_frame_ = _sdf->Get<std::string>("base_frame");

    if (_sdf && _sdf->HasElement("publish_encoders"))
      this->publish_encoders_ = _sdf->Get<bool>("publish_encoders");
    if (_sdf && _sdf->HasElement("ticks_topic"))
      this->ticks_topic_ = _sdf->Get<std::string>("ticks_topic");
    if (_sdf && _sdf->HasElement("joint_state_topic"))
      this->joint_state_topic_ = _sdf->Get<std::string>("joint_state_topic");
    if (_sdf && _sdf->HasElement("ticks_per_rev"))
      this->ticks_per_rev_ = _sdf->Get<int>("ticks_per_rev");
    if (_sdf && _sdf->HasElement("wheel_radius"))
      this->wheel_radius_ = std::max(1e-6, _sdf->Get<double>("wheel_radius"));
    if (_sdf && _sdf->HasElement("base_length"))
      this->base_length_ = std::max(1e-6, _sdf->Get<double>("base_length"));
    if (_sdf && _sdf->HasElement("base_width"))
      this->base_width_ = std::max(1e-6, _sdf->Get<double>("base_width"));
    if (_sdf && _sdf->HasElement("front_left_joint"))
      this->fl_joint_name_ = _sdf->Get<std::string>("front_left_joint");
    if (_sdf && _sdf->HasElement("front_right_joint"))
      this->fr_joint_name_ = _sdf->Get<std::string>("front_right_joint");
    if (_sdf && _sdf->HasElement("back_left_joint"))
      this->bl_joint_name_ = _sdf->Get<std::string>("back_left_joint");
    if (_sdf && _sdf->HasElement("back_right_joint"))
      this->br_joint_name_ = _sdf->Get<std::string>("back_right_joint");

    // Optional covariance diagonals (6 values: x y z roll pitch yaw)
    if (_sdf && _sdf->HasElement("pose_cov_diag"))
    {
      auto s = _sdf->Get<std::string>("pose_cov_diag");
      ParseDiag6(s, pose_cov_diag_);
    }
    if (_sdf && _sdf->HasElement("twist_cov_diag"))
    {
      auto s = _sdf->Get<std::string>("twist_cov_diag");
      ParseDiag6(s, twist_cov_diag_);
    }

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

    if (publish_odom_)
    {
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    }
    if (publish_encoders_)
    {
      if (!ticks_topic_.empty())
        ticks_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>(ticks_topic_, rclcpp::QoS(10));
      if (!joint_state_topic_.empty())
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::QoS(10));
    }

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

  void PreUpdate(const UpdateInfo & _info, EntityComponentManager &_ecm) override
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

    // Publish odometry and encoder data at the configured rate
    const auto now = _info.simTime;
    if (now - last_pub_time_ >= publish_period_)
    {
      last_pub_time_ = now;

      // Read current pose
      auto poseComp = _ecm.Component<components::Pose>(ent);
      math::Pose3d pose = poseComp ? poseComp->Data() : math::Pose3d::Zero;

      // Compute body-frame twist from pose delta
      double dt = std::chrono::duration<double>(now - last_odom_time_).count();
      if (dt <= 0.0)
        dt = std::chrono::duration<double>(publish_period_).count();

      math::Vector3d body_lin{0, 0, 0};
      double body_yaw_rate = 0.0;
      if (has_prev_pose_)
      {
        math::Vector3d dpos_world = pose.Pos() - prev_pose_.Pos();
        // Rotate world delta into body frame using current orientation
        body_lin = pose.Rot().RotateVectorReverse(dpos_world / dt);
        // Yaw rate (about Z)
        double yaw = pose.Rot().Yaw();
        double yaw_prev = prev_pose_.Rot().Yaw();
        double dyaw = math::Angle(yaw).Normalized().Radian() - math::Angle(yaw_prev).Normalized().Radian();
        // Normalize to [-pi, pi]
        dyaw = math::Angle(dyaw).Normalized().Radian();
        body_yaw_rate = dyaw / dt;
      }
      prev_pose_ = pose;
      has_prev_pose_ = true;
      last_odom_time_ = now;

      // Odometry publishing
      if (publish_odom_ && odom_pub_)
      {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = ToRosTime(now);
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = pose.Pos().X();
        odom.pose.pose.position.y = pose.Pos().Y();
        odom.pose.pose.position.z = pose.Pos().Z();
        odom.pose.pose.orientation.x = pose.Rot().X();
        odom.pose.pose.orientation.y = pose.Rot().Y();
        odom.pose.pose.orientation.z = pose.Rot().Z();
        odom.pose.pose.orientation.w = pose.Rot().W();

        // Velocities in base frame
        odom.twist.twist.linear.x = body_lin.X();
        odom.twist.twist.linear.y = body_lin.Y();
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.z = body_yaw_rate;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;

        // Set covariances from configured diagonals
        FillCovariance(pose_cov_diag_, odom.pose.covariance);
        FillCovariance(twist_cov_diag_, odom.twist.covariance);

        odom_pub_->publish(odom);
      }

      // Encoder emulation (ticks and/or joint states)
      if (publish_encoders_ && (ticks_pub_ || joint_state_pub_))
      {
        // Wheel angular velocities from mecanum IK using body-frame twist
        const double L = base_length_ / 2.0;
        const double W = base_width_  / 2.0;
        const double R = wheel_radius_ > 0.0 ? wheel_radius_ : 0.05;
        const double vx = body_lin.X();
        const double vy = body_lin.Y();
        const double wz = body_yaw_rate;

        double w_fl = (vx - vy - (L + W) * wz) / R;
        double w_fr = (vx + vy + (L + W) * wz) / R;
        double w_bl = (vx + vy - (L + W) * wz) / R;
        double w_br = (vx - vy + (L + W) * wz) / R;

        // Integrate positions and ticks
        wheel_pos_[0] += w_fl * dt;
        wheel_pos_[1] += w_fr * dt;
        wheel_pos_[2] += w_bl * dt;
        wheel_pos_[3] += w_br * dt;

        const double ticks_per_rad = ticks_per_rev_ / (2.0 * 3.14159265358979323846);
        wheel_ticks_[0] = static_cast<int64_t>(std::llround(wheel_pos_[0] * ticks_per_rad));
        wheel_ticks_[1] = static_cast<int64_t>(std::llround(wheel_pos_[1] * ticks_per_rad));
        wheel_ticks_[2] = static_cast<int64_t>(std::llround(wheel_pos_[2] * ticks_per_rad));
        wheel_ticks_[3] = static_cast<int64_t>(std::llround(wheel_pos_[3] * ticks_per_rad));

        if (ticks_pub_)
        {
          std_msgs::msg::Int32MultiArray msg;
          msg.data = {
            static_cast<int32_t>(wheel_ticks_[0]),
            static_cast<int32_t>(wheel_ticks_[1]),
            static_cast<int32_t>(wheel_ticks_[2]),
            static_cast<int32_t>(wheel_ticks_[3])
          };
          ticks_pub_->publish(msg);
        }

        if (joint_state_pub_)
        {
          sensor_msgs::msg::JointState js;
          js.header.stamp = ToRosTime(now);
          js.name = {fl_joint_name_, fr_joint_name_, bl_joint_name_, br_joint_name_};
          js.position = {wheel_pos_[0], wheel_pos_[1], wheel_pos_[2], wheel_pos_[3]};
          js.velocity = {w_fl, w_fr, w_bl, w_br};
          joint_state_pub_->publish(js);
        }
      }
    }
  }

private:
  Model model_{kNullEntity};
  std::string topic_{"/cmd_vel"};
  double max_linear_{2.0};     // m/s
  double max_angular_{3.0};    // rad/s

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ticks_pub_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  math::Vector3d cmd_lin_{0, 0, 0};
  math::Vector3d cmd_ang_{0, 0, 0};

  // Odom / encoder options
  bool publish_odom_{true};
  std::string odom_topic_{"/odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  bool publish_encoders_{false};
  std::string joint_state_topic_{"/encoder_joint_states"};
  std::string ticks_topic_{"/wheel_ticks"};
  double wheel_radius_{0.0485};
  double base_length_{0.297};
  double base_width_{0.256};
  int ticks_per_rev_{2048};
  std::string fl_joint_name_{"front_left_wheel_joint"};
  std::string fr_joint_name_{"front_right_wheel_joint"};
  std::string bl_joint_name_{"back_left_wheel_joint"};
  std::string br_joint_name_{"back_right_wheel_joint"};

  // Covariance diagonals [x y z roll pitch yaw]
  double pose_cov_diag_[6]{1e-4, 1e-4, 1e6, 1e6, 1e6, 1e-3};
  double twist_cov_diag_[6]{1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2};

  // Encoder state
  double wheel_pos_[4]{0.0,0.0,0.0,0.0};
  long long wheel_ticks_[4]{0,0,0,0};

  // Publish throttling
  std::chrono::steady_clock::duration publish_period_{std::chrono::milliseconds(20)}; // 50 Hz
  std::chrono::steady_clock::duration last_pub_time_{};

  // Odom derivation helpers
  std::chrono::steady_clock::duration last_odom_time_{};
  math::Pose3d prev_pose_{};
  bool has_prev_pose_{false};

  // Helper: convert sim time to ROS 2 time
  static builtin_interfaces::msg::Time ToRosTime(const std::chrono::steady_clock::duration &t)
  {
    using namespace std::chrono;
    auto ns = duration_cast<nanoseconds>(t).count();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return stamp;
  }

  static void FillCovariance(const double diag[6], std::array<double, 36> &cov)
  {
    cov.fill(0.0);
    // Row-major 6x6: indices (0..5) along x y z r p y
    for (int i = 0; i < 6; ++i)
    {
      cov[i * 6 + i] = diag[i];
    }
  }

  static void ParseDiag6(const std::string &s, double out[6])
  {
    std::stringstream ss(s);
    for (int i = 0; i < 6; ++i)
    {
      if (!(ss >> out[i]))
        break;
    }
  }
};
}  // namespace slambot

GZ_ADD_PLUGIN(
  slambot::KinematicBaseSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(slambot::KinematicBaseSystem, "slambot::KinematicBaseSystem")
GZ_ADD_PLUGIN_ALIAS(slambot::KinematicBaseSystem, "slambot_kinematic_base")
