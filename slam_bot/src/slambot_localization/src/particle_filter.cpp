#include "particle_filter.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "rclcpp_components/register_node_macro.hpp"

namespace slambot_localization
{

ParticleFilter::ParticleFilter(const rclcpp::NodeOptions &options)
    : rclcpp::Node("particle_filter", options),
      last_odom_time_(0, 0, RCL_ROS_TIME),
      rng_(std::random_device{}())
{
  num_particles_ = std::max<int>(1, declare_parameter<int>("num_particles", 100));
  particles_x_initial_ = declare_parameter<double>("particles_x_initial", 0.25);
  particles_y_initial_ = declare_parameter<double>("particles_y_initial", 0.25);
  particles_theta_initial_ = declare_parameter<double>("particles_theta_initial", 0.15);
  x_noise_ = declare_parameter<double>("x_noise", 0.05);
  y_noise_ = declare_parameter<double>("y_noise", 0.075);
  theta_noise_ = declare_parameter<double>("theta_noise", 0.05);
  init_x_ = declare_parameter<double>("init_x", 0.0);
  init_y_ = declare_parameter<double>("init_y", 0.0);
  init_yaw_ = declare_parameter<double>("init_yaw", 0.0);
  alpha_ = declare_parameter<double>("alpha", 0.2); // scan score -> weight: weight = exp(alpha * score)

  // Frames
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "base_laser");

  // IMPORTANT: particles represent BASE pose in MAP
  // (p.x, p.y, p.theta) == map->base_link
  // lidar offset is handled ONLY inside the scan model.

  particles_.resize(static_cast<size_t>(num_particles_));
  for (auto &p : particles_) {
    p.x = randomUniform(-particles_x_initial_, particles_x_initial_);
    p.y = randomUniform(-particles_y_initial_, particles_y_initial_);
    p.theta = randomUniform(-particles_theta_initial_, particles_theta_initial_);
    p.weight = 1.0 / static_cast<double>(num_particles_);
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", rclcpp::SystemDefaultsQoS(),
      std::bind(&ParticleFilter::odomCallback, this, std::placeholders::_1));

  particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/particle_cloud", rclcpp::SystemDefaultsQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", rclcpp::SystemDefaultsQoS());

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SystemDefaultsQoS(),
      std::bind(&ParticleFilter::scanCallback, this, std::placeholders::_1));

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        have_map_ = true;
        RCLCPP_INFO(get_logger(),
                    "Map received in particle filter: size %d x %d, resolution %.3f",
                    map_.info.width, map_.info.height, map_.info.resolution);
      });

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

double ParticleFilter::randomUniform(double min, double max)
{
  std::uniform_real_distribution<double> dist(min, max);
  return dist(rng_);
}

double ParticleFilter::gaussianNoise(double stddev)
{
  std::normal_distribution<double> dist(0.0, stddev);
  return dist(rng_);
}

double ParticleFilter::wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

bool ParticleFilter::lookupOdomToBase(double &x, double &y, double &yaw)
{
  if (!tf_buffer_) return false;

  try {
    const auto tf = tf_buffer_->lookupTransform("odom", base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    yaw = tf2::getYaw(q);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// Returns BASE -> LIDAR as (dx, dy, dyaw) expressed in BASE frame.
bool ParticleFilter::lookupBaseToLidar(double &x, double &y, double &yaw)
{
  if (!tf_buffer_) return false;

  try {
    // target=base, source=lidar => base->lidar (lidar origin in base frame)
    const auto tf = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    yaw = tf2::getYaw(q);
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed TF lookup base->lidar (%s->%s): %s",
                         base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    return false;
  }
}

double ParticleFilter::effectiveSampleSize() const
{
  double sum_sq = 0.0;
  for (const auto &p : particles_) sum_sq += p.weight * p.weight;
  if (sum_sq <= 1e-12) return 0.0;
  return 1.0 / sum_sq;
}

void ParticleFilter::roughen()
{
  if (!have_map_) return;

  const double sigma_xy = 0.5 * map_.info.resolution;
  const double sigma_th = 0.01; // rad

  for (auto &p : particles_) {
    p.x += gaussianNoise(sigma_xy);
    p.y += gaussianNoise(sigma_xy);
    p.theta = wrapAngle(p.theta + gaussianNoise(sigma_th));
  }
}

void ParticleFilter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto current_time = rclcpp::Time(msg->header.stamp);

  // Use /odom/filtered as base source; override with TF odom->base if available.
  double odom_x = msg->pose.pose.position.x;
  double odom_y = msg->pose.pose.position.y;
  tf2::Quaternion odom_q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
  double odom_yaw = tf2::getYaw(odom_q);
  (void)lookupOdomToBase(odom_x, odom_y, odom_yaw);

  if (!initialized_particles_) {
    for (auto &p : particles_) {
      p.x = init_x_ + randomUniform(-particles_x_initial_, particles_x_initial_);
      p.y = init_y_ + randomUniform(-particles_y_initial_, particles_y_initial_);
      p.theta = wrapAngle(init_yaw_ + randomUniform(-particles_theta_initial_, particles_theta_initial_));
      p.weight = 1.0 / static_cast<double>(num_particles_);
    }
    initialized_particles_ = true;
    last_odom_x_ = odom_x;
    last_odom_y_ = odom_y;
    last_odom_yaw_ = odom_yaw;
    last_odom_time_ = current_time;
  } else {
    const double dt = (current_time - last_odom_time_).seconds();
    if (!(dt > 0.0)) return;

    const double dx = odom_x - last_odom_x_;
    const double dy = odom_y - last_odom_y_;
    const double dyaw = wrapAngle(odom_yaw - last_odom_yaw_);

    // convert delta into last odom body frame
    const double c = std::cos(last_odom_yaw_);
    const double s = std::sin(last_odom_yaw_);
    const double dx_body = c * dx + s * dy;
    const double dy_body = -s * dx + c * dy;

    const double trans = std::hypot(dx, dy);
    const double rot = std::abs(dyaw);

    // Deadband: if odom barely moved, do not diffuse particles.
    const bool moved = !(trans < 1e-4 && rot < 1e-4);
    if (moved) {
      const double sigma_x = x_noise_ * trans;
      const double sigma_y = y_noise_ * trans;
      const double sigma_theta = theta_noise_ * rot;

      for (auto &p : particles_) {
        const double cp = std::cos(p.theta);
        const double sp = std::sin(p.theta);

        // apply body-frame delta in each particle's heading
        p.x += cp * dx_body - sp * dy_body + gaussianNoise(sigma_x);
        p.y += sp * dx_body + cp * dy_body + gaussianNoise(sigma_y);
        p.theta = wrapAngle(p.theta + dyaw + gaussianNoise(sigma_theta));
      }
    }

    last_odom_x_ = odom_x;
    last_odom_y_ = odom_y;
    last_odom_yaw_ = odom_yaw;
    last_odom_time_ = current_time;
  }

  // Publish particle cloud (poses are in MAP)
  geometry_msgs::msg::PoseArray particles_msg;
  particles_msg.header = msg->header;
  particles_msg.header.frame_id = "map";
  particles_msg.poses.resize(particles_.size());

  for (size_t i = 0; i < particles_.size(); ++i) {
    const auto &p = particles_[i];
    particles_msg.poses[i].position.x = p.x;
    particles_msg.poses[i].position.y = p.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.theta);
    particles_msg.poses[i].orientation.x = q.x();
    particles_msg.poses[i].orientation.y = q.y();
    particles_msg.poses[i].orientation.z = q.z();
    particles_msg.poses[i].orientation.w = q.w();
  }
  particles_pub_->publish(particles_msg);
}

void ParticleFilter::computeEndpointUnderPandScore(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!have_map_ || particles_.empty()) return;

  const double res = map_.info.resolution;
  const double ox  = map_.info.origin.position.x;
  const double oy  = map_.info.origin.position.y;
  const int width  = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double rmax = msg->range_max;

  // base->lidar extrinsic (in base frame)
  double b2l_x = 0.0, b2l_y = 0.0, b2l_yaw = 0.0;
  const bool have_extrinsic = lookupBaseToLidar(b2l_x, b2l_y, b2l_yaw);

  double weight_sum = 0.0;

  for (auto &p : particles_) {
    // particles are base pose in map
    double lidar_x = p.x;
    double lidar_y = p.y;
    double lidar_yaw = p.theta;

    // convert base pose -> lidar pose in map
    if (have_extrinsic) {
      const double c = std::cos(p.theta);
      const double s = std::sin(p.theta);
      lidar_x   = p.x + c * b2l_x - s * b2l_y;
      lidar_y   = p.y + s * b2l_x + c * b2l_y;
      lidar_yaw = wrapAngle(p.theta + b2l_yaw);
    }

    double score = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); i += 5) {
      const double range = msg->ranges[i];
      if (!std::isfinite(range) || range >= rmax * 0.99) continue;

      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const double phi = lidar_yaw + angle;

      const double xe = lidar_x + range * std::cos(phi);
      const double ye = lidar_y + range * std::sin(phi);

      const int gx = static_cast<int>(std::floor((xe - ox) / res));
      const int gy = static_cast<int>(std::floor((ye - oy) / res));
      if (gx < 0 || gx >= width || gy < 0 || gy >= height) continue;

      const int idx = gy * width + gx;
      const int8_t occ = map_.data[idx];

      if (occ == -1) continue;      // unknown -> ignore
      if (occ >= 50) score += 1.0; // occupied -> good
      else score -= 1.0;           // free -> bad
    }

    p.weight = std::exp(alpha_ * score); // keep >0
    weight_sum += p.weight;
  }

  if (weight_sum > 0.0) {
    for (auto &p : particles_) p.weight /= weight_sum;
  }
}

void ParticleFilter::systematicResample()
{
  const size_t n = particles_.size();
  if (n == 0) return;

  std::vector<double> cdf(n);
  double cumulative = 0.0;
  for (size_t i = 0; i < n; ++i) {
    cumulative += particles_[i].weight;
    cdf[i] = cumulative;
  }

  const double step = 1.0 / static_cast<double>(n);
  const double r = randomUniform(0.0, step);

  std::vector<Particle> new_particles(n);
  size_t idx = 0;

  for (size_t i = 0; i < n; ++i) {
    const double u = r + static_cast<double>(i) * step;
    while (idx + 1 < n && u > cdf[idx]) ++idx;
    new_particles[i] = particles_[idx];
    new_particles[i].weight = 1.0 / static_cast<double>(n);
  }

  particles_ = std::move(new_particles);
}

void ParticleFilter::broadCastMapToOdomTf(const rclcpp::Time &stamp)
{
  if (!tf_broadcaster_) return;
  if (!initialized_particles_) return;

  const double odom_x = last_odom_x_;
  const double odom_y = last_odom_y_;
  const double odom_yaw = last_odom_yaw_;

  // map->base estimated from particles (weighted mean)
  double est_x = 0.0, est_y = 0.0;
  double sum_sin = 0.0, sum_cos = 0.0;
  for (const auto &p : particles_) {
    est_x += p.x * p.weight;
    est_y += p.y * p.weight;
    sum_sin += std::sin(p.theta) * p.weight;
    sum_cos += std::cos(p.theta) * p.weight;
  }
  const double est_yaw = std::atan2(sum_sin, sum_cos);

  tf2::Transform map_to_base;
  map_to_base.setOrigin(tf2::Vector3(est_x, est_y, 0.0));
  tf2::Quaternion q_map;
  q_map.setRPY(0.0, 0.0, est_yaw);
  map_to_base.setRotation(q_map);

  tf2::Transform odom_to_base;
  odom_to_base.setOrigin(tf2::Vector3(odom_x, odom_y, 0.0));
  tf2::Quaternion q_odom;
  q_odom.setRPY(0.0, 0.0, odom_yaw);
  odom_to_base.setRotation(q_odom);

  // map->odom = map->base * (odom->base)^-1
  const tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";
  tf_msg.transform.translation.x = map_to_odom.getOrigin().x();
  tf_msg.transform.translation.y = map_to_odom.getOrigin().y();
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = map_to_odom.getRotation().x();
  tf_msg.transform.rotation.y = map_to_odom.getRotation().y();
  tf_msg.transform.rotation.z = map_to_odom.getRotation().z();
  tf_msg.transform.rotation.w = map_to_odom.getRotation().w();

  tf_broadcaster_->sendTransform(tf_msg);
}

void ParticleFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 1) measurement update
  computeEndpointUnderPandScore(msg);

  // 2) publish map->odom using current particle estimate
  broadCastMapToOdomTf(rclcpp::Time(msg->header.stamp));

  // 3) publish estimated pose (map->base) as weighted mean
  geometry_msgs::msg::PoseStamped est_pose;
  est_pose.header = msg->header;
  est_pose.header.frame_id = "map";

  double est_x = 0.0, est_y = 0.0;
  double sum_sin = 0.0, sum_cos = 0.0;
  for (const auto &p : particles_) {
    est_x += p.x * p.weight;
    est_y += p.y * p.weight;
    sum_sin += std::sin(p.theta) * p.weight;
    sum_cos += std::cos(p.theta) * p.weight;
  }
  const double est_yaw = std::atan2(sum_sin, sum_cos);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, est_yaw);

  est_pose.pose.position.x = est_x;
  est_pose.pose.position.y = est_y;
  est_pose.pose.position.z = 0.0;
  est_pose.pose.orientation.x = q.x();
  est_pose.pose.orientation.y = q.y();
  est_pose.pose.orientation.z = q.z();
  est_pose.pose.orientation.w = q.w();

  pose_pub_->publish(est_pose);

  // 4) resample only when needed (for next cycle)
  const double neff = effectiveSampleSize();
  if (neff < 0.5 * particles_.size()) {
    systematicResample();
    roughen();
  }
}

} // namespace slambot_localization

RCLCPP_COMPONENTS_REGISTER_NODE(slambot_localization::ParticleFilter)
