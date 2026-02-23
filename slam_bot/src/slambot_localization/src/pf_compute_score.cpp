#include "particle_filter.hpp"

#include <cmath>

namespace slambot_localization
{

void ParticleFilter::computeEndpointUnderPandScore(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!have_map_ || particles_.empty()) return;

  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;
  const int width = static_cast<int>(map_.info.width);
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
      lidar_x = p.x + c * b2l_x - s * b2l_y;
      lidar_y = p.y + s * b2l_x + c * b2l_y;
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
      if (occ >= 50) score += 1.0;  // occupied -> good
      else score -= 1.0;            // free -> bad
    }

    p.weight = std::exp(alpha_ * score);  // keep >0
    weight_sum += p.weight;
  }

  if (weight_sum > 0.0) {
    for (auto &p : particles_) p.weight /= weight_sum;
  }
}

} // namespace slambot_localization
