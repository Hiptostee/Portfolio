#include "particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace slambot_localization
{
namespace
{

constexpr int8_t kOccupiedThreshold = 50;
constexpr double kMaxObstacleLookupDistanceMeters = 1.0;
constexpr double kHugeSquaredDistanceCells = 1e12;

std::pair<double, double> computeEndpoint(
    const Particle &particle,
    size_t beam_index,
    const sensor_msgs::msg::LaserScan::SharedPtr &msg,
    double b2l_x,
    double b2l_y,
    double b2l_yaw,
    bool have_extrinsic)
{
  const double beam_angle =
      msg->angle_min + static_cast<double>(beam_index) * msg->angle_increment;
  const double range = msg->ranges[beam_index];

  const double c = std::cos(particle.theta);
  const double s = std::sin(particle.theta);

  const double lidar_x = have_extrinsic ? particle.x + c * b2l_x - s * b2l_y : particle.x;
  const double lidar_y = have_extrinsic ? particle.y + s * b2l_x + c * b2l_y : particle.y;
  const double lidar_yaw = have_extrinsic ? particle.theta + b2l_yaw : particle.theta;

  const double ray_yaw = lidar_yaw + beam_angle;
  return {
      lidar_x + range * std::cos(ray_yaw),
      lidar_y + range * std::sin(ray_yaw),
  };
}

void distanceTransform1D(const std::vector<double> &f, int n, std::vector<double> &d,
                         std::vector<int> &v, std::vector<double> &z)
{
  if (n <= 0) return;

  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int vk = v[k];
      const double qf = f[q] + static_cast<double>(q) * static_cast<double>(q);
      const double vkf = f[vk] + static_cast<double>(vk) * static_cast<double>(vk);
      s = (qf - vkf) / (2.0 * static_cast<double>(q - vk));
      if (s > z[k]) break;
      --k;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) ++k;
    const double delta = static_cast<double>(q - v[k]);
    d[q] = delta * delta + f[v[k]];
  }
}

}  // namespace

void ParticleFilter::rebuildDistanceField()
{
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double res = map_.info.resolution;

  const size_t cell_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (width <= 0 || height <= 0 || res <= 0.0 || map_.data.size() != cell_count) {
    distance_field_m_.clear();
    have_distance_field_ = false;
    return;
  }

  std::vector<double> col_sq_dist(cell_count, kHugeSquaredDistanceCells);
  std::vector<double> row_sq_dist(cell_count, kHugeSquaredDistanceCells);

  const int max_dim = std::max(width, height);
  std::vector<double> f(static_cast<size_t>(max_dim));
  std::vector<double> d(static_cast<size_t>(max_dim));
  std::vector<int> v(static_cast<size_t>(max_dim));
  std::vector<double> z(static_cast<size_t>(max_dim + 1));

  // Pass 1: column transform.
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      f[static_cast<size_t>(y)] = (map_.data[idx] >= kOccupiedThreshold) ? 0.0 : kHugeSquaredDistanceCells;
    }
    distanceTransform1D(f, height, d, v, z);
    for (int y = 0; y < height; ++y) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      col_sq_dist[idx] = d[static_cast<size_t>(y)];
    }
  }

  // Pass 2: row transform.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      f[static_cast<size_t>(x)] = col_sq_dist[idx];
    }
    distanceTransform1D(f, width, d, v, z);
    for (int x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
      row_sq_dist[idx] = d[static_cast<size_t>(x)];
    }
  }

  distance_field_m_.resize(cell_count);
  for (size_t i = 0; i < cell_count; i++){
    distance_field_m_[i] = static_cast<float>(std::sqrt(row_sq_dist[i]) * res);
  }
  have_distance_field_ = true;
}

double ParticleFilter::distanceToNearestObstacle(double map_x, double map_y) const
{
  if (!have_distance_field_) return kMaxObstacleLookupDistanceMeters;

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;
  if (width <= 0 || height <= 0 || res <= 0.0) return kMaxObstacleLookupDistanceMeters;

  const size_t cell_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (distance_field_m_.size() != cell_count) return kMaxObstacleLookupDistanceMeters;

  const int mx = static_cast<int>(std::floor((map_x - ox) / res));
  const int my = static_cast<int>(std::floor((map_y - oy) / res));
  if (mx < 0 || my < 0 || mx >= width || my >= height) return kMaxObstacleLookupDistanceMeters;

  const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(width) + static_cast<size_t>(mx);
  const double d = static_cast<double>(distance_field_m_[idx]);
  if (!std::isfinite(d)) return kMaxObstacleLookupDistanceMeters;
  return std::min(d, kMaxObstacleLookupDistanceMeters);
}

void ParticleFilter::score(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg || !have_map_ || !have_distance_field_ || particles_.empty()) return;

  const double rmax = msg->range_max;

  // Params you should declare:
  // double sigma_hit_ (meters) e.g. 0.10
  // double z_hit_     e.g. 0.9
  // double z_rand_    e.g. 0.1
  const double sigma = std::max(sigma_hit_, 1e-6);
  const double inv_zmax = 1.0 / std::max(rmax, 1e-6);

  // Precompute constants
  const double log_norm = -std::log(std::sqrt(2.0 * M_PI) * sigma);  // Gaussian normalizer
  const double inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

  const double log_z_hit  = std::log(std::max(z_hit_, 1e-12));
  const double log_z_rand = std::log(std::max(z_rand_, 1e-12));
  const double log_rand   = std::log(inv_zmax);

  // base->lidar extrinsic
  double b2l_x = 0.0, b2l_y = 0.0, b2l_yaw = 0.0;
  const bool have_extrinsic = lookupBaseToLidar(b2l_x, b2l_y, b2l_yaw);

  size_t valid_beam_count = 0;
  for (size_t i = 0; i < msg->ranges.size(); i += static_cast<size_t>(beam_stride_)) {
    const double range = msg->ranges[i];
    if (!std::isfinite(range)) continue;
    if (range < msg->range_min) continue;
    if (range >= rmax * 0.99) continue;
    ++valid_beam_count;
  }
  if (valid_beam_count == 0) return;

  std::vector<double> logw(particles_.size(), 0.0);

  for (size_t pi = 0; pi < particles_.size(); ++pi) {
    const Particle &p = particles_[pi];
    double acc = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); i += static_cast<size_t>(beam_stride_)) {
      const double range = msg->ranges[i];

      // Skip invalid + max-range (usually no information for endpoint model)
      if (!std::isfinite(range)) continue;
      if (range < msg->range_min) continue;
      if (range >= rmax * 0.99) continue;

      const auto endpoint = computeEndpoint(p, i, msg, b2l_x, b2l_y, b2l_yaw, have_extrinsic);
      const double d = distanceToNearestObstacle(endpoint.first, endpoint.second);

      // log p_hit = log(z_hit * N(d;0,sigma))
      const double log_p_hit = log_z_hit + log_norm - (d * d) * inv_2sigma2;

      // log p_rand = log(z_rand * 1/zmax)
      const double log_p_rand = log_z_rand + log_rand;

      // log p = log( exp(log_p_hit) + exp(log_p_rand) )
      const double m = std::max(log_p_hit, log_p_rand);
      const double log_p = m + std::log(std::exp(log_p_hit - m) + std::exp(log_p_rand - m));

      acc += log_p;
    }

    logw[pi] = acc;
  }

  // Normalize weights with log-sum-exp (you already do the max trick)
  const double max_logw = *std::max_element(logw.begin(), logw.end());
  double sum_exp_shifted = 0.0;
  for (const double lw : logw) sum_exp_shifted += std::exp(lw - max_logw);

  if (sum_exp_shifted > 0.0) {
    const double logsumexp = max_logw + std::log(sum_exp_shifted);
    const double log_mean_likelihood =
        logsumexp - std::log(static_cast<double>(particles_.size()));
    const double scan_quality =
        std::exp(log_mean_likelihood / static_cast<double>(valid_beam_count));

    if (std::isfinite(scan_quality)) {
      if (!weight_averages_initialized_) {
        w_fast_ = scan_quality;
        w_slow_ = scan_quality;
        weight_averages_initialized_ = true;
      } else {
        w_fast_ += alpha_fast_ * (scan_quality - w_fast_);
        w_slow_ += alpha_slow_ * (scan_quality - w_slow_);
        
      }
    }
  }

  double total = 0.0;
  for (size_t i = 0; i < particles_.size(); ++i) {
    particles_[i].weight *= std::exp(logw[i] - max_logw);
    total += particles_[i].weight;
  }

  if (total > 1e-12) {
    for (auto &p : particles_) p.weight /= total;
    have_measurement_ = true;
  } else {
    const double w = 1.0 / static_cast<double>(particles_.size());
    for (auto &p : particles_) p.weight = w;
  }
}

} // namespace slambot_localization
