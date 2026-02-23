#include "particle_filter.hpp"

#include <utility>
#include <vector>

namespace slambot_localization
{

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

} // namespace slambot_localization
