#include "particle_filter.hpp"

#include <cmath>
#include <random>

namespace slambot_localization
{

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

} // namespace slambot_localization
