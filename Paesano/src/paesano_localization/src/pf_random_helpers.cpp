#include "paesano_localization/particle_filter.hpp"

#include <cmath>
#include <random>

namespace paesano_localization
{

// The motion update applies the odometry delta to each particle with added noise. 
double ParticleFilter::randomUniform(double min, double max)
{
  return uniform_dist_(rng_, decltype(uniform_dist_)::param_type(min, max));
}

// Sample from a zero-mean Gaussian distribution with the given standard deviation. This is used to add noise to the particles 
// during the motion update, with the amount of noise scaled based on the amount of movement according to odometry.
double ParticleFilter::gaussianNoise(double stddev)
{
  return normal_dist_(rng_, decltype(normal_dist_)::param_type(0.0, stddev));
}

// Wrap angle to [-pi, pi]. This is used to ensure that the particle orientations remain within a standard range, which helps with convergence and prevents issues with angle discontinuities.
double ParticleFilter::wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

} // namespace paesano_localization
