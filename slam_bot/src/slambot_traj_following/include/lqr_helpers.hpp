#ifndef SLAMBOT_TRAJ_FOLLOWING__LQR_HELPERS_HPP_
#define SLAMBOT_TRAJ_FOLLOWING__LQR_HELPERS_HPP_

#include <Eigen/Dense>

namespace lqr
{

using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;

struct Pose
{
  double x;
  double y;
  double theta;
};

}  // namespace lqr

#endif  // SLAMBOT_TRAJ_FOLLOWING__LQR_HELPERS_HPP_
