#ifndef PAESANO_NAVIGATION__A_STAR_HELPERS_HPP_
#define PAESANO_NAVIGATION__A_STAR_HELPERS_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace paesano_navigation
{

struct Coordinate
{
  int x;
  int y;
};

}  // namespace paesano_navigation

#endif  // PAESANO_NAVIGATION__A_STAR_HELPERS_HPP_
