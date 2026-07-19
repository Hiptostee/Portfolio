#include "paesano_explorer/frontier_clusterer.hpp"

namespace paesano_explorer
{

std::vector<FrontierCluster> FrontierClusterer::cluster(
  const std::vector<GridCell> & frontier_cells,
  int map_width,
  int map_height,
  std::size_t minimum_cluster_size) const
{
  (void)frontier_cells;
  (void)map_width;
  (void)map_height;
  (void)minimum_cluster_size;

  // TODO(joseph): Build a frontier mask and run eight-connected BFS.
  return {};
}

}  // namespace paesano_explorer
