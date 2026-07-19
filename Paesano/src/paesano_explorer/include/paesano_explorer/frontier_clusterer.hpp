#ifndef PAESANO_EXPLORER__FRONTIER_CLUSTERER_HPP_
#define PAESANO_EXPLORER__FRONTIER_CLUSTERER_HPP_

#include <cstddef>
#include <vector>

#include "paesano_explorer/types.hpp"

namespace paesano_explorer
{

class FrontierClusterer
{
public:
  std::vector<FrontierCluster> cluster(
    const std::vector<GridCell> & frontier_cells,
    int map_width,
    int map_height,
    std::size_t minimum_cluster_size) const;
};

}  // namespace paesano_explorer

#endif  // PAESANO_EXPLORER__FRONTIER_CLUSTERER_HPP_
