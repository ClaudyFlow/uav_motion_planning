#ifndef SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_ASTAR
#define SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_ASTAR

#pragma region include
#pragma region include::project
#include "plan_env/grid_map.hh"
#pragma endregion include::project
#pragma region include::third
#include <Eigen/Eigen>
#pragma endregion include::third
#pragma region include::standard
#include <limits>
#include <queue>
#include <unordered_map>
#pragma endregion include::standard
#pragma endregion include

namespace path_searching {

enum class NodeStateEnum {
  NOT_EXPANDED,
  IN_OPEN_LIST,
  IN_CLOSE_LIST,
};

class PathNode {
 public:
  Eigen::Vector3d position;
  double f_cost;
  double g_cost;
  NodeStateEnum node_state;
  PathNode* parent;

  PathNode() {
    g_cost = std::numeric_limits<double>::infinity();
    f_cost = std::numeric_limits<double>::infinity();
    node_state = NodeStateEnum::NOT_EXPANDED;
    parent = nullptr;
  }
  ~PathNode() = default;
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(const PathNodePtr& node1, const PathNodePtr& node2) const {
    return node1->f_cost > node2->f_cost;
  }
};

template <typename T>
struct hash_func : std::unary_function<T, size_t> {
  size_t operator()(T const& x) const {
    size_t seed = 0;
    for (size_t i = 0; i < x.size(); ++i) {
      auto elem = *(x.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 public:
  NodeHashTable() = default;
  ~NodeHashTable() = default;

  inline void insert(Eigen::Vector3d pos, PathNodePtr node) {
    node_table_.insert(std::make_pair(pos, node));
  }

  inline PathNodePtr find(Eigen::Vector3d pos) {
    auto it = node_table_.find(pos);
    if (it != node_table_.end()) {
      return it->second;
    } else {
      return nullptr;
    }
  }

  inline void erase(Eigen::Vector3d pos) { node_table_.erase(pos); }

  inline void clear() { node_table_.clear(); }

 private:
  // don't use Eigen::Vector3i index as key, because different position may have
  // the same index
  std::unordered_map<Eigen::Vector3d, PathNodePtr, hash_func<Eigen::Vector3d>>
      node_table_;
};

class Astar {
 public:
  Astar() = default;
  ~Astar();
  void setParam(ros::NodeHandle& nh);
  void init();
  void setGridMap(GridMap::Ptr& grid_map);
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
             std::vector<Eigen::Vector3d>& path);
  void reset();

  typedef std::shared_ptr<Astar> Ptr;

 private:
  /* main data structure */
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_list_;
  NodeHashTable close_list_;
  NodeHashTable expanded_nodes_;
  std::vector<PathNodePtr> path_node_pool_;
  std::vector<Eigen::Vector3d> path_;

  /* main search parameters */
  int allocated_node_num_;  // pre-allocated the nodes num
  int use_node_num_;        // number of the nodes expanded
  double lambda_;           // weight for the heuristic term
  double tie_breaker_;      // enhance the search velocity

  enum { REACH_END = 1, NO_PATH_FOUND = 2 };

  /* main map parameters */
  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_;
  double resolution_;
  double inv_resolution_;
  GridMap::Ptr grid_map_;

  /* helper function */
  // Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  void retrievePath(PathNodePtr end_node, std::vector<Eigen::Vector3d>& path);
};

}  // namespace path_searching

#endif /* SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_ASTAR */
