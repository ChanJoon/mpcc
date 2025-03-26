#include "mpcc/astar.hpp"

AStar::AStar() {}

AStar::~AStar() {
  for (int i = 0; i < use_node_num_; i++) {
    delete path_node_pool_[i];
  }
}

void AStar::setGridMap(GridMap::Ptr& grid_map) {
  grid_map_ = grid_map;
}

void AStar::init() {
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node0();
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

void AStar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::set<NodePtr, NodeComparator> empty_set;
  open_set_.swap(empty_set);

  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void AStar::setParam() {
  lnh_.param("astar/resolution", resolution_, -1.0);
  lnh_.param("astar/time_resolution", time_resolution_, -1.0);
  lnh_.param("astar/lambda_heuristic", lambda_heuristic_, -1.0);
  lnh_.param("astar/allocate_num", allocate_num_, -1);
  ROS_INFO("astar/resolution: %f", resolution_);
  ROS_INFO("astar/time_resolution: %f", time_resolution_);
  ROS_INFO("astar/lambda_heuristic: %f", lambda_heuristic_);
  ROS_INFO("astar/allocate_num: %d", allocate_num_);
  tie_breaker_ = 1.0 + 1.0 / 10000;
}

double AStar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

double AStar::getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = std::abs(x1(0) - x2(0));
  double dy = std::abs(x1(1) - x2(1));
  double dz = std::abs(x1(2) - x2(2));
  double min_xyz = std::min({dx, dy, dz});
  double h = dx + dy + dz + (std::sqrt(3) - 3) * min_xyz;
  return tie_breaker_ * h;
}

inline void AStar::ComputeCost(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) {
  double new_g_score = current->g_score + d_pos.norm();
  double new_f_score = new_g_score + lambda_heuristic_ * getEuclHeu(neighbor->position, target);

  neighbor->parent = current;
  neighbor->g_score = new_g_score;
  neighbor->f_score = new_f_score;
}

inline void AStar::UpdateVertex(NodePtr current, NodePtr neighbor, const Eigen::Vector3d& d_pos, const Eigen::Vector3d& target) {
  double old_g_score = neighbor->g_score;

  ComputeCost(current, neighbor, d_pos, target);

  if (neighbor->g_score < old_g_score) {
    if (neighbor->node_state == IN_OPEN_SET) {
      open_set_.insert(neighbor);
    } else {
      neighbor->node_state = IN_OPEN_SET;
      open_set_.insert(neighbor);
    }
  }
}

int AStar::findPath(Eigen::Vector3d _source,
                    Eigen::Vector3d _target) {
  /* ---------- initialize ---------- */
  NodePtr start_node = path_node_pool_[0];
  start_node->parent = NULL;
  start_node->position = _source;
  start_node->g_score = 0.0;
  start_node->f_score = lambda_heuristic_ * getEuclHeu(start_node->position, _target);
  start_node->node_state = IN_OPEN_SET;

  open_set_.insert(start_node);
  use_node_num_ += 1;

  expanded_nodes_.insert(start_node->position, start_node);

  NodePtr cur_node = NULL;
  NodePtr terminate_node = NULL;

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node and pop node ---------- */
    cur_node = *open_set_.begin();
    open_set_.erase(open_set_.begin());

    /* ---------- determine termination ---------- */

    bool reach_end = abs(cur_node->position(0) - _target(0)) <= resolution_ &&
                     abs(cur_node->position(1) - _target(1)) <= resolution_ &&
                     abs(cur_node->position(2) - _target(2)) <= resolution_;

    if (reach_end) {
      terminate_node = cur_node;
      retrievePath(terminate_node);

      return REACH_END;
    }

    /* ---------- add to close set ---------- */
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */
    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d neighbor_pos;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          d_pos << dx, dy, dz;

          if (d_pos.norm() < 1e-3) continue;

          neighbor_pos = cur_pos + d_pos;
          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (!grid_map_->isInMap(neighbor_pos) || grid_map_->getInflateOccupancy(neighbor_pos)) {
            continue;
          }

          /* not in close set */
          NodePtr neighbor_node = expanded_nodes_.find(neighbor_pos);
          if (neighbor_node != NULL && neighbor_node->node_state == IN_CLOSE_SET) {
            continue;
          }

          if (neighbor_node == NULL) {
            neighbor_node = path_node_pool_[use_node_num_];
            neighbor_node->position = neighbor_pos;
            neighbor_node->g_score = std::numeric_limits<double>::infinity();

            expanded_nodes_.insert(neighbor_pos, neighbor_node);

            use_node_num_ += 1;

            if (use_node_num_ == allocate_num_) {
              std::cout << "run out of memory." << std::endl;
              return NO_PATH;
            }
          }
          UpdateVertex(cur_node, neighbor_node, d_pos, _target);
        }
  }

  /* ---------- open set empty, no path ---------- */
  std::cout << "open set empty, no path!" << std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;
  return NO_PATH;
}

std::vector<Eigen::Vector3d> AStar::getPath() {
  std::vector<Eigen::Vector3d> path;
  for (size_t i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

void AStar::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
