#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>

using std::cout;
using std::endl;

namespace {
constexpr std::uint8_t kNav2FreeSpace = 0;
constexpr std::uint8_t kNav2InscribedInflatedObstacle = 253;
constexpr std::uint8_t kNav2LethalObstacle = 254;
}  // namespace

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};



void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  auto t0 = std::chrono::high_resolution_clock::now();

  cost_threshold_ = cost_threshold;
  resolution_ = resolution;
  step_cost_weight_  = step_cost_weight;

  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;
  perception_active_cells_ = 0;
  perception_source_active_cells_ = 0;
  perception_source_indices_.clear();
  perception_cost_indices_.clear();
  global_path_perception_enabled_ = true;
  search_bounds_enabled_ = false;

  int row_offset = 0;
  grid_map_.resize(max_layers_);
  for (size_t i = 0; i < max_layers_; ++i) {
    row_offset = i * max_y_;
    grid_map_[i].resize(max_y_);
    for (size_t j = 0; j < max_y_; ++j) {
      grid_map_[i][j].resize(max_x_);
      for (size_t k = 0; k < max_x_; ++k) {
        double height = height_map(j + row_offset, k);
        double z = static_cast<int>(height / resolution);
        grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr);
        grid_map_[i][j][k].static_cost = cost_map(j + row_offset, k);
        grid_map_[i][j][k].perception_cost = 0.0;
        grid_map_[i][j][k].perception_nav2_cost = kNav2FreeSpace;
        grid_map_[i][j][k].perception_source_stamp = -1.0;
        grid_map_[i][j][k].cost = grid_map_[i][j][k].static_cost;
        grid_map_[i][j][k].height = height;
        grid_map_[i][j][k].ele = ele_map(j + row_offset, k);
        grid_map_[i][j][k].layer = i;
      }
    }
  }
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);

  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0);
  for (int i = 0; i < num_layers; ++i) {
    search_layers_offset_.emplace_back(-(i + 1));
    search_layers_offset_.emplace_back(i + 1);
  }

  printf(
      "Astar initialized, max_x: %d, max_y: %d, max_layers: %d, time elapsed: "
      "%f ms\n",
      max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

double Astar::EffectiveCost(const Node& node) const {
  if (!global_path_perception_enabled_) {
    return node.static_cost;
  }
  return std::max(node.static_cost, node.perception_cost);
}

void Astar::RefreshNodeCost(Node& node) {
  node.cost = EffectiveCost(node);
}

void Astar::SetGlobalPathPerceptionEnabled(bool enabled) {
  if (global_path_perception_enabled_ == enabled) {
    return;
  }
  global_path_perception_enabled_ = enabled;
  for (const int key : perception_cost_indices_) {
    const Eigen::Vector3i index = DecodePerceptionKey(key);
    RefreshNodeCost(grid_map_[index[0]][index[1]][index[2]]);
  }
}

bool Astar::UpdatePerceptionInflationParams(double inflation_radius,
                                            double inscribed_radius,
                                            double peak_cost,
                                            double cost_scaling_factor) {
  inflation_radius = std::max(0.0, inflation_radius);
  inscribed_radius = std::max(0.0, inscribed_radius);
  peak_cost = std::max(0.0, peak_cost);
  cost_scaling_factor = std::max(0.0, cost_scaling_factor);

  const bool changed =
      std::abs(perception_inflation_radius_ - inflation_radius) > 1.0e-9 ||
      std::abs(perception_inscribed_radius_ - inscribed_radius) > 1.0e-9 ||
      std::abs(perception_peak_cost_ - peak_cost) > 1.0e-9 ||
      std::abs(perception_cost_scaling_factor_ - cost_scaling_factor) > 1.0e-9;

  perception_inflation_radius_ = inflation_radius;
  perception_inscribed_radius_ = inscribed_radius;
  perception_peak_cost_ = peak_cost;
  perception_cost_scaling_factor_ = cost_scaling_factor;
  return changed;
}

std::uint8_t Astar::PerceptionInflationCost(
    int drow, int dcol, double inflation_radius, double inscribed_radius,
    double cost_scaling_factor) const {
  if (inflation_radius < 0.0 || resolution_ <= 0.0) {
    return kNav2FreeSpace;
  }

  const double distance =
      resolution_ * std::sqrt(static_cast<double>(drow * drow + dcol * dcol));
  if (distance > inflation_radius) {
    return kNav2FreeSpace;
  }

  if (drow == 0 && dcol == 0) {
    return kNav2LethalObstacle;
  }
  const double inscribed = std::max(0.0, inscribed_radius);
  if (distance <= inscribed) {
    return kNav2InscribedInflatedObstacle;
  }

  const double scale = std::max(0.0, cost_scaling_factor);
  const double factor = std::exp(-scale * (distance - inscribed));
  return static_cast<std::uint8_t>(
      (kNav2InscribedInflatedObstacle - 1) * factor);
}

double Astar::PctPerceptionCost(std::uint8_t nav2_cost) const {
  if (nav2_cost == kNav2FreeSpace || perception_peak_cost_ <= 0.0) {
    return 0.0;
  }
  return perception_peak_cost_ * static_cast<double>(nav2_cost) /
         static_cast<double>(kNav2LethalObstacle);
}

int Astar::PerceptionKey(int layer, int row, int col) const {
  return (layer * max_y_ + row) * max_x_ + col;
}

Eigen::Vector3i Astar::DecodePerceptionKey(int key) const {
  const int col = key % max_x_;
  const int matrix_row = key / max_x_;
  return Eigen::Vector3i(matrix_row / max_y_, matrix_row % max_y_, col);
}

bool Astar::MarkPerceptionSource(int layer, int row, int col, double stamp) {
  if (layer < 0 || layer >= max_layers_ || row < 0 || row >= max_y_ ||
      col < 0 || col >= max_x_) {
    return false;
  }
  Node& node = grid_map_[layer][row][col];
  const int key = PerceptionKey(layer, row, col);
  const bool was_inactive = perception_source_indices_.insert(key).second;
  if (was_inactive) {
    perception_source_active_cells_ += 1;
  }
  node.perception_source_stamp = stamp;
  return was_inactive;
}

bool Astar::ClearPerceptionSource(int layer, int row, int col) {
  if (layer < 0 || layer >= max_layers_ || row < 0 || row >= max_y_ ||
      col < 0 || col >= max_x_) {
    return false;
  }

  Node& node = grid_map_[layer][row][col];
  const int key = PerceptionKey(layer, row, col);
  if (perception_source_indices_.erase(key) == 0) {
    return false;
  }

  node.perception_source_stamp = -1.0;
  perception_source_active_cells_ = std::max(0, perception_source_active_cells_ - 1);
  return true;
}

int Astar::ClearPerceptionSourceCircle(const Eigen::Vector3i& center,
                              double radius) {
  if (radius < 0.0 || resolution_ <= 0.0 ||
      center[0] < 0 || center[0] >= max_layers_ ||
      center[1] < 0 || center[1] >= max_y_ ||
      center[2] < 0 || center[2] >= max_x_) {
    return 0;
  }

  const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(radius / resolution_)));
  int changed = 0;
  const int radius_sq = radius_cells * radius_cells;
  const int layer = center[0];
  for (int row = center[1] - radius_cells; row <= center[1] + radius_cells; ++row) {
    if (row < 0 || row >= max_y_) {
      continue;
    }
    for (int col = center[2] - radius_cells; col <= center[2] + radius_cells; ++col) {
      if (col < 0 || col >= max_x_) {
        continue;
      }
      const int drow = row - center[1];
      const int dcol = col - center[2];
      if (drow * drow + dcol * dcol > radius_sq) {
        continue;
      }
      if (ClearPerceptionSource(layer, row, col)) {
        changed += 1;
      }
    }
  }
  return changed;
}

int Astar::SelectPerceptionLayerForCell(int row, int col, int current_layer,
                                        double robot_height) const {
  if (row < 0 || row >= max_y_ || col < 0 || col >= max_x_) {
    return -1;
  }

  auto valid_layer = [&](int layer) {
    if (layer < 0 || layer >= max_layers_) {
      return false;
    }
    const double height = grid_map_[layer][row][col].height;
    return std::isfinite(height) && height > -99.0;
  };

  if (valid_layer(current_layer)) {
    return current_layer;
  }

  int best_layer = -1;
  double best_diff = std::numeric_limits<double>::infinity();
  for (int layer = 0; layer < max_layers_; ++layer) {
    if (!valid_layer(layer)) {
      continue;
    }
    const double diff =
        std::abs(grid_map_[layer][row][col].height - robot_height);
    if (diff < best_diff) {
      best_diff = diff;
      best_layer = layer;
    }
  }
  return best_layer;
}

std::vector<int> Astar::SelectPerceptionLayersForCell(
    int row, int col, int current_layer, double robot_height,
    double layer_height_tolerance, bool mark_all_layers) const {
  std::vector<int> layers;
  const int reference_layer =
      SelectPerceptionLayerForCell(row, col, current_layer, robot_height);
  if (reference_layer < 0) {
    return layers;
  }

  const double reference_height =
      grid_map_[reference_layer][row][col].height;
  const double tolerance = std::max(0.0, layer_height_tolerance);
  for (int layer = 0; layer < max_layers_; ++layer) {
    const double height = grid_map_[layer][row][col].height;
    if (!std::isfinite(height) || height <= -99.0) {
      continue;
    }
    if (mark_all_layers ||
        std::abs(height - reference_height) <= tolerance) {
      layers.push_back(layer);
    }
  }
  if (layers.empty()) {
    layers.push_back(reference_layer);
  }
  return layers;
}

bool Astar::IsStaticObstacleCell(int layer, int row, int col,
                                 double static_skip_cost) const {
  if (layer < 0 || layer >= max_layers_ || row < 0 || row >= max_y_ ||
      col < 0 || col >= max_x_) {
    return false;
  }
  const double static_cost = grid_map_[layer][row][col].static_cost;
  return std::isfinite(static_cost) && static_cost >= static_skip_cost;
}

std::vector<Eigen::Vector2i> Astar::BresenhamCells(int row0, int col0,
                                                   int row1, int col1) const {
  std::vector<Eigen::Vector2i> cells;
  const int dcol = std::abs(col1 - col0);
  const int drow = -std::abs(row1 - row0);
  const int step_col = col0 < col1 ? 1 : -1;
  const int step_row = row0 < row1 ? 1 : -1;
  int error = dcol + drow;
  int row = row0;
  int col = col0;

  while (true) {
    cells.emplace_back(row, col);
    if (row == row1 && col == col1) {
      break;
    }
    const int double_error = 2 * error;
    if (double_error >= drow) {
      error += drow;
      col += step_col;
    }
    if (double_error <= dcol) {
      error += dcol;
      row += step_row;
    }
  }
  return cells;
}

Eigen::MatrixXi Astar::BuildGlobalPathPerceptionMarkIndices(
    const Eigen::MatrixXi& mark_cells, const int current_layer,
    const double robot_height, const bool skip_static_obstacles,
    const double static_skip_cost, const double layer_height_tolerance,
    const bool mark_all_layers) const {
  if (mark_cells.cols() < 2 || mark_cells.rows() <= 0) {
    return Eigen::MatrixXi(0, 3);
  }

  std::vector<Eigen::Vector3i> indices;
  indices.reserve(mark_cells.rows());
  std::unordered_set<int> seen;
  seen.reserve(mark_cells.rows());

  for (int i = 0; i < mark_cells.rows(); ++i) {
    const int row = mark_cells(i, 0);
    const int col = mark_cells(i, 1);
    const std::vector<int> layers = SelectPerceptionLayersForCell(
        row, col, current_layer, robot_height, layer_height_tolerance,
        mark_all_layers);
    for (const int layer : layers) {
      if (skip_static_obstacles &&
          IsStaticObstacleCell(layer, row, col, static_skip_cost)) {
        continue;
      }
      const int key = (layer * max_y_ + row) * max_x_ + col;
      if (seen.insert(key).second) {
        indices.emplace_back(layer, row, col);
      }
    }
  }

  Eigen::MatrixXi result(indices.size(), 3);
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
    result.row(i) = indices[i].transpose();
  }
  return result;
}

Eigen::MatrixXi Astar::BuildGlobalPathPerceptionClearIndices(
    const Eigen::Vector2i& origin_cell, const Eigen::MatrixXi& endpoint_cells,
    const int current_layer, const double robot_height,
    const double layer_height_tolerance, const bool mark_all_layers) const {
  if (endpoint_cells.cols() < 3 || endpoint_cells.rows() <= 0) {
    return Eigen::MatrixXi(0, 3);
  }

  const int origin_row = origin_cell[0];
  const int origin_col = origin_cell[1];
  if (origin_row < 0 || origin_row >= max_y_ ||
      origin_col < 0 || origin_col >= max_x_) {
    return Eigen::MatrixXi(0, 3);
  }

  std::vector<Eigen::Vector3i> indices;
  std::unordered_set<int> seen;
  seen.reserve(endpoint_cells.rows() * 8);

  for (int i = 0; i < endpoint_cells.rows(); ++i) {
    const int end_row = endpoint_cells(i, 0);
    const int end_col = endpoint_cells(i, 1);
    std::vector<Eigen::Vector2i> cells =
        BresenhamCells(origin_row, origin_col, end_row, end_col);
    if (cells.empty()) {
      continue;
    }

    for (const auto& cell : cells) {
      const int row = cell[0];
      const int col = cell[1];
      const std::vector<int> layers = SelectPerceptionLayersForCell(
          row, col, current_layer, robot_height, layer_height_tolerance,
          mark_all_layers);
      for (const int layer : layers) {
        const int key = (layer * max_y_ + row) * max_x_ + col;
        if (seen.insert(key).second) {
          indices.emplace_back(layer, row, col);
        }
      }
    }
  }

  Eigen::MatrixXi result(indices.size(), 3);
  for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
    result.row(i) = indices[i].transpose();
  }
  return result;
}

int Astar::DecayGlobalPathPerceptionSources(double stamp, double persistence) {
  if (perception_source_indices_.empty()) {
    return 0;
  }

  int changed = 0;
  const bool clear_all = persistence <= 0.0;
  for (auto it = perception_source_indices_.begin();
       it != perception_source_indices_.end();) {
    const Eigen::Vector3i index = DecodePerceptionKey(*it);
    Node& node = grid_map_[index[0]][index[1]][index[2]];
    if (clear_all || stamp - node.perception_source_stamp > persistence) {
      node.perception_source_stamp = -1.0;
      it = perception_source_indices_.erase(it);
      changed += 1;
    } else {
      ++it;
    }
  }
  perception_source_active_cells_ =
      static_cast<int>(perception_source_indices_.size());
  return changed;
}

int Astar::ClearPerceptionSourcesOutside(
    const Eigen::Vector4i& window_bounds) {
  if (perception_source_indices_.empty() || window_bounds[0] < 0) {
    return 0;
  }
  const int min_row = std::max(0, window_bounds[0]);
  const int max_row = std::min(max_y_ - 1, window_bounds[1]);
  const int min_col = std::max(0, window_bounds[2]);
  const int max_col = std::min(max_x_ - 1, window_bounds[3]);
  int changed = 0;
  for (auto it = perception_source_indices_.begin();
       it != perception_source_indices_.end();) {
    const Eigen::Vector3i index = DecodePerceptionKey(*it);
    if (index[1] < min_row || index[1] > max_row ||
        index[2] < min_col || index[2] > max_col) {
      grid_map_[index[0]][index[1]][index[2]].perception_source_stamp = -1.0;
      it = perception_source_indices_.erase(it);
      changed += 1;
    } else {
      ++it;
    }
  }
  perception_source_active_cells_ =
      static_cast<int>(perception_source_indices_.size());
  return changed;
}

int Astar::RebuildGlobalPathPerceptionCosts() {
  std::unordered_map<int, std::uint8_t> previous_costs;
  previous_costs.reserve(perception_cost_indices_.size());
  for (const int key : perception_cost_indices_) {
    const Eigen::Vector3i index = DecodePerceptionKey(key);
    Node& node = grid_map_[index[0]][index[1]][index[2]];
    previous_costs.emplace(key, node.perception_nav2_cost);
    node.perception_nav2_cost = kNav2FreeSpace;
    node.perception_cost = 0.0;
    RefreshNodeCost(node);
  }
  perception_cost_indices_.clear();

  if (!perception_source_indices_.empty() && resolution_ > 0.0 &&
      perception_peak_cost_ > 0.0) {
    const int radius = std::max(
        0, static_cast<int>(std::ceil(perception_inflation_radius_ /
                                      resolution_)));
    const int radius_sq = radius * radius;
    for (const int source_key : perception_source_indices_) {
      const Eigen::Vector3i source_index = DecodePerceptionKey(source_key);
      const int layer = source_index[0];
      const int source_row = source_index[1];
      const int source_col = source_index[2];
      for (int row = source_row - radius; row <= source_row + radius; ++row) {
        if (row < 0 || row >= max_y_) {
          continue;
        }
        for (int col = source_col - radius; col <= source_col + radius; ++col) {
          if (col < 0 || col >= max_x_) {
            continue;
          }
          const int drow = row - source_row;
          const int dcol = col - source_col;
          if (drow * drow + dcol * dcol > radius_sq) {
            continue;
          }
          const std::uint8_t nav2_cost = PerceptionInflationCost(
              drow, dcol, perception_inflation_radius_,
              perception_inscribed_radius_, perception_cost_scaling_factor_);
          if (nav2_cost == kNav2FreeSpace) {
            continue;
          }
          Node& node = grid_map_[layer][row][col];
          if (nav2_cost > node.perception_nav2_cost) {
            node.perception_nav2_cost = nav2_cost;
            node.perception_cost = PctPerceptionCost(nav2_cost);
          }
          perception_cost_indices_.insert(PerceptionKey(layer, row, col));
        }
      }
    }
  }

  int changed = 0;
  for (const int key : perception_cost_indices_) {
    const Eigen::Vector3i index = DecodePerceptionKey(key);
    Node& node = grid_map_[index[0]][index[1]][index[2]];
    const auto previous = previous_costs.find(key);
    const std::uint8_t previous_cost =
        previous == previous_costs.end() ? kNav2FreeSpace : previous->second;
    if (node.perception_nav2_cost != previous_cost) {
      changed += 1;
    }
    previous_costs.erase(key);
    RefreshNodeCost(node);
  }
  changed += static_cast<int>(previous_costs.size());
  perception_active_cells_ = static_cast<int>(perception_cost_indices_.size());
  return changed;
}

int Astar::DecayGlobalPathPerception(const double stamp,
                                 const double persistence) {
  const int changed_sources =
      DecayGlobalPathPerceptionSources(stamp, persistence);
  if (changed_sources <= 0) {
    return 0;
  }
  return RebuildGlobalPathPerceptionCosts();
}

int Astar::ClearGlobalPathPerceptionIndices(const Eigen::MatrixXi& clear_indices) {
  if (perception_source_active_cells_ <= 0 || clear_indices.cols() < 3) {
    return 0;
  }

  int changed_sources = 0;
  for (int i = 0; i < clear_indices.rows(); ++i) {
    if (ClearPerceptionSource(clear_indices(i, 0), clear_indices(i, 1),
                              clear_indices(i, 2))) {
      changed_sources += 1;
    }
  }
  if (changed_sources <= 0) {
    return 0;
  }
  return RebuildGlobalPathPerceptionCosts();
}

void Astar::ClearGlobalPathPerception() {
  if (perception_cost_indices_.empty() && perception_source_indices_.empty()) {
    return;
  }

  for (const int key : perception_cost_indices_) {
    const Eigen::Vector3i index = DecodePerceptionKey(key);
    Node& node = grid_map_[index[0]][index[1]][index[2]];
    node.perception_nav2_cost = kNav2FreeSpace;
    node.perception_cost = 0.0;
    RefreshNodeCost(node);
  }
  for (const int key : perception_source_indices_) {
    const Eigen::Vector3i index = DecodePerceptionKey(key);
    grid_map_[index[0]][index[1]][index[2]].perception_source_stamp = -1.0;
  }
  perception_cost_indices_.clear();
  perception_source_indices_.clear();
  perception_active_cells_ = 0;
  perception_source_active_cells_ = 0;
}

int Astar::UpdateGlobalPathPerception(const Eigen::MatrixXi& perception_indices,
                                  const double inflation_radius,
                                  const double inscribed_radius,
                                  const double peak_cost,
                                  const double cost_scaling_factor,
                                  const double stamp,
                                  const double persistence,
                                  const Eigen::Vector3i& clear_center,
                                  const double clear_radius) {
  return ApplyGlobalPathPerception(
      perception_indices, Eigen::MatrixXi(0, 3), inflation_radius,
      inscribed_radius, peak_cost, cost_scaling_factor, stamp, persistence,
      clear_center, clear_radius, Eigen::Vector4i(-1, -1, -1, -1));
}

int Astar::ApplyGlobalPathPerception(
    const Eigen::MatrixXi& mark_indices,
    const Eigen::MatrixXi& clear_indices,
    const double inflation_radius,
    const double inscribed_radius,
    const double peak_cost,
    const double cost_scaling_factor,
    const double stamp,
    const double persistence,
    const Eigen::Vector3i& clear_center,
    const double clear_radius,
    const Eigen::Vector4i& window_bounds) {
  const bool params_changed = UpdatePerceptionInflationParams(
      inflation_radius, inscribed_radius, peak_cost, cost_scaling_factor);
  int changed_sources = DecayGlobalPathPerceptionSources(stamp, persistence);
  changed_sources += ClearPerceptionSourcesOutside(window_bounds);
  if (clear_indices.cols() >= 3) {
    for (int i = 0; i < clear_indices.rows(); ++i) {
      if (ClearPerceptionSource(clear_indices(i, 0), clear_indices(i, 1),
                                clear_indices(i, 2))) {
        changed_sources += 1;
      }
    }
  }
  if (clear_radius >= 0.0) {
    changed_sources += ClearPerceptionSourceCircle(clear_center, clear_radius);
  }

  for (int i = 0; i < mark_indices.rows(); ++i) {
    const int layer = mark_indices(i, 0);
    const int center_row = mark_indices(i, 1);
    const int center_col = mark_indices(i, 2);
    if (MarkPerceptionSource(layer, center_row, center_col, stamp)) {
      changed_sources += 1;
    }
  }

  if (changed_sources <= 0 && !params_changed) {
    return 0;
  }
  return RebuildGlobalPathPerceptionCosts();
}

void Astar::SetSearchBounds(const Eigen::Vector4i& bounds) {
  search_min_row_ = std::max(0, bounds[0]);
  search_max_row_ = std::min(max_y_ - 1, bounds[1]);
  search_min_col_ = std::max(0, bounds[2]);
  search_max_col_ = std::min(max_x_ - 1, bounds[3]);
  search_bounds_enabled_ =
      search_min_row_ <= search_max_row_ && search_min_col_ <= search_max_col_;
}

bool Astar::IsInsideSearchBounds(int row, int col) const {
  return !search_bounds_enabled_ ||
         (row >= search_min_row_ && row <= search_max_row_ &&
          col >= search_min_col_ && col <= search_max_col_);
}

bool Astar::IsGlobalPathPerceptionLethal(
    const Eigen::Vector3i& index) const {
  if (!global_path_perception_enabled_ ||
      index[0] < 0 || index[0] >= max_layers_ ||
      index[1] < 0 || index[1] >= max_y_ ||
      index[2] < 0 || index[2] >= max_x_) {
    return false;
  }
  return grid_map_[index[0]][index[1]][index[2]].perception_nav2_cost >=
         kNav2InscribedInflatedObstacle;
}

bool Astar::HasLethalGlobalPathPerception(
    const Eigen::MatrixXi& indices) const {
  if (indices.cols() < 3) {
    return false;
  }
  for (int i = 0; i < indices.rows(); ++i) {
    if (IsGlobalPathPerceptionLethal(indices.row(i).transpose())) {
      return true;
    }
  }
  return false;
}

void Astar::Reset() {
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
        grid_map_[i][j][k].Reset();
      }
    }
  }
}

int Astar::GetHash(const Eigen::Vector3i& idx) const {
  return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}



// 判断坐标是否在地图合法边界内（图层、y轴、x轴）
bool Astar::IsValidNodeCoord(int layer, int y, int x) const {
    // 校验图层边界（0 <= layer < 最大图层数）
    if (layer < 0 || layer >= max_layers_) {
        std::cerr << "Error: 图层越界，合法范围[0, " << max_layers_ - 1 << "]，输入值：" << layer << std::endl;
        return false;
    }
    // 校验y轴边界（对应grid_map_[layer][y][x]的第二个维度）
    if (y < 0 || y >= max_y_) {
        std::cerr << "Error: y轴坐标越界，合法范围[0, " << max_y_ - 1 << "]，输入值：" << y << std::endl;
        return false;
    }
    // 校验x轴边界（对应grid_map_[layer][y][x]的第三个维度）
    if (x < 0 || x >= max_x_) {
        std::cerr << "Error: x轴坐标越界，合法范围[0, " << max_x_ - 1 << "]，输入值：" << x << std::endl;
        return false;
    }
    // 所有坐标均合法
    return true;
}


bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();

  if (!search_result_.empty()) {
    Reset();
    search_result_.clear();
  }
  
  //检测是否起始点和终点都符合要求
  if (!IsValidNodeCoord(start[0],start[2],start[1]) || !IsValidNodeCoord(goal[0],goal[2],goal[1]))
  {
    return false;
  }
  if (!IsInsideSearchBounds(start[2], start[1]) ||
      !IsInsideSearchBounds(goal[2], goal[1])) {
    printf("start or goal is outside local search bounds\n");
    return false;
  }
  
  auto start_node = &grid_map_[start[0]][start[2]][start[1]];
  auto goal_node = &grid_map_[goal[0]][goal[2]][goal[1]];
  start_node->g = 0.0;

  if (goal_node->perception_nav2_cost >=
          kNav2InscribedInflatedObstacle &&
      global_path_perception_enabled_) {
    printf("goal node is occupied by a dynamic obstacle\n");
    return false;
  }
  if (goal_node->cost > cost_threshold_) {
    printf("goal node is not reachable, cost: %f, layer: %d\n", goal_node->cost, goal_node->layer);
    return false;
  }
  // 优先队列（open_set）：按节点的f值（g+h）从小到大排序（通过NodeCompare实现）
  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
  // 哈希表（closed_set）：存储已处理的节点（避免重复访问）
  std::unordered_map<int, Node*> closed_set;

  open_set.push(start_node);  // 起点加入开放集

  printf("start searching\n");

  Node* best_node = start_node;
  while (!open_set.empty()) {
    // 步骤1：从开放集取出 f 值最小的节点（当前节点）
    Node* current_node = open_set.top();
    open_set.pop();

    // 步骤2：判断是否到达终点
    if (current_node->idx == goal_node->idx) {
      while (current_node->parent != nullptr) {
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("path found, time elapsed: %f ms\n",
             duration.count() / 1000.0);
      return true;
    }

    // 步骤3：将当前节点加入封闭集（标记为已访问）
    closed_set[GetHash(current_node->idx)] = current_node;
    // 步骤4：确定当前节点所在的图层（基于地形特征动态调整）
    int layer = DecideLayer(current_node);
    // 步骤5：遍历所有邻居节点（8方向+对角线，共9个邻居）
    int i, j = 0;
    double tentative_g = 0.0;
    // 遍历2D邻域（8个方向+中心？实际是8个方向，见kNeighbors定义）
    int idx=0;
    for (const auto& neighbor : kNeighbors) {
      // 计算邻居节点的坐标（i为行，j为列）
      idx++;
      i = current_node->idx[1] + neighbor[0];//向xy方向添加节点
      j = current_node->idx[2] + neighbor[1];

          // 检查坐标是否超出地图边界（越界则跳过）
      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_ ||
          !IsInsideSearchBounds(i, j)) {
        continue;
      }
      // 获取邻居节点（基于当前层layer），这个节点在grid_map中会被赋予权重，即costmap的cost
      auto neighbor_node = &grid_map_[layer][i][j];

      // 检查邻居节点是否可通行
      if (global_path_perception_enabled_ &&
          neighbor_node->perception_nav2_cost >=
              kNav2InscribedInflatedObstacle) {
        continue;
      }
      if (neighbor_node->cost > cost_threshold_) {
        // 若成本超过阈值，但ele值较大（可能是特殊地形），进一步检查高度差
        if (abs(neighbor_node->ele) < 0.5 ) {
          continue;// ele值小，直接视为不可通行
        } else {
          // ele值大，但高度差超过0.3也视为不可通行
          if (std::abs(neighbor_node->height - current_node->height) > 0.3) {
            continue;
          }
        }
      }       
      // 计算当前节点到邻居节点的代价
      auto diff = neighbor_node->idx - current_node->idx;// 坐标差      
      double step_cost = step_cost_weight_ * neighbor_node->cost;


      double diff_cost=std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);

      // 总代价 = 当前节点g值 + 欧氏距离（坐标差的3D距离,用于靠近下一个点） + 步骤成本（远离障碍物）
      tentative_g = current_node->g + diff_cost +step_cost;
      

      // 检查邻居是否已在closed_set中
      auto p_neighbor = closed_set.find(GetHash(neighbor_node->idx));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue; // 若新成本更高，无需更新
        }
      }
      // 若新成本更低，更新邻居节点的g、f值和父节点，并加入open_set，未探索过的邻居点一开始的权重是非常高的1e+9
      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;// 相当于更新上一个节点的成本
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);// f = g + 启发式成本h
        neighbor_node->parent = current_node;// 记录父节点（用于回溯路径）
        open_set.push(neighbor_node);// 加入开放集等待探索
      }
    }  

    if (open_set.empty())
    {
      /* code */
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
    }
  }

// 若open_set为空仍未找到终点，说明无路径
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("path not found\n, time elapsed: %f ms\n",
         duration.count() / 1000.0);
  if (debug_) {
    ConvertClosedSetToMatrix(closed_set);// 调试模式下记录访问过的节点
  }
  return false;
}


int Astar::DecideLayer(const Node* cur_node) const {
  int original_layer = cur_node->layer;  // 初始图层（默认返回值）
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int best_layer = original_layer;
  double min_cost = grid_map_[original_layer][i][j].cost;  // 初始化为当前图层的代价

  // 遍历多层偏移（扩大图层搜索范围）
  for (const auto offset : search_layers_offset_) {
    int cur_layer = original_layer + offset;

    // 检查图层是否越界
    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    // 获取当前搜索节点
    // const Node& search_node = grid_map_[cur_layer][ni][nj];
    const Node& search_node = grid_map_[cur_layer][i][j];

    // 筛选：高度差需在合理范围内（避免高度突变）
    if (std::abs(search_node.height - cur_height) > 0.2) {
      continue;
    //便免越界得到空的值
    }else if (search_node.cost > cost_threshold_ || search_node.cost <=0) {
      continue;
    }

    // 更新最小代价图层
    if (search_node.cost < min_cost) {
      min_cost = search_node.cost;
      best_layer = cur_layer;
    }
  }

  // 若未找到更优图层（如所有邻居都越界或高度差过大），返回原始图层
  return best_layer;
}



double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  double cost = 0.0;

  if (h_type_ == kEuclidean) {
    // l2 distance
    cost = (node1->idx - node2->idx).norm();
  } else if (h_type_ == kDiagonal) {
    // octile distance
    Eigen::Vector3i d = node1->idx - node2->idx;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = std::min(dx, std::min(dy, dz));
    int dmax = std::max(dx, std::max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    double h =
        std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
    cost = h;
  } else if (h_type_ == kManhattan) {
    cost = (node1->idx - node2->idx).lpNorm<1>();
  } else {
    assert(false && "not implemented");
  }

  // cost += std::abs(node1->idx[0] - node2->idx[0]) * 10;
  return cost;
}


std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<int, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {

}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = grid_map_[layer][i][j].cost;
    }
  }
  return cost_layer;
}

Eigen::MatrixXd Astar::GetPerceptionCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = grid_map_[layer][i][j].perception_cost;
    }
  }
  return cost_layer;
}

Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      ele_layer(i, j) = grid_map_[layer][i][j].ele;
    }
  }
  return ele_layer;
}
