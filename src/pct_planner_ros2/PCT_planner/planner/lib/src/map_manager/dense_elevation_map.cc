#include "map_manager/dense_elevation_map.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

namespace {
constexpr std::uint8_t kNav2FreeSpace = 0;
constexpr std::uint8_t kNav2InscribedInflatedObstacle = 253;
constexpr std::uint8_t kNav2LethalObstacle = 254;
}  // namespace

void DenseElevationMap::Init(const double resolution, const int num_layers,
                             const Eigen::MatrixXd& cost_map,
                             const Eigen::MatrixXd& ele_mask,
                             const Eigen::MatrixXd& height,
                             const Eigen::MatrixXd& ceiling,
                             const Eigen::MatrixXd& grad_x,
                             const Eigen::MatrixXd& grad_y) {
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution;
  max_layers_ = num_layers;
  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  xy_size_ = max_x_ * max_y_;
  cost_ = cost_map;
  perception_cost_ = Eigen::MatrixXd::Zero(cost_map.rows(), cost_map.cols());
  perception_nav2_cost_ =
      Eigen::MatrixXi::Zero(cost_map.rows(), cost_map.cols());
  perception_source_stamp_ =
      Eigen::MatrixXd::Constant(cost_map.rows(), cost_map.cols(), -1.0);
  perception_active_cells_ = 0;
  perception_source_active_cells_ = 0;
  perception_source_indices_.clear();
  perception_cost_indices_.clear();
  global_path_perception_enabled_ = true;
  ele_mask_ = ele_mask;
  height_ = height;
  ceiling_ = ceiling;
  grad_x_ = grad_x;
  grad_y_ = grad_y;

  printf("max layers: %d, max_x: %d, max_y: %d\n", max_layers_, max_x_, max_y_);
}

double DenseElevationMap::EffectiveCost(int row, int col) const {
  if (!global_path_perception_enabled_ || perception_cost_.size() == 0) {
    return cost_(row, col);
  }
  return std::max(cost_(row, col), perception_cost_(row, col));
}

Eigen::Vector2d DenseElevationMap::EffectiveGradient(int row, int col) const {
  if (!global_path_perception_enabled_ ||
      perception_cost_(row, col) <= cost_(row, col)) {
    return Eigen::Vector2d(grad_x_(row, col), grad_y_(row, col));
  }
  const int layer_row = row % max_y_;
  const int layer_offset = row - layer_row;
  const int row_lo = layer_offset + std::max(0, layer_row - 1);
  const int row_hi = layer_offset + std::min(max_y_ - 1, layer_row + 1);
  const int col_lo = std::max(0, col - 1);
  const int col_hi = std::min(max_x_ - 1, col + 1);
  const double dx_den = std::max(1, col_hi - col_lo);
  const double dy_den = std::max(1, row_hi - row_lo);
  const double grad_x =
      (EffectiveCost(row, col_hi) - EffectiveCost(row, col_lo)) / dx_den;
  const double grad_y =
      (EffectiveCost(row_hi, col) - EffectiveCost(row_lo, col)) / dy_den;
  return Eigen::Vector2d(grad_x, grad_y);
}

bool DenseElevationMap::UpdatePerceptionInflationParams(
    double inflation_radius, double inscribed_radius, double peak_cost,
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

std::uint8_t DenseElevationMap::PerceptionInflationCost(
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

double DenseElevationMap::PctPerceptionCost(std::uint8_t nav2_cost) const {
  if (nav2_cost == kNav2FreeSpace || perception_peak_cost_ <= 0.0) {
    return 0.0;
  }
  return perception_peak_cost_ * static_cast<double>(nav2_cost) /
         static_cast<double>(kNav2LethalObstacle);
}

int DenseElevationMap::PerceptionKey(int row, int col) const {
  return row * max_x_ + col;
}

Eigen::Vector2i DenseElevationMap::DecodePerceptionKey(int key) const {
  return Eigen::Vector2i(key / max_x_, key % max_x_);
}

bool DenseElevationMap::MarkPerceptionSource(int row, int col, double stamp) {
  if (row < 0 || row >= perception_source_stamp_.rows() || col < 0 ||
      col >= perception_source_stamp_.cols()) {
    return false;
  }
  const int key = PerceptionKey(row, col);
  const bool was_inactive = perception_source_indices_.insert(key).second;
  if (was_inactive) {
    perception_source_active_cells_ += 1;
  }
  perception_source_stamp_(row, col) = stamp;
  return was_inactive;
}

bool DenseElevationMap::ClearPerceptionSource(int row, int col) {
  if (row < 0 || row >= perception_source_stamp_.rows() || col < 0 ||
      col >= perception_source_stamp_.cols()) {
    return false;
  }
  const int key = PerceptionKey(row, col);
  if (perception_source_indices_.erase(key) == 0) {
    return false;
  }

  perception_source_stamp_(row, col) = -1.0;
  perception_source_active_cells_ =
      std::max(0, perception_source_active_cells_ - 1);
  return true;
}

int DenseElevationMap::ClearPerceptionSourceCircle(const Eigen::Vector3i& center,
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
      const int matrix_row = row + layer * max_y_;
      if (ClearPerceptionSource(matrix_row, col)) {
        changed += 1;
      }
    }
  }
  return changed;
}

int DenseElevationMap::DecayGlobalPathPerceptionSources(
    double stamp, double persistence) {
  if (perception_source_indices_.empty()) {
    return 0;
  }

  int changed = 0;
  const bool clear_all = persistence <= 0.0;
  for (auto it = perception_source_indices_.begin();
       it != perception_source_indices_.end();) {
    const Eigen::Vector2i index = DecodePerceptionKey(*it);
    if (clear_all ||
        stamp - perception_source_stamp_(index[0], index[1]) > persistence) {
      perception_source_stamp_(index[0], index[1]) = -1.0;
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

int DenseElevationMap::ClearPerceptionSourcesOutside(
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
    const Eigen::Vector2i matrix_index = DecodePerceptionKey(*it);
    const int row = matrix_index[0] % max_y_;
    const int col = matrix_index[1];
    if (row < min_row || row > max_row || col < min_col || col > max_col) {
      perception_source_stamp_(matrix_index[0], col) = -1.0;
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

int DenseElevationMap::RebuildGlobalPathPerceptionCosts() {
  if (perception_cost_.size() == 0) {
    return 0;
  }

  std::unordered_map<int, int> previous_costs;
  previous_costs.reserve(perception_cost_indices_.size());
  for (const int key : perception_cost_indices_) {
    const Eigen::Vector2i index = DecodePerceptionKey(key);
    previous_costs.emplace(key, perception_nav2_cost_(index[0], index[1]));
    perception_nav2_cost_(index[0], index[1]) = kNav2FreeSpace;
    perception_cost_(index[0], index[1]) = 0.0;
  }
  perception_cost_indices_.clear();

  if (!perception_source_indices_.empty() && resolution_ > 0.0 &&
      perception_peak_cost_ > 0.0) {
    const int radius = std::max(
        0, static_cast<int>(std::ceil(perception_inflation_radius_ /
                                      resolution_)));
    const int radius_sq = radius * radius;
    for (const int source_key : perception_source_indices_) {
      const Eigen::Vector2i source_index = DecodePerceptionKey(source_key);
      const int matrix_source_row = source_index[0];
      const int layer = matrix_source_row / max_y_;
      const int source_row = matrix_source_row % max_y_;
      const int source_col = source_index[1];
      for (int row = source_row - radius; row <= source_row + radius; ++row) {
        if (row < 0 || row >= max_y_) {
          continue;
        }
        const int matrix_row = row + layer * max_y_;
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
          if (nav2_cost > perception_nav2_cost_(matrix_row, col)) {
            perception_nav2_cost_(matrix_row, col) = nav2_cost;
            perception_cost_(matrix_row, col) = PctPerceptionCost(nav2_cost);
          }
          perception_cost_indices_.insert(PerceptionKey(matrix_row, col));
        }
      }
    }
  }

  int changed = 0;
  for (const int key : perception_cost_indices_) {
    const Eigen::Vector2i index = DecodePerceptionKey(key);
    const auto previous = previous_costs.find(key);
    const int previous_cost =
        previous == previous_costs.end() ? kNav2FreeSpace : previous->second;
    if (perception_nav2_cost_(index[0], index[1]) != previous_cost) {
      changed += 1;
    }
    previous_costs.erase(key);
  }
  changed += static_cast<int>(previous_costs.size());
  perception_active_cells_ = static_cast<int>(perception_cost_indices_.size());
  return changed;
}

int DenseElevationMap::DecayGlobalPathPerception(const double stamp,
                                             const double persistence) {
  const int changed_sources =
      DecayGlobalPathPerceptionSources(stamp, persistence);
  if (changed_sources <= 0) {
    return 0;
  }
  return RebuildGlobalPathPerceptionCosts();
}

int DenseElevationMap::ClearGlobalPathPerceptionIndices(
    const Eigen::MatrixXi& clear_indices) {
  if (perception_source_active_cells_ <= 0 || perception_cost_.size() == 0 ||
      clear_indices.cols() < 3) {
    return 0;
  }

  int changed_sources = 0;
  for (int i = 0; i < clear_indices.rows(); ++i) {
    const int layer = clear_indices(i, 0);
    const int row = clear_indices(i, 1);
    const int col = clear_indices(i, 2);
    if (layer < 0 || layer >= max_layers_ || row < 0 || row >= max_y_ ||
        col < 0 || col >= max_x_) {
      continue;
    }
    if (ClearPerceptionSource(row + layer * max_y_, col)) {
      changed_sources += 1;
    }
  }
  if (changed_sources <= 0) {
    return 0;
  }
  return RebuildGlobalPathPerceptionCosts();
}

void DenseElevationMap::ClearGlobalPathPerception() {
  if (perception_cost_indices_.empty() && perception_source_indices_.empty()) {
    return;
  }
  for (const int key : perception_cost_indices_) {
    const Eigen::Vector2i index = DecodePerceptionKey(key);
    perception_cost_(index[0], index[1]) = 0.0;
    perception_nav2_cost_(index[0], index[1]) = kNav2FreeSpace;
  }
  for (const int key : perception_source_indices_) {
    const Eigen::Vector2i index = DecodePerceptionKey(key);
    perception_source_stamp_(index[0], index[1]) = -1.0;
  }
  perception_cost_indices_.clear();
  perception_source_indices_.clear();
  perception_active_cells_ = 0;
  perception_source_active_cells_ = 0;
}

int DenseElevationMap::UpdateGlobalPathPerception(
    const Eigen::MatrixXi& perception_indices, const double inflation_radius,
    const double inscribed_radius, const double peak_cost,
    const double cost_scaling_factor, const double stamp,
    const double persistence, const Eigen::Vector3i& clear_center,
    const double clear_radius) {
  return ApplyGlobalPathPerception(
      perception_indices, Eigen::MatrixXi(0, 3), inflation_radius,
      inscribed_radius, peak_cost, cost_scaling_factor, stamp, persistence,
      clear_center, clear_radius, Eigen::Vector4i(-1, -1, -1, -1));
}

int DenseElevationMap::ApplyGlobalPathPerception(
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
      const int layer = clear_indices(i, 0);
      const int row = clear_indices(i, 1);
      const int col = clear_indices(i, 2);
      if (layer >= 0 && layer < max_layers_ &&
          row >= 0 && row < max_y_ && col >= 0 && col < max_x_ &&
          ClearPerceptionSource(row + layer * max_y_, col)) {
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
    if (layer < 0 || layer >= max_layers_ ||
        center_row < 0 || center_row >= max_y_ ||
        center_col < 0 || center_col >= max_x_) {
      continue;
    }
    if (MarkPerceptionSource(center_row + layer * max_y_, center_col, stamp)) {
      changed_sources += 1;
    }
  }
  if (changed_sources <= 0 && !params_changed) {
    return 0;
  }
  return RebuildGlobalPathPerceptionCosts();
}

double DenseElevationMap::GetRealCost(int layer, double x, double y,
                                      Eigen::Vector2d* grad, int* new_layer) {
  const int col = index(x);
  int row = index(y) + layer * max_y_;
  double cost = EffectiveCost(row, col);

  // * grid that is unlikely to be the border between different layers
  if (cost < safe_cost_threshold_) {
    if (grad != nullptr) {
      *grad = EffectiveGradient(row, col);
    }
    return cost;
  }

  double ele_value = ele_mask_(row, col);
  double this_height = height_(row, col);
  int real_row = row;
  int real_layer = layer;

  if (layer > 0) {
    int lower_row = row - max_y_;
    double lower_height = height_(lower_row, col);
    if (abs(this_height - lower_height) < resolution_) {
      double lower_cost = EffectiveCost(lower_row, col);
      if (lower_cost < cost) {
        cost = lower_cost;
        real_row = lower_row;
        real_layer = layer - 1;
      }
    }
  }

  if (layer < max_layers_ - 1) {
    int upper_row = row + max_y_;
    double upper_height = height_(upper_row, col);
    if (abs(this_height - upper_height) < resolution_) {
      double upper_cost = EffectiveCost(upper_row, col);
      if (upper_cost < cost) {
        cost = upper_cost;
        real_row = upper_row;
        real_layer = layer + 1;
      }
    }
  }

  if (grad != nullptr) {
    *grad = EffectiveGradient(real_row, col);
  }
  if (new_layer != nullptr) {
    *new_layer = real_layer;
  }

  return cost;
}

double DenseElevationMap::GetRealCostSafe(int layer, double x, double y,
                                          const double height_hint) {
  int real_layer = UpdateLayerSafe(layer, x, y, height_hint);
  return EffectiveCost(index_y_safe(y) + real_layer * max_y_, index_x_safe(x));
}

int DenseElevationMap::UpdateLayer(const int layer, const double x,
                                   const double y) {
  const int col = index(x);
  int row = index(y) + layer * max_y_;
  double cost = EffectiveCost(row, col);
  int real_layer = layer;
  double lower_cost = 99;
  double upper_cost = 99;
  double lower_height = -99;
  double upper_height = -99;

  // * grid that is unlikely to be the border between different layers
  if (cost < safe_cost_threshold_) {
    if (debug_) {
      printf("cost < safe_cost_threshold_!, cost: %f\n", cost);
    }
    return layer;
  }

  double ele_value = ele_mask_(row, col);
  double this_height = height_(row, col);

  if (layer > 0) {
    int lower_row = row - max_y_;
    lower_height = height_(lower_row, col);
    if (abs(this_height - lower_height) < resolution_ || this_height < -50) {
      lower_cost = EffectiveCost(row - max_y_, col);
      if (lower_cost + offset_ < cost) {
        real_layer = layer - 1;
        cost = lower_cost;
      }
    }
  }

  if (layer < max_layers_ - 1) {
    int upper_row = row + max_y_;
    upper_height = height_(upper_row, col);
    if (abs(this_height - upper_height) < resolution_ || this_height < -50) {
      upper_cost = EffectiveCost(row + max_y_, col);
      if (upper_cost + offset_ < cost) {
        real_layer = layer + 1;
      }
    }
  }

  if (debug_) {
    // std::cout << ele_mask_ << std::endl;
    printf(
        "layer: %d, x: %f, y: %f, row: %d, col: %d, cost: %f, lower_cost: %f, "
        "upper_cost: %f, height: %f\n, lower_height: %f, upper_height: %f\n",
        layer, x, y, index(y), col, cost_(row, col), lower_cost, upper_cost,
        height_(row, col), lower_height, upper_height);
  }

  return real_layer;
}

// int DenseElevationMap::UpdateLayerSafe(const int layer, const double x,
//                                        const double y,
//                                        const double height_hint) {
//   const int col = index(x);
//   int row = index(y) + layer * max_y_;
//   double cost = cost_(row, col);
//   int real_layer = layer;
//   double lower_cost = 99;
//   double upper_cost = 99;
//   double lower_height = -1e6;
//   double upper_height = -1e6;

//   // * grid that is unlikely to be the border between different layers
//   if (cost < safe_cost_threshold_) {
//     if (debug_) {
//       printf("cost < safe_cost_threshold_!, cost: %f\n", cost);
//     }
//     return layer;
//   }

//   double this_height = height_(row, col);

//   // * judge height difference

//   if (abs(height_hint - this_height) > 5 * resolution_) {
//     // * means this grid is not valid in this layer, search up and down
//     if (layer > 0) {
//       lower_height = height_(row - max_y_, col);
//     }
//     if (layer < max_layers_ - 1) {
//       upper_height = height_(row + max_y_, col);
//     }

//     if (abs(lower_height - height_hint) < abs(upper_height - height_hint)) {
//       if (abs(height_hint - lower_height) < 2 * resolution_) {
//         real_layer = layer - 1;
//       }
//     } else {
//       if (abs(height_hint - upper_height) < 2 * resolution_) {
//         real_layer = layer + 1;
//       }
//     }

//     if (debug_) {
//       // std::cout << ele_mask_ << std::endl;
//       printf(
//           "layer: %d, x: %f, y: %f, row: %d, col: %d, cost: %f, lower_cost: "
//           "%f, "
//           "upper_cost: %f, height: %f\n, lower_height: %f, upper_height:
//           %f\n", layer, x, y, index(y), col, cost_(row, col), lower_cost,
//           upper_cost, height_(row, col), lower_height, upper_height);
//     }
//     return real_layer;
//   }

//   // * valid grid
//   if (layer > 0) {
//     int lower_row = row - max_y_;
//     lower_height = height_(lower_row, col);
//     if (abs(this_height - lower_height) < 1.5 * resolution_ ||
//         this_height < -50) {
//       lower_cost = cost_(row - max_y_, col);
//       if (lower_cost + offset_ < cost) {
//         real_layer = layer - 1;
//         cost = lower_cost;
//       }
//     }
//   }

//   if (layer < max_layers_ - 1) {
//     int upper_row = row + max_y_;
//     upper_height = height_(upper_row, col);
//     if (abs(this_height - upper_height) < 1.5 * resolution_ ||
//         this_height < -50) {
//       upper_cost = cost_(row + max_y_, col);
//       if (upper_cost + offset_ < cost) {
//         real_layer = layer + 1;
//       }
//     }
//   }

//   if (debug_) {
//     // std::cout << ele_mask_ << std::endl;
//     printf(
//         "layer: %d, x: %f, y: %f, row: %d, col: %d, cost: %f, lower_cost: %f,
//         " "upper_cost: %f, height: %f\n, lower_height: %f, upper_height:
//         %f\n", layer, x, y, index(y), col, cost_(row, col), lower_cost,
//         upper_cost, height_(row, col), lower_height, upper_height);
//   }

//   return real_layer;
// }

int DenseElevationMap::UpdateLayerSafe(const int layer, const double x,
                                       const double y,
                                       const double height_hint) {
  const int col = index_x_safe(x);
  int row = index_y_safe(y) + layer * max_y_;
  double cost = EffectiveCost(row, col);
  int real_layer = layer;
  double this_height = GetHeight(layer, x, y);
  bool unsafe_grid =
      (this_height < -50) || (abs(this_height - height_hint) > 5 * resolution_);

  // * grid that is unlikely to be the border between different layers
  if ((!unsafe_grid) && (cost < safe_cost_threshold_)) {
    if (debug_) {
      printf("cost < safe_cost_threshold_!, cost: %f, height: %f\n", cost,
             height_(row, col));
    }
    return layer;
  }

  // * for unsafe grid, both cost and height are not reliable

  double upper_height = -200;
  double lower_height = -200;
  double lower_cost = 1000;
  double upper_cost = 1000;

  double min_cost = unsafe_grid ? 1000 : cost;

  if (layer > 0) {
    lower_height = height_(row - max_y_, col);
    lower_cost = EffectiveCost(row - max_y_, col);
    // * filter out jump down scenario
    if (abs(height_hint - lower_height) < 5 * resolution_) {
      if ((abs(this_height - lower_height) < 1.5 * resolution_ &&
           lower_cost < cost) ||
          (unsafe_grid && lower_cost < 2 * safe_cost_threshold_)) {
        real_layer = layer - 1;
        min_cost = lower_cost;
      }
    }
  }

  if (layer < max_layers_ - 1) {
    upper_height = height_(row + max_y_, col);
    upper_cost = EffectiveCost(row + max_y_, col);
    if (abs(height_hint - upper_height) < 5 * resolution_) {
      if ((abs(this_height - upper_height) < 1.5 * resolution_ &&
           upper_cost < cost) ||
          (unsafe_grid && upper_cost < 2 * safe_cost_threshold_)) {
        if (upper_cost < min_cost) {
          real_layer = layer + 1;
          min_cost = upper_cost;
        }
      }
    }
  }

  if (debug_) {
    // std::cout << ele_mask_ << std::endl;
    printf(
        "layer: %d -> %d, x: %f, y: %f, row: %d, col: %d, cost: %f, "
        "lower_cost: %f, "
        "upper_cost: %f, height: %f\n, hint: %f, lower_height: %f, "
        "upper_height: %f\n",
        layer, real_layer, x, y, index(y), col, cost_(row, col), lower_cost,
        upper_cost, height_(row, col), height_hint, lower_height, upper_height);
  }

  return real_layer;
}

double DenseElevationMap::GetHeight(const int layer, const double x,
                                    const double y) {
  return height_(index_y_safe(y) + layer * max_y_, index_x_safe(x));
}

double DenseElevationMap::GetHeightSafe(const int layer, const double x,
                                        const double y,
                                        const double height_hint) {
  double new_height =
      height_(index_y_safe(y) + layer * max_y_, index_x_safe(x));

  if (abs(new_height - height_hint) > 6 * resolution_) {
    return height_hint;
  }

  return new_height;
}

double DenseElevationMap::GetCeiling(const int layer, const double x,
                                     const double y) {
  return ceiling_(index_y_safe(y) + layer * max_y_, index_x_safe(x));
}

double DenseElevationMap::GetValueBilinear(const int layer, const double x,
                                           const double y,
                                           Eigen::Vector2d* grad) {
  double x_lb = std::max(std::floor(x - 0.5), 0.0);
  double y_lb = std::max(std::floor(y - 0.5), 0.0);

  double value[2][2];
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      value[x][y] = GetRealCost(layer, x_lb + x, y_lb + y, grad);
    }
  }

  Eigen::Vector2d diff(x - x_lb, y - y_lb);

  double y0 = (1 - diff(0)) * value[0][0] + diff(0) * value[1][0];
  double y1 = (1 - diff(0)) * value[0][1] + diff(0) * value[1][1];
  double x0 = (1 - diff(1)) * value[0][0] + diff(1) * value[0][1];
  double x1 = (1 - diff(1)) * value[1][0] + diff(1) * value[1][1];

  if (grad) {
    (*grad)(0) = x1 - x0;
    (*grad)(1) = y1 - y0;
  }

  return (1 - diff(1)) * y0 + diff(1) * y1;
}

double DenseElevationMap::GetValueBilinearSafe(const int layer, const double x,
                                               const double y,
                                               const double height_hint,
                                               Eigen::Vector2d* grad) {
  double x_lb = std::max(std::floor(x - 0.5), 0.0);
  double y_lb = std::max(std::floor(y - 0.5), 0.0);

  double value[2][2];
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      value[x][y] = GetRealCostSafe(layer, x_lb + x, y_lb + y, height_hint);
    }
  }

  Eigen::Vector2d diff(x - x_lb, y - y_lb);

  double y0 = (1 - diff(0)) * value[0][0] + diff(0) * value[1][0];
  double y1 = (1 - diff(0)) * value[0][1] + diff(0) * value[1][1];
  double x0 = (1 - diff(1)) * value[0][0] + diff(1) * value[0][1];
  double x1 = (1 - diff(1)) * value[1][0] + diff(1) * value[1][1];

  if (grad) {
    (*grad)(0) = x1 - x0;
    (*grad)(1) = y1 - y0;
  }

  return (1 - diff(1)) * y0 + diff(1) * y1;
}
