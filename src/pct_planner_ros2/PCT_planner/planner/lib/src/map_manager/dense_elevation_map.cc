#include "map_manager/dense_elevation_map.h"

#include <algorithm>
#include <cmath>
#include <iostream>

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
  perception_stamp_ = Eigen::MatrixXd::Constant(cost_map.rows(), cost_map.cols(), -1.0);
  perception_active_cells_ = 0;
  ele_mask_ = ele_mask;
  height_ = height;
  ceiling_ = ceiling;
  grad_x_ = grad_x;
  grad_y_ = grad_y;

  printf("max layers: %d, max_x: %d, max_y: %d\n", max_layers_, max_x_, max_y_);
}

double DenseElevationMap::EffectiveCost(int row, int col) const {
  if (perception_cost_.size() == 0) {
    return cost_(row, col);
  }
  return std::max(cost_(row, col), perception_cost_(row, col));
}

double DenseElevationMap::PerceptionInflationCost(int drow, int dcol,
                                               double inflation_radius,
                                               double inscribed_radius,
                                               double peak_cost,
                                               double cost_scaling_factor) const {
  if (peak_cost <= 0.0 || inflation_radius < 0.0 || resolution_ <= 0.0) {
    return 0.0;
  }

  const double distance =
      resolution_ * std::sqrt(static_cast<double>(drow * drow + dcol * dcol));
  if (distance > inflation_radius) {
    return 0.0;
  }

  const double lethal_cost = std::max(0.0, peak_cost);
  const double inscribed =
      std::max(0.0, std::min(inscribed_radius, inflation_radius));
  if (distance <= inscribed) {
    return lethal_cost;
  }

  const double scale = std::max(0.0, cost_scaling_factor);
  if (scale <= 0.0) {
    return lethal_cost;
  }
  return lethal_cost * std::exp(-scale * (distance - inscribed));
}

void DenseElevationMap::SetPerceptionCost(int row, int col, double cost,
                                       double stamp) {
  if (row < 0 || row >= perception_cost_.rows() || col < 0 ||
      col >= perception_cost_.cols()) {
    return;
  }
  if (cost <= 0.0) {
    return;
  }
  if (perception_cost_(row, col) <= 0.0) {
    perception_active_cells_ += 1;
  }
  perception_cost_(row, col) = std::max(perception_cost_(row, col), cost);
  perception_stamp_(row, col) = stamp;
}

int DenseElevationMap::ClearPerceptionCircle(const Eigen::Vector3i& center,
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
      if (perception_cost_(matrix_row, col) > 0.0) {
        perception_cost_(matrix_row, col) = 0.0;
        perception_stamp_(matrix_row, col) = -1.0;
        perception_active_cells_ = std::max(0, perception_active_cells_ - 1);
        changed += 1;
      }
    }
  }
  return changed;
}

int DenseElevationMap::DecayGlobalPathPerception(const double stamp,
                                             const double persistence) {
  if (perception_active_cells_ <= 0 || perception_cost_.size() == 0) {
    return 0;
  }

  int changed = 0;
  const bool clear_all = persistence <= 0.0;
  for (int row = 0; row < perception_cost_.rows(); ++row) {
    for (int col = 0; col < perception_cost_.cols(); ++col) {
      if (perception_cost_(row, col) <= 0.0) {
        continue;
      }
      if (clear_all || stamp - perception_stamp_(row, col) > persistence) {
        perception_cost_(row, col) = 0.0;
        perception_stamp_(row, col) = -1.0;
        perception_active_cells_ = std::max(0, perception_active_cells_ - 1);
        changed += 1;
      }
    }
  }
  return changed;
}

void DenseElevationMap::ClearGlobalPathPerception() {
  if (perception_cost_.size() == 0) {
    return;
  }
  perception_cost_.setZero();
  perception_stamp_.setConstant(-1.0);
  perception_active_cells_ = 0;
}

int DenseElevationMap::UpdateGlobalPathPerception(
    const Eigen::MatrixXi& perception_indices, const double inflation_radius,
    const double inscribed_radius, const double peak_cost,
    const double cost_scaling_factor, const double stamp,
    const double persistence, const Eigen::Vector3i& clear_center,
    const double clear_radius) {
  int changed = DecayGlobalPathPerception(stamp, persistence);
  if (resolution_ <= 0.0) {
    return changed;
  }
  if (clear_radius >= 0.0) {
    changed += ClearPerceptionCircle(clear_center, clear_radius);
  }

  const int radius = std::max(
      0, static_cast<int>(std::ceil(std::max(0.0, inflation_radius) /
                                    resolution_)));
  const int radius_sq = radius * radius;
  for (int i = 0; i < perception_indices.rows(); ++i) {
    const int layer = perception_indices(i, 0);
    const int center_row = perception_indices(i, 1);
    const int center_col = perception_indices(i, 2);
    if (layer < 0 || layer >= max_layers_ ||
        center_row < 0 || center_row >= max_y_ ||
        center_col < 0 || center_col >= max_x_) {
      continue;
    }
    for (int row = center_row - radius; row <= center_row + radius; ++row) {
      if (row < 0 || row >= max_y_) {
        continue;
      }
      for (int col = center_col - radius; col <= center_col + radius; ++col) {
        if (col < 0 || col >= max_x_) {
          continue;
        }
        const int drow = row - center_row;
        const int dcol = col - center_col;
        if (drow * drow + dcol * dcol > radius_sq) {
          continue;
        }
        const int matrix_row = row + layer * max_y_;
        const double before = perception_cost_(matrix_row, col);
        const double cell_cost = PerceptionInflationCost(
            drow, dcol, inflation_radius, inscribed_radius, peak_cost,
            cost_scaling_factor);
        SetPerceptionCost(matrix_row, col, cell_cost, stamp);
        if (perception_cost_(matrix_row, col) != before) {
          changed += 1;
        }
      }
    }
  }
  return changed;
}

double DenseElevationMap::GetRealCost(int layer, double x, double y,
                                      Eigen::Vector2d* grad, int* new_layer) {
  const int col = index(x);
  int row = index(y) + layer * max_y_;
  double cost = EffectiveCost(row, col);

  // * grid that is unlikely to be the border between different layers
  if (cost < safe_cost_threshold_) {
    if (grad != nullptr) {
      *grad = Eigen::Vector2d(grad_x_(row, col), grad_y_(row, col));
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
    *grad = Eigen::Vector2d(grad_x_(real_row, col), grad_y_(real_row, col));
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
