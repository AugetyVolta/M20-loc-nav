#pragma once

#include <memory>

#include "a_star/a_star_search.h"
#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"

class OfflineElePlanner {
 public:
  OfflineElePlanner(const double max_heading_rate, bool use_quintic)
      : use_quintic_(use_quintic), max_heading_rate_(max_heading_rate) {}
  ~OfflineElePlanner() = default;

  void InitMap(const double a_start_cost_threshold,
               const double safe_cost_margin, const double resolution,
               const int num_layers, const double step_cost_weight, const Eigen::MatrixXd& cost_map,
               const Eigen::MatrixXd& height_map,
               const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
               const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  bool Plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal,
            const bool optimize, const double goal_heading);
  int UpdateGlobalPathPerception(const Eigen::MatrixXi& perception_indices,
                             const double inflation_radius,
                             const double inscribed_radius,
                             const double peak_cost,
                             const double cost_scaling_factor,
                             const double stamp,
                             const double persistence,
                             const Eigen::Vector3i& clear_center,
                             const double clear_radius);
  int ApplyGlobalPathPerception(
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
      const Eigen::Vector4i& window_bounds);
  int DecayGlobalPathPerception(const double stamp, const double persistence);
  int ClearGlobalPathPerceptionIndices(const Eigen::MatrixXi& clear_indices);
  void ClearGlobalPathPerception();
  int GetGlobalPathPerceptionCellCount() const {
    return path_finder_.GetGlobalPathPerceptionCellCount();
  }
  Eigen::MatrixXi BuildGlobalPathPerceptionMarkIndices(
      const Eigen::MatrixXi& mark_cells,
      const int current_layer,
      const double robot_height,
      const bool skip_static_obstacles,
      const double static_skip_cost,
      const double layer_height_tolerance,
      const bool mark_all_layers) const;
  Eigen::MatrixXi BuildGlobalPathPerceptionClearIndices(
      const Eigen::Vector2i& origin_cell,
      const Eigen::MatrixXi& endpoint_cells,
      const int current_layer,
      const double robot_height,
      const double layer_height_tolerance,
      const bool mark_all_layers) const;
  void SetGlobalPathPerceptionEnabled(bool enabled) {
    path_finder_.SetGlobalPathPerceptionEnabled(enabled);
    if (map_) {
      map_->SetGlobalPathPerceptionEnabled(enabled);
    }
  }
  void SetSearchBounds(const Eigen::Vector4i& bounds) {
    path_finder_.SetSearchBounds(bounds);
  }
  void ClearSearchBounds() { path_finder_.ClearSearchBounds(); }
  bool HasLethalGlobalPathPerception(
      const Eigen::MatrixXi& indices) const {
    return path_finder_.HasLethalGlobalPathPerception(indices);
  }

  void SetReferenceHeight(const double height) {
    trajectory_optimizer_wnoj_.SetReferenceHeight(height);
  }

  void Debug() {
    path_finder_.Debug();
    trajectory_optimizer_.SetDebug(true);
    trajectory_optimizer_wnoj_.SetDebug(true);
  }

  Eigen::MatrixXd GetDebugPath() const {
    return path_finder_.GetResultMatrix();
  }

  void set_max_iterations(int max_iterations) {
    trajectory_optimizer_.set_max_iterations(max_iterations);
  }

  const Astar& get_path_finder() const { return path_finder_; }
  const DenseElevationMap& get_map() const { return *map_; }
  const GPMPOptimizerWnoa& get_trajectory_optimizer() const {
    return trajectory_optimizer_;
  }
  const GPMPOptimizer& get_trajectory_optimizer_wnoj() const {
    return trajectory_optimizer_wnoj_;
  }

 private:
  double max_heading_rate_ = 0.5;
  bool use_quintic_ = false;

  std::shared_ptr<DenseElevationMap> map_;
  Astar path_finder_;
  GPMPOptimizerWnoa trajectory_optimizer_;
  GPMPOptimizer trajectory_optimizer_wnoj_;

  std::vector<PathPoint> path_;
  std::vector<Eigen::Vector3d> trajectory_;
};
