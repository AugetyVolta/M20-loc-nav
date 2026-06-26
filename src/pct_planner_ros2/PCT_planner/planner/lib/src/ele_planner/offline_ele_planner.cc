#include "ele_planner/offline_ele_planner.h"
#include <iostream>
#include <fstream>      // 新增：用于文件操作
#include <iomanip>      // 可选：用于格式化输出

void OfflineElePlanner::InitMap(
    const double a_start_cost_threshold, const double safe_cost_margin,
    const double resolution, const int num_layers, const double step_cost_weight,
    const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& height_map,
    const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
    const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y) {
  path_finder_.Init(a_start_cost_threshold, num_layers, resolution, step_cost_weight, cost_map,
                    height_map, ele_map);
  map_ = std::make_shared<DenseElevationMap>();
  map_->Init(resolution, num_layers, cost_map, ele_map, height_map, ceiling,
             grad_x, grad_y);
  trajectory_optimizer_ = GPMPOptimizerWnoa(safe_cost_margin, map_);
  trajectory_optimizer_wnoj_ =
      GPMPOptimizer(safe_cost_margin, max_heading_rate_, map_);
}

int OfflineElePlanner::UpdateGlobalPathPerception(
    const Eigen::MatrixXi& perception_indices, const double inflation_radius,
    const double inscribed_radius, const double peak_cost,
    const double cost_scaling_factor, const double stamp,
    const double persistence, const Eigen::Vector3i& clear_center,
    const double clear_radius) {
  int changed = path_finder_.UpdateGlobalPathPerception(
      perception_indices, inflation_radius, inscribed_radius, peak_cost,
      cost_scaling_factor, stamp, persistence, clear_center, clear_radius);
  if (map_) {
    map_->UpdateGlobalPathPerception(
        perception_indices, inflation_radius, inscribed_radius, peak_cost,
        cost_scaling_factor, stamp, persistence, clear_center, clear_radius);
  }
  return changed;
}

int OfflineElePlanner::DecayGlobalPathPerception(const double stamp,
                                             const double persistence) {
  int changed = path_finder_.DecayGlobalPathPerception(stamp, persistence);
  if (map_) {
    map_->DecayGlobalPathPerception(stamp, persistence);
  }
  return changed;
}

int OfflineElePlanner::ClearGlobalPathPerceptionIndices(
    const Eigen::MatrixXi& clear_indices) {
  int changed = path_finder_.ClearGlobalPathPerceptionIndices(clear_indices);
  if (map_) {
    map_->ClearGlobalPathPerceptionIndices(clear_indices);
  }
  return changed;
}

void OfflineElePlanner::ClearGlobalPathPerception() {
  path_finder_.ClearGlobalPathPerception();
  if (map_) {
    map_->ClearGlobalPathPerception();
  }
}

Eigen::MatrixXi OfflineElePlanner::BuildGlobalPathPerceptionMarkIndices(
    const Eigen::MatrixXi& mark_cells, const int current_layer,
    const double robot_height, const bool skip_static_obstacles,
    const double static_skip_cost) const {
  return path_finder_.BuildGlobalPathPerceptionMarkIndices(
      mark_cells, current_layer, robot_height, skip_static_obstacles,
      static_skip_cost);
}

Eigen::MatrixXi OfflineElePlanner::BuildGlobalPathPerceptionClearIndices(
    const Eigen::Vector2i& origin_cell, const Eigen::MatrixXi& endpoint_cells,
    const int current_layer, const double robot_height) const {
  return path_finder_.BuildGlobalPathPerceptionClearIndices(
      origin_cell, endpoint_cells, current_layer, robot_height);
}

bool OfflineElePlanner::Plan(const Eigen::Vector3i& start,
                             const Eigen::Vector3i& goal, const bool optimize) {

  if (!path_finder_.Search(start, goal)) {
    printf("A star Failed!\n");
    return false;
  }

  if (optimize) {
    path_ = path_finder_.GetPathPoints();

// ====================== 修改部分：写入 log 文件 ======================
    std::ofstream log_file("offline_ele_planner_path.log", std::ios::trunc);  // 追加模式

    if (log_file.is_open()) {
      for (const auto& point : path_) {
        log_file << point.x << "," 
                 << point.y << "," 
                 << point.layer << "\n";
      }
      
      log_file.close();
    } else {
      std::cerr << "Warning: Cannot open offline_ele_planner_path.log for writing!\n";
    }
    // ===================================================================
   
    path_.front().ref_v = 1;
    path_.back().ref_v = 1;

    bool success = false;
    if (use_quintic_) {
      success = trajectory_optimizer_wnoj_.GenerateTrajectory(path_, 200);
    } else {
      success = trajectory_optimizer_.GenerateTrajectory(path_, 200);
    }

    return success;
  }

  return true;
}
