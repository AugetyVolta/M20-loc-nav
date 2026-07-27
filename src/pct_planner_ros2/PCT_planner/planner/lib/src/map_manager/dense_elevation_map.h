#pragma once

#include <array>
#include <Eigen/Dense>
#include <cstdint>
#include <unordered_set>

class DenseElevationMap {
 public:
  DenseElevationMap() = default;
  ~DenseElevationMap() = default;

  void Init(const double resolution, const int num_layers,
            const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& ele_mask,
            const Eigen::MatrixXd& height, const Eigen::MatrixXd& ceiling,
            const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  double GetValueBilinear(const int layer, const double x, const double y,
                          Eigen::Vector2d* grad = nullptr);

  double GetValueBilinearSafe(const int layer, const double x, const double y,
                              const double height_hint,
                              Eigen::Vector2d* grad = nullptr);

  int UpdateLayer(const int layer, const double x, const double y);

  int UpdateLayerSafe(const int layer, const double x, const double y,
                      const double height_hint);

  double GetHeight(const int layer, const double x, const double y);

  double GetHeightSafe(const int layer, const double x, const double y,
                       const double height_hint);

  double GetCeiling(const int layer, const double x, const double y);

  double inline GetNominalCost(int layer, double x, double y) {
    auto idx = CoordsToIndex(layer, x, y);
    return EffectiveCost(idx[0], idx[1]);
  };

  std::array<int, 2> inline CoordsToIndex(int layer, double x, double y) {
    int col = index(x);
    int row = index(y) + layer * max_y_;
    return {row, col};
  }

  void SetDebug(const bool flag) { debug_ = flag; }
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
  bool HasGlobalPathPerception() const { return perception_active_cells_ > 0; }
  int GetGlobalPathPerceptionCellCount() const { return perception_active_cells_; }
  void SetGlobalPathPerceptionEnabled(bool enabled) {
    global_path_perception_enabled_ = enabled;
  }

 private:
  int inline index(double coord) { return static_cast<int>(coord); }

  int inline index_x_safe(double coord) {
    return std::min(std::max(index(coord), 0), max_x_ - 1);
  }

  int inline index_y_safe(double coord) {
    return std::min(std::max(index(coord), 0), max_y_ - 1);
  }

  double GetRealCost(int layer, double x, double y,
                     Eigen::Vector2d* grad = nullptr,
                     int* real_layer = nullptr);

  double GetRealCostSafe(int layer, double x, double y,
                         const double height_hint);
  double EffectiveCost(int row, int col) const;
  Eigen::Vector2d EffectiveGradient(int row, int col) const;
  bool UpdatePerceptionInflationParams(double inflation_radius,
                                       double inscribed_radius,
                                       double peak_cost,
                                       double cost_scaling_factor);
  std::uint8_t PerceptionInflationCost(
      int drow, int dcol, double inflation_radius,
      double inscribed_radius, double cost_scaling_factor) const;
  double PctPerceptionCost(std::uint8_t nav2_cost) const;
  bool MarkPerceptionSource(int row, int col, double stamp);
  bool ClearPerceptionSource(int row, int col);
  int ClearPerceptionSourceCircle(const Eigen::Vector3i& center, double radius);
  int DecayGlobalPathPerceptionSources(double stamp, double persistence);
  int RebuildGlobalPathPerceptionCosts();
  int ClearPerceptionSourcesOutside(const Eigen::Vector4i& window_bounds);
  int PerceptionKey(int row, int col) const;
  Eigen::Vector2i DecodePerceptionKey(int key) const;

 private:
  bool debug_ = false;
  double resolution_ = 0.0;
  double resolution_inv_ = 0.0;
  int max_layers_ = 0;
  int max_x_ = 0;
  int max_y_ = 0;
  int xy_size_ = 0;
  double offset_ = 0;

  double safe_cost_threshold_ = 10;

  Eigen::MatrixXd cost_;
  Eigen::MatrixXd perception_cost_;
  Eigen::MatrixXi perception_nav2_cost_;
  Eigen::MatrixXd perception_source_stamp_;
  Eigen::MatrixXd ele_mask_;
  Eigen::MatrixXd height_;
  Eigen::MatrixXd ceiling_;
  Eigen::MatrixXd grad_x_;
  Eigen::MatrixXd grad_y_;
  int perception_active_cells_ = 0;
  int perception_source_active_cells_ = 0;
  double perception_inflation_radius_ = 0.0;
  double perception_inscribed_radius_ = 0.0;
  double perception_peak_cost_ = 0.0;
  double perception_cost_scaling_factor_ = 0.0;
  std::unordered_set<int> perception_source_indices_;
  std::unordered_set<int> perception_cost_indices_;
  bool global_path_perception_enabled_ = true;
};
