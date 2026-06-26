#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>

#include "common/data_types.h"

enum HeuristicType : int { kEuclidean = 0, kManhattan = 1, kDiagonal = 2 };

class Node {
 public:
  Node() = default;
  Node(Eigen::Vector3i idx, Node* parent) : idx(idx), parent(parent) {}
  ~Node() = default;

  bool operator==(const Node& other) const { return idx == other.idx; }

  void Reset() {
    f = 0.0;
    g = 1e9;
    parent = nullptr;
  }

  double f = 1e9;
  double g = 1e9;
  double height = 0.0;
  double ele = 0;
  double cost = 0.0;
  double static_cost = 0.0;
  double perception_cost = 0.0;
  double perception_source_stamp = -1.0;
  int layer = 0;
  Eigen::Vector3i idx = Eigen::Vector3i(0, 0, 0);  // layer, row, col
  Node* parent = nullptr;

  bool HasPerceptionSource() const { return perception_source_stamp >= 0.0; }
};

struct NodeCompare {
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

using MultiLayerGridMap = std::vector<std::vector<std::vector<Node>>>;

class Astar {
 public:
  Astar(const HeuristicType h_type = kDiagonal) : h_type_(h_type) {
    switch (h_type) {
      case kEuclidean:
        printf("Euclidean heuristic is used\n");
        break;
      case kManhattan:
        printf("Manhattan heuristic is used\n");
        break;
      case kDiagonal:
        printf("Diagonal heuristic is used\n");
        break;
    };
  }
  ~Astar() = default;

  void Init(const double cost_threshold, const int num_layers,
            const double resolution, const double step_cost_weight,  const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& height_map, const Eigen::MatrixXd& ele_map);

  void Reset();

  void Debug() { debug_ = true; }

  bool Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal);

  std::vector<PathPoint> GetPathPoints() const;

  Eigen::MatrixXd GetResultMatrix() const;
  Eigen::MatrixXi GetVisitedSet() const { return visited_set_; }

  Eigen::MatrixXd GetCostLayer(int layer) const;
  Eigen::MatrixXd GetEleLayer(int layer) const;
  Eigen::MatrixXd GetPerceptionCostLayer(int layer) const;

  int UpdateGlobalPathPerception(const Eigen::MatrixXi& perception_indices,
                             const double inflation_radius,
                             const double inscribed_radius,
                             const double peak_cost,
                             const double cost_scaling_factor,
                             const double stamp,
                             const double persistence,
                             const Eigen::Vector3i& clear_center,
                             const double clear_radius);
  int DecayGlobalPathPerception(const double stamp, const double persistence);
  int ClearGlobalPathPerceptionIndices(const Eigen::MatrixXi& clear_indices);
  void ClearGlobalPathPerception();
  bool HasGlobalPathPerception() const { return perception_active_cells_ > 0; }
  int GetGlobalPathPerceptionCellCount() const { return perception_active_cells_; }
  Eigen::MatrixXi BuildGlobalPathPerceptionMarkIndices(
      const Eigen::MatrixXi& mark_cells,
      const int current_layer,
      const double robot_height,
      const bool skip_static_obstacles,
      const double static_skip_cost) const;
  Eigen::MatrixXi BuildGlobalPathPerceptionClearIndices(
      const Eigen::Vector2i& origin_cell,
      const Eigen::MatrixXi& endpoint_cells,
      const int current_layer,
      const double robot_height) const;

 private:
  double CalculateStepCost(const Node* node1, const Node* node2) const;

  int DecideLayer(const Node* cur_node) const;

  int GetHash(const Eigen::Vector3i& idx) const;

  std::vector<Eigen::Vector3i> GetNeighbors(Node* node) const;

  double GetHeuristic(const Node* node1, const Node* node2) const;

  Eigen::MatrixXd PathToMatrix(const std::vector<Eigen::Vector3i>& path);

  void ToPathPoints(const std::vector<Eigen::Vector3i>& path,
                    std::vector<PathPoint>& path_points);

  void ConvertClosedSetToMatrix(
      const std::unordered_map<int, Node*>& closed_set);

  double GetNeighborAverageCost(const Node* node1);

  bool IsValidNodeCoord(int layer, int y, int x) const;
  double EffectiveCost(const Node& node) const;
  void RefreshNodeCost(Node& node);
  bool UpdatePerceptionInflationParams(double inflation_radius,
                                       double inscribed_radius,
                                       double peak_cost,
                                       double cost_scaling_factor);
  double PerceptionInflationCost(int drow, int dcol, double inflation_radius,
                              double inscribed_radius, double peak_cost,
                              double cost_scaling_factor) const;
  bool MarkPerceptionSource(int layer, int row, int col, double stamp);
  bool ClearPerceptionSource(int layer, int row, int col);
  int ClearPerceptionSourceCircle(const Eigen::Vector3i& center, double radius);
  int DecayGlobalPathPerceptionSources(double stamp, double persistence);
  int RebuildGlobalPathPerceptionCosts();
  int SelectPerceptionLayerForCell(int row, int col, int current_layer,
                                   double robot_height) const;
  bool IsStaticObstacleCell(int layer, int row, int col,
                            double static_skip_cost) const;
  std::vector<Eigen::Vector2i> BresenhamCells(int row0, int col0, int row1,
                                              int col1) const;

  //后端路径重新优化，选取代价更低的路径
  bool RefinePathSerach();



 private:
  HeuristicType h_type_ = kDiagonal;

  int max_x_ = 0;
  int max_y_ = 0;
  int max_layers_ = 0;
  int xy_size_ = 0;
  double resolution_ = 0;
  MultiLayerGridMap grid_map_;
  double cost_threshold_ = 35;
  double step_cost_weight_ = 1.0;
  int perception_active_cells_ = 0;
  int perception_source_active_cells_ = 0;
  double perception_inflation_radius_ = 0.0;
  double perception_inscribed_radius_ = 0.0;
  double perception_peak_cost_ = 0.0;
  double perception_cost_scaling_factor_ = 0.0;

  // int search_layer_depth_ = 1;
  std::vector<int> search_layers_offset_;

  bool debug_ = false;
  Eigen::MatrixXi visited_set_;

  // std::vector<Eigen::Vector3i> search_result_;
  std::vector<Node*> search_result_;
};
