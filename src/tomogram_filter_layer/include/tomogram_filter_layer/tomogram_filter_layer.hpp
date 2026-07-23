// Copyright 2026 zby
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the zby nor the names of its contributors may be used
//      to endorse or promote products derived from this software without
//      specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace tomogram_filter_layer
{

class TomogramFilterLayer : public nav2_costmap_2d::Layer
{
public:
  TomogramFilterLayer() = default;
  ~TomogramFilterLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}
  void activate() override;
  void deactivate() override;

private:
  struct CellKey
  {
    int x;
    int y;

    bool operator==(const CellKey & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & key) const;
  };

  struct GroundSample
  {
    float x;
    float y;
    float z;
  };

  struct SurfaceSample
  {
    float x;
    float y;
    float z;
    float cost;
  };

  using GroundGrid = std::unordered_map<CellKey, std::vector<GroundSample>, CellKeyHash>;

  static std::string normalizeFrame(std::string frame);
  static Eigen::Matrix4f transformMatrix(
    const geometry_msgs::msg::TransformStamped & transform);
  static CellKey cellKey(float x, float y, double resolution);

  void createInterfaces();
  void destroyInterfaces();
  bool readSurfaceMetadata(double & resolution, std::string & frame) const;
  bool loadTomogramSurface();
  void publishSurfaceDebug();
  bool lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool isGroundPoint(
    float x, float y, float z, const GroundGrid & ground_grid) const;

  std::string input_cloud_topic_;
  std::string tomogram_surface_file_;
  std::string surface_debug_topic_;
  std::string output_scan_topic_;
  std::string map_frame_;
  std::string base_frame_;

  bool enabled_{true};
  bool publish_surface_debug_{true};
  bool publish_unfiltered_when_not_ready_{false};
  bool use_latest_transform_fallback_{true};
  bool keep_points_without_ground_{true};
  double min_height_{-0.1};
  double max_height_{0.8};
  double angle_min_{-3.141592653589793};
  double angle_max_{3.141592653589793};
  double angle_increment_{0.0087};
  double scan_time_{0.1};
  double range_min_{0.15};
  double range_max_{10.0};
  double tomogram_grid_resolution_{0.2};
  double ground_xy_tolerance_{0.2};
  double ground_z_min_offset_{-0.2};
  double ground_z_max_offset_{0.22};
  double traversable_cost_max_{45.0};
  double transform_tolerance_{0.1};
  int min_tomogram_points_{1000};
  std::size_t scan_size_{1};

  std::mutex ground_mutex_;
  GroundGrid ground_grid_;
  std::vector<SurfaceSample> surface_points_;
  bool tomogram_ready_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_debug_pub_;
};

}  // namespace tomogram_filter_layer
