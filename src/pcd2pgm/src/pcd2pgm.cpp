// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcd2pgm/pcd2pgm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcd2pgm
{
Pcd2PgmNode::Pcd2PgmNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

  applyTransform();
  levelPointCloud();
  autoAlignGroundToMapPlane();

  passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
  radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
  setMapTopicMsg(cloud_after_radius_, map_topic_msg_);

  timer_ =
    create_wall_timer(std::chrono::seconds(1), std::bind(&Pcd2PgmNode::publishCallback, this));
}

void Pcd2PgmNode::publishCallback()
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_after_radius_, output);
  output.header.frame_id = "map";
  pcd_publisher_->publish(output);
  map_publisher_->publish(map_topic_msg_);
}

void Pcd2PgmNode::declareParameters()
{
  declare_parameter("pcd_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("enable_leveling", false);
  declare_parameter(
    "leveling_quaternion_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
  declare_parameter("refine_ground_plane", false);
  declare_parameter("auto_align_ground_to_map", false);
  declare_parameter("ground_plane_voxel_size", 0.08);
  declare_parameter("ground_plane_distance_threshold", 0.03);
  declare_parameter("ground_plane_max_iterations", 3000);
  declare_parameter("ground_plane_min_inliers", 1000);
  declare_parameter("ground_plane_min_normal_z", 0.95);
  declare_parameter("ground_histogram_bin_size", 0.10);
  declare_parameter("map_topic_name", "map");
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // 新增的参数
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("pcd_file", pcd_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("enable_leveling", enable_leveling_);
  get_parameter("leveling_quaternion_xyzw", leveling_quaternion_xyzw_);
  get_parameter("refine_ground_plane", refine_ground_plane_);
  get_parameter("auto_align_ground_to_map", auto_align_ground_to_map_);
  get_parameter("ground_plane_voxel_size", ground_plane_voxel_size_);
  get_parameter("ground_plane_distance_threshold", ground_plane_distance_threshold_);
  get_parameter("ground_plane_max_iterations", ground_plane_max_iterations_);
  get_parameter("ground_plane_min_inliers", ground_plane_min_inliers_);
  get_parameter("ground_plane_min_normal_z", ground_plane_min_normal_z_);
  get_parameter("ground_histogram_bin_size", ground_histogram_bin_size_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);  // 获取新的参数
}

void Pcd2PgmNode::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering: %lu points (relative z=[%.3f, %.3f])",
    cloud_after_pass_through_->points.size(), thre_low, thre_high);
}

void Pcd2PgmNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}

void Pcd2PgmNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < msg.info.width && j >= 0 && j < msg.info.height) {
      msg.data[i + j * msg.info.width] = 100;
    }
  }

  RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

void Pcd2PgmNode::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}

void Pcd2PgmNode::levelPointCloud()
{
  if (!enable_leveling_) {
    return;
  }

  if (leveling_quaternion_xyzw_.size() != 4) {
    RCLCPP_WARN(
      get_logger(),
      "enable_leveling is true, but leveling_quaternion_xyzw must have 4 values [x, y, z, w]. "
      "Skipping fixed leveling rotation.");
  } else {
    Eigen::Quaternionf q(
      static_cast<float>(leveling_quaternion_xyzw_[3]),
      static_cast<float>(leveling_quaternion_xyzw_[0]),
      static_cast<float>(leveling_quaternion_xyzw_[1]),
      static_cast<float>(leveling_quaternion_xyzw_[2]));
    if (!std::isfinite(q.norm()) || q.norm() < 1.0e-6f) {
      RCLCPP_WARN(get_logger(), "Invalid leveling quaternion. Skipping fixed leveling rotation.");
    } else {
      q.normalize();
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.linear() = q.toRotationMatrix();
      pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform);
      RCLCPP_INFO(get_logger(), "Applied fixed leveling rotation from leveling_quaternion_xyzw.");
    }
  }

  if (!refine_ground_plane_) {
    return;
  }

  Eigen::Vector3f normal;
  double d = 0.0;
  int inlier_count = 0;
  if (!fitGroundPlane(normal, d, inlier_count)) {
    RCLCPP_WARN(get_logger(), "Ground-plane refinement requested, but no valid plane was found.");
    return;
  }

  const float normal_z = std::abs(normal.z());
  const double tilt_deg = std::acos(std::min(1.0f, std::max(-1.0f, normal_z))) * 180.0 / M_PI;
  if (inlier_count < ground_plane_min_inliers_ || normal_z < ground_plane_min_normal_z_) {
    RCLCPP_WARN(
      get_logger(),
      "Skipping ground-plane refinement: inliers=%d, normal_z=%.4f, tilt=%.3f deg",
      inlier_count, normal_z, tilt_deg);
    return;
  }

  const Eigen::Matrix3f correction =
    rotationAligningVectors(normal, Eigen::Vector3f::UnitZ());
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.linear() = correction;
  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform);

  RCLCPP_INFO(
    get_logger(),
    "Applied ground-plane refinement: inliers=%d, before tilt=%.3f deg",
    inlier_count, tilt_deg);
}

bool Pcd2PgmNode::fitGroundPlane(Eigen::Vector3f & normal, double & d, int & inlier_count) const
{
  if (!pcd_cloud_ || pcd_cloud_->points.empty()) {
    return false;
  }

  auto sample = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (ground_plane_voxel_size_ > 0.0f) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pcd_cloud_);
    voxel_grid.setLeafSize(ground_plane_voxel_size_, ground_plane_voxel_size_, ground_plane_voxel_size_);
    voxel_grid.filter(*sample);
  } else {
    *sample = *pcd_cloud_;
  }

  if (sample->points.size() < 100) {
    return false;
  }

  pcl::SACSegmentation<pcl::PointXYZ> segmenter;
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmenter.setOptimizeCoefficients(true);
  segmenter.setModelType(pcl::SACMODEL_PLANE);
  segmenter.setMethodType(pcl::SAC_RANSAC);
  segmenter.setMaxIterations(ground_plane_max_iterations_);
  segmenter.setDistanceThreshold(ground_plane_distance_threshold_);
  segmenter.setInputCloud(sample);
  segmenter.segment(inliers, coefficients);

  if (coefficients.values.size() < 4 || inliers.indices.empty()) {
    return false;
  }

  normal = Eigen::Vector3f(
    coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  const float norm = normal.norm();
  if (!std::isfinite(norm) || norm < 1.0e-6f) {
    return false;
  }

  normal /= norm;
  d = coefficients.values[3] / norm;
  if (normal.z() < 0.0f) {
    normal = -normal;
    d = -d;
  }
  inlier_count = static_cast<int>(inliers.indices.size());
  return true;
}

Eigen::Matrix3f Pcd2PgmNode::rotationAligningVectors(
  const Eigen::Vector3f & source, const Eigen::Vector3f & target) const
{
  Eigen::Vector3f src = source.normalized();
  Eigen::Vector3f dst = target.normalized();
  Eigen::Vector3f axis = src.cross(dst);
  const float axis_norm = axis.norm();
  const float dot = std::min(1.0f, std::max(-1.0f, src.dot(dst)));

  if (axis_norm < 1.0e-6f) {
    if (dot > 0.0f) {
      return Eigen::Matrix3f::Identity();
    }
    Eigen::Vector3f orthogonal = Eigen::Vector3f::UnitX();
    if (std::abs(src.x()) > 0.9f) {
      orthogonal = Eigen::Vector3f::UnitY();
    }
    axis = src.cross(orthogonal).normalized();
    return Eigen::AngleAxisf(static_cast<float>(M_PI), axis).toRotationMatrix();
  }

  axis /= axis_norm;
  return Eigen::AngleAxisf(std::acos(dot), axis).toRotationMatrix();
}

void Pcd2PgmNode::autoAlignGroundToMapPlane()
{
  if (!auto_align_ground_to_map_) {
    return;
  }

  double ground_z = 0.0;
  if (!estimateGroundZ(ground_z)) {
    RCLCPP_WARN(
      get_logger(),
      "auto_align_ground_to_map is enabled, but ground z could not be estimated. "
      "Using the original z coordinates.");
    return;
  }

  for (auto & point : pcd_cloud_->points) {
    point.z -= static_cast<float>(ground_z);
  }

  RCLCPP_INFO(
    get_logger(),
    "Auto ground alignment enabled: detected ground z=%.3f m, shifted cloud by z=%+.3f m. "
    "thre_z_min/max are relative to this 2D map plane.",
    ground_z, -ground_z);
}

bool Pcd2PgmNode::estimateGroundZ(double & ground_z) const
{
  if (!pcd_cloud_ || pcd_cloud_->points.empty()) {
    return false;
  }

  std::vector<double> z_values;
  z_values.reserve(pcd_cloud_->points.size());
  for (const auto & point : pcd_cloud_->points) {
    if (std::isfinite(point.z)) {
      z_values.push_back(point.z);
    }
  }
  if (z_values.size() < 100) {
    return false;
  }

  auto median_it = z_values.begin() + static_cast<std::ptrdiff_t>(z_values.size() / 2);
  std::nth_element(z_values.begin(), median_it, z_values.end());
  const double median_z = *median_it;
  const auto minmax = std::minmax_element(z_values.begin(), z_values.end());
  const double z_min = *minmax.first;
  const double z_max = median_z;
  if (!std::isfinite(z_min) || !std::isfinite(z_max) || z_max <= z_min) {
    return false;
  }

  const double bin_size = std::max(0.02, static_cast<double>(ground_histogram_bin_size_));
  const int bin_count = static_cast<int>(std::ceil((z_max - z_min) / bin_size));
  if (bin_count <= 0) {
    return false;
  }

  std::vector<int> histogram(static_cast<size_t>(bin_count), 0);
  for (const double z : z_values) {
    if (z < z_min || z > z_max) {
      continue;
    }
    int bin = static_cast<int>(std::floor((z - z_min) / bin_size));
    bin = std::min(std::max(bin, 0), bin_count - 1);
    histogram[static_cast<size_t>(bin)] += 1;
  }

  const auto best_it = std::max_element(histogram.begin(), histogram.end());
  if (best_it == histogram.end() || *best_it < 100) {
    return false;
  }

  const int best_bin = static_cast<int>(std::distance(histogram.begin(), best_it));
  const int low_bin = std::max(0, best_bin - 1);
  const int high_bin = std::min(bin_count - 1, best_bin + 1);
  const double low_z = z_min + low_bin * bin_size;
  const double high_z = z_min + (high_bin + 1) * bin_size;

  double sum_z = 0.0;
  int count = 0;
  for (const double z : z_values) {
    if (z >= low_z && z < high_z) {
      sum_z += z;
      ++count;
    }
  }
  if (count == 0) {
    return false;
  }

  ground_z = sum_z / count;
  RCLCPP_INFO(
    get_logger(),
    "Estimated ground z from lower-half z histogram: median_z=%.3f, bin=[%.3f, %.3f], points=%d",
    median_z, low_z, high_z, count);
  return std::isfinite(ground_z);
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmNode)
