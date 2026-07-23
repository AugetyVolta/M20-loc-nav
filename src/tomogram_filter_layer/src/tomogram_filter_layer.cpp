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

#include "tomogram_filter_layer/tomogram_filter_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "Eigen/Geometry"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/time.h"

PLUGINLIB_EXPORT_CLASS(
  tomogram_filter_layer::TomogramFilterLayer,
  nav2_costmap_2d::Layer)

namespace tomogram_filter_layer
{

std::size_t TomogramFilterLayer::CellKeyHash::operator()(const CellKey & key) const
{
  const auto ux = static_cast<std::uint32_t>(key.x);
  const auto uy = static_cast<std::uint32_t>(key.y);
  return static_cast<std::size_t>(
    (static_cast<std::uint64_t>(ux) << 32U) ^ static_cast<std::uint64_t>(uy));
}

std::string TomogramFilterLayer::normalizeFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

Eigen::Matrix4f TomogramFilterLayer::transformMatrix(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  Eigen::Quaternionf quaternion(
    static_cast<float>(rotation.w),
    static_cast<float>(rotation.x),
    static_cast<float>(rotation.y),
    static_cast<float>(rotation.z));
  if (quaternion.norm() < 1e-6f) {
    quaternion = Eigen::Quaternionf::Identity();
  } else {
    quaternion.normalize();
  }

  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix(0, 3) = static_cast<float>(translation.x);
  matrix(1, 3) = static_cast<float>(translation.y);
  matrix(2, 3) = static_cast<float>(translation.z);
  return matrix;
}

TomogramFilterLayer::CellKey TomogramFilterLayer::cellKey(
  float x, float y, double resolution)
{
  return CellKey{
    static_cast<int>(std::floor(x / resolution)),
    static_cast<int>(std::floor(y / resolution))};
}

void TomogramFilterLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TomogramFilterLayer: unable to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "input_cloud_topic", rclcpp::ParameterValue(std::string("/cloud_registered_body_1")));
  declareParameter(
    "tomogram_surface_file", rclcpp::ParameterValue(std::string("")));
  declareParameter("publish_surface_debug", rclcpp::ParameterValue(true));
  declareParameter(
    "surface_debug_topic", rclcpp::ParameterValue(std::string("/traversability_tomogram")));
  declareParameter(
    "output_scan_topic", rclcpp::ParameterValue(std::string("/traversability_filtered_scan")));
  declareParameter("map_frame", rclcpp::ParameterValue(std::string("map")));
  declareParameter("base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declareParameter("publish_unfiltered_when_not_ready", rclcpp::ParameterValue(false));
  declareParameter("min_height", rclcpp::ParameterValue(-0.1));
  declareParameter("max_height", rclcpp::ParameterValue(0.8));
  declareParameter("angle_min", rclcpp::ParameterValue(-M_PI));
  declareParameter("angle_max", rclcpp::ParameterValue(M_PI));
  declareParameter("angle_increment", rclcpp::ParameterValue(0.0087));
  declareParameter("scan_time", rclcpp::ParameterValue(0.1));
  declareParameter("range_min", rclcpp::ParameterValue(0.15));
  declareParameter("range_max", rclcpp::ParameterValue(10.0));
  declareParameter("tomogram_grid_resolution", rclcpp::ParameterValue(0.0));
  declareParameter("ground_xy_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("ground_z_min_offset", rclcpp::ParameterValue(-0.2));
  declareParameter("ground_z_max_offset", rclcpp::ParameterValue(0.22));
  declareParameter("traversable_cost_max", rclcpp::ParameterValue(45.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declareParameter("use_latest_transform_fallback", rclcpp::ParameterValue(true));
  declareParameter("keep_points_without_ground", rclcpp::ParameterValue(true));
  declareParameter("min_tomogram_points", rclcpp::ParameterValue(1000));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".input_cloud_topic", input_cloud_topic_);
  node->get_parameter(name_ + ".tomogram_surface_file", tomogram_surface_file_);
  node->get_parameter(name_ + ".publish_surface_debug", publish_surface_debug_);
  node->get_parameter(name_ + ".surface_debug_topic", surface_debug_topic_);
  node->get_parameter(name_ + ".output_scan_topic", output_scan_topic_);
  node->get_parameter(name_ + ".map_frame", map_frame_);
  node->get_parameter(name_ + ".base_frame", base_frame_);
  node->get_parameter(
    name_ + ".publish_unfiltered_when_not_ready", publish_unfiltered_when_not_ready_);
  node->get_parameter(name_ + ".min_height", min_height_);
  node->get_parameter(name_ + ".max_height", max_height_);
  node->get_parameter(name_ + ".angle_min", angle_min_);
  node->get_parameter(name_ + ".angle_max", angle_max_);
  node->get_parameter(name_ + ".angle_increment", angle_increment_);
  node->get_parameter(name_ + ".scan_time", scan_time_);
  node->get_parameter(name_ + ".range_min", range_min_);
  node->get_parameter(name_ + ".range_max", range_max_);
  node->get_parameter(name_ + ".tomogram_grid_resolution", tomogram_grid_resolution_);
  node->get_parameter(name_ + ".ground_xy_tolerance", ground_xy_tolerance_);
  node->get_parameter(name_ + ".ground_z_min_offset", ground_z_min_offset_);
  node->get_parameter(name_ + ".ground_z_max_offset", ground_z_max_offset_);
  node->get_parameter(name_ + ".traversable_cost_max", traversable_cost_max_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(
    name_ + ".use_latest_transform_fallback", use_latest_transform_fallback_);
  node->get_parameter(name_ + ".keep_points_without_ground", keep_points_without_ground_);
  node->get_parameter(name_ + ".min_tomogram_points", min_tomogram_points_);

  map_frame_ = normalizeFrame(map_frame_);
  base_frame_ = normalizeFrame(base_frame_);
  if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
    throw std::invalid_argument("TomogramFilterLayer: invalid angle limits");
  }
  if (range_min_ < 0.0 || range_max_ <= range_min_) {
    throw std::invalid_argument("TomogramFilterLayer: invalid range limits");
  }
  if (min_height_ > max_height_) {
    throw std::invalid_argument("TomogramFilterLayer: invalid height limits");
  }
  if (ground_xy_tolerance_ < 0.0) {
    throw std::invalid_argument("TomogramFilterLayer: invalid ground XY tolerance");
  }
  if (ground_z_min_offset_ > ground_z_max_offset_) {
    throw std::invalid_argument("TomogramFilterLayer: invalid ground Z offsets");
  }
  if (min_tomogram_points_ < 1) {
    throw std::invalid_argument("TomogramFilterLayer: min_tomogram_points must be positive");
  }
  scan_size_ = std::max<std::size_t>(
    1, static_cast<std::size_t>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_)));

  if (enabled_ && !loadTomogramSurface()) {
    throw std::runtime_error(
            "TomogramFilterLayer: failed to load static surface '" +
            tomogram_surface_file_ + "'");
  }
  createInterfaces();
  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "TomogramFilterLayer: enabled=%d cloud=%s surface=%s output=%s "
    "height=[%.2f,%.2f] ground_xy=%.2f ground_z=[%.2f,%.2f] cost<=%.1f",
    static_cast<int>(enabled_), input_cloud_topic_.c_str(), tomogram_surface_file_.c_str(),
    output_scan_topic_.c_str(), min_height_, max_height_, ground_xy_tolerance_,
    ground_z_min_offset_, ground_z_max_offset_, traversable_cost_max_);
}

void TomogramFilterLayer::createInterfaces()
{
  if (!enabled_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    return;
  }

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TomogramFilterLayer::cloudCallback, this, std::placeholders::_1));
  scan_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>(
    output_scan_topic_, rclcpp::SensorDataQoS());
  if (publish_surface_debug_) {
    rclcpp::QoS surface_qos(rclcpp::KeepLast(1));
    surface_qos.reliable().transient_local();
    surface_debug_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      surface_debug_topic_, surface_qos);
    publishSurfaceDebug();
  }
}

void TomogramFilterLayer::destroyInterfaces()
{
  cloud_sub_.reset();
  scan_pub_.reset();
  surface_debug_pub_.reset();
}

bool TomogramFilterLayer::readSurfaceMetadata(
  double & resolution, std::string & frame) const
{
  std::ifstream input(tomogram_surface_file_, std::ios::binary);
  if (!input) {
    return false;
  }

  constexpr char resolution_prefix[] = "# TOMOGRAM_RESOLUTION ";
  constexpr char frame_prefix[] = "# TOMOGRAM_FRAME ";
  std::string line;
  while (std::getline(input, line)) {
    if (line.rfind(resolution_prefix, 0) == 0) {
      std::istringstream value(line.substr(sizeof(resolution_prefix) - 1));
      value >> resolution;
    } else if (line.rfind(frame_prefix, 0) == 0) {
      frame = normalizeFrame(line.substr(sizeof(frame_prefix) - 1));
    } else if (line.rfind("DATA ", 0) == 0) {
      break;
    }
  }
  return true;
}

bool TomogramFilterLayer::loadTomogramSurface()
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }
  if (tomogram_surface_file_.empty()) {
    RCLCPP_ERROR(
      node->get_logger(), "TomogramFilterLayer: tomogram_surface_file is empty");
    return false;
  }

  double file_resolution = 0.0;
  std::string file_frame;
  if (!readSurfaceMetadata(file_resolution, file_frame)) {
    RCLCPP_ERROR(
      node->get_logger(), "TomogramFilterLayer: unable to read metadata from '%s'",
      tomogram_surface_file_.c_str());
    return false;
  }
  if (file_resolution > 0.0) {
    if (tomogram_grid_resolution_ > 0.0 &&
      std::abs(tomogram_grid_resolution_ - file_resolution) > 1e-6)
    {
      RCLCPP_WARN(
        node->get_logger(),
        "TomogramFilterLayer: configured grid resolution %.6f differs from PCD %.6f; "
        "using the PCD value",
        tomogram_grid_resolution_, file_resolution);
    }
    tomogram_grid_resolution_ = file_resolution;
  }
  if (tomogram_grid_resolution_ <= 0.0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "TomogramFilterLayer: PCD has no TOMOGRAM_RESOLUTION metadata and "
      "tomogram_grid_resolution is not positive");
    return false;
  }
  if (!file_frame.empty() && file_frame != map_frame_) {
    RCLCPP_ERROR(
      node->get_logger(),
      "TomogramFilterLayer: PCD frame '%s' does not match configured map_frame '%s'",
      file_frame.c_str(), map_frame_.c_str());
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(tomogram_surface_file_, cloud) < 0) {
    RCLCPP_ERROR(
      node->get_logger(), "TomogramFilterLayer: unable to read PCD '%s'",
      tomogram_surface_file_.c_str());
    return false;
  }

  GroundGrid next_grid;
  next_grid.reserve(cloud.size());
  std::vector<SurfaceSample> next_surface;
  next_surface.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(point.intensity))
    {
      continue;
    }
    if (traversable_cost_max_ > 0.0 && point.intensity > traversable_cost_max_) {
      continue;
    }
    next_grid[cellKey(point.x, point.y, tomogram_grid_resolution_)].push_back(
      GroundSample{point.x, point.y, point.z});
    next_surface.push_back(
      SurfaceSample{point.x, point.y, point.z, point.intensity});
  }

  if (next_surface.size() < static_cast<std::size_t>(min_tomogram_points_)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "TomogramFilterLayer: surface PCD has only %zu/%zu accepted points, minimum=%d",
      next_surface.size(), cloud.size(), min_tomogram_points_);
    return false;
  }

  const auto cell_count = next_grid.size();
  {
    std::lock_guard<std::mutex> lock(ground_mutex_);
    ground_grid_ = std::move(next_grid);
    surface_points_ = std::move(next_surface);
    tomogram_ready_ = !ground_grid_.empty();
  }
  RCLCPP_INFO(
    node->get_logger(),
    "TomogramFilterLayer loaded %zu/%zu surface points in %zu cells from %s "
    "(frame=%s resolution=%.3f)",
    surface_points_.size(), cloud.size(), cell_count, tomogram_surface_file_.c_str(),
    map_frame_.c_str(), tomogram_grid_resolution_);
  return tomogram_ready_;
}

void TomogramFilterLayer::publishSurfaceDebug()
{
  auto node = node_.lock();
  if (!node || !surface_debug_pub_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = node->get_clock()->now();
  message.header.frame_id = map_frame_;
  message.height = 1;
  message.width = static_cast<std::uint32_t>(surface_points_.size());
  message.is_bigendian = false;
  message.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(surface_points_.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(message, "intensity");
  for (const auto & point : surface_points_) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    *iter_intensity = point.cost;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }
  surface_debug_pub_->publish(message);
}

bool TomogramFilterLayer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform)
{
  auto node = node_.lock();
  if (!node || !tf_) {
    return false;
  }
  try {
    transform = tf_->lookupTransform(
      target_frame, source_frame, rclcpp::Time(stamp),
      rclcpp::Duration::from_seconds(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & exact_error) {
    if (!use_latest_transform_fallback_) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "TomogramFilterLayer TF %s <- %s failed: %s",
        target_frame.c_str(), source_frame.c_str(), exact_error.what());
      return false;
    }
    try {
      transform = tf_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance_));
      return true;
    } catch (const tf2::TransformException & latest_error) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "TomogramFilterLayer TF %s <- %s failed at stamp and latest: %s",
        target_frame.c_str(), source_frame.c_str(), latest_error.what());
      return false;
    }
  }
}

bool TomogramFilterLayer::isGroundPoint(
  float x, float y, float z, const GroundGrid & ground_grid) const
{
  const CellKey center = cellKey(x, y, tomogram_grid_resolution_);
  const int search_radius = static_cast<int>(
    std::ceil(ground_xy_tolerance_ / tomogram_grid_resolution_));
  const float xy_tolerance_sq = static_cast<float>(
    ground_xy_tolerance_ * ground_xy_tolerance_);
  bool has_nearby_surface = false;

  for (int dy = -search_radius; dy <= search_radius; ++dy) {
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
      const auto it = ground_grid.find(CellKey{center.x + dx, center.y + dy});
      if (it == ground_grid.end()) {
        continue;
      }
      for (const auto & sample : it->second) {
        const float delta_x = x - sample.x;
        const float delta_y = y - sample.y;
        if (delta_x * delta_x + delta_y * delta_y > xy_tolerance_sq) {
          continue;
        }
        has_nearby_surface = true;
        const float delta_z = z - sample.z;
        if (delta_z >= static_cast<float>(ground_z_min_offset_) &&
          delta_z <= static_cast<float>(ground_z_max_offset_))
        {
          return true;
        }
      }
    }
  }
  return !keep_points_without_ground_ && !has_nearby_surface;
}

void TomogramFilterLayer::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!enabled_ || !scan_pub_) {
    return;
  }
  auto node = node_.lock();
  if (!node || msg->header.frame_id.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped base_transform;
  if (!lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, base_transform)) {
    return;
  }
  geometry_msgs::msg::TransformStamped map_transform;
  const bool map_transform_ok = lookupTransform(
    map_frame_, msg->header.frame_id, msg->header.stamp, map_transform);

  sensor_msgs::msg::LaserScan scan;
  scan.header = msg->header;
  scan.header.frame_id = base_frame_;
  scan.angle_min = static_cast<float>(angle_min_);
  scan.angle_max = static_cast<float>(angle_max_);
  scan.angle_increment = static_cast<float>(angle_increment_);
  scan.time_increment = 0.0f;
  scan.scan_time = static_cast<float>(scan_time_);
  scan.range_min = static_cast<float>(range_min_);
  scan.range_max = static_cast<float>(range_max_);
  scan.ranges.assign(scan_size_, std::numeric_limits<float>::infinity());

  const Eigen::Matrix4f cloud_to_base = transformMatrix(base_transform);
  const Eigen::Matrix4f cloud_to_map = map_transform_ok ?
    transformMatrix(map_transform) : Eigen::Matrix4f::Identity();
  std::size_t height_candidates = 0;
  std::size_t removed = 0;
  std::size_t kept = 0;

  std::lock_guard<std::mutex> lock(ground_mutex_);
  const bool filter_ready = tomogram_ready_ && map_transform_ok;
  if (!filter_ready && !publish_unfiltered_when_not_ready_) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "TomogramFilterLayer suppressing scan: tomogram_ready=%d map_tf=%d",
      static_cast<int>(tomogram_ready_), static_cast<int>(map_transform_ok));
    return;
  }

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const Eigen::Vector4f source_point(x, y, z, 1.0f);
      const Eigen::Vector4f base_point = cloud_to_base * source_point;
      if (base_point.z() < min_height_ || base_point.z() > max_height_) {
        continue;
      }

      const float range = std::hypot(base_point.x(), base_point.y());
      if (range < range_min_ || range > range_max_) {
        continue;
      }
      const float angle = std::atan2(base_point.y(), base_point.x());
      if (angle < angle_min_ || angle > angle_max_) {
        continue;
      }
      height_candidates++;

      if (filter_ready) {
        const Eigen::Vector4f map_point = cloud_to_map * source_point;
        if (isGroundPoint(map_point.x(), map_point.y(), map_point.z(), ground_grid_)) {
          removed++;
          continue;
        }
      }

      auto index = static_cast<std::size_t>(
        std::floor((static_cast<double>(angle) - angle_min_) / angle_increment_));
      if (index >= scan.ranges.size()) {
        index = scan.ranges.size() - 1;
      }
      if (range < scan.ranges[index]) {
        scan.ranges[index] = range;
      }
      kept++;
    }
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "TomogramFilterLayer invalid input PointCloud2: %s", error.what());
    return;
  }

  scan_pub_->publish(scan);
  const auto finite_beams = static_cast<std::size_t>(std::count_if(
      scan.ranges.begin(), scan.ranges.end(),
      [](float range) {return std::isfinite(range);}));
  RCLCPP_DEBUG_THROTTLE(
    node->get_logger(), *node->get_clock(), 2000,
    "TomogramFilterLayer cloud=%zu candidates=%zu removed=%zu kept=%zu beams=%zu",
    static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height),
    height_candidates, removed, kept, finite_beams);
}

void TomogramFilterLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  current_ = true;
}

void TomogramFilterLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &, int, int, int, int)
{
}

void TomogramFilterLayer::reset()
{
  // The tomogram is a static map input, not rolling local-costmap state.
  // Preserve it across clear-costmap/reset requests.
  current_ = true;
}

void TomogramFilterLayer::activate()
{
  createInterfaces();
}

void TomogramFilterLayer::deactivate()
{
  destroyInterfaces();
}

}  // namespace tomogram_filter_layer
