#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace traversability_layer
{

class TomogramScanFilterNode : public rclcpp::Node
{
public:
  TomogramScanFilterNode()
  : Node("tomogram_scan_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/cloud_registered_body_1");
    tomogram_topic_ = declare_parameter<std::string>("tomogram_topic", "/tomogram");
    output_scan_topic_ = declare_parameter<std::string>(
      "output_scan_topic", "/traversability_filtered_scan");
    map_frame_ = normalizeFrame(declare_parameter<std::string>("map_frame", "map"));
    base_frame_ = normalizeFrame(declare_parameter<std::string>("base_frame", "base_link"));
    enabled_ = declare_parameter<bool>("enabled", true);

    min_height_ = declare_parameter<double>("min_height", -0.1);
    max_height_ = declare_parameter<double>("max_height", 0.55);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.0087);
    scan_time_ = declare_parameter<double>("scan_time", 0.1);
    range_min_ = declare_parameter<double>("range_min", 0.15);
    range_max_ = declare_parameter<double>("range_max", 10.0);

    tomogram_grid_resolution_ = declare_parameter<double>("tomogram_grid_resolution", 0.15);
    ground_xy_tolerance_ = declare_parameter<double>("ground_xy_tolerance", 0.12);
    ground_z_min_offset_ = declare_parameter<double>("ground_z_min_offset", -0.08);
    ground_z_max_offset_ = declare_parameter<double>("ground_z_max_offset", 0.12);
    traversable_cost_max_ = declare_parameter<double>("traversable_cost_max", 45.0);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
    use_latest_transform_fallback_ = declare_parameter<bool>(
      "use_latest_transform_fallback", true);
    keep_points_without_ground_ = declare_parameter<bool>(
      "keep_points_without_ground", true);
    reload_tomogram_ = declare_parameter<bool>("reload_tomogram", false);
    min_tomogram_points_ = declare_parameter<int>("min_tomogram_points", 1000);
    if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
      throw std::invalid_argument("angle_increment must be positive and angle_max > angle_min");
    }
    if (range_min_ < 0.0 || range_max_ <= range_min_) {
      throw std::invalid_argument("range limits are invalid");
    }
    if (min_height_ > max_height_) {
      throw std::invalid_argument("min_height must not exceed max_height");
    }
    if (tomogram_grid_resolution_ <= 0.0 || ground_xy_tolerance_ < 0.0) {
      throw std::invalid_argument("tomogram grid and XY tolerance are invalid");
    }
    if (ground_z_min_offset_ > ground_z_max_offset_) {
      throw std::invalid_argument("ground_z_min_offset must not exceed ground_z_max_offset");
    }
    if (min_tomogram_points_ < 1) {
      throw std::invalid_argument("min_tomogram_points must be at least 1");
    }
    scan_size_ = static_cast<size_t>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_));
    scan_size_ = std::max<size_t>(1, scan_size_);

    rclcpp::QoS tomogram_qos(rclcpp::KeepLast(1));
    tomogram_qos.reliable().transient_local();
    tomogram_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      tomogram_topic_, tomogram_qos,
      std::bind(&TomogramScanFilterNode::tomogramCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TomogramScanFilterNode::cloudCallback, this, std::placeholders::_1));
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_scan_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Tomogram scan filter: cloud=%s tomogram=%s output=%s frames=%s->%s "
      "enabled=%d height=[%.2f, %.2f] ground_xy=%.2f ground_z=[%.2f, %.2f] cost<=%.1f",
      input_cloud_topic_.c_str(), tomogram_topic_.c_str(), output_scan_topic_.c_str(),
      base_frame_.c_str(), map_frame_.c_str(), static_cast<int>(enabled_), min_height_, max_height_,
      ground_xy_tolerance_, ground_z_min_offset_, ground_z_max_offset_,
      traversable_cost_max_);
  }

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
    size_t operator()(const CellKey & key) const
    {
      const uint64_t ux = static_cast<uint32_t>(key.x);
      const uint64_t uy = static_cast<uint32_t>(key.y);
      return static_cast<size_t>((ux << 32U) ^ uy);
    }
  };

  struct GroundSample
  {
    float x;
    float y;
    float z;
  };

  using GroundGrid = std::unordered_map<CellKey, std::vector<GroundSample>, CellKeyHash>;

  static std::string normalizeFrame(std::string frame)
  {
    while (!frame.empty() && frame.front() == '/') {
      frame.erase(frame.begin());
    }
    return frame;
  }

  static Eigen::Matrix4f transformMatrix(
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

  static CellKey cellKey(float x, float y, double resolution)
  {
    return CellKey{
      static_cast<int>(std::floor(x / resolution)),
      static_cast<int>(std::floor(y / resolution))};
  }

  bool lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_.lookupTransform(
        target_frame, source_frame, rclcpp::Time(stamp),
        rclcpp::Duration::from_seconds(transform_tolerance_));
      return true;
    } catch (const tf2::TransformException & exact_error) {
      if (!use_latest_transform_fallback_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF %s <- %s failed: %s",
          target_frame.c_str(), source_frame.c_str(), exact_error.what());
        return false;
      }
      try {
        transform = tf_buffer_.lookupTransform(
          target_frame, source_frame, tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance_));
        return true;
      } catch (const tf2::TransformException & latest_error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF %s <- %s failed at stamp and latest: %s",
          target_frame.c_str(), source_frame.c_str(), latest_error.what());
        return false;
      }
    }
  }

  void tomogramCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(ground_mutex_);
      if (tomogram_ready_ && !reload_tomogram_) {
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    GroundGrid next_grid;
    next_grid.reserve(cloud.size());
    size_t accepted = 0;
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
      accepted++;
    }

    const size_t cell_count = next_grid.size();
    const std::string tomogram_frame = normalizeFrame(
      msg->header.frame_id.empty() ? map_frame_ : msg->header.frame_id);
    if (accepted < static_cast<size_t>(min_tomogram_points_)) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring incomplete tomogram: only %zu/%zu traversable points in %zu XY cells "
        "(minimum=%d)",
        accepted, cloud.size(), cell_count, min_tomogram_points_);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(ground_mutex_);
      ground_grid_ = std::move(next_grid);
      tomogram_frame_ = tomogram_frame;
      tomogram_ready_ = !ground_grid_.empty();
    }

    RCLCPP_INFO(
      get_logger(), "Loaded traversable tomogram surface: %zu/%zu points in %zu XY cells, frame=%s",
      accepted, cloud.size(), cell_count, tomogram_frame.c_str());
  }

  bool isGroundPoint(
    float x, float y, float z, const GroundGrid & ground_grid,
    double grid_resolution, double xy_tolerance,
    double z_min_offset, double z_max_offset,
    bool keep_without_surface) const
  {
    const CellKey center = cellKey(x, y, grid_resolution);
    const int search_radius = static_cast<int>(
      std::ceil(xy_tolerance / grid_resolution));
    const float xy_tolerance_sq = static_cast<float>(
      xy_tolerance * xy_tolerance);

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
          if (delta_z >= static_cast<float>(z_min_offset) &&
            delta_z <= static_cast<float>(z_max_offset))
          {
            return true;
          }
        }
      }
    }
    return !keep_without_surface && !has_nearby_surface;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud has no frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped base_transform;
    if (!lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, base_transform)) {
      return;
    }

    geometry_msgs::msg::TransformStamped map_transform;
    const bool map_transform_ok = lookupTransform(
      map_frame_, msg->header.frame_id, msg->header.stamp, map_transform);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

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
    size_t height_candidates = 0;
    size_t tomogram_removed = 0;
    size_t kept_points = 0;
    GroundGrid empty_grid;

    std::lock_guard<std::mutex> lock(ground_mutex_);
    const bool tomogram_filter_ready =
      enabled_ && tomogram_ready_ && map_transform_ok && tomogram_frame_ == map_frame_;
    const GroundGrid & tomogram_grid = tomogram_filter_ready ? ground_grid_ : empty_grid;

    if (tomogram_ready_ && tomogram_frame_ != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Tomogram frame is '%s', expected '%s'; publishing unfiltered fallback scan",
        tomogram_frame_.c_str(), map_frame_.c_str());
    }

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const Eigen::Vector4f source_point(point.x, point.y, point.z, 1.0f);
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

      if (tomogram_filter_ready) {
        const Eigen::Vector4f map_point = cloud_to_map * source_point;
        if (isGroundPoint(
            map_point.x(), map_point.y(), map_point.z(), tomogram_grid,
            tomogram_grid_resolution_, ground_xy_tolerance_,
            ground_z_min_offset_, ground_z_max_offset_, keep_points_without_ground_))
        {
          tomogram_removed++;
          continue;
        }
      }
      size_t index = static_cast<size_t>(
        std::floor((static_cast<double>(angle) - angle_min_) / angle_increment_));
      if (index >= scan.ranges.size()) {
        index = scan.ranges.size() - 1;
      }
      if (range < scan.ranges[index]) {
        scan.ranges[index] = range;
      }
      kept_points++;
    }

    const size_t finite_beams = static_cast<size_t>(std::count_if(
        scan.ranges.begin(), scan.ranges.end(),
        [](float range) {return std::isfinite(range);}));
    scan_pub_->publish(scan);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Filtered scan: cloud=%zu candidates=%zu ground_removed=%zu "
      "kept_points=%zu finite_beams=%zu filter_ready=%d tomogram_ready=%d",
      cloud.size(), height_candidates, tomogram_removed,
      kept_points, finite_beams,
      static_cast<int>(tomogram_filter_ready), static_cast<int>(tomogram_filter_ready));
  }

  std::string input_cloud_topic_;
  std::string tomogram_topic_;
  std::string output_scan_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string tomogram_frame_;
  bool enabled_;

  double min_height_;
  double max_height_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  double tomogram_grid_resolution_;
  double ground_xy_tolerance_;
  double ground_z_min_offset_;
  double ground_z_max_offset_;
  double traversable_cost_max_;
  double transform_tolerance_;
  bool use_latest_transform_fallback_;
  bool keep_points_without_ground_;
  bool reload_tomogram_;
  int min_tomogram_points_;
  size_t scan_size_;

  std::mutex ground_mutex_;
  GroundGrid ground_grid_;
  bool tomogram_ready_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tomogram_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

}  // namespace traversability_layer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<traversability_layer::TomogramScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
