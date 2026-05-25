#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Geometry>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "scancontext/Scancontext.h"

using namespace std::chrono_literals;

struct HistoricalPose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct SCResult
{
    double score;
    int id;
    int yaw_shift;
    bool operator<(const SCResult &other) const { return score < other.score; }
};

class SCRelocalizer : public rclcpp::Node
{
public:
    SCRelocalizer() : Node("sc_relocalizer")
    {
        this->declare_parameter<std::string>("sc_db_path", "/tmp/sc_database.txt");
        this->declare_parameter<std::string>("scan_topic", "/cloud_registered_body_1");
        this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
        this->declare_parameter<std::string>("confidence_topic", "/localization_3d_confidence");
        this->declare_parameter<double>("match_threshold", 0.45);
        this->declare_parameter<double>("confidence_threshold", 0.55);
        this->declare_parameter<double>("publish_cooldown_sec", 15.0);
        this->declare_parameter<bool>("require_low_confidence", true);
        this->declare_parameter<int>("num_sectors", 60);
        this->declare_parameter<bool>("apply_leveling_rotation", true);
        this->declare_parameter<std::vector<double>>(
            "leveling_quaternion",
            {-0.00394028, 0.24367785, 0.00970223, 0.96979969});

        db_path_ = this->get_parameter("sc_db_path").as_string();
        match_threshold_ = this->get_parameter("match_threshold").as_double();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        publish_cooldown_sec_ = this->get_parameter("publish_cooldown_sec").as_double();
        require_low_confidence_ = this->get_parameter("require_low_confidence").as_bool();
        num_sectors_ = this->get_parameter("num_sectors").as_int();
        apply_leveling_rotation_ = this->get_parameter("apply_leveling_rotation").as_bool();
        const auto leveling_quat = this->get_parameter("leveling_quaternion").as_double_array();
        if (leveling_quat.size() == 4)
        {
            q_level_ = Eigen::Quaterniond(leveling_quat[3], leveling_quat[0], leveling_quat[1], leveling_quat[2]).normalized();
        }
        else
        {
            q_level_ = Eigen::Quaterniond::Identity();
        }

        if (!loadDatabase(db_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load SC database: %s", db_path_.c_str());
            return;
        }

        const auto scan_topic = this->get_parameter("scan_topic").as_string();
        const auto initialpose_topic = this->get_parameter("initialpose_topic").as_string();
        const auto confidence_topic = this->get_parameter("confidence_topic").as_string();

        pub_initial_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic, 1);
        sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            scan_topic, rclcpp::SensorDataQoS(),
            std::bind(&SCRelocalizer::scanCallback, this, std::placeholders::_1));
        sub_confidence_ = this->create_subscription<std_msgs::msg::Float32>(
            confidence_topic, 10,
            std::bind(&SCRelocalizer::confidenceCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "SC relocalizer ready. db=%s, scan_topic=%s, confidence_topic=%s, initialpose_topic=%s",
                    db_path_.c_str(), scan_topic.c_str(), confidence_topic.c_str(), initialpose_topic.c_str());
    }

private:
    bool loadDatabase(const std::string &path)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open())
        {
            return false;
        }

        int num_frames = 0;
        if (!(ifs >> num_frames) || num_frames <= 0)
        {
            return false;
        }

        db_poses_.reserve(num_frames);
        db_scs_.reserve(num_frames);
        for (int i = 0; i < num_frames; ++i)
        {
            int id = 0;
            HistoricalPose pose{};
            ifs >> id >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw;
            if (!ifs.good())
            {
                return false;
            }
            db_poses_.push_back(pose);

            int rows = 0;
            int cols = 0;
            ifs >> rows >> cols;
            if (!ifs.good() || rows <= 0 || cols <= 0)
            {
                return false;
            }
            Eigen::MatrixXd sc(rows, cols);
            for (int r = 0; r < rows; ++r)
            {
                for (int c = 0; c < cols; ++c)
                {
                    ifs >> sc(r, c);
                }
            }
            if (!ifs.good())
            {
                return false;
            }
            db_scs_.push_back(sc);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu historical poses and scan contexts", db_poses_.size());
        return true;
    }

    void confidenceCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        latest_confidence_ = static_cast<double>(msg->data);
        have_confidence_ = true;
    }

    bool inCooldown() const
    {
        if (!have_last_publish_time_)
        {
            return false;
        }
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_publish_time_).count();
        return elapsed < publish_cooldown_sec_;
    }

    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (db_scs_.empty())
        {
            return;
        }

        if (require_low_confidence_ && have_confidence_ && latest_confidence_ >= confidence_threshold_)
        {
            return;
        }

        if (inCooldown())
        {
            return;
        }

        pcl::PointCloud<SCPointType>::Ptr current_cloud(new pcl::PointCloud<SCPointType>());
        pcl::fromROSMsg(*msg, *current_cloud);
        if (current_cloud->empty())
        {
            return;
        }

        Eigen::MatrixXd current_sc = scManager_.makeScancontext(*current_cloud);
        std::vector<SCResult> results;
        results.reserve(db_scs_.size());

        for (size_t i = 0; i < db_scs_.size(); ++i)
        {
            auto sc_dist_result = scManager_.distanceBtnScanContext(current_sc, db_scs_[i]);
            results.push_back({sc_dist_result.first, static_cast<int>(i), sc_dist_result.second});
        }

        std::sort(results.begin(), results.end());
        if (results.empty())
        {
            return;
        }

        const SCResult &best = results.front();
        if (best.score >= match_threshold_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "SC relocalizer best score %.3f is above threshold %.3f",
                                 best.score, match_threshold_);
            return;
        }

        const HistoricalPose &p = db_poses_[best.id];
        const double sector_res = 2.0 * M_PI / std::max(1, num_sectors_);
        const double yaw_offset = static_cast<double>(best.yaw_shift) * sector_res;

        Eigen::Vector3d position(p.x, p.y, p.z);
        tf2::Quaternion q_raw;
        q_raw.setRPY(p.roll, p.pitch, p.yaw - yaw_offset);
        Eigen::Quaterniond q_hist(q_raw.w(), q_raw.x(), q_raw.y(), q_raw.z());

        if (apply_leveling_rotation_)
        {
            position = q_level_ * position;
            q_hist = (q_level_ * q_hist).normalized();
        }

        geometry_msgs::msg::PoseWithCovarianceStamped initialpose;
        initialpose.header.stamp = msg->header.stamp;
        initialpose.header.frame_id = "map";
        initialpose.pose.pose.position.x = position.x();
        initialpose.pose.pose.position.y = position.y();
        initialpose.pose.pose.position.z = position.z();
        initialpose.pose.pose.orientation.x = q_hist.x();
        initialpose.pose.pose.orientation.y = q_hist.y();
        initialpose.pose.pose.orientation.z = q_hist.z();
        initialpose.pose.pose.orientation.w = q_hist.w();

        initialpose.pose.covariance[0] = 0.25;
        initialpose.pose.covariance[7] = 0.25;
        initialpose.pose.covariance[35] = 0.15;

        pub_initial_pose_->publish(initialpose);
        last_publish_time_ = std::chrono::steady_clock::now();
        have_last_publish_time_ = true;

        RCLCPP_WARN(this->get_logger(),
                    "SC relocalizer published /initialpose: score=%.3f, id=%d, yaw_shift=%d, confidence=%.3f",
                    best.score, best.id, best.yaw_shift, latest_confidence_);
    }

    std::string db_path_;
    double match_threshold_ = 0.45;
    double confidence_threshold_ = 0.55;
    double publish_cooldown_sec_ = 15.0;
    bool require_low_confidence_ = true;
    int num_sectors_ = 60;
    bool apply_leveling_rotation_ = true;
    Eigen::Quaterniond q_level_ = Eigen::Quaterniond::Identity();

    double latest_confidence_ = 0.0;
    bool have_confidence_ = false;
    std::chrono::steady_clock::time_point last_publish_time_{};
    bool have_last_publish_time_ = false;

    SCManager scManager_;
    std::vector<HistoricalPose> db_poses_;
    std::vector<Eigen::MatrixXd> db_scs_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_confidence_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SCRelocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
