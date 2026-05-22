/**
 * laserPosegraphOptimization.cpp - ROS2 Version
 * 
 * Ported from ROS1 to ROS2 Humble
 * Original: FAST_LIO_LC PGO module
 */

#include <fstream>
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <chrono>
#include <functional>
#include <memory>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "slam_mapping/common.h"
#include "slam_mapping/tic_toc.h"
#include "scancontext/Scancontext.h"

using namespace gtsam;
using std::cout;
using std::endl;

// ============================================================
// LaserPGO Node Class
// ============================================================
class LaserPGONode : public rclcpp::Node
{
public:
    LaserPGONode() : Node("laserPGO")
    {
        // 初始化
        initParameters();
        initNoises();
        initPublishers();
        initSubscribers();
        
        // 初始化 ISAM2
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam_ = new gtsam::ISAM2(parameters);
        
        // 初始化 ScanContext
        scManager_.setSCdistThres(scDistThres_);
        scManager_.setMaximumRadius(scMaximumRadius_);
        
        // 初始化滤波器
        float filter_size = 0.4f;
        downSizeFilterScancontext_.setLeafSize(filter_size, filter_size, filter_size);
        downSizeFilterICP_.setLeafSize(filter_size, filter_size, filter_size);
        downSizeFilterMapPGO_.setLeafSize(mapVizFilterSize_, mapVizFilterSize_, mapVizFilterSize_);
        
        // 初始化文件保存
        initFileSave();
        
        // 启动处理线程
        posegraphThread_ = std::thread(&LaserPGONode::process_pg, this);
        lcDetectionThread_ = std::thread(&LaserPGONode::process_lcd, this);
        icpThread_ = std::thread(&LaserPGONode::process_icp, this);
        isamThread_ = std::thread(&LaserPGONode::process_isam, this);
        vizMapThread_ = std::thread(&LaserPGONode::process_viz_map, this);
        
        RCLCPP_INFO(this->get_logger(), "LaserPGO node initialized successfully!");
    }
    
~LaserPGONode()
    {
        // 设置停止标志
        stopThreads_ = true;
        
        // 等待线程结束
        if (posegraphThread_.joinable()) posegraphThread_.join();
        if (lcDetectionThread_.joinable()) lcDetectionThread_.join();
        if (icpThread_.joinable()) icpThread_.join();
        if (isamThread_.joinable()) isamThread_.join();
        if (vizMapThread_.joinable()) vizMapThread_.join();
        
        // ============================================================
        // 【新增】：在节点关闭(Ctrl+C)时自动保存 SC 数据库和全局点云地图！
        // ============================================================
        saveOutputs();
        // ============================================================

        // 关闭文件流
        if (pgTimeSaveStream_.is_open()) {
            pgTimeSaveStream_.close();
        }
        
        delete isam_;
    }

private:
    // ============================================================
    // 参数初始化
    // ============================================================
    void initParameters()
    {
        // 声明并获取参数
        this->declare_parameter<std::string>("save_directory", "/tmp/fast_lio_lc/");
        this->declare_parameter<double>("keyframe_meter_gap", 2.0);
        this->declare_parameter<double>("keyframe_deg_gap", 10.0);
        this->declare_parameter<double>("sc_dist_thres", 0.2);
        this->declare_parameter<double>("sc_max_radius", 80.0);
        this->declare_parameter<double>("historyKeyframeSearchRadius", 10.0);
        this->declare_parameter<double>("historyKeyframeSearchTimeDiff", 30.0);
        this->declare_parameter<int>("historyKeyframeSearchNum", 25);
        this->declare_parameter<double>("loopNoiseScore", 0.5);
        this->declare_parameter<int>("graphUpdateTimes", 2);
        this->declare_parameter<double>("loopFitnessScoreThreshold", 0.3);
        this->declare_parameter<double>("speedFactor", 1.0);
        this->declare_parameter<double>("loopClosureFrequency", 2.0);
        this->declare_parameter<double>("graphUpdateFrequency", 1.0);
        this->declare_parameter<double>("vizmapFrequency", 0.1);
        this->declare_parameter<double>("vizPathFrequency", 10.0);
        this->declare_parameter<double>("mapviz_filter_size", 0.4);
        
        // 获取参数值
        saveDirectory_ = this->get_parameter("save_directory").as_string();
        if (!saveDirectory_.empty() && saveDirectory_.back() != '/')
        {
            saveDirectory_ += "/";
        }
        keyframeMeterGap_ = this->get_parameter("keyframe_meter_gap").as_double();
        keyframeDegGap_ = this->get_parameter("keyframe_deg_gap").as_double();
        keyframeRadGap_ = deg2rad(keyframeDegGap_);
        scDistThres_ = this->get_parameter("sc_dist_thres").as_double();
        scMaximumRadius_ = this->get_parameter("sc_max_radius").as_double();
        historyKeyframeSearchRadius_ = this->get_parameter("historyKeyframeSearchRadius").as_double();
        historyKeyframeSearchTimeDiff_ = this->get_parameter("historyKeyframeSearchTimeDiff").as_double();
        historyKeyframeSearchNum_ = this->get_parameter("historyKeyframeSearchNum").as_int();
        loopNoiseScore_ = this->get_parameter("loopNoiseScore").as_double();
        graphUpdateTimes_ = this->get_parameter("graphUpdateTimes").as_int();
        loopFitnessScoreThreshold_ = this->get_parameter("loopFitnessScoreThreshold").as_double();
        speedFactor_ = this->get_parameter("speedFactor").as_double();
        loopClosureFrequency_ = this->get_parameter("loopClosureFrequency").as_double() * speedFactor_;
        graphUpdateFrequency_ = this->get_parameter("graphUpdateFrequency").as_double() * speedFactor_;
        vizmapFrequency_ = this->get_parameter("vizmapFrequency").as_double() * speedFactor_;
        vizPathFrequency_ = this->get_parameter("vizPathFrequency").as_double() * speedFactor_;
        mapVizFilterSize_ = this->get_parameter("mapviz_filter_size").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "  save_directory: %s", saveDirectory_.c_str());
        RCLCPP_INFO(this->get_logger(), "  keyframe_meter_gap: %.2f", keyframeMeterGap_);
        RCLCPP_INFO(this->get_logger(), "  loopClosureFrequency: %.2f", loopClosureFrequency_);
    }
    
    void initFileSave()
    {
        pgKITTIformat_ = saveDirectory_ + "optimized_poses.txt";
        odomKITTIformat_ = saveDirectory_ + "odom_poses.txt";
        pgScansDirectory_ = saveDirectory_ + "Scans/";
        
        // 创建目录
        std::string cmd1 = "mkdir -p " + saveDirectory_;
        std::string cmd2 = "rm -rf " + pgScansDirectory_;
        std::string cmd3 = "mkdir -p " + pgScansDirectory_;
        
        auto ret1 = system(cmd1.c_str());
        auto ret2 = system(cmd2.c_str());
        auto ret3 = system(cmd3.c_str());
        (void)ret1; (void)ret2; (void)ret3;
        
        pgTimeSaveStream_.open(saveDirectory_ + "times.txt", std::fstream::out);
        pgTimeSaveStream_.precision(std::numeric_limits<double>::max_digits10);
    }

    // ============================================================
    // 【新增】保存 SC 数据库 (为后续重定位准备)
    // ============================================================
    void saveSCDatabase(const std::string& filename)
    {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SC database file!");
            return;
        }

        mKF_.lock(); 
        int num_frames = keyframePosesUpdated_.size();
        int sc_size = scManager_.polarcontexts_.size();
        int save_num = std::min(num_frames, sc_size);

        // 第一行记录总帧数
        ofs << save_num << "\n";
        for (int i = 0; i < save_num; ++i) {
            // 写入 3D 位姿 (x y z roll pitch yaw)
            Pose6D p = keyframePosesUpdated_[i];
            ofs << i << " " << p.x << " " << p.y << " " << p.z << " " 
                << p.roll << " " << p.pitch << " " << p.yaw << "\n";
            
            // 写入 2D ScanContext 特征矩阵 (通常是 20x60)
            Eigen::MatrixXd sc = scManager_.polarcontexts_[i];
            ofs << sc.rows() << " " << sc.cols() << "\n";
            for (int r = 0; r < sc.rows(); ++r) {
                for (int c = 0; c < sc.cols(); ++c) {
                    ofs << sc(r, c) << " ";
                }
                ofs << "\n";
            }
        }
        mKF_.unlock();
        ofs.close();
        RCLCPP_INFO(this->get_logger(), "⭐ SC Database saved to %s (Total %d frames)", filename.c_str(), save_num);
    }

    bool saveOutputs()
    {
        const std::string sc_database = saveDirectory_ + "sc_database.txt";
        const std::string global_map = saveDirectory_ + "global_map.pcd";

        saveSCDatabase(sc_database);

        if (recentIdxUpdated_ > 1) {
            pubMap();
        }

        if (laserCloudMapPGO_->empty()) {
            RCLCPP_WARN(this->get_logger(), "PGO map is empty, skip writing %s", global_map.c_str());
            return false;
        }

        const int ret = pcl::io::savePCDFileBinary(global_map, *laserCloudMapPGO_);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save %s", global_map.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "⭐ Global Map saved to %s", global_map.c_str());
        return true;
    }
    
    // ============================================================
    // 噪声模型初始化
    // ============================================================
    void initNoises()
    {
        gtsam::Vector priorNoiseVector6(6);
        priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
        priorNoise_ = noiseModel::Diagonal::Variances(priorNoiseVector6);

        gtsam::Vector odomNoiseVector6(6);
        odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
        odomNoise_ = noiseModel::Diagonal::Variances(odomNoiseVector6);

        gtsam::Vector robustNoiseVector6(6);
        robustNoiseVector6 << loopNoiseScore_, loopNoiseScore_, loopNoiseScore_, 
                             loopNoiseScore_, loopNoiseScore_, loopNoiseScore_;
        robustLoopNoise_ = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

        double bigNoiseTolerentToXY = 1000000000.0;
        double gpsAltitudeNoiseScore = 250.0;
        gtsam::Vector robustNoiseVector3(3);
        robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore;
        robustGPSNoise_ = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
    }
    
    // ============================================================
    // 发布者初始化
    // ============================================================
    void initPublishers()
    {
        pubOdomAftPGO_ = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
        pubPathAftPGO_ = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
        pubMapAftPGO_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);
        
        pubLoopScanLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
        pubLoopSubmapLocal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);
        pubLoopScanLocalRegisted_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local_registed", 100);
        
        pubLoopConstraintEdge_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 1);
        pubKeyFramesId_ = this->create_publisher<std_msgs::msg::Header>("/key_frames_ids", 10);
        pubOdomRepubVerifier_ = this->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);
        
        // TF broadcaster
        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        saveMapSrv_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_pgo_map",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                const bool ok = saveOutputs();
                response->success = ok;
                response->message =
                    ok
                        ? "Saved global_map.pcd and sc_database.txt to " + saveDirectory_
                        : "Saved sc_database.txt, but global_map.pcd is empty or failed to write.";
            });
    }
    
    // ============================================================
    // 订阅者初始化
    // ============================================================
    void initSubscribers()
    {
        subLaserCloudFullRes_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_cloud_registered_local", 100,
            std::bind(&LaserPGONode::laserCloudFullResHandler, this, std::placeholders::_1));
            
        subLaserOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", 100,
            std::bind(&LaserPGONode::laserOdometryHandler, this, std::placeholders::_1));
            
        subGPS_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 100,
            std::bind(&LaserPGONode::gpsHandler, this, std::placeholders::_1));
    }
    
    // ============================================================
    // 回调函数
    // ============================================================
    void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        mBuf_.lock();
        odometryBuf_.push(msg);
        mBuf_.unlock();
    }
    
    void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        mBuf_.lock();
        fullResBuf_.push(msg);
        mBuf_.unlock();
    }
    
    void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (useGPS_) {
            mBuf_.lock();
            gpsBuf_.push(msg);
            mBuf_.unlock();
        }
    }
    
    // ============================================================
    // 工具函数
    // ============================================================
    std::string padZeros(int val, int num_digits = 6)
    {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
        return out.str();
    }
    
    gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), 
                           gtsam::Point3(p.x, p.y, p.z));
    }
    
    Pose6D getOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        auto tx = odom->pose.pose.position.x;
        auto ty = odom->pose.pose.position.y;
        auto tz = odom->pose.pose.position.z;

        // 使用 tf2 进行四元数转欧拉角
        tf2::Quaternion q(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        Pose6D pose;
        pose.x = tx;
        pose.y = ty;
        pose.z = tz;
        pose.roll = roll;
        pose.pitch = pitch;
        pose.yaw = yaw;
        pose.seq = 0;  // ROS2 中 seq 已废弃，设为 0
        
        return pose;
    }
    
    Pose6D diffTransformation(const Pose6D& p1, const Pose6D& p2)
    {
        Eigen::Affine3f SE3_p1 = pcl::getTransformation(p1.x, p1.y, p1.z, p1.roll, p1.pitch, p1.yaw);
        Eigen::Affine3f SE3_p2 = pcl::getTransformation(p2.x, p2.y, p2.z, p2.roll, p2.pitch, p2.yaw);
        Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
        Eigen::Affine3f SE3_delta;
        SE3_delta.matrix() = SE3_delta0;
        float dx, dy, dz, droll, dpitch, dyaw;
        pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

        Pose6D dtf;
        dtf.x = double(std::abs(dx));
        dtf.y = double(std::abs(dy));
        dtf.z = double(std::abs(dz));
        dtf.roll = double(std::abs(droll));
        dtf.pitch = double(std::abs(dpitch));
        dtf.yaw = double(std::abs(dyaw));
        dtf.seq = 0;
        
        return dtf;
    }
    
    pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr& cloudIn, const Pose6D& tf)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
        
        int numberOfCores = 16;
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto& pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }
    
    Eigen::Affine3f Pose6dToAffine3f(const Pose6D& pose)
    {
        return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    }
    
    gtsam::Pose3 Pose6dTogtsamPose3(const Pose6D& pose)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch), double(pose.yaw)),
                            gtsam::Point3(double(pose.x), double(pose.y), double(pose.z)));
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(const std::vector<Pose6D>& vectorPose6d)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : vectorPose6d) {
            res->points.emplace_back(p.x, p.y, p.z);
        }
        return res;
    }
    
    // ============================================================
    // 文件保存函数
    // ============================================================
    void saveOdometryVerticesKITTIformat(const std::string& filename)
    {
        std::fstream stream(filename.c_str(), std::fstream::out);
        for (const auto& pose6d : keyframePoses_) {
            gtsam::Pose3 pose = Pose6DtoGTSAMPose3(pose6d);
            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto col1 = R.column(1);
            auto col2 = R.column(2);
            auto col3 = R.column(3);

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
                   << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
                   << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
        }
    }
    
    void saveOptimizedVerticesKITTIformat(const gtsam::Values& estimates, const std::string& filename)
    {
        std::fstream stream(filename.c_str(), std::fstream::out);
        for (const auto& key_value : estimates) {
            auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
            if (!p) continue;

            const Pose3& pose = p->value();
            Point3 t = pose.translation();
            Rot3 R = pose.rotation();
            auto col1 = R.column(1);
            auto col2 = R.column(2);
            auto col3 = R.column(3);

            stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
                   << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
                   << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
        }
    }
    
    // ============================================================
    // 🔙 绝对还原的发布函数：绝不碰 TF 和坐标系名字！
    // ============================================================
    void pubPath()
    {
        nav_msgs::msg::Odometry odomAftPGO;
        nav_msgs::msg::Path pathAftPGO;
        pathAftPGO.header.frame_id = "camera_init";
        
        mKF_.lock();
        for (int node_idx = 0; node_idx < recentIdxUpdated_; node_idx++)
        {
            const Pose6D& pose_est = keyframePosesUpdated_.at(node_idx);

            nav_msgs::msg::Odometry odomAftPGOthis;
            odomAftPGOthis.header.frame_id = "camera_init";
            odomAftPGOthis.child_frame_id = "aft_pgo";
            odomAftPGOthis.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.at(node_idx) * 1e9));
            odomAftPGOthis.pose.pose.position.x = pose_est.x;
            odomAftPGOthis.pose.pose.position.y = pose_est.y;
            odomAftPGOthis.pose.pose.position.z = pose_est.z;
            
            tf2::Quaternion q;
            q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
            odomAftPGOthis.pose.pose.orientation.x = q.x();
            odomAftPGOthis.pose.pose.orientation.y = q.y();
            odomAftPGOthis.pose.pose.orientation.z = q.z();
            odomAftPGOthis.pose.pose.orientation.w = q.w();
            
            odomAftPGO = odomAftPGOthis;

            geometry_msgs::msg::PoseStamped poseStampAftPGO;
            poseStampAftPGO.header = odomAftPGOthis.header;
            poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

            pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
            pathAftPGO.header.frame_id = "camera_init";
            pathAftPGO.poses.push_back(poseStampAftPGO);
        }
        mKF_.unlock();
        
        pubOdomAftPGO_->publish(odomAftPGO);
        pubPathAftPGO_->publish(pathAftPGO);

        // 🔙 还原：这里原本只是把 aft_pgo 挂在 camera_init 下面显示用，绝没有计算复杂的逆矩阵！
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = odomAftPGO.header.stamp;
        transformStamped.header.frame_id = "camera_init";
        transformStamped.child_frame_id = "aft_pgo";
        transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
        transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
        transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
        transformStamped.transform.rotation = odomAftPGO.pose.pose.orientation;
        tfBroadcaster_->sendTransform(transformStamped);
    }
    
    void pubMap()
    {
        int SKIP_FRAMES = 1;
        int counter = 0;

        laserCloudMapPGO_->clear();

        mKF_.lock();
        for (int node_idx = 0; node_idx < recentIdxUpdated_; node_idx++) {
            if (counter % SKIP_FRAMES == 0) {
                *laserCloudMapPGO_ += *local2global(keyframeLaserClouds_[node_idx], keyframePosesUpdated_[node_idx]);
            }
            counter++;
        }
        mKF_.unlock();

        downSizeFilterMapPGO_.setInputCloud(laserCloudMapPGO_);
        downSizeFilterMapPGO_.filter(*laserCloudMapPGO_);

        sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
        pcl::toROSMsg(*laserCloudMapPGO_, laserCloudMapPGOMsg);
        laserCloudMapPGOMsg.header.frame_id = "camera_init"; // 🔙 还原为 camera_init
        laserCloudMapPGOMsg.header.stamp = this->now();
        pubMapAftPGO_->publish(laserCloudMapPGOMsg);
    }
    
    void visualizeLoopClosure()
    {
        if (loopIndexContainer_.empty())
            return;
        
        visualization_msgs::msg::MarkerArray markerArray;
        
        // 闭环顶点
        visualization_msgs::msg::Marker markerNode;
        markerNode.header.frame_id = "camera_init"; // 🔙 还原为 camera_init
        markerNode.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.back() * 1e9));
        markerNode.action = visualization_msgs::msg::Marker::ADD;
        markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3;
        markerNode.scale.y = 0.3;
        markerNode.scale.z = 0.3;
        markerNode.color.r = 0;
        markerNode.color.g = 0.8;
        markerNode.color.b = 1;
        markerNode.color.a = 1;
        
        // 闭环边
        visualization_msgs::msg::Marker markerEdge;
        markerEdge.header.frame_id = "camera_init"; // 🔙 还原为 camera_init
        markerEdge.header.stamp = rclcpp::Time(static_cast<int64_t>(keyframeTimes_.back() * 1e9));
        markerEdge.action = visualization_msgs::msg::Marker::ADD;
        markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9;
        markerEdge.color.g = 0.9;
        markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer_.begin(); it != loopIndexContainer_.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::msg::Point p;
            p.x = keyframePosesUpdated_[key_cur].x;
            p.y = keyframePosesUpdated_[key_cur].y;
            p.z = keyframePosesUpdated_[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = keyframePosesUpdated_[key_pre].x;
            p.y = keyframePosesUpdated_[key_pre].y;
            p.z = keyframePosesUpdated_[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge_->publish(markerArray);
    }


    // ============================================================
    // 优化相关函数
    // ============================================================
    void updatePoses()
    {
        mKF_.lock();
        for (int node_idx = 0; node_idx < int(isamCurrentEstimate_.size()); node_idx++)
        {
            Pose6D& p = keyframePosesUpdated_[node_idx];
            p.x = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).translation().x();
            p.y = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).translation().y();
            p.z = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).translation().z();
            p.roll = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).rotation().roll();
            p.pitch = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).rotation().pitch();
            p.yaw = isamCurrentEstimate_.at<gtsam::Pose3>(node_idx).rotation().yaw();
        }
        mKF_.unlock();

        mtxRecentPose_.lock();
        const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate_.at<gtsam::Pose3>(int(isamCurrentEstimate_.size()) - 1);
        recentOptimizedX_ = lastOptimizedPose.translation().x();
        recentOptimizedY_ = lastOptimizedPose.translation().y();
        recentIdxUpdated_ = int(keyframePosesUpdated_.size()) - 1;
        mtxRecentPose_.unlock();
    }
    
    void runISAM2opt()
    {
        isam_->update(gtSAMgraph_, initialEstimate_);
        isam_->update();
        for (int i = graphUpdateTimes_; i > 0; --i) {
            isam_->update();
        }
        
        gtSAMgraph_.resize(0);
        initialEstimate_.clear();

        isamCurrentEstimate_ = isam_->calculateEstimate();
        updatePoses();
        pubPath();
    }
    
    // ============================================================
    // 回环检测相关函数
    // ============================================================
    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        nearKeyframes->clear();
        int cloudSize = keyframeLaserClouds_.size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;
            mKF_.lock();
            *nearKeyframes += *local2global(keyframeLaserClouds_[keyNear], keyframePosesUpdated_[keyNear]);
            mKF_.unlock();
        }

        if (nearKeyframes->empty())
            return;

        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP_.setInputCloud(nearKeyframes);
        downSizeFilterICP_.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }
    
    gtsam::Pose3 doICPVirtualRelative(int loop_kf_idx, int curr_kf_idx)
    {
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
        
        loopFindNearKeyframes(cureKeyframeCloud, curr_kf_idx, 0);
        loopFindNearKeyframes(targetKeyframeCloud, loop_kf_idx, historyKeyframeSearchNum_);

        // 发布调试点云
        sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
        pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
        cureKeyframeCloudMsg.header.frame_id = "camera_init";
        cureKeyframeCloudMsg.header.stamp = this->now();
        pubLoopScanLocal_->publish(cureKeyframeCloudMsg);

        sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
        pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
        targetKeyframeCloudMsg.header.frame_id = "camera_init";
        targetKeyframeCloudMsg.header.stamp = this->now();
        pubLoopSubmapLocal_->publish(targetKeyframeCloudMsg);

        // ICP 设置
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(150);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(targetKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        sensor_msgs::msg::PointCloud2 cureKeyframeCloudRegMsg;
        pcl::toROSMsg(*unused_result, cureKeyframeCloudRegMsg);
        cureKeyframeCloudRegMsg.header.frame_id = "camera_init";
        cureKeyframeCloudRegMsg.header.stamp = this->now();
        pubLoopScanLocalRegisted_->publish(cureKeyframeCloudRegMsg);
        
        if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold_) {
            RCLCPP_INFO(this->get_logger(), "[SC loop] ICP fitness test failed (%.3f > %.3f). Reject this SC loop.",
                       icp.getFitnessScore(), loopFitnessScoreThreshold_);
            return gtsam::Pose3();
        } else {
            RCLCPP_INFO(this->get_logger(), "[SC loop] ICP fitness test passed (%.3f < %.3f). Add this SC loop.",
                       icp.getFitnessScore(), loopFitnessScoreThreshold_);
        }

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        Eigen::Affine3f tWrong = Pose6dToAffine3f(keyframePosesUpdated_[curr_kf_idx]);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose6dTogtsamPose3(keyframePosesUpdated_[loop_kf_idx]);

        return poseFrom.between(poseTo);
    }
    
    bool detectLoopClosureDistance(int* loopKeyCur, int* loopKeyPre)
    {
        auto it = loopIndexContainer_.find(*loopKeyCur);
        if (it != loopIndexContainer_.end())
            return false;

        pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D = vector2pc(keyframePoses_);
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses_->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses_->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius_, 
                                            pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (size_t i = 0; i < pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (std::abs(keyframeTimes_[id] - keyframeTimes_[*loopKeyCur]) > historyKeyframeSearchTimeDiff_)
            {
                *loopKeyPre = id;
                break;
            }
        }

        if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre)
            return false;

        return true;
    }

    // ============================================================
    // 真正的 ScanContext 回环检测 
    // ============================================================
    void performSCLoopClosure()
    {
        if (int(keyframePoses_.size()) < scManager_.NUM_EXCLUDE_RECENT) 
            return;

        int curr_node_idx = keyframePoses_.size() - 1;

        auto detectResult = scManager_.detectLoopClosureID(); 
        int SCclosestHistoryFrameID = detectResult.first;
        
        if (SCclosestHistoryFrameID != -1) { 
            const int prev_node_idx = SCclosestHistoryFrameID;

            // 保留真实的物理时间锁，防止刚起步的几秒钟内自己和自己互怼
            double time_diff = std::abs(keyframeTimes_[curr_node_idx] - keyframeTimes_[prev_node_idx]);
            if (time_diff < historyKeyframeSearchTimeDiff_) {
                return;
            }

            RCLCPP_INFO(this->get_logger(), "⭐ ScanContext find loop! - node %d and %d", prev_node_idx, curr_node_idx);
            
            mBuf_.lock();
            scLoopICPBuf_.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
            mBuf_.unlock();
        }
    }
    
    void performRSLoopClosure()
    {
        if (keyframePoses_.empty())
            return;

        int loopKeyCur = keyframePoses_.size() - 1;
        int loopKeyPre = -1;
        
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre)) {
            RCLCPP_INFO(this->get_logger(), "Loop detected! - between %d and %d", loopKeyPre, loopKeyCur);
            mBuf_.lock();
            scLoopICPBuf_.push(std::pair<int, int>(loopKeyPre, loopKeyCur));
            loopIndexContainer_[loopKeyCur] = loopKeyPre;
            mBuf_.unlock();
        }
    }
    
    // ============================================================
    // 处理线程
    // ============================================================
    void process_pg()
    {
        while (rclcpp::ok() && !stopThreads_)
        {
            while (!odometryBuf_.empty() && !fullResBuf_.empty())
            {
                mBuf_.lock();
                while (!odometryBuf_.empty() && 
                       rclcpp::Time(odometryBuf_.front()->header.stamp).seconds() < 
                       rclcpp::Time(fullResBuf_.front()->header.stamp).seconds())
                    odometryBuf_.pop();
                    
                if (odometryBuf_.empty())
                {
                    mBuf_.unlock();
                    break;
                }

                timeLaserOdometry_ = rclcpp::Time(odometryBuf_.front()->header.stamp).seconds();
                timeLaser_ = rclcpp::Time(fullResBuf_.front()->header.stamp).seconds();

                pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
                pcl::fromROSMsg(*fullResBuf_.front(), *thisKeyFrame);
                fullResBuf_.pop();

                Pose6D pose_curr = getOdom(odometryBuf_.front());
                odometryBuf_.pop();

                // GPS 处理
                double eps = 0.1;
                while (!gpsBuf_.empty()) {
                    auto thisGPS = gpsBuf_.front();
                    auto thisGPSTime = rclcpp::Time(thisGPS->header.stamp).seconds();
                    if (std::abs(thisGPSTime - timeLaserOdometry_) < eps) {
                        currGPS_ = thisGPS;
                        hasGPSforThisKF_ = true;
                        break;
                    } else {
                        hasGPSforThisKF_ = false;
                    }
                    gpsBuf_.pop();
                }
                mBuf_.unlock();

                // 关键帧判断
                odom_pose_prev_ = odom_pose_curr_;
                odom_pose_curr_ = pose_curr;
                Pose6D dtf = diffTransformation(odom_pose_prev_, odom_pose_curr_);

                double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
                translationAccumulated_ += delta_translation;
                rotaionAccumulated_ += (dtf.roll + dtf.pitch + dtf.yaw);

                if (translationAccumulated_ > keyframeMeterGap_ || rotaionAccumulated_ > keyframeRadGap_) {
                    isNowKeyFrame_ = true;
                    translationAccumulated_ = 0.0;
                    rotaionAccumulated_ = 0.0;
                } else {
                    isNowKeyFrame_ = false;
                }

                if (!isNowKeyFrame_)
                    continue;

                if (!gpsOffsetInitialized_) {
                    if (hasGPSforThisKF_) {
                        gpsAltitudeInitOffset_ = currGPS_->altitude;
                        gpsOffsetInitialized_ = true;
                    }
                }
                
                // 1. 专门给 ScanContext 用的 0.4m 稀疏点云 (保持原样，不破坏查表逻辑)
                pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
                downSizeFilterScancontext_.setInputCloud(thisKeyFrame);
                downSizeFilterScancontext_.filter(*thisKeyFrameDS);

                // 2. 专门给全局地图相册存底片用的高密点云 (使用你 yaml 里的 mapVizFilterSize_)
                pcl::PointCloud<PointType>::Ptr thisKeyFrameMapDS(new pcl::PointCloud<PointType>());
                pcl::VoxelGrid<PointType> filterForMap; // 局部新建，防止多线程冲突
                filterForMap.setLeafSize(mapVizFilterSize_, mapVizFilterSize_, mapVizFilterSize_);
                filterForMap.setInputCloud(thisKeyFrame);
                filterForMap.filter(*thisKeyFrameMapDS);

                mKF_.lock();

                // 3. 致命修改点：把高密度的 thisKeyFrameMapDS 存进去地图相册，而不是 0.4m 的 thisKeyFrameDS！
                keyframeLaserClouds_.push_back(thisKeyFrameMapDS); 
                keyframePoses_.push_back(pose_curr);
                
                // 发布关键帧 ID
                std_msgs::msg::Header keyFrameHeader;
                keyFrameHeader.stamp = this->now();
                pubKeyFramesId_->publish(keyFrameHeader);
                
                keyframePosesUpdated_.push_back(pose_curr);
                keyframeTimes_.push_back(timeLaserOdometry_);
                
                // 4. SC 依然使用 0.4m 的稀疏点云去生成上下文，绝不破坏以前的字典匹配逻辑！
                scManager_.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);
                laserCloudMapPGORedraw_ = true;
                mKF_.unlock();

                const int prev_node_idx = keyframePoses_.size() - 2;
                const int curr_node_idx = keyframePoses_.size() - 1;
                
                if (!gtSAMgraphMade_) {
                    const int init_node_idx = 0;
                    gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses_.at(init_node_idx));

                    mtxPosegraph_.lock();
                    gtSAMgraph_.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise_));
                    initialEstimate_.insert(init_node_idx, poseOrigin);
                    mtxPosegraph_.unlock();

                    gtSAMgraphMade_ = true;
                    RCLCPP_INFO(this->get_logger(), "Posegraph prior node %d added", init_node_idx);
                } else {
                    gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses_.at(prev_node_idx));
                    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses_.at(curr_node_idx));

                    mtxPosegraph_.lock();
                    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, 
                                   poseFrom.between(poseTo), odomNoise_));

                    if (hasGPSforThisKF_) {
                        double curr_altitude_offseted = currGPS_->altitude - gpsAltitudeInitOffset_;
                        mtxRecentPose_.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX_, recentOptimizedY_, curr_altitude_offseted);
                        mtxRecentPose_.unlock();
                        gtSAMgraph_.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise_));
                        RCLCPP_INFO(this->get_logger(), "GPS factor added at node %d", curr_node_idx);
                    }
                    initialEstimate_.insert(curr_node_idx, poseTo);
                    mtxPosegraph_.unlock();

                    if (curr_node_idx % 100 == 0)
                        RCLCPP_INFO(this->get_logger(), "Posegraph odom node %d added.", curr_node_idx);
                }

                // 保存点云和时间戳
                std::string curr_node_idx_str = padZeros(curr_node_idx);
                pcl::io::savePCDFileBinary(pgScansDirectory_ + curr_node_idx_str + ".pcd", *thisKeyFrame);
                pgTimeSaveStream_ << timeLaser_ << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    
    void process_lcd()
    {
        auto rate_duration = std::chrono::duration<double>(1.0 / loopClosureFrequency_);
        while (rclcpp::ok() && !stopThreads_)
        {
            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
            // performRSLoopClosure();
            performSCLoopClosure();
            visualizeLoopClosure();
        }
    }
    
    void process_icp()
    {
        while (rclcpp::ok() && !stopThreads_)
        {
            while (!scLoopICPBuf_.empty())
            {
                if (scLoopICPBuf_.size() > 30) {
                    RCLCPP_WARN(this->get_logger(), 
                               "Too many loop closure candidates waiting for ICP... Consider reducing loopClosureFrequency");
                }

                mBuf_.lock();
                std::pair<int, int> loop_idx_pair = scLoopICPBuf_.front();
                scLoopICPBuf_.pop();
                mBuf_.unlock();

                const int prev_node_idx = loop_idx_pair.first;
                const int curr_node_idx = loop_idx_pair.second;
                auto relative_pose = doICPVirtualRelative(prev_node_idx, curr_node_idx);
                
                if (!relative_pose.equals(gtsam::Pose3())) {
                    mtxPosegraph_.lock();
                    gtSAMgraph_.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, 
                                   relative_pose, robustLoopNoise_));
                    mtxPosegraph_.unlock();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    
    void process_isam()
    {
        auto rate_duration = std::chrono::duration<double>(1.0 / graphUpdateFrequency_);
        while (rclcpp::ok() && !stopThreads_)
        {
            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
            if (gtSAMgraphMade_) {
                mtxPosegraph_.lock();
                runISAM2opt();
                mtxPosegraph_.unlock();

                saveOptimizedVerticesKITTIformat(isamCurrentEstimate_, pgKITTIformat_);
                saveOdometryVerticesKITTIformat(odomKITTIformat_);
            }
        }
    }
    
    void process_viz_map()
    {
        auto rate_duration = std::chrono::duration<double>(1.0 / vizmapFrequency_);
        while (rclcpp::ok() && !stopThreads_)
        {
            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(rate_duration));
            if (recentIdxUpdated_ > 1) {
                pubMap();
            }
        }
    }

private:
    // ============== 参数 ==============
    std::string saveDirectory_;
    double keyframeMeterGap_;
    double keyframeDegGap_;
    double keyframeRadGap_;
    double scDistThres_;
    double scMaximumRadius_;
    double historyKeyframeSearchRadius_;
    double historyKeyframeSearchTimeDiff_;
    int historyKeyframeSearchNum_;
    double loopNoiseScore_;
    int graphUpdateTimes_;
    double loopFitnessScoreThreshold_;
    double speedFactor_;
    double loopClosureFrequency_;
    double graphUpdateFrequency_;
    double vizmapFrequency_;
    double vizPathFrequency_;
    double mapVizFilterSize_;
    
    std::string pgKITTIformat_;
    std::string odomKITTIformat_;
    std::string pgScansDirectory_;
    std::fstream pgTimeSaveStream_;
    
    // ============== 发布者 ==============
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocalRegisted_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pubKeyFramesId_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveMapSrv_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    
    // ============== 订阅者 ==============
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloudFullRes_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGPS_;
    
    // ============== 数据缓存 ==============
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullResBuf_;
    std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gpsBuf_;
    std::queue<std::pair<int, int>> scLoopICPBuf_;
    
    // ============== 互斥锁 ==============
    std::mutex mBuf_;
    std::mutex mKF_;
    std::mutex mtxICP_;
    std::mutex mtxPosegraph_;
    std::mutex mtxRecentPose_;
    
    // ============== 里程计状态 ==============
    double translationAccumulated_ = 1000000.0;
    double rotaionAccumulated_ = 1000000.0;
    bool isNowKeyFrame_ = false;
    Pose6D odom_pose_prev_;
    Pose6D odom_pose_curr_;
    double timeLaserOdometry_ = 0.0;
    double timeLaser_ = 0.0;
    
    // ============== 关键帧数据 ==============
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds_;
    std::vector<Pose6D> keyframePoses_;
    std::vector<Pose6D> keyframePosesUpdated_;
    std::vector<double> keyframeTimes_;
    int recentIdxUpdated_ = 0;
    
    // ============== 回环检测 ==============
    std::map<int, int> loopIndexContainer_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses_{new pcl::KdTreeFLANN<pcl::PointXYZ>()};
    
    // ============== GTSAM ==============
    gtsam::NonlinearFactorGraph gtSAMgraph_;
    bool gtSAMgraphMade_ = false;
    gtsam::Values initialEstimate_;
    gtsam::ISAM2* isam_;
    gtsam::Values isamCurrentEstimate_;
    
    noiseModel::Diagonal::shared_ptr priorNoise_;
    noiseModel::Diagonal::shared_ptr odomNoise_;
    noiseModel::Base::shared_ptr robustLoopNoise_;
    noiseModel::Base::shared_ptr robustGPSNoise_;
    
    // ============== 滤波器 ==============
    pcl::VoxelGrid<PointType> downSizeFilterScancontext_;
    pcl::VoxelGrid<PointType> downSizeFilterICP_;
    pcl::VoxelGrid<PointType> downSizeFilterMapPGO_;
    
    // ============== ScanContext ==============
    SCManager scManager_;
    
    // ============== 点云地图 ==============
    pcl::PointCloud<PointType>::Ptr laserCloudMapPGO_{new pcl::PointCloud<PointType>()};
    bool laserCloudMapPGORedraw_ = true;
    
    // ============== GPS ==============
    bool useGPS_ = true;
    sensor_msgs::msg::NavSatFix::SharedPtr currGPS_;
    bool hasGPSforThisKF_ = false;
    bool gpsOffsetInitialized_ = false;
    double gpsAltitudeInitOffset_ = 0.0;
    double recentOptimizedX_ = 0.0;
    double recentOptimizedY_ = 0.0;
    
    // ============== 线程 ==============
    std::thread posegraphThread_;
    std::thread lcDetectionThread_;
    std::thread icpThread_;
    std::thread isamThread_;
    std::thread vizMapThread_;
    std::atomic<bool> stopThreads_{false};
};

// ============================================================
// main 函数
// ============================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LaserPGONode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
