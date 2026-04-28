#ifndef SP_DETECTOR_MANAGER_HPP
#define SP_DETECTOR_MANAGER_HPP

#include <iostream>
#include <vector>
#include <thread>
#include <stack>
#include <mutex>
#include <future>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/car_bbox.hpp>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> // <--- 确保包含这个头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "deploy/vision/inference.hpp"
#include "deploy/vision/result.hpp"
#include "tools/timer.hpp"
#include "tracker.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>



struct armor_result
{
    cv::Mat detcted_img;
    int class_id;
    float class_score;
};

class DetectorManager 
{
public:
    DetectorManager(int armor_detector_num, rclcpp::Clock::SharedPtr clock = nullptr);
    ~DetectorManager();
    radar_msgs::msg::CarBbox detect_once(cv::Mat &image, float elapsed, float display_fps);
    void set_timer(std::shared_ptr<tools::Timer> timer);
    std::future<armor_result> submit_car(cv::Mat &img);
    armor_result process_armor(cv::Mat &img, size_t id);
    void set_thread(size_t id);
    void filteredDetect(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filteredDetect2(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filteredDetect3(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg,
        std::vector<std::vector<float>>& goodbboxes);
    std::vector<std::vector<float>> filteredDetect3Helper(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    geometry_msgs::msg::TransformStamped tum_to_transform_stamped(std::vector<double> TUM);
    geometry_msgs::msg::TransformStamped inverse_transform(
        const geometry_msgs::msg::TransformStamped& transform);
    std::vector<std::vector<float>> pointclouds_to_image(const pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Mat& img);

        // void get_param(std::string &save_folder, bool &use_camera, std::string &vedio_path, std::string &camera_config_file);

private:
    cv::Rect get_rect(cv::Mat &img, deploy::Box &bbox);
    rclcpp::Clock::SharedPtr clock_; //try to get time
    void transform_point_cloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const geometry_msgs::msg::TransformStamped &transform,
        pcl::PointCloud<pcl::PointXYZ> &transformed_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr project(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz);

    std::vector<cv::Point2f> points_list;
    std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> points_list_;
    int accumulate_frame = 2;
    
    std::vector<std::vector<float>> bbox_list;
    cv::Point2f top_left, bottom_right;
    
    cv::Mat cv_image_;

    std::vector<int> i_list;
    size_t num_threads_;
    std::queue<std::pair<cv::Mat, std::promise<armor_result>>> tasks_;
    std::vector<std::thread> threads_;
    std::mutex tasks_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_;

    std::mutex mtx_;
    double max_distance_ = 0;
    int debug_flag_;
    geometry_msgs::msg::TransformStamped transform_L2C_, transform_C2L_, transform_L2M_;


    std::vector<std::shared_ptr<deploy::DeployDet>> detectors_;
    std::shared_ptr<tools::Timer> timer_;
    //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    rclcpp::Publisher<radar_msgs::msg::CarBbox>::SharedPtr carbbox_publisher_;

    double camera_time_;
    double lidar_time_;
    cv::Mat camera_matrix_, distort_coeffs_, pointcloud_img_;
    bool outside; 

    radar_msgs::msg::CarBbox detect_armors_on_bboxes(cv::Mat &img,
    const std::vector<std::vector<float>>& bboxes);
    std::shared_ptr<TrackerManager> tracker_manager_;
};
#endif