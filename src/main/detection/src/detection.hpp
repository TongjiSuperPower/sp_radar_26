#ifndef DETECTION_HPP
#define DETECTION_HPP

#include "detector_manager.hpp"
#include "tools/timer.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>


struct TimedBBoxArray {
    std::vector<std::vector<float>> bboxes;
    rclcpp::Time timestamp;
    double relative_time; 
};

class Detection : public rclcpp::Node
{
public:
    Detection();
    ~Detection() = default;  

private:    

    void Detecter(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg);
    void filteredCallback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg,
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filteredCallback2(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filteredCallback3(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg);
    std::vector<std::vector<float>> bboxcreater(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg); 
    
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription3_;

    bool initial = false;

    float average_fps;
    float passtime;
    std::shared_ptr<DetectorManager> detector_manager_;  
    std::shared_ptr<tools::Timer> timer_;  
    cv::Mat oldimage;
    cv_bridge::CvImagePtr cv_ptr;   
    int frame_count = 0;
    float total_duration;
    cv::Mat camera_matrix_, distort_coeffs_, pointcloud_img_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    geometry_msgs::msg::TransformStamped transform_L2C_, transform_C2L_, transform_L2M_;

    rclcpp::Time first_lidar_time_;
    rclcpp::Time first_image_time_;
    bool first_lidar_received_ = false;
    bool first_image_received_ = false;

    rclcpp::Publisher<radar_msgs::msg::CarBbox>::SharedPtr carbbox_publisher_;

    std::deque<TimedBBoxArray> bbox_cache_;
    std::mutex cache_mutex_;
    size_t max_cache_size_ = 10;
};

#endif