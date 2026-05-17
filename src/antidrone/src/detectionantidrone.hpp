#ifndef DETECTION_ANTIDRONE_HPP
#define DETECTION_ANTIDRONE_HPP

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

//#include "deploy/vision/inference.hpp"
//#include "deploy/vision/result.hpp"
#include "tools/timer.hpp" 

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include "TensorRT-YOLO/include/trtyolo.hpp"


struct armor_result
{
    cv::Mat detcted_img;
    int class_id;
    float class_score;
};

class DetectionAntiDrone : public rclcpp::Node
{
public:
    DetectionAntiDrone();
    void detect_once(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg);
    std::future<armor_result> submit_car(cv::Mat &img);
    void set_thread(size_t id);
private:
    cv::Rect get_rect(cv::Mat &img, const trtyolo::Box &bbox);
    void draw_car_bbox(const radar_msgs::msg::CarBbox msg, cv::Mat& frame);
    void set_timer(std::shared_ptr<tools::Timer> timer);
    std::shared_ptr<trtyolo::DetectModel> model_;

    std::shared_ptr<tools::Timer> timer_;

    rclcpp::Clock::SharedPtr clock_; //try to get time
    
    std::vector<cv::Point2f> points_list;
    std::vector<std::vector<float>> bbox_list;
    cv::Point2f top_left, bottom_right;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<radar_msgs::msg::CarBbox>::SharedPtr carbbox_publisher_;
    
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



    std::vector<std::shared_ptr<trtyolo::DetectModel>> detectors_;
};
#endif