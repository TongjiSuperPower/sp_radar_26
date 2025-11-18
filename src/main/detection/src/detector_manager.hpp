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

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/car_bbox.hpp>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "deploy/vision/inference.hpp"
#include "deploy/vision/result.hpp"
#include "tools/timer.hpp"
#include "tracker.hpp"

struct armor_result
{
    cv::Mat detcted_img;
    int class_id;
    float class_score;
};

class DetectorManager : public rclcpp::Node
{
public:
    DetectorManager(int armor_detector_num);
    ~DetectorManager();
    void detect_once(cv::Mat &image);
    void set_timer(std::shared_ptr<tools::Timer> timer);
    std::future<armor_result> submit_car(cv::Mat &img);
    armor_result process_armor(cv::Mat &img, size_t id);
    void set_thread(size_t id);
    // void get_param(std::string &save_folder, bool &use_camera, std::string &vedio_path, std::string &camera_config_file);
private:
    cv::Rect get_rect(cv::Mat &img, deploy::Box &bbox);
    size_t num_threads_;
    std::queue<std::pair<cv::Mat, std::promise<armor_result>>> tasks_;
    std::vector<std::thread> threads_;
    std::mutex tasks_mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stop_;

    std::vector<std::shared_ptr<deploy::DeployDet>> detectors_;
    std::shared_ptr<tools::Timer> timer_;
    rclcpp::Publisher<radar_msgs::msg::CarBbox>::SharedPtr carbbox_publisher_;

    std::shared_ptr<TrackerManager> tracker_manager_;
};
#endif