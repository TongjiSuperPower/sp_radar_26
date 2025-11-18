#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/car_bbox.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tools/extended_kalman_filter.hpp"
#include "tools/recorder.hpp"

#define MAX_CAR_NUMBER 12
#define IOU_THRESHOLD 0.5
#define EPSILON 10

class Tracker {
public:
    Tracker(radar_msgs::msg::Bbox Bbox, std::string config_path);
    ~Tracker();
    void set_delta_time(double delta_time);
    void predict();
    void update(radar_msgs::msg::Bbox Bbox);
    void update(cv::Mat mask);
    void update();
    int get_id();
    double get_confidence(int id) { return id_confidence_[id]; }
    radar_msgs::msg::Bbox get_bbox();
    
private:
    std::vector<double> id_confidence_; // confidence of all ids
    Eigen::VectorXd x_; // include x, y, w, h, dx, dy, dw, dh
    Eigen::MatrixXd P_; // covariance martix of x_
    double dt_;
    std::shared_ptr<tools::ExtendedKalmanFilter> ekf_;
    double past_confidence_alpha_;
    double decrease_confidence_alpha_;

    void update_confidence(int id, double confidence);

};


struct PairedResult {   // used in TrackerManager::match
    int status; // 0 paired, 1 only tracker, 2 only input msg
    std::vector<Tracker>::iterator tracker;
    radar_msgs::msg::Bbox bbox;
};

class TrackerManager {
public:
    TrackerManager(std::string config_path);
    radar_msgs::msg::CarBbox callback(const cv::Mat frame, radar_msgs::msg::CarBbox msg);
    void record(const radar_msgs::msg::CarBbox msg, cv::Mat image);

private:
    std::vector<Tracker> trackers_;
    double last_update_time_;  
    double delta_time_;
    std::string config_path_;
    bool save_flag_;
    std::shared_ptr<tools::Recorder> recorder_;
    cv::Ptr<cv::BackgroundSubtractorKNN> background_subtractor_;
    bool use_background_substractor_; 
    double confidence_threshold_;
    double past_confidence_alpha_;
    double decrease_confidence_alpha_;
    
    double IOU(radar_msgs::msg::Bbox bbox_1, radar_msgs::msg::Bbox bbox_2);
    std::vector<PairedResult> match(const radar_msgs::msg::CarBbox msg);
    void push_bboxes(radar_msgs::msg::CarBbox &tracked_car_bbox);
    void auto_release();    
    void clear_mask_in_bbox(cv::Mat mask, radar_msgs::msg::Bbox bbox);
};

class MeanShift {
public:
    MeanShift(cv::Mat mask, radar_msgs::msg::Bbox init_bbox);
    radar_msgs::msg::Bbox do_mean_shift();

private:
    void shift_once();

    int stop_;
    cv::Mat mask_;
    radar_msgs::msg::Bbox bbox_;
};
