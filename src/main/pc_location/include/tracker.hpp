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

#define MAX_CAR_NUMBER 12
#define IOU_THRESHOLD 0.5
#define MAX_LOSE_TRACK_FRAME 5
#define X_LENGTH 6  // x, y, z and their derivative

const float R_arg = 1.0f;   // how uncertain observation value is
const float Q_arg = 1.0f;   // how uncertain prediction value is

class Tracker : public tools::ExtendedKalmanFilter{
public:
    Tracker();
    ~Tracker();
    void set_delta_time(double delta_time);
    void predict();
    void update(cv::Point3f center_in_map);
    void update();

    cv::Point3f get_center();
    int get_lose_track_count();

    void set_bbox(radar_msgs::msg::Bbox bbox);
    radar_msgs::msg::Bbox get_bbox();
    
private:
    double dt_;
    int lose_track_count_;
    radar_msgs::msg::Bbox bbox_;
    
    void update_confidence(int id, double confidence);
};


struct PairedResult {   // used in TrackerManager::match
    int status; // 0 paired, 1 only tracker, 2 only input msg
    int tracker_index;
    int input_index;
};

class TrackerManager {
public:
    TrackerManager();
    std::vector<cv::Point3f> callback(radar_msgs::msg::CarBbox msg, std::vector<cv::Point3f> car_centers);

private:
    std::vector<Tracker> trackers_;
    double last_update_time_;  
    double delta_time_;
    
    double IOU(radar_msgs::msg::Bbox bbox_1, radar_msgs::msg::Bbox bbox_2);
    std::vector<PairedResult> match(const radar_msgs::msg::CarBbox msg);
    void auto_release();    
};

