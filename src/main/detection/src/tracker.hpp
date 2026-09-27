#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <map>
#include <list>
#include <chrono>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <yaml-cpp/yaml.h>

#include <radar_msgs/msg/car_bbox.hpp>

#include "tools/extended_kalman_filter.hpp"

#define STATE_SIZE 8
#define MEASUREMENT_SIZE 4
#define ID_KINDS 12

const int HISTORY_SIZE = 10;
const double IOU_THRESHOLD = 0.25;
const double TIME_THRESHOLD = 1.5;

// x: x, y, w. h, dx, dy, dw, dh

class Tracker : public tools::ExtendedKalmanFilter {
public:
    Tracker();
    void predict(rclcpp::Time time);
    void update(radar_msgs::msg::Bbox bbox);

    radar_msgs::msg::Bbox get_bbox();
    bool has_lost_track();
    
private:
    std::pair<int, double> get_id_and_confidence();
    Eigen::VectorXd bbox_to_measurement(const radar_msgs::msg::Bbox& bbox);
    void init_state_by_bbox(const radar_msgs::msg::Bbox& bbox);

    rclcpp::Time last_update_time_;
    rclcpp::Time last_time_;

    bool init_flag_;
    
    std::list<int> history_;    // id
    int no_id_count_;

    // 最近一次"有分类结果"的观测的装甲板得分和车体得分，原样往下传，不参与投票。
    // 决策端要靠它们判断这次身份断言值不值得信，见 Car.msg。默认 -1 = 没有这个信息。
    float last_class_confidence_ = -1.0f;
    float last_car_confidence_ = -1.0f;

    // refence: Bot-SORT (https://arxiv.org/pdf/2206.14651) 
    float sigma_p = 0.05;
    float sigma_v = 0.00625;
    float sigma_m = 0.000000000005; // 0.05
};

class TrackerManager {
public:
    radar_msgs::msg::CarBbox callback(radar_msgs::msg::CarBbox cars);

private:
    double IOU(Tracker tracker, radar_msgs::msg::Bbox car);

    std::list<Tracker> trackers_;
    radar_msgs::msg::CarBbox::ConstPtr cars_;
};

#endif