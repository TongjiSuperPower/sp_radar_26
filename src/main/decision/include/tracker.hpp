#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <map>
#include <list>
#include <vector>
#include <chrono>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <yaml-cpp/yaml.h>

#include "radar_msgs/msg/car.h"
#include "radar_msgs/msg/cars.hpp"

#include "../tools/extended_kalman_filter.hpp"
#include "hungarian_optimizer.h"

#define STATE_SIZE 4
#define MEASUREMENT_SIZE 2
#define ID_KINDS 12

const int HISTORY_SIZE = 10;
const double TIME_THRESHOLD = 1.5;

class Tracker : public tools::ExtendedKalmanFilter {
public:
    Tracker();
    void predict(rclcpp::Time time);
    void update(radar_msgs::msg::Car car, rclcpp::Time stamp);
    std::map<int, double> get_id_and_confidence();
    std::pair<double, double> get_position();
    
    double distance(radar_msgs::msg::Car car);
    bool is_near(radar_msgs::msg::Car car);
    bool has_lost_track(rclcpp::Time now);

private:
    rclcpp::Time last_update_time_;
    rclcpp::Time last_time_;
    double dt_;
    bool init_flag_;
    // int id_;
    std::list<int> history_;    // id
    int no_id_count_;

    float sigma_q_x_ = 50.0f;//越小相信模型
    float sigma_q_y_ = 50.0f;
    float sigma_r_x_ = 0.1f;//越小相信观测
    float sigma_r_y_ = 0.1f;

    // squared Mahalanobis (chi-square) gate for the 2-DOF innovation
    // 4.605 = 90%, 5.991 = 95%, 9.21 = 99%, 13.816 = 99.9%
    double mahalanobis_threshold_ = 9.21;
};

class TrackerManager {
public:
    radar_msgs::msg::Cars::SharedPtr callback(radar_msgs::msg::Cars::ConstPtr cars);

private:
    std::vector<Tracker> trackers_;
    HungarianOptimizer<float> optimizer_;
    radar_msgs::msg::Cars::ConstPtr cars_;
};

#endif