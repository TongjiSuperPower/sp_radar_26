#include "tracker.hpp"
// x: x, y, w. h, dx, dy, dw, dh

Tracker::Tracker() : 
    tools::ExtendedKalmanFilter(Eigen::VectorXd::Zero(STATE_SIZE), 
        Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
{
    init_flag_ = 0;
    no_id_count_ = 0;

    // the uncertainty of v of each measurement value is larger than the measurement's
    for (int i = 1; i < STATE_SIZE; i += 2) {
        P(i, i) *= 10000; 
    }
}

Eigen::VectorXd Tracker::bbox_to_measurement(const radar_msgs::msg::Bbox& bbox)
{
    Eigen::VectorXd measurement(4);  
    float x = (bbox.x_max + bbox.x_min) / 2;
    float y = (bbox.y_max + bbox.y_min) / 2;
    float w = bbox.x_max - bbox.x_min;
    float h = bbox.y_max - bbox.y_min;
    measurement << x, y, w, h; 
    return measurement;
}

void Tracker::init_state_by_bbox(const radar_msgs::msg::Bbox& bbox)
{
    Eigen::VectorXd measurement = bbox_to_measurement(bbox);
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        x(i) = measurement(i);
    }
}

radar_msgs::msg::Bbox Tracker::get_bbox()
{
    radar_msgs::msg::Bbox bbox;
    bbox.x_max = x(0) + x(2) / 2;
    bbox.x_min = x(0) - x(2) / 2;
    bbox.y_max = x(1) + x(3) / 2;
    bbox.y_min = x(1) - x(3) / 2;

    auto [id, confidence] = get_id_and_confidence();
    bbox.class_id = id;
    bbox.class_confidence = confidence;

    return bbox;
}

std::pair<int, double> Tracker::get_id_and_confidence()
{
    int best_id = -1;
    double best_confidence = 0.0;

    std::vector<int> id_count(ID_KINDS, 0);
    for (auto id = history_.begin(); id != history_.end(); id++) {
        if (*id >= 0 && *id < ID_KINDS) {  // Validate class_id
            id_count[*id]++;
        }
    }

    for (int id = 0; id < ID_KINDS; id++) {
        double confidence = id_count[id] * 1.0 / (HISTORY_SIZE + no_id_count_);
        if (confidence > best_confidence) {
            best_id = id;
            best_confidence = confidence;
        }
    }

    return std::pair<int, double>(best_id, best_confidence);
}

bool Tracker::has_lost_track()
{
    int flag = 0;
    rclcpp::Time now = rclcpp::Clock().now();
    if ((now.nanoseconds() - last_update_time_.nanoseconds()) / 1e9 > TIME_THRESHOLD)    
        flag = 1;

    return flag;
}

void Tracker::update(radar_msgs::msg::Bbox bbox)
{
    if (init_flag_ == 0) {
        init_state_by_bbox(bbox);
        init_flag_ = 1;
    }
    else {
        Eigen::VectorXd z = bbox_to_measurement(bbox);

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
        R.diagonal() << pow(sigma_m * x(2), 2), 
                        pow(sigma_m * x(3), 2),
                        pow(sigma_m * x(2), 2), 
                        pow(sigma_m * x(3), 2);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            H(i, i) = 1;
        }

        tools::ExtendedKalmanFilter::update(z, H, R);
    }

    if (bbox.class_id != -1) {
        history_.push_back(bbox.class_id);
        if (history_.size() > HISTORY_SIZE)
            history_.pop_front();
    }
    else 
        no_id_count_++;

    last_update_time_ = rclcpp::Clock().now();
}

void Tracker::predict(rclcpp::Time now)
{
    double dt_ = (now.nanoseconds() - last_time_.nanoseconds()) / 1e9;
    last_time_ = rclcpp::Clock().now();
    
    Eigen::MatrixXd F{{1.0, 0.0, 0.0, 0.0, dt_, 0.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0, 0.0, dt_, 0.0, 0.0},
                      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt_, 0.0},
                      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt_},
                      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    Q.diagonal() << pow((sigma_p * x(2)), 2), 
                    pow((sigma_p * x(3)), 2),
                    pow((sigma_p * x(2)), 2), 
                    pow((sigma_p * x(3)), 2),
                    pow((sigma_v * x(2)), 2), 
                    pow((sigma_v * x(3)), 2),
                    pow((sigma_v * x(2)), 2), 
                    pow((sigma_v * x(3)), 2);

    tools::ExtendedKalmanFilter::predict(F, Q);
}

radar_msgs::msg::CarBbox TrackerManager::callback(radar_msgs::msg::CarBbox cars)
{
    // match tracker with msg
    for (auto& tracker : trackers_) {
        tracker.predict(cars.header.stamp);
        for (auto bbox_iterator = cars.bboxs.begin(); bbox_iterator != cars.bboxs.end(); bbox_iterator++) {
            if (IOU(tracker, *bbox_iterator) > IOU_THRESHOLD){
                tracker.update(*bbox_iterator);
                cars.bboxs.erase(bbox_iterator);
                break;
            }
        }
    }

    // for every car left, create a new tracker 
    for (auto& car : cars.bboxs) {
        Tracker new_tracker;
        new_tracker.update(car);
        trackers_.push_back(new_tracker);
    }

    // delete trackers that lose track
    for (auto tracker = trackers_.begin(); tracker != trackers_.end(); ) {
        if (tracker->has_lost_track()) {
            tracker = trackers_.erase(tracker);
        }
        else {
            tracker++;
        }
    }

    cars.bboxs.clear();
    for (auto& tracker : trackers_) {
        cars.bboxs.push_back(tracker.get_bbox());
    }

    return cars;
}

double TrackerManager::IOU(Tracker tracker, radar_msgs::msg::Bbox bbox_2)
{
    auto bbox_1 = tracker.get_bbox();
    double s1 = (bbox_1.x_max - bbox_1.x_min) * (bbox_1.y_max - bbox_1.y_min);
    double s2 = (bbox_2.x_max - bbox_2.x_min) * (bbox_2.y_max - bbox_2.y_min);

    double inter_right = std::min(static_cast<double>(bbox_1.x_max), static_cast<double>(bbox_2.x_max));
    double inter_left = std::max(static_cast<double>(bbox_1.x_min), static_cast<double>(bbox_2.x_min));
    double inter_down = std::min(static_cast<double>(bbox_1.y_max), static_cast<double>(bbox_2.y_max));
    double inter_up = std::max(static_cast<double>(bbox_1.y_min), static_cast<double>(bbox_2.y_min));

    double inter_w = inter_right - inter_left;
    if (inter_w < 0)
        return 0;
    double inter_h = inter_down - inter_up;
    if (inter_h < 0)
        return 0;

    double inter_s = inter_w * inter_h;
    return inter_s / (s1 + s2 - inter_s);
}