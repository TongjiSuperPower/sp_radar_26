#include "../include/tracker.hpp"

#include <iostream>

// x = [x, vx, y, vy]
// P的初值没细调

Tracker::Tracker() : 
    tools::ExtendedKalmanFilter(Eigen::VectorXd::Zero(STATE_SIZE), 
        Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
{
    init_flag_ = 0;
    no_id_count_ = 0;
    P(1, 1) *= 100;
    P(3, 3) *= 100;

    auto config = YAML::LoadFile("./src/main/decision/config/decision.yaml");
    sigma_q_x_ = config["sigma_q_x"].as<double>();
    sigma_q_y_ = config["sigma_q_y"].as<double>();
    sigma_r_x_ = config["sigma_r_x"].as<double>();
    sigma_r_y_ = config["sigma_r_y"].as<double>();
    mahalanobis_threshold_ = config["mahalanobis_threshold"].as<double>(mahalanobis_threshold_);
    // TIME_THRESHOLD = config["TIME_THRESHOLD"].as<double>();

}

void Tracker::update(radar_msgs::msg::Car car, rclcpp::Time stamp)
{
    if (init_flag_ == 0) {
        x(0) = car.x;
        x(2) = car.y;  
        init_flag_ = 1;
    }
    else {
        Eigen::VectorXd z = Eigen::VectorXd(MEASUREMENT_SIZE);
        z << car.x, car.y;

        Eigen::MatrixXd R(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
        R << sigma_r_x_,          0, 
                      0, sigma_r_y_;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
        H << 1, 0, 0, 0,
             0, 0, 1, 0;

        tools::ExtendedKalmanFilter::update(z, H, R);
    }

    if (car.class_id != -1) {
        history_.push_back(car.class_id);
        if (history_.size() > HISTORY_SIZE) {
            history_.pop_front();
        }
    }
    else {
        no_id_count_++;
    }

    // if (car.class_id != -1) {
    //     id_ = car.class_id;
    // }
    last_update_time_ = stamp;
    last_time_ = stamp;
}

void Tracker::predict(rclcpp::Time now)
{
    dt_ = (now.nanoseconds() - last_time_.nanoseconds()) / 1e9;
    last_time_ = now;
    
    Eigen::MatrixXd Q(STATE_SIZE, STATE_SIZE);
    Q << sigma_q_x_*pow(dt_, 3)/3, sigma_q_x_*pow(dt_, 2)/2, 0, 0,
         sigma_q_x_*pow(dt_, 2)/2, sigma_q_x_*pow(dt_, 1), 0, 0,
         0, 0, sigma_q_y_*pow(dt_, 3)/3, sigma_q_y_*pow(dt_, 2)/2,
         0, 0, sigma_q_y_*pow(dt_, 2)/2, sigma_q_y_*pow(dt_, 1);

    // Q << sigma_q_x_, sigma_q_x_, 0, 0,
    //     sigma_q_x_, sigma_q_x_, 0, 0,
    //     0, 0, sigma_q_y_, sigma_q_y_,
    //     0, 0, sigma_q_y_, sigma_q_y_;

    Eigen::MatrixXd F{{1.0, dt_, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0},
                      {0.0, 0.0, 1.0, dt_},
                      {0.0, 0.0, 0.0, 1.0}};

    tools::ExtendedKalmanFilter::predict(F, Q);

}

std::map<int, double> Tracker::get_id_and_confidence()
{
    std::map<int, double> id_and_confidence;

    std::vector<int> id_count(ID_KINDS, 0);
    for (auto id : history_) {
        if (id >= 0 && id < ID_KINDS) {  // Validate class_id
            id_count[id]++;
        }
    }

    for (int id = 0; id < ID_KINDS; id++) {
        if (id_count[id] > 0) {
            id_and_confidence.insert(std::make_pair(id, id_count[id] * 1.0 / (HISTORY_SIZE + no_id_count_)));
        }
    }


    return id_and_confidence;
}

std::pair<double, double> Tracker::get_position()
{
    return std::make_pair(x(0), x(2));
}

double Tracker::distance(radar_msgs::msg::Car car)
{ 
    return std::sqrt(std::pow(car.x - x(0), 2) + std::pow(car.y - x(2), 2));
}   

bool Tracker::is_near(radar_msgs::msg::Car car)
{
    // Gate the measurement with the squared Mahalanobis distance of the
    // innovation: d^2 = y^T * S^-1 * y, where y = z - H*x and
    // S = H * P * H^T + R is the innovation covariance.
    // S is positive definite because R is positive definite.
    Eigen::VectorXd z(MEASUREMENT_SIZE);
    z << car.x, car.y;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
    H << 1, 0, 0, 0,
         0, 0, 1, 0;

    Eigen::MatrixXd R(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
    R << sigma_r_x_,          0,
                  0, sigma_r_y_;

    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    const double d2 = y.dot(S.ldlt().solve(y));

    return d2 < mahalanobis_threshold_;
}

bool Tracker::has_lost_track(rclcpp::Time now)
{
    int flag = 0;
    if ((now.nanoseconds() - last_update_time_.nanoseconds()) / 1e9 > TIME_THRESHOLD)    
        flag = 1;
    // else if ((x(0) < 1 && x(2) < 1) || (x(0) > 27 && x(2) > 14))  
    //     flag = 1;

    return flag;
}

radar_msgs::msg::Cars::SharedPtr TrackerManager::callback(radar_msgs::msg::Cars::ConstPtr cars)
{
    auto cars_msg = std::make_shared<radar_msgs::msg::Cars>(*cars);
    const size_t n_trackers = trackers_.size();
    const size_t n_cars = cars_msg->cars.size();

    // predict all trackers to the current stamp
    for (auto& tracker : trackers_) {
        tracker.predict(cars->header.stamp);
    }

    // build the association cost matrix:
    // distance for pairs within the gate, a large sentinel otherwise
    constexpr float kMaxCost = 1e6f;
    std::vector<std::vector<float>> cost_matrix(n_trackers, std::vector<float>(n_cars, kMaxCost));
    for (size_t i = 0; i < n_trackers; ++i) {
        for (size_t j = 0; j < n_cars; ++j) {
            if (trackers_[i].is_near(cars_msg->cars[j])) {
                cost_matrix[i][j] = static_cast<float>(trackers_[i].distance(cars_msg->cars[j]));
            }
        }
    }

    // solve the global optimal assignment (Munkres / Hungarian)
    std::vector<std::pair<size_t, size_t>> assignments;
    SecureMat<float>* costs = optimizer_.costs();
    costs->Resize(n_trackers, n_cars);
    for (size_t i = 0; i < n_trackers; ++i) {
        for (size_t j = 0; j < n_cars; ++j) {
            (*costs)(i, j) = cost_matrix[i][j];
        }
    }
    optimizer_.Minimize(&assignments);

    // apply the valid matched pairs
    std::vector<bool> car_matched(n_cars, false);
    for (const auto& [tracker_idx, car_idx] : assignments) {
        if (tracker_idx >= n_trackers || car_idx >= n_cars) {
            continue;  // ignore padding cells used to square the matrix
        }
        if (!trackers_[tracker_idx].is_near(cars_msg->cars[car_idx])) {
            continue;
        }
        trackers_[tracker_idx].update(cars_msg->cars[car_idx], cars->header.stamp);
        car_matched[car_idx] = true;
    }

    // create a new tracker for every unmatched car
    for (size_t j = 0; j < n_cars; ++j) {
        if (car_matched[j]) {
            continue;
        }
        Tracker new_tracker;
        new_tracker.update(cars_msg->cars[j], cars->header.stamp);
        trackers_.push_back(new_tracker);
    }

    // delete trackers that lose track
    for (auto tracker = trackers_.begin(); tracker != trackers_.end(); ) {
        if (tracker->has_lost_track(cars->header.stamp)) {
            tracker = trackers_.erase(tracker);
        }
        else {
            tracker++;
        }
    }

    // for each id, gets the car with the highest confidence
    auto result = std::make_shared<radar_msgs::msg::Cars>();
    std::vector<double> id_confidence(ID_KINDS, 0.0);
    std::map<int, radar_msgs::msg::Car> id_car;
    for (auto& tracker : trackers_) {
        auto id_and_confidences = tracker.get_id_and_confidence();
        for (auto [id, confidence] : id_and_confidences) {
            // std::cout << "id: " << id << " and confidence: " << confidence << ", now this id has confidence: " << id_confidence[id]<< std::endl;

            if (id == -1) {
                continue;
            }
            else if (confidence > id_confidence[id]) {
                radar_msgs::msg::Car car;
                car.x = tracker.get_position().first;
                car.y = tracker.get_position().second;
                car.class_id = id;
                id_car.insert_or_assign(id, car);
                id_confidence[id] = confidence;
            }

            for (auto [id, car] : id_car) {
                // std::cout << "id: " << id << ", position: " << car.x << std::endl;
            }
        }
        // std::cout << std::endl;
    }
    // std::cout << "end of callback" << std::endl << std::endl << std::endl;


    // // ensure each tracker at most outputs one car
    // const double distance_between_car = 0.25;
    // std::vector<int> id_to_be_delete;
    // for (int id_1 = 0; id_1 < 6; id_1++) {
    //     for (int id_2 = id_1 + 1; id_2 < 6; id_2++){
    //         if (pow(id_car[id_1].x - id_car[2].x, 2) && pow(id_car[id_1].y - id_car[id_2].y, 2) 
    //             < pow(distance_between_car, 2)) {
    //                 if (id_confidence[id_1] > id_confidence[id_2])
    //                     id_to_be_delete.push_back(id_2);
    //                 else
    //                     id_to_be_delete.push_back(id_1);
    //         }
    //     }
    // }
    // for (auto id : id_to_be_delete) {
    //     id_car.erase(id);
    // }


    for (auto& [id, car] : id_car) {
        result->cars.push_back(car);
    }

    return result;
}