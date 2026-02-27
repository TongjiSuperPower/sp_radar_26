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
    // TIME_THRESHOLD = config["TIME_THRESHOLD"].as<double>();
    // DISTANCE_THRESHOLD = config["DISTANCE_THRESHOLD"].as<double>();

}

void Tracker::update(radar_msgs::msg::Car car)
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
    last_update_time_ = rclcpp::Clock().now();
    last_time_ = rclcpp::Clock().now();
}

void Tracker::predict(rclcpp::Time now)
{
    dt_ = (now.nanoseconds() - last_time_.nanoseconds()) / 1e9;
    last_time_ = rclcpp::Clock().now();
    
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
    float v = sqrt(pow(x(1),2) + pow(x(3),2));
    return distance(car) < DISTANCE_THRESHOLD;
}

bool Tracker::has_lost_track()
{
    int flag = 0;
    rclcpp::Time now = rclcpp::Clock().now();
    if ((now.nanoseconds() - last_update_time_.nanoseconds()) / 1e9 > TIME_THRESHOLD)    
        flag = 1;
    // else if ((x(0) < 1 && x(2) < 1) || (x(0) > 27 && x(2) > 14))  
    //     flag = 1;

    return flag;
}

void TrackerManager::update_costs(const std::vector<std::vector<float>>& association_mat, SecureMat<float>* costs)
{
    size_t rows_size = association_mat.size();
    size_t cols_size = rows_size > 0 ? association_mat.at(0).size() : 0;

    costs->Resize(rows_size, cols_size);

    for (size_t row_idx = 0; row_idx < rows_size; ++row_idx)
    {
        for (size_t col_idx = 0; col_idx < cols_size; ++col_idx)
        {
            (*costs)(row_idx, col_idx) = association_mat.at(row_idx).at(col_idx);
        }
    }
}

radar_msgs::msg::Cars::SharedPtr TrackerManager::callback(radar_msgs::msg::Cars::ConstPtr cars)
{
    // match tracker with msg
    auto cars_msg = std::make_shared<radar_msgs::msg::Cars>(*cars);
    
    int n_trackers = trackers_.size();
    int n_cars = cars_msg->cars.size();
    std::vector<std::vector<float>> cost_matrix(n_trackers, std::vector<float>(n_cars, 1000.0));

    int i = 0;
    for (auto& tracker : trackers_)
    {
        tracker.predict(cars->header.stamp);
        for (size_t j = 0; j < cars_msg->cars.size() ; ++j)
        {
            if (tracker.is_near(cars_msg->cars[j]))
            {
                cost_matrix[i][j] = tracker.distance(cars_msg->cars[j]);
            }
        }
        i++;
    }

    HungarianOptimizer<float> optimizer;
    std::vector<std::pair<size_t, size_t>> assignments;
    update_costs(cost_matrix, optimizer.costs());
    optimizer.Minimize(&assignments);

    std::set<size_t> indices_to_remove;
    for (const auto& assignment : assignments)
    {
        size_t tracker_idx = assignment.first;
        size_t car_idx = assignment.second;
        std::cout << "distance:" << trackers_[tracker_idx].distance(cars_msg->cars[car_idx]) << std::endl;
        if (trackers_[tracker_idx].is_near(cars_msg->cars[car_idx])) continue;
        trackers_[tracker_idx].update(cars_msg->cars[car_idx]);
        
        if (car_idx < cars_msg->cars.size()) {
            indices_to_remove.insert(car_idx);
        }
    }

    for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it) {
        if (*it < cars_msg->cars.size()) {
            cars_msg->cars.erase(cars_msg->cars.begin() + *it);
        }
    }

    // for every car left, create a new tracker 
    for (auto& car : cars_msg->cars) {
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

    // for each id, gets the car with the highest confidence
    auto result = std::make_shared<radar_msgs::msg::Cars>();
    std::vector<double> id_confidence(ID_KINDS, 0.0);
    std::map<int, radar_msgs::msg::Car> id_car;
    for (auto& tracker : trackers_) {
        auto id_and_confidences = tracker.get_id_and_confidence();
        for (auto [id, confidence] : id_and_confidences) {
            if (id == -1) {
                continue;
            }
            else if (confidence > id_confidence[id]) {
                radar_msgs::msg::Car car;
                car.x = tracker.get_position().first;
                car.y = tracker.get_position().second;
                car.class_id = id;
                id_car.insert_or_assign(id, car);
            }
        }
    }

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