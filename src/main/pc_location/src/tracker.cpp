#include "../include/tracker.hpp"

Tracker::Tracker() : ExtendedKalmanFilter(Eigen::VectorXd::Zero(X_LENGTH), Eigen::MatrixXd::Identity(X_LENGTH, X_LENGTH))
{
    P *= 10;   
    // the first position is more convincing than velocity of them
    for (int i = X_LENGTH / 2; i < X_LENGTH; i++) {
        P(i, i) *= 10000;
    }

    lose_track_count_ = MAX_LOSE_TRACK_FRAME;
}

Tracker::~Tracker()
{
}


/******************************************************************************************************************
    @brief update the condition of tracker by bbox
    @param bbox the matched bbox of a new yolo result
******************************************************************************************************************/
void Tracker::update(cv::Point3f center_in_map)
{
    // about z, the infomation about car in this frame
    Eigen::VectorXd z(X_LENGTH / 2);
    z(0) = center_in_map.x;
    z(1) = center_in_map.y;
    z(2) = center_in_map.z;
 
    predict();

    const double sigma_m = 0.05;
    Eigen::MatrixXd   H{{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
                        {0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};
    
    Eigen::MatrixXd R(X_LENGTH / 2, X_LENGTH / 2);
    R.setZero();
    R.diagonal() << R_arg, R_arg, R_arg;

    try {
        tools::ExtendedKalmanFilter::update(z, H, R);
    }
    catch (...) {
        throw std::runtime_error("update error");
    }

    lose_track_count_ = 0;
}


/******************************************************************************************************************
    @brief update the condition of tracker only by inertia
******************************************************************************************************************/
void Tracker::update()
{
    predict();
    int temp_lose_track = lose_track_count_++;
    update(get_center());
    lose_track_count_ = temp_lose_track;
}

int Tracker::get_lose_track_count()
{
    return lose_track_count_;
}

cv::Point3f Tracker::get_center()
{
    cv::Point3f center;
    center.x = x(0);
    center.y = x(1);
    center.z = x(2);

    return center;
}

/******************************************************************************************************************
    @brief Predict x, y, w, h of bbox by inertia， using Kalman filter
******************************************************************************************************************/
void Tracker::predict()
{   
    Eigen::MatrixXd F{{1.0, 0.0, 0.0, dt_, 0.0, 0.0}, 
                      {0.0, 1.0, 0.0, 0.0, dt_, 0.0},
                      {0.0, 0.0, 1.0, 0.0, 0.0, dt_},
                      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

    Eigen::MatrixXd Q(X_LENGTH, X_LENGTH);
    Q.setZero();
    Q.diagonal() << Q_arg, Q_arg, Q_arg, Q_arg, Q_arg, Q_arg;
    
    try {
        tools::ExtendedKalmanFilter::predict(F, Q);
    }
    catch (...) {
        throw std::runtime_error("predict error");
    }
}

void Tracker::set_delta_time(double delta_time)
{
    dt_ = delta_time;
}

radar_msgs::msg::Bbox Tracker::get_bbox()
{
    return bbox_;
}

void Tracker::set_bbox(radar_msgs::msg::Bbox bbox)
{
    bbox_ = bbox;
}


/******************************************************************************************************************
    @brief constructor of TrackerManager
    @param path video saving path
******************************************************************************************************************/
TrackerManager::TrackerManager()
{
    last_update_time_ = 0.0;    // a flag means that tracker hasn't started
    delta_time_ = 0.0;
}

/******************************************************************************************************************
    @brief implement of tracker, please call it every time after yolo detection
    @param frame the frame followed
    @param car_bbox_msg yolo result
    @return the result of tracker that conbine yolo result from the frame and history
    @todo drawing a UI to manually calibrate the energy machine(能量机关)
******************************************************************************************************************/
std::vector<cv::Point3f> TrackerManager::callback(radar_msgs::msg::CarBbox car_bbox_msg, std::vector<cv::Point3f> car_centers)
{
    std::cout << "car_bbox size: " << car_bbox_msg.bboxs.size() << " car center size: " << car_centers.size() << std::endl;
    // update time and delta time
    double this_update_time = car_bbox_msg.header.stamp.sec + static_cast<double>(car_bbox_msg.header.stamp.nanosec) * 1e-9;
    delta_time_ = this_update_time - last_update_time_;
    last_update_time_ = this_update_time;
    if (fabs(delta_time_) > 10) {
        return std::vector<cv::Point3f>(MAX_CAR_NUMBER);
    }
    
    std::vector<PairedResult> paired_results = match(car_bbox_msg);

    for (auto &paired_result : paired_results) {
        Tracker* t;
        cv::Point3f center;
        switch (paired_result.status)
        {
        case 0: // paired
            t = &trackers_[paired_result.tracker_index];
            t->set_delta_time(delta_time_);
            center = car_centers[car_bbox_msg.bboxs[paired_result.input_index].class_id];
            if (center.x == 0 && center.y == 0) {
                t->update();
            }
            else {
                t->update(center);
            }
            t->set_bbox(car_bbox_msg.bboxs[paired_result.input_index]);
            break;
        case 1: // only tracker
            t = &trackers_[paired_result.tracker_index];
            t->set_delta_time(delta_time_);
            t->update();
            break;
        case 2: // only input msg
            t = &trackers_.emplace_back();
            t->set_delta_time(delta_time_);
            t->update(car_centers[car_bbox_msg.bboxs[paired_result.input_index].class_id]);
            t->set_bbox(car_bbox_msg.bboxs[paired_result.input_index]);
            break;
        default:
            break;
        }
    }
    
    for (auto &tracker : trackers_) {
        std::cout << "id " << (int)tracker.get_bbox().class_id << std::endl;
        std::cout << "center " << tracker.get_center().x << " " << tracker.get_center().y << std::endl;
        car_centers[tracker.get_bbox().class_id] = tracker.get_center();
    }
    std::cout << "trackers count" << trackers_.size() << std::endl;
    auto_release();

    return car_centers;
}


/******************************************************************************************************************
    @brief match the bboxes and trackers by IOU and id
    @param tracked_car_bbox the origin message from yolo detection
    @note greed algorithm, do not ensure best match, next possible step-Hungarian Algorithm
******************************************************************************************************************/
std::vector<PairedResult> TrackerManager::match(const radar_msgs::msg::CarBbox car_bbox_msg)
{
    // calculate IOU
    std::vector<std::vector<double>> IOUs(car_bbox_msg.bboxs.size());  // IOUs between bboxes from message and tracker
    for (int bbox_index = 0; bbox_index < car_bbox_msg.bboxs.size(); bbox_index++) {
        for (int tracker_index = 0; tracker_index < trackers_.size(); tracker_index++) {
            IOUs[bbox_index].push_back(IOU(car_bbox_msg.bboxs[bbox_index], trackers_[tracker_index].get_bbox()));
        }
    }
 
    std::vector<PairedResult> results;
    std::vector<int> bboxes_paired(car_bbox_msg.bboxs.size()), trackers_paired(trackers_.size());
    int match_end_flag = 0;
    // push in paired bboxes
    while (!match_end_flag) {
        double max_IOU = 0.0;
        int max_IOU_bbox_index = -1, max_IOU_tracker_index = -1;
        for (int bbox_index = 0; bbox_index < car_bbox_msg.bboxs.size(); bbox_index++) {
            if (bboxes_paired[bbox_index] == 1) 
                continue;
            for (int tracker_index = 0; tracker_index < trackers_.size(); tracker_index++) {
                if (trackers_paired[tracker_index] == 1) 
                    continue;
                if (IOUs[bbox_index][tracker_index] > max_IOU) {
                    max_IOU = IOUs[bbox_index][tracker_index];
                    max_IOU_bbox_index = bbox_index;
                    max_IOU_tracker_index = tracker_index;
                }
            }
        }
        if (max_IOU == 0.0) {   // all bboxes and trackers that can be paired has been paired
            match_end_flag = 1;
        }
        else {
            bboxes_paired[max_IOU_bbox_index] = 1;
            trackers_paired[max_IOU_tracker_index] = 1;
            PairedResult paired_result = {0, max_IOU_tracker_index, max_IOU_bbox_index};
            results.push_back(paired_result);
        }
    }
    // push in the unpaired trackers
    for (int index = 0; index < trackers_.size(); index++) {
        if (trackers_paired[index] == 0) {
            PairedResult paired_result = {1, index};
            results.push_back(paired_result);
        }
    }
    // push in the unpaired bboxes
    for (int index = 0; index < car_bbox_msg.bboxs.size(); index++) {
        if (bboxes_paired[index] == 0) {
            PairedResult paired_result = {2, -1, index};    // -1 means no matched
            results.push_back(paired_result);
        }
    }
    return results;
}

void TrackerManager::auto_release()
{
    for (int i = 0; i < trackers_.size(); i++) {
        if (trackers_[i].get_lose_track_count() >= MAX_LOSE_TRACK_FRAME) {
            trackers_.erase(trackers_.begin() + i);
        }
    }
}

double TrackerManager::IOU(radar_msgs::msg::Bbox bbox_1, radar_msgs::msg::Bbox bbox_2)
{
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
