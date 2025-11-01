#include "tracker.hpp"

/******************************************************************************************************************
    @brief constructor of TrackerManager
    @param path video saving path
******************************************************************************************************************/
TrackerManager::TrackerManager(std::string config_path)
{
    // // test
    // save_flag_ = 1;
    // use_background_substractor_ = 0;
    // confidence_threshold_ = 0.001;
    // past_confidence_alpha_ =  0.98;
    // decrease_confidence_alpha_ = 0.8;
    // std::string save_path = "./src/main/detection/media/tracked_output";

    const auto yaml_config = YAML::LoadFile(config_path);
    config_path_ = config_path;
    save_flag_ = yaml_config["tracker"]["save_tracker_vedio"].as<bool>();
    std::string save_path = yaml_config["tracker"]["save_folder_path"].as<std::string>();
    last_update_time_ = 0.0; // a flag means that tracker hasn't started
    delta_time_ = 0.0;
    if (save_flag_)
    {
        recorder_ = std::make_shared<tools::Recorder>(60, save_path);
    }
    use_background_substractor_ = yaml_config["tracker"]["use_background_substractor"].as<bool>();
    confidence_threshold_ = yaml_config["tracker"]["confidence_threshold"].as<double>();
    // past_confidence_alpha_ =  yaml_config["tracker"]["past_confidence_alpha"].as<double>();
    // decrease_confidence_alpha_ = yaml_config["tracker"]["decrease_confidence_alpha"].as<double>();
    if (use_background_substractor_)
    {
        background_subtractor_ = cv::createBackgroundSubtractorKNN(50, 400, false);
    }
}

/******************************************************************************************************************
    @brief implement of tracker, please call it every time after yolo detection
    @param frame the frame followed
    @param car_bbox_msg yolo result
    @return the result of tracker that conbine yolo result from the frame and history
    @todo drawing a UI to manually calibrate the energy machine(能量机关)
******************************************************************************************************************/
radar_msgs::msg::CarBbox TrackerManager::callback(const cv::Mat frame, radar_msgs::msg::CarBbox car_bbox_msg)
{
    // update time and delta time
    double this_update_time = car_bbox_msg.header.stamp.sec + static_cast<double>(car_bbox_msg.header.stamp.nanosec) * 1e-9;
    delta_time_ = this_update_time - last_update_time_;
    last_update_time_ = this_update_time;

    std::vector<PairedResult> paired_results = match(car_bbox_msg);
    cv::Mat mask;
    if (use_background_substractor_)
    {
        background_subtractor_->apply(frame, mask);
        // mask剔除能量机关
        cv::Rect rec(mask.cols * 5 / 12, mask.rows / 6, mask.cols / 6, mask.rows / 6);
        mask(rec).setTo(0);
    }

    for (auto &paired_result : paired_results)
    {
        auto tracker = paired_result.tracker;
        switch (paired_result.status)
        {
        case 0: // paired
            tracker->set_delta_time(delta_time_);
            tracker->update(paired_result.bbox);
            clear_mask_in_bbox(mask, tracker->get_bbox());
            break;
        case 1: // only tracker
            tracker->set_delta_time(delta_time_);
            if (use_background_substractor_)
            {
                tracker->update(mask);
            }
            else
            {
                tracker->update();
            }
            break;
        case 2: // only input msg
            trackers_.emplace_back(paired_result.bbox, config_path_);
            break;
        default:
            break;
        }
    }
    push_bboxes(car_bbox_msg);
    auto_release();
    if (save_flag_)
    {
        record(car_bbox_msg, frame);
    }

    return car_bbox_msg;
}

/******************************************************************************************************************
    @brief Push bboxes from tracker into message, ensure no repetition
    @param tracked_car_bbox the origin message from yolo detection
    @note will clear the bbox before processing
******************************************************************************************************************/
void TrackerManager::push_bboxes(radar_msgs::msg::CarBbox &tracked_car_bbox)
{
    std::vector<int> used_id(MAX_CAR_NUMBER, 0);
    std::vector<int> used_tracker(trackers_.size(), 0);
    tracked_car_bbox.bboxs.clear();
    for (int i = 0; i < trackers_.size(); i++)
    {
        double max_confidence = 0.0;
        int most_confident_tracker = -1, most_confident_id = -1;
        for (int tracker_index = 0; tracker_index < trackers_.size(); tracker_index++)
        {
            if (used_tracker[tracker_index] == 1)
            {
                continue;
            }
            for (int id = 0; id < MAX_CAR_NUMBER; id++)
            {
                if (used_id[id] == 1)
                {
                    continue;
                }
                if (trackers_[tracker_index].get_confidence(id) > max_confidence)
                {
                    most_confident_tracker = tracker_index;
                    max_confidence = trackers_[tracker_index].get_confidence(id);
                    most_confident_id = id;
                }
            }
        }
        if (max_confidence == 0.0)
        {
            break;
        }
        auto bbox = trackers_[most_confident_tracker].get_bbox();
        bbox.class_id = most_confident_id;
        tracked_car_bbox.bboxs.push_back(bbox);
        used_tracker[most_confident_tracker] = 1;
        used_id[most_confident_id] = 1;
    }
}

/******************************************************************************************************************
    @brief match the bboxes and trackers by IOU and id
    @param tracked_car_bbox the origin message from yolo detection
    @note greed algorithm, do not ensure best match, next possible step-Hungarian Algorithm
******************************************************************************************************************/
std::vector<PairedResult> TrackerManager::match(const radar_msgs::msg::CarBbox car_bbox_msg)
{
    // calculate IOU
    const double IOU_weight = 0.8;                                    // how you trust IOU, compared with id, 0.8 for example
    std::vector<std::vector<double>> IOUs(car_bbox_msg.bboxs.size()); // IOUs between bboxes from message and tracker
    for (int bbox_index = 0; bbox_index < car_bbox_msg.bboxs.size(); bbox_index++)
    {
        for (int tracker_index = 0; tracker_index < trackers_.size(); tracker_index++)
        {
            IOUs[bbox_index].push_back(IOU_weight * IOU(car_bbox_msg.bboxs[bbox_index], trackers_[tracker_index].get_bbox()) + (1 - IOU_weight) * (car_bbox_msg.bboxs[bbox_index].class_id == trackers_[tracker_index].get_id()));
        }
    }

    std::vector<PairedResult> results;
    std::vector<int> bboxes_paired(car_bbox_msg.bboxs.size()), trackers_paired(trackers_.size());
    int match_end_flag = 0;
    // push in paired bboxes
    while (!match_end_flag)
    {
        double max_IOU = 0.0;
        int max_IOU_bbox_index = -1, max_IOU_tracker_index = -1;
        for (int bbox_index = 0; bbox_index < car_bbox_msg.bboxs.size(); bbox_index++)
        {
            if (bboxes_paired[bbox_index] == 1)
                continue;
            for (int tracker_index = 0; tracker_index < trackers_.size(); tracker_index++)
            {
                if (trackers_paired[tracker_index] == 1)
                    continue;
                if (IOUs[bbox_index][tracker_index] > max_IOU)
                {
                    max_IOU = IOUs[bbox_index][tracker_index];
                    max_IOU_bbox_index = bbox_index;
                    max_IOU_tracker_index = tracker_index;
                }
            }
        }
        if (max_IOU == 0.0)
        { // all bboxes and trackers that can be paired has been paired
            match_end_flag = 1;
        }
        else
        {
            bboxes_paired[max_IOU_bbox_index] = 1;
            trackers_paired[max_IOU_tracker_index] = 1;
            PairedResult paired_result = {0, trackers_.begin() + max_IOU_tracker_index, car_bbox_msg.bboxs[max_IOU_bbox_index]};
            results.push_back(paired_result);
        }
    }
    // push in the unpaired trackers
    std::vector<Tracker>::iterator tracker_iterator = trackers_.begin();
    for (int index = 0; index < trackers_.size(); index++, tracker_iterator++)
    {
        if (trackers_paired[index] == 0)
        {
            PairedResult paired_result = {1, tracker_iterator};
            results.push_back(paired_result);
        }
    }
    // push in the unpaired bboxes
    for (int index = 0; index < car_bbox_msg.bboxs.size(); index++)
    {
        if (bboxes_paired[index] == 0)
        {
            PairedResult paired_result = {2, trackers_.end(), car_bbox_msg.bboxs[index]};
            results.push_back(paired_result);
        }
    }
    return results;
}

void TrackerManager::auto_release()
{
    for (int i = 0; i < trackers_.size(); i++)
    {
        if (trackers_[i].get_confidence(trackers_[i].get_id()) < confidence_threshold_)
        {
            trackers_.erase(trackers_.begin() + i);
        }
    }
}

void TrackerManager::clear_mask_in_bbox(cv::Mat mask, radar_msgs::msg::Bbox bbox)
{
    // make sure not over range
    int max_x = std::min(mask.cols - 1, int(bbox.x_max));
    // delete the points under the car to avoid being affected by the reflection of light
    int max_y = std::min(mask.rows - 1, int(2 * bbox.y_max - bbox.y_min));
    int min_x = std::max(0, int(bbox.x_min));
    int min_y = std::max(0, int(bbox.y_min));

    int w = max_x - min_x;
    int h = max_y - min_y;
    cv::Rect rect(min_x, min_y, w, h);
    rect &= cv::Rect(0, 0, mask.cols, mask.rows);
    // set 0 the points in rectangle, meaning no foreground
    mask(rect).setTo(0);
}

Tracker::Tracker(radar_msgs::msg::Bbox bbox, std::string config_path) : id_confidence_(MAX_CAR_NUMBER, 0)
{
    // // test
    // past_confidence_alpha_ = 0.98;
    // decrease_confidence_alpha_ = 0.8;
    YAML::Node yaml_config;
    try
    {
        yaml_config = YAML::LoadFile(config_path);
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "YAML 解析错误: " << e.what() << std::endl;
    }
    past_confidence_alpha_ = yaml_config["tracker"]["past_confidence_alpha"].as<double>();
    decrease_confidence_alpha_ = yaml_config["tracker"]["decrease_confidence_alpha"].as<double>();

    const int x_length = 8;
    x_ = Eigen::VectorXd::Zero(x_length);
    P_ = Eigen::MatrixXd::Zero(x_length, x_length);

    x_(0) = (bbox.x_max + bbox.x_min) / 2;
    x_(1) = (bbox.y_max + bbox.y_min) / 2;
    x_(2) = (bbox.x_max - bbox.x_min);
    x_(3) = (bbox.y_max - bbox.y_min);

    // init P_
    P_.setIdentity();

    P_ *= 10;
    // the first position is more convincing than velocity of them
    for (int i = 4; i < 8; i++)
    {
        P_(i, i) *= 10000;
    }

    ekf_ = std::make_shared<tools::ExtendedKalmanFilter>(x_, P_);

    update_confidence(bbox.class_id, bbox.class_confidence);

    // if (bbox.class_id == -1) {
    //     for (int i = 0; i < MAX_CAR_NUMBER; i++) {
    //         id_confidence_[i] += 0.1;
    //     }
    // }
    // else {
    //     id_confidence_[bbox.class_id] += 0.3 * bbox.class_confidence;
    // }
}

Tracker::~Tracker()
{
    // delete ekf_;
}

/******************************************************************************************************************
    @brief get the most possible id of the tracker
    @note the id confidence is an array that contains confidence of all ids,
        what you get is just the most confidence one of them
******************************************************************************************************************/
int Tracker::get_id()
{
    int most_convincing_id = -1;
    double max_confidence = 0.0;
    for (int i = 0; i < MAX_CAR_NUMBER; i++)
    {
        if (id_confidence_[i] > max_confidence)
        {
            most_convincing_id = i;
            max_confidence = id_confidence_[i];
        }
    }
    return most_convincing_id;
}

/******************************************************************************************************************
    @brief update the confidence
    @param id id
    @param confidence confidence of id,
        0 represent even you don't believe it so that confidence of all id will decrease rapidly
******************************************************************************************************************/
void Tracker::update_confidence(int id, double confidence)
{
    double alpha = past_confidence_alpha_; // how you trust the past
    if (confidence == 0.0)
    {                                       // if you don't believe your judge
        alpha = decrease_confidence_alpha_; //  decrease all id confidence
    }

    /* plan A:
    if a car is extinguished, add both red and blue confidence
    */

    if (id < MAX_CAR_NUMBER)
    {
        for (int i = 0; i < MAX_CAR_NUMBER; i++)
        {
            id_confidence_[i] = alpha * id_confidence_[i] + (1 - alpha) * confidence * (id == i);
        }
    }
    else
    {
        for (int i = 0; i < MAX_CAR_NUMBER; i++)
        {
            id_confidence_[i] = alpha * id_confidence_[i] + (1 - alpha) * confidence * ((id - 6 == i) + (id - 12 == i));
        }
    }

    /* plan B
    if a car is extinguished, if it's more likely to be red, add red confidence, if blue, add blue

    if (id > MAX_CAR_NUMBER) {
        if (id_confidence_[id % 6] > id_confidence_[id - 6]) {  // more likely to be blue
            id = id % 6;
        }
        else {
            id = id - 6;
        }
    }
    for (int i = 0; i < MAX_CAR_NUMBER; i++) {
        id_confidence_[i] = alpha * id_confidence_[i] + (1 - alpha) * confidence * (id == i);
    }
    */
}

/******************************************************************************************************************
    @brief update the condition of tracker by bbox
    @param bbox the matched bbox of a new yolo result
******************************************************************************************************************/
void Tracker::update(radar_msgs::msg::Bbox bbox)
{
    // about z, the infomation about car in this frame
    Eigen::VectorXd z(4);
    z(0) = (bbox.x_max + bbox.x_min) / 2;
    z(1) = (bbox.y_max + bbox.y_min) / 2;
    z(2) = (bbox.x_max - bbox.x_min);
    z(3) = (bbox.y_max - bbox.y_min);
    // confidence_[bbox.class_id] = bbox.class_confidence;

    predict();

    /* Followed by some constant used in R
    refence: Bot-SORT (https://arxiv.org/pdf/2206.14651) */
    const double sigma_m = 0.05;
    Eigen::MatrixXd H{{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};

    Eigen::MatrixXd R(4, 4);
    R.setZero();
    R.diagonal() << pow(sigma_m * x_(2), 2), pow(sigma_m * x_(3), 2),
        pow(sigma_m * x_(2), 2), pow(sigma_m * x_(3), 2);
    ekf_->update(z, H, R);

    x_ = ekf_->x;
    update_confidence(bbox.class_id, bbox.class_confidence);
}

/******************************************************************************************************************
    @brief update the condition of tracker when no bbox is matched
    @param mask the result of background substractor, showing the foreground (or the moving parts of the frame)
******************************************************************************************************************/
void Tracker::update(cv::Mat mask)
{
    MeanShift mean_shift(mask, get_bbox());
    auto bbox = mean_shift.do_mean_shift();
    if (bbox.class_confidence == 0.0)
    {
        update_confidence(-1, 0.0); //
    }
    bbox.class_id = -1; // decrease confidence of all id
    update(bbox);
}

/******************************************************************************************************************
    @brief update the condition of tracker only by inertia
******************************************************************************************************************/
void Tracker::update()
{
    predict();
    auto bbox = get_bbox();
    bbox.class_id = -1; // decrease confidence of all id
    bbox.class_confidence = 0.0;
    update(bbox);
}

/******************************************************************************************************************
    @brief Predict x, y, w, h of bbox by inertia， using Kalman filter
******************************************************************************************************************/
void Tracker::predict()
{
    Eigen::MatrixXd F{{1.0, 0.0, 0.0, 0.0, dt_, 0.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0, 0.0, dt_, 0.0, 0.0},
                      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt_, 0.0},
                      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt_},
                      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

    /* Followed by some constant used in Q
    refence: Bot-SORT (https://arxiv.org/pdf/2206.14651) */
    const double sigma_p = 0.05, sigma_v = 0.00625;
    Eigen::MatrixXd Q(8, 8);
    Q.setZero();
    Q.diagonal() << pow((sigma_p * x_(2)), 2), pow((sigma_p * x_(3)), 2),
        pow((sigma_p * x_(2)), 2), pow((sigma_p * x_(3)), 2),
        pow((sigma_v * x_(2)), 2), pow((sigma_v * x_(3)), 2),
        pow((sigma_v * x_(2)), 2), pow((sigma_v * x_(3)), 2);

    ekf_->predict(F, Q);
}

void Tracker::set_delta_time(double delta_time)
{
    dt_ = delta_time;
}

radar_msgs::msg::Bbox Tracker::get_bbox()
{
    radar_msgs::msg::Bbox bbox;
    bbox.class_id = get_id();
    bbox.class_confidence = get_confidence(bbox.class_id);
    bbox.x_min = x_(0) - x_(2) / 2;
    bbox.x_max = x_(0) + x_(2) / 2;
    bbox.y_min = x_(1) - x_(3) / 2;
    bbox.y_max = x_(1) + x_(3) / 2;
    return bbox;
}

void TrackerManager::record(const radar_msgs::msg::CarBbox msg, cv::Mat frame)
{
    static int count = 0;
    for (auto bbox : msg.bboxs)
    {
        // if (bbox.class_id != -1) {
        cv::Scalar color = (bbox.class_id < 6) ? cv::Scalar(255, 128, 0) : cv::Scalar(50, 50, 255);
        if (bbox.x_min > 0 || bbox.y_min > 20 || bbox.x_max < frame.cols - 50 || bbox.y_max < frame.rows)
        {
            cv::rectangle(frame, cv::Point(bbox.x_min, bbox.y_min), cv::Point(bbox.x_max, bbox.y_max), color, 10);
            cv::putText(frame, std::to_string((bbox.class_id) % 6 + 1), cv::Point(bbox.x_min + 40, bbox.y_min - 10), cv::FONT_HERSHEY_PLAIN, 6, color, 6);
        }
        // }
    }
    // cv::imshow("frame", frame);
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::time_point::clock::now();
    recorder_->record(frame, current_time);

    count++;
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

MeanShift::MeanShift(cv::Mat mask, radar_msgs::msg::Bbox init_bbox)
{
    mask_ = mask;
    bbox_ = init_bbox;
    stop_ = 0;
}

radar_msgs::msg::Bbox MeanShift::do_mean_shift()
{
    bbox_.class_id = -1;
    bbox_.class_confidence *= 0.5;
    while (!stop_)
    {
        shift_once();
    }
    return bbox_;
}

void MeanShift::shift_once()
{
    const double threshold = 0.1; // If the moving points rate in the bbox is lower than threshold, we assume it loss track.
    int row_sum = 0;
    int col_sum = 0;
    int weight_sum = 0;
    for (int row = bbox_.y_min; row < bbox_.y_max; row++)
    {
        for (int col = bbox_.x_min; col < bbox_.x_max; col++)
        {
            if (mask_.at<uchar>(row, col) != 0)
            {
                row_sum += row;
                col_sum += col;
                weight_sum++;
            }
        }
    }

    if (weight_sum <= threshold * (bbox_.x_max - bbox_.x_min) * (bbox_.y_max - bbox_.y_min))
    { // no foreground in bbox
        bbox_.class_confidence = 0.0f;
        stop_ = true;
    }
    else
    {
        int shift_x = col_sum / weight_sum - (bbox_.x_min + bbox_.x_max) / 2;
        int shift_y = row_sum / weight_sum - (bbox_.y_min + bbox_.y_max) / 2;
        bbox_.x_max += shift_x;
        bbox_.x_min += shift_x;
        bbox_.y_max += shift_y;
        bbox_.y_min += shift_y;

        double shift_distance = sqrt(shift_x * shift_x + shift_y * shift_y);
        if (shift_distance < EPSILON)
        {
            stop_ = true;
        }
    }
}