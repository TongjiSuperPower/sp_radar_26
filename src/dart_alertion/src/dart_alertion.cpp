#include "../include/dart_alertion.hpp"

DartAlertion::DartAlertion()
    : Node("dart_alertion")
{
    std::string config_path = "src/dart_alertion/config/config.yaml";
    loadConfig(config_path);

    rclcpp::QoS qos_profile(1);
    qos_profile.best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "camera/image_compressed", qos_profile,
        std::bind(&DartAlertion::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Int8>("dart_gate_status", 10);

    RCLCPP_INFO(this->get_logger(), "DartAlertion node initialized");
}

void DartAlertion::loadConfig(const std::string& config_path)
{
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load config: %s", e.what());
        throw;
    }

    // 区域比例
    region_x_ratio_      = config["region_x_ratio"].as<double>();
    region_y_ratio_      = config["region_y_ratio"].as<double>();
    region_width_ratio_  = config["region_width_ratio"].as<double>();
    region_height_ratio_ = config["region_height_ratio"].as<double>();

    // 圆形检测
    min_contour_area_    = config["min_contour_area"].as<int>();
    max_contour_area_    = config["max_contour_area"].as<int>();
    circularity_threshold_ = config["circularity_threshold"].as<double>();
    contour_binary_threshold_ = config["contour_binary_threshold"].as<int>();
    circle_reappear_brightness_threshold_ = config["circle_reappear_brightness_threshold"].as<double>();
    circle_reappear_whiteness_threshold_  = config["circle_reappear_whiteness_threshold"].as<double>();

    // 亮度/白度变化
    history_size_             = config["history_size"].as<int>();
    brightness_change_threshold_ = config["brightness_change_threshold"].as<double>();
    whiteness_threshold_      = config["whiteness_threshold"].as<double>();
    min_whiteness_for_flash_  = config["min_whiteness_for_flash"].as<double>();
    whiteness_change_threshold_ = config["whiteness_change_threshold"].as<double>();

    // 圆丢失容忍
    lost_frames_max_ = config["lost_frames_max"].as<int>();

    // 闪光 & 报警间隔
    min_flash_interval_ms_ = config["min_flash_interval_ms"].as<int>();
    min_alert_interval_ms_ = config["min_alert_interval_ms"].as<int>();

    // 移动跟踪
    move_tracking_duration_ms_ = config["move_tracking_duration_ms"].as<int>();
    max_history_points_        = config["max_history_points"].as<int>();
    opening_change_min_ = config["opening_change_min"].as<int>();
    opening_change_max_ = config["opening_change_max"].as<int>();
    closing_change_min_ = config["closing_change_min"].as<int>();
    closing_change_max_ = config["closing_change_max"].as<int>();

    // 圆消失条件
    circle_disappear_brightness_max_ = config["circle_disappear_brightness_max"].as<double>();
    circle_disappear_whiteness_max_  = config["circle_disappear_whiteness_max"].as<double>();

    RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully.");
}

void DartAlertion::callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        cv::Mat image = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if (!init_flag_) {
            int w = image.cols, h = image.rows;
            region_ = cv::Rect(
                static_cast<int>(w * region_x_ratio_),
                static_cast<int>(h * region_y_ratio_),
                static_cast<int>(w * region_width_ratio_),
                static_cast<int>(h * region_height_ratio_)
            );
            last_circle_center_ = cv::Point(region_.x + region_.width/2,
                                           region_.y + region_.height/2);
            init_flag_ = true;
        }
        ProcessFrame(image);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in callback: %s", e.what());
    }
}

void DartAlertion::ProcessFrame(const cv::Mat& frame)
{
    cv::Point circle_center;
    float circle_radius;
    double circularity, circle_brightness;
    bool circle_detected;

    bool flash_detected = DartDetection(frame, circle_center, circle_radius,
                                        circularity, circle_brightness, circle_detected);

    if (flash_detected && alert_ && !is_tracking_move_) {
        StartMoveTracking();
    }

    if (is_tracking_move_) {
        UpdateMoveTracking(circle_center);
    } else {
        PublishStatus(STABLE);
    }

    frame_count_++;
}

// ---------- 移动跟踪 ----------
void DartAlertion::StartMoveTracking()
{
    is_tracking_move_ = true;
    move_start_time_ = std::chrono::steady_clock::now();
    circle_center_history_.clear();
    RCLCPP_INFO(this->get_logger(), "Start move-tracking");
}

void DartAlertion::UpdateMoveTracking(const cv::Point& current_center)
{
    if (!is_tracking_move_) return;
    circle_center_history_.push_back(current_center);
    if (circle_center_history_.size() > static_cast<size_t>(max_history_points_))
        circle_center_history_.pop_front();

    auto now = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(now - move_start_time_);
    if (dur.count() >= move_tracking_duration_ms_)
        AnalyzeMove();
}

void DartAlertion::AnalyzeMove()
{
    if (!is_tracking_move_ || circle_center_history_.size() < 2) {
        is_tracking_move_ = false;
        return;
    }

    int start_y = circle_center_history_.front().y;
    int end_y   = circle_center_history_.back().y;
    int change  = start_y - end_y;   // 正 → 向上移动（开门），负 → 向下移动（关门）
    int gate_status = STABLE;

    std::cout << "change: " << change << std::endl;

    if (change >= opening_change_min_ && change <= opening_change_max_) {
        gate_status = OPENING;
        is_alert_pub_ = true;
        std::cout << "========================================" << std::endl;
        std::cout << "!!! Dart Gate is Opening !!!" << std::endl;
        std::cout << "========================================" << std::endl;
    }
    else if (change <= -closing_change_min_ && change >= -closing_change_max_) {
        gate_status = CLOSING;
        is_alert_pub_ = true;
        std::cout << "========================================" << std::endl;
        std::cout << "!!! Dart Gate is Closing !!!" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    PublishStatus(gate_status);
    is_tracking_move_ = false;
    circle_center_history_.clear();
}

// ---------- 圆形检测 ----------
double DartAlertion::CalculateCircularity(const std::vector<cv::Point>& contour)
{
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 0) return 0;
    return 4 * CV_PI * area / (perimeter * perimeter);
}

bool DartAlertion::DetectCircularContour(const cv::Mat& frame, std::vector<cv::Point>& best_contour,
                                         cv::Point& circle_center, float& circle_radius, double& circularity)
{
    cv::Mat roi = frame(region_);
    cv::Mat gray, binary;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, contour_binary_threshold_, 255, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    double best_circularity = 0;
    int best_idx = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < min_contour_area_ || area > max_contour_area_) continue;

        double c = CalculateCircularity(contours[i]);
        if (c > best_circularity && c >= circularity_threshold_) {
            best_circularity = c;
            best_idx = i;
        }
    }
    if (best_idx == -1) return false;

    best_contour = contours[best_idx];
    circularity = best_circularity;

    cv::Point2f center;
    cv::minEnclosingCircle(best_contour, center, circle_radius);
    circle_center = cv::Point(region_.x + static_cast<int>(center.x),
                              region_.y + static_cast<int>(center.y));
    return true;
}

double DartAlertion::CalculateCircleBrightness(const cv::Mat& frame, const cv::Point& center, float radius)
{
    if (radius <= 5) return 0;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::circle(mask, center, static_cast<int>(radius), cv::Scalar(255), -1);
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    return cv::mean(gray, mask)[0];
}

double DartAlertion::CalculateCircleWhiteness(const cv::Mat& frame, const cv::Point& center, float radius)
{
    if (radius <= 5) return 0;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::circle(mask, center, static_cast<int>(radius), cv::Scalar(255), -1);
    cv::Scalar mean_val = cv::mean(frame, mask);
    return (mean_val[0] + mean_val[1] + mean_val[2]) / 3.0;
}

// ---------- 白度分析 ----------
void DartAlertion::AnalyzeWhitenessChange(double current_whiteness)
{
    whiteness_history_.push_back(current_whiteness);
    if (whiteness_history_.size() > static_cast<size_t>(history_size_))
        whiteness_history_.pop_front();
    last_whiteness_ = current_whiteness;
}

bool DartAlertion::IsWhiteLight(double current_whiteness)
{
    if (whiteness_history_.size() < static_cast<size_t>(history_size_)) return false;
    if (current_whiteness < whiteness_threshold_) return false;

    double recent_avg = 0.0;
    for (size_t i = 0; i < whiteness_history_.size() - 1; ++i)
        recent_avg += whiteness_history_[i];
    recent_avg /= (whiteness_history_.size() - 1);

    double change = current_whiteness - recent_avg;
    return (change > whiteness_change_threshold_ && current_whiteness >= min_whiteness_for_flash_);
}

// ---------- 闪光状态判断（逻辑已统一）----------
DartAlertion::FlashState DartAlertion::AnalyzeBrightnessChange(
    double current_brightness, double current_whiteness,
    bool circle_detected, bool circle_lost_too_long)
{
    brightness_history_.push_back(current_brightness);
    if (brightness_history_.size() > static_cast<size_t>(history_size_))
        brightness_history_.pop_front();

    if (brightness_history_.size() < 3) {
        last_brightness_ = current_brightness;
        return normal;
    }

    double recent_avg = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < brightness_history_.size() - 1; ++i) {
        recent_avg += brightness_history_[i];
        cnt++;
    }
    if (cnt > 0) recent_avg /= cnt;

    double brightness_change = std::abs(current_brightness - recent_avg);
    FlashState new_state = normal;

    // 路径 A：圆持续存在且亮度、白度发生突增
    if (circle_detected && brightness_change > brightness_change_threshold_ && IsWhiteLight(current_whiteness)) {
        new_state = flashing;
    }
    // 路径 B：圆刚刚消失（且未丢失过久）
    else if (!circle_lost_too_long && last_circle_detected_ && !circle_detected &&
             current_brightness < circle_disappear_brightness_max_ &&
             current_whiteness < circle_disappear_whiteness_max_ &&
             last_whiteness_ > whiteness_threshold_) {
        new_state = flashing;
    }
    // 路径 C：圆重新出现（且未丢失过久）
    else if (!circle_lost_too_long && !last_circle_detected_ && circle_detected &&
             current_brightness > circle_reappear_brightness_threshold_ &&
             current_whiteness > circle_reappear_whiteness_threshold_) {
        new_state = flashing;
    }

    last_brightness_ = current_brightness;
    return new_state;
}

bool DartAlertion::DecideAlert(bool flash_detected)
{
    auto now = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_alert_time_);

    if (flash_detected && dur.count() >= min_alert_interval_ms_) {
        alert_ = true;
        last_alert_time_ = now;
        return true;
    } else if (!flash_detected) {
        alert_ = false;
    }
    return false;
}

bool DartAlertion::DetectFlashEvent(FlashState new_state)
{
    previous_state_ = current_state_;
    current_state_ = new_state;

    bool flash_detected = false;
    if (previous_state_ == normal && current_state_ == flashing) {
        flash_detected = true;
    }

    if (flash_detected) {
        auto now = std::chrono::steady_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flash_time_);
        if (dur.count() >= min_flash_interval_ms_) {
            last_flash_time_ = now;
            flash_times_.push_back(now);
        }
        if (flash_times_.size() > 50) flash_times_.erase(flash_times_.begin());
        DecideAlert(true);
        return true;
    } else {
        DecideAlert(false);
    }
    return false;
}

// ---------- 主检测流程 ----------
bool DartAlertion::DartDetection(const cv::Mat& frame,
                                 cv::Point& detected_circle_center,
                                 float& detected_circle_radius,
                                 double& detected_circularity,
                                 double& circle_brightness,
                                 bool& circle_detected)
{
    detected_circle_center = last_circle_center_;
    detected_circle_radius = last_circle_radius_;
    detected_circularity = 0.0;
    circle_brightness = 0.0;
    circle_detected = false;
    is_alert_pub_ = false;

    double circle_whiteness = 0.0;
    cv::Point circle_center;
    float circle_radius;
    double circularity;
    std::vector<cv::Point> contour;
    bool circle_found = DetectCircularContour(frame, contour, circle_center, circle_radius, circularity);

    // 圆丢失帧计数
    if (circle_found) {
        lost_frames_ = 0;
    } else {
        lost_frames_++;
    }
    bool lost_too_long = (lost_frames_ > lost_frames_max_);

    if (circle_found) {
        circle_brightness = CalculateCircleBrightness(frame, circle_center, circle_radius);
        circle_whiteness  = CalculateCircleWhiteness(frame, circle_center, circle_radius);
        detected_circle_center = circle_center;
        detected_circle_radius = circle_radius;
        detected_circularity = circularity;
        circle_tracked_ = true;
        circle_detected = true;
        last_circle_center_ = circle_center;
        last_circle_radius_ = circle_radius;
    } else {
        // 如果圆丢失时间过长，不再使用旧圆心计算，直接置零
        if (lost_too_long) {
            circle_brightness = 0.0;
            circle_whiteness  = 0.0;
            circle_tracked_ = false;
        } else {
            circle_brightness = CalculateCircleBrightness(frame, last_circle_center_, last_circle_radius_);
            circle_whiteness  = CalculateCircleWhiteness(frame, last_circle_center_, last_circle_radius_);
        }
        detected_circle_center = last_circle_center_;
        detected_circle_radius = last_circle_radius_;
        circle_detected = false;
    }

    AnalyzeWhitenessChange(circle_whiteness);
    FlashState new_state = AnalyzeBrightnessChange(circle_brightness, circle_whiteness,
                                                   circle_detected, lost_too_long);
    bool flash_detected = DetectFlashEvent(new_state);

    last_circle_detected_ = circle_detected;

    // ---------- 可视化（保留原有风格）----------
    cv::Mat display = frame.clone();
    cv::rectangle(display, region_, cv::Scalar(0,255,0), 2);

    if (circle_found && circle_radius > 0) {
        cv::circle(display, circle_center, static_cast<int>(circle_radius), cv::Scalar(0,255,255), 2);
        cv::circle(display, circle_center, 3, cv::Scalar(0,255,255), -1);
        std::stringstream ss;
        ss << "Area: " << static_cast<int>(CV_PI * circle_radius * circle_radius);
        cv::putText(display, ss.str(), cv::Point(circle_center.x-40, circle_center.y-static_cast<int>(circle_radius)-50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
        ss.str(""); ss << "Brightness: " << static_cast<int>(circle_brightness);
        cv::putText(display, ss.str(), cv::Point(circle_center.x-40, circle_center.y-static_cast<int>(circle_radius)-10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
        ss.str(""); ss << "Whiteness: " << static_cast<int>(circle_whiteness);
        cv::putText(display, ss.str(), cv::Point(circle_center.x-40, circle_center.y-static_cast<int>(circle_radius)-30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    circle_whiteness > whiteness_threshold_ ? cv::Scalar(255,255,255) : cv::Scalar(100,100,100), 1);
    } else {
        cv::circle(display, last_circle_center_, static_cast<int>(last_circle_radius_), cv::Scalar(255,0,0), 1);
    }

    std::stringstream state_text;
    state_text << "State: " << (current_state_ == flashing ? "Flashing" : "Normal");
    cv::putText(display, state_text.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 2,
                current_state_ == flashing ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0), 2);
    std::stringstream frame_text;
    frame_text << "Frame: " << frame_count_;
    cv::putText(display, frame_text.str(), cv::Point(10,100), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 2);
    std::stringstream bright_text;
    bright_text << "Brightness: " << static_cast<int>(circle_brightness);
    cv::putText(display, bright_text.str(), cv::Point(10,150), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 2);
    std::stringstream white_text;
    white_text << "Whiteness: " << static_cast<int>(circle_whiteness);
    cv::putText(display, white_text.str(), cv::Point(10,200), cv::FONT_HERSHEY_SIMPLEX, 2,
                circle_whiteness > whiteness_threshold_ ? cv::Scalar(255,255,255) : cv::Scalar(200,200,200), 2);
    std::stringstream track_text;
    track_text << "Movement Tracking: " << (is_tracking_move_ ? "ON" : "OFF");
    cv::putText(display, track_text.str(), cv::Point(10,250), cv::FONT_HERSHEY_SIMPLEX, 2,
                is_tracking_move_ ? cv::Scalar(0,255,255) : cv::Scalar(150,150,150), 2);

    if (is_alert_pub_) {
        cv::putText(display, "ALERT: DART GATE OPENING!",
                    cv::Point(display.cols/2 - 200, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 3);
        cv::rectangle(display, cv::Rect(0,0,display.cols,display.rows), cv::Scalar(0,0,255), 5);
    }

    cv::resize(display, display, cv::Size(1080, 720));

    cv::imshow("Dart Gate Detection", display);
    cv::waitKey(1);

    return flash_detected;
}

void DartAlertion::PublishStatus(int gate_status)
{
    auto msg = std_msgs::msg::Int8();
    msg.data = gate_status;
    pub_->publish(msg);
    if (gate_status == OPENING) {
        RCLCPP_INFO(this->get_logger(), "Dart gate status published: OPENING");
    } else if (gate_status == CLOSING) {
        RCLCPP_INFO(this->get_logger(), "Dart gate status published: CLOSING");
    }
}

// ---------- main ----------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DartAlertion>();
    RCLCPP_INFO(node->get_logger(), "Dart Alertion node started successfully");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}