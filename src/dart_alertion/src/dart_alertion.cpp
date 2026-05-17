#include "include/dart_alertion.hpp"

DartAlertion::DartAlertion()
    : Node("dart_alertion_node"),
      min_ContourArea(300.0), max_ContourArea(5000.0),
      circularity_threshold(0.4),
      brightness_threshold(100.0),
      brightness_change_threshold(30.0),
      whiteness_threshold(190.0),
      min_whiteness_for_flash(190.0),
      current_state(normal), previous_state(normal),
      circle_tracked(false),
      alert(false),
      last_brightness(0),
      last_whiteness(0),
      is_tracking_move(false)
{
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera/image_compressed", 10, std::bind(&DartAlertion::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Int8>("dart_gate_status", 10);

    RCLCPP_INFO(this->get_logger(), "DartAlertion node initialized");
}
void DartAlertion::callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try{
        cv::Mat image = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if(!init_flag)
        {
            int frame_width = image.cols;
            int frame_height = image.rows;
            region = cv::Rect(
                frame_width * 2 / 3,
                0,
                frame_width / 6,
                frame_height / 5
            );

            last_circle_center = cv::Point(
                region.x + region.width / 2,
                region.y + region.height / 2
            );
            last_circle_radius = 30.0;
            init_flag = 1;
        }
        ProcessFrame(image);
    } catch (const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception in callback: %s", e.what());
    }
}

void DartAlertion::StartMoveTracking()
{
    is_tracking_move = true;
    move_start_time = std::chrono::steady_clock::now();
    circle_center_history.clear();
    RCLCPP_INFO(this->get_logger(), "Start move-tracking");
}

void DartAlertion::UpdateMoveTracking(const cv::Point& current_center)
{
    if (!is_tracking_move) return;
    circle_center_history.push_back(current_center);
    if (circle_center_history.size() > max_history_points) circle_center_history.pop_front();

    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - move_start_time);
    if(duration.count() >= 1000) AnalyzeMove();
}

void DartAlertion::AnalyzeMove()
{
    if (!is_tracking_move || circle_center_history.size() < 2)
    {
        is_tracking_move = false;
        return;
    } 

    int start_y = circle_center_history.front().y;
    int end_y = circle_center_history.back().y;
    int change = start_y - end_y;
    int gate_status = STABLE;

    count++;
    std::cout << "change: " << change << std::endl;
    if (change > 5)
    {
        gate_status = OPENING;
        is_alert_pub = true;
        std::cout << "========================================" << std::endl;
        std::cout << "!!! Dart Gate is Opening !!!" << std::endl;
        std::cout << "========================================" << std::endl;
        PublishStatus(gate_status);
    }
    else if (change < -5)
    {
        gate_status = CLOSING;
        is_alert_pub = true;
        std::cout << "========================================" << std::endl;
        std::cout << "!!! Dart Gate is Closing !!!" << std::endl;
        std::cout << "========================================" << std::endl;
        PublishStatus(gate_status);
    }
    else PublishStatus(gate_status);

    is_tracking_move = false;
    circle_center_history.clear();
}

void DartAlertion::ProcessFrame(const cv::Mat& frame)
{
    cv::Point circle_center;
    float circle_radius;
    double circularity;
    double circle_brightness;
    bool circle_detected;

    bool flash_detected = DartDetection(frame, circle_center, circle_radius, circularity,
                                        circle_brightness, circle_detected, frame_count_);

    if (flash_detected) 
    {
        if(alert && !is_tracking_move) StartMoveTracking();
    }

    if (is_tracking_move) UpdateMoveTracking(circle_center);
    else PublishStatus(STABLE);

    // Debugging
    frame_count_++;
}

void DartAlertion::PublishStatus(int gate_status){
    auto msg = std_msgs::msg::Int8();
    msg.data = gate_status;
    pub_->publish(msg);
    if (gate_status == OPENING) {
        RCLCPP_INFO(this->get_logger(), "Dart gate status published: OPENING");
    } else if (gate_status == CLOSING) {
        RCLCPP_INFO(this->get_logger(), "Dart gate status published: CLOSING");
    } else {
        RCLCPP_INFO(this->get_logger(), "Dart gate status published: STABLE");
    }
}

double DartAlertion::CalculateCircularity(const std::vector<cv::Point>& contour)
{
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);

    if(perimeter <= 0) return 0;

    double circularity = 4 * CV_PI * area / (perimeter * perimeter);
    return circularity;
}

bool DartAlertion::DetectCircularContour(const cv::Mat& frame, std::vector<cv::Point>& best_contour,
                            cv::Point& circle_center, float& circle_radius, double& circularity)
{
    cv::Mat roi = frame(region);
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, brightness_threshold, 255, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(contours.empty()) return false;

    // search for the max circularity
    double best_circularity = 0;
    int best_index = -1;
    for(size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        
        if(area < min_ContourArea || area > max_ContourArea) continue;

        // RCLCPP_INFO(this->get_logger(), "Area: %lf", area);
        double current_circularity = CalculateCircularity(contours[i]);
        if(current_circularity > best_circularity && current_circularity >= circularity_threshold)
        {
            best_circularity = current_circularity;
            best_index = i;
        }
    } 
    if(best_index == -1) return false;
    best_contour = contours[best_index];
    circularity = best_circularity;

    cv::Point2f center;
    cv::minEnclosingCircle(best_contour, center, circle_radius);
    circle_center = cv::Point(
        region.x + static_cast<int>(center.x),
        region.y + static_cast<int>(center.y)
    );
    return true;
}

double DartAlertion::CalculateCircleBrightness(const cv::Mat& frame, const cv::Point& center, float radius)
{
    if(radius <= 5) return 0;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::circle(mask, center, static_cast<int>(radius), cv::Scalar(255), -1);

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar mean_value = cv::mean(gray, mask);

    return mean_value[0];
}

double DartAlertion::CalculateCircleWhiteness(const cv::Mat& frame, const cv::Point& center, float radius)
{
    if(radius <= 5) return 0;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::circle(mask, center, static_cast<int>(radius), cv::Scalar(255), -1);
    
    cv::Scalar mean_value = cv::mean(frame, mask);
    double whiteness = (mean_value[0] + mean_value[1] + mean_value[2]) / 3.0;
    
    return whiteness;
}

void DartAlertion::AnalyzeWhitenessChange(double current_whiteness)
{
    whiteness_history.push_back(current_whiteness);
    if(whiteness_history.size() > history_size) whiteness_history.pop_front();
    last_whiteness = current_whiteness;
}

bool DartAlertion::IsWhiteLight(double current_whiteness)
{
    if(whiteness_history.size() < history_size) return false;
    
    if(current_whiteness < whiteness_threshold) return false;
    
    double recent_avg = 0;
    for(size_t i = 0; i < whiteness_history.size() - 1; ++i)
    {
        recent_avg += whiteness_history[i];
    }
    recent_avg /= (whiteness_history.size() - 1);
    
    double whiteness_change = current_whiteness - recent_avg;
    
    if(whiteness_change > 50.0 && current_whiteness >= min_whiteness_for_flash)
    {
        return true;
    }
    
    return false;
}

DartAlertion::FlashState DartAlertion::AnalyzeBrightnessChange(double current_brightness, double current_whiteness, bool circle_detected)
{
    brightness_history.push_back(current_brightness);
    if(brightness_history.size() > history_size) brightness_history.pop_front();

    if(brightness_history.size() < 3)
    {
        last_brightness = current_brightness;
        return normal;
    }

    double recent_avg = 0;
    int cnt = 0;
    for(size_t i = 0; i < brightness_history.size() - 1; ++i)
    {
        recent_avg += brightness_history[i];
        cnt++;
    }
    if(cnt > 0) recent_avg /= cnt;

    double brightness_change = std::abs(current_brightness - recent_avg);

    FlashState new_state = normal;
    
    bool is_white_flash = IsWhiteLight(current_whiteness);
    
    if(brightness_change > brightness_change_threshold && is_white_flash) 
    {
        new_state = flashing;
    }
    else if(last_circle_detected && !circle_detected && current_brightness < 30 && current_whiteness < 30) 
    {
        if(last_whiteness > whiteness_threshold)
        {
            new_state = flashing;
        }
    }
    else if(!last_circle_detected && circle_detected && 
            current_brightness > brightness_threshold && 
            current_whiteness > whiteness_threshold) 
    {
        new_state = flashing;
    }

    last_brightness = current_brightness;
    return new_state;
}

bool DartAlertion::DecideAlert(bool flash_detected)
{
    auto now = std::chrono::steady_clock::now();
    auto during_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_alert_time);

    if(flash_detected && during_time.count() >= min_alert_interval)
    {
        alert = true;
        last_alert_time = now;

        return true;
    }
    else if(!flash_detected) alert = false;

    return false;
}

bool DartAlertion::DetectFlashEvent(DartAlertion::FlashState new_state, double current_brightness, double current_whiteness, bool circle_detected)
{
    previous_state = current_state;
    current_state = new_state;

    bool flash_detected = false;
    if(previous_state == normal)
        if(current_state == flashing)
            flash_detected = true;
    else if(previous_state == flashing && current_state == normal);

    if(flash_detected)
    {
        auto now = std::chrono::steady_clock::now();
        auto during_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flash_time);
        if(during_time.count() >= min_flash_interval)
        {
            last_flash_time = now;
            flash_times.push_back(now);
        }

        if(flash_times.size() > 50) flash_times.erase(flash_times.begin());
        
        DecideAlert(true);
        return true;
    }
    else DecideAlert(false);
    return false;
}

bool DartAlertion::DartDetection(const cv::Mat& frame, cv::Point& detected_circle_center, float& detected_circle_radius,
                    double& detected_circularity, double& circle_brightness, bool& circle_detected, int frame_count)
{
    detected_circle_center = last_circle_center;
    detected_circle_radius = last_circle_radius;
    detected_circularity = 0;
    circle_brightness = 0;
    circle_detected = false;
    is_alert_pub = false;
    
    double circle_whiteness = 0;

    cv::Point circle_center;
    float circle_radius;
    double circularity;
    std::vector<cv::Point> contour;
    bool circle_found = DetectCircularContour(frame, contour, circle_center, circle_radius, circularity);

    if(circle_found)
    {
        circle_brightness = CalculateCircleBrightness(frame, circle_center, circle_radius);
        circle_whiteness = CalculateCircleWhiteness(frame, circle_center, circle_radius);
        detected_circle_center = circle_center;
        detected_circle_radius = circle_radius;
        detected_circularity = circularity;
        circle_tracked = true;
        circle_detected = true;

        last_circle_center = circle_center;
        last_circle_radius = circle_radius;
    }
    else
    {
        circle_brightness = CalculateCircleBrightness(frame, last_circle_center, last_circle_radius);
        circle_whiteness = CalculateCircleWhiteness(frame, last_circle_center, last_circle_radius);
        detected_circle_center = last_circle_center;
        detected_circle_radius = last_circle_radius;
        circle_tracked = false;
        circle_detected = false;
    }
    
    AnalyzeWhitenessChange(circle_whiteness);
    
    FlashState new_state = AnalyzeBrightnessChange(circle_brightness, circle_whiteness, circle_found);
    bool flash_detected = DartAlertion::DetectFlashEvent(new_state, circle_brightness, circle_whiteness, circle_found);

    last_circle_detected = circle_found;

    // Visualization
    cv::Mat display_frame = frame.clone();
    
    cv::rectangle(display_frame, region, cv::Scalar(0, 255, 0), 2);
    
    if(circle_found && circle_radius > 0)
    {
        cv::circle(display_frame, circle_center, static_cast<int>(circle_radius), 
                    cv::Scalar(0, 255, 255), 2);
        cv::circle(display_frame, circle_center, 3, cv::Scalar(0, 255, 255), -1);

        std::stringstream area_text;
        area_text << "Area: " << static_cast<int>(CV_PI * circle_radius * circle_radius);
        cv::putText(display_frame, area_text.str(),
                    cv::Point(circle_center.x - 40, circle_center.y - static_cast<int>(circle_radius) - 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        std::stringstream brightness_text;
        brightness_text << "Brightness: " << static_cast<int>(circle_brightness);
        cv::putText(display_frame, brightness_text.str(),
                    cv::Point(circle_center.x - 40, circle_center.y - static_cast<int>(circle_radius) - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        std::stringstream whiteness_text;
        whiteness_text << "Whiteness: " << static_cast<int>(circle_whiteness);
        cv::putText(display_frame, whiteness_text.str(),
                    cv::Point(circle_center.x - 40, circle_center.y - static_cast<int>(circle_radius) - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                    circle_whiteness > whiteness_threshold ? cv::Scalar(255, 255, 255) : cv::Scalar(100, 100, 100), 
                    1);
    }
    else
    {
        cv::circle(display_frame, last_circle_center, static_cast<int>(last_circle_radius), 
                    cv::Scalar(255, 0, 0), 1);
    }
    
    std::stringstream state_text;
    state_text << "State: " << (current_state == flashing ? "Flashing" : "Normal");
    cv::putText(display_frame, state_text.str(), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                current_state == flashing ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
    
    std::stringstream frame_text;
    frame_text << "Frame: " << frame_count;
    cv::putText(display_frame, frame_text.str(), cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    std::stringstream brightness_text;
    brightness_text << "Brightness: " << static_cast<int>(circle_brightness);
    cv::putText(display_frame, brightness_text.str(), cv::Point(10, 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    std::stringstream whiteness_text;
    whiteness_text << "Whiteness: " << static_cast<int>(circle_whiteness);
    cv::putText(display_frame, whiteness_text.str(), cv::Point(10, 120),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                circle_whiteness > whiteness_threshold ? cv::Scalar(255, 255, 255) : cv::Scalar(200, 200, 200), 
                2);
    
    std::stringstream tracking_text;
    tracking_text << "Movement Tracking: " << (is_tracking_move ? "ON" : "OFF");
    cv::putText(display_frame, tracking_text.str(), cv::Point(10, 150),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                is_tracking_move ? cv::Scalar(0, 255, 255) : cv::Scalar(150, 150, 150), 
                2);

    
    if(is_alert_pub)
    {
        cv::putText(display_frame, "ALERT: DART GATE OPENING!", 
                    cv::Point(display_frame.cols/2 - 200, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
        
        cv::rectangle(display_frame, cv::Rect(0, 0, display_frame.cols, display_frame.rows),
                    cv::Scalar(0, 0, 255), 5);
    }

    cv::imshow("Dart Gate Detection", display_frame);
    cv::waitKey(1);

    return flash_detected;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<DartAlertion>();
    RCLCPP_INFO(node->get_logger(), "Dart Alertion node started successfully");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

