#ifndef DART_ALERTION_HPP
#define DART_ALERTION_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>

class DartAlertion : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

    cv::Rect region;
    double min_ContourArea;
    double max_ContourArea;
    double circularity_threshold;
    std::deque<double> brightness_history;
    const int history_size = 5;
    std::deque<double> whiteness_history;
    double whiteness_threshold;
    double min_whiteness_for_flash;

    enum FlashState{
        normal,
        flashing
    };
    FlashState previous_state;
    FlashState current_state;

    cv::Point last_circle_center;
    float last_circle_radius;
    bool circle_tracked;
    bool last_circle_detected;

    std::chrono::steady_clock::time_point last_flash_time;
    const int min_flash_interval = 50; //ms
    std::chrono::steady_clock::time_point last_alert_time;
    const int min_alert_interval = 20000; //ms

    std::deque<std::chrono::steady_clock::time_point> flash_times;
    bool alert, is_alert_pub;
    double last_brightness;
    double last_whiteness;
    double frame_interval; //ms
    int count = 0;

    void PublishStatus(bool gate_open);
    void ProcessFrame(const cv::Mat& frame);
    int frame_count_ = 0;

    bool init_flag = 0;
    bool is_up = 0;

    bool is_tracking_move = false;
    std::chrono::steady_clock::time_point move_start_time;
    std::deque<cv::Point> circle_center_history;
    const int max_history_points = 30;

    double brightness_threshold;
    double brightness_change_threshold;

    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    double CalculateCircularity(const std::vector<cv::Point>& contour);
    bool DetectCircularContour(const cv::Mat& frame, std::vector<cv::Point>& best_contour,
                               cv::Point& circle_center, float& circle_radius, double& circularity);
    double CalculateCircleBrightness(const cv::Mat& frame, const cv::Point& center, float radius);
    double CalculateCircleWhiteness(const cv::Mat& frame, const cv::Point& center, float radius);
    void AnalyzeWhitenessChange(double current_whiteness);
    bool IsWhiteLight(double current_whiteness);
    FlashState AnalyzeBrightnessChange(double current_brightness, double current_whiteness, bool circle_detected);
    bool DecideAlert(bool flash_detected);
    bool DetectFlashEvent(FlashState new_state, double current_brightness, double current_whiteness, bool circle_detected);
    bool DartDetection(const cv::Mat& frame, cv::Point& detected_circle_center, float& detected_circle_radius,
                       double& detected_circularity, double& circle_brightness, bool& circle_detected, int frame_count);

    void StartMoveTracking();
    void UpdateMoveTracking(const cv::Point& current_center);
    void AnalyzeMove();
public:
    DartAlertion();
};

#endif //DART_ALERTION_HPP