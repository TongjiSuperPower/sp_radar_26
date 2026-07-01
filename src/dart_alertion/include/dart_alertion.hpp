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
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int8.hpp>

class DartAlertion : public rclcpp::Node
{
public:
    DartAlertion();

private:
    // ROS2 接口
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;

    // ---------- 可配置参数（YAML 加载） ----------
    // 检测区域比例
    double region_x_ratio_, region_y_ratio_;
    double region_width_ratio_, region_height_ratio_;

    // 圆形检测
    int min_contour_area_, max_contour_area_;
    double circularity_threshold_;
    int contour_binary_threshold_;              // 二值化阈值
    double circle_reappear_brightness_threshold_; // 圆重新出现时的亮度下限
    double circle_reappear_whiteness_threshold_;  // 圆重新出现时的白度下限（新增，原共用 whiteness_threshold）

    // 亮度 / 白度变化
    int history_size_;
    double brightness_change_threshold_;
    double whiteness_threshold_;
    double min_whiteness_for_flash_;
    double whiteness_change_threshold_;

    // 圆丢失容忍帧数（问题5）
    int lost_frames_max_;

    // 闪光 & 报警间隔
    int min_flash_interval_ms_;
    int min_alert_interval_ms_;

    // 移动跟踪
    int move_tracking_duration_ms_;
    int max_history_points_;
    int opening_change_min_, opening_change_max_;
    int closing_change_min_, closing_change_max_;

    // 状态判断额外阈值（用于圆消失条件）
    double circle_disappear_brightness_max_;
    double circle_disappear_whiteness_max_;

    // ---------- 内部状态 ----------
    cv::Rect region_;
    bool init_flag_ = false;

    enum FlashState { normal, flashing };
    FlashState previous_state_ = normal;
    FlashState current_state_ = normal;

    enum GateStatus { STABLE = 0, OPENING = 1, CLOSING = 2 };

    cv::Point last_circle_center_;
    float last_circle_radius_ = 30.0;
    bool last_circle_detected_ = false;
    bool circle_tracked_ = false;

    std::deque<double> brightness_history_;
    std::deque<double> whiteness_history_;

    std::chrono::steady_clock::time_point last_flash_time_;
    std::chrono::steady_clock::time_point last_alert_time_;
    std::deque<std::chrono::steady_clock::time_point> flash_times_;

    bool alert_ = false;
    bool is_alert_pub_ = false;
    double last_brightness_ = 0.0;
    double last_whiteness_ = 0.0;

    bool is_tracking_move_ = false;
    std::chrono::steady_clock::time_point move_start_time_;
    std::deque<cv::Point> circle_center_history_;

    int frame_count_ = 0;
    int lost_frames_ = 0;   // 圆连续丢失帧数

    // ---------- 私有函数 ----------
    void loadConfig(const std::string& config_path);

    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void ProcessFrame(const cv::Mat& frame);

    double CalculateCircularity(const std::vector<cv::Point>& contour);
    bool DetectCircularContour(const cv::Mat& frame, std::vector<cv::Point>& best_contour,
                               cv::Point& circle_center, float& circle_radius, double& circularity);
    double CalculateCircleBrightness(const cv::Mat& frame, const cv::Point& center, float radius);
    double CalculateCircleWhiteness(const cv::Mat& frame, const cv::Point& center, float radius);

    void AnalyzeWhitenessChange(double current_whiteness);
    bool IsWhiteLight(double current_whiteness);
    FlashState AnalyzeBrightnessChange(double current_brightness, double current_whiteness,
                                       bool circle_detected, bool circle_lost_too_long);
    bool DecideAlert(bool flash_detected);
    bool DetectFlashEvent(FlashState new_state);
    bool DartDetection(const cv::Mat& frame, cv::Point& detected_circle_center,
                       float& detected_circle_radius, double& detected_circularity,
                       double& circle_brightness, bool& circle_detected);

    void StartMoveTracking();
    void UpdateMoveTracking(const cv::Point& current_center);
    void AnalyzeMove();
    void PublishStatus(int gate_status);
};

#endif // DART_ALERTION_HPP