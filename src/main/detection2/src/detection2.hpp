#ifndef DETECTION2_HPP
#define DETECTION2_HPP

#include "detector_manager.hpp"
#include "tools/timer.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


class Detection2 : public rclcpp::Node
{
public:
    Detection2();
    ~Detection2() = default;  

private:    
    void Detecter(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    float average_fps;
    float passtime;
    std::shared_ptr<DetectorManager> detector_manager_;  
    std::shared_ptr<tools::Timer> timer_;  
    cv::Mat oldimage;
    cv_bridge::CvImagePtr cv_ptr;   
    int frame_count = 0;
    float total_duration;

};

#endif