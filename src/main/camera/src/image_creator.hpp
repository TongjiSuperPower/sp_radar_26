#ifndef SP_IMAGE_CREATOR_HPP
#define SP_IMAGE_CREATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include "../camera_io/tools/exiter.hpp"
#include "../camera_io/tools/logger.hpp"
#include "../camera_io/tools/math_tools.hpp"
#include "../camera_io/tools/recorder.hpp"
#include "../camera_io/io/camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "../camera_io/tools/buffer.hpp"
#include "../camera_io/tools/timestamp_recorder.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <sensor_msgs/msg/compressed_image.hpp>


class ImageCreator : public rclcpp::Node
{
public:
    ImageCreator();
    ~ImageCreator();
private:
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::TimerBase::SharedPtr video_timer_;
    int input_; 

    std::string* file_path;
    std::string camera_config_file_;
    std::string vedio_path_;
    std::string image_path_;

    std::unique_ptr<io::Camera> camera_;  
    std::unique_ptr<cv::VideoCapture> cap_;  
   
   
    // void ImageGetter(int type, std::string path, std::string camera_config_file);
    void cameraCallback();
    void videoCallback();
    void mat2image2bag(cv::Mat frame);


};

#endif
