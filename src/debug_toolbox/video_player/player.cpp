#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <radar_msgs/msg/car_bbox.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

class VideoPlayer : public rclcpp::Node
{ 
public:
    VideoPlayer() : Node("video_player")
    {
        auto config = YAML::LoadFile("./src/debug_toolbox/video_player/player.yaml");
        subscription_ = this->create_subscription<radar_msgs::msg::CarBbox>(
            "car_bbox", 10, std::bind(&VideoPlayer::car_bbox_callback, this, std::placeholders::_1));
        video_capture_ = std::make_shared<cv::VideoCapture>(config["video_path"].as<std::string>());
        file_ = std::make_shared<std::ifstream>(config["txt_path"].as<std::string>());
        if (!file_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "File not found");
            return;
        }
        if (!video_capture_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Video not found");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Video player node started");

        last_frame_timestamp_ = 0;
    }

private:
    void car_bbox_callback(const radar_msgs::msg::CarBbox::SharedPtr msg) 
    {
        auto milliseconds = (long long)msg->header.stamp.sec * 1000 + msg->header.stamp.nanosec / 1000000;
        std::cout << "ros bag"<< milliseconds << std::endl;
        while (milliseconds > last_frame_timestamp_) {
            std::string timestamp_line, timestamp_str;
            std::getline(*file_, timestamp_line);
            cv::Mat frame;
            // if (video_capture_)
            *video_capture_ >> frame;
            cv::imshow("Synchronous Frame", frame);
            cv::waitKey(1);
            int start = 0;
            for (int i = 0; i < timestamp_line.size(); i++) {
                if (start == 1) {
                    timestamp_str += timestamp_line[i];
                }
                if (timestamp_line[i] == ' ') {
                    start = 1;
                }
            }
            last_frame_timestamp_ = std::stoll(timestamp_str);
            std::cout << "video timestamp" << last_frame_timestamp_ << std::endl;
        }
    }

    rclcpp::Subscription<radar_msgs::msg::CarBbox>::SharedPtr subscription_;
    std::shared_ptr<cv::VideoCapture> video_capture_;
    std::shared_ptr<std::ifstream> file_;

    long long last_frame_timestamp_; 

};  

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPlayer>());
  rclcpp::shutdown();
  return 0;
}