#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/map_robot_data.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include "../tools/timer.hpp"

class MapDrawer : public rclcpp::Node
{
public:
    MapDrawer();
    ~MapDrawer();
private:
    void draw_robots();
    void draw_a_robot(int id, int x, int y);
    void draw_sentry(cv::Mat &image, cv::Point center, int radius, cv::Scalar color);

    cv::VideoCapture cap_;
    cv::Mat background_;
    cv::Mat background_frame_;

    radar_msgs::msg::MapRobotData::SharedPtr blue_;
    radar_msgs::msg::MapRobotData::SharedPtr red_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<radar_msgs::msg::MapRobotData>::SharedPtr map_robot_subscribe_;

    std::string resource_path_;
    std::string enemy_color_;

    tools::Timer timer_tool_;
};
