#include "minimap_drawer.hpp"
#include <iostream>

#define default_fps 30

#define minimap_length 700
#define minimap_width 375


MapDrawer::MapDrawer() : Node("map_drawer")
{
    std::string config_file;
    config_file =  "src/debug_toolbox/minimap/config/minimap_drawer.yaml";
    const auto yaml_config = YAML::LoadFile(config_file);
    
    bool compare = yaml_config["compare"].as<bool>();
    enemy_color_ = yaml_config["enemy_color"].as<std::string>();
    resource_path_ = yaml_config["resource_folder_path"].as<std::string>();
    std::string video_path = resource_path_ + "/video";
    std::string image_path = resource_path_ + "/images";

    robots_ = std::make_shared<radar_msgs::msg::MapRobotData>();

    // subscribe to the MapRobotData, both opponent and teammate
    map_robot_subscribe_ = this->create_subscription<radar_msgs::msg::MapRobotData>("map_robot_data", 10,
        [this] (const radar_msgs::msg::MapRobotData::SharedPtr msg) {
            std::cout << "time" << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
            timer_tool_.syn_start("map_robot_data callback");
            robots_ = msg;
            timer_tool_.syn_stop("map_robot_data callback");
            // this->background_frame_ = background_;
            // this->draw_robots();
        });
    if (compare) {
        cv::namedWindow("minimap", 0);
        // cv::resizeWindow("minimap", cv::Size(674, 1280.));
        cv::moveWindow("minimap", 1280, 0);
        std::string compare_video_path = video_path + "/minimap.mp4";
        cap_.open(compare_video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", compare_video_path.c_str());
            throw std::runtime_error("Failed to open video file.");
        }

        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / default_fps), 
            [this](){
                cap_ >> this->background_frame_;
                if (background_frame_.empty())
                    rclcpp::shutdown();
                this->draw_robots();
            });
    }
    else {
        std::string minimap_image_path = image_path + "/minimap_25.png";
        background_ = cv::imread(minimap_image_path);
        if (background_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open minimap image: %s", minimap_image_path.c_str());
            throw std::runtime_error("Failed to open minimap image.");
        }
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / default_fps), 
            [this](){
                // std::cout << "time" << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
                this->background_frame_ = background_;
                this->draw_robots();
            });
    }
}

MapDrawer::~MapDrawer()
{ 
    timer_tool_.print();
}

void MapDrawer::draw_robots()
{
    // resize background frame
    timer_tool_.syn_start("draw robots");
    cv::resize(background_frame_, background_frame_, cv::Size(minimap_length, minimap_width));
    
    if (enemy_color_ == "blue"){
        timer_tool_.syn_start("draw blue");
        draw_a_robot(0, robots_->opponent_hero_position_x, robots_->opponent_hero_position_y);
        draw_a_robot(1, robots_->opponent_engineer_position_x, robots_->opponent_engineer_position_y);
        draw_a_robot(2, robots_->opponent_infantry_3_position_x, robots_->opponent_infantry_3_position_y);
        draw_a_robot(3, robots_->opponent_infantry_4_position_x, robots_->opponent_infantry_4_position_y);
        draw_a_robot(4, robots_->opponent_aerial_position_x, robots_->opponent_aerial_position_y);
        draw_a_robot(5, robots_->opponent_sentry_position_x, robots_->opponent_sentry_position_y);
        timer_tool_.syn_stop("draw blue");

        timer_tool_.syn_start("draw red");
        draw_a_robot(6, robots_->ally_hero_position_x, robots_->ally_hero_position_y);
        draw_a_robot(7, robots_->ally_engineer_position_x, robots_->ally_engineer_position_y);
        draw_a_robot(8, robots_->ally_infantry_3_position_x, robots_->ally_infantry_3_position_y);
        draw_a_robot(9, robots_->ally_infantry_4_position_x, robots_->ally_infantry_4_position_y);
        draw_a_robot(10, robots_->ally_aerial_position_x, robots_->ally_aerial_position_y);
        draw_a_robot(11, robots_->ally_sentry_position_x, robots_->ally_sentry_position_y);
        timer_tool_.syn_stop("draw red");
    }

    if (enemy_color_ == "red"){
        timer_tool_.syn_start("draw red");
        draw_a_robot(0, robots_->opponent_hero_position_x, robots_->opponent_hero_position_y);
        draw_a_robot(1, robots_->opponent_engineer_position_x, robots_->opponent_engineer_position_y);
        draw_a_robot(2, robots_->opponent_infantry_3_position_x, robots_->opponent_infantry_3_position_y);
        draw_a_robot(3, robots_->opponent_infantry_4_position_x, robots_->opponent_infantry_4_position_y);
        draw_a_robot(4, robots_->opponent_aerial_position_x, robots_->opponent_aerial_position_y);
        draw_a_robot(5, robots_->opponent_sentry_position_x, robots_->opponent_sentry_position_y);
        timer_tool_.syn_stop("draw red");

        timer_tool_.syn_start("draw blue");
        draw_a_robot(6, robots_->ally_hero_position_x, robots_->ally_hero_position_y);
        draw_a_robot(7, robots_->ally_engineer_position_x, robots_->ally_engineer_position_y);
        draw_a_robot(8, robots_->ally_infantry_3_position_x, robots_->ally_infantry_3_position_y);
        draw_a_robot(9, robots_->ally_infantry_4_position_x, robots_->ally_infantry_4_position_y);
        draw_a_robot(10, robots_->ally_aerial_position_x, robots_->ally_aerial_position_y);
        draw_a_robot(11, robots_->ally_sentry_position_x, robots_->ally_sentry_position_y);
        timer_tool_.syn_stop("draw blue");
    }
    

    // cv::Mat rotated_frame;
    // cv::rotate(background_frame_, rotated_frame, cv::ROTATE_90_COUNTERCLOCKWISE);

    cv::imshow("minimap", background_frame_);
    cv::waitKey(1);
    timer_tool_.syn_stop("draw robots");
    
}

void MapDrawer::draw_a_robot(int id, int x, int y)
{
    const int map_length = 2800, map_width = 1500;

    // const int map_length = 1150, map_width = 650;
    // A car with x = 0 and y = 0 represent it isn't been discovered, and prevent over range.
    int x_on_map = 1.0 * x / map_length * minimap_length;
    int y_on_map = (1 - 1.0 * y / map_width)  * minimap_width;
    if (x_on_map <= 23 || x_on_map >= minimap_length || y_on_map <= 0 || y_on_map >= minimap_width - 23) {  
        return;
    }


    int radius = 23;
    cv::Scalar color = id < 6 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

    // 绘制描边
    cv::Scalar outline_color = cv::Scalar(255, 255, 255);
    int outline_thickness = 1;
    int line_thickness = 2;
    cv::Point center_point_on_minimap = cv::Point(x_on_map, y_on_map);
    for (int x = -outline_thickness; x <= outline_thickness; x += outline_thickness) {
        for (int y = -outline_thickness; y <= outline_thickness; y += outline_thickness) {
            if (x == 0 && y == 0) continue; // 跳过中心点（主体部分）
            cv::circle(background_frame_, cv::Point(x_on_map + x, y_on_map + y), radius, outline_color, line_thickness); 
        }
    }
    cv::circle(background_frame_, center_point_on_minimap, radius, color, line_thickness); 

    if (id % 6 == 5) {  // sentry draws a image
        int sign_radius = 15;
        for (int x = -outline_thickness; x <= outline_thickness; x += outline_thickness) {
            for (int y = -outline_thickness; y <= outline_thickness; y += outline_thickness) {
                if (x == 0 && y == 0) continue; // 跳过中心点（主体部分）
                draw_sentry(background_frame_, cv::Point(x_on_map + x, y_on_map + y), sign_radius, outline_color);
            }
        }
        draw_sentry(background_frame_, center_point_on_minimap, sign_radius, color);
        
    }
    else {  // others draw its id
        // 定义字体类型、大小、颜色和厚度
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 1.3;
        int thickness = 2;
        std::string text = std::to_string(id % 6 + 1);if (enemy_color_ == "blue"){
        timer_tool_.syn_start("draw blue");
        draw_a_robot(0, robots_->opponent_hero_position_x, robots_->opponent_hero_position_y);
        draw_a_robot(1, robots_->opponent_engineer_position_x, robots_->opponent_engineer_position_y);
        draw_a_robot(2, robots_->opponent_infantry_3_position_x, robots_->opponent_infantry_3_position_y);
        draw_a_robot(3, robots_->opponent_infantry_4_position_x, robots_->opponent_infantry_4_position_y);
        draw_a_robot(4, robots_->opponent_aerial_position_x, robots_->opponent_aerial_position_y);
        draw_a_robot(5, robots_->opponent_sentry_position_x, robots_->opponent_sentry_position_y);
        timer_tool_.syn_stop("draw blue");

        timer_tool_.syn_start("draw red");
        draw_a_robot(6, robots_->ally_hero_position_x, robots_->ally_hero_position_y);
        draw_a_robot(7, robots_->ally_engineer_position_x, robots_->ally_engineer_position_y);
        draw_a_robot(8, robots_->ally_infantry_3_position_x, robots_->ally_infantry_3_position_y);
        draw_a_robot(9, robots_->ally_infantry_4_position_x, robots_->ally_infantry_4_position_y);
        draw_a_robot(10, robots_->ally_aerial_position_x, robots_->ally_aerial_position_y);
        draw_a_robot(11, robots_->ally_sentry_position_x, robots_->ally_sentry_position_y);
        timer_tool_.syn_stop("draw red");
    }
        cv::Size text_size = cv::getTextSize(text, fontFace, fontScale, thickness, nullptr);
        cv::Point org(x_on_map - (text_size.width) / 2, y_on_map + (text_size.height) / 2);
        for (int x = -outline_thickness; x <= outline_thickness; x += outline_thickness) {
            for (int y = -outline_thickness; y <= outline_thickness; y += outline_thickness) {
                if (x == 0 && y == 0) continue; // 跳过中心点（主体部分）
                cv::putText(background_frame_, text, cv::Point(org.x + x, org.y + y), fontFace, fontScale, outline_color, thickness);
            }
        }
        cv::putText(background_frame_, text, org, fontFace, fontScale, color, thickness);
    }
}

void MapDrawer::draw_sentry(cv::Mat &image, cv::Point center, int radius, cv::Scalar color)
{
    int width = 90, height = 75;
    std::vector<cv::Point> upper_points, lower_points;
    upper_points.push_back(cv::Point(50, 0));
    upper_points.push_back(cv::Point(2, 10));
    upper_points.push_back(cv::Point(5, 35));
    upper_points.push_back(cv::Point(15, 40));
    upper_points.push_back(cv::Point(13, 53));
    upper_points.push_back(cv::Point(53, 53));
    upper_points.push_back(cv::Point(51, 36));
    upper_points.push_back(cv::Point(65, 30));
    upper_points.push_back(cv::Point(78, 30));
    upper_points.push_back(cv::Point(89, 27));
    upper_points.push_back(cv::Point(89, 16));
    upper_points.push_back(cv::Point(85, 14));
    upper_points.push_back(cv::Point(80, 17));
    upper_points.push_back(cv::Point(67, 17));

    lower_points.push_back(cv::Point(18, 57));
    lower_points.push_back(cv::Point(0, 74));
    lower_points.push_back(cv::Point(75, 74));
    lower_points.push_back(cv::Point(55, 57));

    for (auto &point : upper_points) {
        point = (point - 0.5 * cv::Point(width, height)) * radius * 2 / width + center;
    }
    for (auto &point : lower_points) {
        point = (point - 0.5 * cv::Point(width, height)) * radius * 2 / width + center;
    }

    cv::fillPoly(image, upper_points, color);   // -1: fill the inner part 
    cv::fillPoly(image, lower_points, color); 
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}