#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "camera_io/io/camera.hpp"
#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("img_shoot");
    std::string config_file;
    config_file = "src/calibration/configs/camera_shoot.yaml";
    const auto yaml_config = YAML::LoadFile(config_file);
    auto config_path = yaml_config["config_path"].as<std::string>();
    auto output_path = yaml_config["output_path"].as<std::string>();

    cv::namedWindow("img", 0);
    cv::resizeWindow("img", 1536, 720);

    tools::Exiter exiter;
    io::Camera camera(config_path);
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    auto last_stamp = std::chrono::steady_clock::now();

    int i = 0;
    while (!exiter.exit())
    {
        camera.read(img, timestamp);

        auto dt = tools::delta_time(timestamp, last_stamp);
        last_stamp = timestamp;

        tools::logger()->info("{:.2f} fps", 1 / dt);
        tools::logger()->info("img size: {}x{}", img.cols, img.rows);

        cv::imshow("img", img);
        int key = cv::waitKey(1);
        if (key == 'q')
            break;
        if (key == 's')
        {
            auto output_path_i = output_path + "/" + std::to_string(i) + ".jpg";
            cv::imwrite(output_path_i, img);
            tools::logger()->info("img saved to {}", output_path_i);
            i++;
        }
    }

    return 0;
}
