#include "detector_manager.hpp"
#include "camera_io/tools/recorder.hpp"
#include "camera_io/tools/timer.hpp"
#include "camera_io/io/camera.hpp"
#include <opencv2/opencv.hpp>
#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"
#include "camera_io/tools/buffer.hpp"
#include "camera_io/tools/timestamp_recorder.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto timer = std::make_shared<tools::Timer>();
    auto detector_manager = std::make_shared<DetectorManager>(3);

    std::string config_file;
    detector_manager->get_parameter("config_file", config_file);
    const auto yaml_config = YAML::LoadFile(config_file);
    std::string record_folder_path = yaml_config["record_folder_path"].as<std::string>();
    std::string output_folder_path = yaml_config["output_folder_path"].as<std::string>();
    bool save_flag = yaml_config["save"].as<bool>();
    int input = yaml_config["input"].as<int>();
    std::string vedio_path = yaml_config["vedio_path"].as<std::string>();
    std::string camera_config_file = yaml_config["camera_config_file"].as<std::string>();
    std::string image_path = yaml_config["image_path"].as<std::string>();
    detector_manager->set_timer(timer);

    bool if_show_img = true;
    int key = 0;

    if (input == 0) // camera input
    {
        const int fps = 30;
        io::Camera camera(camera_config_file);
        tools::Recorder recorder(fps, record_folder_path);
        // tools::TimestampRecorder timestamp_recorder(record_folder_path);
        cv::Mat frame;
        
        int frame_index = 0;
        timer->syn_start("main loop");
        while (rclcpp::ok()) {
            std::chrono::steady_clock::time_point timestamp;
            camera.read(frame, timestamp);
            if (save_flag) {
                recorder.record(frame, timestamp);
                // timestamp_recorder.record_timestamp();
            }
            detector_manager->detect_once(frame);
            if(if_show_img)
            {
                cv::resize(frame, frame, cv::Size(1080, 720));
                cv::imshow("detect", frame);
                key = cv::waitKey(1);
                if(key == 'q' || key == 'Q') {
                    if_show_img = false;
                    // cv::destroyWindow("detect");
                    cv::destroyAllWindows();
                }
            }
            frame_index++;

            std::cout << "now frame " << frame_index << std::endl;
        }
        std::cout << "total frame count: " << frame_index << std::endl;
        timer->syn_stop("main loop");
    }
    else if (input == 1)    // video input
    {
        auto cap = cv::VideoCapture(vedio_path);
        int width = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int height = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        int fps = int(cap.get(cv::CAP_PROP_FPS));

        tools::Recorder recorder(60, output_folder_path);
        // tools::TimestampRecorder timestamp_recorder(output_folder_path);

        cv::Mat frame;
        int frame_index{0};


        int total_frame = cap.get(cv::CAP_PROP_FRAME_COUNT), frame_count = 0;
        while (cap.isOpened() && rclcpp::ok())
        {
            timer->asyn_start("main loop");
            try {
                cap >> frame;
            }
            catch (const cv::Exception& e) {
                std::cerr << "读取帧错误" << std::endl;
                continue;
            }
            if (frame.empty())
            {
                std::cout << "\n文件处理完毕" << std::endl;
                break;
            }
            frame_index++;
            detector_manager->detect_once(frame);

            std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::time_point::clock::now();
            cv::Mat frame_copy = frame.clone(); // 保证安全
            if (save_flag) {
                recorder.record(frame_copy, current_time);
                // timestamp_recorder.record_timestamp();
            }
            cv::resize(frame_copy, frame_copy, cv::Size(1080, 720));
            if(if_show_img)
            {
                cv::imshow("detect", frame_copy);
                key = cv::waitKey(1);
                if(key == 'q' || key == 'Q') {
                    if_show_img = false;
                    // cv::destroyWindow("detect");
                    cv::destroyAllWindows();
                }
            }
            
            std::cout << "处理完第" << frame_index << "帧" << std::setw(3) << frame_count * 100 / total_frame << "%" << std::endl;
            frame_count++;
            timer->asyn_stop("main loop");
        }
    }
    else if (input == 2)    // image input
    {
        cv::Mat image = cv::imread(image_path);
        detector_manager->detect_once(image);
        cv::imshow("frame", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    else
    {
        RCLCPP_ERROR(detector_manager->get_logger(), "please choose input method in 0, 1, 2");
    }

    rclcpp::shutdown();
    timer->print();
    return 0;
}
