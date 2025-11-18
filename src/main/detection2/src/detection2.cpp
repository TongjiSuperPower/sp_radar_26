#include "detection2.hpp"

Detection2::Detection2() : Node("detection2_node")
{
    // Initialize member variables, not local variables
    detector_manager_ = std::make_shared<DetectorManager>(3);
    timer_ = std::make_shared<tools::Timer>();
    detector_manager_->set_timer(timer_);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_jpg", 10,
        std::bind(&Detection2::Detecter, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Detection2 node initialized");
}

void Detection2::Detecter(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        if (image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Converted image is empty");
            return;
        }
        cv::Mat gray_current, gray_old;

        if (oldimage.empty())
        {
            detector_manager_->detect_once(image, passtime * 1000, average_fps);
            oldimage = image.clone();
            timer_->syn_start("msdetect");
        } // size stays same so i used the
        else {
            detector_manager_->detect_once(image, passtime * 1000, average_fps);
            passtime = timer_->syn_stop("msdetect");
            timer_->syn_start("msdetect");
            total_duration += passtime;
            frame_count++;
            RCLCPP_INFO(this->get_logger(), "Detection time: %.4f ms", 1000 * passtime);
            }
            // average_fps = 1 / passtime;
            if (frame_count >= 60) {
                average_fps = frame_count / total_duration;
                RCLCPP_INFO(this->get_logger(), "Average FPS over last %d frames: %.2f", frame_count, average_fps);
                frame_count = 0;
                total_duration = 0.0f;
            }

            // frame_count = 0;
            // total_duration = 0.0f;
            cv::resize(image, image, cv::Size(960, 540));
            cv::imshow("detection2_result", image);
            cv::waitKey(1);
        }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Detection2>());
    rclcpp::shutdown();
    return 0;
}