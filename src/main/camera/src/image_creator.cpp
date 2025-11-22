#include "image_creator.hpp"

ImageCreator::ImageCreator() : Node("image_creator_node")
{
    auto config = YAML::LoadFile("./src/main/camera/config/run_detect.yaml");
    input_ = config["input"].as<int>();
    RCLCPP_INFO(this->get_logger(), "ImageCreator node initialized");
    compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_compressed", 1); //maybe
    auto needed_file_ = std::make_unique<std::string>();    
    
    if (input_ == 0) {
        *needed_file_ = config["camera_config_file"].as<std::string>();
        camera_ = std::make_unique<io::Camera>(*needed_file_);
        camera_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(24), std::bind(&ImageCreator::cameraCallback, this)); 
    } 
    else if (input_ == 1) {
        *needed_file_ = config["vedio_path"].as<std::string>();
        cap_ = std::make_unique<cv::VideoCapture>(*needed_file_);
        video_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&ImageCreator::videoCallback, this));
    } 
    else if (input_ == 2) {
        *needed_file_ = config["image_path"].as<std::string>();
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Invalid input type specified in config");
    }

}
ImageCreator::~ImageCreator()
{
    RCLCPP_INFO(this->get_logger(), "ImageCreator node shutting down");
}

void ImageCreator::cameraCallback()
{
    static int frame_index = 0;
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
    camera_->read(frame, timestamp);
    frame_index++;
    std::cout << "Frame index: " << frame_index << std::endl;
    mat2image2bag(frame);
}
void ImageCreator::videoCallback()
{
    cv::Mat frame;
    if (cap_->isOpened() && rclcpp::ok()) {
        *cap_ >> frame;
        mat2image2bag(frame);
    } else {
        RCLCPP_ERROR(this->get_logger(), "VideoCapture not opened or rclcpp not ok");
    }
}
// void ImageCreator::ImageGetterVideo()
// {
    
// }
// void ImageCreator::ImageGetterCamera(){
//     static int frame_index = 0;
//     cv::Mat frame;
//     std::chrono::steady_clock::time_point timestamp;
//     camera_->.read(frame, timestamp);
//     frame_index++;
//     std::cout << "Frame index: " << frame_index << std::endl;
//     mat2image2bag(frame);
// }
void ImageCreator::mat2image2bag(cv::Mat frame)
{
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame, skipping publication");
        return;
    }

    try {
        std_msgs::msg::Header header; 
        header.stamp = this->now();
        header.frame_id = "camera_frame";
        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90}; // 90% JPEG quality
        cv::imencode(".jpg", frame, buf, params);
        auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        
        compressed_msg->header = header;
        compressed_msg->format = "jpeg";
        compressed_msg->data = buf; 

        auto bridge = cv_bridge::CvImage(header, "bgr8", frame);
        auto image_msg = bridge.toImageMsg();
        
        compressed_publisher_->publish(std::move(compressed_msg));
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageCreator>());
    rclcpp::shutdown();
    return 0;
}
