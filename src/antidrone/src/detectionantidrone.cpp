#include "detectionantidrone.hpp"


DetectionAntiDrone::DetectionAntiDrone() : Node("DetectionAntiDrone")
{
    // declare_parameter<std::string>("config_file", "");
    std::string config_file = "./src/antidrone/config/antidrone.yaml";
    // get_parameter("config_file", config_file);
    const auto yaml_config = YAML::LoadFile(config_file);
    std::string drone_engine_file = yaml_config["drone_engine_file"].as<std::string>();
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image_compressed", 1,
            std::bind(&DetectionAntiDrone::detect_once, this, std::placeholders::_1));
    trtyolo::InferOption option;
    option.enableSwapRB();
    model_ = std::make_shared<trtyolo::DetectModel>(drone_engine_file, option);

    std::cout << "DetectionAntiDrone::DetectionAntiDrone()" << std::endl;
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    carbbox_publisher_ = this->create_publisher<radar_msgs::msg::CarBbox>("car_bbox", 10);

}

void DetectionAntiDrone::draw_car_bbox(const radar_msgs::msg::CarBbox msg, cv::Mat& frame)
{
    for (auto bbox : msg.bboxs)
    {
        cv::Scalar color = (bbox.class_id < 6) ? cv::Scalar(255, 128, 0) : cv::Scalar(50, 50, 255);
        if (bbox.x_min > 0 || bbox.y_min > 20 || bbox.x_max < frame.cols - 50 || bbox.y_max < frame.rows)
        {
            cv::rectangle(frame, cv::Point(bbox.x_min, bbox.y_min), cv::Point(bbox.x_max, bbox.y_max), color, 10);
            cv::putText(frame, std::to_string((bbox.class_id) % 6 + 1), cv::Point(bbox.x_min + 40, bbox.y_min - 10), cv::FONT_HERSHEY_PLAIN, 6, color, 6);
            cv::circle(frame, cv::Point((bbox.x_min+bbox.x_max)/2, (bbox.y_max + bbox.y_min)/2), 2, color, 10);
        }
    }
}

cv::Rect DetectionAntiDrone::get_rect(cv::Mat &img, const trtyolo::Box& bbox)
{
    float left = bbox.left;
    float top = bbox.top;
    float right = bbox.right;
    float bottom = bbox.bottom;
    cv::Rect r = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));
    return r;
}

std::future<armor_result> DetectionAntiDrone::submit_car(cv::Mat &img) 
{
    std::promise<armor_result> promise;
    std::future<armor_result> future = promise.get_future();
    {
        std::unique_lock<std::mutex> lock(tasks_mutex_);
        tasks_.emplace(img, std::move(promise));
    }
    condition_.notify_one();
    return future;
}

void DetectionAntiDrone::set_timer(std::shared_ptr<tools::Timer> timer)
{
    timer_ = timer;
}


void DetectionAntiDrone::detect_once(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg)
{
    
    cv::Mat image = cv::imdecode(compressed_msg->data, cv::IMREAD_COLOR);
    cv::Mat cloned_image = image.clone();


    radar_msgs::msg::CarBbox car_bboxs;
    car_bboxs.header.stamp = clock_->now();
    car_bboxs.img_height = image.rows;
    car_bboxs.img_width = image.cols;

    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    trtyolo::Image input_image(
            image.data,     // Pixel data pointer
            image.cols,     // Image width
            image.rows     // Image height
        );

    trtyolo::DetectRes result = model_->predict(input_image);

    std::vector<cv::Rect> car_r;
    std::vector<std::future<armor_result>> futures;
    for (size_t j = 0; j < result.num; j++)
    {
        cv::Rect r = get_rect(image, result.boxes[j]);
        radar_msgs::msg::Bbox bbox;
        bbox.x_min = r.x;
        bbox.y_min = r.y;
        bbox.x_max = r.x + r.width;
        bbox.y_max = r.y + r.height;
        bbox.class_id = -1;
        bbox.class_confidence = -1;
        car_bboxs.bboxs.push_back(bbox);

        r &= cv::Rect(0, 0, image.cols, image.rows);
        cv::rectangle(image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::Mat region = image(r);
    }

    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    draw_car_bbox(car_bboxs, image);

    carbbox_publisher_->publish(car_bboxs);

    cv::resize(image, image, cv::Size(1500, 1000));
    cv::imshow("detection_result", image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionAntiDrone>());
    rclcpp::shutdown();
    return 0;
}