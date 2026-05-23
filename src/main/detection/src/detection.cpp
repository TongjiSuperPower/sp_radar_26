#include "detection.hpp"

//Information regarding Detection :
//Detection has been created in order to accomplish 3 tasks:
//1. Using different nodes for camera and detection
//2. Using compressed images
//3. Using filtered points with armor detection
//The filteredCallback1 and 2 are for detection only for points without any image or armor
//filteredCallback3 and bboxcreater are required for filtered detection, the other functions are not required.
//in the config file if you put in both 0 and 1 it will create two different videos at the same time. one using only images and one using images and points


Detection::Detection() : Node("detection_node")
{
    // Initialize member variables, not local variables
    // It's the detector_manager that cause the Ctrl-C problem
    detector_manager_ = std::make_shared<DetectorManager>(3, this->get_clock());
    timer_ = std::make_shared<tools::Timer>();
    std::string config_file = "./src/main/detection/config/run_detect.yaml";
    // get_parameter("config_file", config_file);
    const auto yaml_config = YAML::LoadFile(config_file);
    detector_manager_->set_timer(timer_);
    std::vector<int> input_for_method = yaml_config["input_for_method"].as<std::vector<int>>();
    bool i_want_to = false;

    bool has_0 = std::find(input_for_method.begin(), input_for_method.end(), 0) != input_for_method.end();
    bool has_1 = std::find(input_for_method.begin(), input_for_method.end(), 1) != input_for_method.end();

    if (has_0 && !has_1) {
        RCLCPP_INFO(this->get_logger(), "Starting Image-only detection (Method 0)");

        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image_compressed", qos_profile,
            std::bind(&Detection::Detecter, this, std::placeholders::_1));
    }

    // Method 1: LiDAR+Image fusion (if 1 is in list)
    if (has_1 && !has_0) {
        RCLCPP_INFO(this->get_logger(), "Starting LiDAR+Image fusion (Method 1)");
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        
        subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/filtered_lidar", 10,
            std::bind(&Detection::bboxcreater, this, std::placeholders::_1));
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image_compressed", qos_profile,
            std::bind(&Detection::filteredCallback3, this, std::placeholders::_1));
    }

    if (has_0 && has_1) {
        RCLCPP_INFO(this->get_logger(), 
            "Running BOTH pipelines: Image detection + LiDAR+Image fusion");
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/filtered_lidar", 10,
            std::bind(&Detection::bboxcreater, this, std::placeholders::_1));
        subscription3_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image_compressed", qos_profile,
            std::bind(&Detection::filteredCallback3, this, std::placeholders::_1));
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/image_compressed", qos_profile,
            std::bind(&Detection::Detecter, this, std::placeholders::_1));
    }
    // if (std::find(input_for_method.begin(), input_for_method.end(), 1) != input_for_method.end() && !i_want_to)
    // {
    //     subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //         "livox/filtered_lidar", 10,
    //         std::bind(&Detection::bboxcreater, this, std::placeholders::_1));
    //     subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    //         "camera/image_compressed", 10,
    //         std::bind(&Detection::filteredCallback3, this, std::placeholders::_1));
    // }  
    // else if (i_want_to) // i_want_to
    // {
    //     RCLCPP_INFO(this->get_logger(), "Detection node with only livox");
    //      subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //         "livox/filtered_lidar", 10,
    //         std::bind(&Detection::filteredCallback2, this, std::placeholders::_1));
    // }
    // else
    // {  
    //     subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    //         "camera/image_compressed", 10,
    //         std::bind(&Detection::Detecter, this, std::placeholders::_1));
    // }
    
    carbbox_publisher_ = this->create_publisher<radar_msgs::msg::CarBbox>("car_bbox", 10);
    
    RCLCPP_INFO(this->get_logger(), "Detection node initialized");
}

void Detection::filteredCallback3(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg)
{
    img_counter++;
    std::cout << "Image Count: " << img_counter << std::endl;
    static size_t call_count = 0;
    call_count++;
    RCLCPP_INFO(this->get_logger(), "Callback #%zu", call_count);

    rclcpp::Time img_time(compressed_msg->header.stamp);

    if(!first_image_received_) { 
        timer_->syn_start("General Timer: ");
        first_image_time_ = img_time;
        first_image_received_ = true;
        RCLCPP_INFO(this->get_logger(),
            "First Image: abs_time=%.3f, relative=0.000",
            img_time.seconds());
    }
    else{
        std::cout << "General Timer: " << timer_->syn_stop("General Timer: ") << std::endl;
        timer_->syn_start("General Timer: ");
    }

    double img_relative = (img_time - first_image_time_).seconds();

    std::vector<std::vector<float>> matched_bboxes;
    double best_time_diff = std::numeric_limits<double>::max();
    double matched_relative_time = 0.0;

    {
        std::lock_guard<std::mutex> lock(cache_mutex_);

        RCLCPP_INFO(this->get_logger(), 
            "Looking: Image@%.3fs (image timeline) | Cache: %zu entries",
            img_relative, bbox_cache_.size());
        

        if (!bbox_cache_.empty())
        {
            double time_diff = std::abs(img_relative - bbox_cache_[0].relative_time);
            
            RCLCPP_INFO(this->get_logger(),
                "  LiDAR: @%.3fs (lidar timeline) Δ=%.3fs (%.1fms)",
                bbox_cache_[0].relative_time, time_diff, time_diff * 1000.0);
            
            if (time_diff < best_time_diff && time_diff < 5.0) {  // 5 second tolerance for now
                best_time_diff = time_diff;
                matched_bboxes = bbox_cache_[0].bboxes;
                matched_relative_time = bbox_cache_[0].relative_time;
            }
        }
    }

    if (!matched_bboxes.empty()) {
        RCLCPP_INFO(this->get_logger(), 
            "MATCHED: Image@%.3fs ↔ LiDAR@%.3fs Δt=%.1fms, %zu bboxes",
            img_relative,
            matched_relative_time,
            best_time_diff * 1000.0,
            matched_bboxes.size());
        
        detector_manager_->filteredDetect3(compressed_msg, matched_bboxes);

    } else {
        RCLCPP_WARN(this->get_logger(), 
            "NO MATCH for Image@%.3fs. Best Δt=%.1fms > 5s tolerance",
            img_relative, best_time_diff * 1000.0);
    }
    
}

std::vector<std::vector<float>> Detection::bboxcreater(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto bbox_array = detector_manager_->filteredDetect3Helper(msg);   
    std::cout << "bbox size:" << bbox_array.size() << std::endl;
    rclcpp::Time lidar_time(msg->header.stamp);
    lidar_counter++;
    std::cout << "Lidar Counter: " << lidar_counter << std::endl;
    if (!first_lidar_received_) {
        //timer_->syn_start("BboxCreater");
        first_lidar_time_ = lidar_time;
        first_lidar_received_ = true;
        RCLCPP_INFO(this->get_logger(), 
            "First LiDAR: abs_time=%.3f, relative=0.000",
            lidar_time.seconds());
    }
    else{
        //std::cout << "Bbox Timer: " << timer_->syn_stop("BboxCreater") << std::endl;
        //timer_->syn_start("BboxCreater");
    }
    double lidar_relative = (lidar_time - first_lidar_time_).seconds();
    
    TimedBBoxArray entry;
    entry.bboxes = bbox_array;
    entry.timestamp = lidar_time;
    entry.relative_time = lidar_relative;  // LiDAR's own timeline
    
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        bbox_cache_.push_back(entry);
        
        if (bbox_cache_.size() > max_cache_size_) {
            bbox_cache_.pop_front();
        }
        
        // Log REAL values
        RCLCPP_INFO(this->get_logger(),
            "Cached LiDAR: abs=%.3f, rel=%.3f (LIDAR timeline), bboxes=%zu",
            lidar_time.seconds(), lidar_relative, bbox_array.size());
    }
    return bbox_array;
}

void Detection::Detecter(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg)
{
    try
    {
        cv::Mat image = cv::imdecode(compressed_msg->data, cv::IMREAD_COLOR); // try without this first
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //cv::Mat image = cv_ptr->image;
        int last_time = (int)(compressed_msg->header.stamp.sec * 1000 + compressed_msg->header.stamp.nanosec / 1e6);
        int this_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "Delay: %d ms", this_time - last_time);
        if (this_time - last_time > 100 && this_time - last_time < 1000) RCLCPP_WARN(this->get_logger(), "WARNING: High Delay!!!");
        if (image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Converted image is empty");
            return;
        }
        cv::Mat gray_current, gray_old;

        if (oldimage.empty())
        {
            carbbox_publisher_->publish(detector_manager_->detect_once(image, passtime * 1000, average_fps));
            oldimage = image.clone();
            timer_->syn_start("msdetect");
        } // size stays same so i used the  
        else {
            carbbox_publisher_->publish(detector_manager_->detect_once(image, passtime * 1000, average_fps));
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
        cv::imshow("detection_result", image);
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
    rclcpp::spin(std::make_shared<Detection>());
    rclcpp::shutdown();
    return 0;
}