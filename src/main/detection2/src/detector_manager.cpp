#include "detector_manager.hpp"

DetectorManager::DetectorManager(int armor_detector_num) : stop_(false), Node("DetectorManager")
{
    // declare_parameter<std::string>("config_file", "");
    std::string config_file = "./src/main/detection2/config/run_detect.yaml";
    // get_parameter("config_file", config_file);
    const auto yaml_config = YAML::LoadFile(config_file);
    std::string car_engine_file = yaml_config["car_engine_file"].as<std::string>();
    std::string armor_engine_file = yaml_config["armor_engine_file"].as<std::string>();
    detectors_.push_back(std::make_shared<deploy::DeployDet>(car_engine_file));
    for (int i = 0; i < armor_detector_num; i++)
    {
        detectors_.push_back(std::make_shared<deploy::DeployDet>(armor_engine_file));
    }
    num_threads_ = detectors_.size() - 1;
    for (size_t i = 0; i < num_threads_; ++i)
    {
        threads_.emplace_back(&DetectorManager::set_thread, this, i);
    }
    carbbox_publisher_ = this->create_publisher<radar_msgs::msg::CarBbox>("car_bbox", 10);

    tracker_manager_ = std::make_shared<TrackerManager>(config_file);

    std::cout << "DetectorManager::DetectorManager()" << std::endl;
}

DetectorManager::~DetectorManager()
{
    stop_.store(true);
    condition_.notify_all();
    for (std::thread &worker : threads_)
    {
        worker.join();
    }
    std::cout << "DetectorManager::~DetectorManager()" << std::endl;
}

void DetectorManager::set_timer(std::shared_ptr<tools::Timer> timer)
{
    timer_ = timer;
}

std::future<armor_result> DetectorManager::submit_car(cv::Mat &img)
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
cv::Rect DetectorManager::get_rect(cv::Mat &img, deploy::Box &bbox)
{
    float left = bbox.left;
    float top = bbox.top;
    float right = bbox.right;
    float bottom = bbox.bottom;
    cv::Rect r = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));
    return r;
}
armor_result DetectorManager::process_armor(cv::Mat &img, size_t id)
{
    armor_result armor_result;
    cv::Mat cvim = img.clone();
    // cv::cvtColor(cvim, cvim, cv::COLOR_BGR2RGB);
    deploy::Image im(cvim.data, cvim.cols, cvim.rows);
    deploy::DetResult result = detectors_[id + 1]->predict(im);
    int class_id = -1;
    float class_score = 0;
    for (size_t j = 0; j < result.boxes.size(); j++)
    {
        if (result.scores[j] > class_score)
        {
            class_id = result.classes[j];
            class_score = result.scores[j];
        }
        cv::Rect r_2 = get_rect(img, result.boxes[j]);
        r_2 &= cv::Rect(0, 0, img.cols, img.rows);
        cv::rectangle(img, r_2, cv::Scalar(0x27, 0xC1, 0x36), 2);
        cv::putText(img, std::to_string((int)result.classes[j]), cv::Point(r_2.x, r_2.y - 10), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
    }
    armor_result.detcted_img = img;
    armor_result.class_id = class_id;
    armor_result.class_score = class_score;
    return armor_result;
}

void DetectorManager::set_thread(size_t id)
{
    while (true)
    {
        std::pair<cv::Mat, std::promise<armor_result>> task;
        {
            std::unique_lock<std::mutex> lock(tasks_mutex_);
            condition_.wait(lock, [this]
                            { return stop_.load() || !tasks_.empty(); });
            if (stop_.load() && tasks_.empty())
            {
                return;
            }
            task = std::move(tasks_.front());
            tasks_.pop();
        }

        armor_result result = process_armor(task.first, id);
        task.second.set_value(result);
    }
}
void DetectorManager::detect_once(cv::Mat &image, float elapsed, float display_fps)
{
    cv::Mat cloned_image = image.clone();
    radar_msgs::msg::CarBbox car_bboxs;
    car_bboxs.header.stamp = this->now();
    car_bboxs.img_height = image.rows;
    car_bboxs.img_width = image.cols;

    timer_->syn_start("detect");
    timer_->syn_start("detect_1");
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    deploy::Image im(image.data, image.cols, image.rows);
    deploy::DetResult result = detectors_[0]->predict(im);

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
        car_r.push_back(r);
        auto future = submit_car(region);
        futures.push_back(std::move(future));
        cv::putText(image, std::to_string((int)result.classes[j]), cv::Point(r.x, r.y - 10), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << result.scores[j];
        std::string conf_str = oss.str();
        cv::putText(image, conf_str, cv::Point(r.x + 40, r.y - 10), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
    }
    timer_->syn_stop("detect_1");
    timer_->syn_start("detect_2");
    int num = 0;
    for (auto &future : futures)
    {
        armor_result result1 = future.get();
        result1.detcted_img.copyTo(image(car_r[num]));
        car_bboxs.bboxs[num].class_id = result1.class_id;
        car_bboxs.bboxs[num].class_confidence = result1.class_score;
        // conf of armors
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << result1.class_score;
        std::string conf_str = oss.str();
        cv::putText(image, conf_str, cv::Point(car_r[num].x + 40, car_r[num].y + 20), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
        // cv::putText(image, conf_str, cv::Point(100, 100), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
        num++;
    }
    timer_->syn_stop("detect_2");

    // stable fps
    // float elapsed = 1000 * timer_->syn_stop("detect"); //timer gives in seconds
    auto time_str = std::to_string(elapsed) + "ms";
    // static int count = 0;
    // static float display_fps = 0.0f;    
    // static float duration_60 = 0.0f;    // duration of 60 frames
    // duration_60 += elapsed;
    // count = (count + 1) % 60;
    // if (count == 0) {
    //     display_fps = 1000 * 60 / duration_60;
    //     duration_60 = 0.0f; 
    // }
    // auto fps = 1000.0f / elapsed;
    auto fps_str = std::to_string(display_fps) + "fps";
    cv::putText(image, time_str, cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    cv::putText(image, fps_str, cv::Point(50, 100), cv::FONT_HERSHEY_DUPLEX, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    
    // tracker
    car_bboxs = tracker_manager_->callback(cloned_image, car_bboxs);
    // tracker_manager_->record(car_bboxs, cloned_image);
    // end of tracker
    carbbox_publisher_->publish(car_bboxs);
}
