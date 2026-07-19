#include "detector_manager.hpp"

//1. i read the codes for the radar 
//2. i divided the detection node into camera node and detection 2
//3. i devised the usage of pointclouds in regard of finding the robots and drawing bboxes


DetectorManager::DetectorManager(int armor_detector_num, rclcpp::Clock::SharedPtr clock) 
        : clock_(clock ? clock : std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
        stop_(false)
{
    // declare_parameter<std::string>("config_file", "");
    camera_time_ = 0;
    std::string config_file = "./src/main/detection/config/run_detect.yaml";
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
    std::vector<int> input_for_method = yaml_config["input_for_method"].as<std::vector<int>>();
    if (std::find(input_for_method.begin(), input_for_method.end(), 1) != input_for_method.end())
    {
        std::vector<double> camera_matrix = yaml_config["camera_matrix"].as<std::vector<double>>();
        camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix.data()).clone();
        std::vector<double> distort_coeffs = yaml_config["distort_coeffs"].as<std::vector<double>>();
        distort_coeffs_ = cv::Mat(distort_coeffs.size(), 1, CV_64F, distort_coeffs.data()).clone();  //these matrixes are from locate.cpp(locate.yaml spesifically)
        std::vector<double> TUM_C2L = yaml_config["TUM_camera2lidar"].as<std::vector<double>>(); 
        transform_C2L_ = tum_to_transform_stamped(TUM_C2L);
        transform_L2C_ = inverse_transform(transform_C2L_); 
        // between 23-30 is taken from locate.cpp
        // You don't need to use the transform from lidar to map. It's the function of pointcloud_locate.
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // image_sub_.subscribe(this, "camera/image_compressed");
        // lidar_sub_.subscribe(this, "livox/filtered_lidar");
        // sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        //     SyncPolicy(10), image_sub_, lidar_sub_));
        // sync_->registerCallback(&DetectorManager::filteredDetect, this); 
    }  


    // carbbox_publisher_ = this->create_publisher<radar_msgs::msg::CarBbox>("car_bbox", 10);
//i take this and put it into deterction2
//and i also take the 
    tracker_manager_ = std::make_shared<TrackerManager>();

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

std::future<armor_result> DetectorManager::submit_car(cv::Mat &img) //ffiltere
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

void DetectorManager::filteredDetect3(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& compressed_msg, 
    std::vector<std::vector<float>>& goodbboxes) 
{
    timer_->syn_start("Filtered Detect");
    camera_time_ = (1.0 * compressed_msg->header.stamp.sec * 1000 + 
                   compressed_msg->header.stamp.nanosec / 1e6);
    
    cv::Mat cv_image = cv::imdecode(cv::Mat(compressed_msg->data), cv::IMREAD_COLOR);

    if (cv_image.empty()) {
        return;
    }
    
    auto car_bboxs = detect_armors_on_bboxes(cv_image, goodbboxes);
    
    if (carbbox_publisher_) {
        carbbox_publisher_->publish(car_bboxs);
    }


    
    static bool window_open = true;
    if (window_open) {
        cv::Mat display_img;
        cv::resize(cv_image, display_img, cv::Size(960, 540));

        cv::putText(display_img, "Bboxes: " + std::to_string(goodbboxes.size()), cv::Point(30, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 2);
        
        cv::imshow("LiDAR + Armor Detection", display_img);
        
        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            window_open = false;
            cv::destroyWindow("LiDAR + Armor Detection");
        }
    }
    std::cout << "Filtered Detect Time: " << timer_->syn_stop("Filtered Detect") << std::endl;
}


std::vector<std::vector<float>> DetectorManager::filteredDetect3Helper(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    cv_image_ = cv::Mat::zeros(3072,2048,CV_8UC3); //2048 3072 can be tried as well
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud; // points in camera
    transform_point_cloud(msg, transform_L2C_, transformed_cloud);
    return pointclouds_to_image(transformed_cloud, cv_image_);
}

std::vector<std::vector<float>> DetectorManager::pointclouds_to_image(const pcl::PointCloud<pcl::PointXYZ> &cloud, cv::Mat& img)
{  
    std::vector<std::vector<float>> local_bbox_list;
    float litbox = 50;
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::pair<pcl::PointXYZ, int>> car_points;
    std::vector<cv::Point3f> obj_points; // 相机坐标系 cameara coordinate
    std::vector<double> obj_points_features;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    *cloud_ptr = cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_across_frame(new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
//after this to the next comment is usage of Kdtree and trying it

    points_list_.push_back(cloud_ptr);
    if (points_list_.size() > accumulate_frame)
        points_list_.pop_front();
    for (auto& points: points_list_) {
        *cloud_across_frame += *points;
    }

    cloud_projected = project(cloud_across_frame);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_projected);
    auto time = std::chrono::system_clock::now();
    local_bbox_list.clear();
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.25);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_projected);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    for (const auto& indices : cluster_indices)
    {
        std::vector<cv::Point3f> cluster_obj_points;
        for(const auto& idx : indices.indices)
        {
            const auto& point = cloud_across_frame->points[idx];
            cv::Point3f obj_point(point.x, point.y, point.z);
            cluster_obj_points.push_back(obj_point);
        }
        std::vector<cv::Point2f> cluster_reprojected_points;

        if (!cluster_obj_points.empty()) 
        {
            cv::projectPoints(cluster_obj_points, rvec, tvec, camera_matrix_, distort_coeffs_, cluster_reprojected_points);
        }
        float min_x = cluster_reprojected_points[0].x, max_x = cluster_reprojected_points[0].x;
        float min_y = cluster_reprojected_points[0].y, max_y = cluster_reprojected_points[0].y;

        for (const auto& point : cluster_reprojected_points) 
        {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        float mid_x = (max_x + min_x)/2; 
        float mid_y = (max_y + min_y)/2;
        float dist_x = (max_x - mid_x);
        float dist_y = (max_y - mid_y);

        if (dist_y < 150 || dist_x < 150)
        {
            max_y = mid_y + dist_y*2;
            min_y = mid_y - dist_y*2;
            max_x = mid_x + dist_x*1.5;
            min_x = mid_x - dist_x*1.5;
        }

        std::vector<float> single_bbox =  {min_x, min_y, max_x, max_y};
        std::cout << (max_y - mid_y) << std::endl;
        bool little_box = false;
        if (local_bbox_list.empty() && !little_box){
            local_bbox_list.push_back(single_bbox);
        }
        if (!local_bbox_list.empty()){
            if (!little_box){
                local_bbox_list.push_back(single_bbox);
            }
        }
    }
    for (const auto& bbox1 : local_bbox_list) 
    {
        cv::Point2f top_left(bbox1[0], bbox1[1]); 
        cv::Point2f bottom_right(bbox1[2], bbox1[3]);
        cv::rectangle(img, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
    }
    return local_bbox_list;
//hopefully until here works!

//     for (auto &point : cloud.points)
//     {
//         cv::Point3f obj_point(point.x, point.y, point.z);
//         obj_points.push_back(obj_point);
//         if (obj_point.x > max_distance_)
//         {
//             max_distance_ = obj_point.x;
//         }
//         obj_points_features.push_back(obj_point.x); // 使用x作为特征点
//     }
    
//     std::vector<cv::Point2f> reprojected_points; // 投影至像素平面的点

//     if (obj_points.size() > 0)
//     {
//         cv::projectPoints(obj_points, rvec, tvec, camera_matrix_, distort_coeffs_, reprojected_points);
//         //double delta_time = abs(lidar_time_ - camera_time_);
//         double dist;
//         local_bbox_list.clear();
//         while (reprojected_points.size() > 0)  
//         {
//             points_list.clear();
//             i_list.clear();
//             points_list.push_back(reprojected_points[0]);
//             i_list.push_back(0);
//             cv::Point3f center = obj_points[0];
//             for (int i = 1; i < reprojected_points.size(); i++)
//             {
//                 std::cout << "point (" << reprojected_points[i].x << ", " << reprojected_points[i].y << ")" << std::endl;
//                 if (reprojected_points[i].x < 0 || reprojected_points[i].x >= 3072 || 
//                     reprojected_points[i].y < 0 || reprojected_points[i].y >= 2048) {
//                     std::cout << "has some points outside of view" << std::endl;
//                     continue;
//                 }
//                 //std::cout << "in for loop in pointcloud_to_image" << std::endl;
//                 float distx = abs(obj_points[i].x - center.x);
//                 float disty = abs(obj_points[i].y - center.y);
//                 float distz = abs(obj_points[i].z - center.z);

//                 dist = sqrt(distx*distx + disty*disty + distz*distz);

//                 if (dist < litbox)
//                 {
//                     points_list.push_back(reprojected_points[i]); 
//                     center = (obj_points[i]+center) * 0.5f; //switched between reproj to obj
//                     i_list.push_back(i);
//                 }
                
//             }

//             if (points_list.size() > 10) {
//                 float min_x = points_list[0].x, max_x = points_list[0].x;
//                 float min_y = points_list[0].y, max_y = points_list[0].y;
//                 for (const auto& point : points_list) {
//                     min_x = std::min(min_x, point.x);
//                     max_x = std::max(max_x, point.x);
//                     min_y = std::min(min_y, point.y);
//                     max_y = std::max(max_y, point.y);
//                 }
//                 std::vector<float> single_bbox =  {min_x, min_y, max_x, max_y};
// //                std::cout << "heyo" << min_x << "hey1" << max_x << "hey2" << min_y << max_y << std::endl;
//                 bool little_box = (single_bbox[2]-single_bbox[0] < 0.25 || single_bbox[3]-single_bbox[1] < 0.25);
//                 if (local_bbox_list.empty() && !little_box)
//                 {
//                     local_bbox_list.push_back(single_bbox);
//                     outside = true;
// //                    std::cout << "we are not little[1]" << std::endl;
//                 }
//                 if (!local_bbox_list.empty())
//                 {
//                     // for (auto& bbox:local_bbox_list)
//                     // {
//                     //     outside = true;
//                     //     outside = (single_bbox[3] < bbox[1] || single_bbox[1] > bbox[3]|| single_bbox[2] < bbox[0]|| single_bbox[0] > bbox[2]); // i need to implmenet this outside of while loop bruhhhh
//                     //     if (!outside)
//                     //     {
//                     //         bbox[0] = std::min(single_bbox[0],bbox[0]);
//                     //         bbox[1] = std::min(single_bbox[1],bbox[1]);
//                     //         bbox[2] = std::max(single_bbox[2],bbox[2]);
//                     //         bbox[3] = std::max(single_bbox[3],bbox[3]);
//                     //         break;
//                     //     }
//                     // }
//                     if (outside && !little_box) 
//                     {
// //                    std::cout << "we are not little[1]" << std::endl;
//                     local_bbox_list.push_back(single_bbox);
//                     }
//                 }
//                 for (int j = i_list.size() - 1; j >= 0; j--) 
//                 {
//                     if (i_list[j] < reprojected_points.size()) 
//                     {
//                         reprojected_points.erase(reprojected_points.begin() + i_list[j]);
//                     }
//                 } 
//             } 
//         for (const auto& bbox1 : local_bbox_list)
//         {
//             // bool little_box = (bbox1[2]-bbox1[0] < 0.1 || bbox1[3]-bbox1[1] < 0.1);
//             if (true)
//             {
//                 cv::Point2f top_left(bbox1[0], bbox1[1]);
//                 cv::Point2f bottom_right(bbox1[2], bbox1[3]);
//                 cv::rectangle(img, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
//             }
        
//         }
//     }
//         return local_bbox_list;
//     }
//     return std::vector<std::vector<float>>();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DetectorManager::project(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz) {
    
    // 创建输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 设置点云属性
    cloud_projected->width = cloud_xyz->width;
    cloud_projected->height = cloud_xyz->height;
    cloud_projected->is_dense = cloud_xyz->is_dense;
    cloud_projected->points.resize(cloud_xyz->size());
    
    // 转换每个点
    for (size_t i = 0; i < cloud_xyz->size(); ++i) {
        cloud_projected->points[i].x = cloud_xyz->points[i].x;
        cloud_projected->points[i].y = cloud_xyz->points[i].y;
        cloud_projected->points[i].z = 0.0;
    }
    
    return cloud_projected;
}

void DetectorManager::transform_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
    const geometry_msgs::msg::TransformStamped &transform,
    pcl::PointCloud<pcl::PointXYZ> &transformed_cloud)
{   
    pcl::PointCloud<pcl::PointXYZ> cloud; 
    pcl::fromROSMsg(*msg, cloud);     
    transformed_cloud.header = cloud.header;

    for (auto &point : cloud.points)
    {
        geometry_msgs::msg::PointStamped point_in_lidar;
        point_in_lidar.header.frame_id = "lidar_frame";
        point_in_lidar.point.x = point.x;
        point_in_lidar.point.y = point.y;
        point_in_lidar.point.z = point.z;        
        geometry_msgs::msg::PointStamped point_in_camera;
        try
        {
            tf2::doTransform(point_in_lidar, point_in_camera, transform);
            // 转换后的点坐标
            pcl::PointXYZ transformed_point(point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z);
            transformed_cloud.points.push_back(transformed_point);
        }

        catch (tf2::TransformException &ex)
        {
            //RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
    //RCLCPP_INFO(this->get_logger(), "Transformed %zu points", transformed_cloud.points.size());
}

geometry_msgs::msg::TransformStamped DetectorManager::tum_to_transform_stamped(std::vector<double> TUM)
{
    geometry_msgs::msg::TransformStamped transform;
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();


    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);

    // 设置时间戳
    transform.header.stamp.sec = seconds.count();
    transform.header.stamp.nanosec = nanoseconds.count();

    // 设置坐标系ID，从雷达坐标系到相机坐标系
    transform.header.frame_id = "camera_frame";
    transform.child_frame_id = "lidar_frame";

    // 设置平移
    transform.transform.translation.x = TUM[0];
    transform.transform.translation.y = TUM[1];
    transform.transform.translation.z = TUM[2];

    // 设置旋转（四元数）
    transform.transform.rotation.x = TUM[3];
    transform.transform.rotation.y = TUM[4];
    transform.transform.rotation.z = TUM[5];
    transform.transform.rotation.w = TUM[6];

    return transform;
}

geometry_msgs::msg::TransformStamped DetectorManager::inverse_transform(
    const geometry_msgs::msg::TransformStamped &transform)
{
    // 创建逆变换对象
    geometry_msgs::msg::TransformStamped inverse;

    // 交换源坐标系和目标坐标系
    inverse.header.frame_id = transform.child_frame_id;
    inverse.child_frame_id = transform.header.frame_id;
    inverse.header.stamp = transform.header.stamp;

    // 将原始变换转换为 tf"has some points outside of view"2::Transform
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    // 计算逆变换
    tf2::Transform tf_inverse = tf_transform.inverse();

    // 将逆变换转换回 geometry_msgslocate
    inverse.transform = tf2::toMsg(tf_inverse);

    return inverse;
}

void draw_car_bbox(const radar_msgs::msg::CarBbox msg, cv::Mat& frame)
{
    for (auto bbox : msg.bboxs)
    {
        cv::Scalar color;
        if (bbox.class_id == -1) color = cv::Scalar(255, 0, 255);
        else color = (bbox.class_id < 6) ? cv::Scalar(255, 128, 0) : cv::Scalar(50, 50, 255);
        if (bbox.x_min > 0 || bbox.y_min > 20 || bbox.x_max < frame.cols - 50 || bbox.y_max < frame.rows)
        {
            cv::rectangle(frame, cv::Point(bbox.x_min, bbox.y_min), cv::Point(bbox.x_max, bbox.y_max), color, 10);
            cv::putText(frame, std::to_string((bbox.class_id) % 6 + 1), cv::Point(bbox.x_min + 40, bbox.y_min - 10), cv::FONT_HERSHEY_PLAIN, 6, color, 6);
        }
    }
}

radar_msgs::msg::CarBbox DetectorManager::detect_once(cv::Mat &image, float elapsed, float display_fps)
{
    cv::Mat cloned_image = image.clone();
    radar_msgs::msg::CarBbox car_bboxs;
    car_bboxs.header.stamp = clock_->now();
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

        // cv::putText(image, std::to_string((int)result.classes[j]), cv::Point(r.x, r.y - 10), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
        // std::ostringstream oss;
        // oss << std::fixed << std::setprecision(2) << result.scores[j];
        // std::string conf_str = oss.str();
        // cv::putText(image, conf_str, cv::Point(r.x + 40, r.y - 10), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0x27, 0xC1, 0x36), 2);
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
    draw_car_bbox(car_bboxs, image);
    
    // tracker
    // car_bboxs = tracker_manager_->callback(car_bboxs);
    // draw_car_bbox(car_bboxs, cloned_image);
    // double scale = 0.5;
    // cv::resize(cloned_image, cloned_image, cv::Size(), scale, scale);
    // cv::imshow("tracked_output", cloned_image);

    // tracker_manager_->record(car_bboxs, cloned_image);
    // end of tracker
    return car_bboxs; 
    //carbbox_publisher_->publish(car_bboxs);
}

radar_msgs::msg::CarBbox DetectorManager::detect_armors_on_bboxes(
    cv::Mat &img,
    const std::vector<std::vector<float>>& bboxes)
{
    cv::Mat cloned_image = img.clone();
    radar_msgs::msg::CarBbox car_bboxs;
    car_bboxs.header.stamp = clock_->now();
    car_bboxs.img_height = img.rows;
    car_bboxs.img_width = img.cols;

    timer_->syn_start("armor detect");
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    std::vector<cv::Rect> car_rects;
    std::vector<std::future<armor_result>> futures;

    for (size_t i = 0; i < bboxes.size(); i++)
    {
        if (bboxes[i].size() != 4) continue;

        int x = static_cast<int>(bboxes[i][0]);
        int y = static_cast<int>(bboxes[i][1]);
        int width = static_cast<int>(bboxes[i][2] - bboxes[i][0]);
        int height = static_cast<int>(bboxes[i][3] - bboxes[i][1]);
        
        cv::Rect r(x, y, width, height);

        r &= cv::Rect(0, 0, img.cols, img.rows);

        if (r.width < 5 || r.height < 5) continue;

        radar_msgs::msg::Bbox bbox;
        bbox.x_min = r.x;
        bbox.y_min = r.y;
        bbox.x_max = r.x + r.width;
        bbox.y_max = r.y + r.height;
        bbox.class_id = -1;  
        bbox.class_confidence = -1;
        car_bboxs.bboxs.push_back(bbox);

        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

        cv::Mat region = img(r);
        car_rects.push_back(r);

        auto future = submit_car(region);
        futures.push_back(std::move(future));

        cv::putText(img, std::to_string(i), 
                   cv::Point(r.x, r.y - 10), 
                   cv::FONT_HERSHEY_PLAIN, 1.2, 
                   cv::Scalar(0x27, 0xC1, 0x36), 2);
    }
    for (size_t i = 0; i < futures.size(); i++)
    {
        try
        {
            armor_result result = futures[i].get();

            if (result.detcted_img.rows > 0 && result.detcted_img.cols > 0) {
                result.detcted_img.copyTo(img(car_rects[i]));
            }

            if (i < car_bboxs.bboxs.size()) {
                car_bboxs.bboxs[i].class_id = result.class_id;
                car_bboxs.bboxs[i].class_confidence = result.class_score;
                
                // Add armor confidence text
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << result.class_score;
                cv::putText(img, oss.str(),
                           cv::Point(car_rects[i].x + 40, car_rects[i].y + 20),
                           cv::FONT_HERSHEY_PLAIN, 1.2,
                           cv::Scalar(0x27, 0xC1, 0x36), 2);
            }

        } catch (const std::exception& e) {
        }
    }
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

    car_bboxs = tracker_manager_->callback(car_bboxs);
    
    float elapsed = timer_->syn_stop("armor_detect") * 1000.0f; // Convert to ms
    
    return car_bboxs;
}
