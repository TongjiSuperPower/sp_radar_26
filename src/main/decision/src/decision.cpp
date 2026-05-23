#include "../include/decision.hpp"

DecisionNode::DecisionNode() : Node("decision_node")
{
    auto config = YAML::LoadFile("./src/main/decision/config/decision.yaml");
    enemy_ = config["enemy"].as<std::string>();

    robot_id_ = 0;
    radar_info_chance_ = 0;
    radar_info_istriggered_ = false;
    radar_cmd_cnt_ = 0;

    radar_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarCmd>("/radar_cmd", 1);
    map_robot_data_pub_ = this->create_publisher<radar_msgs::msg::MapRobotData>("/map_robot_data", 1);
    custom_info_pub_ = this->create_publisher<radar_msgs::msg::CustomInfo>("/custom_info", 1);

    map_robot_data_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&DecisionNode::pubMapRobotData, this));   // 5Hz
    radar_cmd_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(35), std::bind(&DecisionNode::pubRadarCmd, this));    // 30Hz
    custom_info_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(350), std::bind(&DecisionNode::pubCustomInfo, this)); // 3Hz

    game_status_sub_ = this->create_subscription<radar_msgs::msg::GameStatus>(
        "/game_status", 10, std::bind(&DecisionNode::gameStatusCallback, this, std::placeholders::_1));
    game_robot_hp_sub_ = this->create_subscription<radar_msgs::msg::GameRobotHP>(
        "/game_robot_hp", 10, std::bind(&DecisionNode::gameRobotHpCallback, this, std::placeholders::_1));
    event_data_sub_ = this->create_subscription<radar_msgs::msg::EventData>(
        "/event_data", 10, std::bind(&DecisionNode::eventDataCallback, this, std::placeholders::_1));
    robot_status_sub_ = this->create_subscription<radar_msgs::msg::RobotStatus>(
        "/robot_status", 10, std::bind(&DecisionNode::robotStatusCallback, this, std::placeholders::_1));
    radar_info_sub_ = this->create_subscription<radar_msgs::msg::RadarInfo>(
        "/radar_info", 10, std::bind(&DecisionNode::radarInfoCallback, this, std::placeholders::_1));
    cars_sub_ = this->create_subscription<radar_msgs::msg::Cars>(
        "/map_robot_data_pc", 10, std::bind(&DecisionNode::CarsCallback, this, std::placeholders::_1));
    secret_key_sub = this->create_subscription<std_msgs::msg::String>(
        "/secret_key", 10, std::bind(&DecisionNode::secretKeyCallback, this, std::placeholders::_1));
    dart_warning_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/dart_gate_status", 10, std::bind(&DecisionNode::dartWarningCallback, this, std::placeholders::_1));

    // map_robot_data_mono_sub_ = this->create_subscription<radar_msgs::msg::MapRobotData>(
    //     "/map_robot_data_mono", 10, std::bind(&DecisionNode::mapRobotDataMonoCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "DecisionNode initialized");
}
DecisionNode::~DecisionNode()
{
    timer_.print();
}

void DecisionNode::pubMapRobotData()
{
    radar_msgs::msg::MapRobotData map_robot_data;
    map_robot_data = map_robot_data_pc_ref_;
    map_robot_data.header.stamp = this->now();

    map_robot_data_pub_->publish(map_robot_data);
} // TODO 相机点云定位融合单目定位

void DecisionNode::pubRadarCmd()
{
    if (!radar_cmd_queue_.empty()) {
        auto cmd = radar_cmd_queue_.front();
        radar_cmd_queue_.pop();
        radar_cmd_pub_->publish(cmd);
    }
}

void DecisionNode::pubCustomInfo()
{
    // RCLCPP_INFO(this->get_logger(), "custom queue size: %d", custom_info_queue_.size());
    while (custom_info_queue_.size() > 10) {
        custom_info_queue_.pop();
    }
    if (!custom_info_queue_.empty()) {
        auto cmd = custom_info_queue_.front();
        custom_info_queue_.pop();
        custom_info_pub_->publish(cmd);
    }
}


void DecisionNode::pushCustomInfo(int custom_info_id)
{
    static int dart_warning_count = 0;
    if (game_process_ != 4)
        return;
        
    RCLCPP_INFO(this->get_logger(), "push custom info, type = %d (1: DOUBLE_VULNERABLE, 2: DART_WARNING)", custom_info_id);
    radar_msgs::msg::CustomInfo custom_info;
    custom_info.header.stamp = this->now();
    custom_info.sender_id = robot_id_;

    const int client_count = 5;
    auto client_ids = (robot_id_ == RED_RADAR ? red_clinets_id : blue_clinets_id);  
    static auto last_dart_warning_time = this->now();
    switch (custom_info_id)
    {
    case DOUBLE_VULNERABLE:
        memcpy(&custom_info.user_data, double_vulnerable_data, sizeof(double_vulnerable_data));
        custom_info.user_data[6] = stage_remain_time_ / 60 + '0';       // 分钟
        custom_info.user_data[10] = stage_remain_time_ % 60 / 10 + '0'; // 秒
        custom_info.user_data[12] = stage_remain_time_ % 60 % 10 + '0'; // 秒

        for (int client_index = 2; client_index < client_count; client_index++) {   // 双倍易伤只发给步兵和云台手
            custom_info.receiver_id = client_ids[client_index];
            custom_info_queue_.push(custom_info);
        }
        break;
    case DART_WARNING:
        if (dart_warning_count % 2 == 0) {
            memcpy(&custom_info.user_data, dart_warning_data1, sizeof(dart_warning_data1));
        } else {
            memcpy(&custom_info.user_data, dart_warning_data2, sizeof(dart_warning_data2));
        }
        ++ dart_warning_count;
        custom_info.user_data[6] = stage_remain_time_ / 60 + '0';       // 分钟
        custom_info.user_data[10] = stage_remain_time_ % 60 / 10 + '0'; // 秒
        custom_info.user_data[12] = stage_remain_time_ % 60 % 10 + '0'; // 秒

        RCLCPP_INFO(this->get_logger(), "Dart warning, stage remain time: %d", stage_remain_time_);
        for (int client_index = 0; client_index < client_count; client_index++) {
            custom_info.receiver_id = client_ids[client_index];
            custom_info_queue_.push(custom_info);
        }
        last_dart_warning_time = this->now();
        
        break;
    default:
        break;
    }
}

void DecisionNode::pushRadarCmd(int times, uint8_t password_cmd ,std::string password)
{
    for (int t = 0; t <= times; t++) {
        radar_msgs::msg::RadarCmd radar_cmd;
        radar_cmd.header.stamp = this->now();
        radar_cmd_cnt_ = t;
        radar_cmd.radar_cmd = radar_cmd_cnt_;
        radar_cmd.password_cmd = password_cmd;
        for (int i = 0; i < 6; ++i) radar_cmd.password[i] = password[i];
        radar_cmd_queue_.push(radar_cmd);
    }
}

void DecisionNode::gameStatusCallback(const radar_msgs::msg::GameStatus::ConstPtr &msg)
// TODO 可能会在1s内接收到多次gameStatus反馈，可以做滤波，保证custom_info只发送一次不过多占用资源
{
    game_status_ref_ = *msg;
    game_type_ = game_status_ref_.game_type;        // 1Byte bit[0:3]
    game_process_ = game_status_ref_.game_progress; // 1Byte bit[4:7]
    stage_remain_time_ = game_status_ref_.stage_remain_time;

    if (game_type_ != 0x01) // RMUC
        return;
    
    if (radar_info_chance_ >= 1 && !radar_info_istriggered_)
    {
        if (stage_remain_time_ < (5 * 60 + 21) && stage_remain_time_ > (4 * 60 + 49)) // 5min20s~4min50s
        {
            if(radar_cmd_cnt_ == 0)
                pushCustomInfo(DOUBLE_VULNERABLE);
            pushRadarCmd(1, 0, secret_key_); // 第一次
        }
        if (stage_remain_time_ < (4 * 60 + 1) && stage_remain_time_ > (3 * 60 + 29)) // 4min0s~3min30s
        {
            if(radar_cmd_cnt_ == 1 || radar_cmd_cnt_ == 0)
                pushCustomInfo(DOUBLE_VULNERABLE);
            pushRadarCmd(2, 0, secret_key_); // 第二次
        }
    }
}

void DecisionNode::gameRobotHpCallback(const radar_msgs::msg::GameRobotHP::ConstPtr &msg)
{
    game_robot_hp_ref_ = *msg;
    // Process the game robot HP
}

void DecisionNode::eventDataCallback(const radar_msgs::msg::EventData::ConstPtr &msg)
{
    event_data_ref_ = *msg;

    is_small_energy_machine_activated_ = (event_data_ref_.event_data >> 28) & 0x01; // 4Byte bit[3]
    is_big_energy_machine_activated_ = (event_data_ref_.event_data >> 27) & 0x01;   // 4Byte bit[4]
    is_someone_in_central_highland_ = (event_data_ref_.event_data >> 25) & 0x03;    // 4Byte bit[5:6]

    if (game_process_ != 0x01) // RMUC
        return;

    if (radar_info_chance_ >= 1 && !radar_info_istriggered_)
    {
        if (is_small_energy_machine_activated_ || is_big_energy_machine_activated_)
        {
            pushRadarCmd(2, 0, secret_key_); // 小能量机关或大能量机关被触发，发布双倍易伤
        }
    }
}

void DecisionNode::robotStatusCallback(const radar_msgs::msg::RobotStatus::ConstPtr &msg)
{
    robot_status_ref_ = *msg;
    robot_id_ = robot_status_ref_.robot_id;
}

void DecisionNode::radarInfoCallback(const radar_msgs::msg::RadarInfo::ConstPtr &msg)
{
    radar_info_ref_ = *msg;
    radar_info_chance_ = radar_info_ref_.radar_info_chance;           // 1Byte bit[0:1]
    radar_info_istriggered_ = radar_info_ref_.radar_info_istriggered; // 1Byte bit[2]
    // Process the radar info
}

void DecisionNode::CarsCallback(const radar_msgs::msg::Cars::ConstPtr &msg)
{
    // // dart warning
    // for (auto center : msg->cars) {
    //     if ((center.x > 27 && center.y > 3.9 && center.y < 4.6) ||
    //         (center.x < 1 && center.y < 11.1 && center.y > 10.4)) {
    //             // RCLCPP_INFO(this->get_logger(), "Dart warning");
    //             pushCustomInfo(DART_WARNING);
    //         }
    // }

    auto result = tracker_manager_.callback(msg);

    map_robot_data_pc_ref_ = radar_msgs::msg::MapRobotData();
    uint16_t* p = &map_robot_data_pc_ref_.opponent_hero_position_x;
    for (auto& car : result->cars) {
        int id;
        if (enemy_ == "blue")
        {
            id = car.class_id;
        }
        else
        {
            if (car.class_id >= 6) id = car.class_id - 6;
            else id = car.class_id + 6;
        }
        *(p + id * 2) = 100 * car.x;
        *(p + id * 2 + 1) = 100 * car.y;
    }
    pubMapRobotData();
}

void DecisionNode::secretKeyCallback(const std_msgs::msg::String::ConstPtr &msg)
{
    if (secret_key_ == msg->data) return; // 已经接收到过secret key了就不再更新了
    secret_key_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received secret key: %s", secret_key_.c_str());
    pushRadarCmd(radar_cmd_cnt_, 2, secret_key_);
}

void DecisionNode::dartWarningCallback(const std_msgs::msg::Int8::ConstPtr &msg)
{
    if (msg->data == 1) { // 接收到敌方飞镖警告
        RCLCPP_INFO(this->get_logger(), "Received enemy dart warning");
        pushCustomInfo(DART_WARNING);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();

    return 0;
}
