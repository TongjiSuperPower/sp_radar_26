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
    radar_demod_config_pub_ = this->create_publisher<radar_msgs::msg::RadarParseEmWaveDemodConfig>("/rmuc/demod_config", 1);

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
        "/map_both_data_pc", 10, std::bind(&DecisionNode::CarsCallback, this, std::placeholders::_1));
    secret_key_sub = this->create_subscription<radar_msgs::msg::RadarParseEmWave0A06InterferenceKey>(
        "/rmuc/rx/packet_0x0a06", 10, std::bind(&DecisionNode::secretKeyCallback, this, std::placeholders::_1));
    dart_warning_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/dart_gate_status", 10, std::bind(&DecisionNode::dartWarningCallback, this, std::placeholders::_1));
    rf_info_wave_sub_ = this->create_subscription<radar_msgs::msg::RfInfoWave>(
        "/rf_info_wave", 10, std::bind(&DecisionNode::rfInfoWaveCallback, this, std::placeholders::_1));

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
    std::cout << "Push RadarCmd: radar_cmd=" << static_cast<int>(radar_cmd_cnt_)
              << ", password_cmd=" << static_cast<int>(password_cmd)
              << ", password=" << password
              << std::endl;
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
    robot_color_ = (robot_id_ >= 100) ? "blue" : "red";
}

void DecisionNode::radarInfoCallback(const radar_msgs::msg::RadarInfo::ConstPtr &msg)
{
    radar_info_ref_ = *msg;
    radar_info_chance_ = radar_info_ref_.radar_info_chance;           // 1Byte bit[0:1]
    radar_info_istriggered_ = radar_info_ref_.radar_info_istriggered; // 1Byte bit[2]

    // Publish DemodConfig with team color and interference level
    radar_msgs::msg::RadarParseEmWaveDemodConfig demod_config;
    demod_config.team = robot_color_;
    demod_config.interference_level = radar_info_ref_.encryption_level;
    radar_demod_config_pub_->publish(demod_config);
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

void DecisionNode::secretKeyCallback(const radar_msgs::msg::RadarParseEmWave0A06InterferenceKey::ConstPtr &msg)
{
    if (secret_key_ == msg->key) return; // 已经接收到过secret key了就不再更新了
    secret_key_ = msg->key;
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

void DecisionNode::rfInfoWaveCallback(const radar_msgs::msg::RfInfoWave::ConstPtr &msg)
{
    RCLCPP_INFO(this->get_logger(), "===== RF Info Wave Data Received =====");

    // Positions (0x0A01)
    RCLCPP_INFO(this->get_logger(),
        "Positions: hero(%d,%d) eng(%d,%d) inf3(%d,%d) inf4(%d,%d) aerial(%d,%d) sentry(%d,%d)",
        msg->positions[0], msg->positions[1],
        msg->positions[2], msg->positions[3],
        msg->positions[4], msg->positions[5],
        msg->positions[6], msg->positions[7],
        msg->positions[8], msg->positions[9],
        msg->positions[10], msg->positions[11]);

    // HP (0x0A02)
    RCLCPP_INFO(this->get_logger(),
        "HP: hero=%d eng=%d inf3=%d inf4=%d sentry=%d",
        msg->hp[0], msg->hp[1], msg->hp[2], msg->hp[3], msg->hp[4]);

    // Ammo (0x0A03)
    RCLCPP_INFO(this->get_logger(),
        "Ammo: hero=%d inf3=%d inf4=%d aerial=%d sentry=%d",
        msg->ammo[0], msg->ammo[1], msg->ammo[2], msg->ammo[3], msg->ammo[4]);

    // Field (0x0A04)
    RCLCPP_INFO(this->get_logger(),
        "Field: remain_coins=%d total_coins=%d status_flags=0x%08X",
        msg->remaining_coins, msg->total_coins, msg->status_flags);

    // Buffs (0x0A05) - summary only
    RCLCPP_INFO(this->get_logger(),
        "Buffs: hero(heal=%d cool=%d def=%d vuln=%d atk=%d) "
        "eng(heal=%d cool=%d def=%d vuln=%d atk=%d) "
        "inf3(heal=%d cool=%d def=%d vuln=%d atk=%d)",
        msg->buffs[0], msg->buffs[1], msg->buffs[2], msg->buffs[3], msg->buffs[4],
        msg->buffs[5], msg->buffs[6], msg->buffs[7], msg->buffs[8], msg->buffs[9],
        msg->buffs[10], msg->buffs[11], msg->buffs[12], msg->buffs[13], msg->buffs[14]);

    RCLCPP_INFO(this->get_logger(),
        "Buffs(cont): inf4(heal=%d cool=%d def=%d vuln=%d atk=%d) "
        "sentry(heal=%d cool=%d def=%d vuln=%d atk=%d) sentry_posture=%d",
        msg->buffs[15], msg->buffs[16], msg->buffs[17], msg->buffs[18], msg->buffs[19],
        msg->buffs[20], msg->buffs[21], msg->buffs[22], msg->buffs[23], msg->buffs[24],
        msg->sentry_posture);

    RCLCPP_INFO(this->get_logger(), "===== RF Info Wave Data End =====");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();

    return 0;
}
