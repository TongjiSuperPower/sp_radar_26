#include "../include/decision.hpp"

DecisionNode::DecisionNode() : Node("decision_node")
{
    auto config = YAML::LoadFile("./src/main/decision/config/decision.yaml");
    enemy_ = config["enemy"].as<std::string>();

    robot_id_ = 0;
    last_em_wave_position_time_ = this->now();
    has_em_wave_opponent_position_ = false;
    radar_info_chance_ = 0;
    radar_info_istriggered_ = false;
    radar_cmd_cnt_ = 0;
    last_big_energy_trigger_stage_time_ = UINT16_MAX;

    radar_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarCmd>("/radar_cmd", 1);
    map_robot_data_pub_ = this->create_publisher<radar_msgs::msg::MapRobotData>("/map_robot_data", 1);
    custom_info_pub_ = this->create_publisher<radar_msgs::msg::CustomInfo>("/custom_info", 1);
    radar_demod_config_pub_ = this->create_publisher<radar_parse_em_wave::msg::RadarParseEmWaveDemodConfig>("/rmuc/demod_config", 1);
    radar_sentry_position_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarSentryPositionCmd>("/radar_sentry_position_cmd", 1);
    combined_data_pub_ = this->create_publisher<radar_msgs::msg::CombinedData>("/combined_data", 10);
    dart_warning_cmd_pub_ = this->create_publisher<radar_msgs::msg::DartWarningCmd>("/dart_warning_cmd", 1);

    map_robot_data_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&DecisionNode::pubMapRobotData, this));   // 5Hz
    radar_cmd_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(35), std::bind(&DecisionNode::pubRadarCmd, this));    // 30Hz
    custom_info_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(350), std::bind(&DecisionNode::pubCustomInfo, this)); // 3Hz
    combined_data_build_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&DecisionNode::buildCombinedData, this)); // 3Hz
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
    secret_key_sub = this->create_subscription<radar_parse_em_wave::msg::RadarParseEmWave0A06InterferenceKey>(
        "/rmuc/rx/packet_0x0a06", 10, std::bind(&DecisionNode::secretKeyCallback, this, std::placeholders::_1));
    dart_warning_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/dart_gate_status", 10, std::bind(&DecisionNode::dartWarningCallback, this, std::placeholders::_1));
    robot_buff_sub_ = this->create_subscription<radar_parse_em_wave::msg::RadarParseEmWave0A05RobotBuff>(
        "/rmuc/rx/packet_0x0a05", 10, std::bind(&DecisionNode::robotBuffCallback, this, std::placeholders::_1));
    robot_position_sub_ = this->create_subscription<radar_parse_em_wave::msg::RadarParseEmWave0A01RobotPosition>(
        "/rmuc/rx/packet_0x0a01", 10, std::bind(&DecisionNode::robotPositionCallback, this, std::placeholders::_1));
    robot_hp_sub_ = this->create_subscription<radar_parse_em_wave::msg::RadarParseEmWave0A02RobotHp>(
        "/rmuc/rx/packet_0x0a02", 10, std::bind(&DecisionNode::robotHpCallback, this, std::placeholders::_1));
    robot_ammo_sub_ = this->create_subscription<radar_parse_em_wave::msg::RadarParseEmWave0A03RobotAmmo>(
        "/rmuc/rx/packet_0x0a03", 10, std::bind(&DecisionNode::robotAmmoCallback, this, std::placeholders::_1));
    field_status_sub_ = this->create_subscription<radar_parse_em_wave::msg::RadarParseEmWave0A04FieldStatus>(
        "/rmuc/rx/packet_0x0a04", 10, std::bind(&DecisionNode::fieldStatusCallback, this, std::placeholders::_1));

    // map_robot_data_mono_sub_ = this->create_subscription<radar_msgs::msg::MapRobotData>((
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

    // 发布后清零缓存，等待下一轮数据到达
    map_robot_data_pc_ref_ = radar_msgs::msg::MapRobotData();
    has_em_wave_opponent_position_ = false;
    has_map_robot_data_ = false;
} // TODO 相机点云定位融合单目定位

void DecisionNode::pubRadarCmd()
{
    // Priority 1: 0x0121 radar commands
    if (!radar_cmd_queue_.empty()) {
        auto cmd = radar_cmd_queue_.front();
        radar_cmd_queue_.pop();
        radar_cmd_pub_->publish(cmd);
    }
    // Priority 2: 0x0211 sentry position (只发给哨兵7号)
    else if (!sentry_position_queue_.empty()) {
        auto cmd = sentry_position_queue_.front();
        sentry_position_queue_.pop_front();
        radar_sentry_position_cmd_pub_->publish(cmd);
    }
    // Priority 3: 0x0210 dart warning (单独话题)
    else if (!dart_warning_queue_.empty()) {
        auto cmd = dart_warning_queue_.front();
        dart_warning_queue_.pop_front();
        dart_warning_cmd_pub_->publish(cmd);
    }
    // Priority 4: 0x0212 combined data (data + routing together)
    else if (!combined_data_queue_.empty()) {
        auto cmd = combined_data_queue_.front();
        combined_data_queue_.pop_front();
        combined_data_pub_->publish(cmd);
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
        // 比赛结束前3分钟仍未触发，则触发第一次双倍易伤
        if (stage_remain_time_ < (3 * 60 + 5) && stage_remain_time_ > (2 * 60 + 55)) // 3min05s~2min55s
        {
            if(radar_cmd_cnt_ == 0)
            {
                pushCustomInfo(DOUBLE_VULNERABLE);
                pushRadarCmd(1, 0, secret_key_);
            }
        }
        // 比赛结束前2分钟仍未触发，则触发第二次双倍易伤
        if (stage_remain_time_ < (2 * 60 + 5) && stage_remain_time_ > (1 * 60 + 55)) // 2min05s~1min55s
        {
            if(radar_cmd_cnt_ <= 1)
            {
                pushCustomInfo(DOUBLE_VULNERABLE);
                pushRadarCmd(2, 0, secret_key_);
            }
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
    is_small_energy_machine_activated_ = (event_data_ref_.event_data >> 3) & 0x01; // 4Byte bit[3]
    is_big_energy_machine_activated_ = (event_data_ref_.event_data >> 5) & 0x01;   // 4Byte bit[4]
    is_someone_in_central_highland_ = (event_data_ref_.event_data >> 25) & 0x03;    // 4Byte bit[5:6]

    if (game_type_ != 0x01) // RMUC
        return;

    if (radar_info_chance_ >= 1 && !radar_info_istriggered_)
    {
        std::cout << "EventData received:" << "big_energy_machine=" << static_cast<int>(is_big_energy_machine_activated_)
                  << std::endl;
        // 大能量机关激活时触发双倍易伤（60s冷却，防止同一次激活重复触发）
        bool cooldown_elapsed = (last_big_energy_trigger_stage_time_ - stage_remain_time_) >= 60;
        if (is_big_energy_machine_activated_ && cooldown_elapsed)
        {
            if (radar_cmd_cnt_ == 0)
            {
                pushCustomInfo(DOUBLE_VULNERABLE);
                pushRadarCmd(1, 0, secret_key_); // 大能量机关被激活，第一次双倍易伤
                std::cout << "Big energy machine activated, first DOUBLE_VULNERABLE triggered." << std::endl;
            }
            else if (radar_cmd_cnt_ == 1)
            {
                pushCustomInfo(DOUBLE_VULNERABLE);
                pushRadarCmd(2, 0, secret_key_); // 大能量机关被激活，第二次双倍易伤
                std::cout << "Big energy machine activated, second DOUBLE_VULNERABLE triggered." << std::endl;
            }
            last_big_energy_trigger_stage_time_ = stage_remain_time_;
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
    //std::cout << "Radar info received: chance=" << static_cast<int>(radar_info_chance_) << std::endl;
    radar_info_istriggered_ = radar_info_ref_.radar_info_istriggered; // 1Byte bit[2]

    // Publish DemodConfig with team color and interference level
    radar_parse_em_wave::msg::RadarParseEmWaveDemodConfig demod_config;
    demod_config.team = robot_color_;   
    demod_config.interference_level = radar_info_ref_.encryption_level;
    radar_demod_config_pub_->publish(demod_config);
}

void DecisionNode::CarsCallback(const radar_msgs::msg::Cars::ConstPtr &msg)
{
    // 有信息波时先保存对方位置，等正常流程跑完再恢复
    bool has_em = has_em_wave_opponent_position_ &&
                  map_robot_data_pc_ref_.opponent_hero_position_x > 0;
    RCLCPP_INFO(this->get_logger(), "has_em_wave_opponent_position_ = %d, map_robot_data_pc_ref_.opponent_hero_position_x = %d", has_em_wave_opponent_position_, map_robot_data_pc_ref_.opponent_hero_position_x);
    RCLCPP_INFO(this->get_logger(), "check has_em %d", has_em);
    uint16_t em_opponent[12];
    if (has_em) {
        memcpy(em_opponent, &map_robot_data_pc_ref_.opponent_hero_position_x, sizeof(em_opponent));
    }

    // 正常流程：重置并填充所有位置（对方+己方）
    map_robot_data_pc_ref_ = radar_msgs::msg::MapRobotData();
    auto result = tracker_manager_.callback(msg);

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

    map_robot_data_pc_ref_.opponent_aerial_position_x = msg->opponent_aerial_position_x;
    map_robot_data_pc_ref_.opponent_aerial_position_y = msg->opponent_aerial_position_y;
    map_robot_data_pc_ref_.ally_aerial_position_x = msg->ally_aerial_position_x;
    map_robot_data_pc_ref_.ally_aerial_position_y = msg->ally_aerial_position_y;

    // 有信息波时，用信息波对手位置覆盖跟踪结果
    if (has_em) {
        memcpy(&map_robot_data_pc_ref_.opponent_hero_position_x, em_opponent, sizeof(em_opponent));
    }

    has_map_robot_data_ = true;

    // 仅在没有信息波位置时(超过500ms未收到)发送雷达定位结果(source=0, 单位: cm)
    if ((this->now() - last_em_wave_position_time_).seconds() > 0.5) {
        radar_msgs::msg::RadarSentryPositionCmd radar_pos_cmd;
        radar_pos_cmd.source = 0;
        radar_pos_cmd.hero_x = map_robot_data_pc_ref_.opponent_hero_position_x;
        radar_pos_cmd.hero_y = map_robot_data_pc_ref_.opponent_hero_position_y;
        radar_pos_cmd.engineer_x = map_robot_data_pc_ref_.opponent_engineer_position_x;
        radar_pos_cmd.engineer_y = map_robot_data_pc_ref_.opponent_engineer_position_y;
        radar_pos_cmd.infantry3_x = map_robot_data_pc_ref_.opponent_infantry_3_position_x;
        radar_pos_cmd.infantry3_y = map_robot_data_pc_ref_.opponent_infantry_3_position_y;
        radar_pos_cmd.infantry4_x = map_robot_data_pc_ref_.opponent_infantry_4_position_x;
        radar_pos_cmd.infantry4_y = map_robot_data_pc_ref_.opponent_infantry_4_position_y;
        radar_pos_cmd.aerial_x = map_robot_data_pc_ref_.opponent_aerial_position_x;
        radar_pos_cmd.aerial_y = map_robot_data_pc_ref_.opponent_aerial_position_y;
        radar_pos_cmd.sentry_x = map_robot_data_pc_ref_.opponent_sentry_position_x;
        radar_pos_cmd.sentry_y = map_robot_data_pc_ref_.opponent_sentry_position_y;
        sentry_position_queue_.push_back(radar_pos_cmd);
    }
}

void DecisionNode::secretKeyCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A06InterferenceKey::ConstPtr &msg)
{
    // 干扰波等级达到3级后不再发送密钥，避免10s冷却期内正确密钥被去重导致永远卡住
    if (radar_info_ref_.encryption_level >= 3) return;
    secret_key_ = msg->key;
    RCLCPP_INFO(this->get_logger(), "Received secret key: %s", secret_key_.c_str());
    pushRadarCmd(radar_cmd_cnt_, 2, secret_key_);
}

void DecisionNode::dartWarningCallback(const std_msgs::msg::Int8::ConstPtr &msg)
{
    if (msg->data == 1) { // 接收到敌方飞镖警告
        RCLCPP_INFO(this->get_logger(), "Received enemy dart warning");
        pushCustomInfo(DART_WARNING);
        // 推送0x0210飞镖警告到单独队列（数据+路由合并，共用30Hz带宽）
        int base = (robot_id_ >= 100) ? 101 : 1;
        for (int i = base; i < base + 7; ++i) {
            if (i == base + 4) continue; // skip inf5
            radar_msgs::msg::DartWarningCmd cmd;
            cmd.dart_gate_status = msg->data;
            cmd.receiver_id = i;
            dart_warning_queue_.push_back(cmd);
        }
    }
}

void DecisionNode::robotBuffCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A05RobotBuff::ConstPtr &msg)
{
    // 缓存0x0A05数据
    latest_buff_data_ = *msg;
    has_buff_data_ = true;
}

void DecisionNode::robotPositionCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A01RobotPosition::ConstPtr &msg)
{
    last_em_wave_position_time_ = this->now();
    has_em_wave_opponent_position_ = true;
    has_map_robot_data_ = true;

    // 用信息波位置(cm)替换对手机器人位置到 map_robot_data
    map_robot_data_pc_ref_.opponent_hero_position_x = static_cast<uint16_t>(msg->hero_x);
    map_robot_data_pc_ref_.opponent_hero_position_y = static_cast<uint16_t>(msg->hero_y);
    map_robot_data_pc_ref_.opponent_engineer_position_x = static_cast<uint16_t>(msg->engineer_x);
    map_robot_data_pc_ref_.opponent_engineer_position_y = static_cast<uint16_t>(msg->engineer_y);
    map_robot_data_pc_ref_.opponent_infantry_3_position_x = static_cast<uint16_t>(msg->infantry3_x);
    map_robot_data_pc_ref_.opponent_infantry_3_position_y = static_cast<uint16_t>(msg->infantry3_y);
    map_robot_data_pc_ref_.opponent_infantry_4_position_x = static_cast<uint16_t>(msg->infantry4_x);
    map_robot_data_pc_ref_.opponent_infantry_4_position_y = static_cast<uint16_t>(msg->infantry4_y);
    map_robot_data_pc_ref_.opponent_aerial_position_x = static_cast<uint16_t>(msg->aerial_x);
    map_robot_data_pc_ref_.opponent_aerial_position_y = static_cast<uint16_t>(msg->aerial_y);
    map_robot_data_pc_ref_.opponent_sentry_position_x = static_cast<uint16_t>(msg->sentry_x);
    map_robot_data_pc_ref_.opponent_sentry_position_y = static_cast<uint16_t>(msg->sentry_y);

    // 将0x0A01信息波位置数据(source=1)转发给己方哨兵(单位: cm)
    radar_msgs::msg::RadarSentryPositionCmd sentry_pos_cmd;
    sentry_pos_cmd.source = 1;
    sentry_pos_cmd.hero_x = msg->hero_x;
    sentry_pos_cmd.hero_y = msg->hero_y;
    sentry_pos_cmd.engineer_x = msg->engineer_x;
    sentry_pos_cmd.engineer_y = msg->engineer_y;
    sentry_pos_cmd.infantry3_x = msg->infantry3_x;
    sentry_pos_cmd.infantry3_y = msg->infantry3_y;
    sentry_pos_cmd.infantry4_x = msg->infantry4_x;
    sentry_pos_cmd.infantry4_y = msg->infantry4_y;
    sentry_pos_cmd.aerial_x = msg->aerial_x;
    sentry_pos_cmd.aerial_y = msg->aerial_y;
    sentry_pos_cmd.sentry_x = msg->sentry_x;
    sentry_pos_cmd.sentry_y = msg->sentry_y;
    sentry_position_queue_.push_back(sentry_pos_cmd);
}

void DecisionNode::robotHpCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A02RobotHp::ConstPtr &msg)
{
    // 缓存0x0A02数据
    latest_hp_data_ = *msg;
    has_hp_data_ = true;
}

void DecisionNode::robotAmmoCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A03RobotAmmo::ConstPtr &msg)
{
    // 缓存0x0A03数据
    latest_ammo_data_ = *msg;
    has_ammo_data_ = true;
}

void DecisionNode::fieldStatusCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A04FieldStatus::ConstPtr &msg)
{
    // 缓存0x0A04数据
    latest_field_data_ = *msg;
    has_field_data_ = true;
}

void DecisionNode::buildCombinedData()
{
    radar_msgs::msg::CombinedData base;
    base.data_cmd_id = 0x0212;

    // 全部字段默认填充-1（uint8→0xFF, uint16→0xFFFF, uint32→0xFFFFFFFF），无新数据时发送无效值

    // 0x0A02 HP数据 (uint16: 0xFFFF)
    base.hero_hp = 0xFFFF;
    base.engineer_hp = 0xFFFF;
    base.infantry3_hp = 0xFFFF;
    base.infantry4_hp = 0xFFFF;
    base.sentry_hp = 0xFFFF;

    // 0x0A03 Ammo数据 (uint16: 0xFFFF)
    base.hero_ammo = 0xFFFF;
    base.infantry3_ammo = 0xFFFF;
    base.infantry4_ammo = 0xFFFF;
    base.aerial_ammo = 0xFFFF;
    base.sentry_ammo = 0xFFFF;

    // 0x0A04 Field数据
    base.remain_coins = 0xFFFF;
    base.total_coins = 0xFFFF;
    base.status_flags = 0xFFFFFFFF;

    // 0x0A05 Buff数据 (uint8: 0xFF, uint16: 0xFFFF)
    base.hero_heal = 0xFF;
    base.hero_cool = 0xFFFF;
    base.hero_def = 0xFF;
    base.hero_vuln = 0xFF;
    base.hero_atk = 0xFFFF;
    base.engineer_heal = 0xFF;
    base.engineer_cool = 0xFFFF;
    base.engineer_def = 0xFF;
    base.engineer_vuln = 0xFF;
    base.engineer_atk = 0xFFFF;
    base.infantry3_heal = 0xFF;
    base.infantry3_cool = 0xFFFF;
    base.infantry3_def = 0xFF;
    base.infantry3_vuln = 0xFF;
    base.infantry3_atk = 0xFFFF;
    base.infantry4_heal = 0xFF;
    base.infantry4_cool = 0xFFFF;
    base.infantry4_def = 0xFF;
    base.infantry4_vuln = 0xFF;
    base.infantry4_atk = 0xFFFF;
    base.sentry_heal = 0xFF;
    base.sentry_cool = 0xFFFF;
    base.sentry_def = 0xFF;
    base.sentry_vuln = 0xFF;
    base.sentry_atk = 0xFFFF;
    base.sentry_posture = 0xFF;
    base.hero_status = 0xFF;
    base.engineer_status = 0xFF;
    base.infantry3_status = 0xFF;
    base.infantry4_status = 0xFF;
    base.sentry_status = 0xFF;

    // 有新数据时覆盖对应字段
    if (has_hp_data_) {
        base.hero_hp = latest_hp_data_.hero_hp;
        base.engineer_hp = latest_hp_data_.engineer_hp;
        base.infantry3_hp = latest_hp_data_.infantry3_hp;
        base.infantry4_hp = latest_hp_data_.infantry4_hp;
        base.sentry_hp = latest_hp_data_.sentry_hp;
    }

    if (has_ammo_data_) {
        base.hero_ammo = latest_ammo_data_.hero_ammo;
        base.infantry3_ammo = latest_ammo_data_.infantry3_ammo;
        base.infantry4_ammo = latest_ammo_data_.infantry4_ammo;
        base.aerial_ammo = latest_ammo_data_.aerial_ammo;
        base.sentry_ammo = latest_ammo_data_.sentry_ammo;
    }

    if (has_field_data_) {
        base.remain_coins = latest_field_data_.remain_coins;
        base.total_coins = latest_field_data_.total_coins;
        base.status_flags = latest_field_data_.status_flags;
    }

    if (has_buff_data_) {
        base.hero_heal = latest_buff_data_.hero_heal;
        base.hero_cool = latest_buff_data_.hero_cool;
        base.hero_def = latest_buff_data_.hero_def;
        base.hero_vuln = latest_buff_data_.hero_vuln;
        base.hero_atk = latest_buff_data_.hero_atk;
        base.engineer_heal = latest_buff_data_.engineer_heal;
        base.engineer_cool = latest_buff_data_.engineer_cool;
        base.engineer_def = latest_buff_data_.engineer_def;
        base.engineer_vuln = latest_buff_data_.engineer_vuln;
        base.engineer_atk = latest_buff_data_.engineer_atk;
        base.infantry3_heal = latest_buff_data_.infantry3_heal;
        base.infantry3_cool = latest_buff_data_.infantry3_cool;
        base.infantry3_def = latest_buff_data_.infantry3_def;
        base.infantry3_vuln = latest_buff_data_.infantry3_vuln;
        base.infantry3_atk = latest_buff_data_.infantry3_atk;
        base.infantry4_heal = latest_buff_data_.infantry4_heal;
        base.infantry4_cool = latest_buff_data_.infantry4_cool;
        base.infantry4_def = latest_buff_data_.infantry4_def;
        base.infantry4_vuln = latest_buff_data_.infantry4_vuln;
        base.infantry4_atk = latest_buff_data_.infantry4_atk;
        base.sentry_heal = latest_buff_data_.sentry_heal;
        base.sentry_cool = latest_buff_data_.sentry_cool;
        base.sentry_def = latest_buff_data_.sentry_def;
        base.sentry_vuln = latest_buff_data_.sentry_vuln;
        base.sentry_atk = latest_buff_data_.sentry_atk;
        base.sentry_posture = latest_buff_data_.sentry_posture;
        base.hero_status = latest_buff_data_.hero_status;
        base.engineer_status = latest_buff_data_.engineer_status;
        base.infantry3_status = latest_buff_data_.infantry3_status;
        base.infantry4_status = latest_buff_data_.infantry4_status;
        base.sentry_status = latest_buff_data_.sentry_status;
    }

    // 推送0x0212到combined_data队列（发送给所有己方机器人）
    int base_id = (robot_id_ >= 100) ? 101 : 1;
    for (int i = base_id; i < base_id + 7; ++i) {
        if (i == base_id + 4) continue; // skip inf5
        base.receiver_id = i;
        combined_data_queue_.push_back(base);
    }

    // 构建完成后清除标志，等待下一轮数据
    has_hp_data_ = false;
    has_ammo_data_ = false;
    has_field_data_ = false;
    has_buff_data_ = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();

    return 0;
}
