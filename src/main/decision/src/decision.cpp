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

    radar_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarCmd>("/radar_cmd", 1);
    map_robot_data_pub_ = this->create_publisher<radar_msgs::msg::MapRobotData>("/map_robot_data", 1);
    custom_info_pub_ = this->create_publisher<radar_msgs::msg::CustomInfo>("/custom_info", 1);
    radar_demod_config_pub_ = this->create_publisher<radar_parse_em_wave::msg::RadarParseEmWaveDemodConfig>("/rmuc/demod_config", 1);
    radar_sentry_position_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarSentryPositionCmd>("/radar_sentry_position_cmd", 1);
    radar_ally_hp_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarAllyHpCmd>("/radar_ally_hp_cmd", 1);
    radar_ally_ammo_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarAllyAmmoCmd>("/radar_ally_ammo_cmd", 1);
    radar_ally_field_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarAllyFieldCmd>("/radar_ally_field_cmd", 1);
    radar_ally_buff_cmd_pub_ = this->create_publisher<radar_msgs::msg::RadarAllyBuffCmd>("/radar_ally_buff_cmd", 1);

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
    has_em_wave_opponent_position_ = false;
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

    // 仅在没有信息波位置时(超过500ms未收到)发送雷达定位结果(source=0)
    if ((this->now() - last_em_wave_position_time_).seconds() > 0.5) {
        radar_msgs::msg::RadarSentryPositionCmd radar_pos_cmd;
        radar_pos_cmd.source = 0;
        radar_pos_cmd.hero_x = map_robot_data_pc_ref_.opponent_hero_position_x / 100;
        radar_pos_cmd.hero_y = map_robot_data_pc_ref_.opponent_hero_position_y / 100;
        radar_pos_cmd.engineer_x = map_robot_data_pc_ref_.opponent_engineer_position_x / 100;
        radar_pos_cmd.engineer_y = map_robot_data_pc_ref_.opponent_engineer_position_y / 100;
        radar_pos_cmd.infantry3_x = map_robot_data_pc_ref_.opponent_infantry_3_position_x / 100;
        radar_pos_cmd.infantry3_y = map_robot_data_pc_ref_.opponent_infantry_3_position_y / 100;
        radar_pos_cmd.infantry4_x = map_robot_data_pc_ref_.opponent_infantry_4_position_x / 100;
        radar_pos_cmd.infantry4_y = map_robot_data_pc_ref_.opponent_infantry_4_position_y / 100;
        radar_pos_cmd.aerial_x = map_robot_data_pc_ref_.opponent_aerial_position_x / 100;
        radar_pos_cmd.aerial_y = map_robot_data_pc_ref_.opponent_aerial_position_y / 100;
        radar_pos_cmd.sentry_x = map_robot_data_pc_ref_.opponent_sentry_position_x / 100;
        radar_pos_cmd.sentry_y = map_robot_data_pc_ref_.opponent_sentry_position_y / 100;
        radar_sentry_position_cmd_pub_->publish(radar_pos_cmd);
    }
}

void DecisionNode::secretKeyCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A06InterferenceKey::ConstPtr &msg)
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

void DecisionNode::robotBuffCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A05RobotBuff::ConstPtr &msg)
{
    // 将0x0A05 buff数据广播给己方全部机器人
    radar_msgs::msg::RadarAllyBuffCmd ally_buff_cmd;
    ally_buff_cmd.hero_heal = msg->hero_heal;
    ally_buff_cmd.hero_cool = msg->hero_cool;
    ally_buff_cmd.hero_def = msg->hero_def;
    ally_buff_cmd.hero_vuln = msg->hero_vuln;
    ally_buff_cmd.hero_atk = msg->hero_atk;
    ally_buff_cmd.engineer_heal = msg->engineer_heal;
    ally_buff_cmd.engineer_cool = msg->engineer_cool;
    ally_buff_cmd.engineer_def = msg->engineer_def;
    ally_buff_cmd.engineer_vuln = msg->engineer_vuln;
    ally_buff_cmd.engineer_atk = msg->engineer_atk;
    ally_buff_cmd.infantry3_heal = msg->infantry3_heal;
    ally_buff_cmd.infantry3_cool = msg->infantry3_cool;
    ally_buff_cmd.infantry3_def = msg->infantry3_def;
    ally_buff_cmd.infantry3_vuln = msg->infantry3_vuln;
    ally_buff_cmd.infantry3_atk = msg->infantry3_atk;
    ally_buff_cmd.infantry4_heal = msg->infantry4_heal;
    ally_buff_cmd.infantry4_cool = msg->infantry4_cool;
    ally_buff_cmd.infantry4_def = msg->infantry4_def;
    ally_buff_cmd.infantry4_vuln = msg->infantry4_vuln;
    ally_buff_cmd.infantry4_atk = msg->infantry4_atk;
    ally_buff_cmd.sentry_heal = msg->sentry_heal;
    ally_buff_cmd.sentry_cool = msg->sentry_cool;
    ally_buff_cmd.sentry_def = msg->sentry_def;
    ally_buff_cmd.sentry_vuln = msg->sentry_vuln;
    ally_buff_cmd.sentry_atk = msg->sentry_atk;
    ally_buff_cmd.sentry_posture = msg->sentry_posture;
    ally_buff_cmd.hero_status = msg->hero_status;
    ally_buff_cmd.engineer_status = msg->engineer_status;
    ally_buff_cmd.infantry3_status = msg->infantry3_status;
    ally_buff_cmd.infantry4_status = msg->infantry4_status;
    ally_buff_cmd.sentry_status = msg->sentry_status;
    radar_ally_buff_cmd_pub_->publish(ally_buff_cmd);
}

void DecisionNode::robotPositionCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A01RobotPosition::ConstPtr &msg)
{
    last_em_wave_position_time_ = this->now();
    has_em_wave_opponent_position_ = true;

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

    // 将0x0A01信息波位置数据(source=1)转发给己方哨兵(单位: m)
    radar_msgs::msg::RadarSentryPositionCmd sentry_pos_cmd;
    sentry_pos_cmd.source = 1;
    sentry_pos_cmd.hero_x = msg->hero_x / 100;
    sentry_pos_cmd.hero_y = msg->hero_y / 100;
    sentry_pos_cmd.engineer_x = msg->engineer_x / 100;
    sentry_pos_cmd.engineer_y = msg->engineer_y / 100;
    sentry_pos_cmd.infantry3_x = msg->infantry3_x / 100;
    sentry_pos_cmd.infantry3_y = msg->infantry3_y / 100;
    sentry_pos_cmd.infantry4_x = msg->infantry4_x / 100;
    sentry_pos_cmd.infantry4_y = msg->infantry4_y / 100;
    sentry_pos_cmd.aerial_x = msg->aerial_x / 100;
    sentry_pos_cmd.aerial_y = msg->aerial_y / 100;
    sentry_pos_cmd.sentry_x = msg->sentry_x / 100;
    sentry_pos_cmd.sentry_y = msg->sentry_y / 100;
    radar_sentry_position_cmd_pub_->publish(sentry_pos_cmd);
}

void DecisionNode::robotHpCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A02RobotHp::ConstPtr &msg)
{
    radar_msgs::msg::RadarAllyHpCmd ally_hp_cmd;
    ally_hp_cmd.hero_hp = msg->hero_hp;
    ally_hp_cmd.engineer_hp = msg->engineer_hp;
    ally_hp_cmd.infantry3_hp = msg->infantry3_hp;
    ally_hp_cmd.infantry4_hp = msg->infantry4_hp;
    ally_hp_cmd.sentry_hp = msg->sentry_hp;
    radar_ally_hp_cmd_pub_->publish(ally_hp_cmd);
}

void DecisionNode::robotAmmoCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A03RobotAmmo::ConstPtr &msg)
{
    radar_msgs::msg::RadarAllyAmmoCmd ally_ammo_cmd;
    ally_ammo_cmd.hero_ammo = msg->hero_ammo;
    ally_ammo_cmd.infantry3_ammo = msg->infantry3_ammo;
    ally_ammo_cmd.infantry4_ammo = msg->infantry4_ammo;
    ally_ammo_cmd.aerial_ammo = msg->aerial_ammo;
    ally_ammo_cmd.sentry_ammo = msg->sentry_ammo;
    radar_ally_ammo_cmd_pub_->publish(ally_ammo_cmd);
}

void DecisionNode::fieldStatusCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A04FieldStatus::ConstPtr &msg)
{
    radar_msgs::msg::RadarAllyFieldCmd ally_field_cmd;
    ally_field_cmd.remain_coins = msg->remain_coins;
    ally_field_cmd.total_coins = msg->total_coins;
    ally_field_cmd.status_flags = msg->status_flags;
    radar_ally_field_cmd_pub_->publish(ally_field_cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();

    return 0;
}
