#ifndef __DECISION_HPP
#define __DECISION_HPP

#include <iostream>
#include <cstdint>
#include <queue>
#include <deque>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include "radar_msgs/msg/game_status.hpp"
#include "radar_msgs/msg/game_robot_hp.hpp"
#include "radar_msgs/msg/event_data.hpp"
#include "radar_msgs/msg/robot_status.hpp"
#include "radar_msgs/msg/radar_info.hpp"
#include "radar_msgs/msg/radar_mark_data.hpp"
#include "radar_msgs/msg/radar_cmd.hpp"
#include "radar_msgs/msg/map_robot_data.hpp"
#include "radar_msgs/msg/custom_info.hpp"
#include "radar_msgs/msg/cars.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave0_a06_interference_key.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave_demod_config.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave0_a05_robot_buff.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave0_a01_robot_position.hpp"
#include "radar_msgs/msg/radar_sentry_position_cmd.hpp"
#include "radar_msgs/msg/radar_ally_combined_data.hpp"
#include "radar_msgs/msg/combined_data.hpp"
#include "radar_msgs/msg/dart_warning_cmd.hpp"
#include "radar_msgs/msg/aerial_countered_cmd.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave0_a02_robot_hp.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave0_a03_robot_ammo.hpp"
#include "radar_parse_em_wave/msg/radar_parse_em_wave0_a04_field_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"

#include "tracker.hpp"
#include "../tools/timer.hpp"

#define DOUBLE_VULNERABLE 1
#define DART_WARNING 2

enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_INFANTRY_3 = 3,
    RED_INFANTRY_4 = 4,
    RED_INFANTRY_5 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    RED_DART = 8,
    RED_RADAR = 9,
    RED_BASE = 10,
    RED_OUTPOST = 11,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_INFANTRY_3 = 103,
    BLUE_INFANTRY_4 = 104,
    BLUE_INFANTRY_5 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
    BLUE_DART = 108,
    BLUE_RADAR = 109,
    BLUE_BASE = 110,
    BLUE_OUTPOST = 111,
} RobotId;

enum
{
    RED_HERO_CLIENT = 0x0101,
    RED_ENGINEER_CLIENT = 0x0102,
    RED_INFANTRY_3_CLIENT = 0x0103,
    RED_INFANTRY_4_CLIENT = 0x0104,
    RED_INFANTRY_5_CLIENT = 0x0105,
    RED_AERIAL_CLIENT = 0x0106,
    BLUE_HERO_CLIENT = 0x0165,
    BLUE_ENGINEER_CLIENT = 0x0166,
    BLUE_INFANTRY_3_CLIENT = 0x0167,
    BLUE_INFANTRY_4_CLIENT = 0x0168,
    BLUE_INFANTRY_5_CLIENT = 0x0169,
    BLUE_AERIAL_CLIENT = 0x016A,
    REFEREE_SYSTEM_SERVER = 0x8080, 
} ClientId;

const uint16_t red_clinets_id[5] = {
    RED_HERO_CLIENT,
    RED_ENGINEER_CLIENT,
    RED_INFANTRY_3_CLIENT,
    RED_INFANTRY_4_CLIENT,
    RED_AERIAL_CLIENT
};

const uint16_t blue_clinets_id[5] = {
    BLUE_HERO_CLIENT,
    BLUE_ENGINEER_CLIENT,
    BLUE_INFANTRY_3_CLIENT,
    BLUE_INFANTRY_4_CLIENT,
    BLUE_AERIAL_CLIENT
};

const uint8_t double_vulnerable_data[30] = {
    0xF7, 0x96, 0xBE, 0x8F, 0x1A, 0xFF, 0x2A, 0x00, 0x3A, 0x00, 
    0x2A, 0x00, 0x2A, 0x00, 0xE6, 0x89, 0xD1, 0x53, 0xCC, 0x53, 
    0x0D, 0x50, 0x13, 0x66, 0x24, 0x4F, 0x00, 0x00, 0x00, 0x00
}; // 雷达：*:**触发双倍易伤

const uint8_t dart_warning_data1[30] = {
    0xF7, 0x96, 0xBE, 0x8F, 0x1A, 0xFF, 0x2A, 0x00, 0x3A, 0x00, 0x2A, 0x00, 0x2A, 0x00,  // 协议头
    0x4C, 0x65,  // 敌
    0xB9, 0x65,  // 方
    0xDE, 0x98,  // 飞
    0x56, 0x95,  // 镖
    0x0A, 0x00,  // \n
    0x0A, 0x00,  // \n
    0x00, 0x5F,  // 开
    0x2F, 0x54   // 启
}; // 雷达：敌方飞镖\n\n开启
const uint8_t dart_warning_data2[30] = {
    0xF7, 0x96, 0xBE, 0x8F, 0x1A, 0xFF, 0x2A, 0x00, 0x3A, 0x00, 0x2A, 0x00, 0x2A, 0x00,  // 协议头
    0x0A, 0x00,  // \n
    0x4C, 0x65,  // 敌
    0xB9, 0x65,  // 方
    0xDE, 0x98,  // 飞
    0x56, 0x95,  // 镖
    0x0A, 0x00,  // \n
    0x00, 0x5F,  // 开
    0x2F, 0x54   // 启
}; // 雷达：敌方飞镖\n\n开启

class DecisionNode : public rclcpp::Node
{
public:
    DecisionNode();
    ~DecisionNode();

private:
    uint8_t game_type_;
    uint8_t game_process_;
    uint16_t stage_remain_time_;
    bool is_small_energy_machine_activated_;
    bool is_big_energy_machine_activated_;
    uint8_t is_someone_in_central_highland_;
    
    uint16_t robot_id_;
    std::string robot_color_;
    rclcpp::Time last_em_wave_position_time_;
    bool has_em_wave_opponent_position_;
    bool has_map_robot_data_ = false;

    uint8_t radar_info_chance_;
    bool radar_info_istriggered_;
    uint8_t radar_cmd_cnt_;
    uint16_t last_big_energy_trigger_stage_time_;

    rclcpp::Publisher<radar_msgs::msg::RadarCmd>::SharedPtr radar_cmd_pub_;
    rclcpp::Publisher<radar_msgs::msg::MapRobotData>::SharedPtr map_robot_data_pub_;
    rclcpp::Publisher<radar_msgs::msg::CustomInfo>::SharedPtr custom_info_pub_;
    rclcpp::Publisher<radar_parse_em_wave::msg::RadarParseEmWaveDemodConfig>::SharedPtr radar_demod_config_pub_;
    rclcpp::Publisher<radar_msgs::msg::RadarSentryPositionCmd>::SharedPtr radar_sentry_position_cmd_pub_;
    rclcpp::Publisher<radar_msgs::msg::CombinedData>::SharedPtr combined_data_pub_;
    rclcpp::Publisher<radar_msgs::msg::DartWarningCmd>::SharedPtr dart_warning_cmd_pub_;
    rclcpp::Publisher<radar_msgs::msg::AerialCounteredCmd>::SharedPtr aerial_countered_cmd_pub_;
    
    rclcpp::Subscription<radar_msgs::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Subscription<radar_msgs::msg::GameRobotHP>::SharedPtr game_robot_hp_sub_;
    rclcpp::Subscription<radar_msgs::msg::EventData>::SharedPtr event_data_sub_;
    rclcpp::Subscription<radar_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<radar_msgs::msg::RadarMarkData>::SharedPtr radar_mark_data_sub_;
    rclcpp::Subscription<radar_msgs::msg::RadarInfo>::SharedPtr radar_info_sub_;
    rclcpp::Subscription<radar_msgs::msg::Cars>::SharedPtr cars_sub_;
    // rclcpp::Subscription<radar_msgs::msg::MapRobotData>::SharedPtr map_robot_data_mono_sub_;
    rclcpp::Subscription<radar_parse_em_wave::msg::RadarParseEmWave0A06InterferenceKey>::SharedPtr secret_key_sub;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr dart_warning_sub_;
    rclcpp::Subscription<radar_parse_em_wave::msg::RadarParseEmWave0A05RobotBuff>::SharedPtr robot_buff_sub_;
    rclcpp::Subscription<radar_parse_em_wave::msg::RadarParseEmWave0A01RobotPosition>::SharedPtr robot_position_sub_;
    rclcpp::Subscription<radar_parse_em_wave::msg::RadarParseEmWave0A02RobotHp>::SharedPtr robot_hp_sub_;
    rclcpp::Subscription<radar_parse_em_wave::msg::RadarParseEmWave0A03RobotAmmo>::SharedPtr robot_ammo_sub_;
    rclcpp::Subscription<radar_parse_em_wave::msg::RadarParseEmWave0A04FieldStatus>::SharedPtr field_status_sub_;

    radar_msgs::msg::GameStatus game_status_ref_;
    radar_msgs::msg::GameRobotHP game_robot_hp_ref_;
    radar_msgs::msg::EventData event_data_ref_;
    radar_msgs::msg::RobotStatus robot_status_ref_;
    radar_msgs::msg::RadarInfo radar_info_ref_;
    radar_msgs::msg::MapRobotData map_robot_data_pc_ref_;
    // radar_msgs::msg::MapRobotData map_robot_data_mono_ref_;
    std::string secret_key_;

    // 缓存0x0A02~0x0A05的最新数据
    radar_parse_em_wave::msg::RadarParseEmWave0A02RobotHp latest_hp_data_;
    radar_parse_em_wave::msg::RadarParseEmWave0A03RobotAmmo latest_ammo_data_;
    radar_parse_em_wave::msg::RadarParseEmWave0A04FieldStatus latest_field_data_;
    radar_parse_em_wave::msg::RadarParseEmWave0A05RobotBuff latest_buff_data_;
    bool has_hp_data_ = false;
    bool has_ammo_data_ = false;
    bool has_field_data_ = false;
    bool has_buff_data_ = false;

    TrackerManager tracker_manager_;
    std::string enemy_;
    tools::Timer timer_;

    rclcpp::TimerBase::SharedPtr map_robot_data_pub_timer_;
    rclcpp::TimerBase::SharedPtr radar_cmd_pub_timer_;
    rclcpp::TimerBase::SharedPtr custom_info_pub_timer_;
    rclcpp::TimerBase::SharedPtr combined_data_build_timer_;
    std::queue<radar_msgs::msg::CustomInfo> custom_info_queue_;
    std::queue<radar_msgs::msg::RadarCmd> radar_cmd_queue_;
    std::deque<radar_msgs::msg::CombinedData> combined_data_queue_;
    std::deque<radar_msgs::msg::RadarSentryPositionCmd> sentry_position_queue_;
    std::deque<radar_msgs::msg::DartWarningCmd> dart_warning_queue_;
    std::deque<radar_msgs::msg::AerialCounteredCmd> aerial_countered_queue_;

    void gameStatusCallback(const radar_msgs::msg::GameStatus::ConstPtr &msg);
    void gameRobotHpCallback(const radar_msgs::msg::GameRobotHP::ConstPtr &msg);
    void eventDataCallback(const radar_msgs::msg::EventData::ConstPtr &msg);
    void robotStatusCallback(const radar_msgs::msg::RobotStatus::ConstPtr &msg);
    void radarMarkDataCallback(const radar_msgs::msg::RadarMarkData::ConstPtr &msg);
    void radarInfoCallback(const radar_msgs::msg::RadarInfo::ConstPtr &msg);
    void CarsCallback(const radar_msgs::msg::Cars::ConstPtr &msg);
    void secretKeyCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A06InterferenceKey::ConstPtr &msg);
    void dartWarningCallback(const std_msgs::msg::Int8::ConstPtr &msg);
    void robotBuffCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A05RobotBuff::ConstPtr &msg);
    void robotPositionCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A01RobotPosition::ConstPtr &msg);
    void robotHpCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A02RobotHp::ConstPtr &msg);
    void robotAmmoCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A03RobotAmmo::ConstPtr &msg);
    void fieldStatusCallback(const radar_parse_em_wave::msg::RadarParseEmWave0A04FieldStatus::ConstPtr &msg);

    void pubMapRobotData();
    void pubCustomInfo();
    void pubRadarCmd();
    void buildCombinedData();

    void pushCustomInfo(int custom_info_id);
    void pushRadarCmd(int times, uint8_t password_cmd ,std::string password);    // 发布双倍易伤，参数：次数
};

#endif
