#pragma once
#define __packed __attribute__((packed))
#include <string>
#include "./protocol.h"

namespace sp_referee
{

    struct RobotInfo
    {
        std::string robot_color_;
        std::string robot_type_;
        int robot_id_;
        int client_id_;
    };

    struct UIConfig
    {
        uint8_t figure_id_[3];
        uint32_t operate_type_ : 3;
        uint32_t figure_type_ : 3;
        uint32_t layer_ : 4;
        uint32_t color_ : 4;
        uint32_t start_angle_ : 9;
        uint32_t end_angle_ : 9;
        uint32_t size_ : 9;
        uint32_t length_ : 9;
        uint32_t width_ : 10;
        uint32_t start_x_ : 11;
        uint32_t start_y_ : 11;
        uint32_t center_x_ : 11;
        uint32_t center_y_ : 11;
        uint32_t radius_ : 10;
        uint32_t end_x_ : 11;
        uint32_t end_y_ : 11;
        uint32_t semi_x_axis_ : 11;
        uint32_t semi_y_axis_ : 11;
        std::string context_;
        int64_t seq;
    };

    //-----------------------------------------------

    typedef struct
    {
        uint8_t sof_;
        uint16_t data_length_;
        uint8_t seq_;
        uint8_t crc_8_;
    } __packed FrameHeader;

    typedef struct
    {
        uint8_t game_type_ : 4;
        uint8_t game_progress_ : 4;
        uint16_t stage_remain_time_;
        uint64_t sync_time_stamp_;
    } __packed GameStatus;  // 0x0001

    typedef struct
    {
        uint8_t winner_;
    } __packed GameResult; // 0x0002

    typedef struct
    {
        uint16_t red_1_robot_hp_;
        uint16_t red_2_robot_hp_;
        uint16_t red_3_robot_hp_;
        uint16_t red_4_robot_hp_;
        uint16_t red_5_robot_hp_;
        uint16_t red_7_robot_hp_;
        uint16_t red_outpost_hp_;
        uint16_t red_base_hp_;
        uint16_t blue_1_robot_hp_;
        uint16_t blue_2_robot_hp_;
        uint16_t blue_3_robot_hp_;
        uint16_t blue_4_robot_hp_;
        uint16_t blue_5_robot_hp_;
        uint16_t blue_7_robot_hp_;
        uint16_t blue_outpost_hp_;
        uint16_t blue_base_hp_;
    } __packed GameRobotHp; // 0x0003

    typedef struct
    { 
        uint32_t event_data_;
    } __packed EventData; // 0x0101

    typedef struct
    {

        uint8_t reserved_;
        uint8_t supply_robot_id_;
        uint8_t supply_projectile_step_;
        uint8_t supply_projectile_num_;
    } __packed ExtSupplyProjectileAction; // 0x0102

    typedef struct
    { 
        uint8_t level_; 
        uint8_t offefending_robot_id_; 
        uint8_t count_;
    } __packed RefereeWarning; // 0x0104

    typedef struct
    { 
        uint8_t dart_remaining_time_; 
        uint16_t dart_info_;
    } __packed DartInfo; // 0x0105


    typedef struct
    {
        uint8_t robot_id_;
        uint8_t robot_level_;
        uint16_t current_hp_;
        uint16_t maximum_hp_;
        uint16_t shooter_barrel_cooling_value_;
        uint16_t shooter_barrel_heat_limit_;
        uint16_t chassis_power_limit_;
        uint8_t power_management_gimbal_output_ : 1;
        uint8_t power_management_chassis_output_ : 1;
        uint8_t power_management_shooter_output_ : 1;
    } __packed RobotStatus; // 0X0201

    typedef struct
    {
       uint16_t chassis_voltage_; 
       uint16_t chassis_current_; 
       float chassis_power_; 
       uint16_t buffer_energy_; 
       uint16_t shooter_17mm_1_barrel_heat_; 
       uint16_t shooter_17mm_2_barrel_heat_; 
       uint16_t shooter_42mm_barrel_heat_;
    } __packed PowerHeatData; // 0X0202

    typedef struct
    {
        float x_;
        float y_;
        float angle_;
    } __packed RobotPos; // 0X0203

    typedef struct
    {
        uint8_t recovery_buff_; 
        uint8_t cooling_buff_; 
        uint8_t defence_buff_; 
        uint8_t vulnerability_buff_; 
        uint16_t attack_buff_;
    } __packed Buff; // 0X0204

    typedef struct
    { 
        uint8_t airfrforce_status_; 
        uint8_t time_remain_;
    } __packed AirSupportData; // 0X0205

    typedef struct
    {
        uint8_t armor_id_ : 4;
        uint8_t hp_deduction_reason_ : 4;
    } __packed HurtData; // 0X0206

    typedef struct
    {
        uint8_t bullet_type_;
        uint8_t shooter_number_;
        uint8_t launching_frequency_;
        float initial_speed_;
    } __packed ShootData; // 0X0207

   typedef struct
    { 
        uint16_t projojectile_allowance_17mm_; 
        uint16_t projectile_allowance_42mm_; 
        uint16_t remaining_gold_coin_;
    } __packed ProjectileAllowance; // 0X0208

    typedef struct
    {
        uint32_t rfid_status_;
    } __packed RfidStatus; // 0X0209

   typedef struct
    { 
        uint8_t dart_launch_opening_status_;
        uint8_t reserved_;
        uint16_t target_change_time_;
        uint16_t latestt_launch_cmd_time_;
    } __packed DartClientCmd; // 0X020A

    typedef struct
    {
        float hero_x_; 
        float hero_y_; 
        float engineer_x_; 
        float engineer_y_; 
        float standard_3_x_; 
        float standard_3_y_; 
        float standard_4_x_; 
        float standard_4_y_; 
        float standard_5_x_; 
        float standard_5_y_;
    } __packed GroundRobotPosition; // 0X020B

    typedef struct
    { 
        uint8_t mark_progress_; 
    } __packed RadarMarkData; // 0X020C

    typedef struct
    { 
        uint32_t sentry_info_;
    } __packed SentryInfo; // 0X020D

    typedef struct
    { 
        uint8_t radar_info_chance_ : 2;
        uint8_t radar_info_istriggered_ : 1;
        uint8_t encryption_level_ : 2;
        uint8_t can_modify_key_ : 1;
        uint8_t reserved_ : 2;
    } __packed RadarInfo; // 0X020E

    //---------------------------------------------------//

    typedef struct
    {
        uint16_t data_cmd_id_;
        uint16_t sender_id_;
        uint16_t receiver_id_;
    } __packed  RobotInteractionDataHeader; // 0X0301

    typedef  struct
    {
        float target_position_x_;
        float target_position_y_;
        uint8_t cmd_keyboard_;
        uint8_t target_robot_id_;
        uint8_t cmd_source_;
    } __packed MapCommand; // 0X0303

    typedef struct
    {
        uint16_t opponent_hero_position_x_;
        uint16_t opponent_hero_position_y_;
        uint16_t opponent_engineer_position_x_;  
        uint16_t opponent_engineer_position_y_;
        uint16_t opponent_infantry_3_position_x_;
        uint16_t opponent_infantry_3_position_y_;
        uint16_t opponent_infantry_4_position_x_;
        uint16_t opponent_infantry_4_position_y_;
        uint16_t opponent_aerial_position_x_;
        uint16_t opponent_aerial_position_y_;
        uint16_t opponent_sentry_position_x_;
        uint16_t opponent_sentry_position_y_;
        uint16_t ally_hero_position_x_;
        uint16_t ally_hero_position_y_;
        uint16_t ally_engineer_position_x_;
        uint16_t ally_engineer_position_y_;
        uint16_t ally_infantry_3_position_x_;
        uint16_t ally_infantry_3_position_y_;
        uint16_t ally_infantry_4_position_x_;
        uint16_t ally_infantry_4_position_y_;
        uint16_t ally_aerial_position_x_;
        uint16_t ally_aerial_position_y_;
        uint16_t ally_sentry_position_x_;
        uint16_t ally_sentry_position_y_;
    } __packed MapRobotData; // 0X0305

    typedef struct
    {
        uint8_t intention_;
        uint16_t start_position_x_;
        uint16_t start_position_y_;
        int8_t delta_x_[49];
        int8_t delta_y_[49];
        uint16_t sender_id_;
    } __packed MapData; // 0X0307

    typedef struct
    { 
        uint16_t sender_id_;
        uint16_t receiver_id_;
        uint8_t user_data_[30];
    } __packed CustomInfo; // 0X0308

  //---------------------------------------------------//

    typedef struct
    {
        int16_t x_vel_;
        int16_t y_vel_;
        int16_t z_vel_;
        int16_t wx_vel_;
        int16_t wy_vel_;
        int16_t wz_vel_;
        uint8_t pump_;
        uint8_t final_push_;
        uint8_t pause_;
    }__packed CustomRobotData; // 0X0302 for engineer's custom controller

    typedef struct
    {
        uint8_t data[30];
        uint16_t tail;
    }__packed CustomRobotDataMsg; // 0X0302 for engineer's custom controller


    typedef struct
    {
        uint8_t data[8];
        uint16_t tail;
    }__packed RemoteControl; // 0X0304

    typedef struct
    {
        uint16_t key_value_; 
        uint16_t x_position_:12; 
        uint16_t mouse_left_:4; 
        uint16_t y_position_:12; 
        uint16_t mouse_right_:4; 
        uint16_t reserved_;
    }__packed CustomClientData; // 0X0306

    //---------------------------------------------------//

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        uint8_t delete_type_;
        uint8_t layer_;
    }__packed InteractionLayerDelete; // 0X0100

    typedef struct
    { 
        uint8_t figure_name_[3]; 
        uint32_t operate_type_:3;
        uint32_t figure_type_:3; 
        uint32_t layer_:4; 
        uint32_t color_:4; 
        uint32_t details_a_:9;
        uint32_t details_b_:9;
        uint32_t width_:10; 
        uint32_t start_x_:11; 
        uint32_t start_y_:11; 
        uint32_t details_c_:10; 
        uint32_t details_d_:11; 
        uint32_t details_e_:11; 
    }__packed InteractionFigure; 

        typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_; 
        InteractionFigure interaction_figure_;
    }__packed InteractionFigureOne; // 0X0101

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_; 
        InteractionFigure interaction_figure_[2];
    }__packed InteractionFigureTwo; // 0X0102

    typedef struct
    { 
        RobotInteractionDataHeader robot_interaction_data_header_;
        InteractionFigure interaction_figure_[5];
    }__packed InteractionFigureFive; // 0X0103

    typedef struct
    { 
        RobotInteractionDataHeader robot_interaction_data_header_;
        InteractionFigure interaction_figure_[7];
    }__packed InteractionFigureSeven; // 0X0104

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        InteractionFigure interaction_figure_;
        uint8_t data_[30];
    }__packed ExtClientCustomCharacter; // 0X0110

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        uint32_t sentry_cmd_; 
    }__packed SentryCmd; // 0X0120

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        uint8_t radar_cmd_;
        uint8_t password_cmd_; 
        uint8_t password_1_;
        uint8_t password_2_;
        uint8_t password_3_;
        uint8_t password_4_;
        uint8_t password_5_;
        uint8_t password_6_;
    }__packed RadarCmd; // 0X0121

    typedef struct 
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        uint8_t dart_gate_status_;
    } __packed RadarEnemyDartWarning; //0x0210 dart warning

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        uint8_t source_;
        int16_t hero_x_;
        int16_t hero_y_;
        int16_t engineer_x_;
        int16_t engineer_y_;
        int16_t infantry3_x_;
        int16_t infantry3_y_;
        int16_t infantry4_x_;
        int16_t infantry4_y_;
        int16_t aerial_x_;
        int16_t aerial_y_;
        int16_t sentry_x_;
        int16_t sentry_y_;
    }__packed RadarSentryPositionCmd; // 0x0211

    typedef struct
    {
        RobotInteractionDataHeader robot_interaction_data_header_;
        // 0x0A02 HP数据
        uint16_t hero_hp_;
        uint16_t engineer_hp_;
        uint16_t infantry3_hp_;
        uint16_t infantry4_hp_;
        uint16_t sentry_hp_;
        // 0x0A03 弹药数据
        uint16_t hero_ammo_;
        uint16_t infantry3_ammo_;
        uint16_t infantry4_ammo_;
        uint16_t aerial_ammo_;
        uint16_t sentry_ammo_;
        // 0x0A04 场地状态数据
        uint16_t remain_coins_;
        uint16_t total_coins_;
        uint32_t status_flags_;
        // 0x0A05 Buff数据
        uint8_t hero_heal_;
        uint16_t hero_cool_;
        uint8_t hero_def_;
        uint8_t hero_vuln_;
        uint16_t hero_atk_;
        uint8_t engineer_heal_;
        uint16_t engineer_cool_;
        uint8_t engineer_def_;
        uint8_t engineer_vuln_;
        uint16_t engineer_atk_;
        uint8_t infantry3_heal_;
        uint16_t infantry3_cool_;
        uint8_t infantry3_def_;
        uint8_t infantry3_vuln_;
        uint16_t infantry3_atk_;
        uint8_t infantry4_heal_;
        uint16_t infantry4_cool_;
        uint8_t infantry4_def_;
        uint8_t infantry4_vuln_;
        uint16_t infantry4_atk_;
        uint8_t sentry_heal_;
        uint16_t sentry_cool_;
        uint8_t sentry_def_;
        uint8_t sentry_vuln_;
        uint16_t sentry_atk_;
        uint8_t sentry_posture_;
        uint8_t hero_status_;
        uint8_t engineer_status_;
        uint8_t infantry3_status_;
        uint8_t infantry4_status_;
        uint8_t sentry_status_;
    }__packed RadarAllyCombinedCmd; // 0x0212 合并0x0A02~0x0A05全部数据
}
