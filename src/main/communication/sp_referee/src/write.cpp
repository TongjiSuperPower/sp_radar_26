#include "../include/sp_referee.h"
#include "../include/check.h"
#include "../include/data.h"
#include "../include/protocol.h"
#include <rclcpp/rclcpp.hpp>

namespace sp_referee
{
    void Referee::write(uint16_t write_cmd, uint16_t child_cmd)
    {
        RCLCPP_INFO(rclcpp::get_logger("Referee"), "write() called: write_cmd=0x%04X (%d), child_cmd=0x%04X (%d)",
                    write_cmd, write_cmd, child_cmd, child_cmd);

        int data_len = 0;
        int frame_len = 0;
        switch (write_cmd)
        {
        case sp_referee::ROBOT_INTERACTIVE_DATA_CMD:
        {
            if (robot_info_.robot_type_ == "radar")
            {
                RCLCPP_INFO(rclcpp::get_logger("Referee"), "Robot type is radar, child_cmd=0x%04X", child_cmd);
                switch (child_cmd)
                {
                case sp_referee::RADAR_CMD_CMD:
                {
                    data_len = static_cast<int>(sizeof(sp_referee::RadarCmd));
                    frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                    sp_referee::RadarCmd radar_cmd;
                    radar_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_CMD_CMD;
                    radar_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                    radar_cmd.robot_interaction_data_header_.receiver_id_ = 0x8080;
                    radar_cmd.radar_cmd_ = radar_cmd_ref_.radar_cmd;
                    radar_cmd.password_cmd_ = radar_cmd_ref_.password_cmd;
                    radar_cmd.password_1_ = radar_cmd_ref_.password[0];
                    radar_cmd.password_2_ = radar_cmd_ref_.password[1];
                    radar_cmd.password_3_ = radar_cmd_ref_.password[2];
                    radar_cmd.password_4_ = radar_cmd_ref_.password[3];
                    radar_cmd.password_5_ = radar_cmd_ref_.password[4];
                    radar_cmd.password_6_ = radar_cmd_ref_.password[5];

                    pack(reinterpret_cast<uint8_t *>(&tx_buffer_), reinterpret_cast<uint8_t *>(&radar_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                    break;
                }
                case sp_referee::RADAR_ENEMY_HERO_POSITION_CMD:
                {
                    break;
                }
                case sp_referee::RADAR_ENEMY_DART_WARNING_CMD:
                {
                    if (robot_info_.robot_id_ == 9)
                    {
                        for (int i = 1; i <= 7; ++i)
                        {
                            if (i == 5 || i == 7) continue; // Skip sending to the sentry and infantry 5
                            data_len = static_cast<int>(sizeof(sp_referee::RadarEnemyDartWarning));
                            frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                            sp_referee::RadarEnemyDartWarning radar_enemy_dart_warning_cmd;
                            radar_enemy_dart_warning_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ENEMY_DART_WARNING_CMD;
                            radar_enemy_dart_warning_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                            radar_enemy_dart_warning_cmd.robot_interaction_data_header_.receiver_id_ = i;
                            radar_enemy_dart_warning_cmd.dart_gate_status_ = radar_enemy_dart_warning_ref_.data;

                            std::cout << "Sending RADAR_ENEMY_DART_WARNING_CMD to robot " << i
                                      << " with dart_gate_status=" << static_cast<int>(radar_enemy_dart_warning_cmd.dart_gate_status_) << std::endl;

                            pack(reinterpret_cast<uint8_t *>(&tx_buffer_), reinterpret_cast<uint8_t *>(&radar_enemy_dart_warning_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                        }
                    }

                    if (robot_info_.robot_id_ == 109)
                    {
                        for (int i = 101; i <= 107; ++i)
                        {
                            if (i == 105 || i == 107) continue; // Skip sending to the sentry and infantry 5
                            data_len = static_cast<int>(sizeof(sp_referee::RadarEnemyDartWarning));
                            frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                            sp_referee::RadarEnemyDartWarning radar_enemy_dart_warning_cmd;
                            radar_enemy_dart_warning_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ENEMY_DART_WARNING_CMD;
                            radar_enemy_dart_warning_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                            radar_enemy_dart_warning_cmd.robot_interaction_data_header_.receiver_id_ = i;
                            radar_enemy_dart_warning_cmd.dart_gate_status_ = radar_enemy_dart_warning_ref_.data;

                            pack(reinterpret_cast<uint8_t *>(&tx_buffer_), reinterpret_cast<uint8_t *>(&radar_enemy_dart_warning_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                        }
                    }

                    break;
                }
                case sp_referee::RADAR_SENTRY_BUFF_CMD:
                {
                    data_len = static_cast<int>(sizeof(sp_referee::RadarSentryBuffCmd));
                    frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                    sp_referee::RadarSentryBuffCmd radar_sentry_buff_cmd;
                    radar_sentry_buff_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_SENTRY_BUFF_CMD;
                    radar_sentry_buff_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                    radar_sentry_buff_cmd.robot_interaction_data_header_.receiver_id_ = (robot_info_.robot_id_ >= 100) ? sp_referee::BLUE_SENTRY : sp_referee::RED_SENTRY;
                    radar_sentry_buff_cmd.hero_heal_ = radar_sentry_buff_cmd_ref_.hero_heal;
                    radar_sentry_buff_cmd.hero_cool_ = radar_sentry_buff_cmd_ref_.hero_cool;
                    radar_sentry_buff_cmd.hero_def_ = radar_sentry_buff_cmd_ref_.hero_def;
                    radar_sentry_buff_cmd.hero_vuln_ = radar_sentry_buff_cmd_ref_.hero_vuln;
                    radar_sentry_buff_cmd.hero_atk_ = radar_sentry_buff_cmd_ref_.hero_atk;
                    radar_sentry_buff_cmd.engineer_heal_ = radar_sentry_buff_cmd_ref_.engineer_heal;
                    radar_sentry_buff_cmd.engineer_cool_ = radar_sentry_buff_cmd_ref_.engineer_cool;
                    radar_sentry_buff_cmd.engineer_def_ = radar_sentry_buff_cmd_ref_.engineer_def;
                    radar_sentry_buff_cmd.engineer_vuln_ = radar_sentry_buff_cmd_ref_.engineer_vuln;
                    radar_sentry_buff_cmd.engineer_atk_ = radar_sentry_buff_cmd_ref_.engineer_atk;
                    radar_sentry_buff_cmd.infantry3_heal_ = radar_sentry_buff_cmd_ref_.infantry3_heal;
                    radar_sentry_buff_cmd.infantry3_cool_ = radar_sentry_buff_cmd_ref_.infantry3_cool;
                    radar_sentry_buff_cmd.infantry3_def_ = radar_sentry_buff_cmd_ref_.infantry3_def;
                    radar_sentry_buff_cmd.infantry3_vuln_ = radar_sentry_buff_cmd_ref_.infantry3_vuln;
                    radar_sentry_buff_cmd.infantry3_atk_ = radar_sentry_buff_cmd_ref_.infantry3_atk;
                    radar_sentry_buff_cmd.infantry4_heal_ = radar_sentry_buff_cmd_ref_.infantry4_heal;
                    radar_sentry_buff_cmd.infantry4_cool_ = radar_sentry_buff_cmd_ref_.infantry4_cool;
                    radar_sentry_buff_cmd.infantry4_def_ = radar_sentry_buff_cmd_ref_.infantry4_def;
                    radar_sentry_buff_cmd.infantry4_vuln_ = radar_sentry_buff_cmd_ref_.infantry4_vuln;
                    radar_sentry_buff_cmd.infantry4_atk_ = radar_sentry_buff_cmd_ref_.infantry4_atk;
                    radar_sentry_buff_cmd.sentry_heal_ = radar_sentry_buff_cmd_ref_.sentry_heal;
                    radar_sentry_buff_cmd.sentry_cool_ = radar_sentry_buff_cmd_ref_.sentry_cool;
                    radar_sentry_buff_cmd.sentry_def_ = radar_sentry_buff_cmd_ref_.sentry_def;
                    radar_sentry_buff_cmd.sentry_vuln_ = radar_sentry_buff_cmd_ref_.sentry_vuln;
                    radar_sentry_buff_cmd.sentry_atk_ = radar_sentry_buff_cmd_ref_.sentry_atk;
                    radar_sentry_buff_cmd.sentry_posture_ = radar_sentry_buff_cmd_ref_.sentry_posture;
                    radar_sentry_buff_cmd.hero_status_ = radar_sentry_buff_cmd_ref_.hero_status;
                    radar_sentry_buff_cmd.engineer_status_ = radar_sentry_buff_cmd_ref_.engineer_status;
                    radar_sentry_buff_cmd.infantry3_status_ = radar_sentry_buff_cmd_ref_.infantry3_status;
                    radar_sentry_buff_cmd.infantry4_status_ = radar_sentry_buff_cmd_ref_.infantry4_status;
                    radar_sentry_buff_cmd.sentry_status_ = radar_sentry_buff_cmd_ref_.sentry_status;

                    pack(reinterpret_cast<uint8_t *>(&tx_buffer_), reinterpret_cast<uint8_t *>(&radar_sentry_buff_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                    break;
                }
                case sp_referee::RADAR_SENTRY_POSITION_CMD:
                {
                    data_len = static_cast<int>(sizeof(sp_referee::RadarSentryPositionCmd));
                    frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                    sp_referee::RadarSentryPositionCmd radar_sentry_position_cmd;
                    radar_sentry_position_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_SENTRY_POSITION_CMD;
                    radar_sentry_position_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                    radar_sentry_position_cmd.robot_interaction_data_header_.receiver_id_ = (robot_info_.robot_id_ >= 100) ? sp_referee::BLUE_SENTRY : sp_referee::RED_SENTRY;
                    radar_sentry_position_cmd.source_ = radar_sentry_position_cmd_ref_.source;
                    radar_sentry_position_cmd.hero_x_ = radar_sentry_position_cmd_ref_.hero_x;
                    radar_sentry_position_cmd.hero_y_ = radar_sentry_position_cmd_ref_.hero_y;
                    radar_sentry_position_cmd.engineer_x_ = radar_sentry_position_cmd_ref_.engineer_x;
                    radar_sentry_position_cmd.engineer_y_ = radar_sentry_position_cmd_ref_.engineer_y;
                    radar_sentry_position_cmd.infantry3_x_ = radar_sentry_position_cmd_ref_.infantry3_x;
                    radar_sentry_position_cmd.infantry3_y_ = radar_sentry_position_cmd_ref_.infantry3_y;
                    radar_sentry_position_cmd.infantry4_x_ = radar_sentry_position_cmd_ref_.infantry4_x;
                    radar_sentry_position_cmd.infantry4_y_ = radar_sentry_position_cmd_ref_.infantry4_y;
                    radar_sentry_position_cmd.aerial_x_ = radar_sentry_position_cmd_ref_.aerial_x;
                    radar_sentry_position_cmd.aerial_y_ = radar_sentry_position_cmd_ref_.aerial_y;
                    radar_sentry_position_cmd.sentry_x_ = radar_sentry_position_cmd_ref_.sentry_x;
                    radar_sentry_position_cmd.sentry_y_ = radar_sentry_position_cmd_ref_.sentry_y;

                    pack(reinterpret_cast<uint8_t *>(&tx_buffer_), reinterpret_cast<uint8_t *>(&radar_sentry_position_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                    break;
                }
                default:
                    RCLCPP_WARN(rclcpp::get_logger("Referee"), "Unknown child_cmd for radar: 0x%04X", child_cmd);
                    break;
                }
                break;
            }
            else
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Referee"), "Robot type is not radar (%s), skipping ROBOT_INTERACTIVE_DATA_CMD",
                             robot_info_.robot_type_.c_str());
                break;
            }
        }
        case sp_referee::MAP_ROBOT_DATA_CMD:
        {
            data_len = sizeof(sp_referee::MapRobotData);
            frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::MapRobotData map_robot_data;
            map_robot_data.opponent_hero_position_x_ = map_robot_data_ref_.opponent_hero_position_x;
            map_robot_data.opponent_hero_position_y_ = map_robot_data_ref_.opponent_hero_position_y;
            map_robot_data.opponent_engineer_position_x_ = map_robot_data_ref_.opponent_engineer_position_x;
            map_robot_data.opponent_engineer_position_y_ = map_robot_data_ref_.opponent_engineer_position_y;
            map_robot_data.opponent_infantry_3_position_x_ = map_robot_data_ref_.opponent_infantry_3_position_x;
            map_robot_data.opponent_infantry_3_position_y_ = map_robot_data_ref_.opponent_infantry_3_position_y;
            map_robot_data.opponent_infantry_4_position_x_ = map_robot_data_ref_.opponent_infantry_4_position_x;
            map_robot_data.opponent_infantry_4_position_y_ = map_robot_data_ref_.opponent_infantry_4_position_y;
            map_robot_data.opponent_aerial_position_x_ = map_robot_data_ref_.opponent_aerial_position_x;
            map_robot_data.opponent_aerial_position_y_ = map_robot_data_ref_.opponent_aerial_position_y;
            map_robot_data.opponent_sentry_position_x_ = map_robot_data_ref_.opponent_sentry_position_x;
            map_robot_data.opponent_sentry_position_y_ = map_robot_data_ref_.opponent_sentry_position_y;
            map_robot_data.ally_hero_position_x_ = map_robot_data_ref_.ally_hero_position_x;
            map_robot_data.ally_hero_position_y_ = map_robot_data_ref_.ally_hero_position_y;
            map_robot_data.ally_engineer_position_x_ = map_robot_data_ref_.ally_engineer_position_x;
            map_robot_data.ally_engineer_position_y_ = map_robot_data_ref_.ally_engineer_position_y;
            map_robot_data.ally_infantry_3_position_x_ = map_robot_data_ref_.ally_infantry_3_position_x;
            map_robot_data.ally_infantry_3_position_y_ = map_robot_data_ref_.ally_infantry_3_position_y;
            map_robot_data.ally_infantry_4_position_x_ = map_robot_data_ref_.ally_infantry_4_position_x;
            map_robot_data.ally_infantry_4_position_y_ = map_robot_data_ref_.ally_infantry_4_position_y;
            map_robot_data.ally_aerial_position_x_ = map_robot_data_ref_.ally_aerial_position_x;
            map_robot_data.ally_aerial_position_y_ = map_robot_data_ref_.ally_aerial_position_y;
            map_robot_data.ally_sentry_position_x_ = map_robot_data_ref_.ally_sentry_position_x;
            map_robot_data.ally_sentry_position_y_ = map_robot_data_ref_.ally_sentry_position_y;

            pack(tx_buffer_, reinterpret_cast<uint8_t *>(&map_robot_data), sp_referee::MAP_ROBOT_DATA_CMD, data_len);
            break;
        }
        case sp_referee::CUSTOM_INFO_CMD:
        {
            data_len = sizeof(sp_referee::CustomInfo);
            frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::CustomInfo custom_info;
            custom_info.sender_id_ = custom_info_ref_.sender_id;
            custom_info.receiver_id_ = custom_info_ref_.receiver_id;
            memcpy(custom_info.user_data_, custom_info_ref_.user_data.data(), 30 * sizeof(uint8_t));
            pack(tx_buffer_, reinterpret_cast<uint8_t *>(&custom_info), sp_referee::CUSTOM_INFO_CMD, data_len);
            break;
        }
        default:
        {
            RCLCPP_WARN(rclcpp::get_logger("Referee"), "Referee command ID 0x%04X not recognized, skipping send", write_cmd);
            break;
        }
        }

        try {
            size_t written = serial_.write(tx_buffer_, frame_len);
            if (written != frame_len) {
                RCLCPP_ERROR(this->get_logger(), "Incomplete write! Only %zu of %d bytes sent", written, frame_len);
            }
        } catch (serial::PortNotOpenedException &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open: %s", e.what());
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial IO error: %s", e.what());
        }

        clearTxBuffer();
    }

    void Referee::pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len)
    {
        auto *frame_header = reinterpret_cast<sp_referee::FrameHeader *>(tx_buffer);
        frame_header->sof_ = 0xA5;
        frame_header->data_length_ = len;
        memcpy(&tx_buffer[frame_header_length_], reinterpret_cast<uint8_t *>(&cmd_id), cmd_id_length_);
        check_.appendCRC8CheckSum(tx_buffer, frame_header_length_);
        memcpy(&tx_buffer[frame_header_length_ + cmd_id_length_], data, len);
        int total_len_before_crc16 = frame_header_length_ + cmd_id_length_ + len + frame_tail_length_;
        check_.appendCRC16CheckSum(tx_buffer, total_len_before_crc16);
    }

    void Referee::clearTxBuffer()
    {
        for (int i = 0; i < 128; i++)
            tx_buffer_[i] = 0;
    }

} // namespace sp_referee