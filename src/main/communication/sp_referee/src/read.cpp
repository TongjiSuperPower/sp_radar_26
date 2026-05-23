#include "../include/sp_referee.h"
#include "../include/check.h"
#include "../include/data.h"
#include "../include/protocol.h"
#include <rclcpp/rclcpp.hpp>  // 用于日志输出

namespace sp_referee
{
    void Referee::read()
    {
        if (serial_.available())
        {
            rx_len_ = static_cast<int>(serial_.available());
            serial_.read(rx_buffer_, rx_len_);
            RCLCPP_INFO(rclcpp::get_logger("Referee"), "Read %d bytes from serial", rx_len_);
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger("Referee"), "No serial data available");
            return;
        }

        uint8_t temp_buffer[256] = {0};
        int frame_len = 0;
        // if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
        //     referee_data_is_online_ = false;
        if (rx_len_ < k_unpack_buffer_length_)
        {
            for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
                temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_]; // 将历史待打包的数据覆写到前面
            for (int k_i = 0; k_i < rx_len_; ++k_i)
                temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i]; // 缓冲区收到多少数据向待打包缓冲区写入多少数据
            for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
                unpack_buffer_[k_i] = temp_buffer[k_i];
        }
        for (int k_i = 0; k_i < k_unpack_buffer_length_ - frame_length_; ++k_i)
        {
            if (unpack_buffer_[k_i] == 0xA5) // 每读到一个串口帧头，打包一次缓冲区数据
            {
                RCLCPP_DEBUG(rclcpp::get_logger("Referee"), "Found frame header at index %d", k_i);
                frame_len = unpack(&unpack_buffer_[k_i]);
                if (frame_len != -1)
                    k_i += frame_len;
            }
        }
        getRobotInfo();
        clearRxBuffer();
    }

    // Data struct:
    // | frame_header:5 bytes | cmd_id:2 bytes | data: n bytes | frame_tail: 2 bytes |
    int Referee::unpack(uint8_t *rx_data)
    {
        uint16_t cmd_id;
        int frame_len;
        sp_referee::FrameHeader frame_header;

        memcpy(&frame_header, rx_data, frame_header_length_);

        if (static_cast<bool>(check_.verifyCRC8CheckSum(rx_data, frame_header_length_)))
        {   
            if (frame_header.data_length_ > 256) // temporary and inaccurate value
            {
                return 0;
            }
            frame_len = frame_header.data_length_ + frame_header_length_ + cmd_id_length_ + frame_tail_length_;

            if (check_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
            {
                cmd_id = (rx_data[6] << 8 | rx_data[5]);
                RCLCPP_INFO(rclcpp::get_logger("Referee"), "Received cmd_id = 0x%04X (%d), frame_len=%d", cmd_id, cmd_id, frame_len);

                switch (cmd_id)
                {
                    case sp_referee::GAME_STATUS_CMD:
                    {
                        sp_referee::GameStatus game_status_ref;
                        radar_msgs::msg::GameStatus game_status;
                        memcpy(&game_status_ref, rx_data + 7, sizeof(sp_referee::GameStatus));

                        game_status.game_type = game_status_ref.game_type_;
                        game_status.game_progress = game_status_ref.game_progress_;
                        game_status.stage_remain_time = game_status_ref.stage_remain_time_;
                        game_status.sync_time_stamp = game_status_ref.sync_time_stamp_;
                        
                        RCLCPP_INFO(rclcpp::get_logger("Referee"), "GameStatus: type=%d, progress=%d, remain_time=%d",
                                    game_status.game_type, game_status.game_progress, game_status.stage_remain_time);
                        game_status_pub_->publish(game_status);
                        break;
                    }
                    case sp_referee::GAME_RESULT_CMD:
                    {
                    //     sp_referee::GameResult game_result_ref;
                    //     memcpy(&game_result_ref, rx_data + 7, sizeof(sp_referee::GameResult));
                    //     break;
                    }
                    case sp_referee::GAME_ROBOT_HP_CMD:
                    {
                    //     sp_referee::GameRobotHp game_robot_hp_ref;
                    //     radar_msgs::msg::GameRobotHP game_robot_hp;
                    //     memcpy(&game_robot_hp_ref, rx_data + 7, sizeof(sp_referee::GameRobotHp));

                    //     game_robot_hp.blue_1_robot_hp = game_robot_hp_ref.blue_1_robot_hp_;
                    //     game_robot_hp.blue_2_robot_hp = game_robot_hp_ref.blue_2_robot_hp_;
                    //     game_robot_hp.blue_3_robot_hp = game_robot_hp_ref.blue_3_robot_hp_;
                    //     game_robot_hp.blue_4_robot_hp = game_robot_hp_ref.blue_4_robot_hp_;
                    //     game_robot_hp.blue_reserved = game_robot_hp_ref.blue_5_robot_hp_;
                    //     game_robot_hp.blue_7_robot_hp = game_robot_hp_ref.blue_7_robot_hp_;
                    //     game_robot_hp.red_1_robot_hp = game_robot_hp_ref.red_1_robot_hp_;
                    //     game_robot_hp.red_2_robot_hp = game_robot_hp_ref.red_2_robot_hp_;
                    //     game_robot_hp.red_3_robot_hp = game_robot_hp_ref.red_3_robot_hp_;
                    //     game_robot_hp.red_4_robot_hp = game_robot_hp_ref.red_4_robot_hp_;
                    //     game_robot_hp.red_reserved = game_robot_hp_ref.red_5_robot_hp_;
                    //     game_robot_hp.red_7_robot_hp = game_robot_hp_ref.red_7_robot_hp_;
                    //     // game_robot_hp.stamp = last_get_data_time_;
                        
                    //     game_robot_hp_pub_->publish(game_robot_hp);
                    //     break;
                    }
                    case sp_referee::EVENT_DATA_CMD:  // 判断己方中央高地增益是否被对方占领，能量机关激活
                    {
                        sp_referee::EventData event_data_ref;
                        radar_msgs::msg::EventData event_data;
                        memcpy(&event_data_ref, rx_data + 7, sizeof(sp_referee::EventData));

                        event_data.event_data = event_data_ref.event_data_;
                        event_data_pub_->publish(event_data);
                        break;
                    }
                    case sp_referee::ROBOT_STATUS_CMD:
                    {
                        sp_referee::RobotStatus robot_status_ref;
                        radar_msgs::msg::RobotStatus robot_status;
                        memcpy(&robot_status_ref, rx_data + 7, sizeof(sp_referee::RobotStatus));

                        if(robot_status_ref.robot_id_ == 0)
                            break;
                        
                        robot_status.robot_id = robot_status_ref.robot_id_;
                        robot_status.robot_level = robot_status_ref.robot_level_;
                        robot_status.current_hp = robot_status_ref.current_hp_;
                        robot_status.maximum_hp = robot_status_ref.maximum_hp_;
                        robot_status.shooter_barrel_cooling_value = robot_status_ref.shooter_barrel_cooling_value_;
                        robot_status.shooter_barrel_heat_limit = robot_status_ref.shooter_barrel_heat_limit_;
                        robot_status.chassis_power_limit = robot_status_ref.shooter_barrel_cooling_value_;
                        robot_status.chassis_power_limit = robot_status_ref.chassis_power_limit_;
                        robot_status.power_management_chassis_output = robot_status_ref.power_management_chassis_output_;
                        robot_status.power_management_gimbal_output = robot_status_ref.power_management_gimbal_output_;
                        robot_status.power_management_shooter_output = robot_status_ref.power_management_shooter_output_;
                        robot_info_.robot_id_ = robot_status_ref.robot_id_;
                        // robot_status.stamp = last_get_data_time_;

                        robot_status_pub_->publish(robot_status);
                        break;
                    }
                    case sp_referee::RADAR_MARK_DATA_CMD:
                    {
                        sp_referee::RadarMarkData radar_mark_data_ref;
                        radar_msgs::msg::RadarMarkData radar_mark_data;
                        memcpy(&radar_mark_data_ref, rx_data + 7, sizeof(sp_referee::RadarMarkData));

                        radar_mark_data.mark_progress = radar_mark_data_ref.mark_progress_;

                        radar_mark_data_pub_->publish(radar_mark_data);
                        break;
                    }
                    case sp_referee::RADAR_INFO_CMD:
                    {
                        sp_referee::RadarInfo radar_info_ref;
                        radar_msgs::msg::RadarInfo radar_info;
                        memcpy(&radar_info_ref, rx_data + 7, sizeof(sp_referee::RadarInfo));

                        radar_info.radar_info_chance = radar_info_ref.radar_info_chance_;
                        radar_info.radar_info_istriggered = radar_info_ref.radar_info_istriggered_;

                        radar_info_pub_->publish(radar_info);
                        break;
                    }
                    default:
                    {
                        RCLCPP_WARN(rclcpp::get_logger("Referee"), "Unknown cmd_id: 0x%04X", cmd_id);
                        break;
                    }
                }
                // referee_data_is_online_ = true;
                // last_get_data_time_ = ros::Time::now();
                return frame_len;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("Referee"), "CRC16 verification failed");
            }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("Referee"), "CRC8 verification failed");
        }
        return -1;
    }

    void Referee::getRobotInfo()
    {
        robot_info_.robot_color_ = robot_info_.robot_id_ >= 100 ? "blue" : "red";
        switch (robot_info_.robot_id_)
        {
        case sp_referee::RED_HERO:
        case sp_referee::BLUE_HERO:
            robot_info_.robot_type_ = "hero";
            break;
        case sp_referee::RED_ENGINEER:
        case sp_referee::BLUE_ENGINEER:
            robot_info_.robot_type_ = "engineer";
            break;
        case sp_referee::RED_INFANTRY_3:
        case sp_referee::RED_INFANTRY_4:
        case sp_referee::RED_INFANTRY_5:
        case sp_referee::BLUE_INFANTRY_3:
        case sp_referee::BLUE_INFANTRY_4:
        case sp_referee::BLUE_INFANTRY_5:
            robot_info_.robot_type_ = "infantry";
            break;
        case sp_referee::RED_AERIAL:
        case sp_referee::BLUE_AERIAL:
            robot_info_.robot_type_ = "aerial";
            break;
        case sp_referee::RED_SENTRY:
        case sp_referee::BLUE_SENTRY:
            robot_info_.robot_type_ = "sentry";
            break;
        case sp_referee::RED_DART:
        case sp_referee::BLUE_DART:
            robot_info_.robot_type_ = "dart";
            break;
        case sp_referee::RED_RADAR:
        case sp_referee::BLUE_RADAR:
            robot_info_.robot_type_ = "radar";
            break;
        }

        switch (robot_info_.robot_id_)
        {
        case sp_referee::BLUE_HERO:
            robot_info_.client_id_ = sp_referee::BLUE_HERO_CLIENT;
            break;
        case sp_referee::BLUE_ENGINEER:
            robot_info_.client_id_ = sp_referee::BLUE_ENGINEER_CLIENT;
            break;
        case sp_referee::BLUE_INFANTRY_3:
            robot_info_.client_id_ = sp_referee::BLUE_INFANTRY_3_CLIENT;
            break;
        case sp_referee::BLUE_INFANTRY_4:
            robot_info_.client_id_ = sp_referee::BLUE_INFANTRY_4_CLIENT;
            break;
        case sp_referee::BLUE_INFANTRY_5:
            robot_info_.client_id_ = sp_referee::BLUE_INFANTRY_5_CLIENT;
            break;
        case sp_referee::RED_HERO:
            robot_info_.client_id_ = sp_referee::RED_HERO_CLIENT;
            break;
        case sp_referee::RED_ENGINEER:
            robot_info_.client_id_ = sp_referee::RED_ENGINEER_CLIENT;
            break;
        case sp_referee::RED_INFANTRY_3:
            robot_info_.client_id_ = sp_referee::RED_INFANTRY_3_CLIENT;
            break;
        case sp_referee::RED_INFANTRY_4:
            robot_info_.client_id_ = sp_referee::RED_INFANTRY_4_CLIENT;
            break;
        case sp_referee::RED_INFANTRY_5:
            robot_info_.client_id_ = sp_referee::RED_INFANTRY_5_CLIENT;
            break;
        }
    }

    void Referee::clearRxBuffer()
    {
        rx_buffer_.clear();
        rx_len_ = 0;
    }

} // namespace sp_referee